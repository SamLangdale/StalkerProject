#!/usr/bin/env python3

from collections import deque
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from laser_geometry import LaserProjection
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class Scan3DNode(Node):
    def __init__(self):
        super().__init__('scan_3d_node')


    # Params
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/cloud')
        self.declare_parameter('fixed_frame', 'base_link')
        self.declare_parameter('accumulation_time_s', 5.0)
        self.declare_parameter('max_points', 120000)
        self.declare_parameter('transform_timeout_s', 0.1)
        self.declare_parameter('min_range_m', 0.0)
        self.declare_parameter('max_range_m', 0.0)
        self.declare_parameter('use_latest_transform', True)
        self.declare_parameter('cloud_scale', 1)

        scan_topic = str(self.get_parameter('scan_topic').value)
        cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.accumulation_time_s = float(
            self.get_parameter('accumulation_time_s').value
        )
        self.max_points = int(self.get_parameter('max_points').value)
        transform_timeout_s = float(
            self.get_parameter('transform_timeout_s').value
        )
        self.min_range_override_m = float(self.get_parameter('min_range_m').value)
        self.max_range_override_m = float(self.get_parameter('max_range_m').value)
        self.use_latest_transform = bool(
            self.get_parameter('use_latest_transform').value
        )
        self.cloud_scale = float(self.get_parameter('cloud_scale').value)

        self.projector = LaserProjection()
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform_timeout = Duration(seconds=transform_timeout_s)

        self.scan_slices = deque()
        self.total_points = 0
        self.tf_warning_count = 0

        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10,
        )
        self.pub = self.create_publisher(PointCloud2, cloud_topic, 10)

        self.get_logger().info(
            f'Accumulating 3D cloud from {scan_topic} into {cloud_topic} '
            f'in frame {self.fixed_frame}'
        )
        if self.use_latest_transform:
            self.get_logger().info(
                'scan_3d will fall back to the latest available transform '
                'when scan-time lookup fails'
            )
        if self.cloud_scale != 1.0:
            self.get_logger().info(
                f'scan_3d is scaling published cloud coordinates by '
                f'{self.cloud_scale:.3f}'
            )

    def scan_callback(self, msg: LaserScan) -> None:
        filtered_scan = self.filter_scan(msg) #removes invalid ranges and applies overrides
        if filtered_scan is None:
            return

        try:
            # laser scan to point cloud in the scan frame
            slice_cloud = self.projector.projectLaser(
                filtered_scan,
                channel_options=LaserProjection.ChannelOption.NONE,
            )
            #gets transformation of laser scan to base link if available
            transform = self.lookup_scan_transform(slice_cloud)
        except TransformException as exc:
            self.tf_warning_count += 1
            if self.tf_warning_count <= 5 or self.tf_warning_count % 25 == 0:
                self.get_logger().warn(
                    f'Unable to transform scan from {msg.header.frame_id} '
                    f'to {self.fixed_frame}: {exc}'
                )
            return
        except Exception as exc:
            self.get_logger().error(f'Projection failed: {type(exc).__name__}: {exc}')
            return

        self.tf_warning_count = 0
        #transforms points from laser scan
        transformed_points = self.transform_points(slice_cloud, transform)
        if not transformed_points:
            return

        # adds timestamp and points to the deque, then trims old slices and excess points before publishing
        stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        self.scan_slices.append((stamp_ns, transformed_points))
        self.total_points += len(transformed_points)

        self.trim_old_slices(stamp_ns)
        self.trim_excess_points()
        self.publish_cloud(msg.header.stamp)



    #filters out invalid readings
    def filter_scan(self, msg: LaserScan) -> LaserScan | None: 
        min_range = (
            self.min_range_override_m
            if self.min_range_override_m > 0.0
            else msg.range_min
        )
        max_range = (
            self.max_range_override_m
            if self.max_range_override_m > 0.0
            else msg.range_max
        )

        filtered_ranges = []
        valid_points = 0

        for distance in msg.ranges:
            is_valid = math.isfinite(distance) and min_range <= distance <= max_range
            if is_valid:
                filtered_ranges.append(distance)
                valid_points += 1
            else:
                filtered_ranges.append(float('inf'))

        if valid_points == 0:
            self.get_logger().debug('Received /scan message with no valid ranges')
            return None

        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = min_range
        filtered_scan.range_max = max_range
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = list(msg.intensities)
        return filtered_scan

    # looksup transformation from laser scan
    def lookup_scan_transform(self, cloud: PointCloud2):
        try:
            return self.tf_buffer.lookup_transform(
                self.fixed_frame,
                cloud.header.frame_id,
                rclpy.time.Time.from_msg(cloud.header.stamp),
                timeout=self.transform_timeout,
            )
        except TransformException as exc:
            if not self.use_latest_transform:
                raise

            self.get_logger().warn(
                'Falling back to latest transform because scan-time lookup '
                f'failed: {exc}'
            )
            # fallback if scan-time is unavailable
            return self.tf_buffer.lookup_transform(
                self.fixed_frame,
                cloud.header.frame_id,
                rclpy.time.Time(),
                timeout=self.transform_timeout,
            )

    # transforms points from laser scan to the base link frame
    def transform_points(self,cloud: PointCloud2,transform) -> list[tuple[float, float, float]]:
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        points = []
        for x, y, z in point_cloud2.read_points(
            cloud,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        ):
            rx, ry, rz = self.rotate_point(x, y, z, qx, qy, qz, qw)
            px = (rx + tx) * self.cloud_scale
            py = (ry + ty) * self.cloud_scale
            pz = (rz + tz) * self.cloud_scale
            points.append((px, py, pz))
        return points

    def rotate_point(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> tuple[float, float, float]:
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rx = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        ry = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        rz = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z
        return rx, ry, rz
    
    # trimming functions
    def trim_old_slices(self, current_stamp_ns: int) -> None:
        max_age_ns = int(self.accumulation_time_s * 1e9)
        cutoff_ns = current_stamp_ns - max_age_ns

        while self.scan_slices and self.scan_slices[0][0] < cutoff_ns:
            _, points = self.scan_slices.popleft()
            self.total_points -= len(points)

    def trim_excess_points(self) -> None:
        while self.scan_slices and self.total_points > self.max_points:
            _, points = self.scan_slices.popleft()
            self.total_points -= len(points)

    # publishes the accumulated point cloud
    def publish_cloud(self, stamp) -> None:
        points = []
        for _, scan_points in self.scan_slices:
            points.extend(scan_points)

        cloud_msg = point_cloud2.create_cloud_xyz32(
            header=self.make_header(stamp),
            points=points,
        )
        self.pub.publish(cloud_msg)

    def make_header(self, stamp):
        return Header(
            stamp=stamp,
            frame_id=self.fixed_frame,
        )

    def stamp_to_nanoseconds(self, stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Scan3DNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
