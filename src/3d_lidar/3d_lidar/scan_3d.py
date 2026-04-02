#!/usr/bin/env python3

#import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection


class Scan3DNode(Node):
    def __init__(self):
        super().__init__('scan_3d_node')
        self.projector = LaserProjection()
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/cloud',
            10
        )
        self.get_logger().info('Scan-3d node subscribed to /scan')

    def scan_callback(self, msg: LaserScan) -> None:
        # valid_ranges = [
        #     distance for distance in msg.ranges
        #     if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max
        # ]

        # if not valid_ranges:
        #     self.get_logger().warn('Received /scan message with no valid ranges')
        #     return

        # min_range = min(valid_ranges)
        # max_range = max(valid_ranges)
        # sample_count = len(valid_ranges)

        # self.get_logger().info(
        #     'Laser scan received: '
        #     f'{sample_count} valid points, '
        #     f'min={min_range:.2f} m, '
        #     f'max={max_range:.2f} m, '
        #     f'angle_min={msg.angle_min:.2f} rad, '
        #     f'angle_max={msg.angle_max:.2f} rad'
        # )
        try:
            cloud_msg = self.projector.projectLaser(
                msg,
                channel_options=LaserProjection.ChannelOption.NONE,
            )
            cloud_msg.header = msg.header
            self.pub.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f'Projection failed: {type(e).__name__}: {e}')





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
