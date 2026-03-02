#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2  # comes with ROS 2 (sensor_msgs_py)

class cloud_reader(Node):
    def __init__(self):
        super().__init__('cloud_reader')
        self.sub = self.create_subscription(
            PointCloud2,
            '/gazebo_ros_lidar_controller/out',  # change to your topic
            self.cb,
            qos_profile_sensor_data
        )

    def cb(self, msg: PointCloud2):
        self.get_logger().info(
            f"Cloud: frame={msg.header.frame_id} size={msg.width}x{msg.height} fields={[f.name for f in msg.fields]}"
        )

        field_names = [f.name for f in msg.fields]
        want = ['x', 'y', 'z']
        if 'intensity' in field_names:
            want.append('intensity')

        # Efficient generator over points
        # skip_nans=True avoids NaNs common in some sensors
        pts = point_cloud2.read_points(msg, field_names=want, skip_nans=True)

        # Example: compute a quick range statistic from first N points
        count = 0
        min_r2 = None
        for p in pts:
            x, y, z = p[0], p[1], p[2]
            r2 = x*x + y*y + z*z
            min_r2 = r2 if min_r2 is None else min(min_r2, r2)
            count += 1
            if count >= 5000:
                break

        if min_r2 is not None:
            self.get_logger().info(f"Read {count} points (sample). Closest^2 ~ {min_r2:.3f}")

        
            

def main():
    rclpy.init()
    node = cloud_reader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()