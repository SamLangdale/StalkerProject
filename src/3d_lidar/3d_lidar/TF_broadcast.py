## marked for death


import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

class RotatingLidarTF(Node):
    def __init__(self):
        super().__init__('rotating_lidar_tf')

        self.dynamic_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_tf)  # 50 Hz

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rotator_frame', 'lidar_rotator')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('rotator_height_m', 0.205)
        self.declare_parameter('laser_offset_x_m', 0.0)
        self.declare_parameter('laser_offset_y_m', 0.0)
        self.declare_parameter('laser_offset_z_m', 0.0)
        self.declare_parameter('laser_roll_rad', 0.0)
        self.declare_parameter('laser_pitch_rad', 0.0)
        self.declare_parameter('laser_yaw_rad', 0.0)
        self.declare_parameter('rotation_speed_rad_s', 0.0)
        self.declare_parameter('initial_angle_rad', 0.0)

        self.angle_rad = float(self.get_parameter('initial_angle_rad').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.rot_frame = str(self.get_parameter('rotator_frame').value)
        self.laser_frame = str(self.get_parameter('laser_frame').value)
        self.rotator_height_m = float(self.get_parameter('rotator_height_m').value)
        self.rotation_speed_rad_s = float(
            self.get_parameter('rotation_speed_rad_s').value
        )

        self.last_time = self.get_clock().now()
        self.publish_static_tf()

    def quaternion_from_rpy(self, roll, pitch, yaw):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.rot_frame
        t.child_frame_id = self.laser_frame

        t.transform.translation.x = float(self.get_parameter('laser_offset_x_m').value)
        t.transform.translation.y = float(self.get_parameter('laser_offset_y_m').value)
        t.transform.translation.z = float(self.get_parameter('laser_offset_z_m').value)

        qx, qy, qz, qw = self.quaternion_from_rpy(
            float(self.get_parameter('laser_roll_rad').value),
            float(self.get_parameter('laser_pitch_rad').value),
            float(self.get_parameter('laser_yaw_rad').value),
        )
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.static_br.sendTransform(t)

    def publish_tf(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        self.angle_rad += self.rotation_speed_rad_s * dt

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.rot_frame

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.rotator_height_m

        half = self.angle_rad / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)

        self.dynamic_br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RotatingLidarTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
