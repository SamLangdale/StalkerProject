import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class LidarJointSpinner(Node):
    def __init__(self):
        super().__init__('lidar_joint_spinner')

        self.declare_parameter('joint_name', 'base_to_lidar_rotator_joint')
        self.declare_parameter('speed_rad_s', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('start_angle_rad', 0.0)

        self.joint_name = str(self.get_parameter('joint_name').value)
        self.speed_rad_s = float(self.get_parameter('speed_rad_s').value)
        self.angle_rad = float(self.get_parameter('start_angle_rad').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_joint_state)

    def publish_joint_state(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.angle_rad = math.fmod(
            self.angle_rad + self.speed_rad_s * dt,
            2.0 * math.pi,
        )

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.angle_rad]
        msg.velocity = [self.speed_rad_s]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarJointSpinner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
