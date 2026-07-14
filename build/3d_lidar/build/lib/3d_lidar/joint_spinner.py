import math
# temporary testing node

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

# todo - config to stepper motor!!!
# Dir i2c - pin 27
# step i2c - pin 28





class LidarJointSpinner(Node):
    def __init__(self):
        super().__init__('lidar_joint_spinner')

        self.declare_parameter('joint_name', 'base_to_lidar_rotator_joint')
        self.declare_parameter('speed_rad_s', 1.0)
        self.declare_parameter('publish_rate_hz', 250.0)
        self.declare_parameter('start_angle_rad', 0.0)
        self.declare_parameter('timestamp_offset_s', 0.05)

        self.joint_name = str(self.get_parameter('joint_name').value)
        self.speed_rad_s = float(self.get_parameter('speed_rad_s').value)
        self.angle_rad = float(self.get_parameter('start_angle_rad').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timestamp_offset_s = float(
            self.get_parameter('timestamp_offset_s').value
        )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(JointState, '/joint_states', qos)
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_joint_state)
        self.get_logger().info(
            f'Publishing {self.joint_name} on /joint_states at '
            f'{publish_rate_hz:.1f} Hz with timestamp offset '
            f'{self.timestamp_offset_s:.3f} s'
        )

    def publish_joint_state(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.angle_rad = math.fmod(
            self.angle_rad + self.speed_rad_s * dt,
            2.0 * math.pi,
        )

        msg = JointState()
        stamp = now - rclpy.duration.Duration(seconds=self.timestamp_offset_s)
        msg.header.stamp = stamp.to_msg()
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
