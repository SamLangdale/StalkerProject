#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math




class TwistToRpm(Node):
    def __init__(self):
        super().__init__('twist_to_rpm')

      
        self.declare_parameter('wheel_base', 0.5)  # Distance between wheels in meters
        self.declare_parameter('wheel_radius', 0.1)  # Wheel radius in meters
        self.declare_parameter('max_rpm', 3000)  # Maximum RPM for motors

      
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_rpm = self.get_parameter('max_rpm').value

        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publishers
        self.left_rpm_publisher = self.create_publisher(Float64, 'left_rpm', 10)
        self.right_rpm_publisher = self.create_publisher(Float64, 'right_rpm', 10)

        self.get_logger().info('TwistToRpm node initialized')

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x  
        angular_velocity = msg.angular.z 

        # Calculate wheel velocities for diff drive
        v_left = linear_velocity - (angular_velocity * self.wheel_base / 2)
        v_right = linear_velocity + (angular_velocity * self.wheel_base / 2)

        # Convert linear velocity to RPM
        left_rpm = (v_left / (2 * math.pi * self.wheel_radius)) * 60 
        right_rpm = (v_right / (2 * math.pi * self.wheel_radius)) * 60


        #Todo: gearbox reduction calculation if needed 4 rotation of motor - 1 rotation of wheel -> multiply by 4
        left_rpm *= 4
        right_rpm *= 4

        # Clamp RPM to max_rpm
        left_rpm = max(-self.max_rpm, min(self.max_rpm, left_rpm))
        right_rpm = max(-self.max_rpm, min(self.max_rpm, right_rpm))

        # Publish RPM values
        left_msg = Float64()
        left_msg.data = left_rpm
        self.left_rpm_publisher.publish(left_msg)

        right_msg = Float64()
        right_msg.data = right_rpm
        self.right_rpm_publisher.publish(right_msg)

        #self.get_logger().debug(f'Published left_rpm: {left_rpm}, right_rpm: {right_rpm}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistToRpm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
