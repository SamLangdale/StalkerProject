#!/usr/bin/env python3

import struct
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

try:
    import can
except ImportError:  # pragma: no cover
    can = None


class RpmToVescCan(Node):
    """Bridge left/right RPM ROS topics to VESC CAN SET_RPM commands."""

    # VESC CAN packet command IDs
    CAN_PACKET_SET_RPM = 3

    def __init__(self) -> None:
        super().__init__('rpm_to_vesc_can')

        # ROS topics
        self.declare_parameter('left_rpm_topic', 'left_rpm')
        self.declare_parameter('right_rpm_topic', 'right_rpm')

        # CAN bus setup
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bustype', 'socketcan')
        self.declare_parameter('can_bitrate', 500000)

        # VESC CAN IDs
        self.declare_parameter('left_can_id', 1)
        self.declare_parameter('right_can_id', 2)

        # Safety/control
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.25)
        self.declare_parameter('max_rpm', 3000.0)

        self.left_rpm_topic = str(self.get_parameter('left_rpm_topic').value)
        self.right_rpm_topic = str(self.get_parameter('right_rpm_topic').value)

        self.can_channel = str(self.get_parameter('can_channel').value)
        self.can_bustype = str(self.get_parameter('can_bustype').value)
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)

        self.left_can_id = int(self.get_parameter('left_can_id').value)
        self.right_can_id = int(self.get_parameter('right_can_id').value)

        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)

        if self.command_rate_hz <= 0.0:
            raise ValueError('command_rate_hz must be > 0')
        if self.cmd_timeout_sec <= 0.0:
            raise ValueError('cmd_timeout_sec must be > 0')
        if self.max_rpm <= 0.0:
            raise ValueError('max_rpm must be > 0')
        if not (0 <= self.left_can_id <= 255 and 0 <= self.right_can_id <= 255):
            raise ValueError('left_can_id and right_can_id must be between 0 and 255')

        self.left_rpm_cmd: float = 0.0
        self.right_rpm_cmd: float = 0.0
        self.last_left_msg_time: Optional[float] = None
        self.last_right_msg_time: Optional[float] = None

        if can is None:
            raise RuntimeError(
                'python-can is not installed. Install it with: pip install python-can'
            )

        self.bus = can.interface.Bus(
            channel=self.can_channel,
            bustype=self.can_bustype,
            bitrate=self.can_bitrate,
        )

        self.create_subscription(Float64, self.left_rpm_topic, self.left_callback, 10)
        self.create_subscription(Float64, self.right_rpm_topic, self.right_callback, 10)

        timer_period = 1.0 / self.command_rate_hz
        self.timer = self.create_timer(timer_period, self.send_commands)

        self.get_logger().info(
            f'rpm_to_vesc_can started. left_topic={self.left_rpm_topic}, '
            f'right_topic={self.right_rpm_topic}, can={self.can_bustype}:{self.can_channel}@{self.can_bitrate}, '
            f'left_can_id={self.left_can_id}, right_can_id={self.right_can_id}'
        )

    def left_callback(self, msg: Float64) -> None:
        self.left_rpm_cmd = self._clamp_rpm(float(msg.data))
        self.last_left_msg_time = time.monotonic()

    def right_callback(self, msg: Float64) -> None:
        self.right_rpm_cmd = self._clamp_rpm(float(msg.data))
        self.last_right_msg_time = time.monotonic()

    def send_commands(self) -> None:
        now = time.monotonic()

        left_rpm = self.left_rpm_cmd
        right_rpm = self.right_rpm_cmd

        if self.last_left_msg_time is None or (now - self.last_left_msg_time) > self.cmd_timeout_sec:
            left_rpm = 0.0
        if self.last_right_msg_time is None or (now - self.last_right_msg_time) > self.cmd_timeout_sec:
            right_rpm = 0.0

        self._send_set_rpm(self.left_can_id, int(left_rpm))
        self._send_set_rpm(self.right_can_id, int(right_rpm))

    def _clamp_rpm(self, rpm: float) -> float:
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    def _send_set_rpm(self, controller_id: int, rpm: int) -> None:
        # VESC expects 4-byte signed int big-endian for SET_RPM payload.
        data = struct.pack('>i', rpm)

        # VESC CAN ID layout for command frames:
        # ext_id = (command_id << 8) | controller_id
        can_id = (self.CAN_PACKET_SET_RPM << 8) | controller_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=True,
            data=data,
        )

        try:
            self.bus.send(msg)
        except can.CanError as exc:
            self.get_logger().error(f'CAN send failed for controller {controller_id}: {exc}')

    def destroy_node(self):
        try:
            if hasattr(self, 'bus') and self.bus is not None:
                self.bus.shutdown()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RpmToVescCan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
