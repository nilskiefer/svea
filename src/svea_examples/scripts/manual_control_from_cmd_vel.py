#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl


class ManualControlFromCmdVel(Node):
    def __init__(self):
        super().__init__("manual_control_from_cmd_vel")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_topic", "/mavros/manual_control/send")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.30)
        self.declare_parameter("steering_gain", 1000.0)   # y = angular.z * gain
        self.declare_parameter("throttle_gain", 500.0)    # z = 500 + linear.x * gain
        self.declare_parameter("default_diff_on", True)
        self.declare_parameter("default_high_gear", False)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.throttle_gain = float(self.get_parameter("throttle_gain").value)
        self.diff_on = bool(self.get_parameter("default_diff_on").value)
        self.high_gear = bool(self.get_parameter("default_high_gear").value)

        self._lin_x = 0.0
        self._ang_z = 0.0
        self._last_cmd_ts = 0.0

        # Keep old LLI polarity.
        self._front_diff_on = 1000.0
        self._front_diff_off = -1000.0
        self._rear_diff_on = -1000.0
        self._rear_diff_off = 1000.0
        self._gear_high = -1000.0
        self._gear_low = 1000.0
        self._aux_on = 1000.0
        self._aux_off = -1000.0

        self.pub = self.create_publisher(ManualControl, self.publish_topic, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_manual_control)

        self.get_logger().info(
            f"bridge ready: cmd_vel='{self.cmd_vel_topic}', publish='{self.publish_topic}', rate={self.publish_rate_hz:.1f}Hz"
        )

    def _on_cmd_vel(self, msg: Twist):
        self._lin_x = float(msg.linear.x)
        self._ang_z = float(msg.angular.z)
        self._last_cmd_ts = time.monotonic()

    def _publish_manual_control(self):
        stale = (time.monotonic() - self._last_cmd_ts) > self.command_timeout_s
        lin_x = 0.0 if stale else self._lin_x
        ang_z = 0.0 if stale else self._ang_z

        y = max(-1000.0, min(1000.0, ang_z * self.steering_gain))
        z = max(0.0, min(1000.0, 500.0 + lin_x * self.throttle_gain))

        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "manual_control_from_cmd_vel"
        msg.x = 0.0
        msg.y = y
        msg.z = z
        msg.r = 0.0
        msg.buttons = 0
        msg.buttons2 = 0
        msg.enabled_extensions = 252
        msg.s = 0.0
        msg.t = 0.0
        msg.aux1 = self._front_diff_on if self.diff_on else self._front_diff_off
        msg.aux2 = self._rear_diff_on if self.diff_on else self._rear_diff_off
        msg.aux3 = self._gear_high if self.high_gear else self._gear_low
        msg.aux4 = self._aux_off
        msg.aux5 = self._aux_off
        msg.aux6 = self._aux_off
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlFromCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
