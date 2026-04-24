#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import ManualControl
from sensor_msgs.msg import Joy


class ManualControlFromCmdVel(Node):
    def __init__(self):
        super().__init__("manual_control_from_cmd_vel")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("publish_topic", "/mavros/manual_control/send")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.30)
        self.declare_parameter("steering_axis", 0)
        self.declare_parameter("throttle_axis", 5)
        self.declare_parameter("steering_gain", 1000.0)
        self.declare_parameter("throttle_gain", 500.0)
        self.declare_parameter("throttle_axis_idle_value", 1.0)
        self.declare_parameter("throttle_axis_full_value", -1.0)
        self.declare_parameter("require_enable_button", True)
        self.declare_parameter("enable_button", 5)
        self.declare_parameter("default_diff_on", True)
        self.declare_parameter("default_high_gear", False)

        self.joy_topic = self.get_parameter("joy_topic").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.steering_axis = int(self.get_parameter("steering_axis").value)
        self.throttle_axis = int(self.get_parameter("throttle_axis").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.throttle_gain = float(self.get_parameter("throttle_gain").value)
        self.throttle_axis_idle_value = float(
            self.get_parameter("throttle_axis_idle_value").value
        )
        self.throttle_axis_full_value = float(
            self.get_parameter("throttle_axis_full_value").value
        )
        self.require_enable_button = bool(
            self.get_parameter("require_enable_button").value
        )
        self.enable_button = int(self.get_parameter("enable_button").value)
        self.diff_on = bool(self.get_parameter("default_diff_on").value)
        self.high_gear = bool(self.get_parameter("default_high_gear").value)

        self._steering_axis_value = 0.0
        self._throttle_axis_value = self.throttle_axis_idle_value
        self._enable_pressed = not self.require_enable_button
        self._last_cmd_ts = 0.0

        self._diff_on = 1000.0
        self._diff_off = -1000.0
        self._gear_high = -1000.0
        self._gear_low = 1000.0
        self._aux_on = 1000.0
        self._aux_off = -1000.0

        self.pub = self.create_publisher(ManualControl, self.publish_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_manual_control)

        self.get_logger().info(
            f"bridge ready: joy='{self.joy_topic}', publish='{self.publish_topic}', rate={self.publish_rate_hz:.1f}Hz"
        )

    def _on_joy(self, msg: Joy):
        if self.steering_axis >= len(msg.axes) or self.throttle_axis >= len(msg.axes):
            raise RuntimeError(
                f"Joy axis out of range: steering_axis={self.steering_axis}, throttle_axis={self.throttle_axis}, axes_len={len(msg.axes)}"
            )

        if self.require_enable_button:
            if self.enable_button >= len(msg.buttons):
                raise RuntimeError(
                    f"Enable button out of range: enable_button={self.enable_button}, buttons_len={len(msg.buttons)}"
                )
            self._enable_pressed = bool(msg.buttons[self.enable_button])
        else:
            self._enable_pressed = True

        self._steering_axis_value = float(msg.axes[self.steering_axis])
        self._throttle_axis_value = float(msg.axes[self.throttle_axis])
        self._last_cmd_ts = time.monotonic()

    def _publish_manual_control(self):
        now = time.monotonic()
        stale = (now - self._last_cmd_ts) > self.command_timeout_s
        active = (not stale) and self._enable_pressed

        if active:
            steering = self._steering_axis_value
            denom = self.throttle_axis_idle_value - self.throttle_axis_full_value
            if abs(denom) < 1e-6:
                raise RuntimeError(
                    "Invalid throttle mapping: throttle_axis_idle_value equals throttle_axis_full_value"
                )
            throttle_norm = (self.throttle_axis_idle_value - self._throttle_axis_value) / denom
            throttle_norm = max(0.0, min(1.0, throttle_norm))
        else:
            steering = 0.0
            throttle_norm = 0.0

        y = max(-1000.0, min(1000.0, steering * self.steering_gain))
        z = max(0.0, min(1000.0, 500.0 + throttle_norm * self.throttle_gain))

        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "manual_control_from_joy"
        msg.x = 0.0
        msg.y = y
        msg.z = z
        msg.r = 0.0
        msg.buttons = 0
        msg.buttons2 = 0
        msg.enabled_extensions = 252
        msg.s = 0.0
        msg.t = 0.0
        msg.aux1 = self._diff_on if self.diff_on else self._diff_off
        msg.aux2 = self._diff_on if self.diff_on else self._diff_off
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
