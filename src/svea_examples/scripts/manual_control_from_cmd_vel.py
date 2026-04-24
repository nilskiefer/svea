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
        self.declare_parameter("steering_axis", 0)  # first axis
        self.declare_parameter("steering_inverted", True)
        self.declare_parameter("throttle_axis", 5)  # sixth axis (forward)
        self.declare_parameter("reverse_axis", 2)   # third axis (reverse)
        self.declare_parameter("steering_gain", 1000.0)
        self.declare_parameter("throttle_gain", 500.0)  # +/- from neutral 500
        self.declare_parameter("throttle_axis_idle_value", 1.0)
        self.declare_parameter("throttle_axis_full_value", -1.0)
        self.declare_parameter("reverse_axis_idle_value", 1.0)
        self.declare_parameter("reverse_axis_full_value", -1.0)
        self.declare_parameter("toggle_diff_button", 0)
        self.declare_parameter("toggle_gear_button", 1)
        self.declare_parameter("toggle_aux4_button", 2)
        self.declare_parameter("toggle_aux5_button", 3)
        self.declare_parameter("proportional_aux_channel", 5)
        self.declare_parameter("aux2_axis", 5)
        self.declare_parameter("aux2_axis_deadzone", 0.10)
        self.declare_parameter("aux2_rate_per_s", 900.0)
        self.declare_parameter("aux2_default", 0.0)
        self.declare_parameter("default_diff_on", True)
        self.declare_parameter("default_high_gear", False)
        self.declare_parameter("default_aux4_on", False)
        self.declare_parameter("default_aux5_on", False)

        self.joy_topic = self.get_parameter("joy_topic").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.steering_axis = int(self.get_parameter("steering_axis").value)
        self.steering_inverted = bool(self.get_parameter("steering_inverted").value)
        self.throttle_axis = int(self.get_parameter("throttle_axis").value)
        self.reverse_axis = int(self.get_parameter("reverse_axis").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.throttle_gain = float(self.get_parameter("throttle_gain").value)
        self.throttle_axis_idle_value = float(
            self.get_parameter("throttle_axis_idle_value").value
        )
        self.throttle_axis_full_value = float(
            self.get_parameter("throttle_axis_full_value").value
        )
        self.reverse_axis_idle_value = float(
            self.get_parameter("reverse_axis_idle_value").value
        )
        self.reverse_axis_full_value = float(
            self.get_parameter("reverse_axis_full_value").value
        )
        self.toggle_diff_button = int(self.get_parameter("toggle_diff_button").value)
        self.toggle_gear_button = int(self.get_parameter("toggle_gear_button").value)
        self.toggle_aux4_button = int(self.get_parameter("toggle_aux4_button").value)
        self.toggle_aux5_button = int(self.get_parameter("toggle_aux5_button").value)
        self.proportional_aux_channel = int(
            self.get_parameter("proportional_aux_channel").value
        )
        self.aux2_axis = int(self.get_parameter("aux2_axis").value)
        self.aux2_axis_deadzone = float(self.get_parameter("aux2_axis_deadzone").value)
        self.aux2_rate_per_s = float(self.get_parameter("aux2_rate_per_s").value)
        self.aux2_default = float(self.get_parameter("aux2_default").value)
        self.diff_on = bool(self.get_parameter("default_diff_on").value)
        self.high_gear = bool(self.get_parameter("default_high_gear").value)
        self.aux4_on = bool(self.get_parameter("default_aux4_on").value)
        self.aux5_on = bool(self.get_parameter("default_aux5_on").value)

        self._steering_axis_value = 0.0
        self._throttle_axis_value = self.throttle_axis_idle_value
        self._reverse_axis_value = self.reverse_axis_idle_value
        self._aux2_axis_value = 0.0
        self._aux_value = max(-1000.0, min(1000.0, self.aux2_default))
        self._last_buttons = []
        self._last_cmd_ts = 0.0
        self._last_publish_ts = time.monotonic()

        self._diff_on = 1000.0
        self._diff_off = -1000.0
        self._gear_high = -1000.0
        self._gear_low = 1000.0
        self._aux_on = 1000.0
        self._aux_off = -1000.0

        self.pub = self.create_publisher(ManualControl, self.publish_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_manual_control)

        if self.proportional_aux_channel < 1 or self.proportional_aux_channel > 6:
            raise RuntimeError(
                f"Invalid proportional_aux_channel={self.proportional_aux_channel}, expected 1..6"
            )

        self.get_logger().info(
            f"bridge ready: joy='{self.joy_topic}', publish='{self.publish_topic}', "
            f"rate={self.publish_rate_hz:.1f}Hz, proportional_aux_channel={self.proportional_aux_channel}"
        )

    def _on_joy(self, msg: Joy):
        if (
            self.steering_axis >= len(msg.axes)
            or self.throttle_axis >= len(msg.axes)
            or self.reverse_axis >= len(msg.axes)
            or self.aux2_axis >= len(msg.axes)
        ):
            raise RuntimeError(
                "Joy axis out of range: "
                f"steering_axis={self.steering_axis}, "
                f"throttle_axis={self.throttle_axis}, "
                f"reverse_axis={self.reverse_axis}, "
                f"aux2_axis={self.aux2_axis}, "
                f"axes_len={len(msg.axes)}"
            )

        toggle_buttons = [
            self.toggle_diff_button,
            self.toggle_gear_button,
            self.toggle_aux4_button,
            self.toggle_aux5_button,
        ]
        for idx in toggle_buttons:
            if idx >= len(msg.buttons):
                raise RuntimeError(
                    f"Toggle button out of range: button={idx}, buttons_len={len(msg.buttons)}"
                )

        if not self._last_buttons:
            self._last_buttons = [0] * len(msg.buttons)

        if msg.buttons[self.toggle_diff_button] and not self._last_buttons[self.toggle_diff_button]:
            self.diff_on = not self.diff_on
        if msg.buttons[self.toggle_gear_button] and not self._last_buttons[self.toggle_gear_button]:
            self.high_gear = not self.high_gear
        if msg.buttons[self.toggle_aux4_button] and not self._last_buttons[self.toggle_aux4_button]:
            self.aux4_on = not self.aux4_on
        if msg.buttons[self.toggle_aux5_button] and not self._last_buttons[self.toggle_aux5_button]:
            self.aux5_on = not self.aux5_on

        self._steering_axis_value = float(msg.axes[self.steering_axis])
        self._throttle_axis_value = float(msg.axes[self.throttle_axis])
        self._reverse_axis_value = float(msg.axes[self.reverse_axis])
        self._aux2_axis_value = float(msg.axes[self.aux2_axis])
        self._last_buttons = list(msg.buttons)
        self._last_cmd_ts = time.monotonic()

    @staticmethod
    def _axis_to_positive_norm(value: float, idle_value: float, full_value: float) -> float:
        denom = idle_value - full_value
        if abs(denom) < 1e-6:
            raise RuntimeError(
                "Invalid axis mapping: idle_value equals full_value"
            )
        mapped = (idle_value - value) / denom
        return max(0.0, min(1.0, mapped))

    def _publish_manual_control(self):
        now = time.monotonic()
        dt = max(0.0, now - self._last_publish_ts)
        self._last_publish_ts = now
        stale = (now - self._last_cmd_ts) > self.command_timeout_s

        if not stale:
            steering = -self._steering_axis_value if self.steering_inverted else self._steering_axis_value
            forward_norm = self._axis_to_positive_norm(
                self._throttle_axis_value,
                self.throttle_axis_idle_value,
                self.throttle_axis_full_value,
            )
            reverse_norm = self._axis_to_positive_norm(
                self._reverse_axis_value,
                self.reverse_axis_idle_value,
                self.reverse_axis_full_value,
            )
            signed_throttle = max(-1.0, min(1.0, forward_norm - reverse_norm))
            aux2_axis_cmd = self._aux2_axis_value
            if abs(aux2_axis_cmd) < self.aux2_axis_deadzone:
                aux2_axis_cmd = 0.0
            self._aux_value += aux2_axis_cmd * self.aux2_rate_per_s * dt
            self._aux_value = max(-1000.0, min(1000.0, self._aux_value))
        else:
            steering = 0.0
            signed_throttle = 0.0

        y = max(-1000.0, min(1000.0, steering * self.steering_gain))
        z = max(0.0, min(1000.0, 500.0 + signed_throttle * self.throttle_gain))

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
        aux_values = {
            1: self._diff_on if self.diff_on else self._diff_off,
            2: 0.0,
            3: self._gear_high if self.high_gear else self._gear_low,
            4: self._aux_on if self.aux4_on else self._aux_off,
            5: self._aux_on if self.aux5_on else self._aux_off,
            6: self._aux_off,
        }
        aux_values[self.proportional_aux_channel] = self._aux_value

        msg.aux1 = aux_values[1]
        msg.aux2 = aux_values[2]
        msg.aux3 = aux_values[3]
        msg.aux4 = aux_values[4]
        msg.aux5 = aux_values[5]
        msg.aux6 = aux_values[6]
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
