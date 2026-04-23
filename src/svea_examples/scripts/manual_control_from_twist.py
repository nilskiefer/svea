#!/usr/bin/env python3

from dataclasses import dataclass
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Bool, Empty


@dataclass
class AuxState:
    diff_on: bool = True
    high_gear: bool = False
    aux4_on: bool = False
    aux5_on: bool = False


class ManualControlFromTwist(Node):
    def __init__(self):
        super().__init__("manual_control_from_twist")

        self.declare_parameter("twist_topic", "/cmd_vel")
        self.declare_parameter("twist_stamped_topic", "/cmd_vel_stamped")
        self.declare_parameter("publish_topic", "/mavros/manual_control/send")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.30)
        self.declare_parameter("steering_gain", 1000.0)    # y = angular.z * gain
        self.declare_parameter("throttle_gain", 500.0)     # z = 500 + linear.x * gain

        self.declare_parameter("steer_rise_rate", 2600.0)
        self.declare_parameter("steer_fall_rate", 3400.0)
        self.declare_parameter("steer_crossover_rate", 5400.0)
        self.declare_parameter("throttle_rise_rate", 1000.0)
        self.declare_parameter("throttle_fall_rate", 1600.0)
        self.declare_parameter("throttle_crossover_rate", 2600.0)

        self.declare_parameter("default_diff_on", True)
        self.declare_parameter("default_high_gear", False)

        self.twist_topic = self.get_parameter("twist_topic").get_parameter_value().string_value
        self.twist_stamped_topic = self.get_parameter("twist_stamped_topic").get_parameter_value().string_value
        self.publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value

        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.command_timeout_s = self.get_parameter("command_timeout_s").get_parameter_value().double_value
        self.steering_gain = self.get_parameter("steering_gain").get_parameter_value().double_value
        self.throttle_gain = self.get_parameter("throttle_gain").get_parameter_value().double_value

        self.steer_rise_rate = self.get_parameter("steer_rise_rate").get_parameter_value().double_value
        self.steer_fall_rate = self.get_parameter("steer_fall_rate").get_parameter_value().double_value
        self.steer_crossover_rate = self.get_parameter("steer_crossover_rate").get_parameter_value().double_value
        self.throttle_rise_rate = self.get_parameter("throttle_rise_rate").get_parameter_value().double_value
        self.throttle_fall_rate = self.get_parameter("throttle_fall_rate").get_parameter_value().double_value
        self.throttle_crossover_rate = self.get_parameter("throttle_crossover_rate").get_parameter_value().double_value

        self.aux = AuxState(
            diff_on=self.get_parameter("default_diff_on").get_parameter_value().bool_value,
            high_gear=self.get_parameter("default_high_gear").get_parameter_value().bool_value,
        )

        self._lin_x = 0.0
        self._ang_z = 0.0
        self._last_cmd_ts = 0.0

        self._y_cmd = 0.0
        self._z_cmd = 500.0
        self._last_publish_ts = time.monotonic()

        # Old LLI parity:
        # diff ON: front high pulse, rear low pulse
        # gear HIGH: low pulse
        self._front_diff_on = 1000.0
        self._front_diff_off = -1000.0
        self._rear_diff_on = -1000.0
        self._rear_diff_off = 1000.0
        self._gear_high = -1000.0
        self._gear_low = 1000.0
        self._aux_on = 1000.0
        self._aux_off = -1000.0

        self.pub = self.create_publisher(ManualControl, self.publish_topic, 10)

        self.create_subscription(Twist, self.twist_topic, self._on_twist, 10)
        self.create_subscription(TwistStamped, self.twist_stamped_topic, self._on_twist_stamped, 10)

        # Button/aux control topics (arbitrary controls while using key_teleop for axes).
        self.create_subscription(Bool, "/svea/key/diff_on/set", self._set_diff, 10)
        self.create_subscription(Empty, "/svea/key/diff_on/toggle", self._toggle_diff, 10)
        self.create_subscription(Bool, "/svea/key/high_gear/set", self._set_high_gear, 10)
        self.create_subscription(Empty, "/svea/key/high_gear/toggle", self._toggle_high_gear, 10)
        self.create_subscription(Bool, "/svea/key/aux4_on/set", self._set_aux4, 10)
        self.create_subscription(Empty, "/svea/key/aux4_on/toggle", self._toggle_aux4, 10)
        self.create_subscription(Bool, "/svea/key/aux5_on/set", self._set_aux5, 10)
        self.create_subscription(Empty, "/svea/key/aux5_on/toggle", self._toggle_aux5, 10)

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_manual_control)

        self.get_logger().info(
            f"bridge ready: twist='{self.twist_topic}', twist_stamped='{self.twist_stamped_topic}', "
            f"publish='{self.publish_topic}', rate={self.publish_rate_hz:.1f}Hz"
        )

    @staticmethod
    def _slew_towards(current: float, target: float, rate: float, dt: float) -> float:
        max_step = rate * dt
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + max_step if delta > 0.0 else current - max_step

    def _on_twist(self, msg: Twist):
        self._lin_x = float(msg.linear.x)
        self._ang_z = float(msg.angular.z)
        self._last_cmd_ts = time.monotonic()

    def _on_twist_stamped(self, msg: TwistStamped):
        self._lin_x = float(msg.twist.linear.x)
        self._ang_z = float(msg.twist.angular.z)
        self._last_cmd_ts = time.monotonic()

    def _set_diff(self, msg: Bool):
        self.aux.diff_on = bool(msg.data)
        self.get_logger().info(f"diff_on={self.aux.diff_on}")

    def _toggle_diff(self, _: Empty):
        self.aux.diff_on = not self.aux.diff_on
        self.get_logger().info(f"diff_on={self.aux.diff_on}")

    def _set_high_gear(self, msg: Bool):
        self.aux.high_gear = bool(msg.data)
        self.get_logger().info(f"high_gear={self.aux.high_gear}")

    def _toggle_high_gear(self, _: Empty):
        self.aux.high_gear = not self.aux.high_gear
        self.get_logger().info(f"high_gear={self.aux.high_gear}")

    def _set_aux4(self, msg: Bool):
        self.aux.aux4_on = bool(msg.data)
        self.get_logger().info(f"aux4_on={self.aux.aux4_on}")

    def _toggle_aux4(self, _: Empty):
        self.aux.aux4_on = not self.aux.aux4_on
        self.get_logger().info(f"aux4_on={self.aux.aux4_on}")

    def _set_aux5(self, msg: Bool):
        self.aux.aux5_on = bool(msg.data)
        self.get_logger().info(f"aux5_on={self.aux.aux5_on}")

    def _toggle_aux5(self, _: Empty):
        self.aux.aux5_on = not self.aux.aux5_on
        self.get_logger().info(f"aux5_on={self.aux.aux5_on}")

    def _publish_manual_control(self):
        now = time.monotonic()
        dt = max(1e-3, now - self._last_publish_ts)
        self._last_publish_ts = now

        stale = (now - self._last_cmd_ts) > self.command_timeout_s
        lin_x = 0.0 if stale else self._lin_x
        ang_z = 0.0 if stale else self._ang_z

        y_target = max(-1000.0, min(1000.0, ang_z * self.steering_gain))
        z_target = max(0.0, min(1000.0, 500.0 + lin_x * self.throttle_gain))

        y_crossing = (self._y_cmd > 0.0 > y_target) or (self._y_cmd < 0.0 < y_target)
        z_offset = self._z_cmd - 500.0
        z_target_offset = z_target - 500.0
        z_crossing = (z_offset > 0.0 > z_target_offset) or (z_offset < 0.0 < z_target_offset)

        if y_crossing:
            y_rate = self.steer_crossover_rate
        else:
            y_rate = self.steer_fall_rate if abs(y_target) < abs(self._y_cmd) else self.steer_rise_rate

        if z_crossing:
            z_rate = self.throttle_crossover_rate
        else:
            z_rate = self.throttle_fall_rate if abs(z_target_offset) < abs(z_offset) else self.throttle_rise_rate

        self._y_cmd = self._slew_towards(self._y_cmd, y_target, y_rate, dt)
        self._z_cmd = self._slew_towards(self._z_cmd, z_target, z_rate, dt)
        self._z_cmd = max(0.0, min(1000.0, self._z_cmd))

        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "manual_control_from_twist"
        msg.x = 0.0
        msg.y = self._y_cmd
        msg.z = self._z_cmd
        msg.r = 0.0
        msg.buttons = 0
        msg.buttons2 = 0
        msg.enabled_extensions = 252
        msg.s = 0.0
        msg.t = 0.0
        msg.aux1 = self._front_diff_on if self.aux.diff_on else self._front_diff_off
        msg.aux2 = self._rear_diff_on if self.aux.diff_on else self._rear_diff_off
        msg.aux3 = self._gear_high if self.aux.high_gear else self._gear_low
        msg.aux4 = self._aux_on if self.aux.aux4_on else self._aux_off
        msg.aux5 = self._aux_on if self.aux.aux5_on else self._aux_off
        msg.aux6 = self._aux_off
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlFromTwist()
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
