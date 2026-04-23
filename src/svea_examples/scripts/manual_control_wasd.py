#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl


@dataclass
class CommandState:
    steer_target: float = 0.0      # y target in [-1000, 1000]
    throttle_target: float = 500.0 # z target in [0, 1000], neutral 500
    diff_on: bool = True
    high_gear: bool = False
    aux4_on: bool = False
    aux5_on: bool = False


class RawTerminal:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old = None

    def __enter__(self):
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read_key(self, timeout_s: float = 0.0):
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            # Handle arrow keys: ESC [ A/B/C/D
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not ready:
                return ch
            second = sys.stdin.read(1)
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            third = sys.stdin.read(1) if ready else ""
            seq = ch + second + third
            return seq
        return ch


class ManualControlWasd(Node):
    def __init__(self):
        super().__init__("manual_control_wasd")
        self.pub = self.create_publisher(ManualControl, "/mavros/manual_control/send", 10)
        self.state = CommandState()

        self.rate_hz = 20.0
        self.steer_step = 170.0
        self.throttle_step = 90.0
        self.steer_return_rate = 1400.0
        self.throttle_return_rate = 1100.0
        self.input_hold_window_s = 0.14
        self.active_decay_scale = 0.10
        self.min_steer = -1000.0
        self.max_steer = 1000.0
        self.min_throttle = 0.0
        self.max_throttle = 1000.0
        self._last_publish_ts = time.monotonic()
        self._last_steer_input_ts = self._last_publish_ts
        self._last_throttle_input_ts = self._last_publish_ts

        # First-order low-pass smoothing. Smaller tau => more responsive.
        self.steer_tau_s = 0.14
        self.throttle_tau_s = 0.12

        # Smoothed command actually published.
        self._y_cmd = 0.0
        self._z_cmd = 500.0

        # Binary outputs mapped in board defaults:
        # aux1: front diff, aux2: rear diff, aux3: gear, aux4/aux5: misc.
        #
        # Old LLI behavior parity:
        # - diff ON  -> front high pulse, rear low pulse
        # - gear HIGH -> low pulse
        self._front_diff_on = 1000.0
        self._front_diff_off = -1000.0
        self._rear_diff_on = -1000.0
        self._rear_diff_off = 1000.0
        self._gear_high = -1000.0
        self._gear_low = 1000.0
        self._aux_on = 1000.0
        self._aux_off = -1000.0

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_manual_control)
        self.print_help()
        self.print_state("startup")

    def clamp(self):
        self.state.steer_target = max(self.min_steer, min(self.max_steer, self.state.steer_target))
        self.state.throttle_target = max(self.min_throttle, min(self.max_throttle, self.state.throttle_target))

    @staticmethod
    def _lowpass_alpha(dt: float, tau_s: float) -> float:
        if tau_s <= 1e-3:
            return 1.0
        return 1.0 - math.exp(-dt / tau_s)

    @staticmethod
    def _move_towards(current: float, target: float, max_step: float) -> float:
        if current < target:
            return min(current + max_step, target)
        if current > target:
            return max(current - max_step, target)
        return current

    def publish_manual_control(self):
        dt = max(1e-3, time.monotonic() - self._last_publish_ts)
        self._last_publish_ts = time.monotonic()

        # Spring-to-center behavior on target values.
        # While key-repeat is active on an axis, reduce decay so input can dominate.
        now = time.monotonic()
        steer_active = (now - self._last_steer_input_ts) <= self.input_hold_window_s
        throttle_active = (now - self._last_throttle_input_ts) <= self.input_hold_window_s
        steer_decay = self.steer_return_rate * (self.active_decay_scale if steer_active else 1.0)
        throttle_decay = self.throttle_return_rate * (self.active_decay_scale if throttle_active else 1.0)

        self.state.steer_target = self._move_towards(
            self.state.steer_target, 0.0, steer_decay * dt
        )
        self.state.throttle_target = self._move_towards(
            self.state.throttle_target, 500.0, throttle_decay * dt
        )
        self.clamp()

        steer_alpha = self._lowpass_alpha(dt, self.steer_tau_s)
        throttle_alpha = self._lowpass_alpha(dt, self.throttle_tau_s)
        self._y_cmd += steer_alpha * (self.state.steer_target - self._y_cmd)
        self._z_cmd += throttle_alpha * (self.state.throttle_target - self._z_cmd)
        self._y_cmd = max(self.min_steer, min(self.max_steer, self._y_cmd))
        self._z_cmd = max(self.min_throttle, min(self.max_throttle, self._z_cmd))
        self.print_live_state()

        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "manual_control_wasd"
        msg.x = 0.0
        msg.y = self._y_cmd
        msg.z = self._z_cmd
        msg.r = 0.0
        msg.buttons = 0
        msg.buttons2 = 0
        msg.enabled_extensions = 252  # aux1..aux6
        msg.s = 0.0
        msg.t = 0.0
        msg.aux1 = self._front_diff_on if self.state.diff_on else self._front_diff_off
        msg.aux2 = self._rear_diff_on if self.state.diff_on else self._rear_diff_off
        msg.aux3 = self._gear_high if self.state.high_gear else self._gear_low
        msg.aux4 = self._aux_on if self.state.aux4_on else self._aux_off
        msg.aux5 = self._aux_on if self.state.aux5_on else self._aux_off
        msg.aux6 = self._aux_off
        self.pub.publish(msg)

    def print_help(self):
        lines = [
            "",
            "manual_control_wasd keys:",
            "  w/s or Up/Down    feed throttle +/- (target always decays to 500)",
            "  a/d or Left/Right feed steering +/- (target always decays to 0)",
            "  c                 center steering",
            "  v                 throttle neutral (500)",
            "  x                 center steering + throttle",
            "  f                 toggle diff lock (default ON)",
            "  g                 toggle high gear",
            "  1                 toggle aux4",
            "  2                 toggle aux5",
            "  r                 reset defaults",
            "  h                 print help",
            "  q                 quit",
            "",
            "Safety: CH5 must be low (~1000) for PX4 to accept MAVLink manual control.",
            "Targets auto-return to center; hold keys to maintain offset via key-repeat.",
            "Low-pass filter smooths published commands so outputs do not snap.",
        ]
        for line in lines:
            self.get_logger().info(line)

    def print_state(self, reason: str):
        steer_str = f"{self.state.steer_target:.0f}"
        thr_str = f"{self.state.throttle_target:.0f}"
        self.get_logger().info(
            f"[{reason}] steer_target={steer_str} throttle_target={thr_str} "
            f"diff={'ON' if self.state.diff_on else 'OFF'} "
            f"gear={'HIGH' if self.state.high_gear else 'LOW'} "
            f"aux4={'ON' if self.state.aux4_on else 'OFF'} "
            f"aux5={'ON' if self.state.aux5_on else 'OFF'}"
        )

    def print_live_state(self):
        line = (
            f"y={self._y_cmd:7.1f} z={self._z_cmd:7.1f} "
            f"yt={self.state.steer_target:7.1f} zt={self.state.throttle_target:7.1f} "
            f"diff={'ON' if self.state.diff_on else 'OFF'} "
            f"gear={'HIGH' if self.state.high_gear else 'LOW'} "
            f"aux4={'ON' if self.state.aux4_on else 'OFF'} "
            f"aux5={'ON' if self.state.aux5_on else 'OFF'}"
        )
        sys.stdout.write("\r" + line)
        sys.stdout.flush()

    def handle_key(self, key: str):
        if key in ("w", "W", "\x1b[A"):
            self.state.throttle_target += self.throttle_step
            self._last_throttle_input_ts = time.monotonic()
        elif key in ("s", "S", "\x1b[B"):
            self.state.throttle_target -= self.throttle_step
            self._last_throttle_input_ts = time.monotonic()
        elif key in ("a", "A", "\x1b[D"):
            self.state.steer_target -= self.steer_step
            self._last_steer_input_ts = time.monotonic()
        elif key in ("d", "D", "\x1b[C"):
            self.state.steer_target += self.steer_step
            self._last_steer_input_ts = time.monotonic()
        elif key == "c":
            self.state.steer_target = 0.0
        elif key == "v":
            self.state.throttle_target = 500.0
        elif key == "x":
            self.state.steer_target = 0.0
            self.state.throttle_target = 500.0
        elif key == "f":
            self.state.diff_on = not self.state.diff_on
        elif key == "g":
            self.state.high_gear = not self.state.high_gear
        elif key == "1":
            self.state.aux4_on = not self.state.aux4_on
        elif key == "2":
            self.state.aux5_on = not self.state.aux5_on
        elif key == "r":
            self.state = CommandState()
        elif key == "h":
            self.print_help()
            return False
        elif key == "q":
            return True
        else:
            return False

        self.clamp()
        self.print_state("key")
        return False

    def send_stop_once(self):
        if not rclpy.ok():
            return
        self.state.steer_target = 0.0
        self.state.throttle_target = 500.0
        self._y_cmd = 0.0
        self._z_cmd = 500.0
        stop = ManualControl()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "manual_control_wasd"
        stop.x = 0.0
        stop.y = self._y_cmd
        stop.z = self._z_cmd
        stop.r = 0.0
        stop.buttons = 0
        stop.buttons2 = 0
        stop.enabled_extensions = 252
        stop.s = 0.0
        stop.t = 0.0
        stop.aux1 = self._front_diff_on if self.state.diff_on else self._front_diff_off
        stop.aux2 = self._rear_diff_on if self.state.diff_on else self._rear_diff_off
        stop.aux3 = self._gear_high if self.state.high_gear else self._gear_low
        stop.aux4 = self._aux_on if self.state.aux4_on else self._aux_off
        stop.aux5 = self._aux_on if self.state.aux5_on else self._aux_off
        stop.aux6 = self._aux_off
        try:
            self.pub.publish(stop)
        except Exception:
            # Shutdown race: context may already be invalid during Ctrl-C teardown.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlWasd()

    try:
        with RawTerminal() as term:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                key = term.read_key(timeout_s=0.01)
                if key is None:
                    continue
                should_quit = node.handle_key(key)
                if should_quit:
                    break
                # Drain queued key events so combined inputs (e.g. w+d) stack in the same cycle.
                while True:
                    key = term.read_key(timeout_s=0.0)
                    if key is None:
                        break
                    should_quit = node.handle_key(key)
                    if should_quit:
                        break
                if should_quit:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop_once()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
