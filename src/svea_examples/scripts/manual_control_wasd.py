#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl


@dataclass
class CommandState:
    steer_dir: int = 0             # -1 left, +1 right
    throttle_dir: int = 0          # -1 reverse, +1 forward
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
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if ready:
                second = sys.stdin.read(1)
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
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
        self.steer_mag = 900.0
        self.throttle_mag = 350.0
        self.motion_timeout_s = 0.20
        self._last_steer_ts = time.monotonic()
        self._last_throttle_ts = time.monotonic()
        self._last_publish_ts = time.monotonic()

        # Game-like response shaping (units per second in MAVLink manual-control space).
        # y uses [-1000, 1000], z uses [0, 1000] with neutral at 500.
        self.steer_rise_rate = 2400.0
        self.steer_fall_rate = 3200.0
        self.steer_crossover_rate = 5200.0
        self.throttle_rise_rate = 900.0
        self.throttle_fall_rate = 1400.0
        self.throttle_crossover_rate = 2400.0

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
        self.state.steer_dir = max(-1, min(1, self.state.steer_dir))
        self.state.throttle_dir = max(-1, min(1, self.state.throttle_dir))

    @staticmethod
    def _slew_towards(current: float, target: float, rate: float, dt: float) -> float:
        max_step = rate * dt
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + max_step if delta > 0.0 else current - max_step

    def publish_manual_control(self):
        now = time.monotonic()
        dt = max(1e-3, now - self._last_publish_ts)
        self._last_publish_ts = now
        steer_alive = (now - self._last_steer_ts) <= self.motion_timeout_s
        throttle_alive = (now - self._last_throttle_ts) <= self.motion_timeout_s

        y_target = float(self.state.steer_dir) * self.steer_mag if steer_alive else 0.0
        z_target = 500.0 + float(self.state.throttle_dir) * self.throttle_mag if throttle_alive else 500.0
        z_target = max(0.0, min(1000.0, z_target))

        # Use faster decay than rise so release feels responsive but not abrupt.
        # If crossing from one side to the other, use dedicated crossover rate.
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
            "  w/s or Up/Down    throttle +/-",
            "  a/d or Left/Right steering +/-",
            "  c                 center steering",
            "  v                 throttle neutral (500)",
            "  f                 toggle diff lock (default ON)",
            "  g                 toggle high gear",
            "  1                 toggle aux4",
            "  2                 toggle aux5",
            "  r                 reset defaults",
            "  h                 print help",
            "  q                 quit",
            "",
            "Safety: CH5 must be low (~1000) for PX4 to accept MAVLink manual control.",
            f"Deadman: steer/throttle auto-neutral after {self.motion_timeout_s:.2f}s without motion key repeats.",
            "Smoothing: commands ramp up/down (game-like), they do not snap to target.",
            "Direction changes use faster crossover ramp (right->left / fwd->rev).",
        ]
        for line in lines:
            self.get_logger().info(line)

    def print_state(self, reason: str):
        steer_str = "RIGHT" if self.state.steer_dir > 0 else "LEFT" if self.state.steer_dir < 0 else "CENTER"
        thr_str = "FWD" if self.state.throttle_dir > 0 else "REV" if self.state.throttle_dir < 0 else "NEUTRAL"
        self.get_logger().info(
            f"[{reason}] steer={steer_str} throttle={thr_str} "
            f"diff={'ON' if self.state.diff_on else 'OFF'} "
            f"gear={'HIGH' if self.state.high_gear else 'LOW'} "
            f"aux4={'ON' if self.state.aux4_on else 'OFF'} "
            f"aux5={'ON' if self.state.aux5_on else 'OFF'}"
        )

    def handle_key(self, key: str):
        now = time.monotonic()
        if key in ("w", "\x1b[A"):
            self.state.throttle_dir = 1
            self._last_throttle_ts = now
        elif key in ("s", "\x1b[B"):
            self.state.throttle_dir = -1
            self._last_throttle_ts = now
        elif key in ("a", "\x1b[D"):
            self.state.steer_dir = -1
            self._last_steer_ts = now
        elif key in ("d", "\x1b[C"):
            self.state.steer_dir = 1
            self._last_steer_ts = now
        elif key == "c":
            self.state.steer_dir = 0
            self._last_steer_ts = now
        elif key == "v":
            self.state.throttle_dir = 0
            self._last_throttle_ts = now
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
        self.pub.publish(stop)


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
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop_once()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
