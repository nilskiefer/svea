#!/usr/bin/env python3
"""
Stub publisher for host->MCU control path through svea_custom_protocol_bridge.

Publishes svea_msgs/LLIControl on lli/ctrl_request at a fixed rate.
"""

from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node
from svea_msgs.msg import LLIControl


def clamp_i8(value: int) -> int:
    return max(-128, min(127, int(value)))


class CustomProtocolCtrlStub(Node):
    def __init__(self) -> None:
        super().__init__("svea_custom_protocol_ctrl_stub")

        self.declare_parameter("ctrl_request_topic", "lli/ctrl_request")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("mode", "random")  # random | sine
        self.declare_parameter("random_bools", True)

        self._topic = str(self.get_parameter("ctrl_request_topic").value)
        self._rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self._mode = str(self.get_parameter("mode").value).strip().lower()
        self._random_bools = bool(self.get_parameter("random_bools").value)
        self._seq = 0

        self._pub = self.create_publisher(LLIControl, self._topic, 10)
        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f"Publishing LLIControl stubs on {self._topic} at {self._rate_hz:.1f} Hz (mode={self._mode})"
        )

    def _tick(self) -> None:
        msg = LLIControl()

        if self._mode == "sine":
            phase = self._seq / 50.0
            msg.steering = clamp_i8(int(round(100.0 * math.sin(phase))))
            msg.velocity = clamp_i8(int(round(80.0 * math.sin(phase * 0.6))))
        else:
            msg.steering = clamp_i8(random.randint(-100, 100))
            msg.velocity = clamp_i8(random.randint(-100, 100))

        trans_diff = 0b00111000  # update all three flags
        if self._random_bools:
            if random.getrandbits(1):
                trans_diff |= 0b00000001  # high gear
            if random.getrandbits(1):
                trans_diff |= 0b00000010  # front diff
            if random.getrandbits(1):
                trans_diff |= 0b00000100  # rear diff
        msg.trans_diff = trans_diff
        msg.ctrl = 0

        self._pub.publish(msg)
        self._seq += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: CustomProtocolCtrlStub | None = None
    try:
        node = CustomProtocolCtrlStub()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
