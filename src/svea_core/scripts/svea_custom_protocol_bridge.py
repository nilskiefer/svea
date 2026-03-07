#!/usr/bin/env python3
"""
Bridge SVEA custom-protocol telemetry (CBOR over UART) into ROS topics.

Default behavior:
- Reads framed CBOR from serial.
- Republishes each telemetry topic as std_msgs/String containing JSON.
- Republishes rc_command as svea_msgs/LLIControl on lli/remote.

Optional behavior:
- Subscribe to lli/ctrl_request and transmit SERVO commands back to firmware.
- Periodically transmit host heartbeat (HB) back to firmware.
"""

from __future__ import annotations

import json
import math
import re
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from svea_msgs.msg import LLIControl

try:
    import cbor2
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing dependency: cbor2. Install with: python3 -m pip install cbor2"
    ) from exc

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing dependency: pyserial. Install with: python3 -m pip install pyserial"
    ) from exc


CBOR_MAGIC = 0xA5
MAX_FRAME_LEN = 4096
TOPIC_NAME_RE = re.compile(r"[^a-zA-Z0-9_/]")
MG_TO_MPS2 = 9.80665e-3
MDPS_TO_RADPS = math.pi / (180.0 * 1000.0)

KNOWN_TOPICS = (
    "status",
    "heartbeat",
    "rc_command",
    "lsm6dsox",
    "ads1115",
    "ina3221_a",
    "ina3221_b",
    "bq76942",
    "ina226_a",
    "ina226_b",
)


def clamp_i8(value: int) -> int:
    return max(-128, min(127, int(value)))


def sanitize_topic_suffix(name: str) -> str:
    if not name:
        return "unknown"
    cleaned = TOPIC_NAME_RE.sub("_", name.strip())
    cleaned = cleaned.strip("_")
    return cleaned or "unknown"


def list_candidate_ports() -> list[Any]:
    candidates = []
    for port in list_ports.comports():
        dev = port.device.lower()
        if "ttyacm" in dev or "ttyusb" in dev or "usbmodem" in dev or dev.startswith("com"):
            candidates.append(port)
    return candidates


def auto_detect_port() -> str | None:
    candidates = list_candidate_ports()
    if len(candidates) == 1:
        return candidates[0].device
    return None


class SveaCustomProtocolBridge(Node):
    def __init__(self) -> None:
        super().__init__("svea_custom_protocol_bridge")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("serial_timeout", 0.001)
        self.declare_parameter("poll_period_s", 0.001)
        self.declare_parameter("max_read_chunk", 4096)
        self.declare_parameter("max_reads_per_poll", 16)
        self.declare_parameter("publish_json_topics", False)
        self.declare_parameter("topic_prefix", "svea/custom_protocol")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("remote_topic", "lli/remote")
        self.declare_parameter("imu_topic", "lli/sensor/imu")
        self.declare_parameter("imu_temp_topic", "lli/sensor/imu_temp")
        self.declare_parameter("battery_topic", "lli/battery/state")
        self.declare_parameter("ads1115_topic", "lli/sensor/ads1115")
        self.declare_parameter("ina226_a_topic", "lli/sensor/ina226_a/state")
        self.declare_parameter("ina226_b_topic", "lli/sensor/ina226_b/state")
        self.declare_parameter("ina3221_a_ch1_topic", "lli/sensor/ina3221_a/ch1/state")
        self.declare_parameter("ina3221_a_ch2_topic", "lli/sensor/ina3221_a/ch2/state")
        self.declare_parameter("ina3221_a_ch3_topic", "lli/sensor/ina3221_a/ch3/state")
        self.declare_parameter("ina3221_b_ch1_topic", "lli/sensor/ina3221_b/ch1/state")
        self.declare_parameter("ina3221_b_ch2_topic", "lli/sensor/ina3221_b/ch2/state")
        self.declare_parameter("ina3221_b_ch3_topic", "lli/sensor/ina3221_b/ch3/state")
        self.declare_parameter("enable_command_bridge", False)
        self.declare_parameter("ctrl_request_topic", "lli/ctrl_request")
        self.declare_parameter("host_heartbeat_hz", 10.0)

        self._topic_prefix = str(self.get_parameter("topic_prefix").value).strip("/")
        self._poll_period_s = float(self.get_parameter("poll_period_s").value)
        self._max_read_chunk = max(64, int(self.get_parameter("max_read_chunk").value))
        self._max_reads_per_poll = max(1, int(self.get_parameter("max_reads_per_poll").value))
        self._publish_json_topics = bool(self.get_parameter("publish_json_topics").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._remote_topic = str(self.get_parameter("remote_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._imu_temp_topic = str(self.get_parameter("imu_temp_topic").value)
        self._battery_topic = str(self.get_parameter("battery_topic").value)
        self._ads1115_topic = str(self.get_parameter("ads1115_topic").value)
        self._ina226_a_topic = str(self.get_parameter("ina226_a_topic").value)
        self._ina226_b_topic = str(self.get_parameter("ina226_b_topic").value)
        self._enable_command_bridge = bool(self.get_parameter("enable_command_bridge").value)
        self._ctrl_request_topic = str(self.get_parameter("ctrl_request_topic").value)
        self._hb_hz = float(self.get_parameter("host_heartbeat_hz").value)

        port = str(self.get_parameter("port").value).strip()
        if not port:
            port = auto_detect_port() or ""
        if not port:
            raise RuntimeError(
                "No serial port configured. Set --ros-args -p port:=/dev/ttyUSB0"
            )

        baud = int(self.get_parameter("baud").value)
        timeout = float(self.get_parameter("serial_timeout").value)
        self._serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self._serial.dtr = True
        self._last_serial_error_log = 0.0

        self._rx_buffer = bytearray()
        self._raw_pub = None
        self._unknown_pub = None
        self._topic_pubs: dict[str, Any] = {}
        if self._publish_json_topics:
            self._raw_pub = self.create_publisher(String, self._topic("raw"), qos_profile_sensor_data)
            self._unknown_pub = self.create_publisher(String, self._topic("unknown"), qos_profile_sensor_data)
            self._topic_pubs = {
                topic: self.create_publisher(String, self._topic(topic), qos_profile_sensor_data)
                for topic in KNOWN_TOPICS
            }

        self._remote_pub = self.create_publisher(LLIControl, self._remote_topic, qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(Imu, self._imu_topic, qos_profile_sensor_data)
        self._imu_temp_pub = self.create_publisher(Temperature, self._imu_temp_topic, qos_profile_sensor_data)
        self._battery_pub = self.create_publisher(BatteryState, self._battery_topic, qos_profile_sensor_data)
        self._ads1115_pub = self.create_publisher(Float32MultiArray, self._ads1115_topic, qos_profile_sensor_data)
        self._ina226_a_pub = self.create_publisher(BatteryState, self._ina226_a_topic, qos_profile_sensor_data)
        self._ina226_b_pub = self.create_publisher(BatteryState, self._ina226_b_topic, qos_profile_sensor_data)
        self._ina3221_pubs = {
            "ina3221_a": (
                self.create_publisher(
                    BatteryState,
                    str(self.get_parameter("ina3221_a_ch1_topic").value),
                    qos_profile_sensor_data,
                ),
                self.create_publisher(
                    BatteryState,
                    str(self.get_parameter("ina3221_a_ch2_topic").value),
                    qos_profile_sensor_data,
                ),
                self.create_publisher(
                    BatteryState,
                    str(self.get_parameter("ina3221_a_ch3_topic").value),
                    qos_profile_sensor_data,
                ),
            ),
            "ina3221_b": (
                self.create_publisher(
                    BatteryState,
                    str(self.get_parameter("ina3221_b_ch1_topic").value),
                    qos_profile_sensor_data,
                ),
                self.create_publisher(
                    BatteryState,
                    str(self.get_parameter("ina3221_b_ch2_topic").value),
                    qos_profile_sensor_data,
                ),
                self.create_publisher(
                    BatteryState,
                    str(self.get_parameter("ina3221_b_ch3_topic").value),
                    qos_profile_sensor_data,
                ),
            ),
        }

        self._servo_seq = 0
        self._hb_seq = 0
        self._high_gear = False
        self._front_diff = False
        self._rear_diff = False

        if self._enable_command_bridge:
            self.create_subscription(
                LLIControl, self._ctrl_request_topic, self._ctrl_request_cb, 10
            )
            self.get_logger().info(
                f"Command bridge enabled: {self._ctrl_request_topic} -> SERIAL SERVO"
            )

        self._poll_timer = self.create_timer(self._poll_period_s, self._poll_serial)
        self._hb_timer = None
        if self._hb_hz > 0.0:
            self._hb_timer = self.create_timer(1.0 / self._hb_hz, self._send_host_heartbeat)

        self.get_logger().info(
            f"Listening on {port} @ {baud} baud, publishing under /{self._topic_prefix}"
        )
        self.get_logger().info(
            "Native typed topics enabled: "
            f"imu={self._imu_topic}, imu_temp={self._imu_temp_topic}, "
            f"battery={self._battery_topic}, ads1115={self._ads1115_topic}, "
            f"ina226_a={self._ina226_a_topic}, ina226_b={self._ina226_b_topic}, "
            f"remote={self._remote_topic}"
        )
        self.get_logger().info(
            "Bridge tuning: "
            f"poll_period_s={self._poll_period_s}, max_read_chunk={self._max_read_chunk}, "
            f"max_reads_per_poll={self._max_reads_per_poll}, publish_json_topics={self._publish_json_topics}, "
            f"host_heartbeat_hz={self._hb_hz}"
        )

    def _topic(self, suffix: str) -> str:
        safe_suffix = sanitize_topic_suffix(suffix)
        return f"{self._topic_prefix}/{safe_suffix}"

    def _poll_serial(self) -> None:
        any_data = False
        for _ in range(self._max_reads_per_poll):
            try:
                waiting = max(1, int(self._serial.in_waiting))
                to_read = min(self._max_read_chunk, waiting)
                chunk = self._serial.read(to_read)
            except serial.SerialException as exc:
                now = time.monotonic()
                if now - self._last_serial_error_log >= 1.0:
                    self.get_logger().error(f"Serial read failed: {exc}")
                    self._last_serial_error_log = now
                return

            if not chunk:
                break

            any_data = True
            self._rx_buffer.extend(chunk)
            if len(chunk) < to_read and self._serial.in_waiting == 0:
                break

        if any_data:
            self._drain_frames()

    def _drain_frames(self) -> None:
        while True:
            if len(self._rx_buffer) < 3:
                return

            sync_idx = self._rx_buffer.find(bytes([CBOR_MAGIC]))
            if sync_idx < 0:
                self._rx_buffer.clear()
                return
            if sync_idx > 0:
                del self._rx_buffer[:sync_idx]
                if len(self._rx_buffer) < 3:
                    return

            frame_len = (self._rx_buffer[1] << 8) | self._rx_buffer[2]
            if frame_len == 0 or frame_len > MAX_FRAME_LEN:
                del self._rx_buffer[0]
                continue
            if len(self._rx_buffer) < 3 + frame_len:
                return

            payload = bytes(self._rx_buffer[3 : 3 + frame_len])
            del self._rx_buffer[: 3 + frame_len]

            try:
                msg = cbor2.loads(payload)
            except Exception:
                continue

            if isinstance(msg, dict):
                self._publish_telemetry(msg)

    def _publish_telemetry(self, payload: dict[str, Any]) -> None:
        normalized = self._normalize_payload(payload)
        topic = str(normalized.get("topic", "unknown"))
        if self._publish_json_topics and self._raw_pub is not None and self._unknown_pub is not None:
            json_msg = String()
            try:
                json_msg.data = json.dumps(normalized, separators=(",", ":"))
            except Exception as exc:
                self.get_logger().warning(f"Failed to serialize payload to JSON: {exc}")
                return
            self._raw_pub.publish(json_msg)
            pub = self._topic_pubs.get(topic, self._unknown_pub)
            pub.publish(json_msg)

        if topic == "lsm6dsox":
            self._publish_imu(normalized)
            self._publish_imu_temp(normalized)
        elif topic == "ads1115":
            self._publish_ads1115(normalized)
        elif topic == "bq76942":
            self._publish_battery(normalized)
        elif topic == "ina226_a":
            self._publish_ina226(normalized, "ina226_a")
        elif topic == "ina226_b":
            self._publish_ina226(normalized, "ina226_b")
        elif topic == "ina3221_a":
            self._publish_ina3221(normalized, "ina3221_a")
        elif topic == "ina3221_b":
            self._publish_ina3221(normalized, "ina3221_b")
        elif topic == "rc_command":
            lli_msg = self._rc_to_lli(normalized)
            self._remote_pub.publish(lli_msg)

    @staticmethod
    def _as_float(msg: dict[str, Any], key: str, default: float = 0.0) -> float:
        try:
            return float(msg.get(key, default))
        except (TypeError, ValueError):
            return default

    def _normalize_payload(self, payload: Any) -> dict[str, Any]:
        if not isinstance(payload, dict):
            return {"topic": "unknown", "payload": self._normalize_value(payload)}
        return {str(k): self._normalize_value(v) for k, v in payload.items()}

    def _normalize_value(self, value: Any) -> Any:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, bytes):
            return value.hex()
        if hasattr(cbor2, "CBORTag") and isinstance(value, cbor2.CBORTag):
            return self._normalize_value(value.value)
        if isinstance(value, dict):
            return {str(k): self._normalize_value(v) for k, v in value.items()}
        if isinstance(value, (list, tuple, set)):
            return [self._normalize_value(v) for v in value]
        return str(value)

    def _publish_imu(self, msg: dict[str, Any]) -> None:
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self._frame_id

        imu.orientation.w = 1.0
        imu.orientation_covariance = [0.0] * 9
        imu.angular_velocity_covariance = [0.0] * 9
        imu.linear_acceleration_covariance = [0.0] * 9

        imu.linear_acceleration.x = self._as_float(msg, "ax_mg") * MG_TO_MPS2
        imu.linear_acceleration.y = self._as_float(msg, "ay_mg") * MG_TO_MPS2
        imu.linear_acceleration.z = self._as_float(msg, "az_mg") * MG_TO_MPS2

        imu.angular_velocity.x = self._as_float(msg, "gx_mdps") * MDPS_TO_RADPS
        imu.angular_velocity.y = self._as_float(msg, "gy_mdps") * MDPS_TO_RADPS
        imu.angular_velocity.z = self._as_float(msg, "gz_mdps") * MDPS_TO_RADPS

        self._imu_pub.publish(imu)

    def _publish_imu_temp(self, msg: dict[str, Any]) -> None:
        temp = Temperature()
        temp.header.stamp = self.get_clock().now().to_msg()
        temp.header.frame_id = self._frame_id
        temp.temperature = self._as_float(msg, "temp_cdeg") / 100.0
        temp.variance = 0.0
        self._imu_temp_pub.publish(temp)

    def _publish_ads1115(self, msg: dict[str, Any]) -> None:
        adc = Float32MultiArray()
        adc.data = [
            self._as_float(msg, "ain0_mv"),
            self._as_float(msg, "ain1_mv"),
            self._as_float(msg, "ain2_mv"),
            self._as_float(msg, "ain3_mv"),
        ]
        self._ads1115_pub.publish(adc)

    def _new_power_state(self, location: str) -> BatteryState:
        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()
        batt.header.frame_id = self._frame_id
        batt.present = True
        batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        batt.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        batt.charge = float("nan")
        batt.capacity = float("nan")
        batt.design_capacity = float("nan")
        batt.percentage = float("nan")
        batt.location = location
        return batt

    def _publish_battery(self, msg: dict[str, Any]) -> None:
        batt = self._new_power_state("bq76942")
        batt.voltage = self._as_float(msg, "pack_mv") / 1000.0
        batt.current = self._as_float(msg, "pack_ma") / 1000.0
        batt.temperature = self._as_float(msg, "temp_cdeg") / 100.0
        batt.percentage = self._as_float(msg, "soc_pct") / 100.0
        batt.cell_voltage = [
            self._as_float(msg, "cell_min_mv") / 1000.0,
            self._as_float(msg, "cell_avg_mv") / 1000.0,
            self._as_float(msg, "cell_max_mv") / 1000.0,
        ]

        self._battery_pub.publish(batt)

    def _publish_ina226(self, msg: dict[str, Any], topic: str) -> None:
        batt = self._new_power_state(topic)
        batt.voltage = self._as_float(msg, "bus_mv") / 1000.0
        batt.current = self._as_float(msg, "current_ma") / 1000.0
        batt.temperature = float("nan")
        if topic == "ina226_a":
            self._ina226_a_pub.publish(batt)
        else:
            self._ina226_b_pub.publish(batt)

    def _publish_ina3221(self, msg: dict[str, Any], topic: str) -> None:
        pub_triplet = self._ina3221_pubs[topic]
        for idx in range(3):
            ch = idx + 1
            batt = self._new_power_state(f"{topic}_ch{ch}")
            batt.voltage = self._as_float(msg, f"ch{ch}_bus_mv") / 1000.0
            batt.current = self._as_float(msg, f"ch{ch}_current_ma") / 1000.0
            batt.temperature = float("nan")
            pub_triplet[idx].publish(batt)

    def _rc_to_lli(self, msg: dict[str, Any]) -> LLIControl:
        lli = LLIControl()
        lli.steering = clamp_i8(int(msg.get("steering", 0)))
        lli.velocity = clamp_i8(int(msg.get("throttle", 0)))

        high_gear = bool(msg.get("high_gear", False))
        diff_lock = bool(msg.get("diff_lock", False))

        trans_diff = 0
        trans_diff |= 0b00001000
        trans_diff |= 0b00010000
        trans_diff |= 0b00100000
        if high_gear:
            trans_diff |= 0b00000001
        if diff_lock:
            trans_diff |= 0b00000010
            trans_diff |= 0b00000100
        lli.trans_diff = trans_diff

        ctrl_flags = 0
        if bool(msg.get("override_mode", False)):
            ctrl_flags |= 0b00000100
        if not bool(msg.get("connected", True)):
            ctrl_flags |= 0b00000010
        lli.ctrl = ctrl_flags
        return lli

    def _ctrl_request_cb(self, msg: LLIControl) -> None:
        trans_diff = int(msg.trans_diff)
        if trans_diff & 0b00001000:
            self._high_gear = bool(trans_diff & 0b00000001)
        if trans_diff & 0b00010000:
            self._front_diff = bool(trans_diff & 0b00000010)
        if trans_diff & 0b00100000:
            self._rear_diff = bool(trans_diff & 0b00000100)

        line = (
            f"SERVO {self._servo_seq} "
            f"{clamp_i8(msg.steering)} {clamp_i8(msg.velocity)} "
            f"{int(self._high_gear)} {int(self._front_diff)} {int(self._rear_diff)} "
            f"{clamp_i8(msg.ctrl)} 0\n"
        )

        try:
            self._serial.write(line.encode("ascii"))
            self._servo_seq += 1
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed (SERVO): {exc}")

    def _send_host_heartbeat(self) -> None:
        line = f"HB {self._hb_seq}\n"
        try:
            self._serial.write(line.encode("ascii"))
            self._hb_seq += 1
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed (HB): {exc}")

    def destroy_node(self) -> bool:
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SveaCustomProtocolBridge | None = None
    try:
        node = SveaCustomProtocolBridge()
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
