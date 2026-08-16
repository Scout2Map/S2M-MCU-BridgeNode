#!/usr/bin/env python3
#
# File   : sensor_bridge_node.py
# Purpose: Bridge the Scout2Map sensor-fusion MCU (Raspberry Pi Pico 2) to ROS 2.
#          Reads JSON-per-line frames over USB CDC, republishes them as typed
#          topics, and keeps a latest-value cache that is published at a fixed
#          rate so downstream nodes never have to join asynchronous streams.
# Author : jihoonkimtech
#
# MCU line format (one JSON object per line, "src" selects the payload):
#   {"src":"sys","event":"boot","aht21":true,"ens160":true,"bh1750":true}
#   {"src":"bh1750","lux":123.4}
#   {"src":"aht21","temp":25.31,"hum":41.02}
#   {"src":"ens160","eco2":412,"tvoc":37,"aqi":1,"valid":0}
#   {"src":"pms7003","pm1":3,"pm25":5,"pm10":6}
#   {"src":"sys","uptime_ms":123456}

import json
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import Illuminance, RelativeHumidity, Temperature
from std_msgs.msg import String

from scout2map_msgs.msg import AirQuality, EnvSnapshot, Particulate, SensorStatus

from .serial_link import LineFramer, SerialLink

# Drop the oldest lines rather than growing without bound if ROS stalls.
RX_QUEUE_MAX = 512

class SensorBridge(Node):
    """Serial-to-topic bridge for the Pico 2 sensor fusion MCU."""

    def __init__(self):
        super().__init__("sensor_bridge")

        # ---------------- Parameters ----------------
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "sensor_fusion")
        self.declare_parameter("snapshot_rate_hz", 5.0)
        self.declare_parameter("status_rate_hz", 1.0)
        self.declare_parameter("publish_raw_json", False)
        # Staleness limits, one per sensor period class
        self.declare_parameter("stale_ambient_s", 3.0)      # AHT21 @1Hz
        self.declare_parameter("stale_illuminance_s", 1.0)  # BH1750 @5Hz
        self.declare_parameter("stale_air_quality_s", 3.0)  # ENS160 @1Hz
        self.declare_parameter("stale_particulate_s", 5.0)  # PMS7003 @1Hz
        self.declare_parameter("link_timeout_s", 3.0)

        gp = self.get_parameter
        self._port = gp("port").value
        self._baudrate = int(gp("baudrate").value)
        self._frame_id = gp("frame_id").value
        self._publish_raw = bool(gp("publish_raw_json").value)
        self._stale_ambient = float(gp("stale_ambient_s").value)
        self._stale_lux = float(gp("stale_illuminance_s").value)
        self._stale_aq = float(gp("stale_air_quality_s").value)
        self._stale_pm = float(gp("stale_particulate_s").value)
        self._link_timeout = float(gp("link_timeout_s").value)

        # ---------------- QoS ----------------
        # Sensor streams are low rate, so reliable delivery costs nothing and
        # spares the event engine from silently missing a threshold crossing.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Latched status so a late subscriber immediately sees the link state
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------- Publishers ----------------
        self._pub_temp = self.create_publisher(
            Temperature, "sensors/temperature", sensor_qos)
        self._pub_hum = self.create_publisher(
            RelativeHumidity, "sensors/humidity", sensor_qos)
        self._pub_lux = self.create_publisher(
            Illuminance, "sensors/illuminance", sensor_qos)
        self._pub_aq = self.create_publisher(
            AirQuality, "sensors/air_quality", sensor_qos)
        self._pub_pm = self.create_publisher(
            Particulate, "sensors/particulate", sensor_qos)
        self._pub_snap = self.create_publisher(
            EnvSnapshot, "sensors/env_snapshot", sensor_qos)
        self._pub_status = self.create_publisher(
            SensorStatus, "sensors/status", status_qos)
        self._pub_rawjson = None
        if self._publish_raw:
            self._pub_rawjson = self.create_publisher(
                String, "sensors/raw_json", sensor_qos)

        # ---------------- Latest-value cache ----------------
        # Each entry is (payload_dict, monotonic_stamp) or None when never seen.
        self._cache = {
            "aht21": None,
            "bh1750": None,
            "ens160": None,
            "pms7003": None,
        }

        # ---------------- Counters and link state ----------------
        self._lines_rx = 0
        self._parse_errors = 0
        self._unknown_src = 0
        self._last_line_mono = 0.0
        self._mcu_uptime_ms = 0
        self._mcu_reboots = 0
        self._present = {"aht21": False, "ens160": False, "bh1750": False}
        self._pms_seen = False

        # ---------------- Serial reader ----------------
        self._rx = deque(maxlen=RX_QUEUE_MAX)
        self._framer = LineFramer()
        self._link = SerialLink(
            self._port, self._baudrate, self._on_serial_bytes, self.get_logger())
        self._link.start()

        # ---------------- Timers ----------------
        # Drain fast so per-sensor topics stay close to the MCU timing
        self.create_timer(0.01, self._drain_rx)
        snap_hz = max(0.1, float(gp("snapshot_rate_hz").value))
        self.create_timer(1.0 / snap_hz, self._publish_snapshot)
        status_hz = max(0.1, float(gp("status_rate_hz").value))
        self.create_timer(1.0 / status_hz, self._publish_status)

        self.get_logger().info(
            f"sensor_bridge up: port={self._port} frame_id={self._frame_id} "
            f"snapshot={snap_hz:.1f}Hz")

    # ------------------------------------------------------------------
    # RX path
    # ------------------------------------------------------------------
    def _on_serial_bytes(self, data: bytes):
        """Runs on the reader thread. Split into lines here, publish on the timer."""
        self._rx.extend(self._framer.feed(data))

    def _drain_rx(self):
        # Bound the work per callback so one burst cannot stall the executor
        for _ in range(RX_QUEUE_MAX):
            try:
                mono, line = self._rx.popleft()
            except IndexError:
                return
            self._handle_line(mono, line)

    def _handle_line(self, mono, line):
        self._lines_rx += 1
        self._last_line_mono = mono

        if self._pub_rawjson is not None:
            self._pub_rawjson.publish(String(data=line))

        try:
            obj = json.loads(line)
        except (json.JSONDecodeError, ValueError):
            self._parse_errors += 1
            # Log sparsely: a noisy line should not flood the console
            if self._parse_errors % 50 == 1:
                self.get_logger().warn(f"bad JSON line: {line[:120]}")
            return

        if not isinstance(obj, dict):
            self._parse_errors += 1
            return

        src = obj.get("src")
        stamp = self.get_clock().now().to_msg()

        if src == "bh1750":
            self._on_bh1750(obj, mono, stamp)
        elif src == "aht21":
            self._on_aht21(obj, mono, stamp)
        elif src == "ens160":
            self._on_ens160(obj, mono, stamp)
        elif src == "pms7003":
            self._on_pms7003(obj, mono, stamp)
        elif src == "sys":
            self._on_sys(obj)
        else:
            self._unknown_src += 1

    # ------------------------------------------------------------------
    # Per-sensor handlers
    # ------------------------------------------------------------------
    def _on_bh1750(self, obj, mono, stamp):
        lux = _as_float(obj.get("lux"))
        if lux is None:
            self._parse_errors += 1
            return
        self._cache["bh1750"] = ({"lux": lux}, mono)

        msg = Illuminance()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.illuminance = lux
        msg.variance = 0.0          # unknown, per sensor_msgs convention
        self._pub_lux.publish(msg)

    def _on_aht21(self, obj, mono, stamp):
        temp = _as_float(obj.get("temp"))
        hum = _as_float(obj.get("hum"))
        if temp is None or hum is None:
            self._parse_errors += 1
            return
        self._cache["aht21"] = ({"temp": temp, "hum": hum}, mono)

        tmsg = Temperature()
        tmsg.header.stamp = stamp
        tmsg.header.frame_id = self._frame_id
        tmsg.temperature = temp     # degrees Celsius
        tmsg.variance = 0.0
        self._pub_temp.publish(tmsg)

        hmsg = RelativeHumidity()
        hmsg.header.stamp = stamp
        hmsg.header.frame_id = self._frame_id
        # sensor_msgs expects a 0.0 - 1.0 ratio, the MCU sends percent
        hmsg.relative_humidity = hum / 100.0
        hmsg.variance = 0.0
        self._pub_hum.publish(hmsg)

    def _on_ens160(self, obj, mono, stamp):
        eco2 = _as_int(obj.get("eco2"))
        tvoc = _as_int(obj.get("tvoc"))
        aqi = _as_int(obj.get("aqi"))
        valid = _as_int(obj.get("valid"))
        if None in (eco2, tvoc, aqi, valid):
            self._parse_errors += 1
            return
        self._cache["ens160"] = (
            {"eco2": eco2, "tvoc": tvoc, "aqi": aqi, "valid": valid}, mono)

        msg = AirQuality()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.eco2_ppm = _clamp_u16(eco2)
        msg.tvoc_ppb = _clamp_u16(tvoc)
        msg.aqi = _clamp_u8(aqi)
        msg.validity = _clamp_u8(valid)
        # Values are still published while warming up, flagged for filtering
        msg.operational = (valid == 0)
        self._pub_aq.publish(msg)

    def _on_pms7003(self, obj, mono, stamp):
        pm1 = _as_int(obj.get("pm1"))
        pm25 = _as_int(obj.get("pm25"))
        pm10 = _as_int(obj.get("pm10"))
        if None in (pm1, pm25, pm10):
            self._parse_errors += 1
            return
        self._pms_seen = True
        self._cache["pms7003"] = (
            {"pm1": pm1, "pm25": pm25, "pm10": pm10}, mono)

        msg = Particulate()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.pm1_0_ug_m3 = _clamp_u16(pm1)
        msg.pm2_5_ug_m3 = _clamp_u16(pm25)
        msg.pm10_ug_m3 = _clamp_u16(pm10)
        self._pub_pm.publish(msg)

    def _on_sys(self, obj):
        if obj.get("event") == "boot":
            for key in self._present:
                self._present[key] = bool(obj.get(key, False))
            self.get_logger().info(
                "MCU boot: "
                + ", ".join(f"{k}={'ok' if v else 'FAIL'}"
                            for k, v in self._present.items()))
            missing = [k for k, v in self._present.items() if not v]
            if missing:
                self.get_logger().error(
                    f"sensor init failed on MCU: {', '.join(missing)} "
                    "(check wiring and I2C address)")
            return

        uptime = _as_int(obj.get("uptime_ms"))
        if uptime is None:
            return
        # A backwards jump means the MCU restarted under us
        if uptime < self._mcu_uptime_ms:
            self._mcu_reboots += 1
            self._pms_seen = False
            self.get_logger().warn(
                f"MCU reboot detected (uptime {self._mcu_uptime_ms} -> {uptime})")
        self._mcu_uptime_ms = uptime

    # ------------------------------------------------------------------
    # Periodic publishers
    # ------------------------------------------------------------------
    def _age_of(self, key, now):
        entry = self._cache.get(key)
        if entry is None:
            return None, None
        payload, mono = entry
        return payload, now - mono

    def _publish_snapshot(self):
        now = time.monotonic()
        msg = EnvSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        aht, age = self._age_of("aht21", now)
        if aht is not None:
            msg.temperature_c = aht["temp"]
            msg.humidity_pct = aht["hum"]
            msg.ambient_age_s = float(age)
            msg.ambient_valid = age <= self._stale_ambient

        lux, age = self._age_of("bh1750", now)
        if lux is not None:
            msg.illuminance_lux = lux["lux"]
            msg.illuminance_age_s = float(age)
            msg.illuminance_valid = age <= self._stale_lux

        ens, age = self._age_of("ens160", now)
        if ens is not None:
            msg.eco2_ppm = _clamp_u16(ens["eco2"])
            msg.tvoc_ppb = _clamp_u16(ens["tvoc"])
            msg.aqi = _clamp_u8(ens["aqi"])
            msg.ens160_validity = _clamp_u8(ens["valid"])
            msg.air_quality_age_s = float(age)
            # Fresh is not enough, the ENS160 must also be out of warm-up
            msg.air_quality_valid = (age <= self._stale_aq and ens["valid"] == 0)

        pms, age = self._age_of("pms7003", now)
        if pms is not None:
            msg.pm1_0_ug_m3 = _clamp_u16(pms["pm1"])
            msg.pm2_5_ug_m3 = _clamp_u16(pms["pm25"])
            msg.pm10_ug_m3 = _clamp_u16(pms["pm10"])
            msg.particulate_age_s = float(age)
            msg.particulate_valid = age <= self._stale_pm

        msg.link_ok = self._link_ok(now)
        msg.mcu_uptime_ms = _clamp_u32(self._mcu_uptime_ms)
        self._pub_snap.publish(msg)

    def _publish_status(self):
        now = time.monotonic()
        msg = SensorStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.port = self._port
        msg.port_open = self._link.is_open
        msg.link_ok = self._link_ok(now)
        msg.last_line_age_s = (
            float(now - self._last_line_mono) if self._last_line_mono else -1.0)
        msg.mcu_uptime_ms = _clamp_u32(self._mcu_uptime_ms)
        msg.mcu_reboot_count = _clamp_u32(self._mcu_reboots)
        msg.lines_received = self._lines_rx
        msg.parse_errors = self._parse_errors
        msg.unknown_src = self._unknown_src
        msg.framing_overflows = self._framer.overflows
        msg.aht21_present = self._present["aht21"]
        msg.ens160_present = self._present["ens160"]
        msg.bh1750_present = self._present["bh1750"]
        msg.pms7003_seen = self._pms_seen
        self._pub_status.publish(msg)

    def _link_ok(self, now):
        if not self._link.is_open or self._last_line_mono == 0.0:
            return False
        return (now - self._last_line_mono) <= self._link_timeout

    def destroy_node(self):
        self._link.stop()
        return super().destroy_node()


# ----------------------------------------------------------------------
# Small helpers: the MCU is trusted but a corrupted line must not crash us
# ----------------------------------------------------------------------
def _as_float(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    # NaN and inf would poison downstream threshold checks
    if out != out or out in (float("inf"), float("-inf")):
        return None
    return out


def _as_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _clamp_u8(value):
    return max(0, min(255, int(value)))


def _clamp_u16(value):
    return max(0, min(65535, int(value)))


def _clamp_u32(value):
    return max(0, min(4294967295, int(value)))


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
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
