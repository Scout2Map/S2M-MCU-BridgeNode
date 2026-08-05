#!/usr/bin/env python3
#
# File   : fake_sensor_node.py
# Purpose: Publish synthetic sensor data on exactly the same topics and types
#          as pico_bridge, so downstream nodes can be developed and tested
#          without the UGV hardware attached.
# Author : jihoonkimtech
#
# The scenario parameter can be changed while running:
#   ros2 param set /fake_sensors scenario gas_leak
# This makes it easy to watch an event engine react to a rising value.

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Illuminance, RelativeHumidity, Temperature

from scout2map_msgs.msg import AirQuality, BridgeStatus, EnvSnapshot, Particulate

# Scenario names accepted by the "scenario" parameter
SCENARIOS = (
    "normal",           # quiet indoor baseline
    "gas_leak",         # TVOC and eCO2 ramp up over 30s
    "high_temp",        # temperature ramps toward 60C
    "low_light",        # illuminance drops to near darkness
    "dust_storm",       # particulate matter ramps up
    "warmup",           # ENS160 stays in warm-up, values must be ignored
    "sensor_dropout",   # ENS160 stops publishing, its cache goes stale
    "link_loss",        # nothing publishes, link_ok goes false
)

# Baseline values for a calm indoor room
BASE_TEMP_C = 24.0
BASE_HUM_PCT = 45.0
BASE_LUX = 320.0
BASE_ECO2 = 450
BASE_TVOC = 60
BASE_PM1 = 4
BASE_PM25 = 7
BASE_PM10 = 9


def _ramp(elapsed, duration, start, end):
    """Linear ramp from start to end over duration, then hold at end."""
    if duration <= 0.0:
        return end
    k = min(1.0, elapsed / duration)
    return start + (end - start) * k


class FakeSensors(Node):
    """Synthetic stand-in for pico_bridge. Publishes the same topic contract."""

    def __init__(self):
        super().__init__("fake_sensors")

        self.declare_parameter("scenario", "normal")
        self.declare_parameter("noise", 1.0)          # 0.0 disables jitter
        self.declare_parameter("ramp_seconds", 30.0)  # how fast a scenario develops
        self.declare_parameter("frame_id", "sensor_fusion")

        self._scenario = self.get_parameter("scenario").value
        self._noise = float(self.get_parameter("noise").value)
        self._ramp_s = float(self.get_parameter("ramp_seconds").value)
        self._frame_id = self.get_parameter("frame_id").value

        if self._scenario not in SCENARIOS:
            self.get_logger().warn(
                f"unknown scenario '{self._scenario}', falling back to normal")
            self._scenario = "normal"

        # Allow switching scenarios at runtime without restarting the node
        self.add_on_set_parameters_callback(self._on_param_change)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

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
            BridgeStatus, "bridge/status", status_qos)

        # Same latest-value cache shape as the real bridge
        self._cache = {"aht21": None, "bh1750": None,
                       "ens160": None, "pms7003": None}

        self._t0 = time.monotonic()          # scenario start
        self._boot = time.monotonic()        # node start
        self._lines = 0

        # Match the real sensor periods so staleness behaves realistically
        self.create_timer(1.0, self._tick_aht21)
        self.create_timer(0.2, self._tick_bh1750)
        self.create_timer(1.0, self._tick_ens160)
        self.create_timer(1.0, self._tick_pms7003)
        self.create_timer(0.2, self._tick_snapshot)
        self.create_timer(1.0, self._tick_status)

        self.get_logger().info(
            f"fake_sensors up: scenario={self._scenario} "
            f"(change it with: ros2 param set /fake_sensors scenario <name>)")
        self.get_logger().info(f"available scenarios: {', '.join(SCENARIOS)}")

    # ------------------------------------------------------------------
    def _on_param_change(self, params):
        for p in params:
            if p.name == "scenario":
                if p.type_ != Parameter.Type.STRING or p.value not in SCENARIOS:
                    return SetParametersResult(
                        successful=False,
                        reason=f"scenario must be one of: {', '.join(SCENARIOS)}")
                self._scenario = p.value
                self._t0 = time.monotonic()   # restart the ramp
                self.get_logger().info(f"scenario -> {self._scenario}")
            elif p.name == "noise":
                self._noise = float(p.value)
            elif p.name == "ramp_seconds":
                self._ramp_s = float(p.value)
        return SetParametersResult(successful=True)

    def _elapsed(self):
        return time.monotonic() - self._t0

    def _jitter(self, scale):
        if self._noise <= 0.0:
            return 0.0
        return random.uniform(-scale, scale) * self._noise

    def _muted(self, sensor):
        """True when the scenario says this sensor should stop publishing."""
        if self._scenario == "link_loss":
            return True
        if self._scenario == "sensor_dropout" and sensor == "ens160":
            return True
        return False

    def _stamp(self):
        return self.get_clock().now().to_msg()

    # ------------------------------------------------------------------
    # Value models
    # ------------------------------------------------------------------
    def _temp_c(self):
        if self._scenario == "high_temp":
            base = _ramp(self._elapsed(), self._ramp_s, BASE_TEMP_C, 62.0)
        else:
            base = BASE_TEMP_C
        # Slow breathing drift so plots do not look like a flat line
        drift = 0.4 * math.sin(self._elapsed() / 12.0)
        return base + drift + self._jitter(0.15)

    def _hum_pct(self):
        value = BASE_HUM_PCT + 2.0 * math.sin(self._elapsed() / 20.0)
        return max(0.0, min(100.0, value + self._jitter(0.4)))

    def _lux(self):
        if self._scenario == "low_light":
            base = _ramp(self._elapsed(), self._ramp_s, BASE_LUX, 8.0)
        else:
            base = BASE_LUX
        return max(1.0, base + self._jitter(12.0))

    def _gas(self):
        if self._scenario == "gas_leak":
            eco2 = _ramp(self._elapsed(), self._ramp_s, BASE_ECO2, 3200.0)
            tvoc = _ramp(self._elapsed(), self._ramp_s, BASE_TVOC, 4500.0)
        else:
            eco2, tvoc = float(BASE_ECO2), float(BASE_TVOC)
        eco2 = max(400.0, eco2 + self._jitter(15.0))
        tvoc = max(0.0, tvoc + self._jitter(8.0))

        # UBA index roughly follows the TVOC level
        if tvoc < 300:
            aqi = 1
        elif tvoc < 1000:
            aqi = 2
        elif tvoc < 3000:
            aqi = 3
        elif tvoc < 10000:
            aqi = 4
        else:
            aqi = 5

        # The real ENS160 reports warm-up for a while after power-up
        if self._scenario == "warmup":
            validity = 1
        elif (time.monotonic() - self._boot) < 10.0:
            validity = 1
        else:
            validity = 0
        return int(eco2), int(tvoc), aqi, validity

    def _pm(self):
        if self._scenario == "dust_storm":
            pm25 = _ramp(self._elapsed(), self._ramp_s, BASE_PM25, 180.0)
        else:
            pm25 = float(BASE_PM25)
        pm25 = max(0.0, pm25 + self._jitter(1.5))
        # Keep the three sizes in a plausible relationship
        pm1 = pm25 * 0.6
        pm10 = pm25 * 1.3
        return int(pm1), int(pm25), int(pm10)

    # ------------------------------------------------------------------
    # Per-sensor timers
    # ------------------------------------------------------------------
    def _tick_aht21(self):
        if self._muted("aht21"):
            return
        temp, hum = self._temp_c(), self._hum_pct()
        self._cache["aht21"] = ({"temp": temp, "hum": hum}, time.monotonic())
        self._lines += 1

        t = Temperature()
        t.header.stamp = self._stamp()
        t.header.frame_id = self._frame_id
        t.temperature = temp
        self._pub_temp.publish(t)

        h = RelativeHumidity()
        h.header.stamp = self._stamp()
        h.header.frame_id = self._frame_id
        h.relative_humidity = hum / 100.0   # standard message wants a ratio
        self._pub_hum.publish(h)

    def _tick_bh1750(self):
        if self._muted("bh1750"):
            return
        lux = self._lux()
        self._cache["bh1750"] = ({"lux": lux}, time.monotonic())
        self._lines += 1

        m = Illuminance()
        m.header.stamp = self._stamp()
        m.header.frame_id = self._frame_id
        m.illuminance = lux
        self._pub_lux.publish(m)

    def _tick_ens160(self):
        if self._muted("ens160"):
            return
        eco2, tvoc, aqi, validity = self._gas()
        self._cache["ens160"] = (
            {"eco2": eco2, "tvoc": tvoc, "aqi": aqi, "valid": validity},
            time.monotonic())
        self._lines += 1

        m = AirQuality()
        m.header.stamp = self._stamp()
        m.header.frame_id = self._frame_id
        m.eco2_ppm = eco2
        m.tvoc_ppb = min(65535, tvoc)
        m.aqi = aqi
        m.validity = validity
        m.operational = (validity == 0)
        self._pub_aq.publish(m)

    def _tick_pms7003(self):
        if self._muted("pms7003"):
            return
        pm1, pm25, pm10 = self._pm()
        self._cache["pms7003"] = (
            {"pm1": pm1, "pm25": pm25, "pm10": pm10}, time.monotonic())
        self._lines += 1

        m = Particulate()
        m.header.stamp = self._stamp()
        m.header.frame_id = self._frame_id
        m.pm1_0_ug_m3 = pm1
        m.pm2_5_ug_m3 = pm25
        m.pm10_ug_m3 = pm10
        self._pub_pm.publish(m)

    # ------------------------------------------------------------------
    # Aggregate timers
    # ------------------------------------------------------------------
    def _age_of(self, key, now):
        entry = self._cache.get(key)
        if entry is None:
            return None, None
        payload, mono = entry
        return payload, now - mono

    def _tick_snapshot(self):
        now = time.monotonic()
        msg = EnvSnapshot()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._frame_id

        aht, age = self._age_of("aht21", now)
        if aht is not None:
            msg.temperature_c = aht["temp"]
            msg.humidity_pct = aht["hum"]
            msg.ambient_age_s = float(age)
            msg.ambient_valid = age <= 3.0

        lux, age = self._age_of("bh1750", now)
        if lux is not None:
            msg.illuminance_lux = lux["lux"]
            msg.illuminance_age_s = float(age)
            msg.illuminance_valid = age <= 1.0

        ens, age = self._age_of("ens160", now)
        if ens is not None:
            msg.eco2_ppm = ens["eco2"]
            msg.tvoc_ppb = min(65535, ens["tvoc"])
            msg.aqi = ens["aqi"]
            msg.ens160_validity = ens["valid"]
            msg.air_quality_age_s = float(age)
            msg.air_quality_valid = (age <= 3.0 and ens["valid"] == 0)

        pms, age = self._age_of("pms7003", now)
        if pms is not None:
            msg.pm1_0_ug_m3 = pms["pm1"]
            msg.pm2_5_ug_m3 = pms["pm25"]
            msg.pm10_ug_m3 = pms["pm10"]
            msg.particulate_age_s = float(age)
            msg.particulate_valid = age <= 5.0

        msg.link_ok = (self._scenario != "link_loss")
        msg.mcu_uptime_ms = int((now - self._boot) * 1000.0)
        self._pub_snap.publish(msg)

    def _tick_status(self):
        now = time.monotonic()
        link_up = (self._scenario != "link_loss")
        msg = BridgeStatus()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._frame_id
        msg.port = "SIMULATED"          # obvious marker that this is not real
        msg.port_open = link_up
        msg.link_ok = link_up
        msg.last_line_age_s = 0.0 if link_up else 99.0
        msg.mcu_uptime_ms = int((now - self._boot) * 1000.0)
        msg.mcu_reboot_count = 0
        msg.lines_received = self._lines
        msg.parse_errors = 0
        msg.unknown_src = 0
        msg.aht21_present = True
        msg.ens160_present = True
        msg.bh1750_present = True
        msg.pms7003_seen = True
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensors()
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
