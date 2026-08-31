#!/usr/bin/env python3
#
# File   : gpio_event_node.py
# Purpose: Drive Raspberry Pi 5 GPIO output pins from Event Engine's /events
#          stream, so a hazard event can flip a relay, buzzer, or indicator
#          LED without any other node needing to know GPIO exists.
# Author : jihoonkimtech
#
# --- Mapping model ---
# An operator (normally through the Web-Monitoring settings panel, relayed
# by S2M-CommRelay) registers any number of (event_type, pin, mode) rows.
# event_type has no fixed limit on how many rows it may own -- one event can
# drive several pins -- and this node places no limit on it either; that is
# a deliberate choice, not an oversight (see project decision log).
#
# mode picks which physical level means "triggered":
#   trigger_high: idle LOW,  event raised -> pin driven HIGH
#   trigger_low : idle HIGH, event raised -> pin driven LOW
# This covers both active-high loads (an LED, a driver board that switches
# on 3.3V) and active-low ones (many relay/opto boards trigger when pulled
# to ground). gpiozero's OutputDevice(active_high=...) does the polarity
# translation, so the rest of this file only ever calls .on()/.off() and
# never touches raw HIGH/LOW.
#
# --- Shared pins ---
# Two different mappings are allowed to name the same physical pin. This
# node does not arbitrate between them -- whichever event transitions last
# decides the pin's level. That is adequate for the common case (one alarm
# output that several hazard types should light up) and deliberately not
# more than that; an operator who wants independent, non-conflicting
# outputs should give each mapping its own pin.
#
# --- One-shot event types ---
# Event Engine's raise/clear latch (see event_engine_node.py's _active dict)
# only exists for continuous-state hazards. VISION_DETECTION and every
# PREDICTED_* type publish 'raised' only -- there is no matching 'cleared'
# (see event_engine_node.py's own comment on prediction events). A mapping
# on one of those types will latch its pin active on the first occurrence
# and hold it there; nothing in this node clears it again short of a
# restart or another mapping/event driving the same pin back down. This is
# a known, accepted limitation rather than a bug -- keep it in mind before
# wiring a one-shot type straight to something that must not stay on.
#
# --- Hardware backend ---
# Raspberry Pi 5 moved GPIO off the old /dev/gpiomem character device that
# RPi.GPIO expects, so this node uses gpiozero, which auto-selects a
# working backend (lgpio on Pi 5, falling back to RPi.GPIO/pigpio on older
# boards) instead of picking one library by hand. Set simulate:=true to
# force gpiozero's MockFactory and develop without any board attached, the
# same idea as fake_sensors.launch.py in this same package.

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger

from .gpio_mapping_db import VALID_MODES, GpioMappingDB

try:
    from gpiozero import Device, OutputDevice
    from gpiozero.pins.mock import MockFactory
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False


class GpioEventNode(Node):
    """Maps Event Engine hazard events onto Raspberry Pi GPIO outputs."""

    def __init__(self):
        super().__init__("gpio_events")

        # ---------------- Parameters ----------------
        self.declare_parameter("events_topic", "/events")
        self.declare_parameter("config_topic", "/gpio_events/config_set")
        self.declare_parameter("get_all_service", "/gpio_events/get_all")
        # '' resolves to GpioMappingDB's own default (~/.scout2map/gpio_events.db)
        self.declare_parameter("db_path", "")
        # True runs against gpiozero's MockFactory instead of real hardware --
        # same purpose as fake_sensors.launch.py, for developing this node
        # (or the web settings panel that drives it) off the robot.
        self.declare_parameter("simulate", False)

        gp = self.get_parameter
        self._events_topic = gp("events_topic").value
        self._config_topic = gp("config_topic").value
        self._get_all_service = gp("get_all_service").value
        db_path = gp("db_path").value or None
        self._simulate = bool(gp("simulate").value)

        if not GPIOZERO_AVAILABLE:
            self.get_logger().error(
                "gpiozero is not installed -- pin_factory-related calls "
                "below will raise. Install it (pip install gpiozero, plus "
                "lgpio on a Raspberry Pi 5) or run with simulate:=true only "
                "after installing gpiozero itself (MockFactory still lives "
                "in the gpiozero package)."
            )
        elif self._simulate:
            Device.pin_factory = MockFactory()
            self.get_logger().warn(
                "gpio_events running in SIMULATE mode (gpiozero MockFactory) "
                "-- no physical pin will actually change state."
            )

        self._db = GpioMappingDB(db_path)
        self.get_logger().info(f"gpio mapping db: {self._db.db_path}")

        # pin -> gpiozero.OutputDevice, created lazily and reused across
        # every mapping that names that pin (see module docstring on shared
        # pins). Keyed on the physical pin number, not the mapping id.
        self._devices = {}

        # Every known pin starts idle so a restart never leaves an output
        # in whatever level it happened to power up in.
        for mapping in self._db.all():
            self._get_device(mapping["pin"], mapping["mode"])

        # ---------------- QoS ----------------
        # Matches event_engine_node's own /events QoS (reliable, depth 10) --
        # this node cares about every transition, not just the latest one.
        events_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            String,
            self._events_topic,
            self._on_event,
            events_qos,
        )

        # Fire-and-forget config channel, same shape as event_engine's
        # /threshold/set: a JSON command in, no reply out. Kept as a plain
        # topic rather than a custom service/message type so this stays a
        # single-file addition to an existing package (2026-08-31).
        self.create_subscription(
            String,
            self._config_topic,
            self._on_config,
            10,
        )

        self.create_service(
            Trigger,
            self._get_all_service,
            self._get_all_callback,
        )

        self.get_logger().info(
            f"gpio_events up: events={self._events_topic} "
            f"config={self._config_topic} get_all={self._get_all_service} "
            f"mappings={len(self._db.all())} "
            f"backend={'simulate' if self._simulate else 'gpiozero-auto'}"
        )

    # ---------------- GPIO device management ----------------

    def _get_device(self, pin, mode):
        """Returns the shared OutputDevice for this pin, creating it (idle)
        on first use. mode only matters the first time a pin is seen --
        polarity is a property of the physical pin here, so if two mappings
        on the same pin disagree on mode the first one registered wins;
        logged so it is not a silent surprise."""
        pin = int(pin)
        device = self._devices.get(pin)

        if device is not None:
            device_mode = getattr(device, "_gpio_events_mode", None)
            if device_mode is not None and device_mode != mode:
                self.get_logger().warn(
                    f"pin {pin} already registered as {device_mode}; "
                    f"ignoring conflicting mode {mode} from this mapping"
                )
            return device

        if not GPIOZERO_AVAILABLE:
            raise RuntimeError("gpiozero is not installed")

        device = OutputDevice(
            pin,
            active_high=(mode == "trigger_high"),
            initial_value=False,
        )
        device._gpio_events_mode = mode
        self._devices[pin] = device
        self.get_logger().info(f"pin {pin} ready, mode={mode}, idle")
        return device

    def _release_pin_if_unclaimed(self, pin):
        """Drops the OutputDevice for a pin once no mapping references it
        any more, so a deleted mapping does not leave a relay energised
        forever with nothing left to turn it back off."""
        if self._db.pin_still_claimed(pin):
            return
        device = self._devices.pop(pin, None)
        if device is not None:
            device.off()
            device.close()
            self.get_logger().info(f"pin {pin} released (no mapping left)")

    # ---------------- /events handling ----------------

    def _on_event(self, msg):
        try:
            payload = json.loads(msg.data)
            event_type = payload["type"]
            state = payload["state"]
        except (json.JSONDecodeError, KeyError, TypeError) as exc:
            self.get_logger().warning(f"unreadable event payload: {exc}")
            return

        mappings = self._db.for_event_type(event_type)
        if not mappings:
            return

        # 'raised' activates, 'cleared' (and anything else event_engine
        # might one day add) idles -- deliberately not an if/elif so a
        # future third state degrades to "safe" rather than "stuck on".
        activate = state == "raised"

        for mapping in mappings:
            try:
                device = self._get_device(mapping["pin"], mapping["mode"])
                if activate:
                    device.on()
                else:
                    device.off()
            except Exception as exc:  # keep one bad pin from blocking others
                self.get_logger().error(
                    f"failed to drive pin {mapping['pin']} for "
                    f"{event_type}/{state}: {exc}"
                )

        self.get_logger().info(
            f"{event_type} {state} -> "
            f"{[m['pin'] for m in mappings]} "
            f"{'ON' if activate else 'idle'}"
        )

    # ---------------- config_set handling ----------------

    def _on_config(self, msg):
        try:
            cmd = json.loads(msg.data)
            action = cmd["action"]

            if action == "add":
                self._handle_add(cmd)
            elif action == "remove":
                self._handle_remove(cmd)
            else:
                self.get_logger().warning(f"unknown gpio config action: {action}")

        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid gpio config message: {exc}")

    def _handle_add(self, cmd):
        event_type = str(cmd["event_type"])
        pin = int(cmd["pin"])
        mode = str(cmd["mode"])
        label = str(cmd.get("label", ""))

        if mode not in VALID_MODES:
            self.get_logger().warning(f"add rejected: unknown mode {mode!r}")
            return

        mapping_id = self._db.add(event_type, pin, mode, label)
        self._get_device(pin, mode)  # create it now, idle, not on first event
        self.get_logger().info(
            f"gpio mapping added: id={mapping_id} {event_type} -> "
            f"pin {pin} ({mode}) {label!r}"
        )

    def _handle_remove(self, cmd):
        mapping_id = int(cmd["id"])
        pin = self._db.remove(mapping_id)
        if pin is None:
            self.get_logger().warning(f"remove rejected: no mapping id={mapping_id}")
            return
        self._release_pin_if_unclaimed(pin)
        self.get_logger().info(f"gpio mapping removed: id={mapping_id}")

    # ---------------- get_all service ----------------

    def _get_all_callback(self, _request, response):
        response.success = True
        response.message = json.dumps(self._db.all())
        return response

    def destroy_node(self):
        for device in self._devices.values():
            try:
                device.close()
            except Exception:
                pass
        self._db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpioEventNode()
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
