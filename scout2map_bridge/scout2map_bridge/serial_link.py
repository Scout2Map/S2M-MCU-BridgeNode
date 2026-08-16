#!/usr/bin/env python3
#
# File   : serial_link.py
# Purpose: Serial port ownership shared by the bridge nodes. Handles opening,
#          reconnection and the reader thread; framing is left to the caller
#          because the two MCUs do not agree on it. The Pico 2 sends newline
#          delimited JSON, the STM32 sends length prefixed binary frames.
# Author : jihoonkimtech

import threading
import time

import serial

REOPEN_DELAY_S = 1.0
READ_CHUNK = 256


class SerialLink(threading.Thread):
    """Owns a serial port and pumps raw bytes into a sink callback.

    The sink runs on this thread, so it must be cheap and must not touch
    ROS publishers directly. Both bridge nodes queue the data and publish
    from a ROS timer instead.
    """

    def __init__(self, port, baudrate, sink, logger, on_open=None):
        super().__init__(daemon=True)
        self._port_name = port
        self._baudrate = baudrate
        self._sink = sink
        self._log = logger
        self._on_open = on_open       # called after each successful open
        self._ser = None
        self._stop = threading.Event()
        self._open_flag = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def is_open(self):
        return self._open_flag.is_set()

    @property
    def port_name(self):
        return self._port_name

    def stop(self):
        self._stop.set()
        self._close()

    def write(self, data: bytes) -> bool:
        """Send bytes. Returns False when the port is down.

        A failed write closes the port so the reader thread reopens it,
        rather than leaving a half dead handle that fails on every call.
        """
        ser = self._ser
        if ser is None or not self._open_flag.is_set():
            return False
        try:
            with self._write_lock:
                ser.write(data)
            return True
        except (serial.SerialException, OSError) as exc:
            self._close(f"write failed: {exc}")
            return False

    def run(self):
        while not self._stop.is_set():
            if self._ser is None:
                if not self._try_open():
                    self._stop.wait(REOPEN_DELAY_S)
                continue

            try:
                chunk = self._ser.read(READ_CHUNK)
            except (serial.SerialException, OSError) as exc:
                self._close(f"read failed: {exc}")
                continue

            if chunk:
                self._sink(chunk)

    def _try_open(self):
        try:
            self._ser = serial.Serial(
                port=self._port_name,
                baudrate=self._baudrate,
                timeout=0.1,
                write_timeout=0.5,
                exclusive=True,
            )
            # USB CDC ignores the line coding, but DTR must be asserted
            self._ser.dtr = True
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._open_flag.set()
            self._log.info(f"serial opened: {self._port_name}")
            if self._on_open is not None:
                self._on_open()
            return True
        except (serial.SerialException, OSError) as exc:
            self._ser = None
            self._open_flag.clear()
            self._log.warn(f"serial open failed ({self._port_name}): {exc}")
            return False

    def _close(self, reason=None):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._open_flag.clear()
        if reason:
            self._log.warn(f"serial closed: {reason}")


class LineFramer:
    """Newline framing on top of SerialLink, for the JSON speaking MCU."""

    def __init__(self, max_line=4096):
        self._buf = bytearray()
        self._max_line = max_line
        self.overflows = 0

    def feed(self, data: bytes):
        self._buf.extend(data)
        # A frame with no terminator is unusable, so reset rather than grow
        if len(self._buf) > self._max_line and b"\n" not in self._buf:
            self._buf.clear()
            self.overflows += 1
            return []

        out = []
        while b"\n" in self._buf:
            raw, _, rest = self._buf.partition(b"\n")
            self._buf = bytearray(rest)
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                out.append((time.monotonic(), line))
        return out
