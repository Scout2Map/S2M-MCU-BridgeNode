#!/usr/bin/env python3
#
# File   : test_serial_link.py
# Purpose: Host side tests for SerialLink's shutdown path. No board and no
#          pyserial needed; a stand-in module reproduces the one behaviour
#          that matters here, which is that close() drops the file
#          descriptor under a read that is already in flight.
# Author : jihoonkimtech

import sys
import threading
import time
import traceback
import types

import pytest


def _install_fake_serial():
    """Stand-in for pyserial that reproduces the shutdown race.

    serialposix.read() does os.read(self.fd, ...), so once close() has set
    fd to None an in-flight read raises TypeError rather than
    SerialException. That is what escaped SerialLink.run()'s handler and
    printed a thread traceback on every clean shutdown.
    """
    mod = types.ModuleType("serial")

    class SerialException(Exception):
        pass

    class Serial:
        def __init__(self, **kwargs):
            self.fd = 1
            self.dtr = False
            self._closed = False

        def read(self, n):
            # Mirrors the 0.1s read timeout the real port is opened with
            for _ in range(10):
                if self._closed:
                    raise TypeError(
                        "'NoneType' object cannot be interpreted as an integer")
                time.sleep(0.01)
            return b""

        def close(self):
            self._closed = True
            self.fd = None

        def reset_input_buffer(self):
            pass

        def reset_output_buffer(self):
            pass

        def write(self, data):
            pass

    mod.Serial = Serial
    mod.SerialException = SerialException
    sys.modules["serial"] = mod


class _Log:
    def __init__(self):
        self.warnings = []

    def info(self, msg):
        pass

    def warn(self, msg):
        self.warnings.append(msg)


@pytest.fixture
def link_cls():
    _install_fake_serial()
    from scout2map_bridge.serial_link import SerialLink
    return SerialLink


def test_stop_is_clean_while_a_read_is_in_flight(link_cls):
    """No thread traceback, and the reader actually exits."""
    caught = []
    previous = threading.excepthook
    threading.excepthook = lambda a: caught.append("".join(
        traceback.format_exception(a.exc_type, a.exc_value, a.exc_traceback)))
    try:
        link = link_cls("/dev/fake", 115200, lambda b: None, _Log())
        link.start()
        time.sleep(0.3)
        assert link.is_open

        link.stop()
        time.sleep(0.3)

        assert not link.is_alive(), "reader thread outlived stop()"
        assert caught == [], f"uncaught exception in reader thread:\n{caught}"
    finally:
        threading.excepthook = previous


def test_join_and_is_alive_are_usable(link_cls):
    """Regression: the stop Event used to be named _stop.

    threading.Thread has its own private _stop(), which join() and
    is_alive() call internally, so shadowing it made both raise
    "'Event' object is not callable" on this class.
    """
    link = link_cls("/dev/fake", 115200, lambda b: None, _Log())
    link.start()
    time.sleep(0.2)

    assert link.is_alive() is True
    link.stop()
    link.join(1.0)
    assert link.is_alive() is False
