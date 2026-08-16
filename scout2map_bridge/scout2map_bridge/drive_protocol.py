#!/usr/bin/env python3
#
# File   : drive_protocol.py
# Purpose: Host side of the STM32 drive controller wire format. Mirrors
#          lib/control/protocol.h and lib/control/framing.c in the
#          S2M-FW-DrivingControl repository, which are the single source
#          of truth. Keep this file in step with that header.
# Author : jihoonkimtech
#
# Frame layout
#   [0]     0xAA          sync high
#   [1]     0x55          sync low
#   [2]     TYPE
#   [3]     LEN           payload length, excludes header and CRC
#   [4..]   PAYLOAD       LEN bytes, little endian
#   [..+2]  CRC16         CCITT over TYPE, LEN and PAYLOAD, BIG endian
#
# Note the asymmetry: payload fields are little endian, but the CRC itself
# goes out high byte first. The firmware writes it that way explicitly
# (framing.c), so a little endian read here would reject every frame.

import struct

PROTO_VERSION = 1

SYNC0 = 0xAA
SYNC1 = 0x55
HEADER_LEN = 4
CRC_LEN = 2
MAX_PAYLOAD = 56
MAX_FRAME = HEADER_LEN + MAX_PAYLOAD + CRC_LEN   # 62, fits one bulk transfer

# ---- Host to MCU ----
MSG_CMD_VELOCITY = 0x01     # differential drive command
MSG_CMD_WHEEL_RAW = 0x02    # direct duty, bypasses the PID
MSG_CMD_ESTOP = 0x03        # immediate coast, latches a fault
MSG_CMD_RESET_ODOM = 0x04
MSG_CMD_CLEAR_FAULT = 0x05
MSG_CMD_PING = 0x06
MSG_CMD_DIAG = 0x07
MSG_CMD_I2C_SCAN = 0x08

# ---- MCU to host ----
MSG_TELEMETRY = 0x81
MSG_PONG = 0x86
MSG_BOOT_INFO = 0x87
MSG_DIAG = 0x88
MSG_I2C_SCAN = 0x89

# The decoder rejects any type outside this set before spending a CRC on it
KNOWN_TYPES = frozenset((
    MSG_CMD_VELOCITY, MSG_CMD_WHEEL_RAW, MSG_CMD_ESTOP, MSG_CMD_RESET_ODOM,
    MSG_CMD_CLEAR_FAULT, MSG_CMD_PING, MSG_CMD_DIAG, MSG_CMD_I2C_SCAN,
    MSG_TELEMETRY, MSG_PONG, MSG_BOOT_INFO, MSG_DIAG, MSG_I2C_SCAN,
))

# ---- Status bits in the telemetry frame ----
STATUS_MOTOR_ENABLED = 1 << 0
STATUS_OPENLOOP = 1 << 1
STATUS_FAULT_STALL = 1 << 2
STATUS_CMD_TIMEOUT = 1 << 3
STATUS_ESTOP_LATCHED = 1 << 4
STATUS_IMU_OK = 1 << 5
STATUS_BATT_WARN = 1 << 6
STATUS_BATT_CRITICAL = 1 << 7
STATUS_IMU_CALIBRATED = 1 << 8
STATUS_BATT_DEAD = 1 << 9

# ---- Payload layouts ----
# Field order follows the packed structs in protocol.h. These format
# strings were cross checked byte for byte against tools/s2m_console.py
# in the firmware repository.
#
# The size comments in protocol.h are stale (they read 44, 16 and 17).
# The struct field lists are authoritative and give 56, 24 and 18.
# Never hard code these numbers; derive them with calcsize so a firmware
# change surfaces as a clear version error instead of a silent misparse.
TELEMETRY_FMT = "<IiihhiiihhhhhhhhHHhhHBB"
BOOT_INFO_FMT = "<BBBBHH"
DIAG_FMT = "<BBBBIIHHHHHH"
I2C_SCAN_FMT = "<BB16s"

TELEMETRY_LEN = struct.calcsize(TELEMETRY_FMT)   # 56
BOOT_INFO_LEN = struct.calcsize(BOOT_INFO_FMT)   # 8
DIAG_LEN = struct.calcsize(DIAG_FMT)             # 24
I2C_SCAN_LEN = struct.calcsize(I2C_SCAN_FMT)     # 18

# ---- Scaling, must match the firmware exactly ----
MMPS_PER_MPS = 1000.0           # linear velocity
MRADPS_PER_RADPS = 1000.0       # angular velocity
MM_PER_M = 1000.0               # position
MRAD_PER_RAD = 1000.0           # heading
QUAT_SCALE = 1.0 / 16384.0      # BNO055 native
GYRO_SCALE = 1.0 / 16.0         # BNO055 native, degrees per second
ACCEL_SCALE = 1.0 / 100.0       # BNO055 native, m/s2, gravity included

# Distance sentinels. Both mean "no usable number", but for opposite reasons,
# and a costmap must treat them differently.
DIST_TOO_CLOSE = 0xFFFE         # something is inside the minimum range
DIST_OUT_OF_RANGE = 0xFFFF      # nothing detected within the maximum range

INT16_MIN, INT16_MAX = -32768, 32767


class ProtocolVersionMismatch(Exception):
    """Payload length does not match this file's expectation.

    Always means the board is running firmware that predates or postdates
    this checkout. Worth naming plainly, because a struct.error names
    neither the frame nor the remedy.
    """


def crc16(data: bytes) -> int:
    """CRC16-CCITT, polynomial 0x1021, initial value 0xFFFF.

    Must match proto_crc16 in the firmware bit for bit.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    """Build a complete frame ready for the wire."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)}B exceeds {MAX_PAYLOAD}B")
    body = bytes((msg_type, len(payload))) + payload
    # CRC goes out big endian even though the payload is little endian
    return bytes((SYNC0, SYNC1)) + body + struct.pack(">H", crc16(body))


def encode_velocity(linear_mps: float, angular_radps: float) -> bytes:
    """Velocity command. Values are clamped to the int16 wire range."""
    mmps = _clamp_i16(round(linear_mps * MMPS_PER_MPS))
    mradps = _clamp_i16(round(angular_radps * MRADPS_PER_RADPS))
    return encode(MSG_CMD_VELOCITY, struct.pack("<hh", mmps, mradps))


def encode_wheel_raw(left_permille: int, right_permille: int) -> bytes:
    """Direct duty command. Bypasses the velocity loop; bring-up only."""
    left = max(-1000, min(1000, int(left_permille)))
    right = max(-1000, min(1000, int(right_permille)))
    return encode(MSG_CMD_WHEEL_RAW, struct.pack("<hh", left, right))


def _clamp_i16(value: int) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(value)))


def unpack(fmt: str, payload: bytes, what: str):
    """Unpack with a diagnosable error instead of a struct traceback."""
    want = struct.calcsize(fmt)
    if len(payload) != want:
        raise ProtocolVersionMismatch(
            f"{what} frame is {len(payload)} bytes, this bridge expects {want}. "
            f"The board is running different firmware than this checkout. "
            f"Rebuild and reflash the MCU, or update scout2map_bridge.")
    return struct.unpack(fmt, payload)


class FrameDecoder:
    """Byte stream to frames. Mirrors the firmware state machine.

    Resynchronisation matters more than it looks. A dropped USB byte shifts
    every subsequent frame, so the decoder must be able to find its footing
    again from arbitrary garbage rather than deadlocking on a bad length.
    """

    def __init__(self, max_buffer: int = 4096):
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.frames_ok = 0
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, data: bytes):
        """Consume bytes, return a list of (msg_type, payload) tuples."""
        self._buf.extend(data)

        # A stream this far out of sync is not going to recover by waiting
        if len(self._buf) > self._max_buffer:
            del self._buf[:-MAX_FRAME]
            self.resyncs += 1

        out = []
        while True:
            start = self._buf.find(bytes((SYNC0, SYNC1)))
            if start < 0:
                # Keep one byte back in case a sync pair straddles the read
                if len(self._buf) > 1:
                    del self._buf[:-1]
                return out
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < HEADER_LEN:
                return out

            mtype, mlen = self._buf[2], self._buf[3]

            # Validate before spending a CRC: a bogus length would otherwise
            # make the decoder wait for bytes that are never coming
            if mtype not in KNOWN_TYPES or mlen > MAX_PAYLOAD:
                del self._buf[:2]
                self.resyncs += 1
                continue

            total = HEADER_LEN + mlen + CRC_LEN
            if len(self._buf) < total:
                return out

            body = bytes(self._buf[2:HEADER_LEN + mlen])
            rx_crc = struct.unpack(
                ">H", bytes(self._buf[HEADER_LEN + mlen:total]))[0]

            if crc16(body) == rx_crc:
                payload = bytes(self._buf[HEADER_LEN:HEADER_LEN + mlen])
                out.append((mtype, payload))
                self.frames_ok += 1
                del self._buf[:total]
            else:
                # Drop only the sync pair: the real frame may start inside
                # what we mistook for a payload
                self.crc_errors += 1
                del self._buf[:2]


def unpack_calib(packed: int):
    """BNO055 calibration byte: bits 7:6 sys, 5:4 gyr, 3:2 acc, 1:0 mag."""
    return (
        (packed >> 6) & 0x03,
        (packed >> 4) & 0x03,
        (packed >> 2) & 0x03,
        packed & 0x03,
    )
