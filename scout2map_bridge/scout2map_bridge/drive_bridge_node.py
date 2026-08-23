#!/usr/bin/env python3
#
# File   : drive_bridge_node.py
# Purpose: Bridge the STM32 drive controller to ROS 2. Decodes the binary
#          telemetry stream into standard messages, and forwards cmd_vel
#          down to the MCU.
# Author : jihoonkimtech
#
# Unlike the Pico 2 bridge this link is bidirectional, which brings two
# obligations. The MCU stops the motors if it hears nothing for 300ms, so
# commands must be repeated; and if the ROS side goes quiet the bridge must
# command a stop rather than let the last command stand.

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, Range
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from scout2map_msgs.msg import DriveDiagnostics, DriveStatus

from . import drive_protocol as proto
from .serial_link import SerialLink

RX_QUEUE_MAX = 256
DEG_TO_RAD = math.pi / 180.0


class DriveBridge(Node):
    """Serial bridge for the STM32 drive controller."""

    def __init__(self):
        super().__init__("drive_bridge")

        # ---------------- Parameters ----------------
        self.declare_parameter("port", "/dev/scout2map_drive")
        self.declare_parameter("baudrate", 115200)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("range_frame", "range_link")
        self.declare_parameter("publish_tf", True)

        # Command handling
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("max_linear_mps", 0.20)
        self.declare_parameter("max_angular_radps", 0.80)

        # Link health
        self.declare_parameter("link_timeout_s", 0.5)
        self.declare_parameter("status_rate_hz", 10.0)

        # Distance sensor geometry, depends on which part is fitted
        self.declare_parameter("range_min_m", 0.04)
        self.declare_parameter("range_max_m", 0.30)
        self.declare_parameter("range_fov_rad", 0.14)

        # Fallback until BOOT_INFO arrives
        self.declare_parameter("default_track_width_m", 0.24)

        # --- Slip detection ---
        # This chassis is a four wheel skid steer, so a turn is produced by
        # dragging the wheels sideways rather than by rolling them. The
        # geometric track width therefore under predicts how much wheel speed
        # difference a given yaw rate needs. The correction is empirical and
        # typically lands between 1.2 and 1.5; leaving it at 1.0 makes every
        # deliberate turn look like slip.
        # Measure it with tools/skid_calib.py before trusting the signal.
        self.declare_parameter("skid_factor", 1.0)

        # BNO055 reports 1-2 deg/s at rest before its gyro calibrates.
        # Measure with s2m_imu_view.py --bias in the firmware repository.
        self.declare_parameter("gyro_bias_radps", 0.0)

        # Below this the ratio denominator is noise, so the signal is muted
        # --- Battery ---
        # State of charge is estimated from pack voltage against a LiPo
        # discharge curve. There is no current sensor, so this is an estimate
        # rather than coulomb counting, and it reads low under motor load.
        self.declare_parameter("battery_cells", 3)
        self.declare_parameter("battery_capacity_ah", 5.25)

        # Per-cell breakpoints, descending. Defaults are a lightly loaded LiPo
        # curve; the points bunch up near 3.7V because that is where the knee
        # is and a coarse table there would read wildly wrong.
        self.declare_parameter("battery_curve_cell_v", [
            4.20, 4.10, 4.00, 3.95, 3.87, 3.83,
            3.79, 3.75, 3.71, 3.65, 3.50, 3.30,
        ])
        self.declare_parameter("battery_curve_soc", [
            1.00, 0.90, 0.80, 0.70, 0.60, 0.50,
            0.40, 0.30, 0.20, 0.10, 0.05, 0.00,
        ])

        # Voltage sags the moment the motors draw current, so an unsmoothed
        # estimate swings by tens of percent every time the robot starts and
        # stops. Smoothing trades responsiveness for a readable number.
        self.declare_parameter("battery_smoothing_s", 10.0)

        # Set false to go back to publishing NaN
        self.declare_parameter("battery_estimate_soc", True)

        self.declare_parameter("slip_rate_floor_radps", 0.10)
        self.declare_parameter("slip_min_wheel_speed_mps", 0.02)

        # Covariance knobs. Encoder odometry is decent in x, poor in yaw.
        self.declare_parameter("odom_xy_variance", 0.001)
        self.declare_parameter("odom_yaw_variance", 0.01)
        self.declare_parameter("odom_openloop_multiplier", 100.0)

        # --- Odom/TF orientation source ---
        # The BNO055 on the same MCU already fuses an absolute orientation and
        # sends it in every telemetry frame, whether or not the wheels are
        # turning. Encoder heading only updates once the robot actually moves,
        # so at power-on it sits at its initial value while the IMU (and the
        # RViz Imu display, which reads orientation straight off the message)
        # already shows the real attitude. That mismatch is what carries into
        # the map as a heading offset if autonomous nav starts before the
        # robot has driven anywhere. Sourcing odom/TF orientation from the IMU
        # instead keeps both in agreement from the first telemetry frame.
        self.declare_parameter("use_imu_orientation", True)

        # Publish only the yaw component of the IMU quaternion. slam_toolbox
        # and Nav2 both treat this chassis as planar, so passing roll/pitch
        # through here would fight that assumption instead of helping it.
        self.declare_parameter("imu_orientation_yaw_only", True)

        # Yaw variance to report while the BNO055 has not finished
        # calibrating. Mirrors the calibrated/uncalibrated split already used
        # in _publish_imu: an uncalibrated heading is a guess, and a filter
        # downstream (or a person watching the topic) needs to be told that.
        self.declare_parameter("odom_yaw_variance_uncalibrated", 1.0)
        # Twist is measured, not integrated, so it gets its own numbers.
        # Derived from the 1.8 mm/s speed quantum as q^2/12.
        self.declare_parameter("odom_vx_variance", 2.7e-7)
        self.declare_parameter("odom_wz_variance", 9.4e-6)

        gp = self.get_parameter
        self._port = gp("port").value
        self._odom_frame = gp("odom_frame").value
        self._base_frame = gp("base_frame").value
        self._imu_frame = gp("imu_frame").value
        self._range_frame = gp("range_frame").value
        self._publish_tf = bool(gp("publish_tf").value)
        self._cmd_timeout = float(gp("command_timeout_s").value)
        self._max_lin = float(gp("max_linear_mps").value)
        self._max_ang = float(gp("max_angular_radps").value)
        self._link_timeout = float(gp("link_timeout_s").value)
        self._range_min = float(gp("range_min_m").value)
        self._range_max = float(gp("range_max_m").value)
        self._range_fov = float(gp("range_fov_rad").value)
        self._track_width = float(gp("default_track_width_m").value)
        self._odom_xy_var = float(gp("odom_xy_variance").value)
        self._odom_yaw_var = float(gp("odom_yaw_variance").value)
        self._openloop_mult = float(gp("odom_openloop_multiplier").value)
        self._use_imu_orientation = bool(gp("use_imu_orientation").value)
        self._imu_yaw_only = bool(gp("imu_orientation_yaw_only").value)
        self._odom_yaw_var_uncal = float(
            gp("odom_yaw_variance_uncalibrated").value)
        self._odom_vx_var = float(gp("odom_vx_variance").value)
        self._odom_wz_var = float(gp("odom_wz_variance").value)
        self._skid_factor = float(gp("skid_factor").value)
        self._gyro_bias = float(gp("gyro_bias_radps").value)
        # --- Battery state of charge ---
        self._batt_cells = max(1, int(gp("battery_cells").value))
        self._batt_capacity_ah = float(gp("battery_capacity_ah").value)
        self._batt_estimate = bool(gp("battery_estimate_soc").value)
        self._batt_smoothing = max(0.0, float(gp("battery_smoothing_s").value))
        self._batt_filtered_v = None
        self._batt_last_mono = None

        curve_v = [float(v) for v in gp("battery_curve_cell_v").value]
        curve_soc = [float(v) for v in gp("battery_curve_soc").value]
        if len(curve_v) != len(curve_soc) or len(curve_v) < 2:
            self.get_logger().warn(
                "battery curve is malformed; state of charge disabled")
            self._batt_estimate = False
            self._batt_curve = []
        else:
            # Sort descending by voltage so the lookup can assume ordering no
            # matter how the parameter file lists them
            self._batt_curve = sorted(
                zip(curve_v, curve_soc), key=lambda p: p[0], reverse=True)

        self._slip_floor = float(gp("slip_rate_floor_radps").value)
        self._slip_min_speed = float(gp("slip_min_wheel_speed_mps").value)

        # ---------------- QoS ----------------
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

        # ---------------- Publishers ----------------
        self._pub_odom = self.create_publisher(Odometry, "drive/odom", sensor_qos)
        self._pub_imu = self.create_publisher(Imu, "drive/imu", sensor_qos)
        self._pub_range = self.create_publisher(Range, "drive/range", sensor_qos)
        self._pub_batt = self.create_publisher(
            BatteryState, "drive/battery", sensor_qos)
        self._pub_status = self.create_publisher(
            DriveStatus, "drive/status", sensor_qos)
        self._pub_diag = self.create_publisher(
            DriveDiagnostics, "drive/diagnostics", status_qos)

        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        # ---------------- Subscriber ----------------
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        # ---------------- Services ----------------
        self.create_service(Trigger, "drive/estop", self._srv_estop)
        self.create_service(Trigger, "drive/clear_fault", self._srv_clear_fault)
        self.create_service(Trigger, "drive/reset_odom", self._srv_reset_odom)
        self.create_service(Trigger, "drive/request_diagnostics", self._srv_diag)

        # ---------------- State ----------------
        self._cmd_lock = threading.Lock()
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._cmd_stamp = 0.0
        self._cmd_zeroed = True     # avoids logging the stop warning repeatedly

        self._decoder = proto.FrameDecoder()
        self._rx = []
        self._rx_lock = threading.Lock()

        self._last_frame_mono = 0.0
        self._mcu_uptime_ms = 0
        self._mcu_reboots = 0
        self._boot_info = None
        self._version_warned = False
        self._estop_warned = False
        self._last_status_flags = 0

        # ---------------- Serial ----------------
        self._link = SerialLink(
            self._port, int(gp("baudrate").value),
            self._on_serial_bytes, self.get_logger(),
            on_open=self._on_port_open)
        self._link.start()

        # ---------------- Timers ----------------
        self.create_timer(0.005, self._drain_rx)
        cmd_hz = max(1.0, float(gp("command_rate_hz").value))
        self.create_timer(1.0 / cmd_hz, self._tick_command)
        status_hz = max(0.1, float(gp("status_rate_hz").value))
        self.create_timer(1.0 / status_hz, self._publish_status)

        self.get_logger().info(
            f"drive_bridge up: port={self._port} "
            f"cmd={cmd_hz:.0f}Hz timeout={self._cmd_timeout:.2f}s "
            f"limits={self._max_lin:.2f}m/s {self._max_ang:.2f}rad/s")

        if self._skid_factor == 1.0:
            self.get_logger().warn(
                "skid_factor is 1.0, the unmeasured default. On a four wheel "
                "skid steer every deliberate turn will register as slip. "
                "Measure it with tools/skid_calib.py before using slip_ratio.")
        if self._gyro_bias == 0.0:
            self.get_logger().warn(
                "gyro_bias_radps is 0.0. The BNO055 reads 1-2 deg/s at rest "
                "before its gyro calibrates, which the slip signal will read "
                "as rotation. Measure with s2m_imu_view.py --bias.")
        if self._use_imu_orientation:
            self.get_logger().info(
                "odom/TF orientation sourced from the BNO055 "
                f"(yaw_only={self._imu_yaw_only}); falls back to encoder "
                "heading until the first STATUS_IMU_OK telemetry frame.")
        else:
            self.get_logger().warn(
                "use_imu_orientation is false, odom/TF orientation is pure "
                "encoder heading and will disagree with the Imu display "
                "until the wheels move.")

    # ------------------------------------------------------------------
    # Serial callbacks
    # ------------------------------------------------------------------
    def _on_serial_bytes(self, data: bytes):
        """Runs on the reader thread. Decode here, publish on the ROS timer."""
        frames = self._decoder.feed(data)
        if not frames:
            return
        now = time.monotonic()
        with self._rx_lock:
            self._rx.extend((now, t, p) for t, p in frames)
            if len(self._rx) > RX_QUEUE_MAX:
                del self._rx[:-RX_QUEUE_MAX]

    def _on_port_open(self):
        # A reopened port means a fresh MCU session; ask who we are talking to
        self._boot_info = None
        self._version_warned = False
        self._link.write(proto.encode(proto.MSG_CMD_PING))

    def _drain_rx(self):
        with self._rx_lock:
            batch, self._rx = self._rx, []
        for mono, mtype, payload in batch:
            self._last_frame_mono = mono
            try:
                self._handle_frame(mtype, payload)
            except proto.ProtocolVersionMismatch as exc:
                if not self._version_warned:
                    self._version_warned = True
                    self.get_logger().error(str(exc))

    def _handle_frame(self, mtype, payload):
        if mtype == proto.MSG_TELEMETRY:
            self._on_telemetry(payload)
        elif mtype == proto.MSG_BOOT_INFO:
            self._on_boot_info(payload)
        elif mtype == proto.MSG_DIAG:
            self._on_diag(payload)
        elif mtype == proto.MSG_PONG:
            self.get_logger().info("MCU responded to ping")
        elif mtype == proto.MSG_I2C_SCAN:
            self._on_i2c_scan(payload)

    # ------------------------------------------------------------------
    # Frame handlers
    # ------------------------------------------------------------------
    def _on_boot_info(self, payload):
        (ver, major, minor, patch,
         counts_per_rev, track_mm) = proto.unpack(
            proto.BOOT_INFO_FMT, payload, "boot info")

        self._boot_info = {
            "proto": ver, "major": major, "minor": minor, "patch": patch,
            "counts_per_rev": counts_per_rev, "track_mm": track_mm,
        }
        if track_mm > 0:
            # Trust the firmware over the parameter: it knows the build
            self._track_width = track_mm / 1000.0

        self.get_logger().info(
            f"MCU boot: fw {major}.{minor}.{patch}, proto v{ver}, "
            f"{counts_per_rev} counts/rev, track {track_mm}mm")

        if ver != proto.PROTO_VERSION:
            self.get_logger().error(
                f"protocol version mismatch: MCU speaks v{ver}, "
                f"this bridge speaks v{proto.PROTO_VERSION}. "
                "Reflash the MCU or update scout2map_bridge.")

    def _on_telemetry(self, payload):
        f = proto.unpack(proto.TELEMETRY_FMT, payload, "telemetry")
        (timestamp_ms, enc_l, enc_r, spd_l, spd_r,
         odom_x, odom_y, odom_th,
         qw, qx, qy, qz, gyro_z, acc_x, acc_y, acc_z,
         dist_mm, batt_mv, duty_l, duty_r, status, calib, _reserved) = f

        # A backwards jump means the MCU restarted under us
        if timestamp_ms < self._mcu_uptime_ms:
            self._mcu_reboots += 1
            self._boot_info = None
            self.get_logger().warn(
                f"MCU reboot detected (uptime {self._mcu_uptime_ms} -> {timestamp_ms})")
        self._mcu_uptime_ms = timestamp_ms

        stamp = self.get_clock().now().to_msg()
        openloop = bool(status & proto.STATUS_OPENLOOP)

        self._publish_odom(stamp, odom_x, odom_y, odom_th, spd_l, spd_r,
                          openloop, qw, qx, qy, qz, status)
        self._publish_imu(stamp, qw, qx, qy, qz, gyro_z, acc_x, acc_y, acc_z, status)
        self._publish_range(stamp, dist_mm)
        self._publish_battery(stamp, batt_mv, status)
        slip = self._compute_slip(spd_l, spd_r, gyro_z, status)
        self._cache_status(timestamp_ms, enc_l, enc_r, spd_l, spd_r,
                           duty_l, duty_r, status, calib, slip)
        self._warn_on_status_change(status)

    def _on_diag(self, payload):
        d = proto.unpack(proto.DIAG_FMT, payload, "diagnostics")
        (init_step, chip_id, calib, _res, read_ok, read_fail,
         i2c_err, i2c_rec, batt_counts, batt_mv,
         dist_counts, dist_mv) = d

        msg = DriveDiagnostics()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.imu_init_step = init_step
        msg.imu_chip_id = chip_id
        (msg.calib_sys, msg.calib_gyro,
         msg.calib_accel, msg.calib_mag) = proto.unpack_calib(calib)
        msg.imu_read_ok = read_ok
        msg.imu_read_fail = read_fail
        msg.i2c_errors = i2c_err
        msg.i2c_recoveries = i2c_rec
        msg.battery_counts = batt_counts
        msg.battery_mv = batt_mv
        msg.distance_counts = dist_counts
        msg.distance_mv = dist_mv
        self._pub_diag.publish(msg)

        self.get_logger().info(
            f"[diag] imu chip 0x{chip_id:02X} step {init_step} "
            f"reads {read_ok}/{read_fail} i2c err/rec {i2c_err}/{i2c_rec} "
            f"batt {batt_mv}mV dist {dist_mv}mV")

    def _on_i2c_scan(self, payload):
        count, lines, bitmap = proto.unpack(
            proto.I2C_SCAN_FMT, payload, "i2c scan")
        found = [addr for addr in range(128)
                 if bitmap[addr // 8] & (1 << (addr % 8))]
        addrs = ", ".join(f"0x{a:02X}" for a in found) if found else "none"
        self.get_logger().info(
            f"[i2c scan] {count} device(s): {addrs} "
            f"(SCL {'high' if lines & 1 else 'LOW'}, "
            f"SDA {'high' if lines & 2 else 'LOW'})")

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_odom(self, stamp, x_mm, y_mm, th_mrad, spd_l, spd_r,
                      openloop, qw, qx, qy, qz, status):
        x = x_mm / proto.MM_PER_M
        y = y_mm / proto.MM_PER_M
        theta_enc = th_mrad / proto.MRAD_PER_RAD

        # Body twist from the two wheel speeds
        v_l = spd_l / proto.MMPS_PER_MPS
        v_r = spd_r / proto.MMPS_PER_MPS
        v = (v_l + v_r) * 0.5
        w = (v_r - v_l) / self._track_width if self._track_width > 0 else 0.0

        # Encoder heading only advances once the wheels turn, so right after
        # boot it sits at its initial value while the BNO055 already knows
        # the real attitude (it fuses accel+gyro+mag continuously, whether or
        # not the robot has moved). Preferring the IMU here is what keeps
        # RobotModel/TF in agreement with the Imu display from the first
        # telemetry frame instead of only after the first deliberate turn.
        orientation, yaw_var = self._resolve_orientation(
            theta_enc, qw, qx, qy, qz, status)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = orientation
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        # Without encoders the pose is a guess, and downstream filters need
        # to be told that rather than left to discover it
        mult = self._openloop_mult if openloop else 1.0

        # Pose variance grows without bound as integration error accumulates,
        # so these are a floor rather than a true estimate. A filter that
        # needs real numbers should fuse the IMU instead of trusting these.
        pose_cov = [0.0] * 36
        pose_cov[0] = self._odom_xy_var * mult      # x
        pose_cov[7] = self._odom_xy_var * mult      # y
        pose_cov[14] = 1e6                          # z, planar robot
        pose_cov[21] = 1e6                          # roll
        pose_cov[28] = 1e6                          # pitch
        pose_cov[35] = yaw_var * mult                # yaw
        msg.pose.covariance = pose_cov

        # Twist is a direct measurement, not an integration, so it carries
        # its own error. Reusing the pose numbers here would be a unit
        # error: those are m^2 and rad^2, these are (m/s)^2 and (rad/s)^2.
        twist_cov = [0.0] * 36
        twist_cov[0] = self._odom_vx_var * mult     # vx
        twist_cov[7] = 1e6                          # vy, non-holonomic
        twist_cov[14] = 1e6                         # vz
        twist_cov[21] = 1e6                         # roll rate
        twist_cov[28] = 1e6                         # pitch rate
        twist_cov[35] = self._odom_wz_var * mult    # yaw rate
        msg.twist.covariance = twist_cov
        self._pub_odom.publish(msg)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    def _resolve_orientation(self, theta_enc, qw, qx, qy, qz, status):
        """Pick the odom/TF orientation and its yaw variance.

        Falls back to encoder heading whenever the IMU is disabled by
        parameter or the current frame reports STATUS_IMU_OK false (BNO055
        still booting, or the MCU lost it). That keeps the very first frames
        after power-on sane instead of publishing a stale or zeroed quaternion.
        """
        imu_ok = bool(status & proto.STATUS_IMU_OK)
        if not (self._use_imu_orientation and imu_ok):
            return _yaw_to_quat(theta_enc), self._odom_yaw_var

        w = qw * proto.QUAT_SCALE
        x = qx * proto.QUAT_SCALE
        y = qy * proto.QUAT_SCALE
        z = qz * proto.QUAT_SCALE

        if self._imu_yaw_only:
            # The rest of the stack (slam_toolbox, Nav2) assumes a planar
            # robot, so roll/pitch from the IMU would only fight that.
            orientation = _yaw_to_quat(_quat_to_yaw(w, x, y, z))
        else:
            orientation = Quaternion(w=w, x=x, y=y, z=z)

        calibrated = bool(status & proto.STATUS_IMU_CALIBRATED)
        yaw_var = self._odom_yaw_var if calibrated else self._odom_yaw_var_uncal
        return orientation, yaw_var

    def _publish_imu(self, stamp, qw, qx, qy, qz, gyro_z,
                     acc_x, acc_y, acc_z, status):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame

        if not (status & proto.STATUS_IMU_OK):
            # Convention: -1 in element 0 marks the whole component absent
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
            self._pub_imu.publish(msg)
            return

        msg.orientation.w = qw * proto.QUAT_SCALE
        msg.orientation.x = qx * proto.QUAT_SCALE
        msg.orientation.y = qy * proto.QUAT_SCALE
        msg.orientation.z = qz * proto.QUAT_SCALE

        # Only the yaw rate is transmitted; roll and pitch rates are unknown
        msg.angular_velocity.z = gyro_z * proto.GYRO_SCALE * DEG_TO_RAD

        # Gravity is included, as sensor_msgs expects
        msg.linear_acceleration.x = acc_x * proto.ACCEL_SCALE
        msg.linear_acceleration.y = acc_y * proto.ACCEL_SCALE
        msg.linear_acceleration.z = acc_z * proto.ACCEL_SCALE

        # Heading drifts until the magnetometer is calibrated, so the
        # covariance has to say so or a filter will over trust it
        calibrated = bool(status & proto.STATUS_IMU_CALIBRATED)
        yaw_var = 0.01 if calibrated else 1.0
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, yaw_var,
        ]
        msg.angular_velocity_covariance = [
            1e6, 0.0, 0.0,      # roll rate not transmitted
            0.0, 1e6, 0.0,      # pitch rate not transmitted
            0.0, 0.0, 0.001,
        ]
        msg.linear_acceleration_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05,
        ]
        self._pub_imu.publish(msg)

    def _publish_range(self, stamp, dist_mm):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = self._range_frame
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self._range_fov
        msg.min_range = self._range_min
        msg.max_range = self._range_max

        if dist_mm == proto.DIST_OUT_OF_RANGE:
            # Nothing detected. Positive infinity is the conventional way to
            # say "clear to the maximum range" and costmaps understand it.
            msg.range = float("inf")
        elif dist_mm == proto.DIST_TOO_CLOSE:
            # Something is inside the blind spot. The distance is unknown but
            # the obstacle is real, so report the closest measurable value
            # rather than infinity, which would read as clear ground.
            msg.range = self._range_min
        else:
            msg.range = dist_mm / proto.MM_PER_M
        self._pub_range.publish(msg)

    def _publish_battery(self, stamp, batt_mv, status):
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True

        if batt_mv == 0:
            # The ADC has not reported yet, or is not fitted on this build
            msg.voltage = float("nan")
            msg.present = False
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        else:
            msg.voltage = batt_mv / 1000.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if status & proto.STATUS_BATT_DEAD:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            else:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        soc = self._estimate_soc(msg.voltage) if msg.present else None

        if soc is None:
            msg.percentage = float("nan")
            msg.charge = float("nan")
        else:
            msg.percentage = soc
            msg.charge = soc * self._batt_capacity_ah

        if self._batt_capacity_ah > 0.0:
            msg.capacity = self._batt_capacity_ah
            msg.design_capacity = self._batt_capacity_ah
        else:
            msg.capacity = float("nan")
            msg.design_capacity = float("nan")

        # There is no shunt on this board, so current stays unknown. That is
        # also why percentage is an estimate and not coulomb counting.
        msg.current = float("nan")
        self._pub_batt.publish(msg)

    def _estimate_soc(self, voltage):
        """State of charge from pack voltage, or None when not estimable.

        Voltage under load sits well below the resting value, so the reading
        is smoothed before lookup. Even so this reads pessimistic while
        driving and recovers a few seconds after stopping. Treat it as a
        coarse gauge, not a fuel meter.
        """
        if not self._batt_estimate or not self._batt_curve:
            return None
        if voltage != voltage or voltage <= 0.0:   # NaN or no reading
            return None

        now = time.monotonic()
        if self._batt_filtered_v is None or self._batt_smoothing <= 0.0:
            self._batt_filtered_v = voltage
        else:
            dt = now - (self._batt_last_mono or now)
            # Exponential moving average with a time constant in seconds, so
            # the behaviour does not change if the telemetry rate does
            alpha = 1.0 - math.exp(-max(0.0, dt) / self._batt_smoothing)
            self._batt_filtered_v += alpha * (voltage - self._batt_filtered_v)
        self._batt_last_mono = now

        cell_v = self._batt_filtered_v / self._batt_cells
        curve = self._batt_curve

        if cell_v >= curve[0][0]:
            return float(curve[0][1])
        if cell_v <= curve[-1][0]:
            return float(curve[-1][1])

        for (v_hi, soc_hi), (v_lo, soc_lo) in zip(curve, curve[1:]):
            if v_lo <= cell_v <= v_hi:
                span = v_hi - v_lo
                if span <= 0.0:
                    return float(soc_lo)
                ratio = (cell_v - v_lo) / span
                return float(soc_lo + ratio * (soc_hi - soc_lo))
        return None

    def _compute_slip(self, spd_l, spd_r, gyro_z, status):
        """Compare the two independent yaw rate estimates.

        Returns (yaw_enc, yaw_imu, error, ratio, valid). Emits a signal only;
        deciding what counts as slip is the event engine's job.
        """
        v_l = spd_l / proto.MMPS_PER_MPS
        v_r = spd_r / proto.MMPS_PER_MPS

        # Effective track width, not the geometric one. On a skid steer the
        # wheels scrub through a turn, so the chassis rotates less than pure
        # rolling would predict.
        effective_track = self._track_width * self._skid_factor
        if effective_track <= 0.0:
            return 0.0, 0.0, 0.0, 0.0, False

        yaw_enc = (v_r - v_l) / effective_track
        yaw_imu = gyro_z * proto.GYRO_SCALE * DEG_TO_RAD - self._gyro_bias
        error = yaw_enc - yaw_imu

        # Gates. Any one of these makes the comparison meaningless, and a
        # meaningless number that looks plausible is worse than no number.
        moving = max(abs(v_l), abs(v_r)) >= self._slip_min_speed
        valid = (
            bool(status & proto.STATUS_IMU_OK)
            and bool(status & proto.STATUS_IMU_CALIBRATED)
            and not (status & proto.STATUS_OPENLOOP)
            and moving
        )

        # Normalise against whichever estimate is larger, with a floor so a
        # near stationary chassis cannot divide a small error into a big ratio
        scale = max(abs(yaw_enc), abs(yaw_imu), self._slip_floor)
        ratio = abs(error) / scale

        return yaw_enc, yaw_imu, error, ratio, valid

    def _cache_status(self, timestamp_ms, enc_l, enc_r, spd_l, spd_r,
                      duty_l, duty_r, status, calib, slip):
        msg = DriveStatus()
        msg.mcu_timestamp_ms = timestamp_ms
        msg.encoder_left = enc_l
        msg.encoder_right = enc_r
        msg.speed_left_mps = spd_l / proto.MMPS_PER_MPS
        msg.speed_right_mps = spd_r / proto.MMPS_PER_MPS
        msg.duty_left_permille = duty_l
        msg.duty_right_permille = duty_r
        msg.status_flags = status
        msg.motor_enabled = bool(status & proto.STATUS_MOTOR_ENABLED)
        msg.openloop = bool(status & proto.STATUS_OPENLOOP)
        msg.fault_stall = bool(status & proto.STATUS_FAULT_STALL)
        msg.cmd_timeout = bool(status & proto.STATUS_CMD_TIMEOUT)
        msg.estop_latched = bool(status & proto.STATUS_ESTOP_LATCHED)
        msg.imu_ok = bool(status & proto.STATUS_IMU_OK)
        msg.imu_calibrated = bool(status & proto.STATUS_IMU_CALIBRATED)
        msg.batt_warn = bool(status & proto.STATUS_BATT_WARN)
        msg.batt_critical = bool(status & proto.STATUS_BATT_CRITICAL)
        msg.batt_dead = bool(status & proto.STATUS_BATT_DEAD)
        (msg.calib_sys, msg.calib_gyro,
         msg.calib_accel, msg.calib_mag) = proto.unpack_calib(calib)
        (msg.yaw_rate_encoder_radps, msg.yaw_rate_imu_radps,
         msg.yaw_rate_error_radps, msg.slip_ratio,
         msg.slip_signal_valid) = slip
        self._pending_status = msg

    def _publish_status(self):
        msg = getattr(self, "_pending_status", None)
        if msg is None:
            msg = DriveStatus()
        now = time.monotonic()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.link_ok = self._link_ok(now)
        msg.last_frame_age_s = (
            float(now - self._last_frame_mono) if self._last_frame_mono else -1.0)
        msg.mcu_reboot_count = self._mcu_reboots
        msg.frames_ok = self._decoder.frames_ok
        msg.crc_errors = self._decoder.crc_errors
        if self._boot_info is not None:
            b = self._boot_info
            msg.proto_version = b["proto"]
            msg.fw_major = b["major"]
            msg.fw_minor = b["minor"]
            msg.fw_patch = b["patch"]
            msg.counts_per_wheel_rev = b["counts_per_rev"]
            msg.track_width_mm = b["track_mm"]
        self._pub_status.publish(msg)

    def _link_ok(self, now):
        if not self._link.is_open or self._last_frame_mono == 0.0:
            return False
        return (now - self._last_frame_mono) <= self._link_timeout

    def _warn_on_status_change(self, status):
        """Announce faults once on the transition, not every frame."""
        changed = status ^ self._last_status_flags
        self._last_status_flags = status
        if not changed:
            return
        log = self.get_logger()
        if changed & proto.STATUS_ESTOP_LATCHED:
            if status & proto.STATUS_ESTOP_LATCHED:
                log.error("E-stop latched. Velocity commands are ignored "
                          "until drive/clear_fault is called.")
            else:
                log.info("E-stop cleared")
        if changed & proto.STATUS_FAULT_STALL and status & proto.STATUS_FAULT_STALL:
            log.error("stall fault: high duty with no motion. Check for a "
                      "jammed wheel before clearing.")
        if changed & proto.STATUS_OPENLOOP:
            if status & proto.STATUS_OPENLOOP:
                log.warn("encoder feedback lost, MCU fell back to open loop. "
                         "Odometry is unreliable from here.")
            else:
                log.info("encoder feedback restored, closed loop resumed")
        if changed & proto.STATUS_BATT_DEAD and status & proto.STATUS_BATT_DEAD:
            log.error("battery below the cell damage point. "
                      "The MCU has cut drive. Land the robot and charge.")
        elif changed & proto.STATUS_BATT_CRITICAL and status & proto.STATUS_BATT_CRITICAL:
            log.warn("battery critical. Return to base now; the MCU will cut "
                     "drive if it falls further.")
        elif changed & proto.STATUS_BATT_WARN and status & proto.STATUS_BATT_WARN:
            log.warn("battery low")

    # ------------------------------------------------------------------
    # Command path
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist):
        lin = max(-self._max_lin, min(self._max_lin, msg.linear.x))
        ang = max(-self._max_ang, min(self._max_ang, msg.angular.z))
        with self._cmd_lock:
            self._cmd_linear = lin
            self._cmd_angular = ang
            self._cmd_stamp = time.monotonic()
            self._cmd_zeroed = False

    def _tick_command(self):
        """Repeat the current command so the MCU watchdog stays fed.

        The MCU stops after 300ms of silence. That is the backstop for a
        dead bridge, not the normal path, so the bridge sends an explicit
        zero once the ROS side goes quiet instead of waiting for it.
        """
        now = time.monotonic()
        with self._cmd_lock:
            stale = (now - self._cmd_stamp) > self._cmd_timeout
            if stale:
                if not self._cmd_zeroed:
                    self._cmd_zeroed = True
                    self.get_logger().warn(
                        f"no cmd_vel for {self._cmd_timeout:.2f}s, commanding stop")
                lin = ang = 0.0
            else:
                lin, ang = self._cmd_linear, self._cmd_angular

        self._link.write(proto.encode_velocity(lin, ang))

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _send_simple(self, msg_type, name, response):
        if self._link.write(proto.encode(msg_type)):
            response.success = True
            response.message = f"{name} sent"
        else:
            response.success = False
            response.message = f"{name} failed: serial port is not open"
        return response

    def _srv_estop(self, _request, response):
        # Zero the cached command too, so recovery needs a deliberate act
        with self._cmd_lock:
            self._cmd_linear = self._cmd_angular = 0.0
            self._cmd_stamp = 0.0
        self.get_logger().warn("E-stop requested")
        return self._send_simple(proto.MSG_CMD_ESTOP, "e-stop", response)

    def _srv_clear_fault(self, _request, response):
        return self._send_simple(
            proto.MSG_CMD_CLEAR_FAULT, "clear fault", response)

    def _srv_reset_odom(self, _request, response):
        return self._send_simple(
            proto.MSG_CMD_RESET_ODOM, "odometry reset", response)

    def _srv_diag(self, _request, response):
        return self._send_simple(
            proto.MSG_CMD_DIAG, "diagnostics request", response)

    def destroy_node(self):
        # Best effort stop: the MCU watchdog covers us if this does not land
        try:
            self._link.write(proto.encode_velocity(0.0, 0.0))
        except Exception:
            pass
        self._link.stop()
        return super().destroy_node()


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Yaw (Z axis) component of a quaternion, standard ZYX convention."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = DriveBridge()
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
