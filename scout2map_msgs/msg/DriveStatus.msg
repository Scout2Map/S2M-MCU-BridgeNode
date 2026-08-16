# Drive controller state from the STM32, decoded from the TELEMETRY frame.
# Published at the telemetry rate (50Hz by default).

std_msgs/Header header

# MCU uptime stamped at sample time. Wraps every ~49.7 days.
# header.stamp is the RPi5 receive time; use that for time alignment.
uint32 mcu_timestamp_ms

# --- Wheels ---
int32   encoder_left            # accumulated counts since MCU boot
int32   encoder_right
float32 speed_left_mps          # measured wheel surface speed
float32 speed_right_mps
int16   duty_left_permille      # applied duty, -1000..1000, before motor inversion
int16   duty_right_permille

# --- Status bitfield, forwarded verbatim from the firmware ---
uint16 STATUS_MOTOR_ENABLED=1
uint16 STATUS_OPENLOOP=2         # encoder feedback lost, running open loop
uint16 STATUS_FAULT_STALL=4
uint16 STATUS_CMD_TIMEOUT=8      # host went quiet, firmware stopped the motors
uint16 STATUS_ESTOP_LATCHED=16
uint16 STATUS_IMU_OK=32
uint16 STATUS_BATT_WARN=64
uint16 STATUS_BATT_CRITICAL=128
uint16 STATUS_IMU_CALIBRATED=256
uint16 STATUS_BATT_DEAD=512      # firmware has cut drive to protect the pack
uint16 status_flags

# Same bits unpacked, so consumers do not have to mask
bool motor_enabled
bool openloop
bool fault_stall
bool cmd_timeout
bool estop_latched
bool imu_ok
bool imu_calibrated
bool batt_warn
bool batt_critical
bool batt_dead

# --- BNO055 calibration, 0 = none, 3 = full ---
uint8 calib_sys
uint8 calib_gyro
uint8 calib_accel
uint8 calib_mag

# --- Link health ---
bool   link_ok                  # port open and a telemetry frame arrived recently
float32 last_frame_age_s
uint32 mcu_reboot_count         # incremented when mcu_timestamp_ms jumps backwards
uint64 frames_ok
uint64 crc_errors

# --- Slip detection signals ---
# Two independent estimates of how fast the chassis is turning. They agree
# while the wheels grip and diverge when they do not, which is the whole
# basis of the slip event.
#
# This is a SIGNAL, not a verdict. Thresholds, debounce and marker emission
# belong to the event engine. Publishing a raw discrepancy keeps the tuning
# in one place instead of splitting it across two repositories.
#
# NOTE: this only catches ROTATIONAL slip. Wheels spinning in a straight
# line produce encoder motion with no yaw change, and the IMU also reports
# no yaw change, so the two agree and nothing is flagged. See the bridge
# README for why the longitudinal case is not covered here.
float32 yaw_rate_encoder_radps  # from wheel speeds and the effective track width
float32 yaw_rate_imu_radps      # gyro_z with the measured bias removed
float32 yaw_rate_error_radps    # encoder minus IMU, signed
float32 slip_ratio              # normalised discrepancy, 0.0 when they agree

# False when the comparison is meaningless: IMU uncalibrated, encoders lost,
# or the chassis moving too slowly for either estimate to mean anything.
# Ignore every field above when this is false.
bool slip_signal_valid

# --- Values reported by BOOT_INFO, zero until it arrives ---
uint8  proto_version
uint8  fw_major
uint8  fw_minor
uint8  fw_patch
uint16 counts_per_wheel_rev
uint16 track_width_mm           # firmware calls this wheel_base; it is the track
