# STM32 bring-up diagnostics, decoded from the DIAG frame.
# Only published in response to a request on the drive/diagnostics service,
# so this topic is silent during normal operation.
#
# The point of this frame is to tell an absent sensor apart from a stuck bus
# apart from a wrong address, none of which are distinguishable from the
# scaled values alone.

std_msgs/Header header

# --- BNO055 ---
uint8  imu_init_step        # where the init state machine stopped
uint8  imu_chip_id          # 0xA0 when a BNO055 answered, 0xFF when nothing did
uint8  calib_sys
uint8  calib_gyro
uint8  calib_accel
uint8  calib_mag
uint32 imu_read_ok
uint32 imu_read_fail

# --- I2C bus ---
uint16 i2c_errors
uint16 i2c_recoveries       # times the bus had to be manually clocked free

# --- Battery ADC ---
# Raw counts are what recalibration needs: the constant being calibrated
# must not appear on both sides of the calculation.
uint16 battery_counts
uint16 battery_mv

# --- Distance sensor ADC ---
uint16 distance_counts      # raw, before the response curve
uint16 distance_mv          # after the median filter
