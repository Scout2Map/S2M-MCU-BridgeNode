#!/usr/bin/env python3
#
# File   : skid_calib.py
# Purpose: Measure the effective track width of the skid steer chassis by
#          comparing encoder derived yaw rate against the IMU while the
#          operator rotates the robot.
# Author : jihoonkimtech
#
# Why this is needed
#   On a differential drive robot the wheels roll cleanly through a turn, so
#   yaw rate is (v_r - v_l) / track. This chassis has four wheels on a 140mm
#   wheelbase, and turning drags all four sideways. The chassis therefore
#   rotates LESS than the geometry predicts, and the correction is a property
#   of the tyres and the floor rather than something derivable on paper.
#
#   Without the correction the encoder estimate over predicts yaw on every
#   turn, and the slip detector fires on ordinary steering.
#
# This tool does not command motion. Driving a robot from a calibration
# script is a good way to have it drive off a bench, so the operator stays
# in control and this only listens.

import argparse
import math
import sys

import rclpy
from rclpy.node import Node

from scout2map_msgs.msg import DriveStatus


class SkidCalib(Node):
    """Integrates both yaw estimates and reports their ratio."""

    def __init__(self, min_rate, settle_s):
        super().__init__("skid_calib")
        self._min_rate = min_rate
        self._settle_s = settle_s

        self._enc_angle = 0.0       # integrated encoder yaw, radians
        self._imu_angle = 0.0       # integrated IMU yaw, radians
        self._samples = 0
        self._rotating_samples = 0
        self._last_stamp = None
        self._warned_openloop = False
        self._warned_calib = False

        self.create_subscription(DriveStatus, "drive/status", self._on_status, 50)
        self.create_timer(1.0, self._report)

        print("skid_calib: measure the effective track width of the chassis")
        print()
        print("Rotate the robot in place, both directions, for 20-30 seconds.")
        print("Use teleop or a steady angular command. Keep it on the surface")
        print("you actually drive on: carpet and concrete give different numbers.")
        print()
        print("Ctrl+C when the estimate stops moving.")
        print()

    def _on_status(self, msg: DriveStatus):
        if msg.openloop:
            if not self._warned_openloop:
                self._warned_openloop = True
                print("  [warn] encoders unavailable, samples are being ignored")
            return
        if not msg.imu_calibrated:
            if not self._warned_calib:
                self._warned_calib = True
                print("  [warn] IMU not calibrated yet. Move the chassis in a "
                      "figure eight until calib_mag reaches 3.")
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_stamp is None:
            self._last_stamp = stamp
            return
        dt = stamp - self._last_stamp
        self._last_stamp = stamp
        # Guard against a stalled or jumping clock
        if dt <= 0.0 or dt > 0.5:
            return

        self._samples += 1
        enc = msg.yaw_rate_encoder_radps
        imu = msg.yaw_rate_imu_radps

        # Only integrate while actually turning. Straight running contributes
        # nothing but noise to a ratio of two rotation rates.
        if abs(imu) < self._min_rate:
            return

        self._rotating_samples += 1
        self._enc_angle += enc * dt
        self._imu_angle += imu * dt

    def _report(self):
        if self._rotating_samples == 0:
            if self._samples > 0:
                print(f"  waiting for rotation "
                      f"(need |yaw| > {math.degrees(self._min_rate):.0f} deg/s)")
            return
        if abs(self._imu_angle) < 1e-6:
            return

        # Both directions cancel in the signed integral, which is deliberate:
        # a tyre that scrubs more one way than the other would otherwise hide
        # inside the average. Use absolute angle for the magnitude.
        ratio = self._enc_angle / self._imu_angle
        secs = self._rotating_samples * 0.02
        print(f"  rotating {secs:5.1f}s   "
              f"enc {math.degrees(self._enc_angle):8.1f} deg   "
              f"imu {math.degrees(self._imu_angle):8.1f} deg   "
              f"ratio {ratio:5.3f}")

    def summary(self):
        print()
        if self._rotating_samples < 100:
            print("Not enough rotation was captured to trust a number.")
            print(f"  rotating samples: {self._rotating_samples} (want a few hundred)")
            if self._samples == 0:
                print("  No DriveStatus arrived at all. Is drive_bridge running?")
            return 1
        if abs(self._imu_angle) < math.radians(180):
            print("Less than half a turn was captured; the estimate is fragile.")
            print(f"  IMU angle: {math.degrees(self._imu_angle):.1f} deg")

        ratio = self._enc_angle / self._imu_angle
        print("=" * 58)
        print(f"  encoder angle   {math.degrees(self._enc_angle):9.1f} deg")
        print(f"  IMU angle       {math.degrees(self._imu_angle):9.1f} deg")
        print(f"  samples         {self._rotating_samples:9d}")
        print()
        print(f"  skid_factor  =  {ratio:.3f}")
        print("=" * 58)
        print()

        if ratio < 1.05:
            print("  A value near 1.0 means the wheels barely scrubbed, which is")
            print("  unusual for a four wheel chassis. Check that the robot was")
            print("  turning in place on its real surface rather than lifted.")
        elif ratio > 1.8:
            print("  Higher than expected. Very slippery flooring, or a gyro bias")
            print("  that has not been subtracted, will both inflate this.")
        else:
            print("  This is the expected range for a four wheel skid steer.")
        print()
        print("  Put it in config/drive_bridge.yaml:")
        print(f"      skid_factor: {ratio:.3f}")
        print()
        print("  Re-measure after changing tyres, load, or driving surface.")
        return 0


def main():
    ap = argparse.ArgumentParser(
        description="Measure the skid steer effective track width correction.")
    ap.add_argument("--min-rate", type=float, default=0.15,
                    help="ignore samples below this yaw rate, rad/s")
    ap.add_argument("--settle", type=float, default=0.0,
                    help="seconds to discard at the start")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = SkidCalib(args.min_rate, args.settle)
    rc = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rc = node.summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
