#!/usr/bin/env python3
# File   : bringup.launch.py
# Purpose: Launch both MCU bridges together. This is the normal entry point
#          on the robot; the per-node launch files are for bring-up and
#          debugging one board at a time.
#
# Note: drive_bridge publishes odometry and IMU under its own names,
# drive/odom and drive/imu. scout2map_event's default parameters (and
# robot_localization's usual defaults) expect /odom and /imu/data instead.
# S2M-SBC-Integration's s2m_onboard_bridge.launch.py applies that remap for
# you; this launch file leaves it off by default (odom_topic/imu_topic
# default to the bridge's own names, i.e. no remap) so standalone bring-up
# is unaffected, but exposes the same launch args here so this file can be
# used with the event engine too, instead of the gap being silent -- no
# error, just an event type that never fires because nothing is publishing
# on the topic it listens to.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("scout2map_bridge")
    sensor_params = os.path.join(pkg_share, "config", "sensor_bridge.yaml")
    drive_params = os.path.join(pkg_share, "config", "drive_bridge.yaml")

    args = [
        DeclareLaunchArgument(
            "sensors", default_value="true",
            description="Start the Pico 2 sensor fusion bridge"),
        DeclareLaunchArgument(
            "drive", default_value="true",
            description="Start the STM32 drive control bridge"),
        DeclareLaunchArgument(
            "sensor_params_file", default_value=sensor_params),
        DeclareLaunchArgument(
            "drive_params_file", default_value=drive_params),
        DeclareLaunchArgument(
            "odom_topic", default_value="drive/odom",
            description="Where drive_bridge odometry is remapped to. Set "
                        "to /odom (or wherever robot_localization/Nav2 "
                        "expect it) when running this file without "
                        "s2m_onboard_bridge.launch.py."),
        DeclareLaunchArgument(
            "imu_topic", default_value="drive/imu",
            description="Where drive_bridge Imu is remapped to. Set to "
                        "/imu/data to match scout2map_event's default "
                        "imu_topic when running the event engine off this "
                        "launch file directly."),
    ]

    nodes = [
        Node(
            package="scout2map_bridge",
            executable="sensor_bridge",
            name="sensor_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("sensor_params_file")],
            condition=IfCondition(LaunchConfiguration("sensors")),
        ),
        Node(
            package="scout2map_bridge",
            executable="drive_bridge",
            name="drive_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("drive_params_file")],
            remappings=[
                ("drive/odom", LaunchConfiguration("odom_topic")),
                ("drive/imu", LaunchConfiguration("imu_topic")),
            ],
            condition=IfCondition(LaunchConfiguration("drive")),
        ),
    ]

    return LaunchDescription(args + nodes)
