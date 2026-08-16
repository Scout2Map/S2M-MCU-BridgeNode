#!/usr/bin/env python3
# File   : bringup.launch.py
# Purpose: Launch both MCU bridges together. This is the normal entry point
#          on the robot; the per-node launch files are for bring-up and
#          debugging one board at a time.

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
            condition=IfCondition(LaunchConfiguration("drive")),
        ),
    ]

    return LaunchDescription(args + nodes)
