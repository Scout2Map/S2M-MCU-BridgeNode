#!/usr/bin/env python3
# File   : drive_bridge.launch.py
# Purpose: Launch the STM32 drive controller bridge.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("scout2map_bridge")
    default_params = os.path.join(pkg_share, "config", "drive_bridge.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="YAML file with drive_bridge parameters")

    bridge = Node(
        package="scout2map_bridge",
        executable="drive_bridge",
        name="drive_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, bridge])
