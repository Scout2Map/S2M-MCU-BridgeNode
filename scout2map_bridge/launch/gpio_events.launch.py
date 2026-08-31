#!/usr/bin/env python3
# File   : gpio_events.launch.py
# Purpose: Launch the event-to-GPIO output node on its own, for bring-up and
#          debugging one board at a time.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("scout2map_bridge")
    default_params = os.path.join(pkg_share, "config", "gpio_events.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="YAML file with gpio_events parameters")

    node = Node(
        package="scout2map_bridge",
        executable="gpio_events",
        name="gpio_events",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, node])
