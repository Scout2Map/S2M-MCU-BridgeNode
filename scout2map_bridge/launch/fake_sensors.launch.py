#!/usr/bin/env python3
# File   : fake_sensors.launch.py
# Purpose: Launch the synthetic sensor publisher for hardware-free development.
#          Pass a scenario to shape the values, e.g. scenario:=gas_leak

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario_arg = DeclareLaunchArgument(
        "scenario", default_value="normal",
        description="normal | gas_leak | high_temp | low_light | dust_storm | "
                    "warmup | sensor_dropout | link_loss")

    ramp_arg = DeclareLaunchArgument(
        "ramp_seconds", default_value="30.0",
        description="How long a scenario takes to reach its peak value")

    noise_arg = DeclareLaunchArgument(
        "noise", default_value="1.0",
        description="Jitter multiplier, 0.0 gives perfectly clean values")

    fake = Node(
        package="scout2map_bridge",
        executable="fake_sensors",
        name="fake_sensors",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "scenario": LaunchConfiguration("scenario"),
            "ramp_seconds": LaunchConfiguration("ramp_seconds"),
            "noise": LaunchConfiguration("noise"),
        }],
    )

    return LaunchDescription([scenario_arg, ramp_arg, noise_arg, fake])
