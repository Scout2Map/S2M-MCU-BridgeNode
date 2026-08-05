import os
from glob import glob

from setuptools import find_packages, setup

package_name = "scout2map_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="jihoonkimtech",
    maintainer_email="jihoonkimtech@naver.com",
    description="Serial bridge between the Pico 2 sensor fusion MCU and ROS 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pico_bridge = scout2map_bridge.pico_bridge_node:main",
            "fake_sensors = scout2map_bridge.fake_sensor_node:main",
        ],
    },
)
