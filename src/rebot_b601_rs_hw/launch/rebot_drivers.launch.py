# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Persistent physical-backend helpers, including the wrist camera."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


LAUNCH_FILES = ("rebot_thermal.launch.py", "wrist_camera.launch.py")


def generate_launch_description():
    launch = Path(get_package_share_directory("rebot_b601_rs_hw")) / "launch"
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch / name)))
            for name in LAUNCH_FILES
        ]
    )
