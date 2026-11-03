# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Boston cameras and USB leader; thermal integration remains a separate prerequisite."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch = Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw")) / "launch"
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch / name)))
            for name in ("cameras.launch.py", "rebot_leader.launch.py")
        ]
    )
