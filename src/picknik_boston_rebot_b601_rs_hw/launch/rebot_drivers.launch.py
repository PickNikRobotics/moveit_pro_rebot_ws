# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Boston scene camera plus the inherited stock drivers (wrist camera, optional leader)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    office_launch = (
        Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw")) / "launch"
    )
    stock_launch = Path(get_package_share_directory("rebot_b601_rs_hw")) / "launch"
    return LaunchDescription(
        [
            # The stock wrist launch selects the first attached Gemini 305; pass
            # wrist_camera_serial_number here to pin a calibrated unit.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(stock_launch / "rebot_drivers.launch.py")
                )
            ),
        ]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(office_launch / "cameras.launch.py"))
            )
        ]
    )
