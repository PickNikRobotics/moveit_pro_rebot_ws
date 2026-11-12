# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Boston scene camera plus the inherited stock drivers (wrist camera, optional leader)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# This is the only Gemini 305 whose office mount has been hand-eye calibrated,
# rather than eye-measured. The stock launch retains factory camera_info.
OFFICE_WRIST_CAMERA_SERIAL = "CV2F861000FP"


def generate_launch_description():
    office_launch = (
        Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw")) / "launch"
    )
    stock_launch = Path(get_package_share_directory("rebot_b601_rs_hw")) / "launch"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(stock_launch / "rebot_drivers.launch.py")
                ),
                # Pass the per-unit selection to the inherited stock wrist launch.
                launch_arguments={
                    "wrist_camera_serial_number": OFFICE_WRIST_CAMERA_SERIAL,
                }.items(),
            ),
        ]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(office_launch / "cameras.launch.py"))
            )
        ]
    )
