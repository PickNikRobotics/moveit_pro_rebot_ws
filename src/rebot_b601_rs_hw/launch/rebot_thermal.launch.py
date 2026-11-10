# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Persistent thermal recovery for the selected physical backend only.

Extracted from the current example workspace's rebot_leader.launch.py;
leader startup and calibrated camera mounts remain office-specific.
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import get_config_package, load_system_config


def configured_nodes(config):
    """Construct launch actions without executing a driver or guard."""
    urdf_params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    mode = urdf_params.get("hardware_interface", "mock")
    if mode not in ("mock", "real", "mujoco"):
        raise ValueError(f"Unsupported reBot hardware_interface: {mode}")
    if mode != "real":
        return []
    return [
        Node(
            package="rebot_b601_rs_hw",
            executable="rebot_thermal_guard.py",
            name="rebot_thermal_guard",
            output="screen",
        )
    ]


def launch_configured_nodes(context):
    del context
    config, _, _ = load_system_config(get_config_package(), Path(os.environ["USER_WS"]))
    return configured_nodes(config)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_configured_nodes)])
