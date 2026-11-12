# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Optional Star Arm leader bridge, selected by the merged config YAML.

Off unless ros_global_params.rebot_leader_enabled is true; then it starts one
bridge (which owns one USB reader) and nothing else. The physical driver launch
separately includes the stock thermal guard; this module never starts one.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import get_config_package, load_system_config


PACKAGE = "rebot_b601_rs_hw"
DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
DEFAULT_CALIBRATION = "config/leader_calibration/rebot_arm_102_leader.json"
# Engage caps proven on a bench unit; direct following stays uncapped afterwards.
DEFAULT_ENGAGE_SPEED_RAD_S = 0.2
DEFAULT_ENGAGE_JAW_SPEED_RAD_S = 1.0


def srdf_file(config):
    """Installed SRDF of the selected config, for the bridge's collision model."""
    srdf = config.hardware.robot_description.srdf
    path = Path(get_package_share_directory(srdf.package)) / srdf.path
    if not path.is_file():
        raise ValueError(f"Leader collision SRDF not found: {path}")
    return path


def startup_settings(config):
    """Return plain startup values from merged YAML without starting processes."""
    params = config.ros_global_params or {}
    enabled = params.get("rebot_leader_enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("rebot_leader_enabled must be a YAML boolean")
    urdf_params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    mode = urdf_params.get("hardware_interface", "mock")
    if mode not in ("mock", "real", "mujoco"):
        raise ValueError(f"Unsupported reBot hardware_interface: {mode}")
    settings = {
        "hardware_interface": mode,
        "thermal_guard": False,
        "bridge": None,
    }
    if enabled:
        calibration = Path(
            params.get("rebot_leader_calibration_file", DEFAULT_CALIBRATION)
        )
        if not calibration.is_absolute():
            calibration = Path(get_package_share_directory(PACKAGE)) / calibration
        settings["bridge"] = {
            "publish_rate": float(params.get("rebot_leader_publish_rate", 100.0)),
            "engage_speed_rad_s": float(
                params.get(
                    "rebot_leader_engage_speed_rad_s", DEFAULT_ENGAGE_SPEED_RAD_S
                )
            ),
            "engage_jaw_speed_rad_s": float(
                params.get(
                    "rebot_leader_engage_jaw_speed_rad_s",
                    DEFAULT_ENGAGE_JAW_SPEED_RAD_S,
                )
            ),
            "engage_max_offset_rad": float(
                params.get("rebot_leader_engage_max_offset_rad", 0.6)
            ),
            "arguments": [
                "--port",
                str(params.get("rebot_leader_port", DEFAULT_PORT)),
                "--calibration-file",
                str(calibration),
                "--srdf-file",
                str(srdf_file(config)),
            ],
        }
    return settings


def configured_nodes(config):
    """Create the backend-owned processes selected by the startup contract."""
    settings = startup_settings(config)
    nodes = []
    if settings["bridge"] is not None:
        bridge = settings["bridge"]
        nodes.append(
            Node(
                package=PACKAGE,
                executable="rebot_leader_bridge.py",
                name="rebot_leader_bridge",
                output="screen",
                # Leave enough time for the existing native HOME recovery.
                sigterm_timeout="130",
                parameters=[
                    {
                        name: bridge[name]
                        for name in (
                            "publish_rate",
                            "engage_speed_rad_s",
                            "engage_jaw_speed_rad_s",
                            "engage_max_offset_rad",
                        )
                    }
                ],
                arguments=bridge["arguments"],
            )
        )
    return nodes


def launch_configured_nodes(context):
    del context
    config, _, _ = load_system_config(get_config_package(), Path(os.environ["USER_WS"]))
    return configured_nodes(config)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_configured_nodes)])
