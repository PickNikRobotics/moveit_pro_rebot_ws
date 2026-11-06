#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Expand the real public hardware contract with a virtual-only bus override."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.system_config import load_system_config
import xacro


def prepare(source_share, destination):
    """Resolve inheritance, retain the real plugin, and override only the bus."""
    package = "rebot_b601_rs_hw"
    share = Path(source_share)
    config = load_system_config(package, share)[0]
    description = config.hardware.robot_description
    params = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for entry in description.urdf_params
        for key, value in entry.items()
    }
    params["can_interface"] = "vcan0"
    source = (
        Path(get_package_share_directory(description.urdf.package))
        / description.urdf.path
    )
    robot = ET.fromstring(xacro.process_file(str(source), mappings=params).toxml())
    assert robot.findtext("ros2_control/hardware/plugin") == (
        "robstride_hardware_interface/RobstrideHardware"
    )
    assert [
        gpio.findtext("param[@name='can_interface']")
        for gpio in robot.findall("ros2_control/gpio")
    ] == ["vcan0"] * 7
    ET.ElementTree(robot).write(destination, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    prepare(*sys.argv[1:])
