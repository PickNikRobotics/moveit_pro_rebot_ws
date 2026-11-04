#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Build a test-only memory backend with the physical command-interface contract.

This never loads the RobStride driver. GenericSystem command propagation is
explicitly disabled so an old commanded pose cannot masquerade as measurement.
The nominal stock model, payload and production controllers are not replaced.
"""
import sys
import xml.etree.ElementTree as ET

import xacro


def prepare(source, destination):
    robot = ET.fromstring(
        xacro.process_file(
            source,
            mappings={
                "hardware_interface": "real",
                "arm_control_mode": "motion",
                "arm_effort_command": "true",
            },
        ).toxml()
    )
    control = robot.find("ros2_control")
    hardware = control.find("hardware")
    assert (
        hardware.findtext("plugin") == "robstride_hardware_interface/RobstrideHardware"
    )
    hardware.clear()
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    ET.SubElement(hardware, "param", name="disable_commands").text = "true"
    for gpio in control.findall("gpio"):
        control.remove(gpio)
    assert [
        joint.get("name")
        for joint in control.findall("joint")
        if joint.find("command_interface[@name='effort']") is not None
    ] == [f"joint{i}" for i in range(1, 7)]
    ET.ElementTree(robot).write(destination, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    prepare(*sys.argv[1:])
