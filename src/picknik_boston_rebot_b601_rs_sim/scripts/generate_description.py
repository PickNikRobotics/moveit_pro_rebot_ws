#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Freeze the nominal office URDF at the same layout as the generated MJCF.

This is a build-time description adapter, not a runtime robot or controller.
Only world/camera placement and the simulator backend are replaced. The office
wrapper still supplies the arm, jaw and trim; fitted hardware inertials cannot
enter this output.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml

XACRO = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", XACRO)


def generate(source, layout, output):
    mappings = {"hardware_interface": "mujoco", "boston_gravity_fit": "false"}
    for prefix, key in (
        ("world_to_base", "robot_base"),
        ("scene_camera", "scene_camera"),
        ("wrist_camera_mount", "wrist_camera_mount"),
    ):
        for field in ("xyz", "rpy"):
            mappings[f"{prefix}_{field}"] = " ".join(map(str, layout[key][field]))
    robot = ET.fromstring(xacro.process_file(str(source), mappings=mappings).toxml())
    systems = robot.findall("ros2_control")
    if len(systems) != 1 or systems[0].findtext("hardware/plugin") != (
        "picknik_mujoco_ros/MujocoSystem"
    ):
        raise ValueError("Expected exactly one nominal office MuJoCo backend")
    robot.remove(systems[0])
    ET.SubElement(
        robot,
        f"{{{XACRO}}}include",
        filename="mujoco_control.xacro",
    )
    ET.indent(robot)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(output, encoding="unicode", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("layout", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    generate(args.source, yaml.safe_load(args.layout.read_text()), args.output)


if __name__ == "__main__":
    main()
