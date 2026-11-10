#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Add an isolated native-jaw contact fixture to the generated stock model.

Reuse the separately modeled PLA pads, not a convex hull spanning each pad and
carriage. The block is fixed (mocap), parked away from the robot at default, and
placed between the open jaws only by the contact_smoke keyframe. No grasp weld,
bench calibration, camera, or office geometry is imported.
"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
import mujoco
import yaml


PARKED = "0.7 -0.5 0.05"


def add_fixture(scene_path, waypoints_path):
    root = ET.parse(
        scene_path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    ).getroot()
    root.insert(
        1,
        ET.Comment(
            " PickNik Inc., 2026: isolated native-pad contact smoke fixture; no grasp weld. "
        ),
    )
    assets = {mesh.get("name"): mesh.get("file") for mesh in root.findall("asset/mesh")}
    for side in ("left", "right"):
        finger = root.find(f".//body[@name='gripper_{side}']")
        pad = next(
            geom
            for geom in finger.findall("geom")
            if assets[geom.get("mesh")].endswith(f"/meshes/visual/pla_{side}.STL")
        )
        # Retain the native pad frame, separately from the carriage. Do not
        # rely on the source URDF's visual element order to select the pad.
        ET.SubElement(
            finger,
            "geom",
            name=f"{side}_pad_contact",
            type="mesh",
            mesh=pad.get("mesh"),
            pos=pad.get("pos", "0 0 0"),
            euler=pad.get("euler", "0 0 0"),
            density="0",
            group="3",
            contype="1",
            conaffinity="2",
            friction="1 0.005 0.0001",
            solimp="0.999 0.999 0.001",
        )
    block = ET.SubElement(
        root.find("worldbody"), "body", name="contact_block", mocap="true", pos=PARKED
    )
    ET.SubElement(
        block,
        "geom",
        name="contact_block_geom",
        type="box",
        size="0.025 0.025 0.025",
        contype="2",
        conaffinity="1",
        rgba="0.1 0.3 0.8 1",
        solimp="0.999 0.999 0.001",
    )
    default = root.find("keyframe/key[@name='default']")
    default.set("mpos", PARKED)
    default.set("mquat", "1 0 0 0")

    # Use the actual compiled kinematics, including the source's rounded RPY,
    # to express the 50 mm block in the stock gripper_end frame.
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    opened = next(
        waypoint["joint_state"]
        for waypoint in yaml.safe_load(Path(waypoints_path).read_text())
        if waypoint["name"] == "Gripper Open"
    )
    positions = dict(zip(opened["name"], opened["position"], strict=True))
    for name, value in positions.items():
        data.joint(name).qpos = value
        data.ctrl[model.actuator(name).id] = value
    for name in ("gripper_joint1", "gripper_joint2"):
        data.joint(name).qpos = 0.008 * positions["joint7"]
    mujoco.mj_forward(model, data)
    tip = data.body("gripper_end")
    center = tip.xpos - 0.01 * tip.xmat.reshape(3, 3)[:, 0]
    ET.SubElement(
        root.find("keyframe"),
        "key",
        name="contact_smoke",
        qpos=" ".join(map(str, data.qpos)),
        ctrl=" ".join(map(str, data.ctrl)),
        mpos=" ".join(map(str, center)),
        mquat=" ".join(map(str, tip.xquat)),
    )
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(scene_path, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    add_fixture(*sys.argv[1:])
