# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Static contract for teleoperation and gripper control on the mock backend.

The live sequence is test/mock_teleop_smoke.py; these checks need no runtime.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

SHARE = Path(get_package_share_directory("rebot_b601_rs_mock"))


def load(path):
    return yaml.safe_load((SHARE / path).read_text())


@pytest.fixture(scope="module")
def urdf():
    return ET.fromstring(
        xacro.process_file(
            str(SHARE / "description/rebot.urdf.xacro"),
            mappings={"hardware_interface": "mock"},
        ).toxml()
    )


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(SHARE / "config/moveit/rebot.srdf").getroot()


def test_jog_groups_map_to_distinct_velocity_controllers():
    joint = load("config/moveit/joint_jog.yaml")
    pose = load("config/moveit/pose_jog.yaml")
    params = load("config/control/rebot.ros2_control.yaml")
    assert joint["planning_groups"] == ["manipulator", "gripper"]
    assert joint["controllers"] == [
        "joint_velocity_controller",
        "gripper_joint_velocity_controller",
    ]
    assert pose == {
        "planning_groups": ["manipulator"],
        "controllers": ["velocity_force_controller"],
    }
    for group, controller in zip(joint["planning_groups"], joint["controllers"]):
        assert params[controller]["ros__parameters"]["planning_group_name"] == group
    assert (
        params["velocity_force_controller"]["ros__parameters"]["planning_group_name"]
        == "manipulator"
    )


def test_velocity_controllers_command_position_and_start_inactive():
    params = load("config/control/rebot.ros2_control.yaml")
    for name in (
        "joint_velocity_controller",
        "gripper_joint_velocity_controller",
        "velocity_force_controller",
    ):
        assert params[name]["ros__parameters"]["command_interfaces"] == ["position"]
    config = load("config/config.yaml")["ros2_control"]
    assert config["controllers_active_at_startup"] == [
        "joint_state_broadcaster",
        "joint_trajectory_admittance_controller",
    ]
    assert set(config["controllers_inactive_at_startup"]) == {
        "joint_velocity_controller",
        "gripper_joint_velocity_controller",
        "velocity_force_controller",
    }


def test_gripper_group_commands_one_motor_and_fingers_are_mimics(urdf, srdf):
    group = srdf.find("group[@name='gripper']")
    assert [j.get("name") for j in group.findall("joint")] == [
        "joint7",
        "gripper_joint1",
        "gripper_joint2",
    ]
    params = load("config/control/rebot.ros2_control.yaml")
    assert (
        len(
            params["gripper_joint_velocity_controller"]["ros__parameters"][
                "max_joint_velocity"
            ]
        )
        == 1
    )
    commanded = {
        j.get("name")
        for j in urdf.findall("ros2_control/joint")
        if j.find("command_interface") is not None
    }
    assert commanded == {f"joint{i}" for i in range(1, 8)}
    for finger in ("gripper_joint1", "gripper_joint2"):
        mimic = urdf.find(f"joint[@name='{finger}']/mimic")
        assert mimic.get("joint") == "joint7"
        assert float(mimic.get("multiplier")) == pytest.approx(0.008)


def test_trajectory_controller_owns_arm_and_jaw_for_partial_goals():
    params = load("config/control/rebot.ros2_control.yaml")
    assert (
        params["joint_trajectory_admittance_controller"]["ros__parameters"][
            "planning_group_name"
        ]
        == "manipulator_and_gripper"
    )


@pytest.mark.parametrize(
    "objective,waypoint",
    [("open_gripper", "Gripper Open"), ("close_gripper", "Gripper Closed")],
)
def test_gripper_objectives_move_only_the_gripper_group(objective, waypoint):
    root = ET.parse(SHARE / f"objectives/{objective}.xml").getroot()
    step = root.find(".//SubTree[@ID='Move to Named Waypoint']")
    assert step.get("waypoint_name") == waypoint
    assert step.get("joint_group_name") == "gripper"
    assert step.get("controller_names") == "joint_trajectory_admittance_controller"


def test_teleoperate_uses_trajectory_controller_for_moves():
    root = ET.parse(SHARE / "objectives/teleoperate.xml").getroot()
    sub = root.find(".//SubTree[@ID='Request Teleoperation']")
    assert (
        sub.get("joint_trajectory_controller_name")
        == "joint_trajectory_admittance_controller"
    )
    assert sub.get("execution_pipeline") == "jtac"
