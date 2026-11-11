# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise the stock model with MoveIt Pro's model and FCL consumers.

Adapted from moveit_pro_example_ws at c7422c8853e495f0e481daa639c2919be24d73f9:
stock mesh package, shared model/MoveIt bounds, and no office camera frames.
"""
from math import radians
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_pro_base import planning_scene, robot_model, robot_state
import numpy as np
import pytest
import yaml
import xacro


@pytest.fixture(scope="module")
def model(tmp_path_factory):
    share = Path(get_package_share_directory("rebot_b601_rs_mock"))
    urdf = tmp_path_factory.mktemp("description") / "rebot.urdf"
    urdf.write_text(
        xacro.process_file(
            str(share / "description/rebot.urdf.xacro"),
            mappings={"hardware_interface": "mock"},
        ).toxml()
    )
    return robot_model.RobotModel(str(urdf), str(share / "config/moveit/rebot.srdf"))


def test_motor_limits_and_native_mimics(model):
    low, high = model.get_group_position_bounds("manipulator")
    share = Path(get_package_share_directory("rebot_b601_rs_mock"))
    limits = yaml.safe_load((share / "config/moveit/joint_limits.yaml").read_text())[
        "joint_limits"
    ]
    np.testing.assert_allclose(
        low,
        [
            radians(-145),
            limits["joint2"]["min_position"],
            limits["joint3"]["min_position"],
            radians(-106),
            radians(-101),
            radians(-175),
        ],
    )
    np.testing.assert_allclose(high, np.radians([145, 203, 239, 86, 108, 175]))
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    state.joint_positions = {"joint7": radians(310)}
    state.update()
    for joint in ["gripper_joint1", "gripper_joint2"]:
        assert state.joint_positions[joint] == pytest.approx(0.008 * radians(310))
    low, high = model.get_group_position_bounds("gripper")
    assert high[-1] == pytest.approx(radians(310))
    assert high[-1] >= radians(310)
    assert low[-1] == pytest.approx(-0.15)


def test_closed_named_state_and_close_waypoint_do_not_command_extra_squeeze(model):
    state = robot_state.RobotState(model)
    state.set_to_default_values("gripper", "gripper_closed")
    assert state.joint_positions["joint7"] == 0
    assert waypoints()["Gripper Closed"]["joint7"] == 0
    assert waypoints()["Gripper Open"]["joint7"] == pytest.approx(radians(310))


def waypoints():
    share = Path(get_package_share_directory("rebot_b601_rs_mock"))
    saved = yaml.safe_load((share / "waypoints/rebot_waypoints.yaml").read_text())
    return {
        w["name"]: dict(zip(w["joint_state"]["name"], w["joint_state"]["position"]))
        for w in saved
    }


def test_waypoints_and_interpolated_moves_are_collision_free(model):
    poses = waypoints()
    scene = planning_scene.PlanningScene(model)
    # Check the actual unpadded geometry even for these extra adjacency exclusions.
    scene.allowed_collision_matrix.set_entry("link5", "gripper_end", False)
    scene.allowed_collision_matrix.set_entry("gripper_left", "gripper_right", False)
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    for start, goal in [("Zeroed Rest", "Raised"), ("Raised", "Gripper Open")]:
        for fraction in np.linspace(0, 1, 51):
            state.joint_positions = {
                joint: value + fraction * (poses[goal][joint] - value)
                for joint, value in poses[start].items()
            }
            state.update()
            assert scene.is_state_valid(state, "", False), (start, goal, fraction)
    state.set_to_default_values("manipulator", "zeroed_rest")
    state.update()
    rest_z = state.get_global_link_transform("gripper_end")[2, 3]
    state.set_to_default_values("manipulator", "raised")
    state.update()
    assert state.get_global_link_transform("gripper_end")[2, 3] > rest_z + 0.2


def vertices(relative_path):
    share = Path(get_package_share_directory("rebot_description"))
    triangle = np.dtype(
        [("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    return np.fromfile(share / relative_path, dtype=triangle, offset=84)[
        "vertices"
    ].reshape(-1, 3)


def test_wrist_mesh_clearance_across_roll(model):
    meshes = {
        name: vertices(f"meshes/shared/{name}.STL") for name in ["link5", "gripper_end"]
    }
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    for roll in np.radians([-175, 0, 175]):
        state.joint_positions = {"joint6": roll}
        state.update()
        wrist_from_world = np.linalg.inv(state.get_global_link_transform("link6"))
        extents = {}
        for name, points in meshes.items():
            transform = wrist_from_world @ state.get_global_link_transform(name)
            transformed = points @ transform[:3, :3].T + transform[:3, 3]
            extents[name] = (transformed[:, 2].min(), transformed[:, 2].max())
        gap = extents["gripper_end"][0] - extents["link5"][1]
        assert 0.0089 < gap < 0.0091


def test_opening_increases_native_jaw_separation(model):
    poses = waypoints()
    fingers = {
        side: vertices(f"meshes/visual/pla_{side}.STL") for side in ["left", "right"]
    }
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    gaps = []
    closed, opened = poses["Gripper Closed"], poses["Gripper Open"]
    for fraction in np.linspace(0, 1, 21):
        state.joint_positions = {
            joint: value + fraction * (opened[joint] - value)
            for joint, value in closed.items()
        }
        state.update()
        gripper_from_world = np.linalg.inv(
            state.get_global_link_transform("gripper_end")
        )
        lateral = {}
        for side, points in fingers.items():
            transform = gripper_from_world @ state.get_global_link_transform(
                f"gripper_{side}"
            )
            lateral[side] = (points @ transform[:3, :3].T + transform[:3, 3])[:, 1]
        gaps.append(lateral["right"].min() - lateral["left"].max())
    assert 0 <= gaps[0] < 0.0002
    assert 0.086 < gaps[-1] < 0.087
    assert np.all(np.diff(gaps) > 0), gaps
    assert gaps[-1] - gaps[0] == pytest.approx(0.016 * radians(310), abs=1e-8)
