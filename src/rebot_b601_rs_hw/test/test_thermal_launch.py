"""Consume the configured launch description without executing its Node actions."""

import copy
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from moveit_studio_utils_py.system_config import load_system_config
import pytest


@pytest.fixture(scope="module")
def thermal_launch():
    path = Path(__file__).resolve().parents[1] / "launch/rebot_thermal.launch.py"
    spec = importlib.util.spec_from_file_location("stock_thermal_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolved(mode):
    package = f"rebot_b601_rs_{mode}"
    return load_system_config(package, Path(get_package_share_directory(package)))[0]


@pytest.mark.parametrize("mode", ["mock", "sim", "hw"])
def test_only_real_backend_constructs_one_guard(thermal_launch, mode):
    nodes = thermal_launch.configured_nodes(resolved(mode))
    if mode != "hw":
        assert nodes == []
        return
    assert len(nodes) == 1
    node = nodes[0]
    assert isinstance(node, Node)
    context = LaunchContext()
    assert (
        perform_substitutions(
            context, normalize_to_list_of_substitutions(node.node_package)
        )
        == "rebot_b601_rs_hw"
    )
    assert (
        perform_substitutions(
            context, normalize_to_list_of_substitutions(node.node_executable)
        )
        == "rebot_thermal_guard.py"
    )


def test_invalid_backend_fails_without_launch(thermal_launch):
    config = copy.deepcopy(resolved("hw"))
    config.hardware.robot_description.urdf_params = [{"hardware_interface": "invalid"}]
    with pytest.raises(ValueError, match="Unsupported reBot hardware_interface"):
        thermal_launch.configured_nodes(config)


def test_driver_launch_includes_thermal_guard_and_wrist_camera():
    path = Path(__file__).resolve().parents[1] / "launch/rebot_drivers.launch.py"
    spec = importlib.util.spec_from_file_location("stock_driver_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    launch = module.generate_launch_description()
    assert len(launch.entities) == 2
    assert all(
        isinstance(action, IncludeLaunchDescription) for action in launch.entities
    )
    context = LaunchContext()
    child_descriptions = [
        action.launch_description_source.get_launch_description(context)
        for action in launch.entities
    ]
    included_files = {
        Path(action.launch_description_source.location).name
        for action in launch.entities
    }
    assert included_files == set(module.LAUNCH_FILES)
    wrist_description = next(
        child
        for child in child_descriptions
        if any(
            isinstance(entity, DeclareLaunchArgument)
            and entity.name == "enable_wrist_camera"
            for entity in child.entities
        )
    )
    assert [
        argument.name
        for argument in wrist_description.entities
        if isinstance(argument, DeclareLaunchArgument)
    ] == [
        "enable_wrist_camera",
        "wrist_camera_serial_number",
        "wrist_camera_color_width",
        "wrist_camera_color_height",
        "wrist_camera_color_fps",
    ]
    assert any(
        isinstance(entity, OpaqueFunction) for entity in wrist_description.entities
    )
    thermal_description = next(
        child for child in child_descriptions if child is not wrist_description
    )
    assert any(
        isinstance(entity, OpaqueFunction) for entity in thermal_description.entities
    )


def test_launch_evaluates_selected_config_without_starting_node(
    thermal_launch, monkeypatch
):
    monkeypatch.setattr(
        thermal_launch, "get_config_package", lambda: "rebot_b601_rs_hw"
    )
    monkeypatch.setattr(
        thermal_launch, "load_system_config", lambda *_: (resolved("hw"), None, None)
    )
    monkeypatch.setenv("USER_WS", "/unused-source-selection")
    launch = thermal_launch.generate_launch_description()
    assert len(launch.entities) == 1
    action = launch.entities[0]
    assert isinstance(action, OpaqueFunction)
    nodes = action.execute(LaunchContext())
    assert len(nodes) == 1
    assert isinstance(nodes[0], Node)
    # Deliberately do not execute the returned Node action.
