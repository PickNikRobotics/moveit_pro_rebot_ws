"""Consume the configured launch description without executing its Node actions."""

import copy
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
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
