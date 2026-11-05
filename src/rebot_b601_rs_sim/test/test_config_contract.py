"""Resolve inheritance with the installed MoveIt Pro loader; open no hardware."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
from moveit_studio_utils_py.system_config import load_system_config
import xacro
import yaml


def resolved(mode):
    package = f"rebot_b601_rs_{mode}"
    return load_system_config(package, Path(get_package_share_directory(package)))


def expanded(config):
    description = config.hardware.robot_description
    params = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for entry in description.urdf_params
        for key, value in entry.items()
    }
    path = (
        Path(get_package_share_directory(description.urdf.package))
        / description.urdf.path
    )
    return ET.fromstring(xacro.process_file(str(path), mappings=params).toxml())


@pytest.mark.parametrize(
    "mode,plugin",
    [
        ("mock", "mock_components/GenericSystem"),
        ("sim", "picknik_mujoco_ros/MujocoSystem"),
    ],
)
def test_explicit_selector_never_includes_real_transport(mode, plugin):
    config, _, chain = resolved(mode)
    robot = expanded(config)
    assert chain == (
        ["rebot_b601_rs_mock"]
        if mode == "mock"
        else ["rebot_b601_rs_mock", "rebot_b601_rs_sim"]
    )
    assert [node.text for node in robot.findall("ros2_control/hardware/plugin")] == [
        plugin
    ]
    assert robot.findall("ros2_control/gpio") == []
    assert robot.find(".//param[@name='can_interface']") is None
    assert robot.find(".//param[@name='torque_enable']") is None
    assert (
        config.hardware.additional_driver_launch_file.path == "launch/blank.launch.py"
    )
    assert config.ros_global_params["use_sim_time"] is False
    assert config.hardware.robot_description.urdf.package == "rebot_b601_rs_mock"
    assert config.hardware.robot_description.srdf.package == "rebot_b601_rs_mock"
    assert config.ros2_control["config"] == {
        "package": "rebot_b601_rs_mock",
        "path": "config/control/rebot.ros2_control.yaml",
    }
    assert config.ros2_control["controllers_active_at_startup"] == [
        "joint_state_broadcaster",
        "joint_trajectory_admittance_controller",
    ]
    selector = config.ros2_control["config"]
    controllers = yaml.safe_load(
        (
            Path(get_package_share_directory(selector["package"])) / selector["path"]
        ).read_text()
    )
    for name in config.ros2_control["controllers_inactive_at_startup"]:
        assert name in controllers
        assert name in controllers["controller_manager"]["ros__parameters"]
    assert robot.findall("ros2_control/sensor") == []


def test_installed_mujoco_resource_and_reset_are_simulation_only():
    config, _, _ = resolved("sim")
    robot = expanded(config)
    params = {
        node.get("name"): node.text
        for node in robot.findall("ros2_control/hardware/param")
    }
    assert params["mujoco_model_package"] == "rebot_b601_rs_sim"
    assert params["mujoco_model"] == "description/scene.xml"
    assert (
        Path(get_package_share_directory(params["mujoco_model_package"]))
        / params["mujoco_model"]
    ).is_file()
    assert params["mujoco_keyframe"] == "default"
    assert params["render_publish_rate"] == params["point_cloud_publish_rate"] == "0"
    assert config.objectives.behavior_loader_plugins["mujoco"] == [
        "moveit_pro::behaviors::MujocoBehaviorsLoader"
    ]
    assert "mujoco" not in resolved("mock")[0].objectives.behavior_loader_plugins
