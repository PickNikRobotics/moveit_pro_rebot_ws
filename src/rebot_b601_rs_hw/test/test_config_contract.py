"""Verify backend isolation and the shared stock-product semantic contract."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
from moveit_studio_utils_py.system_config import load_system_config


def resolved(mode):
    package = f"rebot_b601_rs_{mode}"
    # This only reads config resources; it never constructs the hardware plugin.
    return load_system_config(package, Path(get_package_share_directory(package)))[0]


def expanded(config):
    description = config.hardware.robot_description
    params = {
        k: str(v).lower() if isinstance(v, bool) else str(v)
        for entry in description.urdf_params
        for k, v in entry.items()
    }
    path = (
        Path(get_package_share_directory(description.urdf.package))
        / description.urdf.path
    )
    return ET.fromstring(xacro.process_file(str(path), mappings=params).toxml())


@pytest.mark.parametrize(
    "mode,plugin,sim_time",
    [
        ("mock", "mock_components/GenericSystem", False),
        ("sim", "picknik_mujoco_ros/MujocoSystem", True),
        ("hw", "robstride_hardware_interface/RobstrideHardware", False),
    ],
)
def test_backend_isolation(mode, plugin, sim_time):
    config = resolved(mode)
    robot = expanded(config)
    assert [p.text for p in robot.findall("ros2_control/hardware/plugin")] == [plugin]
    assert config.ros_global_params["use_sim_time"] is sim_time
    assert robot.findall("ros2_control/sensor") == []
    assert (
        config.hardware.additional_driver_launch_file.path == "launch/blank.launch.py"
    )
    assert config.hardware.robot_description.urdf.package == "rebot_b601_rs_mock"
    assert config.ros2_control["config"]["package"] == "rebot_b601_rs_mock"
    links = {link.get("name") for link in robot.findall("link")}
    assert not any(
        "camera" in name or "d435" in name or "bench" in name for name in links
    )
    origin = robot.find("joint[@name='world_to_base']/origin")
    assert origin.get("xyz") == origin.get("rpy") == "0 0 0"
    for mesh in robot.findall(".//mesh"):
        package, path = mesh.get("filename")[len("package://") :].split("/", 1)
        assert package == "rebot_description"
        assert (Path(get_package_share_directory(package)) / path).is_file()
    loaders = config.objectives.behavior_loader_plugins
    assert "rebot" not in loaders
    assert all("Mujoco" not in loader for group in loaders.values() for loader in group)
    # The last library is the selected source config's own save destination.
    assert (
        list(config.objectives.objective_library_paths.values())[-1].package_name
        == f"rebot_b601_rs_{mode}"
    )


def test_physical_torque_policy_and_joint_contract():
    robot = expanded(resolved("hw"))
    params = {
        p.get("name"): p.text for p in robot.findall("ros2_control/hardware/param")
    }
    assert params["hold_torque_on_deactivate"] == "true"
    assert params["motor_can_timeout_ms"] == "0"
    assert params["freeze_on_joint_loss"] == "true"
    assert params["torque_enable"] == "true"
    control = robot.find("ros2_control")
    joints = control.findall("joint")
    assert [j.get("name") for j in joints] == [f"joint{i}" for i in range(1, 8)]
    assert all(
        [i.get("name") for i in j.findall("command_interface")] == ["position"]
        for j in joints
    )
    assert len(control.findall("gpio")) == 7
    for name in ["gripper_joint1", "gripper_joint2"]:
        mimic = robot.find(f"joint[@name='{name}']/mimic")
        assert mimic.get("joint") == "joint7"
        assert float(mimic.get("multiplier")) == 0.008
    path = (
        Path(get_package_share_directory("rebot_b601_rs_mock"))
        / "config/moveit/rebot.srdf"
    )
    srdf = ET.parse(path).getroot()
    links = {link.get("name") for link in robot.findall("link")}
    for group in srdf.findall("group"):
        for chain in group.findall("chain"):
            assert chain.get("base_link") in links and chain.get("tip_link") in links
    ee = srdf.find("end_effector")
    assert ee.get("parent_link") == "gripper_end" and ee.get("group") == "gripper"
