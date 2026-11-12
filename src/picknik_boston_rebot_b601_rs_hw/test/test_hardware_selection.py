# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The application inherits the stock hardware semantics and isolates an optional fitted model."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.system_config import load_system_config
import pytest
import xacro
import yaml

PACKAGE = "picknik_boston_rebot_b601_rs_hw"
WRIST_FRAME = "wrist_mounted_camera_color_optical_frame"
NOMINAL_XYZ = "-0.111 0.009 0.109"
NOMINAL_RPY = "-2.109 0.003 -1.579"


def expand(package=PACKAGE, **mappings):
    path = Path(get_package_share_directory(package)) / "description/rebot.urdf.xacro"
    return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


def params(element):
    return {p.get("name"): p.text for p in element.findall("param")}


def test_loaded_boston_config_has_one_real_backend_and_office_save_destination(
    tmp_path,
):
    config = load_system_config(PACKAGE, tmp_path)[0]
    mappings = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    model = expand(**mappings)
    assert [p.text for p in model.findall("ros2_control/hardware/plugin")] == [
        "robstride_hardware_interface/RobstrideHardware"
    ]
    assert config.hardware.robot_description.srdf.package == PACKAGE
    assert config.ros_global_params["use_sim_time"] is False
    assert config.hardware.additional_driver_launch_file.package == PACKAGE
    assert (
        list(config.objectives.objective_library_paths.values())[-1].package_name
        == PACKAGE
    )
    assert config.objectives.behavior_loader_plugins["rebot_hardware"] == [
        "rebot_b601_rs_hw::RebotHardwareBehaviorsLoader"
    ]
    assert config.ros2_control["controllers_active_at_startup"] == [
        "joint_state_broadcaster",
        "joint_trajectory_admittance_controller",
        "gravity_compensation_controller",
    ]
    selected = {
        key: value
        for item in config.hardware.robot_description.urdf_params
        for key, value in item.items()
    }
    # Uncalibrated defaults: no wrist mount override, nominal gravity model.
    assert "wrist_camera_color_optical_xyz" not in selected
    assert "wrist_camera_color_optical_rpy" not in selected
    assert selected["boston_gravity_fit"] is False


def test_arm_interfaces_modes_limits_and_torque_policy_match_vendor():
    for mode in ("mock", "real", "mujoco"):
        mappings = {"hardware_interface": mode, "arm_effort_command": "true"}
        stock, office = expand("rebot_b601_rs_mock", **mappings), expand(**mappings)
        assert ET.tostring(stock.find("ros2_control")) == ET.tostring(
            office.find("ros2_control")
        )
        for name in [f"joint{i}" for i in range(1, 8)]:
            a, b = [root.find(f"joint[@name='{name}']") for root in (stock, office)]
            assert a.attrib == b.attrib
            for child in ("parent", "child", "origin", "axis", "limit"):
                x, y = a.find(child), b.find(child)
                assert (x.attrib if x is not None else None) == (
                    y.attrib if y is not None else None
                )
    real = expand(hardware_interface="real", arm_effort_command="true")
    assert (
        params(real.find("ros2_control/hardware"))["hold_torque_on_deactivate"]
        == "true"
    )
    assert (
        params(real.find("ros2_control/gpio[@name='joint2_motor']"))[
            "max_feedforward_effort"
        ]
        == "18.0"
    )


def test_fit_changes_only_documented_six_inertial_origins_and_cannot_enter_simulation():
    nominal = expand(hardware_interface="real", boston_gravity_fit="false")
    fitted = expand(hardware_interface="real", boston_gravity_fit="true")
    share = Path(get_package_share_directory(PACKAGE))
    origins = yaml.safe_load(
        (share / "config/gravity/identified_first_moments.yaml").read_text()
    )
    assert set(origins) == {"link2", "link3", "link4", "link5", "link6", "gripper_end"}
    for name, value in origins.items():
        changed = fitted.find(f"link[@name='{name}']/inertial/origin")
        assert changed.get("xyz") == value
        changed.set(
            "xyz", nominal.find(f"link[@name='{name}']/inertial/origin").get("xyz")
        )
    assert ET.tostring(fitted) == ET.tostring(nominal)
    for backend in ("mock", "mujoco"):
        with pytest.raises(xacro.XacroException):
            expand(hardware_interface=backend, boston_gravity_fit="true")
    control = yaml.safe_load(
        (share / "config/control/rebot.ros2_control.yaml").read_text()
    )
    gravity = control["gravity_compensation_controller"]["ros__parameters"]
    stock = yaml.safe_load(
        (
            Path(get_package_share_directory("rebot_b601_rs_mock"))
            / "config/control/rebot.ros2_control.yaml"
        ).read_text()
    )["gravity_compensation_controller"]["ros__parameters"]
    # Uncalibrated default: the nominal stock model, gains and jaw payload.
    assert gravity == stock
    assert config_flag("boston_gravity_fit") is False


def config_flag(name):
    import tempfile

    with tempfile.TemporaryDirectory() as directory:
        config = load_system_config(PACKAGE, Path(directory))[0]
    return next(
        entry[name]
        for entry in config.hardware.robot_description.urdf_params
        if name in entry
    )


def test_default_wrist_mount_is_the_stock_nominal_frame_without_d435_links():
    stock = expand("rebot_b601_rs_hw", hardware_interface="real")
    office = expand(hardware_interface="real")
    stock_joint = stock.find("joint[@name='wrist_mounted_camera_color_optical_joint']")
    office_joint = office.find(
        "joint[@name='wrist_mounted_camera_color_optical_joint']"
    )
    assert stock_joint is not None
    assert office_joint is not None
    assert office.find(f"link[@name='{WRIST_FRAME}']") is not None
    assert office_joint.find("parent").get("link") == "gripper_end"
    assert office_joint.find("child").get("link") == WRIST_FRAME
    assert office_joint.find("origin").attrib == {
        "xyz": NOMINAL_XYZ,
        "rpy": NOMINAL_RPY,
    }
    assert stock_joint.find("origin").attrib == office_joint.find("origin").attrib
    assert not any(element.get("name", "").startswith("d435_") for element in office)
    assert office.find("link[@name='scene_camera_color_optical_frame']") is not None
    origin = office.find("joint[@name='world_to_base']/origin")
    assert origin.get("xyz") == "0.726 0.022 0.0"
    assert origin.get("rpy") == "0 0 3.14159"
    for mesh in office.findall(".//mesh"):
        uri = mesh.get("filename")
        package, relative = uri.removeprefix("package://").split("/", 1)
        path = Path(get_package_share_directory(package)) / relative
        assert path.stat().st_size > 200, path
        assert not path.read_bytes().startswith(
            b"version https://git-lfs.github.com/spec/v1"
        )


def test_backend_and_jaw_overrides_remain_available():
    with pytest.raises(xacro.XacroException):
        expand(hardware_interface="bogus")
    control = expand(
        hardware_interface="real",
        arm_control_mode="position_csp",
        can_interface="can1",
        torque_enable="false",
        jaw_kp="10",
        jaw_kd="0.7",
    ).find("ros2_control")
    assert params(control.find("hardware"))["torque_enable"] == "false"
    gpios = {g.get("name"): params(g) for g in control.findall("gpio")}
    assert {g["can_interface"] for g in gpios.values()} == {"can1"}
    assert all(
        gpios[f"joint{i}_motor"]["control_mode"] == "position_csp" for i in range(1, 7)
    )
    assert gpios["joint7_motor"]["control_mode"] == "motion"
    assert (gpios["joint7_motor"]["kp"], gpios["joint7_motor"]["kd"]) == ("10", "0.7")
