"""Verify backend isolation and the shared stock-product semantic contract."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml
from moveit_studio_utils_py.system_config import load_system_config, merge


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
        ("sim", "picknik_mujoco_ros/MujocoSystem", False),
        ("hw", "robstride_hardware_interface/RobstrideHardware", False),
    ],
)
def test_backend_isolation(mode, plugin, sim_time):
    config = resolved(mode)
    robot = expanded(config)
    assert [p.text for p in robot.findall("ros2_control/hardware/plugin")] == [plugin]
    assert config.ros_global_params["use_sim_time"] is sim_time
    assert robot.findall("ros2_control/sensor") == []
    if mode != "sim":
        controller_path = (
            Path(get_package_share_directory("rebot_b601_rs_mock"))
            / "config/control/rebot.ros2_control.yaml"
        )
        parameters = yaml.safe_load(controller_path.read_text())[
            "joint_state_broadcaster"
        ]["ros__parameters"]
        exported = {
            f"{joint.get('name')}/{interface.get('name')}"
            for joint in robot.findall("ros2_control/joint")
            for interface in joint.findall("state_interface")
        }
        requested = {
            f"{joint}/{interface}"
            for joint in parameters["joints"]
            for interface in parameters["interfaces"]
        }
        assert requested <= exported
    driver_launch = config.hardware.additional_driver_launch_file
    if mode == "hw":
        assert driver_launch.package == "rebot_b601_rs_hw"
        assert driver_launch.path == "launch/rebot_drivers.launch.py"
    else:
        assert driver_launch.path == "launch/blank.launch.py"
    assert config.hardware.robot_description.urdf.package == (
        "rebot_b601_rs_hw" if mode == "hw" else "rebot_b601_rs_mock"
    )
    assert config.ros2_control["config"]["package"] == "rebot_b601_rs_mock"
    links = {link.get("name") for link in robot.findall("link")}
    wrist_frame = "wrist_mounted_camera_color_optical_frame"
    if mode == "hw":
        assert wrist_frame in links
        wrist_joint = robot.find(
            "joint[@name='wrist_mounted_camera_color_optical_joint']"
        )
        assert wrist_joint.find("parent").get("link") == "gripper_end"
        assert wrist_joint.find("child").get("link") == wrist_frame
        assert wrist_joint.find("origin").attrib == {
            "xyz": "-0.111 0.009 0.109",
            "rpy": "-2.109 0.003 -1.579",
        }
    else:
        assert wrist_frame not in links
    assert not any("d435" in name or "bench" in name for name in links)
    origin = robot.find("joint[@name='world_to_base']/origin")
    assert origin.get("xyz") == origin.get("rpy") == "0 0 0"
    for mesh in robot.findall(".//mesh"):
        package, path = mesh.get("filename")[len("package://") :].split("/", 1)
        assert package == "rebot_description"
        assert (Path(get_package_share_directory(package)) / path).is_file()
    loaders = config.objectives.behavior_loader_plugins
    assert "rebot" not in loaders
    if mode == "hw":
        assert loaders["motion_recovery"] == [
            "motion_recovery_behaviors::MotionRecoveryBehaviorsLoader"
        ]
        assert loaders["rebot_hardware"] == [
            "rebot_b601_rs_hw::RebotHardwareBehaviorsLoader"
        ]
    else:
        assert "rebot_hardware" not in loaders
    mujoco_loaders = [
        loader for group in loaders.values() for loader in group if "Mujoco" in loader
    ]
    assert mujoco_loaders == (
        ["moveit_pro::behaviors::MujocoBehaviorsLoader"] if mode == "sim" else []
    )
    if mode == "hw":
        assert all(
            library.package_name != "rebot_b601_rs_sim"
            for library in config.objectives.objective_library_paths.values()
        )
    # The last library is the selected source config's own save destination.
    assert (
        list(config.objectives.objective_library_paths.values())[-1].package_name
        == f"rebot_b601_rs_{mode}"
    )


@pytest.mark.parametrize("mode", ["mock", "sim", "hw"])
def test_jaw_feedback_allowance_is_shared(mode):
    config = resolved(mode)
    robot = expanded(config)
    selection = config.moveit_params.joint_limits
    limits = yaml.safe_load(
        (
            Path(get_package_share_directory(selection.package)) / selection.path
        ).read_text()
    )["joint_limits"]
    jaw = robot.find("joint[@name='joint7']/limit")
    assert float(jaw.get("lower")) == limits["joint7"]["min_position"] == -0.15
    assert float(jaw.get("upper")) == limits["joint7"]["max_position"] == 6.10
    for name in ("gripper_joint1", "gripper_joint2"):
        finger = robot.find(f"joint[@name='{name}']")
        mimic = finger.find("mimic")
        factor = float(mimic.get("multiplier"))
        offset = float(mimic.get("offset"))
        lower = float(finger.find("limit").get("lower"))
        upper = float(finger.find("limit").get("upper"))
        assert lower == pytest.approx(factor * -0.15 + offset)
        assert upper == pytest.approx(factor * 6.10 + offset)


def test_nominal_wrist_camera_mount_is_overridable_for_an_office_calibration():
    config = resolved("hw")
    override = {
        "hardware": {
            "robot_description": {
                "urdf_params": [
                    {"wrist_camera_color_optical_xyz": "1 2 3"},
                    {"wrist_camera_color_optical_rpy": "0.1 0.2 0.3"},
                ]
            }
        }
    }
    merged = merge(config.model_dump(), override)
    config.hardware.robot_description.urdf_params = merged["hardware"][
        "robot_description"
    ]["urdf_params"]
    origin = expanded(config).find(
        "joint[@name='wrist_mounted_camera_color_optical_joint']/origin"
    )
    assert origin.attrib == {"xyz": "1 2 3", "rpy": "0.1 0.2 0.3"}


def test_physical_torque_policy_and_joint_contract():
    robot = expanded(resolved("hw"))
    params = {
        p.get("name"): p.text for p in robot.findall("ros2_control/hardware/param")
    }
    assert params["hold_torque_on_deactivate"] == "true"
    assert params["motor_can_timeout_ms"] == "0"
    assert params["freeze_on_joint_loss"] == "true"
    assert float(params["thermal_cutoff_c"]) == 100.0
    assert float(params["thermal_clear_c"]) == 75.0
    assert params["torque_enable"] == "true"
    control = robot.find("ros2_control")
    for gpio in control.findall("gpio"):
        cap = gpio.findtext("param[@name='max_feedforward_effort']")
        if gpio.get("name") == "joint2_motor":
            assert float(cap) == 18.0
        else:
            assert cap is None  # SDK/driver defaults, no extra overrides.
    joints = control.findall("joint")
    assert [j.get("name") for j in joints] == [f"joint{i}" for i in range(1, 8)]
    for index, joint in enumerate(joints):
        expected = ["position", "effort"] if index < 6 else ["position"]
        assert [i.get("name") for i in joint.findall("command_interface")] == expected
        assert [i.get("name") for i in joint.findall("state_interface")] == [
            "position",
            "velocity",
            "effort",
        ]
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


def test_physical_spring_and_nominal_gravity_startup():
    config = resolved("hw")
    robot = expanded(config)
    assert config.ros2_control["controllers_active_at_startup"] == [
        "joint_state_broadcaster",
        "joint_trajectory_admittance_controller",
        "gravity_compensation_controller",
    ]
    arm_kp = [50.0, 150.0, 150.0, 50.0, 50.0, 50.0]
    arm_kd = [3.0, 10.0, 10.0, 5.0, 4.0, 4.0]
    for index, gpio in enumerate(robot.findall("ros2_control/gpio")):
        params = {p.get("name"): p.text for p in gpio.findall("param")}
        assert params["control_mode"] == "motion"
        if index < 6:
            assert float(params["kp"]) == arm_kp[index]
            assert float(params["kd"]) == arm_kd[index]
    path = Path(get_package_share_directory("rebot_b601_rs_mock"))
    controllers = yaml.safe_load(
        (path / "config/control/rebot.ros2_control.yaml").read_text()
    )
    gravity = controllers["gravity_compensation_controller"]["ros__parameters"]
    assert gravity["gains"] == [1.0] * 6
    assert gravity["offsets"] == [0.0] * 6
    assert gravity["payload_mass"] == 0.1504
    assert controllers["controller_manager"]["ros__parameters"]["update_rate"] == 100
    jtac = controllers["joint_trajectory_admittance_controller"]["ros__parameters"]
    assert jtac["default_path_tolerance"] == 0.20
    assert jtac["default_goal_tolerance"] == 0.1
    assert jtac["default_goal_time_tolerance"] == 2.0
    assert jtac["stop_accelerations"] == [0.6] * 6 + [6.0]
    limits = yaml.safe_load((path / "config/moveit/joint_limits.yaml").read_text())[
        "joint_limits"
    ]
    for joint, lower, upper in [
        ("joint2", -0.02, 3.543018381548),
        ("joint3", -0.01, 4.181336912266),
    ]:
        limit = robot.find(f"joint[@name='{joint}']/limit")
        assert limits[joint]["min_position"] == lower
        assert limits[joint]["max_position"] == upper
        assert float(limit.get("lower")) == lower
        assert float(limit.get("upper")) == upper
    # Stop's source values must track the configured spring gains. The execution
    # regression separately proves the ordering and actual service requests.
    objectives = Path(get_package_share_directory("rebot_b601_rs_hw")) / "objectives"
    stop = ET.parse(objectives / "rebot_set_spring_gains.xml").getroot()
    requests = stop.findall(".//Action[@ID='SetRebotMotorGain']")
    assert [
        (int(r.get("motor_id")), r.get("gain"), float(r.get("value"))) for r in requests
    ] == [
        (i + 1, gain, value)
        for gain, values in [("kd", arm_kd), ("kp", arm_kp)]
        for i, value in enumerate(values)
    ]
    # Both mode Objectives first make JTAC the position owner (a jog or leader
    # controller may hold it), then run their gain subtree unchanged.
    for name, subtree in [
        ("start_gravity_comp.xml", "reBot Set Float Gains"),
        ("stop_gravity_comp.xml", "reBot Set Spring Gains"),
    ]:
        steps = list(ET.parse(objectives / name).getroot().find("BehaviorTree")[0])
        assert [(s.tag, s.get("ID")) for s in steps] == [
            ("Action", "SwitchController"),
            ("SubTree", subtree),
        ]
        assert steps[0].attrib == {
            "ID": "SwitchController",
            "activate_controllers": "joint_trajectory_admittance_controller",
            "automatic_deactivation": "true",
            "strictness": "2",
        }
    for mode in ["mock", "sim"]:
        child = resolved(mode)
        assert (
            "gravity_compensation_controller"
            not in child.ros2_control["controllers_active_at_startup"]
        )
