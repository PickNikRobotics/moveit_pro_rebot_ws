# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The optional leader stays off in every stock config and starts one bridge when enabled."""
import importlib.metadata
import importlib.util
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.system_config import load_system_config
import pytest


STOCK_CONFIGS = ("rebot_b601_rs_mock", "rebot_b601_rs_sim", "rebot_b601_rs_hw")
SHARE = Path(get_package_share_directory("rebot_b601_rs_hw"))


@pytest.fixture
def startup():
    path = SHARE / "launch/rebot_leader.launch.py"
    spec = importlib.util.spec_from_file_location("rebot_leader_startup", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stock_srdf(config):
    srdf = config.hardware.robot_description.srdf
    return Path(get_package_share_directory(srdf.package)) / srdf.path


@pytest.mark.parametrize("package", STOCK_CONFIGS)
def test_stock_configs_leave_the_leader_off(startup, package, tmp_path):
    config = load_system_config(package, tmp_path)[0]
    assert "rebot_leader_enabled" not in (config.ros_global_params or {})
    settings = startup.startup_settings(config)
    assert settings["bridge"] is None and settings["thermal_guard"] is False
    assert startup.configured_nodes(config) == []
    # No leader library, loader or controller is selected without the opt-in.
    libraries = config.objectives.objective_library_paths.values()
    assert all(entry.relative_path != "leader_objectives" for entry in libraries)
    loaders = config.objectives.behavior_loader_plugins.values()
    assert all(
        "rebot_b601_rs_hw::RebotLeaderBehaviorsLoader" not in group for group in loaders
    )
    controllers = config.ros2_control.get(
        "controllers_active_at_startup", []
    ) + config.ros2_control.get("controllers_inactive_at_startup", [])
    assert "leader_position_controller" not in controllers


@pytest.mark.parametrize("mode", ["mock", "real", "mujoco"])
@pytest.mark.parametrize("package", STOCK_CONFIGS)
def test_enabling_the_flag_alone_starts_one_bridge_with_stock_defaults(
    startup, package, mode, tmp_path
):
    config = load_system_config(package, tmp_path)[0]
    config.ros_global_params["rebot_leader_enabled"] = True
    for entry in config.hardware.robot_description.urdf_params:
        if "hardware_interface" in entry:
            entry["hardware_interface"] = mode
    settings = startup.startup_settings(config)
    assert settings["hardware_interface"] == mode
    assert settings["thermal_guard"] is False
    assert settings["bridge"] == {
        "publish_rate": 100.0,
        "engage_speed_rad_s": 0.2,
        "engage_jaw_speed_rad_s": 1.0,
        "engage_max_offset_rad": 0.6,
        "arguments": [
            "--port",
            startup.DEFAULT_PORT,
            "--calibration-file",
            str(SHARE / "config/leader_calibration/rebot_arm_102_leader.json"),
            "--srdf-file",
            str(stock_srdf(config)),
        ],
    }
    assert len(startup.configured_nodes(config)) == 1
    assert startup.generate_launch_description().entities
    assert Path(settings["bridge"]["arguments"][3]).is_file()
    assert Path(settings["bridge"]["arguments"][5]).is_file()


@pytest.mark.parametrize("value", ["true", 1, None])
def test_invalid_yaml_enable_fails_without_starting_anything(startup, value):
    config = SimpleNamespace(ros_global_params={"rebot_leader_enabled": value})
    with pytest.raises(ValueError, match="YAML boolean"):
        startup.startup_settings(config)


def test_absolute_calibration_and_custom_reader_settings(startup, tmp_path):
    config = load_system_config("rebot_b601_rs_hw", tmp_path)[0]
    calibration = str(tmp_path / "calibration.json")
    config.ros_global_params.update(
        rebot_leader_enabled=True,
        rebot_leader_port="/dev/leader",
        rebot_leader_calibration_file=calibration,
        rebot_leader_publish_rate=20.0,
        rebot_leader_engage_speed_rad_s=0.03,
        rebot_leader_engage_jaw_speed_rad_s=0.07,
        rebot_leader_engage_max_offset_rad=0.4,
    )
    assert startup.startup_settings(config)["bridge"] == {
        "publish_rate": 20.0,
        "engage_speed_rad_s": 0.03,
        "engage_jaw_speed_rad_s": 0.07,
        "engage_max_offset_rad": 0.4,
        "arguments": [
            "--port",
            "/dev/leader",
            "--calibration-file",
            calibration,
            "--srdf-file",
            str(stock_srdf(config)),
        ],
    }


def test_relative_calibration_resolves_in_the_stock_share(startup, tmp_path):
    config = load_system_config("rebot_b601_rs_hw", tmp_path)[0]
    config.ros_global_params.update(
        rebot_leader_enabled=True,
        rebot_leader_calibration_file="config/leader_calibration/other.json",
    )
    arguments = startup.startup_settings(config)["bridge"]["arguments"]
    assert arguments[3] == str(SHARE / "config/leader_calibration/other.json")


def test_missing_srdf_refuses_before_starting_the_bridge(startup, tmp_path):
    config = load_system_config("rebot_b601_rs_hw", tmp_path)[0]
    config.ros_global_params["rebot_leader_enabled"] = True
    config.hardware.robot_description.srdf.path = "config/moveit/missing.srdf"
    with pytest.raises(ValueError, match="SRDF not found"):
        startup.configured_nodes(config)


def test_integer_engage_settings_reach_the_bridge_as_doubles(startup, tmp_path):
    config = load_system_config("rebot_b601_rs_hw", tmp_path)[0]
    config.ros_global_params.update(
        rebot_leader_enabled=True,
        rebot_leader_engage_speed_rad_s=1,
        rebot_leader_engage_jaw_speed_rad_s=1,
        rebot_leader_engage_max_offset_rad=1,
    )
    bridge = startup.startup_settings(config)["bridge"]
    for name in (
        "engage_speed_rad_s",
        "engage_jaw_speed_rad_s",
        "engage_max_offset_rad",
    ):
        assert type(bridge[name]) is float and bridge[name] == 1.0


def test_leader_controller_is_declared_but_not_spawned_by_stock_configs(tmp_path):
    import yaml

    for package in STOCK_CONFIGS:
        config = load_system_config(package, tmp_path)[0]
        location = config.ros2_control["config"]
        control = yaml.safe_load(
            (
                Path(get_package_share_directory(location["package"]))
                / location["path"]
            ).read_text()
        )
        manager = control["controller_manager"]["ros__parameters"]
        assert (
            manager["leader_position_controller"]["type"]
            == "position_controllers/JointGroupPositionController"
        )
        assert control["leader_position_controller"]["ros__parameters"]["joints"] == [
            f"joint{i}" for i in range(1, 8)
        ]


def test_leader_objectives_override_the_stock_no_leader_subtrees_by_file_name():
    stock = {path.name for path in (SHARE / "objectives").glob("*.xml")}
    leader = {path.name for path in (SHARE / "leader_objectives").glob("*.xml")}
    assert leader == {
        "mirror_rebot_leader.xml",
        "rebot_check_leader_idle.xml",
        "rebot_notify_leader_home.xml",
    }
    # Same file names replace the no-op subtrees once the library is listed later.
    assert leader & stock == {
        "rebot_check_leader_idle.xml",
        "rebot_notify_leader_home.xml",
    }


def test_installed_sdk_imports_in_ros_python_without_training_dependencies():
    import sys
    import numpy
    import motorbridge_smart_servo
    from cv_bridge import CvBridge

    dist = importlib.metadata.distribution("motorbridge-smart-servo")
    assert dist.version == "0.0.4"
    assert dist.metadata["License-Expression"] == "MIT"
    assert dist.requires is None
    assert callable(motorbridge_smart_servo.FashionStarServo)
    cv = CvBridge()
    bgr = numpy.full((4, 4, 3), [0, 1, 2], dtype=numpy.uint8)
    image = cv.cv2_to_imgmsg(bgr, "bgr8")
    assert numpy.array_equal(cv.imgmsg_to_cv2(image, "rgb8"), bgr[:, :, ::-1])
    assert not any(name.startswith(("lerobot", "torch")) for name in sys.modules)
