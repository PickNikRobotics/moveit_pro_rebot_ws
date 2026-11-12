# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The office config enables the stock leader; the office driver launch adds only cameras."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.system_config import load_system_config
import pytest


OFFICE_SERIAL = "CV2F861000FP"


@pytest.fixture
def startup():
    path = (
        Path(get_package_share_directory("rebot_b601_rs_hw"))
        / "launch/rebot_leader.launch.py"
    )
    spec = importlib.util.spec_from_file_location("rebot_startup", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("mode", ["mock", "real", "mujoco"])
@pytest.mark.parametrize("enabled", [False, True])
def test_merged_yaml_selects_one_reader_without_duplicating_stock_guard(
    startup, mode, enabled, tmp_path
):
    config = load_system_config("picknik_boston_rebot_b601_rs_hw", tmp_path)[0]
    config.ros_global_params["rebot_leader_enabled"] = enabled
    for entry in config.hardware.robot_description.urdf_params:
        if "hardware_interface" in entry:
            entry["hardware_interface"] = mode
    settings = startup.startup_settings(config)
    assert settings["hardware_interface"] == mode
    assert settings["thermal_guard"] is False
    assert (settings["bridge"] is not None) == enabled
    # Smoke-load the actual launch actions without executing hardware processes.
    assert len(startup.configured_nodes(config)) == int(enabled)
    if enabled:
        assert settings["bridge"]["arguments"] == [
            "--port",
            config.ros_global_params["rebot_leader_port"],
            "--calibration-file",
            str(
                Path(get_package_share_directory("rebot_b601_rs_hw"))
                / "config/leader_calibration/rebot_arm_102_leader.json"
            ),
            # Collision checks use the office SRDF, not the stock one.
            "--srdf-file",
            str(
                Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw"))
                / "config/moveit/rebot.srdf"
            ),
        ]


def test_office_config_enables_the_stock_leader_with_its_bench_settings(
    startup, tmp_path
):
    config = load_system_config("picknik_boston_rebot_b601_rs_hw", tmp_path)[0]
    assert config.ros_global_params["rebot_leader_enabled"] is True
    bridge = startup.startup_settings(config)["bridge"]
    assert bridge["publish_rate"] == 100.0
    assert bridge["engage_speed_rad_s"] == 0.2
    assert bridge["engage_jaw_speed_rad_s"] == 1.0
    assert bridge["engage_max_offset_rad"] == 0.6
    library = config.objectives.objective_library_paths["rebot_leader_objectives"]
    assert (library.package_name, library.relative_path) == (
        "rebot_b601_rs_hw",
        "leader_objectives",
    )
    paths = list(config.objectives.objective_library_paths)
    # After the stock no-leader library, before the office library that saves edits.
    assert (
        paths.index("rebot_hw_objectives")
        < paths.index("rebot_leader_objectives")
        < paths.index("boston_objectives")
        == len(paths) - 1
    )
    assert config.objectives.behavior_loader_plugins["rebot_leader"] == [
        "rebot_b601_rs_hw::RebotLeaderBehaviorsLoader"
    ]
    assert (
        "leader_position_controller"
        in config.ros2_control["controllers_inactive_at_startup"]
    )


def test_driver_launch_reuses_stock_wrist_camera_with_the_calibrated_unit_serial():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

    share = Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw"))
    path = share / "launch/rebot_drivers.launch.py"
    spec = importlib.util.spec_from_file_location("boston_drivers", path)
    drivers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(drivers)
    launch = drivers.generate_launch_description()
    context = LaunchContext()
    includes = [
        action
        for action in launch.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 2
    stock_driver = next(action for action in includes if action.launch_arguments)
    assert list(stock_driver.launch_arguments) == [
        ("wrist_camera_serial_number", OFFICE_SERIAL)
    ]
    # Loading declarations does not execute the USB probe or any camera driver.
    set_serial, stock_description = stock_driver.execute(context)
    set_serial.execute(context)
    stock_includes = [
        action
        for action in stock_description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    # Stock thermal guard, wrist-camera launch and the optional leader.
    assert len(stock_includes) == 3
    wrist_launch = None
    for action in stock_includes:
        child = action.launch_description_source.get_launch_description(context)
        if any(
            isinstance(entity, DeclareLaunchArgument)
            and entity.name == "wrist_camera_serial_number"
            for entity in child.entities
        ):
            wrist_launch = child
            break
    assert wrist_launch is not None
    serial = next(
        entity
        for entity in wrist_launch.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == "wrist_camera_serial_number"
    )
    serial.execute(context)
    assert context.launch_configurations["wrist_camera_serial_number"] == OFFICE_SERIAL
    # The remaining include is Boston's scene camera; the leader comes from stock.
    assert sum(not action.launch_arguments for action in includes) == 1
