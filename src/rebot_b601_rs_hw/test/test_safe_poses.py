# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""One HOME source, strict admission data, and generated motion consumers."""
import copy
from dataclasses import replace
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC / "python"))
from rebot_safe_poses import (  # noqa: E402
    JOINT_NAMES,
    generate_objective,
    generate_waypoints,
    load_home,
    load_poses,
)


def share(package):
    return Path(get_package_share_directory(package))


GENERATED = {
    "return_rebot_leader_home.xml": "Return reBot Leader Home",
    "relax_at_home.xml": "Relax at HOME",
}


def generated(name):
    return share("rebot_b601_rs_hw_generated_objectives") / "objectives" / name


def test_shipped_home_contract():
    home = load_home()
    assert home.name == "HOME"
    assert len(load_poses(SRC / "config/relax_safe_poses.yaml")) == 1
    assert home.positions == (0.0,) * 7
    assert home.tolerances == (0.08,) * 7
    assert home.window_s == 0.3 and home.max_span_rad == 0.005


@pytest.mark.parametrize(
    "package", ["rebot_b601_rs_mock", "picknik_boston_rebot_b601_rs_hw"]
)
def test_installed_home_waypoints_are_generated_from_one_entry(package):
    entries = yaml.safe_load(
        (share(package) / "waypoints/rebot_waypoints.yaml").read_text()
    )
    home_state = next(w["joint_state"] for w in entries if w["name"] == "Zeroed Rest")
    home = load_home()
    assert dict(zip(home_state["name"], home_state["position"])) == dict(
        zip(JOINT_NAMES, home.positions)
    )


@pytest.mark.parametrize(
    "package", ["rebot_b601_rs_hw", "picknik_boston_rebot_b601_rs_hw"]
)
def test_hardware_xacro_passes_canonical_path_and_accepts_override(package):
    path = share(package) / "description/rebot.urdf.xacro"
    for override in (None, "/tmp/installation-safe-poses.yaml"):
        mappings = {"hardware_interface": "real", "torque_enable": "false"}
        if override is not None:
            mappings["relax_safe_poses_file"] = override
        robot = ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())
        actual = robot.findtext(
            "ros2_control/hardware/param[@name='relax_safe_poses_file']"
        )
        assert actual == (
            override or str(share("rebot_b601_rs_hw") / "config/relax_safe_poses.yaml")
        )
        assert (
            robot.findtext("ros2_control/hardware/param[@name='torque_enable']")
            == "false"
        )


def test_nonzero_home_and_distinct_tolerances_drive_all_generated_consumers(tmp_path):
    home = replace(
        load_home(),
        positions=(0.1, 0.2, 0.3, -0.4, 0.5, -0.6, 0.7),
        tolerances=(0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07),
    )
    text = generate_objective(SRC / "objectives/return_rebot_leader_home.xml.in", home)
    root = ET.fromstring(text)
    assert (
        float(
            root.find(".//Action[@ID='CreateRobotState'][@joint_names='joint5']").get(
                "positions"
            )
        )
        == home.positions[4]
    )
    assert (
        float(
            root.find(".//Action[@ID='CreateRobotState'][@joint_names='joint7']").get(
                "positions"
            )
        )
        == home.positions[6]
    )
    skip = root.find(".//Sequence[@name='MoveArmUnlessHome']").get("_skipIf")
    for j, q, t in zip(JOINT_NAMES[:6], home.positions, home.tolerances):
        assert f"home_{j} >= {q - t:.17g}" in skip
        assert f"home_{j} <= {q + t:.17g}" in skip
    template = SRC.parent / "rebot_b601_rs_mock/waypoints/rebot_waypoints.yaml.in"
    entries = yaml.safe_load(generate_waypoints(template, home))
    assert next(w for w in entries if w["name"] == "Zeroed Rest")["joint_state"][
        "position"
    ] == list(home.positions)
    for package in ("rebot_b601_rs_mock", "picknik_boston_rebot_b601_rs_hw"):
        srdf = ET.fromstring(
            generate_objective(
                SRC.parent / package / "config/moveit/rebot.srdf.in", home
            )
        )
        state = srdf.find("group_state[@name='zeroed_rest']")
        assert {j.get("name"): float(j.get("value")) for j in state} == dict(
            zip(JOINT_NAMES[:6], home.positions[:6])
        )
    assert home.matches(home.positions)
    edge = list(home.positions)
    edge[2] += 2 * home.tolerances[2]
    assert not home.matches(edge)


@pytest.mark.parametrize("joint", range(7))
def test_each_joint_has_its_own_inclusive_tolerance(joint):
    home = load_home()
    q = list(home.positions)
    q[joint] += home.tolerances[joint]
    assert home.matches(q)
    q[joint] += 1e-6
    assert not home.matches(q)
    q[joint] = math.nan
    assert not home.matches(q)


def test_stillness_needs_full_fresh_all_joint_span():
    home = load_home()
    window = [(1.0 + 0.01 * i, [0.0] * 7) for i in range(32)]
    assert home.stationary(window, 1.31)
    assert not home.stationary(window[-1:], 1.31)
    assert not home.stationary(window, 1.5)
    assert not home.stationary([window[0], window[-1]], 1.31)
    window[-1][1][6] = 0.006
    assert not home.stationary(window, 1.31)
    assert home.stationary(window, 1.31, arm_only=True)


@pytest.mark.parametrize(
    "mutation",
    [
        "missing_joint",
        "extra_joint",
        "missing_tolerance",
        "nan",
        "infinite",
        "bool",
        "negative_tolerance",
        "zero_window",
        "duplicate_name",
        "home_not_first",
        "wrong_version",
        "bool_version",
        "overlong_window",
        "bad_velocity",
        "unknown_stillness",
    ],
)
def test_malformed_admission_data_is_rejected(tmp_path, mutation):
    values = yaml.safe_load((SRC / "config/relax_safe_poses.yaml").read_text())
    pose = values["relax_safe_poses"][0]
    if mutation == "missing_joint":
        del pose["joints"]["joint7"]
    elif mutation == "extra_joint":
        pose["joints"]["joint8"] = 0
    elif mutation == "missing_tolerance":
        del pose["joints"]["joint3"]["tolerance_rad"]
    elif mutation in ("nan", "infinite", "bool"):
        pose["joints"]["joint1"]["position_rad"] = {
            "nan": math.nan,
            "infinite": math.inf,
            "bool": True,
        }[mutation]
    elif mutation == "negative_tolerance":
        pose["joints"]["joint1"]["tolerance_rad"] = -0.1
    elif mutation == "zero_window":
        pose["stillness"]["window_s"] = 0
    elif mutation == "duplicate_name":
        values["relax_safe_poses"].append(copy.deepcopy(pose))
    elif mutation == "home_not_first":
        pose["name"] = "OTHER"
    elif mutation in ("wrong_version", "bool_version"):
        values["version"] = 2 if mutation == "wrong_version" else True
    elif mutation == "overlong_window":
        pose["stillness"]["window_s"] = 10.1
    elif mutation == "bad_velocity":
        pose["stillness"]["max_abs_velocity_rad_s"] = 0
    else:
        pose["stillness"]["unknown"] = 1
    path = tmp_path / "poses.yaml"
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError):
        load_poses(path)


def test_duplicate_yaml_key_is_rejected(tmp_path):
    path = tmp_path / "poses.yaml"
    path.write_text("version: 1\nrelax_safe_poses: []\nrelax_safe_poses: []\n")
    with pytest.raises(ValueError, match="Duplicate YAML key"):
        load_poses(path)


def test_motion_entries_engage_before_any_motion_or_prepare():
    hw = share("rebot_b601_rs_hw") / "objectives"
    mirror = share("rebot_b601_rs_hw") / "leader_objectives/mirror_rebot_leader.xml"
    boston = share("picknik_boston_rebot_b601_rs_hw") / "objectives"
    # One known-state entry: engage, JTAC owner, then spring gains. Engage
    # while ON keeps runtime gains, so it ends any gravity-comp float too.
    first, switch, spring = list(
        ET.parse(hw / "rebot_engage_arm.xml").getroot().find("BehaviorTree/Control")
    )
    assert first.get("ID") == "CallTriggerService"
    assert first.get("service_name") == "/robstride_hardware_interface/engage"
    assert switch.get("ID") == "SwitchController"
    assert switch.get("activate_controllers") == (
        "joint_trajectory_admittance_controller"
    )
    assert spring.get("ID") == "reBot Set Spring Gains"
    # Every top-level arm motion entry runs it before anything else.
    for path in (
        generated("return_rebot_leader_home.xml"),
        hw / "teleoperate.xml",
        hw / "move_to_raised.xml",
        hw / "move_to_rest.xml",
        mirror,
        boston / "collect_rebot_gravity_data.xml",
    ):
        sequence = ET.parse(path).getroot().find("BehaviorTree")[0]
        assert sequence.tag in ("Sequence", "Control"), path
        assert (sequence[0].tag, sequence[0].get("ID")) == (
            "SubTree",
            "reBot Engage Arm",
        ), path
    # The physical overrides keep the stock waypoint calls unchanged.
    for name in ("move_to_raised.xml", "move_to_rest.xml"):
        stock = ET.parse(share("rebot_b601_rs_mock") / "objectives" / name).getroot()
        physical = ET.parse(hw / name).getroot()
        assert physical.find("BehaviorTree").get("ID") == stock.find(
            "BehaviorTree"
        ).get("ID")
        assert (
            physical.find(".//SubTree[@ID='Move to Named Waypoint']").attrib
            == stock.find(".//SubTree[@ID='Move to Named Waypoint']").attrib
        )
    # Native nonhardware teleop remains usable without a fake driver server.
    assert (
        ET.parse(share("rebot_b601_rs_mock") / "objectives/teleoperate.xml").find(
            ".//Action[@ID='CallTriggerService']"
        )
        is None
    )


def test_home_notification_is_config_specific_and_optional():
    home = ET.parse(generated("return_rebot_leader_home.xml"))
    assert (
        home.find(".//Action[@service_name='/rebot_leader_bridge/home_verified']")
        is None
    )
    sequence = home.find(
        "BehaviorTree[@ID='Return reBot Leader Home Under Power']/Sequence"
    )
    wait_index = next(
        i for i, step in enumerate(sequence) if step.get("ID") == "WaitForDuration"
    )
    assert sequence[wait_index + 1].get("ID") == "reBot Notify Leader Home"
    assert sequence[wait_index + 2].get("ID") == "ForceSuccess"  # Jaw HOME unchanged.
    stock = ET.parse(
        share("rebot_b601_rs_hw") / "objectives/rebot_notify_leader_home.xml"
    )
    assert stock.find("BehaviorTree/AlwaysSuccess") is not None
    assert stock.find(".//Action") is None
    leader = ET.parse(
        share("rebot_b601_rs_hw") / "leader_objectives/rebot_notify_leader_home.xml"
    )
    call = leader.find("BehaviorTree/Decorator[@ID='ForceSuccess']/Action")
    assert call.attrib == {
        "ID": "CallTriggerService",
        "service_name": "/rebot_leader_bridge/home_verified",
        "wait_for_server_available_timeout": "1.0",
        "response_timeout": "1.0",
    }


def test_built_objectives_are_installed_only_in_their_own_library():
    for name in GENERATED:
        assert not (share("rebot_b601_rs_hw") / "objectives" / name).exists()
        assert not (SRC / "objectives" / name).exists()
        assert (SRC / "objectives" / f"{name}.in").is_file()
        assert "@HOME_" not in generated(name).read_text()


@pytest.mark.parametrize(
    "config",
    [
        "rebot_b601_rs_hw",
        "picknik_boston_rebot_b601_rs_hw",
        "picknik_boston_rebot_b601_rs_sim",
    ],
)
def test_every_source_runtime_start_links_built_home_and_relax_xml(
    monkeypatch, tmp_path, config
):
    """A source/dev start: materialize, then REST startup sync, then restarts.

    Startup resolves workspace packages to source, where only the templates
    exist, and deletes config links it cannot find there. The built XML must
    survive every start and be listed, never the .xml.in templates.
    """
    from moveit_studio_utils_py import system_config

    workspace = tmp_path / "user_ws"
    workspace.mkdir()
    (workspace / "src").symlink_to(SRC.parent)
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("USER_WS", str(workspace))
    monkeypatch.setenv("MOVEIT_CONFIG_PACKAGE", config)
    from moveit_studio_rest_api.routers.objective_api import objective_common

    system_config.materialize_config_state()
    objectives = system_config.get_config_folder() / "objectives"
    for _ in range(3):  # Each Runtime start syncs twice before serving.
        objective_common.sync_objective_library_links(objectives)
        objective_common.sync_objective_library_links(objectives)
        listed = {
            path.name: path
            for path in objective_common.find_objective_files(objectives)
        }
        for name, tree in GENERATED.items():
            link = objectives / name
            assert link.is_symlink()
            assert link.resolve() == generated(name).resolve()
            assert listed[name].resolve() == generated(name).resolve()
            assert ET.parse(link).getroot().get("main_tree_to_execute") == tree
        assert not list(objectives.glob("*.in"))


def test_relax_follows_successful_home_and_is_not_best_effort():
    sequence = ET.parse(generated("relax_at_home.xml")).find("BehaviorTree/Sequence")
    assert sequence[0].get("ID") == "reBot Check Leader Idle"
    # Float ends before HOME: engage, own JTAC, re-anchor and restore spring gains.
    assert sequence[1].get("service_name") == "/robstride_hardware_interface/engage"
    assert sequence[2].get("activate_controllers") == (
        "joint_trajectory_admittance_controller"
    )
    assert sequence[3].get("ID") == "reBot Set Spring Gains"
    assert sequence[4].get("ID") == "Return reBot Leader Home"
    assert sequence[-2].get("ID") == "reBot Check Leader Idle"
    assert sequence[-1].get("ID") == "CallTriggerService"
    assert sequence[-1].get("service_name") == "/robstride_hardware_interface/relax"
    assert not sequence.findall(".//ForceSuccess")
