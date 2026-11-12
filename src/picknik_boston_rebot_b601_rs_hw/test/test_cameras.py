# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The office launch owns only the optional OAK-D scene camera."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def cameras():
    spec = importlib.util.spec_from_file_location(
        "rebot_camera_launch", PACKAGE / "launch/cameras.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.logger = Mock()
    return module


def context(**overrides):
    settings = yaml.safe_load((PACKAGE / "config/cameras.yaml").read_text())
    settings.update(overrides)
    result = LaunchContext()
    result.launch_configurations.update(
        {
            key: str(value).lower() if isinstance(value, bool) else str(value)
            for key, value in settings.items()
        }
    )
    return result


def executables(actions, ctx):
    return [
        perform_substitutions(
            ctx, normalize_to_list_of_substitutions(node.node_executable)
        )
        for node in actions
        if isinstance(node, Node)
    ]


def test_defaults_enable_only_the_optional_scene_camera(cameras):
    ctx = context()
    assert ctx.launch_configurations["enable_cameras"] == "true"
    assert ctx.launch_configurations["enable_scene_camera"] == "true"
    assert "wrist_camera_serial_number" not in ctx.launch_configurations
    assert cameras.generate_launch_description().entities


@pytest.mark.parametrize(
    "attached,expected_warnings",
    [
        (False, 1),
        (True, 0),
    ],
)
def test_missing_scene_camera_skips_only_its_processes(
    cameras, monkeypatch, attached, expected_warnings
):
    monkeypatch.setattr(cameras, "scene_camera_detected", lambda: attached)
    warnings = []
    monkeypatch.setattr(cameras.logger, "warning", warnings.append)
    ctx = context(scene_depth_scale=1.0)
    actions = cameras.launch_cameras(ctx)
    assert executables(actions, ctx) == (
        ["camera_node", "point_cloud_xyz_node"] if attached else []
    )
    assert len(warnings) == expected_warnings


def test_scene_camera_switch_never_probes_usb(cameras, monkeypatch):
    monkeypatch.setattr(
        cameras,
        "scene_camera_detected",
        lambda: pytest.fail("disabled scene camera must not probe USB"),
    )
    assert cameras.launch_cameras(context(enable_scene_camera=False)) == []


def test_usb_probe_recognizes_only_scene_models_and_ignores_unknown_or_incomplete(
    cameras, tmp_path
):
    for name, vendor, product in [
        ("scene_unbooted", "03e7", "2485"),
        ("scene_booted", "03e7", "f63b"),
        ("wrist", "2bc5", "0840"),
        ("unknown", "1234", "abcd"),
    ]:
        device = tmp_path / name
        device.mkdir()
        (device / "idVendor").write_text(vendor + "\n")
        (device / "idProduct").write_text(product + "\n")
    (tmp_path / "incomplete").mkdir()
    assert cameras.scene_camera_detected(tmp_path)
    assert not cameras.scene_camera_detected(tmp_path / "missing")


def test_depth_is_unscaled_unless_a_measured_factor_is_configured(cameras, monkeypatch):
    monkeypatch.setattr(cameras, "scene_camera_detected", lambda: True)
    unscaled = context(scene_depth_scale=1.0)
    assert "scale_depth.py" not in executables(
        cameras.launch_cameras(unscaled), unscaled
    )
    # The shipped default publishes depth unchanged; a measured factor scales it.
    assert "scale_depth.py" not in executables(
        cameras.launch_cameras(context()), context()
    )
    scaled = context(scene_depth_scale=0.96)
    assert "scale_depth.py" in executables(cameras.launch_cameras(scaled), scaled)


@pytest.mark.parametrize(
    "settings,error",
    [
        ({"image_width": 0}, "Camera dimensions/fps must be positive"),
        ({"scene_image_height": 480}, "DepthAI ISP output requires"),
        ({"scene_mjpeg_quality": 101}, "Camera dimensions/fps must be positive"),
        ({"scene_depth_scale": 1.6}, "scene_depth_scale must be between"),
    ],
)
def test_invalid_scene_settings_are_rejected(cameras, monkeypatch, settings, error):
    monkeypatch.setattr(cameras, "scene_camera_detected", lambda: True)
    with pytest.raises(ValueError, match=error):
        cameras.launch_cameras(context(**settings))
