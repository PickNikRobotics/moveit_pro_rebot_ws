"""Verify the stock Gemini 305 launch contract without starting a driver."""

import importlib.util
from pathlib import Path
from unittest.mock import Mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def wrist_camera_launch():
    spec = importlib.util.spec_from_file_location(
        "rebot_wrist_camera_launch", PACKAGE / "launch/wrist_camera.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.logger = Mock()
    return module


def context(**overrides):
    settings = yaml.safe_load((PACKAGE / "config/wrist_camera.yaml").read_text())
    settings.update(overrides)
    result = LaunchContext()
    result.launch_configurations.update(
        {
            key: str(value).lower() if isinstance(value, bool) else str(value)
            for key, value in settings.items()
        }
    )
    return result


def value(context, substitution):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(substitution)
    )


def default_context(wrist_camera_launch):
    result = LaunchContext()
    for argument in wrist_camera_launch.generate_launch_description().entities:
        if not isinstance(argument, DeclareLaunchArgument):
            continue
        result.launch_configurations[argument.name] = value(
            result, argument.default_value
        )
    return result


def test_defaults_enable_camera_startup_and_select_no_unit(wrist_camera_launch):
    settings = yaml.safe_load((PACKAGE / "config/wrist_camera.yaml").read_text())
    assert settings == {
        "enable_wrist_camera": True,
        "wrist_camera_serial_number": "",
        "wrist_camera_color_width": 1280,
        "wrist_camera_color_height": 800,
        "wrist_camera_color_fps": 10,
    }
    launch_context = default_context(wrist_camera_launch)
    assert launch_context.launch_configurations == {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for key, value in settings.items()
    }


def test_disabled_camera_never_probes_usb_or_constructs_driver(
    wrist_camera_launch, monkeypatch
):
    monkeypatch.setattr(
        wrist_camera_launch,
        "gemini305_probe",
        lambda: pytest.fail("disabled camera must not probe USB"),
    )
    assert (
        wrist_camera_launch.launch_wrist_camera(context(enable_wrist_camera=False))
        == []
    )
    wrist_camera_launch.logger.warning.assert_not_called()


def test_enabled_but_absent_camera_warns_without_starting_driver(
    wrist_camera_launch, monkeypatch
):
    monkeypatch.setattr(
        wrist_camera_launch,
        "gemini305_probe",
        lambda: wrist_camera_launch.UsbProbe(Path("/usb"), 3, False),
    )
    assert wrist_camera_launch.launch_wrist_camera(context()) == []
    wrist_camera_launch.logger.warning.assert_called_once()
    warning = wrist_camera_launch.logger.warning.call_args.args[0]
    assert "3 USB entries" in warning
    assert "2bc5:0840" in warning
    wrist_camera_launch.logger.info.assert_any_call(
        "The arm backend remains available. See README: Gemini 305 wrist camera."
    )
    remediation = wrist_camera_launch.logger.info.call_args_list[-1].args[0]
    assert "enable_wrist_camera: false" in remediation
    assert "config/wrist_camera.yaml" in remediation
    assert "rebuild" in remediation


def test_missing_usb_root_is_preserved_as_probe_failure(wrist_camera_launch, tmp_path):
    root = tmp_path / "missing"
    probe = wrist_camera_launch.gemini305_probe(root)
    assert probe.root == root
    assert not probe.found
    assert probe.error is not None


def test_unreadable_usb_root_reports_probe_failure(wrist_camera_launch, monkeypatch):
    probe = wrist_camera_launch.UsbProbe(
        Path("/usb"), 0, False, error=OSError("Permission denied")
    )
    monkeypatch.setattr(wrist_camera_launch, "gemini305_probe", lambda: probe)
    assert wrist_camera_launch.launch_wrist_camera(context()) == []
    warning = wrist_camera_launch.logger.warning.call_args.args[0]
    assert "USB scan failed" in warning
    assert "/usb" in warning
    assert "Permission denied" in warning
    assert "no Gemini 305 device" not in warning
    wrist_camera_launch.logger.info.assert_any_call(
        "Ensure /sys/bus/usb/devices is readable and visible in the container."
    )


@pytest.mark.parametrize("value", ["TRUE", "False"])
def test_camera_switch_accepts_case_insensitive_boolean(wrist_camera_launch, value):
    assert wrist_camera_launch.wrist_camera_enabled(
        context(enable_wrist_camera=value)
    ) == (value.lower() == "true")


@pytest.mark.parametrize("value", ["1", "yes", "true "])
def test_invalid_camera_switch_skips_camera_without_probing_usb(
    wrist_camera_launch, monkeypatch, value
):
    monkeypatch.setattr(
        wrist_camera_launch,
        "gemini305_probe",
        lambda: pytest.fail("invalid camera switch must not probe USB"),
    )
    assert (
        wrist_camera_launch.launch_wrist_camera(context(enable_wrist_camera=value))
        == []
    )
    warning = wrist_camera_launch.logger.warning.call_args.args[0]
    assert "enable_wrist_camera in wrist_camera.yaml" in warning
    assert repr(value) in warning


def test_missing_driver_skips_camera_without_affecting_backend(
    wrist_camera_launch, monkeypatch
):
    monkeypatch.setattr(
        wrist_camera_launch,
        "gemini305_probe",
        lambda: wrist_camera_launch.UsbProbe(Path("/usb"), 1, True),
    )
    monkeypatch.setattr(wrist_camera_launch, "orbbec_camera_available", lambda: False)
    assert wrist_camera_launch.launch_wrist_camera(context()) == []
    wrist_camera_launch.logger.warning.assert_called_once_with(
        "Gemini 305 detected, but orbbec_camera is unavailable; skipping driver."
    )
    wrist_camera_launch.logger.info.assert_called_once_with(
        "Install ros-jazzy-orbbec-camera in the selected image, then restart."
    )


@pytest.mark.parametrize(
    "setting,value",
    [
        ("wrist_camera_color_width", "invalid"),
        ("wrist_camera_color_height", "0"),
        ("wrist_camera_color_fps", "-1"),
    ],
)
def test_invalid_stream_setting_skips_camera_without_probing_usb(
    wrist_camera_launch, monkeypatch, setting, value
):
    monkeypatch.setattr(
        wrist_camera_launch,
        "gemini305_probe",
        lambda: pytest.fail("invalid stream setting must not probe USB"),
    )
    assert wrist_camera_launch.launch_wrist_camera(context(**{setting: value})) == []
    warning = wrist_camera_launch.logger.warning.call_args.args[0]
    assert setting in warning
    assert repr(value) in warning
    wrist_camera_launch.logger.info.assert_called_once_with(
        "The arm backend remains available. Rebuild after updating "
        "config/wrist_camera.yaml."
    )


def test_enabled_camera_uses_stock_color_only_factory_contract(
    wrist_camera_launch, monkeypatch
):
    monkeypatch.setattr(
        wrist_camera_launch,
        "gemini305_probe",
        lambda: wrist_camera_launch.UsbProbe(Path("/usb"), 1, True),
    )
    monkeypatch.setattr(wrist_camera_launch, "orbbec_camera_available", lambda: True)
    launch_context = default_context(wrist_camera_launch)
    (node,) = wrist_camera_launch.launch_wrist_camera(launch_context)
    assert isinstance(node, Node)
    assert value(launch_context, node.node_package) == "orbbec_camera"
    assert value(launch_context, node.node_executable) == "orbbec_camera_node"
    node._perform_substitutions(launch_context)
    assert node.node_name == "/wrist_mounted_camera/wrist_mounted_camera"
    assert node.expanded_node_namespace == "/wrist_mounted_camera"
    assert wrist_camera_launch.wrist_camera_parameters(launch_context) == {
        "camera_name": "wrist_mounted_camera",
        "serial_number": "",
        "enable_color": True,
        "color_width": 1280,
        "color_height": 800,
        "color_fps": 10,
        "color_format": "MJPG",
        "ae_mode": "colorbased",
        "enable_depth": False,
        "enable_left_ir": False,
        "enable_right_ir": False,
        "enable_point_cloud": False,
        "enable_colored_point_cloud": False,
        "publish_tf": False,
    }
    # Without a color_info_url, Orbbec derives CameraInfo from device intrinsics.
    assert "color_info_url" not in wrist_camera_launch.wrist_camera_parameters(
        launch_context
    )


def test_auto_exposure_meters_the_enabled_color_stream(wrist_camera_launch):
    parameters = wrist_camera_launch.wrist_camera_parameters(context())
    assert parameters["enable_color"] is True
    assert parameters["enable_depth"] is False
    assert parameters["ae_mode"] == "colorbased"
    assert not any("roi" in name for name in parameters)


def test_serial_can_select_a_specific_camera_without_changing_stream_contract(
    wrist_camera_launch,
):
    parameters = wrist_camera_launch.wrist_camera_parameters(
        context(wrist_camera_serial_number="serial-for-test")
    )
    assert parameters["serial_number"] == "serial-for-test"
    assert parameters["color_width"] == 1280
    assert parameters["color_height"] == 800
    assert parameters["color_fps"] == 10
    assert parameters["color_format"] == "MJPG"
    assert parameters["ae_mode"] == "colorbased"


def test_usb_probe_ignores_interface_entries_and_accepts_gemini305(
    wrist_camera_launch, tmp_path
):
    (tmp_path / "1-0:1.0").mkdir()
    device = tmp_path / "gemini305"
    device.mkdir()
    (device / "idVendor").write_text("2BC5\n")
    (device / "idProduct").write_text("0840\n")
    probe = wrist_camera_launch.gemini305_probe(tmp_path)
    assert probe.found
    assert probe.entries == 2
    assert probe.unreadable_entries == 0


def test_usb_probe_counts_unreadable_device_descriptors(
    wrist_camera_launch, monkeypatch, tmp_path
):
    unreadable = tmp_path / "a_unreadable"
    unreadable.mkdir()
    (unreadable / "idVendor").write_text("2bc5\n")
    product = unreadable / "idProduct"
    product.write_text("0840\n")
    gemini = tmp_path / "z_gemini"
    gemini.mkdir()
    (gemini / "idVendor").write_text("2bc5\n")
    (gemini / "idProduct").write_text("0840\n")
    original_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == product:
            raise PermissionError("descriptor denied")
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    probe = wrist_camera_launch.gemini305_probe(tmp_path)
    assert probe.found
    assert probe.entries == 2
    assert probe.unreadable_entries == 1
    assert probe.unreadable_error is not None


def test_unreadable_descriptor_warns_without_claiming_usb_scan_failure(
    wrist_camera_launch, monkeypatch
):
    probe = wrist_camera_launch.UsbProbe(
        Path("/usb"),
        5,
        False,
        unreadable_entries=2,
        unreadable_path=Path("/usb/1-2"),
        unreadable_error=PermissionError("descriptor denied"),
    )
    monkeypatch.setattr(wrist_camera_launch, "gemini305_probe", lambda: probe)
    assert wrist_camera_launch.launch_wrist_camera(context()) == []
    warning = wrist_camera_launch.logger.warning.call_args.args[0]
    assert "2 unreadable" in warning
    assert "descriptor denied" in warning
    assert "USB scan failed" not in warning


@pytest.mark.parametrize("vendor,product", [("2bc5", "0841"), ("1234", "0840")])
def test_usb_probe_rejects_other_devices(
    wrist_camera_launch, tmp_path, vendor, product
):
    device = tmp_path / "other_camera"
    device.mkdir()
    (device / "idVendor").write_text(vendor + "\n")
    (device / "idProduct").write_text(product + "\n")
    assert not wrist_camera_launch.gemini305_probe(tmp_path).found
    assert not wrist_camera_launch.gemini305_probe(tmp_path / "missing").found


@pytest.mark.parametrize(
    "setting",
    [
        "wrist_camera_color_width",
        "wrist_camera_color_height",
        "wrist_camera_color_fps",
    ],
)
def test_noninteger_stream_setting_names_the_invalid_argument(
    wrist_camera_launch, setting
):
    with pytest.raises(ValueError, match=setting):
        wrist_camera_launch.wrist_camera_parameters(context(**{setting: "invalid"}))


@pytest.mark.parametrize("value", ["0", "-1"])
@pytest.mark.parametrize(
    "setting",
    [
        "wrist_camera_color_width",
        "wrist_camera_color_height",
        "wrist_camera_color_fps",
    ],
)
def test_nonpositive_stream_setting_names_the_invalid_argument(
    wrist_camera_launch, setting, value
):
    with pytest.raises(ValueError, match=setting):
        wrist_camera_launch.wrist_camera_parameters(context(**{setting: value}))


@pytest.mark.parametrize(
    "setting,parameter",
    [
        ("wrist_camera_color_width", "color_width"),
        ("wrist_camera_color_height", "color_height"),
        ("wrist_camera_color_fps", "color_fps"),
    ],
)
def test_one_is_the_smallest_valid_stream_setting(
    wrist_camera_launch, setting, parameter
):
    assert (
        wrist_camera_launch.wrist_camera_parameters(context(**{setting: "1"}))[
            parameter
        ]
        == 1
    )
