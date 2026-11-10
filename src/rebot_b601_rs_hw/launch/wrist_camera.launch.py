# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Start one stock Orbbec Gemini 305 wrist camera when attached."""

from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


logger = get_logger("rebot.wrist_camera")
USB_ROOT = Path("/sys/bus/usb/devices")


@dataclass(frozen=True)
class UsbProbe:
    """Result of the read-only Gemini 305 USB descriptor scan."""

    root: Path
    entries: int
    found: bool
    unreadable_entries: int = 0
    error: OSError | None = None
    unreadable_path: Path | None = None
    unreadable_error: OSError | None = None


def gemini305_probe(usb_root=USB_ROOT):
    """Read USB descriptors without opening or claiming a camera."""
    try:
        devices = sorted(usb_root.iterdir())
    except OSError as error:
        return UsbProbe(usb_root, 0, False, error=error)
    unreadable_entries = 0
    unreadable_path = None
    unreadable_error = None
    for device in devices:
        try:
            vendor = (device / "idVendor").read_text().strip().lower()
            product = (device / "idProduct").read_text().strip().lower()
        except FileNotFoundError:
            # USB interface entries have no device descriptors.
            continue
        except OSError as error:
            unreadable_entries += 1
            unreadable_path = unreadable_path or device
            unreadable_error = unreadable_error or error
            continue
        if (vendor, product) == ("2bc5", "0840"):
            return UsbProbe(
                usb_root,
                len(devices),
                True,
                unreadable_entries,
                unreadable_path=unreadable_path,
                unreadable_error=unreadable_error,
            )
    return UsbProbe(
        usb_root,
        len(devices),
        False,
        unreadable_entries,
        unreadable_path=unreadable_path,
        unreadable_error=unreadable_error,
    )


def orbbec_camera_available():
    """Report whether the selected ROS environment provides the driver package."""
    try:
        get_package_prefix("orbbec_camera")
    except PackageNotFoundError:
        return False
    return True


def displayed_value(value, limit=32):
    """Render a bounded configuration value for an operator-facing diagnostic."""
    rendered = repr(value)
    return rendered if len(rendered) <= limit else rendered[: limit - 3] + "..."


def error_reason(error):
    """Preserve the useful OS error text in a short diagnostic."""
    return error.strerror or str(error) or type(error).__name__


def wrist_camera_enabled(context):
    """Resolve the camera switch without silently accepting misspellings."""
    value = LaunchConfiguration("enable_wrist_camera").perform(context)
    normalized = value.lower()
    if normalized not in ("true", "false"):
        raise ValueError(
            "enable_wrist_camera in wrist_camera.yaml must be true or false "
            f"(got {displayed_value(value)})"
        )
    return normalized == "true"


def camera_stream_value(context, name):
    """Parse a named integer stream setting with an actionable error."""
    value = LaunchConfiguration(name).perform(context)
    try:
        parsed = int(value)
    except ValueError as error:
        raise ValueError(
            f"{name} in wrist_camera.yaml must be an integer "
            f"(got {displayed_value(value)})"
        ) from error
    if parsed <= 0:
        raise ValueError(
            f"{name} in wrist_camera.yaml must be positive "
            f"(got {displayed_value(value)})"
        )
    return parsed


def wrist_camera_parameters(context):
    """Resolve the stock color-only driver contract."""
    width = camera_stream_value(context, "wrist_camera_color_width")
    height = camera_stream_value(context, "wrist_camera_color_height")
    fps = camera_stream_value(context, "wrist_camera_color_fps")
    return {
        # camera_name supplies the frame prefix used in image CameraInfo.
        "camera_name": "wrist_mounted_camera",
        "serial_number": LaunchConfiguration("wrist_camera_serial_number").perform(
            context
        ),
        "enable_color": True,
        "color_width": width,
        "color_height": height,
        "color_fps": fps,
        "color_format": "MJPG",
        # Meter color AE on the color image: with depth disabled, the default
        # depth-based AE stopped regulating and the image washed out (2026-11-10).
        "ae_mode": "colorbased",
        "enable_depth": False,
        "enable_left_ir": False,
        "enable_right_ir": False,
        "enable_point_cloud": False,
        "enable_colored_point_cloud": False,
        # The nominal robot frame is the sole TF authority for this mount.
        "publish_tf": False,
    }


def launch_wrist_camera(context):
    """Construct the driver only for an enabled, attached camera."""
    try:
        if not wrist_camera_enabled(context):
            return []
        parameters = wrist_camera_parameters(context)
    except ValueError as error:
        logger.warning(f"Invalid config: {error}; skipping camera.")
        logger.info(
            "The arm backend remains available. Rebuild after updating "
            "config/wrist_camera.yaml."
        )
        return []
    probe = gemini305_probe()
    if not probe.found:
        if probe.error:
            logger.warning(
                f"USB scan failed at {probe.root}: {error_reason(probe.error)}. "
                "Skipping camera."
            )
            logger.info(
                "Ensure /sys/bus/usb/devices is readable and visible in the container."
            )
        elif probe.unreadable_entries:
            entry = probe.unreadable_path or probe.root
            logger.warning(
                f"USB scan: {probe.unreadable_entries} unreadable at "
                f"{entry.name} ({error_reason(probe.unreadable_error)})."
            )
            logger.info(
                "Ensure the device descriptors are readable in /sys/bus/usb/devices."
            )
        else:
            logger.warning(
                f"Scanned {probe.entries} USB entries; no Gemini 305 device "
                "2bc5:0840 found."
            )
        logger.info(
            "The arm backend remains available. See README: Gemini 305 wrist camera."
        )
        logger.info(
            "To disable probing, set enable_wrist_camera: false in "
            "config/wrist_camera.yaml, rebuild, then restart."
        )
        return []
    if not orbbec_camera_available():
        logger.warning(
            "Gemini 305 detected, but orbbec_camera is unavailable; skipping driver."
        )
        logger.info(
            "Install ros-jazzy-orbbec-camera in the selected image, then restart."
        )
        return []
    return [
        Node(
            package="orbbec_camera",
            executable="orbbec_camera_node",
            name="wrist_mounted_camera",
            namespace="wrist_mounted_camera",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description():
    path = (
        Path(get_package_share_directory("rebot_b601_rs_hw"))
        / "config/wrist_camera.yaml"
    )
    defaults = yaml.safe_load(path.read_text())
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name,
                default_value=(
                    str(value).lower() if isinstance(value, bool) else str(value)
                ),
            )
            for name, value in defaults.items()
        ]
        + [OpaqueFunction(function=launch_wrist_camera)]
    )
