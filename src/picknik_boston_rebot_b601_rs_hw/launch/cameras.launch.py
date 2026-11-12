# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Start the optional office OAK-D scene camera without a wrist-camera driver."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


logger = get_logger("rebot.optional_cameras")


def scene_camera_detected(usb_root=Path("/sys/bus/usb/devices")):
    """Inspect USB descriptors without opening or claiming the OAK-D."""
    try:
        devices = list(usb_root.iterdir())
    except OSError:
        return False
    for device in devices:
        try:
            vendor = (device / "idVendor").read_text().strip().lower()
            product = (device / "idProduct").read_text().strip().lower()
        except OSError:
            continue
        if vendor == "03e7" and product in ("2485", "f63b"):
            return True
    return False


def launch_cameras(context):
    """Resolve scene settings only after the cameras-on condition has passed."""
    if LaunchConfiguration("enable_scene_camera").perform(context).lower() != "true":
        return []
    if not scene_camera_detected():
        logger.warning(
            "Optional scene camera (Luxonis OAK-D) not found on USB; skipping it. "
            "Backend and leader remain available. Configure another driver separately "
            "or disable enable_scene_camera."
        )
        return []

    width = int(LaunchConfiguration("image_width").perform(context))
    scene_height = int(LaunchConfiguration("scene_image_height").perform(context))
    fps = int(LaunchConfiguration("framerate").perform(context))
    quality = int(LaunchConfiguration("scene_mjpeg_quality").perform(context))
    depth = (
        LaunchConfiguration("scene_aligned_depth").perform(context).lower() == "true"
    )
    if min(width, scene_height, fps) <= 0 or not 1 <= quality <= 100:
        raise ValueError(
            "Camera dimensions/fps must be positive; MJPEG quality must be 1–100"
        )
    # The scene camera publishes ISP output at a fixed 1/2 scale, which ignores
    # i_width/i_height, so any other size would be silently replaced by 640x400.
    if width != 640 or scene_height != 400:
        raise ValueError(
            "DepthAI ISP output requires image_width=640 and scene_image_height=400"
        )

    # Stereo depth aligned to CAM_A takes the colour socket's size; with
    # i_rs_compat it publishes depth/image_rect_raw in the colour optical frame.
    depth_parameters = (
        {
            "depth.i_align_depth": True,
            "depth.i_board_socket_id": 0,
        }
        if depth
        else {}
    )
    nodes = [
        Node(
            package="depthai_ros_driver",
            executable="camera_node",
            name="scene_camera",
            namespace="/",
            output="screen",
            parameters=[
                {
                    "camera.i_mx_id": ParameterValue(
                        LaunchConfiguration("scene_serial_no"), value_type=str
                    ),
                    "camera.i_usb_speed": "HIGH",
                    "camera.i_pipeline_type": "RGBD" if depth else "RGB",
                    "camera.i_nn_type": "none",
                    "pipeline_gen.i_enable_imu": False,
                    "pipeline_gen.i_enable_sync": False,
                    "camera.i_enable_ir": False,
                    "camera.i_publish_tf_from_calibration": False,
                    "camera.i_rs_compat": True,
                    "color.i_resolution": "800P",
                    # Take the ISP output at an explicit 1/2 scale rather than
                    # the video path's crop. The video path at 640x480 keeps only
                    # the central 640x480 of the 1280x800 sensor - 59.1 of its
                    # 97.1 degrees. The ratio must be stated: left to its own
                    # default the ISP picks 2/3 and then centre-crops to the
                    # requested size, which covers only the central 960x600
                    # (80.7 deg). At 1/2 the whole sensor maps to 640x400 at
                    # offset (0,0), measured on the bench, so the full field of
                    # view survives and the driver's width-proportional K and P
                    # come out as exactly the native intrinsics halved - correct,
                    # with no CameraInfo correction needed anywhere.
                    "color.i_set_isp_scale": True,
                    "color.i_isp_num": 1,
                    "color.i_isp_den": 2,
                    "color.i_output_isp": True,
                    "color.i_width": width,
                    "color.i_height": scene_height,
                    "color.i_fps": float(fps),
                    # DepthAI v2's MJPEG decoder uses OpenCV's BGR output.
                    # RGB would label those bytes rgb8 without swapping channels.
                    "color.i_color_order": "BGR",
                    "color.r_set_luma_denoise": True,
                    "color.r_luma_denoise": 0,
                    "color.r_set_chroma_denoise": True,
                    "color.r_chroma_denoise": 0,
                    "color.r_set_sharpness": True,
                    "color.r_sharpness": 0,
                    "color.i_low_bandwidth": True,
                    "color.i_low_bandwidth_profile": 4,  # DepthAI MJPEG
                    "color.i_low_bandwidth_quality": quality,
                    "color.i_publish_compressed": False,
                    "color.i_enable_preview": False,
                    "color.i_enable_lazy_publisher": False,
                    **depth_parameters,
                }
            ],
        )
    ]
    if depth:
        depth_topic = "/scene_camera/depth/image_rect_raw"
        scale = float(LaunchConfiguration("scene_depth_scale").perform(context))
        if not 0.5 <= scale <= 1.5:
            raise ValueError("scene_depth_scale must be between 0.5 and 1.5")
        if scale != 1.0:
            nodes.append(
                Node(
                    package="picknik_boston_rebot_b601_rs_hw",
                    executable="scale_depth.py",
                    name="scene_depth_scale",
                    output="screen",
                    parameters=[{"scale": scale}],
                    remappings=[
                        ("depth", depth_topic),
                        ("depth_scaled", "/scene_camera/depth/image_scaled"),
                    ],
                )
            )
            depth_topic = "/scene_camera/depth/image_scaled"
        nodes.append(
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="scene_camera_points",
                output="screen",
                remappings=[
                    ("image_rect", depth_topic),
                    ("camera_info", "/scene_camera/depth/camera_info"),
                    ("points", "/scene_camera/depth/color/points"),
                ],
            )
        )
    return nodes


def generate_launch_description():
    path = (
        Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw"))
        / "config/cameras.yaml"
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
        + [
            OpaqueFunction(
                function=launch_cameras,
                condition=IfCondition(LaunchConfiguration("enable_cameras")),
            )
        ]
    )
