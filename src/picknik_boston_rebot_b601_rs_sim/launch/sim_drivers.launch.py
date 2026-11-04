#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Launch simulated cameras and the explicitly enabled cube proxy, never camera drivers."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import get_config_package, load_system_config
import yaml


def configured_nodes(config):
    share = Path(get_package_share_directory("picknik_boston_rebot_b601_rs_sim"))
    params = {
        k: v
        for item in config.hardware.robot_description.urdf_params
        for k, v in item.items()
    }
    if params.get("hardware_interface") != "mujoco":
        raise ValueError("The Boston simulation launch requires the MuJoCo backend")
    # Each scene freezes its own proxy decision together with its model.
    model_share = Path(get_package_share_directory(params["mujoco_model_package"]))
    layout = yaml.safe_load((model_share / "description/bench_layout.yaml").read_text())
    # The episode recorder's training config records the compressed topics.
    cameras = Node(
        package="picknik_boston_rebot_b601_rs_sim",
        executable="sim_compressed_cameras.py",
        output="screen",
    )
    # Inherit exactly the base YAML leader selection, without real camera drivers.
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw"))
                / "launch/rebot_leader.launch.py"
            )
        )
    )
    if not layout.get("sim_grasp_proxy", False):
        return [cameras, leader]
    return [
        cameras,
        leader,
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "sim_grasp_proxy_controller",
                "--param-file",
                str(share / "config/sim_grasp_proxy.yaml"),
            ],
            output="screen",
        ),
        Node(
            package="picknik_boston_rebot_b601_rs_sim",
            executable="sim_grasp_proxy.py",
            output="screen",
        ),
    ]


def launch_configured_nodes(context):
    del context
    config = load_system_config(get_config_package(), Path(os.environ["USER_WS"]))[0]
    return configured_nodes(config)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_configured_nodes)])
