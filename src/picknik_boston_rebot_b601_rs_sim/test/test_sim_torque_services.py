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
"""The twin's torque services: engage succeeds, relax refuses, nothing else."""
import importlib.util
import time
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger


@pytest.fixture
def services():
    rclpy.init()
    script = (
        Path(get_package_prefix("picknik_boston_rebot_b601_rs_sim"))
        / "lib/picknik_boston_rebot_b601_rs_sim"
    )
    spec = importlib.util.spec_from_file_location(
        "sim_torque_services", script / "sim_torque_services.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.SimTorqueServices()
    client_node = rclpy.create_node("sim_torque_services_client")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)

    def call(name):
        client = client_node.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        assert future.done()
        return future.result()

    yield node, client_node, call
    executor.shutdown()
    client_node.destroy_node()
    node.destroy_node()
    rclpy.try_shutdown()


def test_engage_succeeds_and_relax_refuses_every_time(services):
    _, _, call = services
    for _ in range(2):
        engaged = call("/robstride_hardware_interface/engage")
        assert engaged.success
        assert "no torque transition" in engaged.message
        relaxed = call("/robstride_hardware_interface/relax")
        assert not relaxed.success
        assert "relax refused" in relaxed.message


def test_only_engage_and_relax_are_offered(services):
    node, client_node, _ = services
    deadline = time.monotonic() + 5.0
    offered = set()
    while len(offered) < 2 and time.monotonic() < deadline:
        time.sleep(0.1)
        offered = {
            name
            for name, _ in client_node.get_service_names_and_types_by_node(
                node.get_name(), node.get_namespace()
            )
            if name.startswith("/robstride_hardware_interface/")
        }
    # No gain (set_data) service: the twin never accepts a gain write.
    assert offered == {
        "/robstride_hardware_interface/engage",
        "/robstride_hardware_interface/relax",
    }
