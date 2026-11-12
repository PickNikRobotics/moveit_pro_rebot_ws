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
"""SIM-ONLY: answer the RobStride driver's engage and relax services in MuJoCo.

MuJoCo actuators are always powered, so there is no torque transition to make.
Engage succeeds without commanding anything, which lets the engage-first
Objectives (Teleoperate, Mirror, Return HOME) run in the twin. Relax refuses:
the twin cannot turn torque off, and a fake success would let Relax at HOME
pass without testing anything. No gain service is offered; the twin's
gain Objectives are overrides that state what they do not test.
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

ENGAGE = "/robstride_hardware_interface/engage"
RELAX = "/robstride_hardware_interface/relax"


class SimTorqueServices(Node):
    def __init__(self):
        super().__init__("sim_torque_services")
        self.create_service(Trigger, ENGAGE, self.engage)
        self.create_service(Trigger, RELAX, self.relax)
        self.get_logger().info(
            "SIM-ONLY torque services: engage succeeds without a torque "
            "transition, relax is refused."
        )

    def engage(self, request, response):
        del request
        response.success = True
        response.message = "MuJoCo is always powered; no torque transition"
        self.get_logger().info(f"engage: {response.message}")
        return response

    def relax(self, request, response):
        del request
        response.success = False
        response.message = "MuJoCo cannot turn torque off; relax refused"
        self.get_logger().warning(f"relax: {response.message}")
        return response


def main():
    rclpy.init()
    node = SimTorqueServices()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
