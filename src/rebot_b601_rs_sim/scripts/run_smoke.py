#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Simulation-only MoveIt Pro acceptance with recorded joint/simulator state.

Run in a named rebot_b601_rs_sim Runtime's sourced ROS environment. No direct
trajectory-controller commands: every motion enters through /do_objective and
JointJog/PoseJog's safety-filtered input topics. Fails closed on another plugin.
"""

import argparse
import json
import os
from pathlib import Path
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.msg import JointJog, JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
import mujoco
import numpy as np
from moveit_pro_controllers_msgs.msg import VelocityForceCommand
from moveit_studio_internal_msgs.msg import MujocoQpos, MujocoQposSchema
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from moveit_studio_sdk_msgs.msg import BehaviorParameter
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState


JOINTS = [f"joint{i}" for i in range(1, 8)]
NOMINAL_OPEN_JAW = 5.410520681182422
TRAJECTORY = "joint_trajectory_admittance_controller"
# The Objective layer reports BT execution failure as FAILURE, not CONTROL_FAILED.
OBJECTIVE_FAILURE = 99999


class Smoke(Node):
    def __init__(self, output, reset_cycles=0):
        super().__init__("rebot_sim_smoke")
        self.output = output
        self.reset_cycles = reset_cycles
        self.trace = (output / "state.jsonl").open("w")
        self.phase = "preflight"
        self.joints = None
        self.qpos = None
        self.active = False
        self.schema = None
        self.samples = 0
        self.joint_samples = 0
        self.result = {"reset_snapbacks": 0}
        self.create_subscription(JointState, "/joint_states", self.on_joints, 100)
        self.create_subscription(
            MujocoQpos,
            "/mujoco/qpos_stamped",
            self.on_qpos,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            MujocoQposSchema,
            "/mujoco/qpos_schema",
            self.on_schema,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            f"/{TRAJECTORY}/controller_state",
            lambda message: self.record("trajectory_reference", message),
            100,
        )
        self.create_subscription(
            GoalStatusArray,
            "/do_objective/_action/status",
            self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.action = ActionClient(self, DoObjectiveSequence, "/do_objective")
        mujoco.mj_loadAllPluginLibraries(
            str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
        )
        self.model = mujoco.MjModel.from_xml_path(
            str(
                Path(get_package_share_directory("rebot_b601_rs_sim"))
                / "description/scene.xml"
            )
        )

    def record(self, kind, message):
        self.trace.write(
            json.dumps(
                {
                    "wall": time.time(),
                    "phase": self.phase,
                    "kind": kind,
                    "message": message_to_ordereddict(message),
                }
            )
            + "\n"
        )
        self.trace.flush()

    def on_joints(self, message):
        self.joints = message
        self.joint_samples += 1
        self.record("joint_states", message)

    def on_qpos(self, message):
        self.qpos = message
        self.samples += 1
        self.record("mujoco_qpos", message)

    def on_schema(self, message):
        self.schema = message
        self.record("mujoco_schema", message)

    def on_status(self, message):
        self.active = any(
            status.status
            in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            )
            for status in message.status_list
        )

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"{self.phase}: timed out after {timeout}s")
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, service, kind):
        client = self.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=15):
            raise RuntimeError(f"Missing {service}")
        future = client.call_async(kind.Request())
        self.wait(future.done, 15)
        response = future.result()
        self.destroy_client(client)
        return response

    def state(self):
        assert self.joints is not None and self.qpos is not None
        values = dict(zip(self.joints.name, self.joints.position, strict=True))
        assert all(name in values for name in JOINTS)
        assert np.isfinite(list(values.values())).all()
        age = self.get_clock().now().nanoseconds / 1e9 - (
            self.joints.header.stamp.sec + self.joints.header.stamp.nanosec / 1e9
        )
        assert 0 <= age < 0.5, f"Stale measured state: {age:.3f}s"
        qpos_age = self.get_clock().now().nanoseconds / 1e9 - (
            self.qpos.header.stamp.sec + self.qpos.header.stamp.nanosec / 1e9
        )
        assert 0 <= qpos_age < 0.5, f"Stale simulation state: {qpos_age:.3f}s"
        assert len(self.qpos.qpos) == self.model.nq
        assert (
            self.schema is not None and self.schema.schema_hash == self.qpos.schema_hash
        )
        return np.array([values[name] for name in JOINTS])

    def tree(self, body):
        return f'<root BTCPP_format="4" main_tree_to_execute="Sim Smoke"><BehaviorTree ID="Sim Smoke">{body}</BehaviorTree></root>'

    def goal(self, name=None, xml=None, keyframe=None):
        goal = DoObjectiveSequence.Goal()
        goal.objective_name = name or ""
        goal.objective_xml_string = xml or ""
        if keyframe is not None:
            parameter = BehaviorParameter()
            parameter.description.name = "keyframe_name"
            parameter.description.type = parameter.description.TYPE_STRING
            parameter.string_value = keyframe
            goal.parameter_overrides = [parameter]
        future = self.action.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        assert handle.accepted, f"{self.phase}: Objective rejected"
        result = handle.get_result_async()
        return handle, result

    def finish(self, handle, future, timeout=60, expected=1):
        try:
            self.wait(future.done, timeout)
        except BaseException:
            cancellation = handle.cancel_goal_async()
            self.wait(cancellation.done)
            self.wait(future.done)
            raise
        response = future.result()
        self.result[self.phase + "_objective"] = message_to_ordereddict(response.result)
        assert response.result.error_code.val == expected, response.result.error_message
        self.spin(0.5)

    def run(self, name=None, xml=None, keyframe=None, expected=1):
        handle, future = self.goal(name, xml, keyframe)
        self.finish(handle, future, expected=expected)

    def reset(self, keyframe="default"):
        desired = self.model.key(keyframe).qpos[:7]
        self.run("Reset MuJoCo Sim", keyframe=keyframe)
        self.spin(0.5)
        error = max(abs(self.state() - desired))
        if error >= 0.01:
            # Known 10.2.0-rc10 MujocoSystem race: a successful reset can keep
            # commanding the pre-reset pose. Count it and retry exactly once.
            self.result["reset_snapbacks"] = self.result.get("reset_snapbacks", 0) + 1
            self.result.setdefault("reset_snapback_phases", []).append(self.phase)
            self.run("Reset MuJoCo Sim", keyframe=keyframe)
            self.spin(0.5)
            error = max(abs(self.state() - desired))
        assert error < 0.01, f"Reset error {error:.6f}rad"
        assert max(abs(np.array(self.joints.velocity))) < 0.02
        self.result[self.phase + "_reset_error_rad"] = float(error)

    def jog(self, pose=False):
        controller = (
            "velocity_force_controller" if pose else "joint_velocity_controller"
        )
        behavior = (
            f'<Action ID="PoseJog" controller_names="{controller}" planning_group_names="manipulator" collision_check_params="[{{override_scene_link_padding: true, link_padding: 0.0}}]" />'
            if pose
            else f'<Action ID="JointJog" controller_name="{controller}" collision_check_params="[{{override_scene_link_padding: true, link_padding: 0.0}}]" />'
        )
        xml = self.tree(
            f'<Control ID="Sequence"><Action ID="SwitchController" activate_controllers="{controller}" automatic_deactivation="true" />'
            f'<Decorator ID="ForceSuccess"><Decorator ID="Timeout" msec="5000">{behavior}</Decorator></Decorator>'
            f'<Action ID="SwitchController" activate_controllers="{TRAJECTORY}" automatic_deactivation="true" /></Control>'
        )
        handle, future = self.goal(xml=xml)
        kind = VelocityForceCommand if pose else JointJog
        # These are the same safety-filtered input topics used by the Desktop.
        topic = f"/{'pose_jog' if pose else 'joint_jog'}/manipulator"
        publisher = self.create_publisher(kind, topic, 10)
        try:
            self.wait(lambda: publisher.get_subscription_count() > 0)
            before = self.state()
            end = time.monotonic() + 0.75
            while time.monotonic() < end:
                message = kind()
                message.header.stamp = self.get_clock().now().to_msg()
                if pose:
                    message.control_frame_id = "base_link"
                    message.velocity_controlled_axes.x = True
                    message.twist.linear.x = 0.01
                else:
                    message.joint_names = ["joint1"]
                    message.velocities = [0.05]
                publisher.publish(message)
                self.spin(0.02)
            self.finish(handle, future)
        finally:
            if not future.done():
                cancellation = handle.cancel_goal_async()
                self.wait(cancellation.done)
                self.wait(future.done)
            self.destroy_publisher(publisher)
        after = self.state()
        if pose:

            def tip(q):
                data = mujoco.MjData(self.model)
                data.qpos[:7] = q
                data.qpos[7:] = 0.008 * q[6]
                mujoco.mj_forward(self.model, data)
                return data.body("gripper_end").xpos.copy()

            delta = tip(after) - tip(before)
            assert 0.001 < delta[0] < 0.02, delta
            assert np.linalg.norm(delta[1:]) < 0.005, delta
            self.result["pose_jog_tip_delta_m"] = delta.tolist()
        else:
            assert 0.005 < after[0] - before[0] < 0.1
            assert max(abs((after - before)[1:6])) < 0.01
            self.result["joint_jog_delta_rad"] = (after - before).tolist()

    def execute(self):
        if os.environ.get("MOVEIT_CONFIG_PACKAGE") != "rebot_b601_rs_sim":
            raise RuntimeError("Run only in the selected rebot_b601_rs_sim environment")
        components = self.call(
            "/controller_manager/list_hardware_components", ListHardwareComponents
        )
        if len(components.component) != 1 or (
            components.component[0].plugin_name != "picknik_mujoco_ros/MujocoSystem"
            or components.component[0].state.label != "active"
        ):
            raise RuntimeError("Refusing motion: expected one active MujocoSystem")
        self.result["hardware"] = message_to_ordereddict(components)
        assert self.action.wait_for_server(timeout_sec=60)
        self.spin(1)
        self.wait(
            lambda: self.joints is not None
            and self.qpos is not None
            and self.schema is not None
        )
        if self.active:
            raise RuntimeError("Refusing to preempt an active Objective")
        self.state()
        self.phase = "reset"
        self.reset()
        self.phase = "move_to_raised"
        self.run("Move to Raised")
        raised = self.state()
        assert max(abs(raised[:6] - [0, 0.65, 0.85, 0.3, 0, 0])) < 0.02
        self.result["raised_state_rad"] = raised.tolist()
        self.phase = "joint_jog"
        self.jog()
        self.phase = "pose_jog"
        self.jog(pose=True)
        self.phase = "open_gripper"
        self.run("Open Gripper")
        assert abs(self.state()[6] - NOMINAL_OPEN_JAW) < 0.02
        self.phase = "close_gripper"
        self.run("Close Gripper")
        assert abs(self.state()[6]) < 0.02
        self.phase = "contact_fixture"
        self.reset("contact_smoke")
        self.phase = "contact_close"
        # The fixture deliberately blocks the commanded close: JTAC must abort
        # rather than report a false successful grasp/closed state.
        self.run("Close Gripper", expected=OBJECTIVE_FAILURE)
        assert (
            "Path tolerance violated on joint7"
            in self.result["contact_close_objective"]["error_message"]
        )
        self.spin(0.5)
        measured = self.state()
        assert measured[6] > 2.9, measured
        data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(
            self.model, data, self.model.key("contact_smoke").id
        )
        data.qpos[:] = self.qpos.qpos
        mujoco.mj_forward(self.model, data)
        pairs = [
            sorted(self.model.geom(g).name for g in contact.geom)
            for contact in data.contact[: data.ncon]
        ]
        assert len(pairs) == 2
        assert all("contact_block_geom" in pair for pair in pairs)
        for jaw in ("gripper_joint1", "gripper_joint2"):
            assert abs(data.joint(jaw).qpos[0] - 0.008 * measured[6]) < 0.0002
        self.result["contact_measured_motor_rad"] = float(measured[6])
        self.result["contact_qpos"] = list(self.qpos.qpos)
        self.result["contact_keyframe_mpos"] = self.model.key(
            "contact_smoke"
        ).mpos.tolist()
        self.result["contact_keyframe_mquat"] = self.model.key(
            "contact_smoke"
        ).mquat.tolist()
        self.result["contact_geometry_replay_pairs"] = pairs
        self.phase = "contact_open"
        self.run("Open Gripper")
        assert abs(self.state()[6] - NOMINAL_OPEN_JAW) < 0.02
        self.phase = "final_reset"
        self.reset()
        # A snapped-back reset holds the pre-reset pose, so alternate keyframes
        # whose arm and jaw poses differ and check each measured hold.
        for cycle in range(self.reset_cycles):
            for keyframe in ("contact_smoke", "default"):
                self.phase = f"reset_cycle_{cycle}_{keyframe}"
                self.reset(keyframe)
        self.result["reset_cycles"] = self.reset_cycles
        self.phase = "invalid_reset"
        before = self.state()
        self.run(
            "Reset MuJoCo Sim", keyframe="missing_keyframe", expected=OBJECTIVE_FAILURE
        )
        assert (
            "not found"
            in self.result["invalid_reset_objective"]["error_message"].lower()
        )
        assert max(abs(self.state() - before)) < 0.01
        controllers = self.call("/controller_manager/list_controllers", ListControllers)
        active = {
            controller.name
            for controller in controllers.controller
            if controller.state == "active"
        }
        assert active == {"joint_state_broadcaster", TRAJECTORY}
        self.result["controllers_final"] = message_to_ordereddict(controllers)
        self.result["schema_hash"] = self.schema.schema_hash
        self.result["qpos_publish_rate_hz"] = self.schema.publish_rate_hz
        self.result["joint_state_samples"] = self.joint_samples
        self.result["qpos_samples"] = self.samples
        self.result["passed"] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--reset-cycles",
        type=int,
        default=0,
        help="extra alternating contact_smoke/default reset pairs (reset stress)",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    smoke = Smoke(args.output, args.reset_cycles)
    try:
        smoke.execute()
    except BaseException as error:
        smoke.result["passed"] = False
        smoke.result["failure_phase"] = smoke.phase
        smoke.result["failure"] = str(error)
        raise
    finally:
        (args.output / "summary.json").write_text(
            json.dumps(smoke.result, indent=2) + "\n"
        )
        smoke.trace.close()
        smoke.destroy_node()
        rclpy.shutdown()
    print(json.dumps(smoke.result, indent=2))


if __name__ == "__main__":
    main()
