#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Operator-workflow smoke sequence for the rebot_b601_rs_mock configuration.

Run against a live mock runtime (``moveit_pro run --config-package
rebot_b601_rs_mock``) from a shell in the runtime container. The script acts as
the web UI: it starts the ``Teleoperate`` Objective, answers the
``/do_teleoperate`` bridge (mode, planning groups and controllers), publishes
the same jog topics the UI publishes, and records ``/joint_states`` for every
check. It refuses to run unless the only hardware plugin is
``mock_components/GenericSystem``.

Usage: mock_teleop_smoke.py OUT_DIR   (writes joint_states.jsonl, summary.json)
Exit status is non-zero when any check fails.
"""
import json
import math
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import tf2_ros
from builtin_interfaces.msg import Time
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from geometry_msgs.msg import PoseStamped
from moveit_pro_controllers_msgs.msg import VelocityForceCommand
from moveit_studio_agent_msgs.msg import Json
from moveit_studio_agent_msgs.srv import StoreJointState, StorePose
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARM = [f"joint{i}" for i in range(1, 7)]
ALL = ARM + ["joint7"]
JVC_ARM = "joint_velocity_controller"
JVC_GRIP = "gripper_joint_velocity_controller"
VFC = "velocity_force_controller"
JTAC = "joint_trajectory_admittance_controller"
OPEN_Q, CLOSED_Q = 5.410520681182422, 0.0  # Gripper Open / Gripper Closed waypoints
J_LIMITS = {"joint2": (-0.02, 3.543018381548), "joint3": (-0.01, 4.181336912266)}
HZ = 30.0


class Smoke(Node):
    def __init__(self):
        super().__init__("mock_teleop_smoke")
        cb = ReentrantCallbackGroup()
        self.q = {}
        self.samples = []  # (t, phase, {joint: pos}, {joint: vel})
        self.phase = "init"
        self.t0 = time.monotonic()
        self.goal_id = None
        self.checks = []
        self.description = []
        self.create_subscription(
            String,
            "/robot_description",
            lambda m: self.description.append(m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            JointState, "/joint_states", self._js, 100, callback_group=cb
        )
        self.create_subscription(
            Json, "/moveit_pro_ui/do_teleoperate/goal", self._goal, 10
        )
        self.goal_handle = None
        self.create_subscription(
            Json, "/moveit_pro_ui/do_teleoperate/cancel", self._cancel, 10
        )
        self.fb = self.create_publisher(
            Json, "/moveit_pro_ui/do_teleoperate/feedback", 10
        )
        self.res = self.create_publisher(
            Json, "/moveit_pro_ui/do_teleoperate/result", 10
        )
        self.jog = {
            "manipulator": self.create_publisher(
                JointJog, "/joint_jog/manipulator", 10
            ),
            "gripper": self.create_publisher(JointJog, "/joint_jog/gripper", 10),
        }
        self.pose_jog = self.create_publisher(
            VelocityForceCommand, "/pose_jog/manipulator", 10
        )
        self.lc = self.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=cb
        )
        self.lhi = self.create_client(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            callback_group=cb,
        )
        self.store_pose = self.create_client(
            StorePose, "/store_pose", callback_group=cb
        )
        self.store_js = self.create_client(
            StoreJointState, "/store_joint_state", callback_group=cb
        )
        self.objective = ActionClient(
            self, DoObjectiveSequence, "/do_objective", callback_group=cb
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buffer, self)

    # -- recording ---------------------------------------------------------
    def _js(self, m):
        pos = dict(zip(m.name, m.position))
        vel = dict(zip(m.name, m.velocity)) if m.velocity else {}
        self.q.update(pos)
        self.samples.append((time.monotonic() - self.t0, self.phase, pos, vel))

    def _goal(self, m):
        self.goal_id = json.loads(m.data)["goal_id"]

    def check(self, name, ok, **evidence):
        ev = {
            k: (round(v, 5) if isinstance(v, float) else v) for k, v in evidence.items()
        }
        self.checks.append({"phase": self.phase, "check": name, "pass": bool(ok), **ev})
        print(("PASS " if ok else "FAIL ") + f"[{self.phase}] {name} {ev}", flush=True)

    def observe(self, name, **evidence):
        """Record behaviour that is reported, not asserted."""
        ev = {
            k: (round(v, 5) if isinstance(v, float) else v) for k, v in evidence.items()
        }
        self.checks.append({"phase": self.phase, "check": name, "pass": None, **ev})
        print(f"OBSERVE [{self.phase}] {name} {ev}", flush=True)

    def start_teleop(self):
        self.goal_id = None
        goal = DoObjectiveSequence.Goal()
        goal.objective_name = "Teleoperate"
        fut = self.objective.send_goal_async(goal)
        assert self.wait_until(
            lambda: self.goal_id, 30
        ), "no /do_teleoperate goal from the bridge"
        assert self.wait_until(fut.done, 10)
        self.goal_handle = fut.result()
        self.hold_mode(1.0, 3, ["manipulator"], [JTAC])

    def exit_teleop(self):
        """Operator presses Exit: the UI returns a successful result."""
        self.res.publish(
            Json(
                data=json.dumps(
                    {
                        "goal_id": self.goal_id,
                        "result": {"success": True, "reason": "operator exit"},
                    }
                )
            )
        )
        return self.wait_terminal()

    def cancel_teleop(self):
        """Cancel the Objective; the UI acknowledges the cancel envelope."""
        fut = self.goal_handle.cancel_goal_async()
        self.wait_until(fut.done, 15)
        return self.wait_terminal()

    def wait_terminal(self):
        """Final goal status: 4 SUCCEEDED, 5 CANCELED, 6 ABORTED, else unfinished."""
        self.wait_until(lambda: self.goal_handle.status in (4, 5, 6), 15)
        return self.goal_handle.status

    def _cancel(self, m):
        self.res.publish(
            Json(
                data=json.dumps(
                    {
                        "goal_id": json.loads(m.data)["goal_id"],
                        "result": {"success": False, "reason": "canceled"},
                    }
                )
            )
        )

    def mark(self, phase):
        self.phase = phase
        time.sleep(0.05)

    def snap(self):
        return dict(self.q)

    def span(self, start_idx, joint):
        v = [s[2][joint] for s in self.samples[start_idx:] if joint in s[2]]
        return (min(v), max(v)) if v else (math.nan, math.nan)

    # -- helpers -----------------------------------------------------------
    def controllers(self):
        r = self.lc.call(ListControllers.Request())
        return {c.name: (c.state, sorted(c.claimed_interfaces)) for c in r.controller}

    def tool_pose(self):
        t = self.tf_buffer.lookup_transform(
            "base_link", "gripper_end", rclpy.time.Time()
        )
        p, o = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def send_mode(
        self, mode, groups, controllers, tips=("gripper_end",), skip_collisions=False
    ):
        feedback = {
            "current_mode": {"value": mode},
            "planning_groups": list(groups),
            "controllers": list(controllers),
            "tip_links": list(tips),
            "skip_collision_checks": skip_collisions,
            "velocity_scale_factor": 0.5,
            "skip_user_approval": True,
        }
        self.fb.publish(
            Json(data=json.dumps({"goal_id": self.goal_id, "feedback": feedback}))
        )

    def hold_mode(self, seconds, *args, **kw):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send_mode(*args, **kw)
            time.sleep(0.1)

    def jog_joint(self, group, joint, velocity, seconds, release="zero"):
        # JointJog subscribes only after the controller switch; wait for it.
        self.wait_until(lambda: self.jog[group].get_subscription_count() > 0, 10)
        names = ARM if group == "manipulator" else ["joint7"]
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            m = JointJog()
            m.header.stamp = self.get_clock().now().to_msg()
            m.joint_names = names
            m.velocities = [velocity if n == joint else 0.0 for n in names]
            self.jog[group].publish(m)
            time.sleep(1.0 / HZ)
        if release == "zero":  # button-up: the UI sends explicit zero velocities
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                m = JointJog()
                m.header.stamp = self.get_clock().now().to_msg()
                m.joint_names = names
                m.velocities = [0.0] * len(names)
                self.jog[group].publish(m)
                time.sleep(1.0 / HZ)

    def jog_pose(self, linear, angular, seconds, frame="base_link"):
        self.wait_until(lambda: self.pose_jog.get_subscription_count() > 0, 10)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            m = VelocityForceCommand()
            m.header.stamp = self.get_clock().now().to_msg()
            m.control_frame_id = frame
            for a in ("x", "y", "z", "rx", "ry", "rz"):
                setattr(m.velocity_controlled_axes, a, True)
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (
                float(v) for v in linear
            )
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = (
                float(v) for v in angular
            )
            self.pose_jog.publish(m)
            time.sleep(1.0 / HZ)

    def release(self, seconds=1.2):
        """No input: the command timeout must stop motion and hold position."""
        i = len(self.samples)
        time.sleep(seconds)
        q_end = self.snap()
        tail = (
            [s for s in self.samples[i:] if s[0] > self.samples[i][0] + 0.7]
            if len(self.samples) > i
            else []
        )
        drift = (
            max((max(s[2][j] for s in tail) - min(s[2][j] for s in tail)) for j in ALL)
            if tail
            else math.nan
        )
        return q_end, drift

    def wait_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(0.05)
        return False

    def settle(self, timeout=15.0):
        """Wait until all joints are stationary for 0.5 s."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            time.sleep(0.5)
            recent = [
                s for s in self.samples if s[0] > time.monotonic() - self.t0 - 0.5
            ]
            if len(recent) > 3 and all(
                max(s[2][j] for s in recent) - min(s[2][j] for s in recent) < 5e-4
                for j in ALL
            ):
                return True
        return False


def disjoint(controllers):
    claims = [i for v in controllers.values() if v[0] == "active" for i in v[1]]
    return len(claims) == len(set(claims))


def goto(n, positions):
    """Waypoint-button move (mode 3) of the six arm joints, as the UI does."""
    js = JointState()
    js.name, js.position = list(ARM), [float(p) for p in positions]
    n.send_mode(3, ["manipulator"], [JTAC])
    time.sleep(0.5)
    ok = n.store_js.call(StoreJointState.Request(joint_state=js)).success
    n.hold_mode(0.5, 3, ["manipulator"], [JTAC])
    reached = lambda: max(abs(n.q[j] - p) for j, p in zip(ARM, positions)) < 5e-3
    n.wait_until(reached, 60)
    n.settle(10)
    return ok and reached()


def quat_angle(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def main(out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Smoke()
    ex = MultiThreadedExecutor(8)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        run(n)
    finally:
        with open(out / "joint_states.jsonl", "w") as f:
            last = None
            for k, (t, phase, pos, vel) in enumerate(n.samples):
                # 100 Hz source: keep every 5th sample and every phase boundary.
                if k % 5 and phase == last:
                    continue
                last = phase
                f.write(
                    json.dumps(
                        {
                            "t": round(t, 4),
                            "phase": phase,
                            "position": pos,
                            "velocity": vel,
                        }
                    )
                    + "\n"
                )
        failed = [c for c in n.checks if c["pass"] is False]
        summary = {
            "checks": n.checks,
            "passed": sum(c["pass"] is True for c in n.checks),
            "observations": sum(c["pass"] is None for c in n.checks),
            "failed": len(failed),
            "joint_state_samples": len(n.samples),
        }
        (out / "summary.json").write_text(json.dumps(summary, indent=1))
        print(
            f"{summary['passed']} passed, {summary['observations']} observed, {summary['failed']} failed, {summary['joint_state_samples']} joint_state samples"
        )
        rclpy.try_shutdown()
    return 1 if failed else 0


def run(n: Smoke):
    # ---- preconditions: mock only -----------------------------------------
    assert n.wait_until(
        lambda: n.description and len(n.q) >= 7, 30
    ), "no model/joint states"
    plugins = [
        e.text
        for e in ET.fromstring(n.description[-1]).findall(
            "ros2_control/hardware/plugin"
        )
    ]
    if plugins != ["mock_components/GenericSystem"]:
        raise RuntimeError(f"refusing non-mock hardware: {plugins}")
    n.mark("startup")
    c = n.controllers()
    n.check(
        "trajectory controller owns all seven positions at startup",
        c[JTAC][0] == "active" and c[JTAC][1] == [f"{j}/position" for j in ALL],
        claimed=c[JTAC][1],
    )
    n.check(
        "velocity controllers inactive at startup",
        all(c[k][0] == "inactive" for k in (JVC_ARM, JVC_GRIP, VFC)),
    )
    hw = n.lhi.call(ListHardwareInterfaces.Request())
    cmd = sorted(i.name for i in hw.command_interfaces if i.name.endswith("/position"))
    n.check(
        "coupled fingers expose no command interface",
        cmd == sorted(f"{j}/position" for j in ALL),
        position_interfaces=cmd,
    )

    assert n.objective.wait_for_server(30), "objective server unavailable"
    n.start_teleop()

    # ---- joint jog, each of the six axes ----------------------------------
    for joint in ARM:
        n.mark(f"joint_jog_{joint}")
        n.send_mode(1, ["manipulator"], [JVC_ARM])
        n.wait_until(lambda: n.controllers()[JVC_ARM][0] == "active", 5)
        before = n.snap()
        n.jog_joint("manipulator", joint, 0.2, 1.0)
        after, drift = n.release()
        moved = after[joint] - before[joint]
        others = max(abs(after[j] - before[j]) for j in ALL if j != joint)
        n.check(f"{joint} responds to +jog", moved > 0.02, delta=moved)
        n.check(f"{joint} jog moves no other axis", others < 2e-3, max_other=others)
        n.check(f"{joint} holds after input release", drift < 1e-3, drift=drift)
        # Back toward the start, stopping short so J2/J3 stay clear of their lower limits.
        n.jog_joint("manipulator", joint, -0.2, 0.8)
        n.release(1.6)
    c = n.controllers()
    n.check(
        "arm jog: JTAC inactive, arm JVC owns six positions only",
        c[JTAC][0] == "inactive" and c[JVC_ARM][1] == [f"{j}/position" for j in ARM],
        claimed=c[JVC_ARM][1],
    )

    # ---- gripper jog -------------------------------------------------------
    n.mark("gripper_jog")
    n.send_mode(1, ["gripper"], [JVC_GRIP])
    n.wait_until(lambda: n.controllers()[JVC_GRIP][0] == "active", 5)
    before = n.snap()
    c = n.controllers()
    n.check(
        "gripper jog claims joint7 only",
        c[JVC_GRIP][1] == ["joint7/position"],
        claimed=c[JVC_GRIP][1],
    )
    n.check(
        "gripper jog: nothing else claims joint7",
        all("joint7/position" not in v[1] for k, v in c.items() if k != JVC_GRIP),
    )
    n.jog_joint("gripper", "joint7", 1.0, 1.0)
    after, drift = n.release()
    n.check(
        "joint7 responds to +jog",
        after["joint7"] - before["joint7"] > 0.1,
        delta=after["joint7"] - before["joint7"],
    )
    n.check(
        "gripper jog leaves the arm still",
        max(abs(after[j] - before[j]) for j in ARM) < 1e-3,
    )
    n.check("joint7 holds after input release", drift < 1e-3, drift=drift)
    n.jog_joint("gripper", "joint7", -1.0, 1.0)
    n.release(0.8)

    # ---- Cartesian jog -----------------------------------------------------
    n.mark("waypoint_raised")
    n.check("waypoint move reaches Raised", goto(n, [0, 0.65, 0.85, 0.3, 0, 0]))
    n.mark("pose_jog_setup")
    n.send_mode(2, ["manipulator"], [VFC])
    n.wait_until(lambda: n.controllers()[VFC][0] == "active", 5)
    c = n.controllers()
    n.check(
        "pose jog: VFC owns six arm positions, JTAC inactive",
        c[VFC][1] == [f"{j}/position" for j in ARM] and c[JTAC][0] == "inactive",
        claimed=c[VFC][1],
    )
    for axis, vec in (("x", (0.05, 0, 0)), ("y", (0, 0.05, 0)), ("z", (0, 0, 0.05))):
        n.mark(f"pose_jog_{axis}")
        p0, _ = n.tool_pose()
        n.jog_pose(vec, (0, 0, 0), 1.0)
        n.release(1.0)
        p1, _ = n.tool_pose()
        d = [b - a for a, b in zip(p0, p1)]
        k = "xyz".index(axis)
        n.check(
            f"pose jog +{axis} moves gripper_end along {axis}",
            d[k] > 0.01
            and all(abs(d[i]) < 0.35 * abs(d[k]) for i in range(3) if i != k),
            delta_m=[round(x, 4) for x in d],
        )
        n.jog_pose(tuple(-v for v in vec), (0, 0, 0), 1.0)
        n.release(1.0)
    n.mark("pose_jog_rotation")
    p0, o0 = n.tool_pose()
    n.jog_pose((0, 0, 0), (0, 0, 0.3), 1.0, frame="gripper_end")
    n.release(1.0)
    p1, o1 = n.tool_pose()
    n.check(
        "pose jog rotates about the gripper_end origin",
        quat_angle(o0, o1) > 0.05 and math.dist(p0, p1) < 0.01,
        rotation_rad=quat_angle(o0, o1),
        origin_shift_m=math.dist(p0, p1),
    )

    # ---- open / close from jog mode ---------------------------------------
    for label, mode, target in (("open", 6, OPEN_Q), ("close", 7, CLOSED_Q)):
        n.mark(f"{label}_gripper_from_pose_jog")
        n.send_mode(2, ["manipulator"], [VFC])
        n.wait_until(lambda: n.controllers()[VFC][0] == "active", 5)
        before = n.snap()
        n.check(
            f"{label} gripper starts away from the waypoint",
            abs(before["joint7"] - target) > 0.5,
            joint7=before["joint7"],
            target=target,
        )
        n.hold_mode(0.2, mode, ["manipulator"], [VFC])
        n.send_mode(mode, ["manipulator"], [VFC])
        n.wait_until(lambda: abs(n.q["joint7"] - target) < 0.02, 30)
        n.settle()
        after = n.snap()
        n.check(
            f"{label} gripper reaches the waypoint",
            abs(after["joint7"] - target) < 0.02,
            joint7=after["joint7"],
            target=target,
        )
        n.check(
            f"{label} gripper leaves arm joints in place (partial goal)",
            max(abs(after[j] - before[j]) for j in ARM) < 2e-3,
            max_arm_delta=max(abs(after[j] - before[j]) for j in ARM),
        )
        c = n.controllers()
        n.check(
            f"{label} gripper: single command owner per interface",
            disjoint(c),
            active={k: v[1] for k, v in c.items() if v[0] == "active"},
        )
    n.mark("joint_jog_after_gripper")
    n.send_mode(1, ["manipulator"], [JVC_ARM])
    n.wait_until(lambda: n.controllers()[JVC_ARM][0] == "active", 5)
    before = n.snap()
    n.jog_joint("manipulator", "joint1", 0.2, 1.0)
    after, _ = n.release()
    n.check(
        "joint jog works after gripper action",
        after["joint1"] - before["joint1"] > 0.02,
    )
    n.check(
        "no stale-target jump on switch back to jog",
        max(
            abs(after[j] - before[j])
            for j in ["joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        )
        < 2e-3,
    )

    # ---- interactive marker (pose goal) -----------------------------------
    n.mark("interactive_marker")
    p0, o0 = n.tool_pose()
    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x, target.pose.position.y, target.pose.position.z = (
        p0[0],
        p0[1],
        p0[2] + 0.03,
    )
    (
        target.pose.orientation.x,
        target.pose.orientation.y,
        target.pose.orientation.z,
        target.pose.orientation.w,
    ) = o0
    n.send_mode(4, ["manipulator"], [JTAC])
    time.sleep(0.5)
    n.check(
        "store target pose", n.store_pose.call(StorePose.Request(pose=target)).success
    )
    n.hold_mode(0.5, 4, ["manipulator"], [JTAC])
    goal_xyz = (target.pose.position.x, target.pose.position.y, target.pose.position.z)
    n.wait_until(lambda: math.dist(n.tool_pose()[0], goal_xyz) < 0.003, 60)
    n.settle(10)
    p1, o1 = n.tool_pose()
    err = math.dist(
        p1, (target.pose.position.x, target.pose.position.y, target.pose.position.z)
    )
    n.check(
        "interactive-marker plan executes to the target",
        err < 0.005,
        error_m=err,
        moved_m=math.dist(p0, p1),
    )
    n.check(
        "trajectory controller owns all seven positions after marker move",
        n.controllers()[JTAC][1] == [f"{j}/position" for j in ALL],
    )

    # ---- waypoint move back, then cancellation ----------------------------
    n.mark("waypoint_rest")
    n.check("waypoint move reaches rest", goto(n, [0.0] * 6))

    # ---- limit handling ----------------------------------------------------
    n.mark("limit_joint2_negative")
    n.send_mode(1, ["manipulator"], [JVC_ARM])
    n.wait_until(lambda: n.controllers()[JVC_ARM][0] == "active", 5)
    i0 = len(n.samples)
    n.jog_joint("manipulator", "joint2", -0.3, 4.0)
    n.release()
    lo = n.span(i0, "joint2")[0]
    n.check(
        "joint2 jog stops short of the lower limit",
        lo >= J_LIMITS["joint2"][0] - 1e-6,
        min_joint2=lo,
        limit=J_LIMITS["joint2"][0],
    )
    n.mark("limit_latched_jog")
    before = n.snap()
    n.jog_joint("manipulator", "joint2", 0.2, 1.5)
    n.jog_joint("manipulator", "joint1", 0.2, 1.0)
    after, _ = n.release()
    n.observe(
        "jog input after a limit stop (every axis)",
        max_delta_rad=max(abs(after[j] - before[j]) for j in ARM),
        joint2=after["joint2"],
    )
    n.mark("limit_recover_by_trajectory")
    n.check(
        "waypoint move still recovers from the limit stop",
        goto(n, [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]),
    )
    n.mark("limit_jog_after_trajectory")
    n.send_mode(1, ["manipulator"], [JVC_ARM])
    n.wait_until(lambda: n.controllers()[JVC_ARM][0] == "active", 5)
    before = n.snap()
    n.jog_joint("manipulator", "joint1", 0.2, 1.0)
    after, _ = n.release()
    n.observe(
        "jog after trajectory recovery in the same session",
        moved_rad=after["joint1"] - before["joint1"],
    )

    # ---- exit, switch back to trajectory, restart -------------------------
    n.mark("exit_teleop")
    status = n.exit_teleop()
    n.check("Teleoperate exit succeeds", status == 4, goal_status=status)
    time.sleep(1.0)
    c = n.controllers()
    n.observe(
        "controllers after Teleoperate exit",
        active={k: v[1] for k, v in c.items() if v[0] == "active"},
    )
    n.check("exit leaves no two controllers on one interface", disjoint(c))
    n.mark("trajectory_objective_after_exit")
    goal = DoObjectiveSequence.Goal()
    goal.objective_name = "Move to Rest"
    fut = n.objective.send_goal_async(goal)
    n.wait_until(fut.done, 10)
    handle = fut.result()
    result = handle.get_result_async()
    n.wait_until(result.done, 120)
    n.settle(10)
    c = n.controllers()
    n.check(
        "Move to Rest succeeds after Teleoperate",
        result.done() and result.result().result.error_code.val == 1,
    )
    n.check(
        "Move to Rest reaches zero",
        max(abs(n.q[j]) for j in ARM) < 5e-3,
        max_abs=max(abs(n.q[j]) for j in ARM),
    )
    n.check(
        "trajectory controller owns all seven positions again, velocity controllers inactive",
        c[JTAC][1] == [f"{j}/position" for j in ALL]
        and all(c[k][0] == "inactive" for k in (JVC_ARM, JVC_GRIP, VFC)),
        active={k: v[1] for k, v in c.items() if v[0] == "active"},
    )

    n.mark("restart_teleop")
    n.start_teleop()
    n.send_mode(1, ["manipulator"], [JVC_ARM])
    n.wait_until(lambda: n.controllers()[JVC_ARM][0] == "active", 5)
    before = n.snap()
    n.jog_joint("manipulator", "joint1", 0.2, 1.0)
    after, _ = n.release()
    n.check(
        "restarting Teleoperate clears the limit inhibit",
        after["joint1"] - before["joint1"] > 0.02,
        moved_rad=after["joint1"] - before["joint1"],
    )

    n.mark("cancel_during_jog")
    n.jog_joint("manipulator", "joint1", 0.2, 0.6, release="none")
    status = n.cancel_teleop()
    n.check("Teleoperate cancel ends the goal", status == 5, goal_status=status)
    after, drift = n.release(2.0)
    c = n.controllers()
    n.check(
        "Objective cancel while jogging stops motion and holds",
        drift < 1e-3,
        drift=drift,
    )
    n.check(
        "cancel leaves no two controllers on one interface",
        disjoint(c),
        active={k: v[1] for k, v in c.items() if v[0] == "active"},
    )
    n.mark("end_state")
    n.observe(
        "controllers after cancel",
        active={k: v[1] for k, v in n.controllers().items() if v[0] == "active"},
    )


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/mock_teleop_smoke"))
