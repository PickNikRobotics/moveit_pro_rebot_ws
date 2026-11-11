# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Leader engage, deadman and fresh-sample contract, independent of ROS and the SDK."""
import math

from rebot_leader import DIRECTIONS, JOINT_NAMES, LEADER_NAMES, LIMITS_DEG

UNPLUGGED = (
    "The reader does not reopen its port: if the Star Arm USB was unplugged, reconnect it and, "
    "once reBot is at HOME, restart the backend (moveit_pro run). "
)


def joint_label(index):
    return f"{LEADER_NAMES[index].replace('_', ' ')} ({JOINT_NAMES[index]})"


ENGAGE_TOLERANCES = (0.02,) * 6 + (0.05,)
HANDOFF_BLEND_S = 0.5


def alignment_fault(target, measured, tolerance=0.6):
    """Explain gross arm mismatches in the Star Arm's calibrated frame; no jaw gate."""
    offsets = []
    corrections = []
    for index, (leader, follower, direction) in enumerate(
        zip(target[:6], measured[:6], DIRECTIONS[:6])
    ):
        delta = leader - follower
        if abs(delta) <= tolerance:
            continue
        offsets.append(
            f"{joint_label(index)}: {math.degrees(delta):+.1f} deg ({delta:+.3f} rad)"
        )
        correction = -delta / direction
        corrections.append(
            f"{joint_label(index)} toward a {'higher' if correction > 0 else 'lower'} calibrated angle "
            f"by {abs(math.degrees(correction)):.1f} deg ({abs(correction):.3f} rad)"
        )
    if not offsets:
        return ""
    return (
        "Following refused: gross pose mismatch; the Star Arm's pose does not match reBot. "
        f"Leader minus reBot offset: {'; '.join(offsets)}. "
        f"Arm joints joint1-joint6 must be within {math.degrees(tolerance):.1f} deg ({tolerance:g} rad). "
        f"Move only the Star Arm: {'; '.join(corrections)}, toward reBot's matching pose. "
        "Then run Mirror reBot Leader again."
    )


def operator_fault(reason):
    """Add an operator recovery step without hiding the original diagnostic."""
    if reason.startswith("Following refused") or reason.startswith("Following stopped"):
        return reason
    if "thermal" in reason.lower():
        return (
            "Following refused: a motor reached the configured temperature limit and thermal protection is latched. "
            "The backend handles HOME recovery and turns torque off only at HOME. "
            "If HOME recovery fails, torque stays on: stop and get operator help. "
            "After successful HOME recovery, wait at HOME with all seven motors OFF until every motor "
            "is below thermal_clear_c (75 C by default); the driver then clears the latch automatically. "
            "Torque stays OFF. Use the supported HOME enable procedure, match the Star Arm to reBot, "
            "then explicitly run Mirror reBot Leader again at HOME. "
            "If all motors were already OFF when the latch tripped (hot at startup), it does not clear "
            "automatically: let reBot cool at HOME, then restart the backend (moveit_pro run)."
        )
    if "did not start" in reason.lower():
        return (
            "Following refused: it did not start within 3 seconds. reBot is returning HOME; "
            "wait for recovery to finish. Check the Star Arm USB connection, fresh joint feedback and collision scene. "
            + UNPLUGGED
            + "Then match the Star Arm to reBot and run Mirror reBot Leader again."
        )
    if "convergence timed out" in reason.lower():
        return (
            "Following stopped: engage convergence timed out. reBot is returning HOME; "
            "wait for recovery to finish. Keep the Star Arm near reBot's HOME pose and still during ease-in. "
            "Then explicitly run Mirror reBot Leader again; it does not re-engage automatically."
        )
    if any(word in reason.lower() for word in ("sample", "usb", "stream")):
        return (
            "Following stopped: fresh Star Arm or reBot joint data is missing or invalid. "
            f"Detail: {reason}. Check the Star Arm USB connection and backend joint feedback; "
            "wait for HOME recovery to finish. "
            + UNPLUGGED
            + "Then match the Star Arm to reBot and run Mirror reBot Leader again."
        )
    return (
        f"Following refused: {reason}. Wait for any HOME recovery to finish; "
        "check the backend diagnostics and match the Star Arm to reBot before running Mirror reBot Leader again."
    )


class LeaderState:
    """A fault requires explicit reset at home; recovered USB never re-engages."""

    def __init__(
        self,
        sample_timeout=0.1,
        heartbeat_timeout=0.5,
        engage_speed_rad_s=0.05,
        engage_jaw_speed_rad_s=0.10,
        engage_max_offset_rad=0.6,
    ):
        for name, value in (
            ("engage_speed_rad_s", engage_speed_rad_s),
            ("engage_jaw_speed_rad_s", engage_jaw_speed_rad_s),
            ("engage_max_offset_rad", engage_max_offset_rad),
        ):
            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        self.engage_speeds = (engage_speed_rad_s,) * 6 + (engage_jaw_speed_rad_s,)
        self.engage_max_offset_rad = engage_max_offset_rad
        self.engaging = False
        self.engage_deadline = -math.inf
        self.command = None
        self.command_time = -math.inf
        self.seed_pending = False
        self.proposal = None
        self.blend_start = None
        self.blend = 0.0
        self.handoff_offset = None
        self.sample_timeout = sample_timeout
        self.heartbeat_timeout = heartbeat_timeout
        self.target = None
        self.sample_time = -math.inf
        self.measured = None
        self.measured_time = -math.inf
        self.heartbeat = -math.inf
        self.active = False
        self.fault = ""
        self.recover = False
        self.rejection = ""

    def accept(self, sample, now):
        try:
            if "error" in sample:
                raise ValueError(f"Star Arm USB read failed: {sample['error']}")
            values = [float(v) for v in sample["positions"]]
            stamp = float(sample["sample_time"])
            if len(values) != 7 or not all(math.isfinite(v) for v in values):
                raise ValueError("Incomplete/non-finite leader sample")
            if not 0 <= now - stamp < self.sample_timeout or stamp <= self.sample_time:
                raise ValueError("Stale/out-of-order leader sample")
            for value, (lower, upper) in zip(values, LIMITS_DEG):
                if not math.radians(lower) <= value <= math.radians(upper):
                    raise ValueError("Leader target exceeds approved motor limits")
        except (KeyError, TypeError, ValueError) as exc:
            self.target = None
            self.rejection = str(exc)
            self.stop(self.rejection)
            return False
        self.target, self.sample_time = values, stamp
        self.rejection = ""
        return True

    def observe(self, names, positions, now):
        if len(names) != len(positions) or len(set(names)) != len(names):
            return
        lookup = dict(zip(names, positions))
        if not all(j in lookup and math.isfinite(lookup[j]) for j in JOINT_NAMES):
            return
        self.measured = [lookup[j] for j in JOINT_NAMES]
        self.measured_time = now

    def fresh(self, now):
        return (
            self.target is not None
            and self.measured is not None
            and 0 <= now - self.sample_time < self.sample_timeout
            and 0 <= now - self.measured_time < self.sample_timeout
        )

    def engage(self, now):
        if self.fault or self.recover or not self.fresh(now):
            return False
        if not self.active:
            self.fault = alignment_fault(
                self.target, self.measured, self.engage_max_offset_rad
            )
            if self.fault:
                return False
            self.command = [
                max(math.radians(lower), min(math.radians(upper), position))
                for position, (lower, upper) in zip(self.measured, LIMITS_DEG)
            ]
            self.command_time = now
            self.seed_pending = True
            self.proposal = list(self.command)
            self.blend_start = None
            self.blend = 0.0
            self.handoff_offset = None
            self.engaging = True
            self.engage_deadline = (
                now
                + max(
                    abs(target - measured) / speed
                    for target, measured, speed in zip(
                        self.target, self.measured, self.engage_speeds
                    )
                )
                + 5.0
            )
        self.active = True
        self.heartbeat = now
        return True

    def next_command(self, now):
        """Propose a live-target step; only a collision-checked publication advances it."""
        if not self.active:
            return None
        if not self.engaging:
            return list(self.target)
        if self.seed_pending:
            return list(self.command)
        if self.blend_start is not None:
            self.blend = min(1.0, max(0.0, now - self.blend_start) / HANDOFF_BLEND_S)
            return [
                max(
                    math.radians(lower),
                    min(math.radians(upper), target + (1.0 - self.blend) * offset),
                )
                for target, offset, (lower, upper) in zip(
                    self.target, self.handoff_offset, LIMITS_DEG
                )
            ]
        elapsed = max(0.0, now - self.command_time)
        self.proposal = [
            reference + max(-speed * elapsed, min(speed * elapsed, target - reference))
            for reference, target, speed in zip(
                self.command, self.target, self.engage_speeds
            )
        ]
        return list(self.proposal)

    def command_sent(self, now):
        """Commit the published proposal at its actual send time, not collision-check start."""
        if self.blend_start is not None:
            if self.blend >= 1.0:
                self.engaging = False
            return
        self.command = list(self.proposal)
        self.command_time = now
        if (
            self.engaging
            and not self.seed_pending
            and all(
                abs(target - command) <= tolerance
                for target, command, tolerance in zip(
                    self.target, self.command, ENGAGE_TOLERANCES
                )
            )
        ):
            self.blend_start = now
            self.handoff_offset = [
                command - target for command, target in zip(self.command, self.target)
            ]
        self.seed_pending = False

    def stop(self, reason):
        if self.active:
            self.recover = True
            self.fault = reason
        self.active = False
        self.engaging = False
        self.seed_pending = False

    def reset(self):
        if (
            self.recover
            or self.measured is None
            or any(abs(q) > 0.08 for q in self.measured[:6])
        ):
            return False
        self.fault = ""
        return True

    def check_deadman(self, now):
        if self.active and (
            not self.fresh(now) or now - self.heartbeat >= self.heartbeat_timeout
        ):
            self.stop("Leader/follower sample or Objective heartbeat expired")
        elif self.active and self.engaging and now >= self.engage_deadline:
            self.stop("Leader engage convergence timed out; returning HOME")
        return self.active
