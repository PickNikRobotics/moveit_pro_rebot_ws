# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Leader mapping, deadman, position-command, and collision contracts."""
import math
import os
from pathlib import Path
import random
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rebot_leader import (
    JOINT_NAMES,
    LEADER_NAMES,
    LEADER_RANGES,
    LIMITS_DEG,
    map_positions,
)  # noqa: E402
from rebot_leader_bridge import LeaderBridge  # noqa: E402
from rebot_leader_state import LeaderState  # noqa: E402

ONE_COUNT = [math.radians(0.1)] * 6 + [math.radians(0.6)]
BANDS = [0.02] * 6 + [0.05]
BLEND_S = 0.5


def assert_handoff_step(
    before, after, previous_target, target, speeds, interval, offset=None
):
    """A ramp tick moves at most one engage-speed step. A handoff tick moves with
    the leader plus at most |offset0| / 0.5 s, which is within the engage speed."""
    for joint, (old, new, old_goal, goal, speed) in enumerate(
        zip(before, after, previous_target, target, speeds)
    ):
        if offset is None:
            assert abs(new - old) <= speed * interval + 1e-12
        else:
            assert abs(offset[joint]) <= speed * BLEND_S + 1e-12
            fade = abs(offset[joint]) * interval / BLEND_S
            assert abs(new - old) <= abs(goal - old_goal) + fade + 1e-12


def state_at_home(now=10.0, **settings):
    state = LeaderState(**settings)
    state.observe(JOINT_NAMES, [0.0] * 7, now)
    assert state.accept({"positions": [0.0] * 7, "sample_time": now}, now)
    return state


def test_calibrated_mapping_order_sign_scale_and_limits():
    raw = dict(zip(LEADER_NAMES, [10, 20, -30, -40, -50, 60, 70]))
    actual = map_positions(raw, lambda angle, lower, upper: (angle, 0))
    assert actual == pytest.approx(
        [math.radians(q) for q in [10, 20, 30, 40, 50, 60, 310]]
    )


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_leader_angle_rejected(bad):
    raw = dict(zip(LEADER_NAMES, [0.0] * 7))
    raw["elbow_flex"] = bad
    with pytest.raises(ValueError):
        map_positions(raw, lambda angle, lower, upper: (angle, 0))


def test_missing_joint_rejected():
    with pytest.raises(ValueError):
        map_positions({"shoulder_pan": 0}, lambda angle, lower, upper: (angle, 0))


def test_joint_state_order_is_resolved_by_name():
    state = LeaderState()
    positions = [0.1 * i for i in range(7)]
    state.observe(list(reversed(JOINT_NAMES)), list(reversed(positions)), 1.0)
    assert state.measured == positions


@pytest.mark.parametrize(
    "sample",
    [
        {"positions": [0.0] * 6, "sample_time": 10.01},
        {"positions": [math.nan] * 7, "sample_time": 10.01},
        {"positions": [0.0] * 7, "sample_time": 9.0},
        {"positions": [0.0] * 7, "sample_time": 11.0},
        {"positions": [0.0] * 7, "sample_time": 10.0},
        {"positions": [8.0] * 7, "sample_time": 10.01},
        {"error": "USB read failed", "sample_time": 10.01},
    ],
)
def test_invalid_frame_stops_and_latches_recovery(sample):
    state = state_at_home()
    assert state.engage(10.0)
    assert not state.accept(sample, 10.01)
    assert not state.active and state.recover and state.fault
    assert not state.engage(10.01)
    assert not state.reset()


def test_no_usb_auto_reengage_after_recovery():
    state = state_at_home()
    assert state.engage(10.0)
    state.stop("USB disconnected")
    state.recover = False
    assert state.accept({"positions": [0.0] * 7, "sample_time": 10.01}, 10.01)
    assert not state.engage(10.01)
    state.observe(JOINT_NAMES, [0.2] * 7, 10.01)
    assert not state.reset()
    state.observe(JOINT_NAMES, [0.0] * 7, 10.01)
    assert state.reset() and state.engage(10.01)


def test_stale_measured_state_stops_updates():
    state = state_at_home()
    assert state.engage(10.0)
    assert not state.check_deadman(10.101)
    assert state.recover


def test_objective_heartbeat_expiry_stops_even_with_fresh_samples():
    state = state_at_home()
    assert state.engage(10.0)
    state.observe(JOINT_NAMES, [0.0] * 7, 10.49)
    assert state.accept({"positions": [0.0] * 7, "sample_time": 10.49}, 10.49)
    assert not state.check_deadman(10.501)
    assert state.recover


def refresh_session(state, now, target=None, measured=None):
    if target is None:
        target = state.target
    if measured is None:
        measured = state.measured
    state.observe(JOINT_NAMES, measured, now)
    assert state.accept({"positions": target, "sample_time": now}, now)
    assert state.engage(now)  # Objective heartbeat must not restart ease-in.


def test_rest_offset_point_one_one_engages_from_measured_pose():
    state = state_at_home()
    measured = [0.01, 0.02, 0.03, -0.01, 0.02, -0.01, 0.4]
    target = [measured[0] + 0.110] + measured[1:]
    state.observe(JOINT_NAMES, measured, 10.01)
    assert state.accept({"positions": target, "sample_time": 10.01}, 10.01)
    assert state.engage(10.01)
    assert state.engaging and not state.fault
    seed = state.next_command(10.01)
    assert seed == measured
    state.command_sent(10.01)
    command = state.next_command(10.02)
    assert command[0] == pytest.approx(measured[0] + 0.0005)
    assert command[1:] == measured[1:]


def test_full_range_positions_are_forwarded_uncapped_after_convergence():
    state = state_at_home()
    assert state.engage(10.0)
    state.command_sent(10.0)
    refresh_session(state, 10.01)
    command = state.next_command(10.01)
    assert command == [0.0] * 7 and state.engaging
    state.command_sent(10.01)
    assert state.engaging and state.blend_start == 10.01
    refresh_session(state, 10.52)
    assert state.next_command(10.52) == [0.0] * 7
    state.command_sent(10.52)
    assert not state.engaging
    target = [0.6, 1.0, 1.0, 1.0, -1.0, -1.0, 5.0]
    refresh_session(state, 10.53, target=target)
    assert state.check_deadman(10.53)
    assert state.next_command(10.53) == target
    assert not state.engaging


def test_alignment_fault_names_only_arm_joints_beyond_gross_bound():
    from rebot_leader_state import alignment_fault

    follower = [0.0] * 6 + [0.5]
    assert alignment_fault([0.6, -0.6, 0.0, 0.0, 0.0, 0.0, 5.4], follower) == ""
    reason = alignment_fault([0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.15], follower)
    assert "shoulder lift (joint2): +43.0 deg (+0.750 rad)" in reason
    assert "gripper" not in reason and "joint7" not in reason
    assert "joint1-joint6 must be within 34.4 deg (0.6 rad)" in reason
    assert "gross pose mismatch" in reason
    assert (
        "shoulder lift (joint2) toward a lower calibrated angle by 43.0 deg (0.750 rad)"
        in reason
    )
    assert "run Mirror reBot Leader again" in reason


@pytest.mark.parametrize("offset", [0.25, 0.35, 5.4])
def test_jaw_has_no_engage_alignment_gate(offset):
    state = state_at_home()
    assert state.accept(
        {"positions": [0.0] * 6 + [offset], "sample_time": 10.01}, 10.01
    )
    assert state.engage(10.01) and state.engaging
    seed = state.next_command(10.01)
    assert seed == [0.0] * 7
    state.command_sent(10.01)
    assert state.next_command(10.02)[6] == pytest.approx(0.001)
    assert state.engage_deadline == pytest.approx(10.01 + offset / 0.10 + 5.0)


@pytest.mark.parametrize("joint", range(6))
@pytest.mark.parametrize(
    "offset, expected", [(0.110, True), (0.6, True), (0.600001, False)]
)
def test_arm_engage_refuses_only_gross_mismatch(joint, offset, expected):
    state = state_at_home()
    target = [0.0] * 7
    target[joint] = -offset if joint >= 4 else offset
    assert state.accept({"positions": target, "sample_time": 10.01}, 10.01)
    assert state.engage(10.01) is expected


def test_gross_mismatch_refusal_latches_until_explicit_home_reset():
    state = state_at_home()
    assert state.accept({"positions": [0.65] + [0.0] * 6, "sample_time": 10.01}, 10.01)
    assert not state.engage(10.01)
    assert (
        not state.active
        and "shoulder pan (joint1): +37.2 deg (+0.650 rad)" in state.fault
    )
    assert state.accept({"positions": [0.0] * 7, "sample_time": 10.02}, 10.02)
    assert not state.engage(10.02)
    assert state.reset() and state.engage(10.02)


@pytest.mark.parametrize("arm_speed,jaw_speed", [(0.05, 0.10), (0.08, 0.30)])
def test_engage_steps_obey_each_joint_speed_and_never_overshoot(arm_speed, jaw_speed):
    state = state_at_home(
        engage_speed_rad_s=arm_speed, engage_jaw_speed_rad_s=jaw_speed
    )
    measured = [0.1, 0.1, 0.1, 0.1, -0.1, -0.1, 0.5]
    target = [0.3, 0.3, 0.3, -0.1, 0.1, 0.1, 1.5]
    state.observe(JOINT_NAMES, measured, 10.01)
    assert state.accept({"positions": target, "sample_time": 10.01}, 10.01)
    assert state.engage(10.01)
    state.command_sent(10.01)
    previous = measured
    now = 10.01
    for interval in [0.01, 0.02, 0.05, 0.01] * 10:
        now += interval
        refresh_session(state, now, measured=previous)
        assert state.check_deadman(now)
        command = state.next_command(now)
        assert state.engaging
        for before, after, goal, speed in zip(
            previous, command, target, [arm_speed] * 6 + [jaw_speed]
        ):
            assert abs(after - before) <= speed * interval + 1e-12
            assert min(before, goal) <= after <= max(before, goal)
        state.command_sent(now)
        previous = command


@pytest.mark.parametrize(
    "arm_speed,jaw_speed", [(0.05, 0.10), (0.08, 0.30), (0.2, 1.0)]
)
@pytest.mark.parametrize("intervals", [(0.01,), (0.01, 0.02, 0.05)])
@pytest.mark.parametrize("moving_leader", [False, True])
@pytest.mark.parametrize("jaw_offset", [0.2, 1.0])
def test_entire_engage_including_handoff_obeys_each_joint_speed(
    arm_speed, jaw_speed, intervals, moving_leader, jaw_offset
):
    state = state_at_home(
        engage_speed_rad_s=arm_speed, engage_jaw_speed_rad_s=jaw_speed
    )
    measured = [0.1, 0.1, 0.1, 0.1, -0.1, -0.1, 0.5]
    initial_target = [0.3, 0.3, 0.3, -0.1, 0.1, 0.1, measured[6] + jaw_offset]
    directions = [1, 1, 1, -1, 1, 1, 1]
    state.observe(JOINT_NAMES, measured, 10.01)
    assert state.accept({"positions": initial_target, "sample_time": 10.01}, 10.01)
    assert state.engage(10.01)
    deadline = state.engage_deadline
    previous = state.next_command(10.01)
    assert previous == measured
    state.command_sent(10.01)
    now = 10.01
    previous_target = initial_target
    blend_start = None
    for tick in range(2000):
        interval = intervals[tick % len(intervals)]
        now += interval
        target = [
            (
                initial + direction * speed * 0.2 * (now - 10.01)
                if moving_leader
                else initial
            )
            for initial, direction, speed in zip(
                initial_target, directions, state.engage_speeds
            )
        ]
        # A lagging follower must not determine when the command ramp completes.
        refresh_session(state, now, target=target, measured=measured)
        assert state.engage_deadline == deadline
        assert state.check_deadman(now)
        command = state.next_command(now)
        assert_handoff_step(
            previous,
            command,
            previous_target,
            target,
            state.engage_speeds,
            interval,
            None if blend_start is None else state.handoff_offset,
        )
        if blend_start is None:
            assert command == state.proposal  # Pure ramp until the band is reached.
            for before, after, goal in zip(previous, command, target):
                assert min(before, goal) <= after <= max(before, goal)
        else:
            # The handoff offset only fades; it never overshoots the live leader.
            for after, goal, offset in zip(command, target, state.handoff_offset):
                assert 0.0 <= (after - goal) * math.copysign(1.0, offset) <= abs(offset)
        state.command_sent(now)
        if blend_start is None:
            blend_start = state.blend_start
        previous, previous_target = command, target
        if not state.engaging:
            break
    else:
        pytest.fail("Engage never reached the live target")
    assert blend_start is not None and now - blend_start <= BLEND_S + max(intervals)
    assert command == pytest.approx(target)
    # The engage cap must disappear after the bounded handoff.
    uncapped_target = [0.6, 1.0, 1.0, 1.0, -1.0, -1.0, 5.0]
    now += 0.01
    refresh_session(state, now, target=uncapped_target)
    assert state.check_deadman(now)
    assert state.next_command(now) == uncapped_target
    assert not state.engaging


@pytest.mark.parametrize("flicker_ticks", [1, 3, 20])
def test_quantized_flickering_leader_still_converges_with_bounded_handoff(
    flicker_ticks,
):
    rng = random.Random(flicker_ticks)
    state = state_at_home()
    base = [0.05, 0.05, 0.05, 0.0, 0.0, 0.0, 0.2]
    state.target = list(base)
    assert state.engage(10.0)
    state.command_sent(10.0)
    previous = previous_target = state.command
    deadline = state.engage_deadline
    now = 10.0
    for tick in range(3000):
        now += 0.01
        if tick % flicker_ticks == 0:
            # Stationary Star Arm at count boundaries: each joint reads +-1 raw count.
            target = [
                position + count * rng.choice((-1, 0, 1))
                for position, count in zip(base, ONE_COUNT)
            ]
        refresh_session(state, now, target=target)
        assert state.check_deadman(now), state.fault
        assert state.engage_deadline == deadline
        command = state.next_command(now)
        if tick:
            assert_handoff_step(
                previous,
                command,
                previous_target,
                target,
                state.engage_speeds,
                0.01,
                None if state.blend_start is None else state.handoff_offset,
            )
        state.command_sent(now)
        previous, previous_target = command, target
        if not state.engaging:
            break
    else:
        pytest.fail("Engage never handed off to direct following")
    assert now - state.blend_start <= BLEND_S + 0.01 + 1e-9
    assert command == pytest.approx(target)
    refresh_session(state, now + 0.01, target=base)
    assert state.next_command(now + 0.01) == base


@pytest.mark.parametrize("joint,side", [(6, 0), (6, 1), (2, 0), (0, 1)])
def test_handoff_at_a_joint_limit_with_one_count_flicker_stays_in_bounds(joint, side):
    limits = [(math.radians(lower), math.radians(upper)) for lower, upper in LIMITS_DEG]
    limit = limits[joint][side]
    inward = limit + (ONE_COUNT[joint] if side == 0 else -ONE_COUNT[joint])
    measured = [0.0] * 7
    measured[joint] = limit
    state = LeaderState()
    state.observe(JOINT_NAMES, measured, 10.0)
    assert state.accept({"positions": measured, "sample_time": 10.0}, 10.0)
    assert state.engage(10.0)
    state.command_sent(10.0)
    previous = previous_target = list(measured)
    now = 10.0
    for tick in range(200):
        now += 0.01
        # The leader reads at the limit and one raw count inside it, alternately.
        target = list(measured)
        target[joint] = inward if tick % 2 == 0 else limit
        refresh_session(state, now, target=target)
        assert state.check_deadman(now), state.fault
        command = state.next_command(now)
        for position, (lower, upper) in zip(command, limits):
            assert lower <= position <= upper
        assert_handoff_step(
            previous,
            command,
            previous_target,
            target,
            state.engage_speeds,
            0.01,
            None if state.blend_start is None else state.handoff_offset,
        )
        state.command_sent(now)
        previous, previous_target = command, target
        if not state.engaging:
            break
    else:
        pytest.fail("Engage never handed off to direct following")
    assert now - state.blend_start <= BLEND_S + 0.01 + 1e-9


def test_engage_chases_live_target_in_both_directions_without_reseeding():
    state = state_at_home()
    state.target[0] = 0.3
    assert state.engage(10.0)
    state.command_sent(10.0)
    deadline = state.engage_deadline
    for now, target, expected in [
        (10.02, 0.4, 0.001),
        (10.04, -0.2, 0.0),
        (10.06, -0.3, -0.001),
    ]:
        refresh_session(state, now, target=[target] + [0.0] * 6)
        command = state.next_command(now)
        assert command[0] == pytest.approx(expected)
        assert state.engaging and state.engage_deadline == deadline
        state.command_sent(now)


@pytest.mark.parametrize("joint", range(7))
@pytest.mark.parametrize("outside_band", [False, True])
def test_band_entry_starts_a_fixed_blend_instead_of_a_jump(joint, outside_band):
    state = state_at_home()
    speed = state.engage_speeds[joint]
    step = speed * 0.01
    state.target[joint] = BANDS[joint] + (2.0 if outside_band else 0.5) * step
    assert state.engage(10.0)
    assert state.next_command(10.0) == [0.0] * 7  # Always send the measured seed first.
    state.command_sent(10.0)
    command = state.next_command(10.01)
    assert command[joint] == pytest.approx(
        step
    )  # The band is never skipped in one tick.
    assert state.engaging  # A proposal is not a collision-checked publication.
    state.command_sent(10.01)
    assert state.engaging
    assert (state.blend_start is None) is outside_band
    if outside_band:
        return
    # The ramp freezes at band entry; the blend moves from it to the live leader.
    refresh_session(state, 10.26)
    command = state.next_command(10.26)
    assert command[joint] == pytest.approx(step + 0.5 * (state.target[joint] - step))
    state.command_sent(10.26)
    assert state.engaging
    refresh_session(state, 10.52)
    assert state.next_command(10.52) == pytest.approx(state.target)
    state.command_sent(10.52)
    assert not state.engaging


def test_convergence_uses_the_command_reference_not_the_measured_follower():
    state = state_at_home()
    state.target[0] = 0.110
    assert state.engage(10.0)
    state.command_sent(10.0)
    # Measured state at target must not bypass a command reference still far behind.
    refresh_session(state, 10.01, measured=state.target)
    assert state.next_command(10.01)[0] == pytest.approx(0.0005)
    assert state.engaging and state.blend_start is None
    # Spring-mode follower lag must not hold engage open once the reference converges.
    now = 10.01
    while state.engaging:
        state.command_sent(now)
        now += 0.01
        refresh_session(state, now, measured=[0.0] * 7)
        state.next_command(now)
        assert now < 13.0
    assert state.blend_start == pytest.approx(10.01 + (0.110 - 0.02) / 0.05, abs=0.011)


def test_only_published_commands_advance_the_ramp_reference_and_clock():
    state = state_at_home()
    state.target[0] = 0.110
    assert state.engage(10.0)
    seed = state.next_command(10.01)
    assert state.next_command(10.02) == seed == [0.0] * 7
    state.command_sent(10.02)
    first = state.next_command(10.03)
    assert state.next_command(10.03) == first
    # Collision-check time is not credited to the next command step.
    state.command_sent(10.04)
    assert state.next_command(10.05)[0] - first[0] == pytest.approx(0.0005)


def test_unpublished_final_step_cannot_complete_engage_or_bypass_timeout():
    state = state_at_home()
    state.target[0] = 0.019
    assert state.engage(10.0)
    state.command_sent(10.0)
    deadline = state.engage_deadline
    refresh_session(state, deadline - 0.01)
    assert state.next_command(deadline - 0.01) == state.target
    assert state.engaging
    # No command_sent: collision checking has not authorized the final step.
    refresh_session(state, deadline)
    assert not state.check_deadman(deadline)
    assert not state.active and state.recover
    assert "convergence timed out" in state.fault


def test_convergence_timeout_uses_largest_offset_over_its_speed_and_stays_fixed():
    state = state_at_home()
    state.target = [0.110] + [0.0] * 5 + [0.8]
    assert state.engage(10.0)
    assert state.engage_deadline == pytest.approx(23.0)  # Jaw: 0.8/0.10 + 5 s.
    state.command_sent(10.0)
    refresh_session(state, 22.99)
    assert state.engage_deadline == pytest.approx(23.0)
    assert state.check_deadman(22.99)
    refresh_session(state, 23.0)
    assert not state.check_deadman(23.0)
    assert not state.active and state.recover and not state.engaging
    assert "convergence timed out" in state.fault
    assert not state.engage(23.0) and state.next_command(23.0) is None


def test_gross_offset_bound_is_configurable_without_gating_the_jaw():
    state = state_at_home(engage_max_offset_rad=0.2)
    state.target = [0.2] + [0.0] * 5 + [5.4]
    assert state.engage(10.0)
    state = state_at_home(engage_max_offset_rad=0.2)
    state.target[0] = 0.200001
    assert not state.engage(10.0)
    assert "(0.2 rad)" in state.fault


@pytest.mark.parametrize(
    "setting", ["engage_speed_rad_s", "engage_jaw_speed_rad_s", "engage_max_offset_rad"]
)
@pytest.mark.parametrize("bad", [0.0, -0.1, math.nan, math.inf, True, "0.05"])
def test_engage_settings_must_be_finite_positive_numbers(setting, bad):
    with pytest.raises(ValueError, match=f"{setting} must be finite and positive"):
        LeaderState(**{setting: bad})


def test_hold_cannot_command_negative_home_side_position(monkeypatch):
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.state.measured = [0.0, -0.01, -0.008, 0.0, 0.0, 0.0, -0.1]
    commands = []
    bridge.publish = commands.append
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.01)
    bridge.hold()
    assert commands == [[0.0] * 7]
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.2)
    bridge.hold()
    assert len(commands) == 1  # No position hold from stale feedback.


def test_engage_requires_collision_scene():
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.start_checked = True
    bridge.verified = True
    bridge.scene_ready = False
    bridge.home_failed = False
    bridge.reader_exited = False
    response = bridge.mirror(None, SimpleNamespace())
    assert not response.success and not bridge.state.active


def test_incomplete_follower_state_does_not_refresh_timestamp():
    state = state_at_home()
    state.observe(JOINT_NAMES[:-1], [0.0] * 6, 10.09)
    assert not state.fresh(10.101)


def test_collision_path_checks_intermediate_poses():
    from rebot_leader_collision import LeaderCollision

    checker = LeaderCollision.__new__(LeaderCollision)
    checker.bounds = [(-10.0, 10.0)] * 7
    poses = []

    def check(positions):
        poses.append(positions)
        return "blocked" if 0.009 < positions[0] < 0.011 else ""

    checker.check_pose = check
    assert checker.check_path([0.0] * 7, [0.02] * 7) == "blocked"
    assert poses[0] == [0.0] * 7
    assert poses[-1] == pytest.approx([0.01] * 7)


def test_collision_path_samples_the_jaw_at_its_own_spacing():
    from rebot_leader_collision import LeaderCollision

    checker = LeaderCollision.__new__(LeaderCollision)
    checker.bounds = [(-10.0, 10.0)] * 7
    closed, opened = [0.0] * 7, [0.0] * 6 + [5.41]
    poses = []
    checker.check_pose = lambda positions: poses.append(positions) or ""
    assert checker.check_path(closed, opened) == ""
    # 0.005 rad on the jaw too would take 1083 checks for this sweep.
    assert len(poses) < 1000
    assert poses[0] == closed and poses[-1] == pytest.approx(opened)
    assert max(b[6] - a[6] for a, b in zip(poses, poses[1:])) <= 0.05
    checker.check_pose = lambda p: "jaw blocked" if 2.0 < p[6] < 2.06 else ""
    assert checker.check_path(closed, opened) == "jaw blocked"
    checker.check_pose = lambda p: "arm blocked" if 0.009 < p[0] < 0.011 else ""
    assert checker.check_path(closed, [0.02] * 6 + [5.41]) == "arm blocked"


def test_collision_target_bounds_do_not_reject_measured_overshoot():
    from rebot_leader_collision import LeaderCollision

    checker = LeaderCollision.__new__(LeaderCollision)
    checker.bounds = [(0.0, 1.0)] * 7
    poses = []
    checker.check_pose = lambda positions: poses.append(positions) or ""
    assert checker.check_path([-0.01] * 7, [0.0] * 7) == ""
    assert poses[0] == [-0.01] * 7
    assert (
        checker.check_path([0.0] * 7, [-0.001] * 7)
        == "Target exceeds robot joint bounds"
    )


@pytest.fixture(scope="module")
def collision_checker():
    from ament_index_python.packages import get_package_share_directory
    import xacro
    from rebot_leader_collision import LeaderCollision

    # The stock model: the stock URDF and the SRDF the stock configs inherit.
    urdf = xacro.process_file(
        str(
            Path(get_package_share_directory("rebot_b601_rs_hw"))
            / "description/rebot.urdf.xacro"
        ),
        mappings={"hardware_interface": "mock"},
    ).toxml()
    return LeaderCollision(
        urdf,
        Path(get_package_share_directory("rebot_b601_rs_mock"))
        / "config/moveit/rebot.srdf",
    )


def test_native_fcl_checks_monitored_world_objects(collision_checker):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive

    checker = collision_checker
    assert checker.check_path([0.0] * 7, [0.0] * 7) == ""
    scene = PlanningScene()
    scene.is_diff = True
    obj = CollisionObject()
    obj.id = "obstructed_bench"
    obj.header.frame_id = "base_link"
    obj.operation = CollisionObject.ADD
    obj.primitives = [
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 2.0])
    ]
    pose = Pose()
    pose.orientation.w = 1.0
    obj.primitive_poses = [pose]
    scene.world.collision_objects = [obj]
    checker.update_world(scene)
    assert "obstructed_bench" in checker.check_path([0.0] * 7, [0.0] * 7)
    scene.is_diff = False
    scene.world.collision_objects = []
    checker.update_world(scene)
    assert checker.check_path([0.0] * 7, [0.0] * 7) == ""


@pytest.mark.parametrize("end", [0, 1])
def test_leader_range_endpoints_pass_robot_joint_bounds(collision_checker, end):
    raw = {
        name: leader_range[end]
        for name, leader_range in zip(LEADER_NAMES, LEADER_RANGES)
    }
    target = map_positions(raw, lambda angle, lower, upper: (angle, 0))
    assert LeaderState().accept({"positions": target, "sample_time": 10.0}, 10.0)
    assert (
        collision_checker.check_path(target, target)
        != "Target exceeds robot joint bounds"
    )


def test_engage_seed_below_a_lower_bound_passes_robot_joint_bounds(collision_checker):
    state = state_at_home()
    measured = [0.0, -0.03, -0.02, 0.0, 0.0, 0.0, -0.0001]
    state.observe(JOINT_NAMES, measured, 10.0)
    assert state.engage(10.0)
    seed = state.next_command(10.0)
    assert seed == [0.0] * 7
    # The stock SRDF reports folded link2/link5 contact here (see the next test);
    # a config whose SRDF excludes that pair sees a clear path. The bounds hold either way.
    acm = collision_checker.scene.allowed_collision_matrix
    assert collision_checker.check_path(measured, seed) == "Collision: link2 / link5"
    acm.set_entry("link2", "link5", True)
    try:
        assert collision_checker.check_path(measured, seed) == ""
    finally:
        acm.set_entry("link2", "link5", False)


def test_native_full_jaw_sweep_is_checked_in_fewer_than_a_thousand_poses(
    collision_checker, monkeypatch
):
    checks = []
    check_pose = collision_checker.check_pose
    monkeypatch.setattr(
        collision_checker, "check_pose", lambda p: checks.append(p) or check_pose(p)
    )
    assert collision_checker.check_path([0.0] * 7, [0.0] * 6 + [5.41]) == ""
    assert 100 < len(checks) < 1000


def test_octomap_refuses_following(collision_checker):
    from moveit_msgs.msg import PlanningScene

    scene = PlanningScene()
    scene.world.octomap.octomap.data = [1]
    with pytest.raises(ValueError, match="Octomap"):
        collision_checker.update_world(scene)


def test_stock_srdf_reports_folded_link_contact_as_its_only_collision(
    collision_checker,
):
    # The stock SRDF keeps link2/link5 checked; the office SRDF excludes the pair
    # (picknik_boston_rebot_b601_rs_hw test_leader_office_model). The bridge uses
    # whichever SRDF the selected config names.
    checker = collision_checker
    folded = [
        0.080285143927,
        0.0,
        0.017453292520,
        -0.043633231300,
        0.057595864484,
        -0.062831851407,
        0.0,
    ]
    assert checker.check_pose([0.0] * 7) == ""
    assert checker.check_pose(folded) == "Collision: link2 / link5"
    checker.scene.allowed_collision_matrix.set_entry("link2", "link5", True)
    try:
        assert checker.check_pose(folded) == ""
    finally:
        checker.scene.allowed_collision_matrix.set_entry("link2", "link5", False)


@pytest.fixture
def home_bridge(monkeypatch):
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.home_failed = False
    bridge.reader_exited = False
    bridge.real_hardware = True
    bridge.verified = bridge.scene_ready = True
    bridge.start_checked = True
    bridge.hardware_state = SimpleNamespace(
        id=list(range(1, 8)), comm_state=0, fault_bits=[0] * 7, enabled=[True] * 7
    )
    bridge.hardware_time = 10.01
    bridge.publish = lambda positions: None
    bridge.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=lambda text: None, warn=lambda text: None, error=lambda text: None
        )
    )
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.01)
    return bridge


def test_objective_preparation_is_explicit_home_only(home_bridge):
    bridge = home_bridge
    bridge.state.measured[2] = 0.2
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.state.measured[2] = 0.0
    assert bridge.prepare(None, SimpleNamespace()).success
    assert not bridge.start_checked and not bridge.state.active
    assert bridge.warmup_heartbeat == -math.inf


def test_sagged_home_is_home_for_preparation(home_bridge):
    home_bridge.state.measured = [0.0, -0.01, -0.008, 0.0, 0.0, 0.0, 0.0]
    assert home_bridge.prepare(None, SimpleNamespace()).success


@pytest.fixture
def unprepared_ticking_bridge(home_bridge):
    read, write = os.pipe()
    os.set_blocking(read, False)
    home_bridge.stream = SimpleNamespace(fileno=lambda: read)
    home_bridge.buffer = b""
    home_bridge.collider = SimpleNamespace(check_path=lambda measured, target: "")
    home_bridge.recovery_started = None
    home_bridge.recovery_future = None
    home_bridge.recovery_result = None
    home_bridge.start_checked = False
    home_bridge.prepared_at = None
    home_bridge.warmup_heartbeat = -math.inf
    home_bridge.warmup_reference = None
    yield home_bridge
    os.close(read)
    os.close(write)


@pytest.mark.parametrize("position", [0.0, 0.2])
def test_mirror_without_preparation_cannot_start(unprepared_ticking_bridge, position):
    bridge = unprepared_ticking_bridge
    bridge.state.measured[2] = position
    assert not bridge.mirror(None, SimpleNamespace()).success
    assert bridge.warmup_heartbeat == -math.inf
    bridge.tick()
    assert not bridge.state.active and not bridge.start_checked
    # The timer must enforce preparation independently of the service gate.
    bridge.warmup_heartbeat = 10.01
    bridge.tick()
    assert not bridge.state.active and not bridge.start_checked


@pytest.fixture
def ticking_bridge(unprepared_ticking_bridge):
    assert unprepared_ticking_bridge.prepare(None, SimpleNamespace()).success
    return unprepared_ticking_bridge


def test_prepared_session_follows_once_mirror_starts(ticking_bridge):
    assert ticking_bridge.mirror(None, SimpleNamespace()).success
    ticking_bridge.tick()
    assert ticking_bridge.state.active and not ticking_bridge.state.recover
    assert ticking_bridge.prepared_at is None


def test_bridge_rest_offset_starts_collision_checked_bounded_ramp(
    ticking_bridge, monkeypatch
):
    bridge = ticking_bridge
    bridge.state.target[0] = 0.110
    published, checks = [], []
    bridge.publish = lambda positions: published.append(list(positions))
    bridge.collider.check_path = (
        lambda measured, command: checks.append((list(measured), list(command))) or ""
    )
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert bridge.state.active and bridge.state.engaging and bridge.start_checked
    assert published[-1] == [0.0] * 7
    assert checks == [([0.0] * 7, [0.0] * 7)]
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.03)
    refresh_session(bridge.state, 10.03)
    bridge.tick()
    assert published[-1][0] == pytest.approx(0.001)
    assert checks[-1][1] == published[-1]
    assert bridge.state.target[0] == 0.110


@pytest.mark.parametrize("leader_speed", [0.0, 0.01, 0.5])
def test_bridge_blends_from_the_band_into_direct_following_within_fixed_time(
    ticking_bridge, monkeypatch, leader_speed
):
    bridge = ticking_bridge
    bridge.real_hardware = False
    initial_target = [0.019] + [0.0] * 5 + [0.049]
    bridge.state.target = initial_target
    published, checks, logs = [], [], []
    bridge.publish = lambda command: published.append(list(command))
    bridge.collider.check_path = (
        lambda measured, command: checks.append(list(command)) or ""
    )
    logger = bridge.node.get_logger()
    logger.info = logs.append
    monkeypatch.setattr(bridge.node, "get_logger", lambda: logger)
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert bridge.state.engaging and published[-1] == [0.0] * 7
    now = 10.01
    previous, previous_target = published[-1], initial_target
    for _ in range(100):
        now += 0.01
        monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: now)
        target = list(initial_target)
        # A leader faster than the engage speed must not prolong the handoff blend.
        moving = max(0.0, now - (bridge.state.blend_start or now))
        target[0] += leader_speed * moving
        target[6] += 2 * leader_speed * moving
        refresh_session(bridge.state, now, target=target)
        handoff = bridge.state.blend_start is not None
        bridge.tick()
        assert bridge.state.active and not bridge.state.recover
        command = published[-1]
        assert command == checks[-1]
        assert_handoff_step(
            previous,
            command,
            previous_target,
            target,
            bridge.state.engage_speeds,
            0.01,
            bridge.state.handoff_offset if handoff else None,
        )
        previous, previous_target = command, target
        if not bridge.state.engaging:
            break
    else:
        pytest.fail("Bridge never published the final bounded step")
    assert now - bridge.state.blend_start <= BLEND_S + 0.01 + 1e-9
    assert command == pytest.approx(target)
    assert logs.count("POSITION FOLLOWING CONVERGED: direct uncapped following") == 1


def test_ramp_starts_with_latest_measured_pose_not_the_warmup_reference(ticking_bridge):
    bridge = ticking_bridge
    bridge.state.measured[0] = 0.01
    bridge.state.target[0] = 0.12
    published = []
    bridge.publish = lambda command: published.append(list(command))
    assert bridge.warmup_reference == [0.0] * 7
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert published[-1] == [0.01] + [0.0] * 6
    assert bridge.state.engaging


@pytest.mark.parametrize("failure", ["collision", "expired", "heartbeat"])
@pytest.mark.parametrize("offset", [0.110, 0.0004])
def test_ramp_never_publishes_an_unchecked_or_expired_step(
    ticking_bridge, monkeypatch, failure, offset
):
    bridge = ticking_bridge
    bridge.state.target[0] = offset
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert bridge.state.engaging
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.03)
    refresh_session(bridge.state, 10.03)
    published, checks = [], []
    bridge.publish = lambda command: published.append(list(command))

    def check(measured, command):
        checks.append(list(command))
        if failure == "expired":
            bridge.state.sample_time = 9.0
        elif failure == "heartbeat":
            bridge.state.heartbeat = 9.0
        return "Collision: link1 / wall" if failure == "collision" else ""

    bridge.collider.check_path = check
    bridge.tick()
    assert checks[0][0] == pytest.approx(min(offset, 0.001))
    assert not bridge.state.active and bridge.state.recover
    assert bridge.state.command == [0.0] * 7
    assert published == [[0.0] * 7]  # Existing measured hold, never the ramp target.
    assert bridge.recovery_started is not None


def test_ramp_convergence_timeout_requests_the_existing_native_home(
    ticking_bridge, monkeypatch
):
    bridge = ticking_bridge
    bridge.real_hardware = False
    bridge.state.target[0] = 0.110
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    deadline = bridge.state.engage_deadline
    assert deadline == pytest.approx(17.21)
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: deadline)
    refresh_session(bridge.state, deadline, target=[0.4] + [0.0] * 6)
    published, goals = [], []
    bridge.publish = lambda command: published.append(list(command))
    bridge.tick()
    assert not bridge.state.active and bridge.state.recover
    assert bridge.recovery_started == deadline
    assert "convergence timed out" in bridge.state.fault
    response = bridge.mirror(None, SimpleNamespace())
    assert not response.success and "reBot is returning HOME" in response.message
    assert "explicitly run Mirror reBot Leader again" in response.message
    bridge.home_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal: goals.append(goal)
        or SimpleNamespace(done=lambda: False),
    )
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: deadline + 0.51)
    bridge.tick()
    assert [goal.objective_name for goal in goals] == ["Return reBot Leader Home"]
    assert all(command == [0.0] * 7 for command in published)
    assert not bridge.state.engaging


def test_ramp_may_continue_beyond_three_seconds_once_started(
    ticking_bridge, monkeypatch
):
    bridge = ticking_bridge
    bridge.state.target[0] = 0.3
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    bridge.real_hardware = False
    now = 10.01
    command = bridge.state.command
    for _ in range(160):
        now += 0.02
        monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: now)
        refresh_session(bridge.state, now, measured=command)
        bridge.tick()
        command = bridge.state.command
    assert bridge.state.active and bridge.state.engaging and not bridge.state.recover
    assert command[0] == pytest.approx(0.16)
    assert bridge.prepared_at is None


def test_fresh_readiness_after_start_deadline_still_returns_home(
    ticking_bridge, monkeypatch
):
    bridge = ticking_bridge
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 13.02)
    bridge.state.observe(JOINT_NAMES, [0.0] * 7, 13.02)
    assert bridge.state.accept(
        {"positions": [0.110] + [0.0] * 6, "sample_time": 13.02}, 13.02
    )
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert not bridge.state.active and bridge.state.recover
    assert "did not start" in bridge.state.fault


def test_grossly_misaligned_leader_never_follows_and_reports_the_joint(ticking_bridge):
    bridge = ticking_bridge
    bridge.state.target[1] = 0.65
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert not bridge.state.active
    response = bridge.mirror(None, SimpleNamespace())
    assert (
        not response.success
        and "shoulder lift (joint2): +37.2 deg (+0.650 rad)" in response.message
    )
    bridge.tick()
    assert bridge.state.recover and not bridge.state.active


@pytest.mark.parametrize("rate", [0, 19, 101, math.nan, math.inf])
def test_bridge_refuses_publish_rate_outside_20_to_100_hz(monkeypatch, rate):
    from unittest.mock import Mock
    import rebot_leader_bridge

    monkeypatch.setattr(rebot_leader_bridge, "ActionClient", Mock())
    node = Mock()
    node.declare_parameter.side_effect = lambda name, default: SimpleNamespace(
        value=rate if name == "publish_rate" else default
    )
    read, write = os.pipe()
    try:
        with pytest.raises(ValueError, match="between 20 and 100"):
            LeaderBridge(node, SimpleNamespace(fileno=lambda: read), "/no.srdf")
    finally:
        os.close(read)
        os.close(write)
    node.create_timer.assert_not_called()


@pytest.mark.parametrize(
    "setting", ["engage_speed_rad_s", "engage_jaw_speed_rad_s", "engage_max_offset_rad"]
)
def test_bridge_rejects_invalid_engage_setting_before_starting_timer(
    monkeypatch, setting
):
    from unittest.mock import Mock

    node = Mock()
    node.declare_parameter.side_effect = lambda name, default: SimpleNamespace(
        value=math.nan if name == setting else default
    )
    with pytest.raises(ValueError, match=f"{setting} must be finite and positive"):
        LeaderBridge(node, None, "/no.srdf")
    node.create_timer.assert_not_called()


def test_prepared_session_that_never_follows_returns_home(ticking_bridge, monkeypatch):
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 12.9)
    ticking_bridge.tick()
    assert not ticking_bridge.state.recover
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 13.02)
    ticking_bridge.tick()
    assert ticking_bridge.state.recover and ticking_bridge.state.fault
    assert ticking_bridge.prepared_at is None
    assert not ticking_bridge.mirror(None, SimpleNamespace()).success


def test_clutch_before_following_returns_home(ticking_bridge):
    ticking_bridge.clutch(None, SimpleNamespace())
    ticking_bridge.tick()
    assert ticking_bridge.state.recover and not ticking_bridge.state.active


def test_objective_preparation_refuses_disabled_or_faulted_hardware(home_bridge):
    bridge = home_bridge
    bridge.hardware_state.enabled[2] = False
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.hardware_state.enabled[2] = True
    bridge.hardware_state.fault_bits[2] = 1
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.hardware_state.fault_bits[2] = 0
    for enabled in ([], [True] * 6, [True] * 8):
        bridge.hardware_state.enabled = enabled
        assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.hardware_state.enabled = [True] * 7
    assert bridge.prepare(None, SimpleNamespace()).success


def _stale(bridge):
    bridge.hardware_time = 9.5


def _comm_lost(bridge):
    bridge.hardware_state.comm_state = 1


def _faulted(bridge):
    bridge.hardware_state.fault_bits[3] = 1


def _disabled(bridge):
    bridge.hardware_state.enabled[4] = False


def _malformed_enabled(bridge):
    bridge.hardware_state.enabled = [True] * 6


@pytest.mark.parametrize(
    "fail", [_stale, _comm_lost, _faulted, _disabled, _malformed_enabled]
)
def test_rejected_preparation_preserves_fault_latch(home_bridge, fail):
    bridge = home_bridge
    # HOME completed after a prior session, but the stop remains latched.
    bridge.state.fault = "Clutched"
    fail(bridge)
    assert not bridge.prepare(None, SimpleNamespace()).success
    assert bridge.state.fault == "Clutched"

    bridge.hardware_state.comm_state = 0
    bridge.hardware_state.fault_bits = [0] * 7
    bridge.hardware_state.enabled = [True] * 7
    bridge.hardware_time = 10.01
    assert not bridge.mirror(None, SimpleNamespace()).success
    assert not bridge.state.active

    # Only a successful preparation may clear the latch and start warm-up.
    assert bridge.prepare(None, SimpleNamespace()).success
    assert not bridge.state.fault and not bridge.start_checked
    assert bridge.mirror(None, SimpleNamespace()).success
    assert not bridge.state.active


@pytest.fixture
def following_bridge(ticking_bridge):
    bridge = ticking_bridge
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert bridge.state.active
    bridge.tick()  # Converge at HOME before checking today's uncapped follow path.
    bridge.state.blend_start = -math.inf  # The fixed handoff blend has elapsed.
    bridge.tick()
    assert not bridge.state.engaging
    published = []
    bridge.publish = published.append
    target = [0.1] * 6 + [1.0]
    assert bridge.state.accept({"positions": target, "sample_time": 10.01}, 10.01)
    return bridge, published, target


def test_healthy_hardware_keeps_following(following_bridge):
    bridge, published, target = following_bridge
    bridge.tick()
    assert bridge.state.active and published == [target]


@pytest.mark.parametrize(
    "fail", [_stale, _comm_lost, _faulted, _disabled, _malformed_enabled]
)
def test_unhealthy_hardware_stops_following_and_returns_home(following_bridge, fail):
    bridge, published, target = following_bridge
    fail(bridge)
    bridge.tick()
    assert not bridge.state.active and bridge.state.recover and bridge.state.fault
    assert bridge.recovery_started is not None
    # Only the measured-position hold goes out, never the leader target.
    assert published == [[0.0] * 7]
    bridge.hardware_state.comm_state = 0
    bridge.hardware_state.fault_bits = [0] * 7
    bridge.hardware_state.enabled = [True] * 7
    bridge.hardware_time = 10.01
    assert not bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert not bridge.state.active and target not in published


def test_simulated_sessions_do_not_require_hardware_status(following_bridge):
    bridge, published, target = following_bridge
    bridge.real_hardware = False
    bridge.hardware_state = None
    bridge.tick()
    assert bridge.state.active and published == [target]


def test_repeated_stop_signals_do_not_abort_the_home_return(monkeypatch):
    import signal
    import rebot_leader_bridge

    bridge = SimpleNamespace(
        state=SimpleNamespace(recover=True, stop=lambda reason: None),
        home_failed=False,
        hold=lambda: None,
    )
    spins = []

    def spin_once(node, timeout_sec):
        spins.append(timeout_sec)
        if len(spins) == 1:
            # A non-interactive launch forwards SIGINT, then escalates to SIGTERM.
            signal.raise_signal(signal.SIGINT)
            signal.raise_signal(signal.SIGTERM)
        else:
            bridge.state.recover = False

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rebot_leader_bridge.py",
            "--port",
            "/no-usb",
            "--calibration-file",
            "/existing.json",
            "--srdf-file",
            "/existing.srdf",
        ],
    )
    child = SimpleNamespace(
        stdout=SimpleNamespace(close=lambda: None),
        terminate=lambda: None,
        wait=lambda **kwargs: None,
    )
    monkeypatch.setattr(
        rebot_leader_bridge.subprocess, "Popen", lambda *args, **kwargs: child
    )
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(buffer=None))
    monkeypatch.setattr(
        rebot_leader_bridge, "LeaderBridge", lambda node, stream, srdf: bridge
    )
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(
        rebot_leader_bridge.rclpy,
        "create_node",
        lambda name: SimpleNamespace(destroy_node=lambda: None),
    )
    monkeypatch.setattr(
        rebot_leader_bridge.rclpy,
        "spin",
        lambda node: signal.raise_signal(signal.SIGINT),
    )
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "spin_once", spin_once)
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "shutdown", lambda: None)
    handlers = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        rebot_leader_bridge.main()
    except KeyboardInterrupt:
        pytest.fail("A repeated stop signal aborted the HOME return")
    finally:
        for number, handler in handlers.items():
            signal.signal(number, handler)
    assert len(spins) == 2 and not bridge.state.recover


def test_launch_does_not_kill_the_bridge_during_its_home_return():
    import importlib.util
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    path = Path(__file__).resolve().parents[1] / "launch" / "rebot_leader.launch.py"
    spec = importlib.util.spec_from_file_location("rebot_leader_launch", path)
    launch_file = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_file)
    context = LaunchContext()
    config = SimpleNamespace(
        ros_global_params={
            "rebot_leader_enabled": True,
            "rebot_leader_port": "/no-usb",
            "rebot_leader_calibration_file": "/existing.json",
        },
        hardware=SimpleNamespace(
            robot_description=SimpleNamespace(
                urdf_params=[{"hardware_interface": "mock"}],
                srdf=SimpleNamespace(
                    package="rebot_b601_rs_mock", path="config/moveit/rebot.srdf"
                ),
            )
        ),
    )
    (node,) = launch_file.configured_nodes(config)
    sigkill_after = sum(
        float(perform_substitutions(context, timeout))
        for timeout in (node.sigterm_timeout, node.sigkill_timeout)
    )
    # The bridge waits up to 120 s for HOME, then 5 s for its reader.
    assert sigkill_after > 125


def finished(status=4, code=1):
    return SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(
            status=status, result=SimpleNamespace(error_code=SimpleNamespace(val=code))
        ),
    )


class ObjectiveClient:
    """Records goals; each future completes immediately with the given outcome."""

    def __init__(self, ready=True, accepted=True, status=4, code=1):
        self.ready, self.accepted = ready, accepted
        self.status, self.code = status, code
        self.goals, self.idle_at_send = [], []
        self.bridge = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal.objective_name)
        self.idle_at_send.append(
            self.bridge.idle(None, SimpleNamespace()).success if self.bridge else None
        )
        result = finished(self.status, self.code)
        handle = SimpleNamespace(
            accepted=self.accepted,
            get_result_async=lambda: SimpleNamespace(
                add_done_callback=lambda callback: callback(result)
            ),
        )
        return SimpleNamespace(
            done=lambda: True,
            result=lambda: handle,
            add_done_callback=lambda callback: callback(
                SimpleNamespace(result=lambda: handle)
            ),
        )


def recovered_bridge(bridge, client):
    client.bridge = bridge
    bridge.home_client = client
    bridge.prepared_at = None
    bridge.state.recover = True
    bridge.state.fault = "Clutched"
    bridge.recovery_started = 9.0
    bridge.recovery_future = SimpleNamespace(done=lambda: True)
    bridge.recovery_result = finished()
    return bridge


def test_successful_home_requires_explicit_reengagement(home_bridge):
    client = ObjectiveClient()
    bridge = recovered_bridge(home_bridge, client)
    bridge.recover_home(10.01)
    assert not bridge.state.recover
    assert not bridge.state.active
    assert bridge.state.fault == "Clutched"
    # Ending Mirror finishes with the stock guarded relax, never a re-engage.
    assert client.goals == ["Relax at HOME"]


def test_relax_after_mirror_is_requested_only_once_the_bridge_is_idle(home_bridge):
    client = ObjectiveClient()
    bridge = recovered_bridge(home_bridge, client)
    bridge.recover_home(10.01)
    # Relax at HOME checks /rebot_leader_bridge/idle before HOME and relax.
    assert client.idle_at_send == [True]
    bridge.recover_home(10.02)
    assert client.goals == ["Relax at HOME"]  # One request per completed recovery.


@pytest.mark.parametrize(
    "client, log",
    [
        (ObjectiveClient(ready=False), "Objective server unavailable"),
        (ObjectiveClient(accepted=False), "goal rejected"),
        (ObjectiveClient(code=99999), "did not relax"),
    ],
)
def test_unavailable_or_refused_relax_keeps_torque_on_at_home(home_bridge, client, log):
    logs = []
    home_bridge.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=logs.append, warn=logs.append, error=logs.append
        )
    )
    bridge = recovered_bridge(home_bridge, client)
    bridge.recover_home(10.01)
    assert any(log in line and "torque stays on at HOME" in line for line in logs)
    assert not bridge.home_failed and not bridge.state.recover


def test_failed_home_never_requests_relax(home_bridge):
    client = ObjectiveClient()
    bridge = recovered_bridge(home_bridge, client)
    bridge.recovery_result = finished(status=6, code=99999)
    bridge.recover_home(10.01)
    assert bridge.home_failed and client.goals == []


def test_warmup_reference_does_not_follow_feedback_drift(monkeypatch):
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.warmup_reference = None
    commands = []
    bridge.publish = commands.append
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.01)
    bridge.hold(fixed=True)
    bridge.state.measured = [0.03, -0.002, -0.001, 0.01, 0.02, -0.01, 0.01]
    bridge.hold(fixed=True)
    assert commands == [[0.0] * 7, [0.0] * 7]
    # Stop/clutch still snapshots the current measured position once.
    bridge.hold()
    assert commands[-1] == [0.03, 0.0, 0.0, 0.01, 0.02, -0.01, 0.01]


def test_explicit_motor_tuning_rejects_invalid_and_incomplete_settings(tmp_path):
    from rebot_leader_tune import settings

    path = tmp_path / "tuning.yaml"
    path.write_text("jaw_torque_limit_nm: 1\njoint4_speed_limit_rad_s: 33\n")
    assert settings(path) == [(7, "0x700B", 1), (4, "0x7017", 33)]
    for bad in (".nan", "0", "15", "true"):
        path.write_text(f"jaw_torque_limit_nm: {bad}\njoint4_speed_limit_rad_s: 33\n")
        with pytest.raises(ValueError):
            settings(path)
    path.write_text("jaw_kp: 12\n")
    with pytest.raises(ValueError):
        settings(path)


def test_motor_tuning_requires_fresh_healthy_home_with_all_seven_off():
    from rebot_leader_tune import safe_home

    health = SimpleNamespace(
        id=list(range(1, 8)), enabled=[False] * 7, fault_bits=[0] * 7, comm_state=0
    )
    joint = SimpleNamespace(name=JOINT_NAMES, position=[0.0] * 7)
    assert safe_home((10.0, health), (10.0, joint), 10.0)
    assert not safe_home((10.0, health), (10.0, joint), 10.2)
    for i in range(7):
        health.enabled[i] = True
        assert not safe_home((10.0, health), (10.0, joint), 10.0)
        health.enabled[i] = False
    joint.position[3] = 0.081
    assert not safe_home((10.0, health), (10.0, joint), 10.0)
    joint.position[3] = math.nan
    assert not safe_home((10.0, health), (10.0, joint), 10.0)
    joint.position[3] = 0
    health.fault_bits[2] = 1
    assert not safe_home((10.0, health), (10.0, joint), 10.0)


def test_driver_thermal_trip_yields_to_one_backend_recovery_owner(following_bridge):
    bridge, published, target = following_bridge
    bridge.observe_thermal(SimpleNamespace(data=True))
    bridge.tick()
    assert bridge.thermal_tripped and not bridge.state.active
    assert not bridge.state.recover and published == []
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.observe_thermal(SimpleNamespace(data=False))
    assert not bridge.thermal_tripped
    assert not bridge.state.active and not bridge.state.recover
    assert bridge.state.fault and bridge.prepared_at is None
    assert not bridge.mirror(None, SimpleNamespace()).success
    assert published == []


def calibration_fixture():
    return {
        name: {
            "id": i,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": -90,
            "range_max": 90,
        }
        for i, name in enumerate(LEADER_NAMES)
    }


def test_reader_rejections_are_rate_limited_without_hiding_error_samples(
    monkeypatch, capsys
):
    import io
    import json
    from unittest.mock import Mock
    import rebot_leader

    # (sample time, read succeeds) with exact clock values: a long outage, a
    # rapid recurrence inside the shared 5 s window, then one after it.
    script = (
        [(t, False) for t in range(7)]
        + [(7, True), (8, False), (9, False), (10, False), (11, True)]
        + [(16, False)]
    )
    step = [0]
    bus = SimpleNamespace(ping=lambda motor_id: True, unlock=Mock(), close=Mock())

    def positions(bus, calibration):
        if script[step[0]][1]:
            return [0.0] * 7
        raise RuntimeError("servo timeout")

    def sleep(duration):
        step[0] += 1
        if step[0] == len(script):
            raise KeyboardInterrupt

    monkeypatch.setattr(rebot_leader, "read_positions", positions)
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        rebot_leader.run_reader(
            "/no-usb",
            calibration_fixture(),
            lambda *a, **kw: bus,
            output,
            clock=lambda: float(script[step[0]][0]),
            sleep=sleep,
        )
    messages = [json.loads(line) for line in output.getvalue().splitlines()]
    assert [(m["sample_time"], m.get("error")) for m in messages] == [
        (t, None if ok else "servo timeout") for t, ok in script
    ]
    assert capsys.readouterr().err.splitlines() == [
        f"Leader sample rejected ({count} reads this outage): servo timeout"
        for count in (1, 6, 3, 1)
    ]
    bus.close.assert_called_once()


def test_slow_reader_failures_throttle_by_warning_emission_time(monkeypatch):
    import io
    import json
    from unittest.mock import Mock
    import rebot_leader

    now_ms = [0]
    step = [0]
    read_durations_ms = [4900, 100, 100, 4900, 100]
    warning_times_ms = []

    class Warnings(io.StringIO):
        def write(self, text):
            if text.startswith("Leader sample rejected"):
                warning_times_ms.append(now_ms[0])
            return super().write(text)

    def positions(bus, calibration):
        now_ms[0] += read_durations_ms[step[0]]
        raise RuntimeError("servo timeout")

    def sleep(duration):
        step[0] += 1
        if step[0] == len(read_durations_ms):
            raise KeyboardInterrupt

    bus = SimpleNamespace(ping=lambda motor_id: True, unlock=Mock(), close=Mock())
    monkeypatch.setattr(rebot_leader, "read_positions", positions)
    monkeypatch.setattr(rebot_leader.sys, "stderr", Warnings())
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        rebot_leader.run_reader(
            "/no-usb",
            calibration_fixture(),
            lambda *a, **kw: bus,
            output,
            clock=lambda: now_ms[0] / 1000,
            sleep=sleep,
        )
    assert warning_times_ms == [4900, 10000]
    assert all(b - a >= 5000 for a, b in zip(warning_times_ms, warning_times_ms[1:]))
    messages = [json.loads(line) for line in output.getvalue().splitlines()]
    assert [message["sample_time"] for message in messages] == [0, 4.9, 5, 5.1, 10]
    assert [message["error"] for message in messages] == ["servo timeout"] * 5
    bus.close.assert_called_once()


def test_thermal_false_requires_new_home_preparation_and_never_reengages(
    ticking_bridge,
):
    bridge = ticking_bridge
    bridge.observe_thermal(SimpleNamespace(data=True))
    bridge.hardware_state.enabled = [False] * 7
    bridge.observe_thermal(SimpleNamespace(data=False))
    bridge.tick()
    assert not bridge.state.active and bridge.prepared_at is None
    assert not bridge.mirror(None, SimpleNamespace()).success
    assert not bridge.prepare(None, SimpleNamespace()).success  # torque still OFF
    bridge.hardware_state.enabled = [
        True
    ] * 7  # separate explicit HOME enable procedure
    bridge.state.measured[0] = 0.081
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.state.measured[0] = 0.0
    assert bridge.prepare(None, SimpleNamespace()).success
    assert not bridge.state.active
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert bridge.state.active


@pytest.mark.parametrize(
    "field,value",
    [
        ("id", 253),
        ("homing_offset", 1),
        ("drive_mode", 1),
        ("range_min", float("nan")),
        ("range_max", -90),
    ],
)
def test_unsupported_calibration_refuses_before_opening_serial(tmp_path, field, value):
    import json
    from rebot_leader import load_calibration

    data = calibration_fixture()
    data[LEADER_NAMES[0]][field] = value
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_calibration(path)


def test_direct_sdk_mapping_keeps_full_ranges_not_legacy_placeholder():
    from rebot_leader import read_positions

    raw = [120, 150, -180, -80, -90, 130, 30]
    bus = SimpleNamespace(
        read_angle=lambda motor_id: SimpleNamespace(raw_deg=raw[motor_id])
    )
    actual = read_positions(bus, calibration_fixture())
    assert actual == pytest.approx(
        [math.radians(q) for q in [120, 150, 180, 80, 90, 130, 180]]
    )


@pytest.mark.parametrize(
    "raw,expected", [(370, 10), (-350, 10), (730, 10), (180, 180), (-180, -180)]
)
def test_nearest_turn_unwrap_matches_seeed_window(raw, expected):
    from rebot_leader import unwrap_angle

    assert unwrap_angle(raw, -150, 150)[0] == expected


def test_reader_failure_and_shutdown_always_close_bus():
    import io
    import json
    from unittest.mock import Mock
    from rebot_leader import run_reader

    bus = SimpleNamespace(
        ping=lambda motor_id: True,
        unlock=Mock(),
        read_angle=Mock(side_effect=RuntimeError("USB missing")),
        close=Mock(),
    )
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        run_reader(
            "/no-usb",
            calibration_fixture(),
            lambda *a, **kw: bus,
            output,
            sleep=Mock(side_effect=KeyboardInterrupt),
        )
    message = json.loads(output.getvalue())
    assert message["error"] == "USB missing" and "positions" not in message
    assert bus.unlock.call_count == 7
    bus.close.assert_called_once()
    bus.close.reset_mock()
    bus.ping = lambda motor_id: False
    with pytest.raises(RuntimeError, match="not found"):
        run_reader("/no-usb", calibration_fixture(), lambda *a, **kw: bus, output)
    bus.close.assert_called_once()


def test_bridge_initialization_failure_reaps_its_reader(monkeypatch):
    import signal
    from unittest.mock import Mock
    import rebot_leader_bridge

    child = SimpleNamespace(
        stdout=SimpleNamespace(close=Mock()), terminate=Mock(), wait=Mock()
    )
    node = SimpleNamespace(destroy_node=Mock())
    shutdown = Mock()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rebot_leader_bridge.py",
            "--port",
            "/no-usb",
            "--calibration-file",
            "/existing.json",
            "--srdf-file",
            "/existing.srdf",
        ],
    )
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "init", lambda **kw: None)
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "create_node", lambda name: node)
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "shutdown", shutdown)
    monkeypatch.setattr(rebot_leader_bridge.subprocess, "Popen", lambda *a, **kw: child)
    monkeypatch.setattr(
        rebot_leader_bridge,
        "LeaderBridge",
        Mock(side_effect=RuntimeError("initialization failed")),
    )
    handlers = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        with pytest.raises(RuntimeError, match="initialization failed"):
            rebot_leader_bridge.main()
    finally:
        for number, handler in handlers.items():
            signal.signal(number, handler)
    child.terminate.assert_called_once()
    child.wait.assert_called_once_with(timeout=5)
    child.stdout.close.assert_called_once()
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


@pytest.mark.parametrize(
    "index,delta,direction",
    [(5, -0.65, "higher"), (5, 0.65, "lower"), (2, 0.7, "higher"), (2, -0.7, "lower")],
)
def test_alignment_instructions_account_for_leader_mapping_sign(
    index, delta, direction
):
    from rebot_leader_state import alignment_fault, joint_label

    target = [0.0] * 7
    target[index] = delta
    message = alignment_fault(target, [0.0] * 7)
    assert f"{joint_label(index)} toward a {direction} calibrated angle" in message
    assert "Star Arm's pose does not match reBot" in message
    assert "deg (" in message and "rad)" in message


def test_home_refusal_names_joint_and_gives_supported_recovery(home_bridge):
    home_bridge.state.measured[5] = -0.2
    message = home_bridge.prepare(None, SimpleNamespace()).message
    assert "wrist roll (joint6) -11.5 deg (-0.200 rad)" in message
    assert "reBot must be at HOME" in message
    assert "Run Return reBot Leader Home" in message
    assert "do not move it by hand or disable torque away from HOME" in message
    assert "run Mirror reBot Leader again" in message


def test_stale_refusal_explains_connection_and_retry(home_bridge):
    home_bridge.state.sample_time = 9.0
    response = home_bridge.prepare(None, SimpleNamespace())
    assert not response.success
    assert "fresh Star Arm or reBot joint data is missing" in response.message
    assert "Check the Star Arm USB connection" in response.message
    assert "once reBot is at HOME, restart the backend" in response.message
    assert "run Mirror reBot Leader again" in response.message


def test_reader_read_error_reaches_refusal_with_backend_restart(
    following_bridge, monkeypatch
):
    import io
    from unittest.mock import Mock
    from rebot_leader import run_reader

    bridge, published, target = following_bridge
    bus = SimpleNamespace(
        ping=lambda motor_id: True,
        unlock=Mock(),
        read_angle=Mock(side_effect=RuntimeError("servo 3 read timeout")),
        close=Mock(),
    )
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        run_reader(
            "/no-usb",
            calibration_fixture(),
            lambda *a, **kw: bus,
            output,
            clock=lambda: 10.01,
            sleep=Mock(side_effect=KeyboardInterrupt),
        )
    read, write = os.pipe()
    os.write(write, output.getvalue().encode())
    bridge.stream = SimpleNamespace(fileno=lambda: read)
    try:
        bridge.tick()
    finally:
        os.close(read)
        os.close(write)
    assert not bridge.state.active and bridge.state.recover
    following = bridge.mirror(None, SimpleNamespace())
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.5)
    retry = bridge.prepare(None, SimpleNamespace())
    for response in (following, retry):
        assert not response.success
        assert "Star Arm USB read failed: servo 3 read timeout" in response.message
        assert "once reBot is at HOME, restart the backend" in response.message


def test_exited_reader_refusal_names_cause_and_backend_restart(
    unprepared_ticking_bridge, monkeypatch
):
    bridge = unprepared_ticking_bridge
    errors = []
    bridge.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=lambda text: None, warn=lambda text: None, error=errors.append
        )
    )
    read, write = os.pipe()
    os.close(write)  # The reader process exited.
    bridge.stream = SimpleNamespace(fileno=lambda: read)
    try:
        bridge.tick()
        monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.5)
        bridge.tick()
    finally:
        os.close(read)
    assert len(errors) == 1
    for call in (bridge.prepare, bridge.mirror):
        response = call(None, SimpleNamespace())
        assert not response.success
        assert "Star Arm reader has exited" in response.message
        assert "rebot_leader_port" in response.message
        assert "Once reBot is at HOME, restart the backend" in response.message
        assert "run Mirror reBot Leader again" in response.message


def test_thermal_refusal_explains_latch_cooling_and_home_only_off(home_bridge):
    home_bridge.thermal_tripped = True
    for call in (home_bridge.prepare, home_bridge.mirror):
        response = call(None, SimpleNamespace())
        assert not response.success
        assert "temperature limit" in response.message and "latched" in response.message
        assert "turns torque off only at HOME" in response.message
        assert "torque stays on" in response.message
        assert "all seven motors OFF" in response.message
        assert "thermal_clear_c (75 C by default)" in response.message
        assert "Torque stays OFF" in response.message
        assert "explicitly run Mirror reBot Leader again at HOME" in response.message
        assert "hot at startup" in response.message
        assert "cool at HOME, then restart the backend" in response.message


def test_three_second_timeout_message_explains_recovery_and_retry(
    ticking_bridge, monkeypatch
):
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 13.02)
    ticking_bridge.tick()
    response = ticking_bridge.mirror(None, SimpleNamespace())
    assert not response.success
    assert "did not start within 3 seconds" in response.message
    assert "wait for recovery to finish" in response.message
    assert "Check the Star Arm USB connection" in response.message
    assert "once reBot is at HOME, restart the backend" in response.message
    assert "run Mirror reBot Leader again" in response.message


def latched_home_bridge(monkeypatch, home=True, moving=False, healthy=True):
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home(now=9.0)
    bridge.reader_exited = False
    bridge.real_hardware = True
    bridge.verified = bridge.scene_ready = True
    bridge.start_checked = True
    bridge.hardware_state = SimpleNamespace(
        id=list(range(1, 8)),
        comm_state=0,
        fault_bits=[0] * 7 if healthy else [1] + [0] * 6,
        enabled=[True] * 7,
    )
    bridge.recovery_started = bridge.recovery_future = bridge.recovery_result = None
    logs = []
    bridge.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=logs.append, warn=logs.append, error=logs.append
        )
    )
    assert bridge.state.engage(9.0)
    bridge.state.stop("Leader/follower sample or Objective heartbeat expired")
    bridge.fail_home("Native HOME recovery failed; torque stays on")
    for k in range(6):  # 0.5 s of joint states at 10 Hz
        t = 9.5 + 0.1 * k
        q = [0.0 if home else 0.3] + [0.0] * 6
        if moving:
            q[1] = 0.002 * k * 3
        monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda t=t: t)
        bridge.observe(SimpleNamespace(name=JOINT_NAMES, position=q))
    bridge.hardware_time = 9.95
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.0)
    return bridge, logs


def test_verified_return_home_clears_the_failed_home_latch(monkeypatch):
    bridge, logs = latched_home_bridge(monkeypatch)
    assert "automatic HOME recovery failed" in bridge.refusal_reason(10.0)
    bridge.recovery_started = 9.0
    bridge.recovery_future = bridge.recovery_result = SimpleNamespace()
    response = bridge.home_verified(None, SimpleNamespace())
    assert response.success and not bridge.home_failed and not bridge.state.recover
    assert (
        bridge.recovery_started
        is bridge.recovery_future
        is bridge.recovery_result
        is None
    )
    assert not bridge.state.active and bridge.state.fault
    assert any("latch cleared" in line for line in logs)
    bridge.state.accept({"positions": [0.0] * 7, "sample_time": 9.99}, 10.0)
    # With the latch and the stop gone, Mirror prepare may reset and run again.
    assert bridge.state.reset()
    assert "automatic HOME recovery failed" not in (bridge.refusal_reason(10.0) or "")


@pytest.mark.parametrize(
    "kwargs", [dict(home=False), dict(moving=True), dict(healthy=False)]
)
def test_unverified_home_keeps_the_failed_home_latch(monkeypatch, kwargs):
    bridge, _ = latched_home_bridge(monkeypatch, **kwargs)
    response = bridge.home_verified(None, SimpleNamespace())
    assert not response.success and bridge.home_failed
    assert "automatic HOME recovery failed" in bridge.refusal_reason(10.0)


@pytest.mark.parametrize(
    "condition",
    [
        "stale_arm",
        "stale_motors",
        "disabled_motor",
        "thermal_trip",
        "active",
        "short_window",
    ],
)
def test_home_verification_keeps_latch_when_a_safety_gate_is_missing(
    monkeypatch, condition
):
    bridge, _ = latched_home_bridge(monkeypatch)
    if condition == "stale_arm":
        bridge.state.measured_time = 9.8
    elif condition == "stale_motors":
        bridge.hardware_time = 9.4
    elif condition == "disabled_motor":
        bridge.hardware_state.enabled[-1] = False
    elif condition == "thermal_trip":
        bridge.thermal_tripped = True
    elif condition == "active":
        bridge.state.active = True
    elif condition == "short_window":
        bridge.home_window = bridge.home_window[-2:]
    response = bridge.home_verified(None, SimpleNamespace())
    assert not response.success and bridge.home_failed and bridge.state.recover


@pytest.mark.parametrize(
    "condition",
    [
        "active",
        "engaging",
        "prepared",
        "recover",
        "recovery_started",
        "recovery_future",
        "recovery_result",
        "home_failed",
        "thermal_trip",
    ],
)
def test_relax_idle_check_refuses_a_live_or_pending_leader_without_changing_it(
    monkeypatch, condition
):
    bridge, _ = latched_home_bridge(monkeypatch)
    bridge.home_failed = False
    bridge.state.recover = False
    bridge.prepared_at = None
    if condition == "active":
        bridge.state.active = True
    elif condition == "engaging":
        bridge.state.engaging = True
    elif condition == "prepared":
        bridge.prepared_at = 10.0
    elif condition == "recover":
        bridge.state.recover = True
    elif condition == "home_failed":
        bridge.home_failed = True
    elif condition == "thermal_trip":
        bridge.thermal_tripped = True
    else:
        setattr(bridge, condition, SimpleNamespace())
    before = (
        bridge.state.active,
        bridge.state.recover,
        bridge.prepared_at,
        bridge.home_failed,
    )
    response = bridge.idle(None, SimpleNamespace())
    assert not response.success and "Relax refused" in response.message
    assert before == (
        bridge.state.active,
        bridge.state.recover,
        bridge.prepared_at,
        bridge.home_failed,
    )


def test_relax_idle_check_succeeds_without_leader_input_or_motor_commands(monkeypatch):
    bridge, _ = latched_home_bridge(monkeypatch)
    bridge.home_failed = False
    bridge.state.recover = False
    bridge.prepared_at = None
    bridge.state.target = None
    bridge.hardware_state = None
    assert bridge.idle(None, SimpleNamespace()).success
    assert not bridge.state.active and bridge.prepared_at is None


def test_home_verified_without_a_latch_reports_success_and_changes_nothing(monkeypatch):
    bridge, _ = latched_home_bridge(monkeypatch)
    bridge.home_failed = False
    bridge.state.recover = True
    response = bridge.home_verified(None, SimpleNamespace())
    assert response.success and bridge.state.recover  # Own recovery is untouched.
