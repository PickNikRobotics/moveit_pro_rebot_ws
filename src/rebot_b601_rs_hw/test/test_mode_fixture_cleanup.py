# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Verify the memory-only mode fixture unwinds failures instead of hiding or hanging them."""

import os
import signal
import subprocess

import pytest


@pytest.mark.parametrize(
    "fault,reason",
    [
        ("--fail-during-switch", "injected failure during switch"),
        ("--fail-after-activation", "injected failure after activation"),
        ("--reject-switch", "manager switch failed"),
    ],
)
def test_failed_mode_fixture_joins_worker_and_releases_resources(fault, reason):
    process = subprocess.Popen(
        [
            os.environ["MODE_TEST_EXECUTABLE"],
            fault,
            "--ros-args",
            "--log-level",
            "error",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        output, _ = process.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        # CM's SIGTERM handler can itself wait on the controller-list lock. Bound
        # both the test and graceful termination, and reap only our process group.
        os.killpg(process.pid, signal.SIGTERM)
        try:
            output, _ = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=3)
        pytest.fail(f"{fault} did not exit within 15 seconds:\n{output}")
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=3)
    assert process.returncode == 1, output
    assert f"Mode test failed: {reason}" in output, output
    assert output.count("Controller switch worker joined") == 1, output
    assert (
        "Fixture cleanup complete: controllers finalized, 13 command loans released"
        in output
    ), output
    assert "ROS context shutdown complete" in output, output
    assert "BEFORE gain-service ACK" not in output, output
    assert "3/3 exact source Objective runs PASS" not in output, output
