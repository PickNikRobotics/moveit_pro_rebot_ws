# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The stock leader collision check on the office model the office config selects."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest
import xacro

sys.path.insert(
    0, str(Path(get_package_prefix("rebot_b601_rs_hw")) / "lib/rebot_b601_rs_hw")
)
from rebot_leader_collision import LeaderCollision  # noqa: E402

SHARE = Path(get_package_share_directory("picknik_boston_rebot_b601_rs_hw"))
FOLDED = [
    0.080285143927,
    0.0,
    0.017453292520,
    -0.043633231300,
    0.057595864484,
    -0.062831851407,
    0.0,
]


@pytest.fixture(scope="module")
def checker():
    urdf = xacro.process_file(
        str(SHARE / "description/rebot.urdf.xacro"),
        mappings={"hardware_interface": "mock"},
    ).toxml()
    return LeaderCollision(urdf, SHARE / "config/moveit/rebot.srdf")


def test_folded_link_contact_is_the_allowed_pair(checker):
    assert checker.check_pose(FOLDED) == ""
    checker.scene.allowed_collision_matrix.set_entry("link2", "link5", False)
    try:
        assert checker.check_pose(FOLDED) == "Collision: link2 / link5"
    finally:
        checker.scene.allowed_collision_matrix.set_entry("link2", "link5", True)


def test_engage_seed_below_a_lower_bound_has_a_clear_path(checker):
    # Measured below the J2/J3 lower bounds, seeded at HOME by engage.
    measured = [0.0, -0.03, -0.02, 0.0, 0.0, 0.0, -0.0001]
    assert checker.check_path(measured, [0.0] * 7) == ""
