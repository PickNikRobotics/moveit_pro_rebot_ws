#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Validate torque-off poses and derive standard waypoint/Objective resources.

The generator is a build tool, not a Behavior or a torque-control owner.
Runtime Python HOME consumers use the same validated entry.
"""
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re

import yaml

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))


class UniqueKeyLoader(yaml.SafeLoader):
    """Reject repeated YAML keys instead of silently taking the last value."""


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f"Duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node)
    return result


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping
)


def number(value, label, *, positive=False):
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or (positive and value <= 0)
    ):
        raise ValueError(f"{label} must be finite{' and positive' if positive else ''}")
    return float(value)


def joint_values(values, label):
    if not isinstance(values, dict) or set(values) != set(JOINT_NAMES):
        raise ValueError(f"{label} must contain exactly joint1..joint7")
    positions, tolerances = [], []
    for joint in JOINT_NAMES:
        entry = values[joint]
        if not isinstance(entry, dict) or set(entry) != {
            "position_rad",
            "tolerance_rad",
        }:
            raise ValueError(f"{label}.{joint} needs position_rad and tolerance_rad")
        positions.append(number(entry["position_rad"], f"{label}.{joint}.position_rad"))
        tolerances.append(
            number(
                entry["tolerance_rad"], f"{label}.{joint}.tolerance_rad", positive=True
            )
        )
    return tuple(positions), tuple(tolerances)


@dataclass(frozen=True)
class SafePose:
    name: str
    positions: tuple
    tolerances: tuple
    window_s: float
    max_span_rad: float
    max_abs_velocity_rad_s: float = 0.01

    def matches(self, positions, *, arm_only=False):
        """HOME arm gates intentionally ignore jaw; relax itself checks all seven."""
        size = 6 if arm_only else 7
        return (
            positions is not None
            and len(positions) == 7
            and all(
                math.isfinite(q) and abs(q - target) <= tolerance
                for q, target, tolerance in zip(
                    positions[:size], self.positions[:size], self.tolerances[:size]
                )
            )
        )

    def stationary(self, window, now, *, arm_only=False):
        """Require a complete fresh position-span window, not one quiet sample."""
        if (
            len(window) < 2
            or window[-1][0] - window[0][0] < self.window_s
            or not 0 <= now - window[-1][0] < 0.1
            or any(
                not 0 < b[0] - a[0] <= 0.1 + 1e-9 for a, b in zip(window, window[1:])
            )
        ):
            return False
        size = 6 if arm_only else 7
        return all(
            all(len(q) == 7 and math.isfinite(q[j]) for _, q in window)
            and max(q[j] for _, q in window) - min(q[j] for _, q in window)
            <= self.max_span_rad
            for j in range(size)
        )


def load_poses(path):
    values = yaml.load(Path(path).read_text(), Loader=UniqueKeyLoader)
    if not isinstance(values, dict) or set(values) != {"version", "relax_safe_poses"}:
        raise ValueError("Safe poses file needs version and relax_safe_poses")
    if type(values["version"]) is not int or values["version"] != 1:
        raise ValueError("Safe poses version must be 1")
    entries = values["relax_safe_poses"]
    if not isinstance(entries, list) or not 1 <= len(entries) <= 32:
        raise ValueError("relax_safe_poses must contain 1..32 poses, HOME first")
    poses = []
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != {
            "name",
            "joints",
            "stillness",
        }:
            raise ValueError("Each pose needs name, joints and stillness")
        name = entry["name"]
        if not isinstance(name, str) or not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_ -]*", name
        ):
            raise ValueError("Pose name must be a nonempty plain name")
        if any(p.name == name for p in poses):
            raise ValueError(f"Duplicate pose name: {name}")
        stillness = entry["stillness"]
        required = {"window_s", "max_span_rad"}
        if (
            not isinstance(stillness, dict)
            or not required <= set(stillness)
            or set(stillness) - required - {"max_abs_velocity_rad_s"}
        ):
            raise ValueError(f"{name}.stillness needs window_s and max_span_rad")
        window_s = number(stillness["window_s"], f"{name}.window_s", positive=True)
        if window_s > 10:
            raise ValueError(f"{name}.window_s must be <= 10 seconds")
        positions, tolerances = joint_values(entry["joints"], f"{name}.joints")
        poses.append(
            SafePose(
                name,
                positions,
                tolerances,
                window_s,
                number(
                    stillness["max_span_rad"], f"{name}.max_span_rad", positive=True
                ),
                number(
                    stillness.get("max_abs_velocity_rad_s", 0.01),
                    f"{name}.max_abs_velocity_rad_s",
                    positive=True,
                ),
            )
        )
    if poses[0].name != "HOME":
        raise ValueError("HOME must be the first relax-safe pose")
    return tuple(poses)


def default_file():
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("rebot_b601_rs_hw"))
        / "config/relax_safe_poses.yaml"
    )


def load_home(path=None):
    return load_poses(default_file() if path is None else path)[0]


def generate_waypoints(template, home):
    entries = yaml.safe_load(Path(template).read_text())
    matches = [w for w in entries if w.get("name") == "Zeroed Rest"]
    if len(matches) != 1 or matches[0]["joint_state"]["position"] != "HOME":
        raise ValueError(
            "Waypoint template needs exactly one Zeroed Rest position: HOME"
        )
    state = matches[0]["joint_state"]
    state["position"] = [home.positions[JOINT_NAMES.index(j)] for j in state["name"]]
    return (
        "# Generated from rebot_b601_rs_hw/config/relax_safe_poses.yaml; do not edit.\n"
        + yaml.safe_dump(entries, sort_keys=False)
    )


def generate_objective(template, home):
    # Yaw alignment precision is a recovery-path choice, not a second HOME
    # definition. Preserve the existing 0.005 rad yaw-first centering threshold.
    yaw_tolerance = min(home.tolerances[4], 0.005)
    tokens = {
        "HOME_ARM_SKIP": " &amp;&amp; ".join(
            f"home_{j} &gt;= {q - t:.17g} &amp;&amp; home_{j} &lt;= {q + t:.17g}"
            for j, q, t in zip(JOINT_NAMES[:6], home.positions, home.tolerances)
        ),
        "HOME_YAW": f"{home.positions[4]:.17g}",
        "HOME_JAW": f"{home.positions[6]:.17g}",
        "HOME_SETTLE": f"{home.window_s + 0.1:.17g}",
        "HOME_YAW_SKIP": f"yaw_position &gt;= {home.positions[4] - yaw_tolerance:.17g} &amp;&amp; yaw_position &lt;= {home.positions[4] + yaw_tolerance:.17g}",
    }
    tokens.update(
        {
            f"HOME_{joint.upper()}": f"{q:.17g}"
            for joint, q in zip(JOINT_NAMES, home.positions)
        }
    )
    text = Path(template).read_text()
    for key, value in tokens.items():
        text = text.replace(f"@{key}@", value)
    if re.search(r"@HOME_[A-Z_0-9]+@", text):
        raise ValueError("Unresolved HOME token")
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", required=True, type=Path)
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--kind", required=True, choices=("waypoints", "objective", "srdf")
    )
    args = parser.parse_args()
    home = load_home(args.poses)
    render = generate_waypoints if args.kind == "waypoints" else generate_objective
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render(args.template, home))


if __name__ == "__main__":
    main()
