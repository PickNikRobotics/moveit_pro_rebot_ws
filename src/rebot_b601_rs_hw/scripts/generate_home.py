#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Build entry point; pure YAML generation without a ROS installation."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from rebot_safe_poses import main  # noqa: E402

if __name__ == "__main__":
    main()
