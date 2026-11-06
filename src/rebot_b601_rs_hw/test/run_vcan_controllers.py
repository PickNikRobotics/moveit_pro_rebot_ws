#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Prepare the real-hardware model, then run the virtual-only controller tests."""

import subprocess
import sys

sys.dont_write_bytecode = True  # Keep the source tree clean when run from ctest.
from prepare_vcan_model import prepare  # noqa: E402


if __name__ == "__main__":
    source, model, executable, *arguments = sys.argv[1:]
    prepare(source, model)
    sys.exit(subprocess.call([executable, *arguments]))
