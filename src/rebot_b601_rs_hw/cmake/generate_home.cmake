# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
# Configure-time generation is repeated when the canonical pose or template changes.
# Pure Python/YAML: the mock package does not link or launch a hardware dependency.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(REBOT_SAFE_POSES_FILE "${CMAKE_CURRENT_LIST_DIR}/../config/relax_safe_poses.yaml")
set(REBOT_SAFE_POSES_GENERATOR "${CMAKE_CURRENT_LIST_DIR}/../scripts/generate_home.py")
set(REBOT_SAFE_POSES_MODULE "${CMAKE_CURRENT_LIST_DIR}/../python/rebot_safe_poses/__init__.py")
function(rebot_generate_home kind template output)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${REBOT_SAFE_POSES_FILE}" "${REBOT_SAFE_POSES_GENERATOR}" "${REBOT_SAFE_POSES_MODULE}" "${template}")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${REBOT_SAFE_POSES_GENERATOR}"
      --poses "${REBOT_SAFE_POSES_FILE}" --kind "${kind}"
      --template "${template}" --output "${output}"
    COMMAND_ERROR_IS_FATAL ANY)
endfunction()
