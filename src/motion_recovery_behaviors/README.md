# Motion recovery Behaviors

A small first-party MoveIt Pro 10.x package containing exactly three generic
Behaviors. It has no reBot model, controller names, hardware service, recorder,
camera or application dependency. Register it explicitly in a config:

```yaml
objectives:
  behavior_loader_plugins:
    motion_recovery:
      - motion_recovery_behaviors::MotionRecoveryBehaviorsLoader
```

- **ClampRobotStateToBounds** copies a RobotState and clamps each named scalar
  joint to its model position bounds; unknown joints and non-finite positions
  fail. It does not establish collision freedom.
- **PlanToJointGoalFromBoundedStart** adapts the installed `PlanToJointGoal`
  planning-scene client to clamp the measured start. Its planner, ports,
  collision checking, service deadlines and cancellation remain inherited.
  Pair it with the same bounded start for command anchoring. Native planner
  execution requires a valid MoveIt Pro license.
- **AnchorTrajectoryToCommand** retains the original command/measurement policy:
  missing or stale (>0.10 rad) commands use measurement, omitted joints are held,
  optional measured joints retain their measured start, and quintic fade-out
  preserves the planned endpoint. The caller commits `end_state` only after
  successful execution. This is not a hardware interlock or continuous measured
  tracking, and does not certify no-snap behavior.

## Provenance and temporary location

Source is extracted, not reimplemented, from
[moveit_pro_example_ws](https://github.com/PickNikRobotics/moveit_pro_example_ws)
main `c7422c8853e495f0e481daa639c2919be24d73f9`:

- `3a14193f5d2dc11c80654891fd3dda28c52091a8`: anchoring source and seven
  anchoring tests from `rebot_bench_behaviors`.
- `fd6f446aba27711a30b09ba974fe5ad1992b77f7`: bounding helper and native planner
  adapter. Original BSD notices, authorship and path-filtered commits are retained.
- Source HOME/waypoint integration tests are retained by `rebot_b601_rs_hw`;
  the office jaw/pose and two recorder tests remain on the Boston branch.

Packaging changes only the C++ namespace/include paths, explicit loader and
catalog grouping for the bounding helpers. Behavior IDs and functional source
remain unchanged. This package is temporarily carried in the reBot workspace
pending a future common-package migration and API review; it is not a general
recovery framework. The Boston application consumes this same implementation
rather than registering duplicate helper IDs. No new common repository is
required by this package.

Tests cover the original anchoring contract, plugin loading and bounded-state
success/failure. Native HOME tree tests cover order, fallback, speed scaling and
jaw preservation with deterministic motion seams; explicitly skipped licensed
planner cases are not physical or planner acceptance evidence.
