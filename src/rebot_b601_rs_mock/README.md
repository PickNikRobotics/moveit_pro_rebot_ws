# Stock reBot mock backend

The shared stock B601-RS description, native jaw, MoveIt settings, controller
configuration and basic Objectives live here. Mock mode uses
`mock_components/GenericSystem`. Geometry comes from the pinned complete Seeed
description repository through `rebot_description`. See the workspace README
for backend selection and the description NOTICE for derivation details.

## Teleoperation on the mock

`Teleoperate` starts in waypoint mode on `joint_trajectory_admittance_controller`,
which owns all seven position interfaces. Jog modes switch controllers with
automatic deactivation, so exactly one controller claims each interface:

| UI mode | Group | Controller | Claims | Input topic |
| --- | --- | --- | --- | --- |
| Joint jog (arm) | `manipulator` | `joint_velocity_controller` | `joint1`–`joint6` | `/joint_jog/manipulator` |
| Joint jog (gripper) | `gripper` | `gripper_joint_velocity_controller` | `joint7` | `/joint_jog/gripper` |
| Cartesian jog | `manipulator` (tip `gripper_end`) | `velocity_force_controller` | `joint1`–`joint6` | `/pose_jog/manipulator` |
| Waypoint, marker, Open/Close Gripper | `manipulator`, `gripper` | trajectory controller | `joint1`–`joint7` | Objective |

The fingers (`gripper_joint1/2`) are mimics of `joint7` and have no command
interface. `Open Gripper` / `Close Gripper` plan the `gripper` group only, so the
arm is not part of those goals (the named waypoints also carry arm values, which
are ignored).

`test/test_teleop_contract.py` checks this wiring offline.
`test/mock_teleop_smoke.py OUT_DIR` runs the operator sequence against a live
mock runtime. It plays the web UI (it answers the `/do_teleoperate` bridge and
publishes the jog topics), refuses any hardware other than
`mock_components/GenericSystem`, and writes `joint_states.jsonl` (every check's
joint states) and `summary.json`. It covers: six-axis joint jog, gripper jog,
Cartesian jog axes and rotation, Open/Close from Cartesian jog, return to joint
jog, interactive-marker pose goal, waypoint moves, joint-limit stop,
exit/cancel while jogging, and a trajectory Objective after Teleoperate.

`test/evidence/` holds one recorded run: `summary.json` (58 passed, 4 observed,
0 failed, 12171 `/joint_states` samples), a 30 fps rendering of those joint
states (`mock_teleop_joint_states.mp4`, `.png`) and the script that renders it.
This is a plot of recorded joint states, not a capture of the web UI.

Observed behaviour (mock, MoveIt Pro 10.2.0-rc10):

- Releasing input or stopping messages stops motion at once (zero drift); no
  trajectory is replayed when switching back from a jog mode.
- Jog toward a joint limit stops 0.02 rad short (`joint_limit_position_tolerance`).
  After that stop the arm jog ignored all axes until the Teleoperate session was
  restarted (a waypoint move still works, and in some runs jog resumed after it).
- Leaving Teleoperate keeps the last jog controller active; the next trajectory
  Objective (for example `Move to Rest`) deactivates it and re-activates the
  trajectory controller.
