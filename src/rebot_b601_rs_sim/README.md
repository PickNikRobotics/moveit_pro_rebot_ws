# Stock reBot MuJoCo simulation

`rebot_b601_rs_sim` inherits the URDF/SRDF, planning groups, controllers,
waypoints and teleoperation Objectives from `rebot_b601_rs_mock`. The explicit
`hardware_interface: mujoco` selector instantiates only
`picknik_mujoco_ros/MujocoSystem`. Selecting `rebot_b601_rs_mock` still selects
`mock_components/GenericSystem`; neither expanded backend contains CAN GPIOs
or torque-enable parameters. The simulation uses the blank driver launch.
Do not select `rebot_b601_rs_hw` for this workflow: that is the physical backend.

## Model and contact scope

The stock model generator and explicit selectors already existed. It generates
body transforms, nominal inertias, joint axes, limits and stock visual assets
from the inherited URDF. Licensed B601-RS simulation adaptations also exist in
[the public bench example](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/main/src/rebot_bench_sim).
This package reuses that joint convention and split PLA-pad contact approach,
not its office calibration, trim, cameras, grasp proxy or welds. Stock meshes
remain unmodified `package://rebot_description/...` references; the installed
CERN-OHL-W-2.0 license and description notice accompany them.

The installed `description/scene.xml` has nine joint coordinates (seven motors
and two passive jaws), seven joint-named position actuators and two keyframes:

- `default`: zeroed stock rest, zero velocity/control, with the contact block
  parked away from the arm. Normal planning and teleoperation use this state.
- `contact_smoke`: inherited Raised arm pose and fully open jaw; a fixed 50 mm
  box is placed at `[-0.01, 0, 0]` in `gripper_end`. This intentionally isolated
  fixture demonstrates transmission of contact through the native jaws.

Only the two separately modeled PLA pads and the box collide. All other
visual geoms remain non-colliding. The box is a fixed mocap body, not an object
that can be lifted. It is **not added to the MoveIt planning scene**: selecting
`contact_smoke` deliberately creates a controlled obstruction. Do not jog or
plan the arm in that keyframe. Close the jaw, reopen it, then reset to `default`.
No grasp attachment, self-contact, bench/floor contact, payload behavior or
insertion fidelity is claimed. There are no cameras or simulated sensors.

The jaw equalities encode the equivalent inverse relation
`joint7 = 125 * gripper_joint{1,2}`. Expressing the residual in motor radians
rather than jaw metres prevents the soft equality from letting the exposed
motor report closed while the pads remain blocked. The simulation-only jaw
uses kp12/kv0.05 and a **±1 Nm** squeeze cap; the URDF's nominal ±14 Nm bound
is unchanged. Pad/block `solimp="0.999 0.999 0.001"` limits fixture penetration.
These numerical settings are not measured physical gains or material data.

## Reset

`Reset MuJoCo Sim` lists and deactivates active command controllers, calls
`ResetMujocoKeyframe`, then restores the previous command controllers. The
joint-state broadcaster stays active. Ordinary reset failure also restores
controllers and returns failure; it is never converted to success. The
physical child clears the MuJoCo loader and replaces this Objective library,
so reset is not offered against CAN hardware.

**Known limitation (MoveIt Pro 10.2.0-rc10):** a keyframe reset that reports
success can intermittently leave `MujocoSystem` commanding the pre-reset pose.
The position actuators then pull the arm and jaw back to where they were, and
the restored trajectory controller holds that pose. This is an upstream
`MujocoSystem` command-resync race, not something this configuration can
remove, and no fixed release has been verified. Measured on a shared 24-core
host with alternating `default`/`contact_smoke` resets: 3 of 60 snapped back
headless (moderate load), 6 of 60 at load average about 80, and 1 to 5 of 60
with the interactive viewer under heavy load. Waiting 0.2 s before restoring
controllers did not help; repeating the reset only lowered the rate (2 of 60
at load average about 80), and an immediate retry repeated the snap-back in 2
of 3 cases, so this Objective keeps the single upstream-shaped reset. After a reset, confirm the measured joint state reached
the keyframe and run the reset again if it did not.

Canceling reset during its controller handoff can leave command controllers
inactive. Recover by restarting the **same named instance**
(`moveit_pro down --instance <name>`, then `moveit_pro run` with the same
`--instance <name>`).
Do not restart unrelated deployments. No point-cloud snapshot cleanup is needed
because this configuration has no perception capture.

## Reproducible simulation-only smoke run

Build the workspace with the installed MoveIt Pro 10.x image and pinned
submodules as described in the root README. Start a unique named instance:

```bash
moveit_pro run --user-workspace "$PWD" --config-package rebot_b601_rs_sim \
  --instance rebot_sim_smoke --headless
```

In another terminal, run the installed probe in that instance. It checks the
active hardware plugin before sending any Objective and refuses to preempt
an active Objective. A frontend key/license uses the normal installed CLI
configuration; neither belongs in the repository.

`moveit_pro shell` runs one-shot commands as root in an unsourced login
shell, so switch to the workspace user and source its ROS environment:

```bash
moveit_pro shell --instance rebot_sim_smoke --service runtime -- \
  gosu "$USER" bash -c 'source ~/.moveit-pro-bashrc && \
  ros2 run rebot_b601_rs_sim run_smoke.py --output /tmp/rebot_sim_smoke'
moveit_pro down --instance rebot_sim_smoke
```

`moveit_pro shell` exits 0 even when the probe fails, so read `"passed"` in
`summary.json` (or the printed result) rather than the shell's exit status.
`summary.json` and `state.jsonl` contain measured joint states, simulator qpos
and its latched schema, trajectory references and terminal Objective results. Save them before removing the runtime container. The probe
executes Raised planning, bounded J1 and base-X pose jogs through the same
MoveIt Pro safety-filtered input topics as Desktop teleoperation,
`/joint_jog/manipulator` and `/pose_jog/manipulator`, free-space open/close,
contact close/reopen, and default/invalid-key reset. When a reset's measured
hold misses its keyframe, the probe counts it in `reset_snapbacks` (with
`reset_snapback_phases`), runs the reset once more and fails if that also
misses. `--reset-cycles N` adds
N alternating `contact_smoke`/`default` reset pairs with a measured hold check
after each; it reproduces the reset limitation above and is off by default.
It uses `/do_objective`, not direct controller trajectory commands. Visual Desktop/IMarker acceptance
is not part of this headless probe.

A successful contact smoke **expects Close Gripper to fail** on J7 path
tolerance while the measured aperture is blocked. That is contact evidence,
not a successful grasp. Reopening and final reset must succeed. The standalone
model test additionally measures nonzero contact forces and compares the motor
and passive-jaw coordinates; replaying qpos alone is not a force measurement.

Focused package tests in a sourced workspace:

```bash
colcon test --packages-select rebot_b601_rs_mock rebot_b601_rs_sim rebot_b601_rs_hw \
  --executor sequential
colcon test-result --all --verbose
```

## Timing and validation limits

The model timestep is **0.003 s**, integrator `implicitfast`; the inherited
controller manager updates at **100 Hz** (10 ms). These are separate rates,
not an integer steps-per-control-cycle assumption. Installed MoveIt Pro
10.2.0-rc10 advances MuJoCo in its own wall-paced thread, publishes simulator
qpos at 60 Hz, and does not publish `/clock`. All consumers keep
`use_sim_time: false`. Joint-state timestamps must advance and remain fresh.

A regression checks the jaw's 3 rad/s, 6 rad/s² close profile at 0.7 simulation
real-time factor against the inherited 0.20 rad path tolerance. Do not loosen
that tolerance to hide slow simulation. Close the interactive viewer for
headless timing validation; do not change the timestep to compensate for load.

Validated on MoveIt Pro **10.2.0-rc10**, ROS 2 **Jazzy**, ros2_control **4.45.2**
and MuJoCo **3.6.0**, reusing existing images without an image build. The simulation package has **7 passing model/config cases**; the three backend
ObjectiveServer cases pass. `colcon test-result` reports **zero failures**; the
three license-gated HOME planner cases skip without a license and pass
separately in the licensed runtime.

One recorded cold-start smoke has **5,839 joint states**, **3,459 simulator
snapshots** and **945 trajectory-reference messages**: J1 moves +0.04729 rad;
base-X pose jog moves +9.146 mm with <0.005 mm cross-axis drift. Raised's largest
arm error is 1.18e-6 rad. The blocked jaw holds at 3.09552 rad; passive jaws hold
at 0.0248053 m. Contact close is rejected, reopen succeeds, and default reset
returns within 1e-8 rad while invalid-key reset fails without losing controller
claims. Joint states run near 100 Hz, with <22 ms observed age; the arm-motion
simulation/wall-time ratio is 0.965 in this run, not a platform guarantee.

The native 6 s contact probe measures **62.744 N** peak normal force,
**0.1215 mm** maximum penetration and **0.0412 mm** maximum mimic error. Default
hold stays finite over 1,000 steps, with drift below 1e-6 rad/m. These tests and
the actual runtime trace are separate evidence. No physical bus, torque, arm
motion, grasp-fidelity or multi-platform timing acceptance is claimed. The headless runtime emits the
core no-octomap-sensor diagnostic because this config has no 3D sensor; no
perception acceptance is claimed.
