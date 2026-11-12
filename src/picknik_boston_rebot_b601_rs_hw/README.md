# Boston reBot B601-RS

Office application source for **MoveIt Pro 10.x / ROS 2 Jazzy**. Select the matching
`picknik_boston_rebot_b601_rs_hw` config. It inherits
`rebot_b601_rs_mock → rebot_b601_rs_sim → rebot_b601_rs_hw` and adds the Boston
assembly, calibrated placement, OAK-D/Gemini 305, USB leader, collection/fit
and cube application sources. Boulder and San Diego are not modeled here.

Optional source twins are
[`picknik_boston_rebot_b601_rs_sim`](../picknik_boston_rebot_b601_rs_sim/README.md)
for the cube bench and
[`picknik_boston_rebot_b601_rs_peg_sim`](../picknik_boston_rebot_b601_rs_peg_sim/README.md)
for the frozen peg registration. They explicitly override the physical backend,
launch and active-gravity defaults; selecting either does not start real drivers,
the guard or a USB leader. This hardware config does not reference or depend on
those child scenes. Both retain nominal dynamic inertias and their documented
contact/calibration limitations.

**Migration checkpoint, not physical readiness.** No physical startup, CAN,
gain write, motion or recording was performed. The driver launch includes the
stock physical-only thermal guard and native HOME; office recovery keeps its
trajectory recorder. This is software dependency closure, not commissioning.
No new sag, float-quality, thermal-recovery or no-snap certification is claimed.

## Model and tuning boundary

The shared arm model owns joints, axes, stock collision meshes, native jaw,
controller interfaces, limits and spring tuning. The wrapper adds the exact
printed trim, calibrated physical wrist-camera frame and Boston transforms;
stock configurations in the same branch retain nominal geometry and zero
placement. See
[description provenance](description/NOTICE.md).

Boston hardware sets `boston_gravity_fit: true`. This selects six **static
gravity-equivalent inertial origins** from
`config/gravity/identified_first_moments.yaml`, with gravity gains
`[0,1,1,1,0,0]` and zero additional payload. The fit already absorbs the observed
empty jaw, camera and cables; adding stock finger payload again would double
count them. These fitted origins are not uniquely measured physical CoMs or
valid dynamic inertias. The wrapper refuses to use them with mock or MuJoCo;
nominal masses, inertia tensors and origins are used for simulation.

The 2026-11-01 fit used24poses/48opposing windows,20training/4held-out poses,
rank10. J2–J4 improved on held-out torque; J5/J6 did not show general improvement.
No new fit or raw bench recording is shipped here. Source limitations and held-out
numbers are documented in [gravity calibration](docs/gravity-calibration.md).

Reusable stock tuning remains the integrated settings: J2/J3 lower allowances
−0.02/−0.01rad, shared path tolerance0.20rad, J2 feedforward cap18Nm,
kp `[50,150,150,50,50,50]`, kd `[3,10,10,5,4,4]`, jawkp12/kd0.5 and100Hz.
Fitted gravity, scene/camera calibration and fixtures remain office-only.

## Sensors and assembly

- Fixed OAK-D:640×400,30Hz, full sensor ISP1/2 scaling, aligned depth and
  compressed color topic. Boston depth correction is0.96 before point-cloud creation.
- Wrist Gemini 305 (serial `CV2F861000FP`):1280×800,10Hz MJPG; no depth/IR
  launch and no duplicate driver TF. Its `gripper_end` to color-optical-frame
  mount is hand-eye calibrated for this unit, not eye-measured. It uses the
  closed-form 11-view SolveEyeInHandCalibration result, not the bundle-refined
  pose. Factory `camera_info` stays in use. The inherited stock launch supplies
  the driver.
- Scene camera/base transforms follow the recorded ChArUco calibration. The
  scene camera mount remains uncalibrated. Camera/mount collision and dynamic
  inertias are incomplete. Verify clearance before any separately authorized
  hardware use.
- Optional scene-camera absence warns and skips only that camera. Its launch's
  USB descriptor read is not run during offline tests; tests supply fixture descriptors.
- The office Star Arm leader enables the stock
  [optional leader](../rebot_b601_rs_hw/README.md#optional-star-arm-leader) with
  the stock calibration file and engage defaults; this config sets only
  `rebot_leader_enabled`, the USB port, the leader loader/library and
  `leader_position_controller`. Do not install a training stack or rewrite
  calibration.

## Mirror reBot Leader

Mirror, its engage/handoff, Stop → HOME → Relax at HOME sequence and the
fail-closed `reBot Check Leader Idle` / `reBot Notify Leader Home` subtrees are
stock pieces, documented in the
[stock hardware README](../rebot_b601_rs_hw/README.md#mirror-rebot-leader-engage).
The office config lists `rebot_b601_rs_hw/leader_objectives` after the inherited
stock library, so those subtrees replace the no-leader ones, and checks leader
targets against this config's own SRDF (office trim and camera mounts). HOME
and Relax at HOME come from the stock
[generated Objective library](../rebot_b601_rs_hw/README.md#generated-objective-library).
A Boston-derived config with the leader disabled must put the stock
`objectives/` definitions of both `reBot Check Leader Idle` and
`reBot Notify Leader Home` in its last Objective library; the YAML enable flag
alone only controls the device launch.

Collect reBot Gravity Data starts with the same `reBot Engage Arm` subtree as
Mirror, so it never moves a relaxed arm.

## Objectives and recordings

Print the cubes and place the bench as in [cube-stacking layout](docs/cube-stacking-layout.md)
before running Stack Blue on Red, Unstack Blue from Red or Stack and Unstack Cycle.

Stock Start/Stop Gravity Comp, native jaw and conference Teleoperate Objectives
are inherited, not duplicated. Office cube/expert/recorder sources remain under
`objectives/` and use the office-only `rebot_bench_behaviors` loader. The three
bounded-start/anchoring helpers come from the stock `motion_recovery_behaviors`
loader, registered only once. Return reBot Leader Home, including its optional
bridge verification and best-effort jaw close, is inherited; the office
arm-waypoint override retains the recorder leaf. The integrated
reach/lift settings are source snapshots, not a validated merge stack.

**Collect reBot Gravity Data** is the later48named-waypoint/single-approach route
from the gravity-identification source. It does not fit, activate or tune gravity.
The original24paired collector is preserved under
`docs/source_snapshots/paired_collection/`, deliberately outside the Objective
library, and in Git history. Never describe the later collector as paired data.
The [collection runbook](docs/gravity-identification.md) preserves failure/halt,
recording and freshness limitations. The offline `fit_gravity.py` reads closed
MCAPs only, rejects incompatible geometry and writes a new output directory; it
never publishes or modifies a running controller.

Trainer config `rebot_bench_jpeg` retains scene/wrist JPEG and raw joint/controller
streams. It contains no dataset namespace or recordings. Save/stop on halt is
not transactional; retain external raw evidence and use normal operator controls.
The peg trial remains **NOTSEATED**; this branch makes no insertion acceptance claim.

## Source mapping

- Current baseline: example workspace `c7422c8853e495f0e481daa639c2919be24d73f9`.
- [Integrated bench snapshot](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1139),
  `7d7761965e8082d169be39817ab3545c924e446c`: fit, application deltas and preserved paired collector.
- [Gravity collection/modes](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1138),
  `2ba9d42c65d21df0919b65b12e181736e0c57b76`: later collection route, fit tooling and tests.
- [Thermal recovery](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1137) and
  [chainable controller](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1134)
  are inherited from the vendor foundation. No duplicate patch or held README import.

Original source snapshots remain open and unchanged. This branch is independently
reviewed against the vendor foundation; neither it nor its dependencies are
represented as merged. See package tests for offline model, route, camera,
fit and Objective contracts. Runtime/UI/physical acceptance remains separate.
