# reBot B601-RS cube-stacking application

Application source for **MoveIt Pro 10.x / ROS 2 Jazzy**, first built on the
PickNik Boston bench. Select the `picknik_boston_rebot_b601_rs_hw` config. It
inherits `rebot_b601_rs_mock → rebot_b601_rs_sim → rebot_b601_rs_hw` and adds
printed trim, a fixed OAK-D scene camera, the Gemini 305 wrist camera, the
optional Star Arm leader, gravity collection/fit tools and the cube-stacking
Objectives.

On `main`, every per-unit value is an **uncalibrated nominal default**: base and
scene-camera placement of the reference layout, the stock wrist-camera mount,
the nominal gravity model and an unscaled scene depth. Calibrate them for your
installation and override them in a child config; see
[calibrating your installation](docs/calibration.md). The `picknik_boston_rebot_b601_rs` branch keeps the
calibrated values of the office bench, its MuJoCo bench twin and its peg scene.

**Not physical readiness.** The driver launch includes the stock physical-only
thermal guard and native HOME; recovery keeps its trajectory recorder. This is
software dependency closure, not commissioning. No sag, float-quality,
thermal-recovery or no-snap certification is claimed.

## Model and tuning boundary

The shared arm model owns joints, axes, stock collision meshes, native jaw,
controller interfaces, limits and spring tuning. The wrapper adds the exact
printed trim, the wrist-camera frame and the base/scene-camera transforms; stock
configurations retain nominal geometry and zero placement. See
[description provenance](description/NOTICE.md).

`boston_gravity_fit` is `false` here, so gravity compensation uses the nominal
URDF inertials with the stock gains and the nominal jaw as payload. Setting it
`true` selects six **static gravity-equivalent inertial origins** from
`config/gravity/identified_first_moments.yaml`, which ships with the nominal
values. Replace them with your own fit (`scripts/fit_gravity.py`, see
[gravity calibration](docs/gravity-calibration.md)) and set the gains and
payload chosen with that fit: a fit that absorbs the jaw, camera and cables uses
zero additional payload, or it double counts them. Fitted origins are not
measured physical CoMs or valid dynamic inertias; the wrapper refuses them with
mock or MuJoCo.

Reusable stock tuning remains the integrated settings: J2/J3 lower allowances
−0.02/−0.01rad, shared path tolerance0.20rad, J2 feedforward cap18Nm,
kp `[50,150,150,50,50,50]`, kd `[3,10,10,5,4,4]`, jawkp12/kd0.5 and100Hz.

## Sensors and assembly

- Fixed OAK-D:640×400,30Hz, full sensor ISP1/2 scaling, aligned depth and
  compressed color topic. `scene_depth_scale` is1.0 (unscaled); set a factor
  measured for your unit in `config/cameras.yaml`.
- Wrist Gemini 305:1280×800,10Hz MJPG; no depth/IR launch and no duplicate
  driver TF. The first attached unit is used, with the stock nominal
  `gripper_end` to color-optical-frame mount and factory `camera_info`. Set
  `wrist_camera_color_optical_xyz/_rpy` from a hand-eye calibration, and pin
  the calibrated unit's serial in `launch/rebot_drivers.launch.py`.
- The scene camera and base placements are nominal values of the reference
  layout in the mat (world) frame; calibrate both before relying on perception.
  Camera/mount collision and dynamic inertias are incomplete. Verify clearance
  before hardware use.
- Optional scene-camera absence warns and skips only that camera. Its launch's
  USB descriptor read is not run during offline tests; tests supply fixture descriptors.
- The Star Arm leader enables the stock
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
This config lists `rebot_b601_rs_hw/leader_objectives` after the inherited
stock library, so those subtrees replace the no-leader ones, and checks leader
targets against this config's own SRDF (printed trim and camera mounts). HOME
and Relax at HOME come from the stock
[generated Objective library](../rebot_b601_rs_hw/README.md#generated-objective-library).
A config derived from this one with the leader disabled must put the stock
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
