# Boston reBot B601-RS

Office application source for **MoveIt Pro 10.x / ROS 2 Jazzy**, on the
`picknik_boston_rebot_b601_rs` branch. Select the matching
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
- The office FashionStar leader uses its retained unit calibration. The reader
  needs `motorbridge-smart-servo==0.0.4` from `config/leader-requirements.txt`;
  do not install a training stack or rewrite calibration as part of migration.

## Mirror reBot Leader engage

Mirror still requires fresh leader/follower feedback, the native collision scene,
a healthy controller handoff and **reBot at HOME**, using the per-joint
allowances in the stock `relax_safe_poses.yaml`. The Objective calls the driver's
`engage` Trigger before bridge preparation or a controller switch.
Start it explicitly; recovered USB, a cleared fault or completed HOME does not
re-engage it automatically. Do not move reBot by hand to match the leader.

Engage now seeds its position command from the measured follower pose, clamped
into the approved joint limits, and eases each joint toward the **live** leader
target. In `config/config.yaml`,
`rebot_leader_engage_speed_rad_s` defaults to **0.2 rad/s** for joint1–joint6;
`rebot_leader_engage_jaw_speed_rad_s` defaults to **1.0 rad/s** for joint7.
Every candidate step uses the existing per-tick native FCL path check before
publication. Keep the leader near HOME and still while ease-in catches up.
Once the published ramp command is within **0.02 rad** on every arm joint and
**0.05 rad** on the jaw of the current target, the bridge does not jump to the
leader. The ramp stops there and the bridge records the per-joint handoff
offset `offset0 = command - leader`. For a fixed **0.5 s** it publishes
`leader + (1 - a) * offset0`, clamped to the same approved joint limits as the
seed, with `a` rising linearly from 0 to 1, then switches to the existing
**direct, uncapped following**. The handoff command therefore moves with the
live leader plus at most `|offset0| / 0.5 s` (at most 0.04 rad/s on the arm and
0.10 rad/s on the jaw, within the default engage speeds), with no step at either
end.
The handoff always ends after 0.5 s, even with a moving or noisy leader.
Convergence does not wait on the measured follower, so spring-mode steady-state
error cannot hold engage open.
The engage cap is not a normal following-speed limit or a safety-rated motion
limit.

The old 0.1 rad alignment refusal is replaced by a gross arm mismatch check:
`rebot_leader_engage_max_offset_rad` defaults to **0.6 rad** per arm joint.
There is no jaw alignment gate; jaw targets still obey the native limits and
collision checks. Gross refusal retains the per-joint signed offset and
leader-only correction instructions. At the earlier 0.05/0.10 rad/s caps,
handoff-window arm speed measured on a bench unit stayed below 0.026 rad/s in
two completed engages, with no reported jump. The configured caps are now
0.2 rad/s arm and 1.0 rad/s jaw; the 0.6 rad gross bound and original unit
calibration are retained. These settings are not physical acceptance for
another installation.

The **3 s start limit** applies to starting the ramp, not completing it.
At engage, the fixed convergence deadline is the largest initial per-joint
`abs(leader - follower) / engage_speed` plus **5 s**, including the jaw at its
own speed, and it stays armed until the handoff blend completes. A moving leader
does not extend that deadline. Timeout, stale data, lost Objective heartbeat,
collision and Stop/clutch use the existing stopped command / planned HOME
recovery path. After that HOME succeeds, the bridge clears its recovery state and
sends **Relax at HOME** once, so ending Mirror finishes relaxed at HOME. That
Objective's idle, pose, stillness and health checks still apply; any refusal
or unavailable server is logged and leaves torque on at HOME. A failed HOME
never requests relax. HOME-before-disable behavior is unchanged.
If HOME fails, do not disable torque away from HOME; use the supported recovery
procedure with the operator. After its arm legs, Return reBot Leader Home waits
0.5 s and runs `reBot Notify Leader Home`. Boston's config-specific override
calls `/rebot_leader_bridge/home_verified` inside `ForceSuccess`; the stock
no-leader definition does nothing. The bridge clears its
failed-HOME latch only while inactive, with fresh arm feedback within 0.08 rad
of HOME (the values and stillness window come from the shared HOME entry), healthy enabled
motors and no thermal trip. It never commands motion or re-engages Mirror.
The Objective then attempts HOME's jaw target, currently nominal zero. Both the optional
verification call and close are best-effort: neither failure turns a successful
arm HOME into a recovery failure. Check the jaw before relying on it being
closed. **Relax at HOME** additionally requires the driver's measured all-seven
pose/stillness/health/no-motion admission before torque-off; its relax call is
not best-effort. Relax at HOME refuses a following/prepared/recovering leader
through `/rebot_leader_bridge/idle` before HOME and again before relax. Finish
or cancel active and queued trajectories/teleop first; command stillness alone
cannot establish that prerequisite. A Boston-derived config with the leader
disabled must override both `reBot Check Leader Idle` and
`reBot Notify Leader Home` with the stock definitions in its last Objective
library; the YAML enable flag alone only controls the device launch. HOME and
Relax at HOME come from the stock [generated Objective library](../rebot_b601_rs_hw/README.md#generated-objective-library),
which every Runtime start links from the build.
HOME and adding measured relax-safe poses are documented in the
[stock hardware README](../rebot_b601_rs_hw/README.md#relax-safe-poses-and-relax-at-home). Explicitly start Mirror again after successful HOME verification.
There is no auto-HOME before engage, calibration re-zero or reader respawn.

Offline coverage includes the +0.110 rad shoulder-pan rest offset, arm/jaw step
bounds and the 0.5 s handoff bound with stationary, moving, faster-than-engage
and one-count flickering leaders, live-target reversals, all-joint convergence,
gross refusal, jaw startup, timeout-to-HOME and preserved deadman/start checks.
Fake-SDK startup exercises the actual configured bridge and reader without USB
or CAN access. Offline or MuJoCo proof does not authorize or certify real-arm
motion.

## Objectives and recordings

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
