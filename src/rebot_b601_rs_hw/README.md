# reBot B601-RS physical backend

This 10.x/Jazzy config inherits the same stock arm and native jaw through the
simulation config, then selects `robstride_hardware_interface/RobstrideHardware`
and wall time. It has a stock Gemini 305 wrist-camera launch, no simulated
sensor and no simulation-only controller. Hardware defaults to motion (MIT
spring) mode with normal configured kp/kd. JTAC owns all seven position
interfaces; the active standalone gravity controller owns the six arm effort
interfaces. The jaw has no gravity effort command interface.

## Integration status

The stock model/controller composition is a software migration, not a physical
acceptance. The physical backend launches the preserved thermal guard with its
native HOME dependency, using the three shared recovery helpers. The latch is
100°C with 75°C clear, matching the source configuration. Mock/MuJoCo do not
launch the guard. Commissioning still requires separately authorized physical
recovery checks; offline closure is not a hardware safety certification.
[Source provenance and limitations](docs/source-provenance.md) distinguish
migrated functionality from unverified runtime behavior.

## Thermal recovery

The RobStride driver latches a thermal trip when any motor reaches 100°C or
higher. It refuses new activation and torque-enable requests while latched;
cooling alone does not clear it. Once all motors report below 75°C, a fresh
stationary-HOME recovery attestation with all motors OFF is required to clear
the latch. A trip received while every motor is already OFF remains OFF; the
guard does not request HOME or enable torque in that case.

For a new latch with fresh healthy feedback from all enabled motors, the guard
first sends JTAC a nonzero-stamped stationary goal with two identical measured
seven-joint poses. After two advancing controller-reference samples match that
pose, it restores the configured `kd` for arm motor IDs 1–6, then their `kp`,
one acknowledged request at a time through
`/robstride_hardware_interface/set_data_to_robstride`. The values come from
the initial `/robot_description` GPIO configuration; the jaw gains and gravity
compensation are unchanged. JTAC stays active, the stationary goal remains
observable for up to 10 s while gains are restored, and native HOME supersedes
it.

Only after that restore does the guard cancel the active Objective and request
**Return reBot Leader Home**. It waits for success and fresh stationary HOME
for 0.5 s, then asks the owning driver to turn torque OFF. A failed anchor,
reference check, gain request, cancellation, HOME, or HOME verification leaves
the latch set and never disables away from HOME. Gain requests are bounded to
0.5 s and not retried or rolled back; their acknowledgement means the driver
queued a write, not that the motor read back an atomic gain update. This
requires no competing motion or gain writer and is not physical no-snap proof.

## Before starting hardware

The arm has no brakes. Activation probes positions using Stop frames, which
can release torque left by an earlier deployment. Support the arm at its
verified home pose before activation or a restart. Bring up a new installation
with `torque_enable: "false"` in this config's `hardware.robot_description.urdf_params`
to inspect readable, uncommanded state. Confirm zero, signs, addressing and
limits before enabling torque. A software stop is not power isolation.

Set up the host SocketCAN adapter at **1 Mbps**, `txqueuelen 1000`, before
starting the physical backend. The driver's installed `setup_can.sh` documents
the host setup. This workspace does not run that setup automatically.
`can_interface` defaults to `can0`; override it explicitly for a different bus.
IDs 1–7 and host/master ID 253 must match the actual motors.

SocketCAN interfaces belong to network namespaces. A normal named simulation
instance does not expose the host CAN interface, and mounting `/dev` does not
expose it. Use the release's documented host-network physical deployment;
for the standard 10.x CLI this is the deployment without `--instance`:

```bash
moveit_pro run --user-workspace "$PWD" --config-package rebot_b601_rs_hw --headless
```

This command is for a commissioned physical installation. It was not executed
as part of the simulation/repository migration.

## Gemini 305 wrist camera

The physical driver's `rebot_drivers.launch.py` includes the Gemini 305 launch,
and `config/wrist_camera.yaml` sets `enable_wrist_camera: true` by default. The
launch detects USB device `2bc5:0840` before starting
`orbbec_camera/orbbec_camera_node`; when no camera is attached it logs a warning
and skips the driver, leaving the arm backend available. To turn the camera off
entirely (no USB probe, no driver), set `enable_wrist_camera: false` in
`config/wrist_camera.yaml`, rebuild the package, then restart the physical
backend. Mock and MuJoCo never include this launch. Leave
`wrist_camera_serial_number` empty to let the driver select its first device,
or set it only for a host with multiple cameras. The scan runs once during
startup; attach or reconnect a camera, then restart the physical backend to
start its driver.

The package declares `orbbec_camera`; the Jazzy rosdep key resolves to
`ros-jazzy-orbbec-camera`. If the selected physical deployment image lacks the
driver, startup logs a warning and leaves the arm backend available; install the
driver image, then restart before camera use.

The launch publishes color only at **1280×800 MJPG, 10 Hz** under
`/wrist_mounted_camera`. It leaves `color_info_url` unset so the driver derives
`camera_info` from the active device's factory stream calibration, disables
depth/IR/point clouds, and does not publish driver TF. `ae_mode: colorbased`
meters auto-exposure from the color stream rather than the disabled depth
stream. No exposure value, gain or AE region of interest is overridden.
The robot description is the sole source of
`wrist_mounted_camera_color_optical_frame`.

Install Orbbec's host USB rule before connecting the camera; the container's
`/dev` mount does not replace host udev configuration:

```bash
sudo install -m 644 \
  <OrbbecSDK_ROS2>/orbbec_camera/scripts/99-obsensor-libusb.rules \
  /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The stock `gripper_end` →
`wrist_mounted_camera_color_optical_frame` transform is a nominal CAD/geometry
estimate, not a calibrated hand-eye result. It has no serial-specific value.
These source settings are not physical-camera acceptance; verify the selected
driver image and stream on a commissioned host before using the camera. A
downstream office package must override
`wrist_camera_color_optical_xyz` and `wrist_camera_color_optical_rpy` with its
own calibrated mount before using the frame for measured perception or motion.
`urdf_params` merge by parameter key, so the child config needs only its two
calibrated entries:

```yaml
hardware:
  robot_description:
    urdf_params:
    - wrist_camera_color_optical_xyz: "x y z"
    - wrist_camera_color_optical_rpy: "roll pitch yaw"
```

Build the child config and restart the backend after changing that source. The
stock camera frame has no body, collision or inertial geometry. The mock and
MuJoCo descriptions gain no camera frame.

## Preserved torque and fault behavior

| Event | Configured behavior |
| --- | --- |
| Clean deactivation | Keep reachable motors holding their measured targets |
| Process exits | Closing the bus does not send Disable |
| Joint loss or fault | Latch freeze, hold reachable joints, ignore further controller commands |

`hold_torque_on_deactivate: true` is paired with `motor_can_timeout_ms: 0`.
An enabled motor watchdog could otherwise release torque after the bus closes.
`freeze_on_joint_loss: true` preserves the latched joint-loss behavior.
A fault reset alone does not rejoin the lost motor or resume motion.
The driver requires fresh measured positions before activation.

Return reBot Leader Home calls the driver's `engage` Trigger service before
any controller switch or motion, then retains torque and the normal trajectory
controller. It holds the jaw during the arm legs, waits 0.5 s at HOME,
runs the config-specific `reBot Notify Leader Home` subtree, then attempts the
configured HOME jaw target (nominal zero). The stock notification is a no-op;
Boston's override retains the optional `/rebot_leader_bridge/home_verified`
call wrapped in `ForceSuccess`.
The notification and close are best-effort, so a successful HOME result
alone does not prove that the jaw closed. The bridge clears a failed-HOME
latch only with fresh, stationary arm feedback at HOME and healthy enabled
motors, while Mirror is inactive and no thermal trip is latched.

Return to a supported home pose under power before normal shutdown. The arm
can remain rigid after the process stops until motor power is cut. A restart
with torque disabled is itself a commissioning operation, with the arm
supported. Validate the manufacturer's independent stop/power-isolation path
before physical motion. The model, reference waypoints and software holding
policy do not establish that an installation is safe to operate.

Keep per-unit measurements and fitted gravity gains in the office configuration.
Reusable tuning from the integrated source is shared by the stock configs:
J2/J3 lower allowances -0.02/-0.01 rad, J3 upper allowance 4.181336912266 rad
and J7 feedback allowance -0.15 to 6.10 rad
([evidence and safe nominal targets](../../README.md#jaw-feedback-allowance)),
shared arm/jaw path tolerance 0.20 rad,
and J2 `max_feedforward_effort` 18 Nm. Other feedforward caps retain their driver
defaults. The arm kp/kd values, jaw kp=12/kd=0.5 and 100 Hz update rate are retained.
These defaults are not calibration or physical acceptance for a new installation.
Motor status and fault telemetry remain available through the pinned driver's
`~/robstride_state` topic and services.

## Relax-safe poses and Relax at HOME

The arm has no brakes. **Relax at HOME** returns HOME under power, waits one
configured stillness window plus 0.1 s after the jaw move, then calls
`/robstride_hardware_interface/relax` (`std_srvs/Trigger`). This is a
request, not an idle timer. A failed HOME prevents the
relax call. A rejected or unavailable relax service fails the Objective; it
never falls back to unguarded `set_torque false`. The only automatic request is
the [optional leader](#optional-star-arm-leader) bridge's: after Mirror ends and
its HOME recovery succeeds, it runs Relax at HOME once, with the same checks.

The matching driver must admit torque-off only when **all seven measured
joints**, including the jaw, match a named relax-safe pose within its own
per-joint `tolerance_rad`, satisfy the configured position-span stillness test,
have fresh fault-free communications, and satisfy the driver's measured and
commanded stillness checks. **The caller must finish/cancel all trajectory,
teleop and queued motion before relax.** Command stillness is not proof that
no future goal is queued; the driver does not inspect Objective/action status.
Success disables all seven motors and names the matched pose. It leaves the driver and telemetry running. This service is not a
safety-rated protective stop or power isolation.

`/robstride_hardware_interface/engage` is also `std_srvs/Trigger`: enable torque
only after the driver's measured-bounds and health checks, re-seed the hold at
the measured pose without a step, and report elapsed milliseconds and maximum
step. Return Home, Mirror's prepare entry, and the physical Teleoperate
Objective wait for engage success before motion. Stock mock/MuJoCo teleop
remains independent of physical services. The Boston twin inherits the physical
entries; validation supplies fake Trigger servers, not a second motor owner.
Relax at HOME checks `reBot Check Leader Idle` before HOME and immediately
before relax. The `objectives/` definition succeeds because a config without a
leader has no other command producer. The `leader_objectives/` definition, listed
by leader-enabled configs, calls `/rebot_leader_bridge/idle` and refuses active
following, prepared following, or pending HOME recovery without changing bridge
state; an absent bridge fails closed. Setting `rebot_leader_enabled` alone
controls only the device launch; the Objective library selects which leaf runs
(see [Optional Star Arm leader](#optional-star-arm-leader)). A config derived
from a leader-enabled one that drops the leader must therefore also put the
`objectives/` definitions of **both** `reBot Check Leader Idle` and
`reBot Notify Leader Home` in its last Objective library.
Neither no-leader leaf grants permission to relax or replaces powered HOME.
Other custom motion entry points must engage explicitly after relaxation;
relaxation does not authorize them to issue motion directly.

The one source is [`config/relax_safe_poses.yaml`](config/relax_safe_poses.yaml):
`version: 1` and `relax_safe_poses` match the pinned driver schema. The latter
is a list of `{name, joints, stillness}` maps with HOME first. `joints` contains
exactly `joint1`–`joint7`, each `{position_rad, tolerance_rad}` in radians;
`stillness` contains positive `window_s` (at most 10 s), `max_span_rad`, and the
optional positive `max_abs_velocity_rad_s` (default 0.01 rad/s).
HOME currently targets nominal zero on all seven, with 0.08 rad per-joint
allowance and a 0.3 s / 0.005 rad position-span window, plus 0.01 rad/s
absolute-velocity bound. Those tolerances are
admission bounds, not permission to command the feedback allowances.

The build derives both stock and Boston `Zeroed Rest` waypoints, SRDF
`zeroed_rest` named states and HOME XML
(targets, per-joint skip checks, jaw target and post-jaw wait) from that entry.
The leader reset/verification, HOME-only tuning and thermal pose gate read the
same installed file. No new Behavior plugin is needed: the service calls use
MoveIt Pro 10.2 rc10's stock `CallTriggerService` ports `service_name`,
`wait_for_server_available_timeout` and `response_timeout`; the existing
bounded-start/anchoring recovery helpers remain unchanged.

The ros2_control Xacro passes the absolute installed path as hardware parameter
`relax_safe_poses_file`. The optional Xacro argument of the same name is for
relocating **the same configuration snapshot**, not silently changing HOME
only in the driver. A changed HOME must be rebuilt into all consumers. Even
with symlink-install, the admission file and generated resources are installed
build snapshots; rebuild stock mock, stock hardware and Boston hardware
(together with any affected twin) after editing it, then restart the selected
mock/twin Runtime. MoveIt Pro preserves an existing user `waypoints.yaml`:
review and update its `Zeroed Rest` entry to the rebuilt value, or archive the
user waypoint file and let the next start populate the rebuilt defaults. Do
not redefine HOME in the waypoint editor or edit generated XML independently.

### Generated Objective library

`return_rebot_leader_home.xml` and `relax_at_home.xml` are built from their
`.xml.in` templates and installed only into the read-only Objective library
`rebot_b601_rs_hw_generated_objectives` (`share/rebot_b601_rs_hw_generated_objectives/objectives`),
which this config lists in `objective_library_paths`. That library has no
source package, so each Runtime start links the built XML into
`~/.config/moveit_pro/$MOVEIT_CONFIG_PACKAGE/objectives/`. Libraries of
workspace packages are read from source, where only the templates exist, so
the built files cannot live there. Rebuild and restart after changing a
template or the relax-safe poses file; no manual materialization step is
needed. Do not edit, clone over or save the generated XML in the UI.

### Add a relax-safe pose

1. Before adding an entry, obtain separate authorization for a **measured
   torque-off settle test of that exact pose** on the supported installation.
   Plan/collision-check it in the twin first. A waypoint, CAD geometry, or
   simulated success alone is not evidence that an unbraked arm supports itself.
2. Record all seven motor-frame joint values, load/tool/cable conditions,
   measured movement throughout torque-off and settling, faults/communications,
   and supported recovery/engage results. Keep personnel clear and verify the
   independent stop/support procedure. Do not reuse HOME's settle result for
   another pose. Reject a pose that drops, drifts, or cannot re-engage safely.
3. Add a unique name **after HOME**, all seven measured `joints` entries with
   `position_rad` and justified positive `tolerance_rad`, and positive
   `stillness.window_s` and `stillness.max_span_rad` (plus a justified velocity
   bound if different from 0.01 rad/s). Keep its measured evidence with the
   installation's commissioning records. Do not widen limits or weaken checks to admit it.
4. Rebuild the affected packages, run the strict config/generation tests and
   mock/twin Objectives, then use the separately approved commissioning
   procedure before physical use. Do not add an untested pose as a placeholder.

HOME's folded torque-off settling has existing bench evidence; it is not
certification of another installation, payload, zero or added pose. This
change's validation is hardware-free: fake services plus the MuJoCo twin.
No physical torque-off/engage, CAN/USB access, or real-arm run is authorized.
Thermal recovery uses the internal, non-runnable **Return reBot Leader Home
Under Power** tree after its existing all-enabled health preflight; it must
not call engage while the thermal latch is set. It retains the original
cancel → HOME → verified OFF → cooling path.

## Optional Star Arm leader

Guides: [Star Arm leader-follower demo](docs/leader-follower-demo.md) and
[behavior reference](docs/rebot-behaviors.md).

**Mirror reBot Leader** lets a Seeed Star Arm 102 (FashionStar servos on one USB
serial port) drive the arm and jaw through `leader_position_controller`. It is
off by default: no stock config starts the bridge, lists the leader Objectives,
loads the leader Behavior or spawns the controller, so a robot without a leader
runs exactly as before.

| Piece | Location |
| --- | --- |
| USB reader, bridge, state machine, native FCL path check | `scripts/rebot_leader*.py` |
| YAML-selected bridge launch | `launch/rebot_leader.launch.py` (included by `launch/rebot_drivers.launch.py`) |
| Mirror reBot Leader and the fail-closed idle/HOME-notify subtrees | `leader_objectives/` |
| `CallRebotLeaderService` Behavior | `rebot_b601_rs_hw::RebotLeaderBehaviorsLoader` |
| `leader_position_controller` (all seven joints, declared, not spawned) | `rebot_b601_rs_mock` control YAML |
| Seeed stored-origin calibration (all homing offsets zero) | `config/leader_calibration/rebot_arm_102_leader.json` |
| FashionStar SDK pin, installed by the workspace `Dockerfile` | `config/leader-requirements.txt` |
| Volatile jaw torque / J4 speed settings, written only by `rebot_leader_tune.py --apply` at HOME with all motors OFF | `config/rebot_leader_tuning.yaml` |

To enable it, add this to a child config of `rebot_b601_rs_hw` (or of
`rebot_b601_rs_sim` / `rebot_b601_rs_mock`), then rebuild and restart:

```yaml
ros_global_params:
  rebot_leader_enabled: true
  # Optional: rebot_leader_port (default /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0)
  # and rebot_leader_calibration_file (absolute, or relative to this package's share).
ros2_control:
  controllers_inactive_at_startup:  # Replaces the inherited list; keep its entries.
    - leader_position_controller
    - joint_velocity_controller
    - gripper_joint_velocity_controller
    - velocity_force_controller
objectives:
  behavior_loader_plugins:
    rebot_leader:
      - rebot_b601_rs_hw::RebotLeaderBehaviorsLoader
  objective_library_paths:
    # After the inherited stock library, so these replace its no-leader subtrees.
    rebot_leader_objectives:
      package_name: rebot_b601_rs_hw
      relative_path: leader_objectives
```

The physical driver launch already includes the leader launch. A mock or MuJoCo
child has no driver launch, so it also sets `hardware.additional_driver_launch_file`
to `rebot_b601_rs_hw` / `launch/rebot_leader.launch.py`. Mirror calls the
driver's `engage` Trigger first, so it runs only on the physical backend.

Other `ros_global_params`, with their defaults: `rebot_leader_publish_rate`
(100 Hz, 20–100), `rebot_leader_engage_speed_rad_s` (0.2, joint1–joint6),
`rebot_leader_engage_jaw_speed_rad_s` (1.0, joint7) and
`rebot_leader_engage_max_offset_rad` (0.6, arm joints only). Any value other than
YAML `true`/`false` for `rebot_leader_enabled` fails the launch. The bridge
checks targets against the selected config's own URDF and SRDF. The stock SRDF
keeps link2/link5 checked: with the arm folded at HOME and J3 or J4 reading
slightly below zero (for example J3 -0.01 rad), the stock model reports that
contact and the bridge refuses to engage or follow. The Boston SRDF excludes
the pair. Excluding it in a child SRDF also changes planning collision checks;
verify the clearance on the installation first.

### Mirror reBot Leader engage

Mirror still requires fresh leader/follower feedback, the native collision scene,
a healthy controller handoff and **reBot at HOME**, using the per-joint
allowances in `config/relax_safe_poses.yaml`. The Objective calls the driver's
`engage` Trigger before bridge preparation or a controller switch.
Start it explicitly; recovered USB, a cleared fault or completed HOME does not
re-engage it automatically. Do not move reBot by hand to match the leader.

Engage seeds its position command from the measured follower pose, clamped
into the approved joint limits, and eases each joint toward the **live** leader
target at the engage speeds above. Every candidate step uses the existing
per-tick native FCL path check before publication. Keep the leader near HOME
and still while ease-in catches up. Once the published ramp command is within
**0.02 rad** on every arm joint and **0.05 rad** on the jaw of the current
target, the bridge does not jump to the leader. The ramp stops there and the
bridge records the per-joint handoff offset `offset0 = command - leader`. For a
fixed **0.5 s** it publishes `leader + (1 - a) * offset0`, clamped to the same
approved joint limits as the seed, with `a` rising linearly from 0 to 1, then
switches to **direct, uncapped following**. The handoff command therefore moves
with the live leader plus at most `|offset0| / 0.5 s` (at most 0.04 rad/s on the
arm and 0.10 rad/s on the jaw), with no step at either end. The handoff always
ends after 0.5 s, even with a moving or noisy leader. Convergence does not wait
on the measured follower, so spring-mode steady-state error cannot hold engage
open. The engage cap is not a normal following-speed limit or a safety-rated
motion limit.

A gross arm mismatch beyond `rebot_leader_engage_max_offset_rad` refuses with
the per-joint signed offset and leader-only correction instructions. There is
no jaw alignment gate; jaw targets still obey the native limits and collision
checks. At earlier 0.05/0.10 rad/s caps, handoff-window arm speed measured on a
bench unit stayed below 0.026 rad/s in two completed engages, with no reported
jump; the 0.2/1.0 rad/s defaults are the caps used on that unit since. These
settings are not physical acceptance for another installation.

The **3 s start limit** applies to starting the ramp, not completing it.
At engage, the fixed convergence deadline is the largest initial per-joint
`abs(leader - follower) / engage_speed` plus **5 s**, including the jaw at its
own speed, and it stays armed until the handoff blend completes. A moving leader
does not extend that deadline. Timeout, stale data, lost Objective heartbeat,
collision and Stop/clutch use the stopped command / planned HOME recovery path.
After that HOME succeeds, the bridge clears its recovery state and sends
**Relax at HOME** once, so ending Mirror finishes relaxed at HOME. That
Objective's idle, pose, stillness and health checks still apply; any refusal or
unavailable server is logged and leaves torque on at HOME. A failed HOME never
requests relax. If HOME fails, do not disable torque away from HOME; use the
supported recovery procedure with the operator. After its arm legs, Return reBot
Leader Home waits 0.5 s and runs `reBot Notify Leader Home`; the leader version
calls `/rebot_leader_bridge/home_verified` inside `ForceSuccess`. The bridge
clears its failed-HOME latch only while inactive, with fresh arm feedback within
0.08 rad of HOME (values and stillness window from the shared HOME entry),
healthy enabled motors and no thermal trip. It never commands motion or
re-engages Mirror. Explicitly start Mirror again after successful HOME
verification. There is no auto-HOME before engage, calibration re-zero or
reader respawn.

Offline coverage includes the +0.110 rad shoulder-pan rest offset, arm/jaw step
bounds and the 0.5 s handoff bound with stationary, moving, faster-than-engage
and one-count flickering leaders, live-target reversals, all-joint convergence,
gross refusal, jaw startup, timeout-to-HOME and preserved deadman/start checks.
Fake-SDK startup exercises the installed bridge and reader without USB or CAN
access. Offline or MuJoCo proof does not authorize or certify real-arm motion.

## Stock gravity model

The controller uses the nominal `base_link` → `gripper_end` chain, gains
`[1,1,1,1,1,1]` and zero offsets. Its existing payload interface adds just the two
supplied finger branches: mass **0.1504 kg**, nominal closed-jaw center
`[-0.06064800135196754, 0, -0.00000006872199924541491]` m in `gripper_end`.
The chain already contains the gripper body; adding that body again would double
count it. The actual URDF retains both moving finger links for planning/physics.

Symmetric jaw travel leaves the combined nominal center effectively fixed. The
URDF's rounded rotations introduce less than 0.16 micrometers of drift over the
full stroke, bounded by 2.4e-7 Nm in the offline whole-tree comparison. This
algebraic agreement does not establish the accuracy of nominal physical masses,
motor-reported torque, sag or float quality. The fitted Boston model is not used,
and no fitted first moments are substituted into simulation inertias.

The generic controller does not supply position holding, damping, a stop policy
or a new torque limiter. Hardware feedforward remains subject to the driver's
existing configured cap and the SDK wire range. Do not infer unbounded delivered
effort from the controller's commanded-effort topic, or reset to the old driver
fault-byte-swap branch.

## Start and Stop Gravity Comp

[Start Gravity Comp and Stop Gravity Comp](docs/gravity-modes.md) preserve the
published measured-pose anchor before damping/stiffness changes. Both finish
without switching controllers; gravity stays active. The independent thermal
recovery sequence remains required during physical use. Source integration and
offline tests do not certify float quality or motor-side gain delivery.

## Verification and rollback

[Virtual CAN qualification](docs/vcan-qualification.md) runs the driver and this
config's controllers against seven fake motors on `vcan`, and lists the bench
timing and watchdog settings.

No physical activation, CAN access or arm motion was performed here. Before
physical use, verify timestamps, all seven measured positions, mode and fault
telemetry, controller claims, actual stop behavior and reduced-scale motion
with an operator present. Revert the workspace commit and recursively update
submodules to restore a prior deployment; never substitute an older SDK pin
without rechecking fault decoding and driver compatibility.
