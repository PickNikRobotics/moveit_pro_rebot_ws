# reBot B601-RS workspace

MoveIt Pro **10.x / ROS 2 Jazzy** configuration for the stock Seeed reBot
B601-RS arm and supplied native jaw. The backend chain is:

| Config | Parent | Backend |
| --- | --- | --- |
| `rebot_b601_rs_mock` | — | ros2_control mock hardware |
| `rebot_b601_rs_sim` | `rebot_b601_rs_mock` | MuJoCo arm with isolated jaw-contact smoke fixture |
| `rebot_b601_rs_hw` | `rebot_b601_rs_sim` | RobStride SocketCAN hardware |

These stock configs are the product configuration, with identity `world` →
`base_link` placement, a stock Orbbec Gemini 305 wrist camera and an
[optional Star Arm leader](src/rebot_b601_rs_hw/README.md#optional-star-arm-leader)
that is off by default. `main` also carries the
[cube-stacking application](src/picknik_boston_rebot_b601_rs_hw/README.md)
(`picknik_boston_rebot_b601_rs_hw`) with uncalibrated nominal placement, camera
and gravity defaults, and printable cubes. The public
`picknik_boston_rebot_b601_rs` branch keeps the Boston office bench's
calibration, its MuJoCo bench twin and its frozen peg twin.
The standalone `rebot_bench_sim` example remains in
[moveit_pro_example_ws](https://github.com/PickNikRobotics/moveit_pro_example_ws).
Copy and rename these configurations when creating a different robot or cell.

Only 10.x is supported. Inherited `v8.3`, `v8.10` and `v9.4` branches are empty
workspace scaffolding, not supported reBot configurations. No earlier-release
reBot checkout is available.

## Install and select a backend

Install MoveIt Pro 10.x, then clone all pinned dependencies:

```bash
git clone --recurse-submodules https://github.com/PickNikRobotics/moveit_pro_rebot_ws.git
cd moveit_pro_rebot_ws
git lfs pull
git submodule update --init --recursive
moveit_pro build --user-workspace "$PWD" user_workspace
moveit_pro run --user-workspace "$PWD" --config-package rebot_b601_rs_mock --instance rebot_mock --headless
```

Select `rebot_b601_rs_sim` for physics simulation. Both simulated backends can
use named instances. The physical backend needs a deployment that exposes the
host SocketCAN network interface; see
[hardware setup](src/rebot_b601_rs_hw/README.md). Selecting the physical config
can enable torque and is a separate commissioning step.

The root config owns the shared URDF/SRDF, controllers and waypoints.
`Open Gripper` / `Close Gripper` provide the standard teleoperation overrides.
`Move to Rest` / `Move to Raised` use the core waypoint Objective with 0.10
velocity and acceleration scales. These poses are reference configurations;
verify collision clearance in the actual installation before physical use.
The JTAC teleoperation override and zero added collision padding are retained
from the compact-arm configuration; mesh collision checking remains enabled.
This is not a measured physical-clearance allowance. Hardware additionally loads
the motor-gain leaf and the small [generic recovery package](src/motion_recovery_behaviors/README.md).

## Joint and controller contract

Motor IDs 1–7 map to `joint1`–`joint7`, in motor-frame radians without an extra
sign inversion. J1–J3 use RS06 motors; J4–J7 use RS00. The primary planning
group is `manipulator` (base_link → gripper_end). `gripper` commands J7 and
includes the two passive native-jaw sliders, coupled at **0.008 m/rad**.
`manipulator_and_gripper` combines the seven motor coordinates.

### Jaw feedback allowance

J7's motor-coordinate range is **-0.15 to 6.10 rad**; the nominal Open
Gripper target remains 5.410520681182422 rad (310°), while gains and
dynamic/effort limits are unchanged. Measured on a bench unit, a closed jaw at
rest reads about **-0.0105 rad**. While Mirror follows a fast trigger close,
feedback briefly measured **-0.042 to -0.120 rad** for 40–100 ms; a fast open
reached **5.72 rad**. The -0.15/6.10 bounds leave margin around those
transients and keep the driver's one-turn wrap window unambiguous: it is
centered at 2.975 rad with edges near -0.167 and 6.117 rad. Both passive
fingers inherit the same allowance at 0.008 m/rad: **-0.0012 to 0.0488 m** each. This models contact compliance, not additional
useful jaw travel.

J3's upper bound is **4.181336912266 rad**, a 0.01-rad feedback allowance for
measured bench overshoot. All other arm-joint limits are unchanged.

`gripper_closed`, `Gripper Closed` / `Close Gripper`, mock initial state and
the simulator's default keyframe still target **zero**; `gripper_open` and
`Gripper Open` still target the nominal **5.410520681182422 rad**. Do not
command the feedback allowances to increase squeeze or opening. URDF, MoveIt
and generated MuJoCo joint/actuator limits agree; raw driver feedback is
neither clamped nor re-zeroed. The pinned driver still refuses
activation/torque enable more than one Type-2 feedback quantum beyond each
joint's own URDF bound.

Historical larger negative closed readings occurred across calibration and
power-cycle changes; later explicit jaw re-zeroing returned contact readings
to near zero. A reading below **-0.15 rad** or above **6.10 rad** remains out
of bounds: verify that installation's zero and mechanics rather than widening
this stock allowance. This software change does not calibrate or certify a
real arm; physical verification requires a separately authorized commissioning
check.

The trajectory controller owns all seven position interfaces and accepts partial
arm/jaw goals. Arm and jaw velocity controllers and the Cartesian controller
start inactive and are switched by MoveIt Pro. The 100 Hz update rate and arm
spring gains are retained. This configured rate is not a measured maximum bus
bandwidth.

The physical configuration starts in spring/MIT mode (`arm_control_mode: motion`)
with the reusable gravity controller active beside the trajectory controller.
Gravity owns the six arm effort interfaces and uses the nominal stock URDF,
explicit unit gains and zero offsets. The two native fingers are represented
once by a 0.1504 kg payload at their nominal combined center of mass. This is
not the fitted Boston model and not a physical balance certification. Mock and
MuJoCo do not spawn this controller; MuJoCo supplies its own gravity compensation.

The physical launch also includes the thermal guard. At the source 100°C latch,
it anchors measured pose, restores configured spring gains, then follows the
preserved cancel → yaw-first HOME → fresh HOME verification → OFF → cooling
sequence (75°C clear). Stock HOME omits only office trajectory recording.
This wiring and its offline tests do not certify real-arm recovery; see the
[recovery limits](src/rebot_b601_rs_hw/docs/source-provenance.md).

The physical launch also includes a Gemini 305 wrist-camera driver. It starts
by default when the camera is attached and can be disabled; see the
[hardware camera setup](src/rebot_b601_rs_hw/README.md#gemini-305-wrist-camera)
for its 1280×800 MJPG/10 Hz contract, host udev rule and nominal-mount limits.

## Relax at HOME

The physical configs provide **Relax at HOME**: complete the shared HOME return,
verify the leader is idle, then request guarded torque-off without stopping the
driver. Return Home, Mirror prepare and physical teleoperation call engage
first. HOME targets, per-joint tolerances and stillness live only in the
[stock relax-safe pose configuration](src/rebot_b601_rs_hw/config/relax_safe_poses.yaml);
waypoints and HOME XML are derived at build time. Each additional pose needs its
own measured torque-off settle test before admission. See the
[service contract and pose runbook](src/rebot_b601_rs_hw/README.md#relax-safe-poses-and-relax-at-home).
Do not call relax during an active or queued motion/teleop session. Mock/twin
proof with fake services is not physical commissioning or torque-off evidence.

## Minimal simulation

The simulator generates its body tree from the same expanded stock URDF.
It has nine generalized coordinates (seven motors and two dependent jaws),
seven named position actuators and `default` / `contact_smoke` reset keyframes.
Stock visuals use `package://rebot_description/...` assets.

Normal operation remains gravity-compensated free-space control. Only the two
native PLA pads and a fixed 50 mm box collide; the box is parked at default and
placed between the raised open jaws by `contact_smoke`. The contact smoke checks
measured motor/jaw state and controller rejection of a blocked close, not grasp
fidelity. It contains no bench, cameras or grasp proxy and does not validate
self-contact, lifting, payload response or insertion. See the
[simulation runbook](src/rebot_b601_rs_sim/README.md) for reset, timing and the
reproducible headless MoveIt Pro smoke run. Hardware uses the physical plugin
and wall time; its resolved config excludes simulation reset and sensors.

## Immutable source dependencies

All dependencies are complete HTTPS submodules under `src/external_dependencies`.
The gitlink, rather than a floating branch name, fixes each revision.

| Repository | Commit | Purpose |
| --- | --- | --- |
| [PickNikRobotics/robstride_hardware_interface](https://github.com/PickNikRobotics/robstride_hardware_interface) | `f72b2d2b52bed069a68254a16b5ece2351bc3b47` | Safe relax, measured-pose engage, soft limits and thermal handoff |
| [PickNikRobotics/RobstrideSDK](https://github.com/PickNikRobotics/RobstrideSDK) | `9d070193758b4d88d065f1b8add815c0f540cebc` | Fault-frame routing, decoding and temperature freshness |
| [kmj060703/robstride_interfaces](https://github.com/kmj060703/robstride_interfaces) | `8278bb1bf3d8f1fb07803459e1de4e14e10b8fc0` | Driver messages/services |
| [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) | `58f9e433deec2ba118c5c22d60c07b9897e92acc` | Stock description and editable mechanical sources |

The driver pin is `main` with the merged
[safe-relax change](https://github.com/PickNikRobotics/robstride_hardware_interface/pull/4)
and [joint-coordinate/soft-limit change](https://github.com/PickNikRobotics/robstride_hardware_interface/pull/3).
The SDK revision above is an integration topic commit, not a merged fork
default. Publication of topic revisions is a prerequisite for publishing these gitlinks.

The upstream description has no ROS package. The small `rebot_description`
ament adapter installs its unmodified RS mesh directory and exact license
from the whole submodule, plus the local nominal Gemini 305 frame-only Xacro.
It maintains no selectively copied upstream tree or per-unit camera calibration.
The modified URDF's [source and change notice](src/rebot_b601_rs_mock/description/NOTICE.md)
and upstream licenses remain with the installed resources.
The [generic gravity controller](src/kdl_gravity_compensation_controller/README.md)
retains the published standalone/chainable implementation from
[its source change](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1134).
The older driver gravity branch must not replace the newer SDK fault/freshness
fixes. Measured office inertial fits, gains and scenes do not belong in this
stock model.

## Validation scope

Offline validation uses MoveIt Pro 10.2.0-rc10/Jazzy, ros2_control 4.45.2 and
MuJoCo 3.6.0. Tests exercise the real configuration loader, all three expanded
backends, MoveIt Pro's model/FCL consumers, stock asset resolution, nominal
inertias, physical spring/effort declarations, the Gemini 305 launch
contract, finite simulation and native-jaw coupling. The six-joint KDL model
plus finger payload is compared with whole-tree inverse dynamics across 50 arm
poses and three jaw positions.

The reusable controller's own model, loaned-interface and real ControllerManager
fake-hardware tests are retained. SDK fault/freshness decoding has eight offline
tests. Driver virtual-CAN lifecycle tests skip without an isolated virtual
interface; a skip is not torque-lifecycle evidence. Live mock teleoperation is
covered by the
[smoke sequence](src/rebot_b601_rs_mock/README.md#teleoperation-on-the-mock).
Simulation has a separate recorded headless Runtime smoke covering
plan/execute, Joint Jog, Pose Jog, open/close, blocked-jaw contact and reset
([simulation README](src/rebot_b601_rs_sim/README.md)). No physical-arm,
Desktop/IMarker or end-to-end deployment acceptance is claimed.

The workspace retains the empty-workspace Docker targets, `base` / `runtime` /
`drivers` services, CI, LFS and formatting setup. See
[MoveIt Pro documentation](https://docs.picknik.ai/) for installation and
runtime configuration.
