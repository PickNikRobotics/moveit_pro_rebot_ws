# reBot B601-RS workspace

MoveIt Pro **10.x / ROS 2 Jazzy** configuration for the stock Seeed reBot
B601-RS arm and supplied native jaw. The backend chain is:

| Config | Parent | Backend |
| --- | --- | --- |
| `rebot_b601_rs_mock` | — | ros2_control mock hardware |
| `rebot_b601_rs_sim` | `rebot_b601_rs_mock` | Minimal MuJoCo free-space arm |
| `rebot_b601_rs_hw` | `rebot_b601_rs_sim` | RobStride SocketCAN hardware |

`main` contains the product configuration, with identity `world` → `base_link`
placement. The separate public `picknik_boston_rebot_b601_rs` branch is reserved for
the complete office application and its cameras, fixtures and calibration.
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

## Minimal simulation

The simulator generates its body tree from the same expanded stock URDF.
It has nine generalized coordinates (seven motors and two dependent jaws),
seven named position actuators and a matching `default` reset keyframe.
Stock visuals use `package://rebot_description/...` assets.

This is a gravity-compensated **free-space control model**. Visual geoms have
contact disabled; it contains no bench, cameras, grasp proxy or contact task.
It does not validate self-contact, grasping, payload response or insertion.
Application simulations own their contact geometry and validation. Hardware
uses the physical plugin and wall time; it starts no simulated sensors or
simulation-specific controllers.

## Immutable source dependencies

All dependencies are complete HTTPS submodules under `src/external_dependencies`.
The gitlink, rather than a floating branch name, fixes each revision.

| Repository | Commit | Purpose |
| --- | --- | --- |
| [PickNikRobotics/robstride_hardware_interface](https://github.com/PickNikRobotics/robstride_hardware_interface) | `a0533258a10783dca4293b5d195adf7728c8ff5f` | Torque retention, thermal handoff and bounded feedforward |
| [PickNikRobotics/RobstrideSDK](https://github.com/PickNikRobotics/RobstrideSDK) | `9d070193758b4d88d065f1b8add815c0f540cebc` | Fault-frame routing, decoding and temperature freshness |
| [kmj060703/robstride_interfaces](https://github.com/kmj060703/robstride_interfaces) | `8278bb1bf3d8f1fb07803459e1de4e14e10b8fc0` | Driver messages/services |
| [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) | `58f9e433deec2ba118c5c22d60c07b9897e92acc` | Stock description and editable mechanical sources |

The driver and SDK revisions above are integration topic commits, not merged
fork defaults; their publication is a prerequisite for publishing these gitlinks.

The upstream description has no ROS package. The small `rebot_description`
ament adapter installs its unmodified RS mesh directory and exact license
from the whole submodule. It maintains no selectively copied upstream tree.
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
inertias, physical spring/effort declarations, finite simulation and native-jaw
coupling. The six-joint KDL model plus finger payload is compared with whole-tree
inverse dynamics across 50 arm poses and three jaw positions.

The reusable controller's own model, loaned-interface and real ControllerManager
fake-hardware tests are retained. SDK fault/freshness decoding has eight offline
tests. Driver virtual-CAN lifecycle tests skip without an isolated virtual
interface; a skip is not torque-lifecycle evidence. Live mock teleoperation is
covered by the
[smoke sequence](src/rebot_b601_rs_mock/README.md#teleoperation-on-the-mock).
No physical-arm or end-to-end deployment acceptance is claimed.

The workspace retains the empty-workspace Docker targets, `base` / `runtime` /
`drivers` services, CI, LFS and formatting setup. See
[MoveIt Pro documentation](https://docs.picknik.ai/) for installation and
runtime configuration.
