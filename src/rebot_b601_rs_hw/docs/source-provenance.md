# Preserved gravity and thermal sources

These imports preserve published source changes, not a validated or merged source
stack. No physical startup, gain write, motion, thermal trip or recording was
performed during this migration.

| Source | Destination and scope |
| --- | --- |
| [Current example workspace](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/c7422c8853e495f0e481daa639c2919be24d73f9/src/rebot_bench_base_config) | Thermal guard and tests imported before applying the measured-pose recovery change. |
| [Thermal recovery change](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1137), `f9b4293a7fad58035a78f9742e8ff7f1367bbbff` | `scripts/rebot_thermal_guard.py` and `test/test_thermal_guard.py`. The functional patch and author history are retained. The only source adaptation removes the optional leader import for the fixed seven hardware joint names. |
| [Gravity modes and identification](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1138), `2ba9d42c65d21df0919b65b12e181736e0c57b76` | `SetRebotMotorGain` source/header/tests, registered by the hardware-only loader, plus Start/Stop Gravity Comp Objectives. Namespace and catalog/model wording changed, not request validation, timeout, halt, gain ordering or measured-anchor semantics. Collection/fit remains a separate office source layer. |
| [Integrated bench snapshot](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1139), `7d7761965e8082d169be39817ab3545c924e446c` | Already includes the same thermal and gain changes; do not apply them twice. Its reusable gains/limits are stock tuning; fitted gravity model/gains, recorder and application content remain office-specific. The stock Gemini 305 driver and nominal frame are maintained here; calibrated camera mounts remain office-specific. |

The reusable chainable controller is maintained as its own
`kdl_gravity_compensation_controller` package, imported from the published
[controller change](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1134)
at `7d1a10ff9fa804694aea52ed729e5a2ec3d55b98`. This hardware loader does not wrap or
replace the controller.

## Recovery integration

The physical-only additional driver launch constructs one thermal guard; mock
and MuJoCo construct none. Its preserved request names `Return reBot Leader Home`,
now in the hardware library along with `reBot Move to Arm Waypoint`. Both are
path-filtered imports from current example main (source commits
`3a14193f5d2dc11c80654891fd3dda28c52091a8`,
`7ff44f38cf5399c19e92acd0906b8035260e6af1`,
`fd6f446aba27711a30b09ba974fe5ad1992b77f7`). Stock omits **only the recorder leaf**;
Boston retains its recorder and consumes the same three helpers from
`motion_recovery_behaviors`, whose README records the temporary common-package
location and original tests. No generic framework or office plugin is loaded by stock.

The arm algorithm remains: activate/retain JTAC, skip arm motion in the inclusive
0.08-rad HOME box, center joint5 first if needed, then return to Zeroed Rest at
0.0833333333 velocity/acceleration scales (0.05 rad/s nominal maximum). Both
plans use bounded measured starts; anchoring holds the jaw and retains the
0.10-rad stale-command fallback. Only the rest leg has the existing collision-
checked point-to-point fallback. After the arm legs, wait 0.5 s, optionally ask
`/rebot_leader_bridge/home_verified` to clear a failed-HOME latch, then attempt
Close Gripper at nominal zero. Verification and close run under ForceSuccess:
a missing bridge or failed close does not fail an otherwise successful arm
HOME. The bridge independently checks fresh, stationary HOME and healthy
motors; it never moves the arm. Failure does not disable torque. These are
source semantics, not a claim that a physical recovery from every sag succeeds.
The source Xacro thermal cutoff/clear parameters are explicitly 100°C/75°C;
leaving cutoff unset would disable the driver latch and is not equivalent.

The preserved recovery sequence and its limitations are described in
[Thermal recovery](../README.md#thermal-recovery). The source retains all stage
deadlines, repeat-latch handling, fault checks and partial-write limitations.

## Offline checks

The package tests exercise all 78 preserved thermal cases, 11 JTAC-ownership
cases, 4 anchor-release cases and all 12 gain Behavior cases, including a real
ROS client with a fake service and actual plugin loading. The Objective library test uses the installed ObjectiveServer to
load plugins, perform canonical full XML validation and instantiate the stock
Objectives for mock, simulation and hardware configurations without ticking
any tree. A separate offline test executes Stop → Start → Stop against actual
ControllerManager/JTAC with memory-only hardware, observing all36gain-service
requests after the measured command anchor while nominal gravity remains active.
The HOME regression executes the actual source trees and real bounding/anchoring
helpers with deterministic planner/trajectory seams: yaw-first order, inclusive
HOME box, source speed scales, bounded starts, jaw holding during arm motion,
post-HOME zero close, optional verification/close failures, rest-only fallback,
validation and failure-before-endpoint-commit. Three preserved native-planner
cases explicitly skip without a MoveIt Pro license. Launch tests evaluate actions
without executing the returned Node. These checks do not exercise a real hardware
interface, licensed native HOME planning, UI controls or physical fault recovery.
