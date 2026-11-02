# Preserved gravity and thermal sources

These imports preserve published source changes, not a validated or merged source
stack. No physical startup, gain write, motion, thermal trip or recording was
performed during this migration.

| Source | Destination and scope |
| --- | --- |
| [Current example workspace](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/c7422c8853e495f0e481daa639c2919be24d73f9/src/rebot_bench_base_config) | Thermal guard and tests imported before applying the measured-pose recovery change. |
| [Thermal recovery change](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1137), `f9b4293a7fad58035a78f9742e8ff7f1367bbbff` | `scripts/rebot_thermal_guard.py` and `test/test_thermal_guard.py`. The functional patch and author history are retained. The only source adaptation removes the optional leader import for the fixed seven hardware joint names. |
| [Gravity modes and identification](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1138), `2ba9d42c65d21df0919b65b12e181736e0c57b76` | `SetRebotMotorGain` source/header/tests, registered by the hardware-only loader, plus Start/Stop Gravity Comp Objectives. Namespace and catalog/model wording changed, not request validation, timeout, halt, gain ordering or measured-anchor semantics. Collection/fit remains a separate office source layer. |
| [Integrated bench snapshot](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1139), `7d7761965e8082d169be39817ab3545c924e446c` | Already includes the same thermal and gain changes; do not apply them twice. Its reusable gains/limits are stock tuning; fitted gravity model/gains, cameras, recorder and application content remain office-specific. |

The reusable chainable controller is maintained as its own
`kdl_gravity_compensation_controller` package, imported from the published
[controller change](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1134)
at `7d1a10ff9fa804694aea52ed729e5a2ec3d55b98`. This hardware loader does not wrap or
replace the controller.

## Recovery integration checkpoint

The thermal guard is built and installed, but **not yet launched by this stock
config**. Its preserved native HOME request names `Return reBot Leader Home`,
which is not in the stock Objective library. That source Objective depends on
bounded-start planning, command-anchored motion and an office recorder. Merely
renaming it to `Move to Rest` would change recovery semantics; copying the whole
bench library would introduce office content. Complete that dependency split
before treating this configuration as a commissioned hardware integration.

The preserved sequence is: fresh measured pose and healthy enabled motors →
nonzero-stamped stationary JTAC anchor → two advancing matching references →
configured arm kd, then kp, through the existing driver service → cancel →
native HOME → fresh stationary HOME verification → torque OFF → cooling.
Hot startup with all motors OFF stays OFF. It never enables torque. The source
retains all stage deadlines, repeat-latch handling, fault checks and partial-write
limitations. A queued gain acknowledgement is not motor readback or an atomic
whole-arm update. Competing writers and continued hand motion are not covered
by a physical no-snap guarantee.

## Offline checks

The package tests exercise all 78 preserved thermal cases and all 12 gain
Behavior cases, including a real ROS client with a fake service and actual
plugin loading. The Objective library test uses the installed ObjectiveServer to
load plugins, perform canonical full XML validation and instantiate the stock
Objectives for mock, simulation and hardware configurations without ticking
any tree. A separate offline test executes Stop → Start → Stop against actual
ControllerManager/JTAC with memory-only hardware, observing all36gain-service
requests after the measured command anchor while nominal gravity remains active.
These checks do not exercise a real hardware interface, native HOME execution,
UI controls or physical fault recovery.
