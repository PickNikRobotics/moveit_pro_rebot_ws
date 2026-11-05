# reBot B601-RS physical backend

This 10.x/Jazzy config inherits the same stock arm and native jaw through the
simulation config, then selects `robstride_hardware_interface/RobstrideHardware`
and wall time. It has no camera launch, simulated sensor or simulation-only
controller. Hardware defaults to motion (MIT spring) mode with normal configured
kp/kd. JTAC owns all seven position interfaces; the active standalone gravity
controller owns the six arm effort interfaces. The jaw has no gravity effort
command interface.

## Integration status

The stock model/controller composition is a software migration, not a physical
acceptance. The physical backend launches the preserved thermal guard with its
native HOME dependency, using the three shared recovery helpers. The latch is
100°C with 75°C clear, matching the source configuration. Mock/MuJoCo do not
launch the guard. Commissioning still requires separately authorized physical
recovery checks; offline closure is not a hardware safety certification.
[Source provenance and limitations](docs/source-provenance.md) distinguish
migrated functionality from unverified runtime behavior.

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

Return to a supported home pose under power before normal shutdown. The arm
can remain rigid after the process stops until motor power is cut. A restart
with torque disabled is itself a commissioning operation, with the arm
supported. Validate the manufacturer's independent stop/power-isolation path
before physical motion. The model, reference waypoints and software holding
policy do not establish that an installation is safe to operate.

Keep per-unit measurements and fitted gravity gains in the office configuration.
Reusable tuning from the integrated source is shared by the stock configs:
J2/J3 lower allowances -0.02/-0.01 rad, shared arm/jaw path tolerance 0.20 rad,
and J2 `max_feedforward_effort` 18 Nm. Other feedforward caps retain their driver
defaults. The arm kp/kd values, jaw kp=12/kd=0.5 and 100 Hz update rate are retained.
These defaults are not calibration or physical acceptance for a new installation.
Motor status and fault telemetry remain available through the pinned driver's
`~/robstride_state` topic and services.

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
