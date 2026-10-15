# reBot B601-RS physical backend

This 10.x/Jazzy config inherits the same stock arm and native jaw through the
simulation config, then selects `robstride_hardware_interface/RobstrideHardware`
and wall time. It has no camera launch, simulated sensor or simulation-only
controller. The physical plugin uses only position commands in the baseline
motion (impedance) mode. No gravity feedforward is enabled.

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

Keep per-unit measurements and optional gravity gains in the office or
installation configuration. The base kp/kd values and 100 Hz controller update
rate are unchanged by this extraction. Motor status and fault telemetry remain
available through the pinned driver's `~/robstride_state` topic and services.

## Verification and rollback

No physical activation, CAN access or arm motion was performed here. Before
physical use, verify timestamps, all seven measured positions, mode and fault
telemetry, controller claims, actual stop behavior and reduced-scale motion
with an operator present. Revert the workspace commit and recursively update
submodules to restore a prior deployment; never substitute an older SDK pin
without rechecking fault decoding and driver compatibility.
