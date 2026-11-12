# Virtual CAN qualification

The physical backend is qualified against seven fake RobStride motors on a kernel
`vcan` link before any bench session. Nothing here opens `can0`, enables a real
motor or moves the arm. The fake follower is an ideal position follower: it is
not a thermal, gravity, load or friction model, so float quality, sag and motor
temperature still need bench measurement.

## What runs

| Test | Package | Covers |
| --- | --- | --- |
| `test_fake_follower` | `robstride_hardware_interface` | Driver lifecycle on vcan: activation from measured state, out-of-bounds refusal, fault versus link loss, silent joint, recovery, 100 °C latch, fault bit 14, kp/kd confirmation, transmit failure, truncated feedback, torque off, shutdown, position modes, motor watchdog |
| `test_joint_coordinates` | `robstride_hardware_interface` | Bounds, signs, whole-turn offsets and continuous wrap, without CAN encoding |
| `test_deactivate_torque` | `robstride_hardware_interface` | Existing single-motor deactivate/hold tests, now behind the same vcan check |
| `test_vcan_controllers` | `rebot_b601_rs_hw` | This config's expanded URDF with only `can_interface` changed to `vcan0`, the real driver plugin, ControllerManager and the stock controllers |

`test_vcan_controllers` checks that:

- JTAC drives all seven motors to a target, and `/joint_states` is recorded.
- Arm jog moves J1 with the commanded sign, stops after `command_timeout`, and
  leaves the jaw alone.
- Jaw jog commands `joint7` in radians. The mimic fingers follow at 0.008 m/rad
  and have no command interface.
- Pose jog moves `gripper_end` along +x/-x in `base_link` and stops on timeout.
- JTAC is re-anchored with a stationary trajectory at the measured pose after a
  jog, and the stiff jaw gets no positive-kp frame away from its current pose.
- Stop Gravity Comp's installed `reBot Set Spring Gains` subtree, run after a float in which the arm
  was moved away from JTAC's held target, sends no positive-kp frame whose
  target differs from the motor's pose before that frame (no snap).

## Running

The tests accept only a link whose kernel kind is `vcan`. A physical interface
renamed `vcan0` is refused. Creating a `vcan` link needs `CAP_NET_ADMIN`, so
run in a separate network namespace. Reuse an existing MoveIt Pro image, for
example:

```bash
docker run --rm --network none --cap-add NET_ADMIN -v "$PWD":/workspace <moveit-pro-image> bash
# inside: build the workspace with BUILD_TESTING=ON, source it, then
python3 install/robstride_hardware_interface/share/robstride_hardware_interface/test/with_vcan.py -- \
  colcon test --packages-select robstride_sdk robstride_hardware_interface rebot_b601_rs_hw
```

`with_vcan.py` creates `vcan0` (or `$ROBSTRIDE_TEST_CAN_IFACE`), removes only a
link it created, and sets `ROBSTRIDE_REQUIRE_VCAN=1` so that a missing link fails
instead of skipping. Without a `vcan` link and that variable, the CAN tests
skip, so a CI job that does not provide a `vcan` link reports them as skipped.
The host needs the `vcan` kernel module loaded.

## Current behavior these tests record

- A lost joint latches the freeze. Peers hold the pose they had at the loss and
  ignore controller commands. A joint that answers again stays disabled.
- Reactivation sends Stop to every motor before re-enabling. Every motor briefly
  releases torque, and a reactivation with a joint still silent ends with all
  motors disabled. The driver does not check the pose: HOME and support of the
  arm (it has no brakes) are operator steps before reactivation.
- A static-hold overload that sets fault bit 14 disables that motor. It does not
  set the thermal latch, which is driven only by reported temperature
  (99.9 °C does not latch; 100 °C does, and stays latched after cooling).
- A `kp`/`kd` write reports success when the gain is queued. Motion-mode gains
  have no readback: `get_data_from_robstride` returns `result: false`, and a
  dropped motion frame is not detected.
- A CAN transmit failure is logged; `write()` still returns `OK`, and the stale
  feedback then becomes link loss.

## Bench timing and watchdog settings

| Setting | Value | Consequence |
| --- | --- | --- |
| SocketCAN bitrate / `txqueuelen` | 1 Mbps / 1000 | Set on the host before starting |
| `update_rate` | 100 Hz | Controller manager rate used by the tests |
| `error_timeout_ms` | 4000 (driver default) | A silent joint is declared lost, and `comm_state` turns `-1`, 4 s after its last feedback. Also the activation probe and enable-confirm deadline. Enable is resent every 200 ms |
| `motor_can_timeout_ms` | 0 | Motor watchdog off. Required with `hold_torque_on_deactivate: true`; an armed watchdog (units of 50 µs on the wire) releases torque that long after the bus goes quiet |
| `hold_torque_on_deactivate` | true | Clean deactivation and process exit leave motors holding |
| `freeze_on_joint_loss` | true | Latched freeze as above |
| `thermal_cutoff_c` / `thermal_clear_c` | 100 / 75 °C | Latch at 100 °C. Cooling alone does not clear it (tested at 25 °C); see the driver's thermal recovery handoff |
| Jog `command_timeout` | 0.2 s | Arm, jaw and pose jog stop 0.2 s after the last command |
| JTAC `default_path_tolerance` | 0.20 rad | Path error that aborts a trajectory |

The driver tests use `error_timeout_ms: 350` only to keep them short. That value
is not a bench recommendation.
