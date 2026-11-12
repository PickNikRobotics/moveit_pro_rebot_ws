# Spring and damped gravity modes

Hardware startup is **spring**, not float: JTAC and standalone gravity are active,
with arm kp `[50,150,150,50,50,50]`, kd `[3,10,10,5,4,4]` and jaw kp=12/kd=0.5.
Stock gravity uses nominal inertials, unit gains, zero offsets and the native-finger
payload. Boston can select its fitted gravity model without changing the mode
Objectives' ordering contract.

**Physical floating still requires commissioned thermal recovery.**
The guard, native HOME dependency and physical-only launch wiring are integrated
and covered by offline tests; see [source provenance](source-provenance.md).
An installed Objective is not permission or evidence of safe physical use.

## Start Gravity Comp

1. Make JTAC the arm position owner (`SwitchController`, automatic deactivation,
   strict). Teleoperation Joint Jog/Pose Jog and Mirror leave another position
   controller active after they end; gravity is effort-only and unaffected.
   The remaining steps are the `reBot Set Float Gains` subtree.
2. Read all seven measured joints from the current planning scene.
3. Send and complete a nonzero-stamped, two-identical-point stationary trajectory
   at 0/0.2 s through JTAC.
4. Queue kd `[1.5,3.5,5,1.25,1,1]`, then kp `[0,0,0,0,0,0]`, for arm motor IDs1–6.
5. Finish successfully. JTAC and gravity remain active; the jaw gains are unchanged.

## Stop Gravity Comp

The same JTAC switch, then the same measured-pose trajectory (the
`reBot Set Spring Gains` subtree), precede all gain writes. Then restore the
configured stock spring kd `[3,10,10,5,4,4]`, followed by kp
`[50,150,150,50,50,50]`, and finish. Stop retains torque; it does not command HOME
or disable motors. These values match the shipped GPIO configuration. A custom
installation that changes its spring configuration must also update this
Objective; the thermal guard separately reads its initial gains from the URDF.

These reusable damping/stiffness values and sequence come from the published
[gravity mode source](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1138)
and [integrated source](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1139),
not a new tuning or physical validation run. The two gain subtrees retain the
original flat sixteen-Action sequence, with only model/category wording changed.
The only controller change is the leading JTAC activation; neither Objective
changes the gravity model, gains, offsets or feedforward caps.

## Preconditions and limits

Support the arm and hold still while switching. The anchor is a measured
snapshot, not continuous tracking. No competing Objective, jog, leader or
manual gain/trajectory writer may command during floating or switching.
Cancelling an already-completed Start does not restore stiffness: run Stop.
Teleoperate, Mirror and Return reBot Leader Home (and so Relax at HOME) also run
the spring subtree after engage. Motion Objectives that do not engage, such as
Move to Raised, keep the float.

`SetRebotMotorGain` allows only arm IDs1–6 and kp/kd, validates protocol bounds,
waits at most3s for service discovery and0.5s for a response, and never accesses
CAN directly. Success means the driver queued a request, not motor gain readback.
A timeout, rejection or cancellation can leave earlier writes applied; there is
no rollback, arbitrary register write, motor enable or recovery monitor inside
these Objectives. Fresh health and the existing HOME-before-OFF policy remain
required. No physical sag, float-quality or no-snap guarantee is made.

## Offline ordering regression

The package test runs the actual source spring → float → spring gain subtrees through their
registered leaves, installed ControllerManager/JTAC and the imported chainable
gravity controller. Only the planning-scene response, gain service and memory
hardware are fake. Command propagation is disabled so stale commands cannot be
hidden by copying them into measured state.

Each run starts with a different measured pose and a deliberately stale position
command. All36gain-service requests observe all7position commands equal to the
captured measured pose within1e-12rad, including12positive-kp restorations. JTAC
stays active and the gravity effort remains nominal KDL plus finger payload with
unit gains. This proves software command ordering, not motor timing, CAN delivery,
physical holding or the independent thermal HOME path.
