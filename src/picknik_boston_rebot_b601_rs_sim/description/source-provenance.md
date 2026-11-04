# Boston twin model provenance

The MuJoCo bench source is migrated from
[moveit_pro_example_ws](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/6bf6c0ed37d580141b2ef72bd67bc351e0b89c37/src/rebot_bench_sim),
commit `6bf6c0ed37d580141b2ef72bd67bc351e0b89c37`. Its package tree is identical to
`c7422c8853e495f0e481daa639c2919be24d73f9`. The original package's path-filtered
history is retained before the package rename, including the initial twin,
layout calibration, JPEG transport, contact proxy, NoSlip/released-stack test,
and leader startup changes. Original authors and source notices remain.

The source package is an office system, not stock vendor geometry. Its nominal
arm dynamics, camera layout, friction, equality constraints, gains, timestep,
NoSlip settings and strict unassisted-grip expected failure are preserved.
The historical evidence under `docs/stack_validation/` and render sheet describes
the original simulation revision, **not a new deployment or physical test**.
Source filenames in that evidence are intentionally unchanged.

Migration changes:

- The optional config is `picknik_boston_rebot_b601_rs_sim`, layered over the
  existing Boston hardware config, and explicitly selects MuJoCo, nominal
  inertias, only state/trajectory controllers, and simulation-only launch.
  This mirrors the source overlay without changing the verified stock chain or
  Boston hardware defaults. Physical drivers, thermal guard and USB leader do
  not start with the twin. The leader can still be explicitly enabled.
- `layout_snapshot.py` expands the common office Xacro rather than parsing one
  monolithic source file. `generate_description.py` freezes that nominal arm,
  trim and camera geometry at the same layout as the MJCF. Its control include
  preserves the original camera topics/rates and contains one MuJoCo system.
  Fitted gravity-equivalent hardware origins are refused in simulation.
- Original stock mesh URIs now use `rebot_description`, backed by the whole
  Seeed submodule. Office trim stays with the Boston description; converted
  meshes stay here with the exact CERN/RealSense license texts and notices.
- The source jaw actuator remains kp12/kv0.05 with its original ±14Nm range;
  it is deliberately **not** represented as the physical jaw's kd0.5/1Nm limit.
  Shared trajectory path tolerance is the migrated 0.20rad setting.
- The launch reads the selected model's frozen proxy choice: cube scene enabled,
  peg scene disabled. The peg adapter removes only the weld referencing the
  removed cube and retains native-jaw equalities.
- Hardware-only Start/Stop Gravity Comp and gravity collection are non-runnable
  refusal overrides in the twin. They fail without gain requests, motion or
  recording; the real hardware trees remain unchanged in the parent.
- The original licensed Runtime integration fixture is opt-in via
  `REBOT_RUN_SIM_INTEGRATION=1`. Deterministic model, config, camera, proxy,
  recorder and native ObjectiveServer tests remain independent of that gate.

This migration does not validate contact fidelity, physical clearance, balance,
no-snap gain changes, thermal recovery, insertion, real-time capacity or a
full Desktop/Runtime deployment. The unassisted physical-grip regression remains
an expected failure; proxy-assisted success is not real-gripper evidence.
