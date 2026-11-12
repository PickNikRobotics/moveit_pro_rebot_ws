# Model provenance and modifications

Arm URDF and the original `assets/{shared,visual}/*.STL` are derived from
[Seeed Studio's reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc/Rebot_Arm_description/RS)
at commit `58f9e433deec2ba118c5c22d60c07b9897e92acc`.
Upstream URDF: `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`.
Stock mesh bytes are unchanged and are referenced through the whole
`rebot_description` dependency; they are not duplicated in this package.

Hardware design copyright © 2026 Seeed Studio Co., Ltd.
The model and derived `rebot.urdf.xacro` are distributed under **CERN-OHL-W-2.0**;
the complete upstream license is in [LICENSE](LICENSE). Seeed's root README
identifies hardware designs under this license separately from its software.
The [upstream source location](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc)
also contains the editable mechanical designs. The original modified model's source is `PickNikRobotics/moveit_pro_example_ws`.
This migrated assembly lives on branch `picknik_boston_rebot_b601_rs` of
`PickNikRobotics/moveit_pro_rebot_ws`.

Modified by PickNik Inc., 2026-10-09:

- Rewrote relative mesh paths as ROS package URIs and normalized XML formatting.
- Retained upstream link frames, joint origins, axes, inertias and collisions.
- Replaced arm position limits with the specified motor-frame limits and set
  conservative velocity limits.
- Added the geometry-free J7 motor coordinate, coupled both original finger
  sliders to it at 0.008 m/rad, and updated slider limits accordingly.
- Added a fixed world frame and uncalibrated, frame-only D435 attachment points.
- Added ros2_control using only `mock_components/GenericSystem`.

Modified by PickNik Inc., 2026-10-10:

- Added uncalibrated, frame-only OAK-D scene-camera attachment points at `world`.

Modified by PickNik Inc., 2026-10-11:

- Added xacro arguments placing `base_link` and the scene-camera mount in
  `world`; both default to coinciding with `world`.
- Added a `mujoco` hardware selector that emits no `ros2_control` block and no
  colour optical frames, for `rebot_bench_sim`'s MuJoCo description.

Modified by PickNik Inc., 2026-10-12:

- Added xacro arguments for the historical `d435_mount_joint` origin; they
  defaulted to the nominal mount.
- A wrapping description that defines the `world` placement properties before
  including this file keeps them over the placement arguments.

Modified by PickNik Inc., 2026-10-14:

- Set conservative home margins of −0.02 rad for J2 and −0.01 rad for J3,
  retaining J7's zero lower bound. These are approved operating limits, not
  measured negative hard stops.

Modified by PickNik Inc., 2026-11-08:

- Replaced the real-hardware D435 visual assembly with the stock Gemini 305
  frame macro, overridden by a hand-eye calibration for serial `CV2F861000FP`.
  The fixed `gripper_end` transform is not eye-measured; no camera mesh,
  collision or inertia is added.

Original URDF export notice retained in `rebot.urdf.xacro`: SolidWorks to URDF
Exporter, Stephen Brawner (brawner@gmail.com), commit `1.6.0-4-g7f85cfe`,
build `1.6.7995.38578`; http://wiki.ros.org/sw_urdf_exporter.

The independent MoveIt Pro configuration, Objectives and tests use the package's
BSD-3-Clause license. There is no Seeed driver or SDK code in this package.

## Printed-part visual replacements (2026-10-10)

The nine `assets/printed/*.dae` files are the supplied redesigned print meshes,
retained byte-for-byte with metre units and embedded purple/white materials.
The six arm trim parts replace Seeed's corresponding side fillers and covers;
D435_mount is the supplied wrist camera bracket. These hardware derivatives,
the STOPPER and RAIL-BASE meshes, and retained internal-plastic subsets are
licensed under CERN-OHL-W-2.0; the full license above applies. The modified
source location is this package in the public MoveIt Pro example workspace.
Brand names and logos retain their owners' trademark rights.

STOPPER and RAIL-BASE were tessellated from Seeed's `1-STOPPER-1.step` and
`1-RAIL-BASE-2.step` at revision
`171ed82cb9a4a037a6beb7d1e034352baa604b63`, under `hardware/reBot_B601_RS/3D_Printed_Parts/`.
The supplied export used 0.025 mm linear / 0.08 rad angular tolerance; DAE
coordinates are scaled from millimetres to metres. Original STEP sources are
available at that public upstream revision. DAE hashes and visual transforms
are recorded in `assets/printed/manifest.json`.

The bracket's mounting-interface reference is Seeed revision
`58f9e433deec2ba118c5c22d60c07b9897e92acc`,
`Rebot_Arm_description/Camera/urdf/d435i.urdf` and its bracket mesh. The
mounting transform, D435 body and nominal optical frame were initially reused
for the visual assembly described below.

PickNik Inc. modified the visual integration on 2026-10-10: replaced matching
trim visuals, retained internal black-plastic triangles as `pla2_internal.STL`
and `pla3_internal.STL`, and added a visual-only bracket. Collision meshes,
inertia and arm joint frames remain unchanged. The real-hardware D435
frame-only placeholder was later replaced by the Gemini 305 frame-only
calibration described below.

## Retained D435 mesh provenance (2026-10-10)

`assets/realsense/d435.dae` is a decimated (15.8 MB to 1.7 MB, quadric edge collapse; same origin, scale and material) copy of Intel's D435 mesh from
[realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros/tree/9215f26e8348ad5922a608b77882a9bfe05940f0/realsense2_description),
commit `9215f26e8348ad5922a608b77882a9bfe05940f0`, path
`realsense2_description/meshes/d435.dae`, licensed **Apache-2.0**.
The upstream license is retained byte-for-byte in `assets/realsense/LICENSE`.
Upstream notices are retained byte-for-byte in
`assets/realsense/UPSTREAM_NOTICE.txt`; `source.json` records the upstream and modified
mesh SHA-256 and Git blob hashes. The assembly offsets are from Seeed's camera URDF referenced
above; the body's mesh and nominal RGB optical offsets follow
`realsense2_description/urdf/_d435.urdf.xacro` at the same RealSense revision
(Copyright 2023 RealSense, Inc.). No driver code is copied.

The real-hardware wrist visual assembly used the bracket's CAD frame. Its
bottom-screw frame, body frame and nominal color optical frame formed one fixed
chain. The current real-hardware wrist configuration uses the Gemini 305
hand-eye calibration recorded above. The scene camera remains frame-only.

The link3 side panels retain the integrated source transforms; mesh bytes and
the lower cover pose are unchanged. The exact origins are in office_geometry.xacro.

## Boston source separation (2026-11-02)

Modified by PickNik Inc.: extracted the existing trim and camera blocks into
`office_geometry.xacro`; reuse shared stock URDF/SRDF/controller interfaces;
keep the six measured gravity-equivalent inertial-origin values in an office
YAML file, restricted to real-backend model expansion. All joint frames and
stock collision geometry are preserved. These fitted origins are not identified
physical dynamic inertias and cannot be selected for mock/MuJoCo. The office
SRDF retains the historical link2/link5 collision exclusion. The per-unit Gemini
305 hand-eye calibration is documented above.

## Main-branch defaults (2026-11-12)

Modified by PickNik Inc.: on `main`, the per-unit values above are replaced by
uncalibrated nominal defaults. The wrist camera uses the stock nominal Gemini
305 mount, the base and scene-camera transforms are rounded reference-layout
values, and `config/gravity/identified_first_moments.yaml` holds the nominal
URDF inertial origins. Trim meshes, collision geometry and the office SRDF are
unchanged. The calibrated values remain on the `picknik_boston_rebot_b601_rs`
branch.
