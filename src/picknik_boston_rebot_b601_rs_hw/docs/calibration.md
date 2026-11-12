# Calibrating your installation

The application config ships **uncalibrated nominal defaults** for every
per-unit value. They let the config build, start and plan, but perception,
grasping and gravity compensation are only as good as the calibration you add.
Put each calibrated value in your own child config (`based_on_package:
picknik_boston_rebot_b601_rs_hw`) rather than editing the shipped defaults;
`urdf_params` merge by key, so the child lists only what it changes. Rebuild
and restart Runtime after each change.

| What | Default | Where you set it |
| --- | --- | --- |
| Base placement in the world (mat) frame | `0.726 0.022 0.0`, yaw π (reference layout) | `urdf_params`: `world_to_base_xyz`, `world_to_base_rpy` |
| Scene camera mount | Reference-layout pose, about 0.565 m above the mat | `urdf_params`: `scene_camera_xyz`, `scene_camera_rpy` |
| Wrist camera mount (hand-eye) | Stock nominal Gemini 305 mount | `urdf_params`: `wrist_camera_color_optical_xyz`, `wrist_camera_color_optical_rpy` |
| Camera serials | First attached Gemini 305 and OAK-D | Wrist: `wrist_camera_serial_number` in `launch/rebot_drivers.launch.py`; scene: `scene_serial_no` in `config/cameras.yaml` |
| Scene depth scale | `1.0` (unscaled) | `scene_depth_scale` in `config/cameras.yaml` |
| Gravity model | Nominal URDF inertials, stock gains, nominal jaw payload | `boston_gravity_fit`, `config/gravity/identified_first_moments.yaml`, gravity controller gains/payload |
| Star Arm leader | Seeed stored-origin calibration file | See the [stock leader notes](../../rebot_b601_rs_hw/README.md#optional-star-arm-leader) |

Place the bench first, as in [cube-stacking layout](cube-stacking-layout.md).
The world frame is the mat's top surface (z = 0) at the origin you choose for
the layout; every placement below is measured in it.

## Camera serials

With more than one camera of a kind attached, pin each calibrated unit. Read
the serial from the driver's startup log, then set the wrist serial as the
`wrist_camera_serial_number` launch argument of the stock driver include in
your config's driver launch, and the scene serial as `scene_serial_no`. A mount
calibration belongs to one physical camera: recalibrate after swapping it.

## Scene camera and base placement

Use one printed ChArUco board lying flat on the mat at a measured world pose
(the reference bench used `DICT_4X4_50`, 7 × 5 squares of 35 mm with 26 mm
markers).

1. **Scene camera.** Detect the board in the scene camera's color image with
   its factory intrinsics and solve the board pose. Compose it with the board's
   world pose to get `scene_camera_link` in the world. `scene_camera_link`
   follows REP-103 (x along the view axis, z up); its color optical frame is
   the fixed rotation in `description/office_geometry.xacro`. Check the corner
   reprojection error (the reference bench reached 0.15 px).
2. **Base.** Either measure the base plate's position and yaw on the mat, or,
   after the wrist camera is calibrated, view the board from the wrist at rest
   and carry the board pose through the arm's forward kinematics. Hold roll,
   pitch and z at the bench plane when the base is bolted level.

Verify both with **Locate reBot Cubes**: a cube resting on the mat must report
a top face 0.05 m above z = 0, and its position must match a tape measurement
to a few millimetres.

## Wrist camera hand-eye

Calibrate the `gripper_end` → `wrist_mounted_camera_color_optical_frame`
transform with MoveIt Pro's eye-in-hand calibration (the reference bench used
the closed-form `SolveEyeInHandCalibration` result from 11 views, residual
0.0124 rad / 3.5 mm). Use views with large relative rotations between them: a
solve from small rotations can be off by centimetres. Set the result as
`wrist_camera_color_optical_xyz` and `wrist_camera_color_optical_rpy`; factory
`camera_info` stays in use.

## Scene depth scale

With the mat empty, read the median depth of the mat in the calibrated world
frame. If it is not near z = 0, set `scene_depth_scale` to the factor that
brings it there, then confirm that a 100 mm two-cube stack's top reads 100 mm.
The reference OAK-D read 4% long and needed 0.96. The factor applies to depth
only, before the point cloud is built.

## Gravity model

The nominal model works out of the box with the stock gains. To fit your arm:

1. Run **Collect reBot Gravity Data** as in the
   [collection runbook](gravity-identification.md).
2. Fit the recordings offline with `scripts/fit_gravity.py`; it reads closed
   MCAPs and writes a new output directory.
3. Copy the fitted origins into `config/gravity/identified_first_moments.yaml`
   (or your package's copy), set `boston_gravity_fit: true` in `urdf_params`,
   and set the gravity controller gains and payload chosen with the fit. A fit
   that absorbs the jaw, camera and cables uses `payload_mass: 0`; the nominal
   jaw payload would count them twice.

[Gravity calibration](gravity-calibration.md) shows the reference bench's fit,
held-out errors and limits. A fit applies only to the real backend.
