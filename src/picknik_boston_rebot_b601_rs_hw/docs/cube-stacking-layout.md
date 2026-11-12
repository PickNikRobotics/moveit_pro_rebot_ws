# Cube-stacking layout

**Stack Blue on Red**, **Unstack Blue from Red** and **Stack and Unstack Cycle**
need two printed cubes, a flat mat and the scene camera, placed as below.
Every position is in the world frame: z = 0 is the mat's top surface. On
`main`, the config's base and scene-camera poses are nominal values for this
layout; after placing them, calibrate both (and the wrist camera) and override
the `urdf_params` in your config before running a stack.

## Print the cubes

Print [`cubes/rebot_cube_50mm.stl`](cubes/rebot_cube_50mm.stl) twice, one in
**blue** and one in **red** matte filament: a solid 50 mm cube in millimetres.
Keep each edge within 50 ± 0.5 mm and the faces flat; the perception fits a
50 mm cube and the grasp closes the jaw to a 45 mm aperture on it. The
reference cubes weigh about 35 g; full infill is not needed.

## Place the bench

| Item | Placement in world (m) |
| --- | --- |
| Mat top surface | z = 0; flat, a contrasting single color, 0.9144 × 0.40 m, centred at (0.133, 0.160), long side along world X |
| Wall | Optional; the planning scene models a 0.6 m tall wall face at y = 0.355 along the mat's +Y edge. Without one, keep that side clear |
| Robot base (`base_link`) | (0.726, 0.022, 0), yaw π: the arm reaches along world −X over the mat; base plate level with the mat |
| Scene camera (`scene_camera_link`) | About (−0.075, 0.07, 0.565), looking down the mat toward the robot (rpy ≈ 0.17, 0.975, 0.245) |
| Cubes | On the mat, centres inside x 0.28–0.46, y 0.02–0.20 (where the reference stacks ran), at least 0.11 m apart and both in the scene camera's view |

**Unstack** sets the blue cube down at a random spot with its centre in
x 0.35–0.46, y 0.02–0.20, clear of red. Keep that area free.

The planning scene adds the mat and wall from **reBot Fixed Bench Scene**
(`objectives/rebot_fixed_bench_scene.xml`). If your bench differs, change those
two boxes, the cube area and the place region in
`objectives/rebot_draw_place_spot.xml` together, and recheck reach with
**reBot Check Stack Reach** before any motion.
