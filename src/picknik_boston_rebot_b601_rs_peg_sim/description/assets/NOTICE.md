# Frozen factory fixture geometry

The three STL meshes and `provenance.json` are unchanged from
[moveit_pro_example_ws at f31ed6ddf8f706d86cb4fd6fa92774c4d8ec698f](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/f31ed6ddf8f706d86cb4fd6fa92774c4d8ec698f/src/rebot_peg_in_hole_sim/description/assets).
Original copyright: PickNik Inc., 2026; BSD-3-Clause, supplied in the package's
`LICENSE`.

Editable source geometry is the three
[`pick_and_place_bracket_*.3mf` files](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/f31ed6ddf8f706d86cb4fd6fa92774c4d8ec698f/src/picknik_accessories/printing/factory_sim/3mf).
The retained converter accepts only metre-unit, single-mesh, identity-transform
source files. Source and output SHA-256 values, triangle counts and native
assembly bounds are recorded in `provenance.json`. No mesh was rescaled,
recentered or modified by this migration.

`charuco_board.png` is the source snapshot's synthetic DICT_4X4_50 board raster,
not a new camera observation. The scene adapter generates the illustrative
12.7mm mat grid at build time.

These are visual-only fixtures. Concave holes must not be replaced by solid
convex contact hulls. Exact simulated feature coordinates reproduce the frozen
registration; they do not certify physical seating, contact, grasping or the
single-observation wrist calibration. The physical trial remains NOTSEATED.
