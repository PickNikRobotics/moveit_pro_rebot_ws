# Model source and modifications

Derived from `Seeed-Projects/reBot-DevArm` commit
`58f9e433deec2ba118c5c22d60c07b9897e92acc`,
`Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`.
Hardware design copyright 2026 Seeed Studio Co., Ltd., CERN-OHL-W-2.0.
The exact upstream license is LICENSE in this directory.

Modified by PickNik Inc., 2026: package mesh URIs, motor-coordinate operating
limits, conservative velocity limits, J7 motor coordinate and 0.008 m/rad
native-jaw coupling, fixed world frame, and mock/physical/MuJoCo backend
selection and arm-only optional effort interfaces. Nominal joint origins, axes,
inertias and stock collision geometry are retained; no measured gravity fit is
applied. The two native-finger masses remain in this URDF and are represented
through the gravity controller's separate payload interface only because that
controller selects a serial chain that excludes those finger branches. Stock visuals reference the whole upstream submodule through the
rebot_description ament adapter. No office cameras or trim are installed.
The configuration/control contract is ported from
`PickNikRobotics/moveit_pro_example_ws@c7422c8853e495f0e481daa639c2919be24d73f9`,
with reusable tuning from integrated source
`7d7761965e8082d169be39817ab3545c924e446c`: J2/J3 lower allowances of
-0.02/-0.01 rad, shared path tolerance 0.20 rad and J2 feedforward cap 18 Nm.
These are software defaults, not per-unit calibration or a new physical acceptance.
The supplied jaw's current kp=12/kd=0.5 and nominal masses are retained; office
camera installations, modified trim, calibrated placements and leader calibration
are not installed. Stock/default numeric boundaries are described in the
configuration rather than asserted as physically commissioned limits.
The public source of this modified model is `PickNikRobotics/moveit_pro_rebot_ws`;
editable upstream hardware sources are retained in the whole submodule.
Independent configuration, Objectives and scripts use BSD-3-Clause.
