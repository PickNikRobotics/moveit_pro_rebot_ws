# Model source and modifications

Derived from `Seeed-Projects/reBot-DevArm` commit
`58f9e433deec2ba118c5c22d60c07b9897e92acc`,
`Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`.
Hardware design copyright 2026 Seeed Studio Co., Ltd., CERN-OHL-W-2.0.
The exact upstream license is LICENSE in this directory.

Modified by PickNik Inc., 2026: package mesh URIs, motor-coordinate operating
limits, conservative velocity limits, J7 motor coordinate and 0.008 m/rad
native-jaw coupling, fixed world frame, and mock/physical/MuJoCo backend
selection. Joint origins, axes, inertias and stock collision geometry are
retained. Stock visuals reference the whole upstream submodule through the
rebot_description ament adapter. No office cameras or trim are installed.
The configuration/control contract is ported from
`PickNikRobotics/moveit_pro_example_ws@ecf364862d16279a85a57804230501c9e40ee9e9`.
The public source of this modified model is `PickNikRobotics/moveit_pro_rebot_ws`;
editable upstream hardware sources are retained in the whole submodule.
Independent configuration, Objectives and scripts use BSD-3-Clause.
