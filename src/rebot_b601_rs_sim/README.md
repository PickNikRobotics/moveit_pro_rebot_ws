# Minimal reBot simulation

The stock arm's free-space MuJoCo backend inherits shared semantics and
controllers from `rebot_b601_rs_mock`. Its generated model uses the same URDF
and stock description assets. No environment or cameras are installed, and
visual meshes have contacts disabled. This model does not demonstrate grasping
or insertion. The standalone bench example owns those scene features.
