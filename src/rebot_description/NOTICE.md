# Stock description source

The complete upstream repository is pinned at
`Seeed-Projects/reBot-DevArm@58f9e433deec2ba118c5c22d60c07b9897e92acc`
in `src/external_dependencies/rebot_description`.
The ament adapter installs its unmodified `Rebot_Arm_description/RS/meshes`
and exact upstream CERN-OHL-W-2.0 license. No copied mesh tree is maintained
here. It also installs `urdf/gemini305_wrist_camera.xacro`, a PickNik-authored,
frame-only nominal Gemini 305 mount. That transform is a CAD/nominal geometry
estimate, not a hand-eye calibration or a serial-specific measurement; a
downstream office package must override it with its calibrated mount. Editable
mechanical sources remain in the same complete submodule. Hardware design
copyright 2026 Seeed Studio Co., Ltd.
