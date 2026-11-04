"""Check generated-model contracts against the authoritative URDF."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory


def model():
    # Native MuJoCo lacks MoveIt Pro's package URI resolver; resolve only in memory.
    root = ET.parse(os.environ["SCENE_XML"]).getroot()
    for mesh in root.findall("asset/mesh"):
        package, relative = mesh.get("file")[len("package://") :].split("/", 1)
        mesh.set("file", str(Path(get_package_share_directory(package)) / relative))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def test_joint_and_actuator_contract():
    m = model()
    urdf = ET.parse(os.environ["ROBOT_URDF"]).getroot()
    joints = {
        j.get("name"): j for j in urdf.findall("joint") if j.get("type") != "fixed"
    }
    assert m.nq == 9 and m.nu == 7
    assert m.nkey == 1 and m.key("default").qpos.shape == (m.nq,)
    for name, joint in joints.items():
        mj = m.joint(name)
        limit = joint.find("limit")
        np.testing.assert_allclose(
            mj.range, [float(limit.get("lower")), float(limit.get("upper"))]
        )
        np.testing.assert_allclose(
            mj.axis, [float(v) for v in joint.find("axis").get("xyz").split()]
        )
    for index in range(1, 8):
        name = f"joint{index}"
        assert m.actuator(name).trnid[0] == m.joint(name).id
    assert all(
        g.contype == 0 and g.conaffinity == 0
        for g in [m.geom(i) for i in range(m.ngeom)]
    )


def test_reset_holds_and_jaw_coupling():
    m = model()
    data = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, data, 0)
    initial = data.qpos.copy()
    for _ in range(1000):
        mujoco.mj_step(m, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    np.testing.assert_allclose(data.qpos, initial, atol=1e-6)
    np.testing.assert_allclose(data.qvel, 0, atol=1e-6)
    data.ctrl[m.actuator("joint7").id] = 1.0
    for _ in range(2000):
        mujoco.mj_step(m, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    motor = data.qpos[m.joint("joint7").qposadr[0]]
    assert abs(motor - 1.0) < 0.01
    for name in ["gripper_joint1", "gripper_joint2"]:
        jaw = data.qpos[m.joint(name).qposadr[0]]
        assert abs(jaw - 0.008 * motor) < 0.0001
