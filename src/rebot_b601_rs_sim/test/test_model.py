"""Check the installed model, measured native-jaw contact and reset contracts."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
import mujoco
import numpy as np


def model():
    # Exercise the same package URI provider as MujocoSystem, not rewritten paths.
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    return mujoco.MjModel.from_xml_path(os.environ["SCENE_XML"])


def step(m, data, count):
    for _ in range(count):
        mujoco.mj_step(m, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
        assert not any(warning.number for warning in data.warning)


def test_joint_and_actuator_contract():
    m = model()
    source = Path(os.environ["SCENE_XML"]).read_text()
    assert "CERN-OHL-W-2.0" in source and "Derived from Seeed" in source
    urdf = ET.parse(os.environ["ROBOT_URDF"]).getroot()
    joints = {
        j.get("name"): j for j in urdf.findall("joint") if j.get("type") != "fixed"
    }
    assert m.nq == 9 and m.nu == 7 and m.nmocap == 1
    assert m.nkey == 2 and m.key("default").qpos.shape == (m.nq,)
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
    colliding = {m.geom(i).name for i in range(m.ngeom) if m.geom(i).contype[0] != 0}
    assert colliding == {"left_pad_contact", "right_pad_contact", "contact_block_geom"}
    assert all(m.eq_type == mujoco.mjtEq.mjEQ_JOINT)  # No attachment/grasp weld.
    assert m.ncam == 0
    np.testing.assert_allclose(m.actuator("joint7").forcerange, [-1, 1])
    for side in ("left", "right"):
        contact = m.geom(f"{side}_pad_contact")
        assert contact.bodyid[0] == m.body(f"gripper_{side}").id
        assert contact.dataid[0] == m.mesh(f"gripper_{side}_visual_0").id
    np.testing.assert_allclose(m.key("default").qpos, 0)
    np.testing.assert_allclose(m.key("default").ctrl, 0)


def test_reset_holds_and_jaw_coupling():
    m = model()
    data = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, data, m.key("default").id)
    initial = data.qpos.copy()
    step(m, data, 1000)
    np.testing.assert_allclose(data.qpos, initial, atol=1e-6)
    np.testing.assert_allclose(data.qvel, 0, atol=1e-6)
    assert data.ncon == 0
    data.ctrl[m.actuator("joint7").id] = 1.0
    step(m, data, 2000)
    motor = data.joint("joint7").qpos[0]
    assert abs(motor - 1.0) < 0.01
    for name in ["gripper_joint1", "gripper_joint2"]:
        assert abs(data.joint(name).qpos[0] - 0.008 * motor) < 0.0001
    mujoco.mj_resetDataKeyframe(m, data, m.key("default").id)
    np.testing.assert_allclose(data.qpos, initial, atol=1e-12)
    np.testing.assert_allclose(data.qvel, 0, atol=1e-12)
    step(m, data, 1000)
    np.testing.assert_allclose(data.qpos, initial, atol=1e-6)


def test_native_jaw_contact_blocks_motor_not_only_passive_fingers():
    m = model()
    data = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, data, m.key("contact_smoke").id)
    step(m, data, 100)
    assert data.ncon == 0, "50 mm block must fit inside the open jaws"
    force = np.zeros(6)
    peak_force = 0.0
    touched = set()
    for index in range(2000):
        data.ctrl[m.actuator("joint7").id] = max(
            0, m.key("contact_smoke").ctrl[6] - index * m.opt.timestep * 1.2
        )
        step(m, data, 1)
        for contact_index, contact in enumerate(data.contact[: data.ncon]):
            touched.update(m.geom(geom).name for geom in contact.geom)
            mujoco.mj_contactForce(m, data, contact_index, force)
            peak_force = max(peak_force, force[0])
    assert touched == {"left_pad_contact", "right_pad_contact", "contact_block_geom"}
    assert 1 < peak_force < 150
    assert data.ctrl[6] == 0
    # An actuator that only copies its command would report closed. Contact
    # must reach the motor coordinate used by ROS, not just the passive jaws.
    assert data.joint("joint7").qpos[0] > 2.9
    for name in ("gripper_joint1", "gripper_joint2"):
        assert abs(data.joint(name).qpos[0] - 0.008 * data.qpos[6]) < 0.0001
    assert min(contact.dist for contact in data.contact[: data.ncon]) > -0.0005
    data.ctrl[6] = m.key("contact_smoke").ctrl[6]
    step(m, data, 1500)
    assert data.ncon == 0
    assert abs(data.qpos[6] - data.ctrl[6]) < 0.01
    mujoco.mj_resetDataKeyframe(m, data, m.key("default").id)
    step(m, data, 1000)
    np.testing.assert_allclose(data.qpos, 0, atol=1e-6)
    assert data.ncon == 0


def test_jaw_profile_tracks_with_control_loop_slower_than_real_time():
    m = model()
    data = mujoco.MjData(m)
    # Block parked. This is a controller tracking regression, not contact.
    mujoco.mj_resetDataKeyframe(m, data, m.key("default").id)
    opened = m.joint("joint7").range[1]
    data.joint("joint7").qpos = opened
    data.ctrl[6] = opened
    for name in ("gripper_joint1", "gripper_joint2"):
        data.joint(name).qpos = 0.008 * opened
    step(m, data, 100)
    speed, accel, factor = 3.0, 6.0, 0.7
    ramp = speed / accel
    duration = ramp + opened / speed

    def reference(t):
        t = min(t, duration)
        if t < ramp:
            return opened - 0.5 * accel * t * t
        if t < duration - ramp:
            return opened - speed * (t - 0.5 * ramp)
        return 0.5 * accel * (duration - t) ** 2

    start = data.time
    peak_error = 0.0
    while data.time < start + (duration + 0.5) * factor:
        data.ctrl[6] = reference((data.time - start) / factor)
        step(m, data, 1)
        peak_error = max(
            peak_error, abs(reference((data.time - start) / factor) - data.qpos[6])
        )
    assert peak_error < 0.20, f"Peak jaw path error: {peak_error:.6f} rad"
    assert abs(data.qpos[6]) < 0.01
