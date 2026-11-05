#!/usr/bin/env python3
"""Generate the minimal stock arm's free-space model from its expanded URDF.

The URDF owns geometry, inertias, frames and joint limits. This model retains
that tree and uses the existing B601-RS simulation actuator settings. Visuals
never collide. The separately generated contact fixture reuses the native pad
meshes; contact-rich applications still own their collision decompositions.
"""

import sys
import xml.etree.ElementTree as ET


def generate(urdf_path, output_path):
    robot = ET.parse(urdf_path).getroot()
    model = ET.Element("mujoco", model="rebot_b601_rs")
    model.append(
        ET.Comment(
            " Derived from Seeed reBot-DevArm@58f9e433deec2ba118c5c22d60c07b9897e92acc, "
            "CERN-OHL-W-2.0. Modified by PickNik Inc., 2026: generated from the shared "
            "motor-coordinate URDF with free-space dynamics and native-jaw coupling. "
            "Stock mesh files are referenced without modification. See the installed LICENSE. "
        )
    )
    ET.SubElement(model, "compiler", angle="radian", eulerseq="XYZ", autolimits="true")
    ET.SubElement(model, "option", timestep="0.003", integrator="implicitfast")
    assets = ET.SubElement(model, "asset")
    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 2", dir="0 0 -1", directional="true")
    links = {link.get("name"): link for link in robot.findall("link")}
    joints = robot.findall("joint")
    materials = {
        m.get("name"): m.find("color").get("rgba")
        for m in robot.findall("material")
        if m.find("color") is not None
    }
    actuator = ET.SubElement(model, "actuator")
    equality = ET.SubElement(model, "equality")
    gains = [(50, 3), (150, 10), (150, 10), (50, 5), (50, 4), (50, 4), (12, 0.05)]
    order = []

    def body(parent, joint):
        name = joint.find("child").get("link")
        link = links[name]
        origin = joint.find("origin")
        attrs = {"name": name, "gravcomp": "1"}
        if origin is not None:
            attrs.update(
                pos=origin.get("xyz", "0 0 0"), euler=origin.get("rpy", "0 0 0")
            )
        node = ET.SubElement(parent, "body", **attrs)
        inertial = link.find("inertial")
        if inertial is not None:
            i = inertial.find("inertia")
            io = inertial.find("origin")
            attrs = {"mass": inertial.find("mass").get("value")}
            if io is not None:
                # The source inertias are expressed in unrotated link axes.
                if any(float(v) != 0 for v in io.get("rpy", "0 0 0").split()):
                    raise ValueError(
                        "Rotated inertial frame requires explicit tensor conversion"
                    )
                attrs["pos"] = io.get("xyz", "0 0 0")
            attrs["fullinertia"] = " ".join(
                i.get(k) for k in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]
            )
            ET.SubElement(node, "inertial", **attrs)
        if joint.get("type") != "fixed":
            jname = joint.get("name")
            order.append(jname)
            limit = joint.find("limit")
            axis = joint.find("axis")
            ja = {
                "name": jname,
                "type": "slide" if joint.get("type") == "prismatic" else "hinge",
                "axis": axis.get("xyz"),
                "range": limit.get("lower") + " " + limit.get("upper"),
            }
            if jname.startswith("joint"):
                idx = int(jname[5:]) - 1
                kp, kd = gains[idx]
                ja.update(
                    armature="0.05" if idx < 3 else "0.002" if idx == 6 else "0.01",
                    damping="1" if idx < 3 else "0.02" if idx == 6 else "0.2",
                )
                # Bound the jaw squeeze in simulation to 1 Nm. The nominal
                # 14 Nm URDF bound crushed this fixture's soft contacts.
                effort = 1.0 if idx == 6 else float(limit.get("effort"))
                ET.SubElement(
                    actuator,
                    "position",
                    name=jname,
                    joint=jname,
                    kp=str(kp),
                    kv=str(kd),
                    ctrlrange=ja["range"],
                    forcerange=f"{-effort:g} {effort:g}",
                )
                if inertial is None:
                    # Geometry-free motor coordinate retained from the prior simulator.
                    ET.SubElement(
                        node,
                        "inertial",
                        pos="0 0 0",
                        mass="0.001",
                        diaginertia="0.000001 0.000001 0.000001",
                    )
            else:
                ja["damping"] = "0.5"
                mimic = joint.find("mimic")
                ET.SubElement(
                    equality,
                    "joint",
                    # Express the residual in motor radians rather than tiny
                    # jaw metres. The equivalent inverse relation transmits
                    # contact to the motor instead of stretching the mimic.
                    joint1=mimic.get("joint"),
                    joint2=jname,
                    polycoef=(
                        f"{-float(mimic.get('offset', '0')) / float(mimic.get('multiplier', '1'))} "
                        f"{1 / float(mimic.get('multiplier', '1'))} 0 0 0"
                    ),
                )
            ET.SubElement(node, "joint", **ja)
        for idx, visual in enumerate(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError("The stock arm visual must be a mesh")
            uri = mesh.get("filename")
            if not uri.startswith("package://"):
                raise ValueError("A package URI is required for stock meshes")
            mesh_name = f"{name}_visual_{idx}"
            ET.SubElement(
                assets,
                "mesh",
                name=mesh_name,
                file=uri,
                scale=mesh.get("scale", "1 1 1"),
            )
            attrs = {
                "type": "mesh",
                "mesh": mesh_name,
                "contype": "0",
                "conaffinity": "0",
                "density": "0",
            }
            vo = visual.find("origin")
            if vo is not None:
                attrs.update(pos=vo.get("xyz", "0 0 0"), euler=vo.get("rpy", "0 0 0"))
            mat = visual.find("material")
            if mat is not None and mat.get("name") in materials:
                attrs["rgba"] = materials[mat.get("name")]
            ET.SubElement(node, "geom", **attrs)
        # Match the established motor-first qpos ordering before the passive jaws.
        children = [j for j in joints if j.find("parent").get("link") == name]
        for child in sorted(children, key=lambda j: j.find("mimic") is not None):
            body(node, child)

    body(world, next(j for j in joints if j.find("child").get("link") == "base_link"))
    keyframe = ET.SubElement(model, "keyframe")
    ET.SubElement(
        keyframe,
        "key",
        name="default",
        qpos=" ".join("0" for _ in order),
        ctrl="0 0 0 0 0 0 0",
    )
    ET.indent(model, space="  ")
    ET.ElementTree(model).write(output_path, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    generate(*sys.argv[1:])
