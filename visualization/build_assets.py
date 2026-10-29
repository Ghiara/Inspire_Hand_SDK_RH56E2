"""Reproducibly build portable URDF/MJCF from the copied RH56E2 Xacro.

Runtime uses only assets/, with no ROS or sibling repository dependency.
"""
from pathlib import Path
import argparse
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import xacro
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
ACTIVE = ("pinky_joint", "ring_joint", "middle_joint", "index_joint", "thumb_joint2", "thumb_joint1")


def add_wrist_adapter(urdf, converted):
    """Attach the same physical adapter to either hand, within hand_base."""
    # GLB's scene transform already converts CAD millimetres to metres.
    mesh = trimesh.load(ASSETS / "adapters/Wrist_RS485_Adapter.glb", force="scene").to_geometry()
    destination = converted / "wrist_rs485_adapter.obj"
    destination.write_text(mesh.export(file_type="obj", include_texture=False).rstrip() + "\n")
    base = urdf.find("link[@name='hand_base']")
    for kind in ("visual", "collision"):
        element = ET.SubElement(base, kind, name="wrist_rs485_adapter")
        # CAD cylinder axis is +Y, centred at X=Z=25 mm. Rotate it to +Z.
        # Radial screw axes at CAD Y=35.5 mm then coincide with wrist Z=0.
        # The 38 mm spigot occupies Z=-5.5..4.5 mm; its shoulder has a
        # 0.5 mm clearance below the wrist lip at Z=-5 mm.
        ET.SubElement(element, "origin", xyz="-0.025 0.025 -0.0355",
                      rpy=f"{np.pi / 2} 0 0")
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "mesh", filename=destination.relative_to(ASSETS).as_posix(),
                      scale="1 1 1")
        if kind == "visual":
            material = ET.SubElement(element, "material", name="wrist_adapter")
            ET.SubElement(material, "color", rgba="0.32 0.36 0.42 1")


def origin(element):
    node = element.find("origin")
    if node is None:
        return {}
    xyz = node.attrib.get("xyz", "0 0 0")
    rpy = np.fromstring(node.attrib.get("rpy", "0 0 0"), sep=" ")
    xyzw = Rotation.from_euler("xyz", rpy).as_quat()
    return {"pos": xyz, "quat": " ".join(map(str, xyzw[[3, 0, 1, 2]]))}


def build(side):
    source = ASSETS / "inspire_description"
    wrapper = ASSETS / f".{side}.xacro"
    wrapper.write_text(f'''<robot name="rh56e2_{side}" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{source / 'xacro/hands/RH56E2.xacro'}"/>
      <xacro:InspireHandRH56E2 direction="{-1 if side == 'right' else 1}"/>
    </robot>''')
    try:
        # Resolve ROS $(find ...) only in memory; preserve copied sources verbatim.
        macro = (source / "xacro/hands/RH56E2.xacro").read_text().replace(
            "$(find inspire_description)", str(source))
        document = xacro.parse(macro)
        # Expand using a wrapper around an in-memory copy of the macro definition.
        wrapper_text = wrapper.read_text()
        import re
        wrapper_text = re.sub(r'<xacro:include[^>]+/>',
                              lambda _: document.documentElement.toxml().split('>', 1)[1].rsplit('</robot>', 1)[0],
                              wrapper_text)
        document = xacro.parse(wrapper_text)
        xacro.process_doc(document)
        urdf = ET.fromstring(document.toxml())
    finally:
        wrapper.unlink(missing_ok=True)
    converted = ASSETS / "converted" / side
    converted.mkdir(parents=True, exist_ok=True)
    converted_meshes = {}
    for mesh in urdf.iter("mesh"):
        path = Path(mesh.attrib["filename"].removeprefix("file://"))
        scale = np.fromstring(mesh.attrib.get("scale", "1 1 1"), sep=" ")
        key = (str(path), tuple(scale))
        if key not in converted_meshes:
            relative = path.relative_to(source / "meshes/RH56E2")
            name = "_".join(relative.with_suffix("").parts)
            loaded = trimesh.load(path, force="scene").to_geometry()
            loaded.apply_transform(np.diag([*scale, 1]))
            destination = converted / f"{name}.obj"
            loaded.export(destination, include_texture=False)
            converted_meshes[key] = destination.relative_to(ASSETS).as_posix()
        mesh.set("filename", converted_meshes[key])
        mesh.set("scale", "1 1 1")
    # Multiple geometries in one link give a rigid attachment without a joint.
    add_wrist_adapter(urdf, converted)
    ET.indent(urdf)
    ET.ElementTree(urdf).write(ASSETS / f"rh56e2_{side}.urdf", encoding="unicode")
    scene = ET.Element("mujoco", model=f"rh56e2_{side}")
    ET.SubElement(scene, "compiler", angle="radian", autolimits="true", balanceinertia="true")
    ET.SubElement(scene, "option", timestep="0.001", gravity="0 0 0", integrator="implicitfast")
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "global", offwidth="960", offheight="720")
    defaults = ET.SubElement(scene, "default")
    ET.SubElement(defaults, "joint", damping="0.015", armature="0.0002",
                  solreflimit="0.002 1", solimplimit="0.99 0.999 0.0001")
    ET.SubElement(defaults, "geom", contype="1", conaffinity="2", friction="0.8 0.01 0.001",
                  solref="0.005 1", rgba="0.75 0.78 0.83 1")
    asset = ET.SubElement(scene, "asset")
    paths = sorted({m.attrib["filename"] for m in urdf.iter("mesh")})
    mesh_names = {path: f"mesh_{i}" for i, path in enumerate(paths)}
    for path, name in mesh_names.items():
        ET.SubElement(asset, "mesh", name=name, file=path)
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", pos="0 -0.5 0.5", dir="0 1 -1", diffuse="0.9 0.9 0.9")
    ET.SubElement(world, "geom", name="floor", type="plane", size="1 1 0.01", pos="0 0 -0.0455",
                  contype="2", conaffinity="1", rgba="0.15 0.18 0.23 1")
    links = {l.attrib["name"]: l for l in urdf.findall("link")}
    joints = urdf.findall("joint")
    children = {}
    for joint in joints:
        children.setdefault(joint.find("parent").attrib["link"], []).append(joint)
    bodies = {}

    def add_link(name, parent, joint=None):
        body = ET.SubElement(parent, "body", name=name, **(origin(joint) if joint is not None else {}))
        bodies[name] = body
        link = links[name]
        inertial = link.find("inertial")
        if inertial is not None:
            inertia = inertial.find("inertia").attrib
            ET.SubElement(body, "inertial", pos=inertial.find("origin").attrib.get("xyz", "0 0 0"),
                          mass=inertial.find("mass").attrib["value"],
                          fullinertia=" ".join(inertia[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
        if joint is not None and joint.attrib["type"] == "revolute":
            limit = joint.find("limit").attrib
            ET.SubElement(body, "joint", name=joint.attrib["name"], type="hinge",
                          axis=joint.find("axis").attrib["xyz"], range=f"{limit['lower']} {limit['upper']}")
        # Use each original visual mesh once as both visual and convex collision geometry.
        for i, v in enumerate(link.findall("visual")):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            color = v.find("material/color")
            ET.SubElement(body, "geom", name=v.attrib.get("name", f"{name}_{i}"), type="mesh", mesh=mesh_names[mesh.attrib["filename"]],
                          rgba=color.attrib["rgba"] if color is not None else "0.75 0.78 0.83 1", **origin(v))
        for child_joint in children.get(name, []):
            add_link(child_joint.find("child").attrib["link"], body, child_joint)

    add_link("hand_base", world)
    # Sites at the terminal tactile patch centers; coordinates inherited from source URDF.
    for finger in ("pinky", "ring", "middle", "index", "thumb"):
        terminal = f"{finger}_force_sensor_{4 if finger == 'thumb' else 3}"
        ET.SubElement(bodies[terminal], "site", name=f"{finger}_tip", size="0.002", rgba="0.1 0.8 0.9 1")
        ET.SubElement(world, "site", name=f"{finger}_target", pos="0 0 -1", size="0.003", rgba="1 0.3 0.1 0.8")
    equality = ET.SubElement(scene, "equality")
    for joint in joints:
        mimic = joint.find("mimic")
        if mimic is not None:
            ET.SubElement(equality, "joint", joint1=joint.attrib["name"], joint2=mimic.attrib["joint"],
                          polycoef=f"{mimic.attrib.get('offset', '0')} {mimic.attrib.get('multiplier', '1')} 0 0 0",
                          solref="0.002 1")
    actuator = ET.SubElement(scene, "actuator")
    for name in ACTIVE:
        ET.SubElement(actuator, "motor", name=f"motor_{name}", joint=name, ctrlrange="-0.4 0.4", forcerange="-0.4 0.4")
    ET.indent(scene)
    ET.ElementTree(scene).write(ASSETS / f"rh56e2_{side}.xml", encoding="unicode")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="Optional original inspire_description directory to vendor")
    args = parser.parse_args()
    if args.source:
        target = ASSETS / "inspire_description"
        shutil.copytree(args.source / "meshes/RH56E2", target / "meshes/RH56E2", dirs_exist_ok=True)
        shutil.copytree(args.source / "xacro", target / "xacro", dirs_exist_ok=True)
        for name in ("README.md", "package.xml"):
            shutil.copy2(args.source / name, target / name)
    for side in ("right", "left"):
        build(side)
        print(f"Built assets/rh56e2_{side}.urdf and .xml")


if __name__ == "__main__":
    main()
