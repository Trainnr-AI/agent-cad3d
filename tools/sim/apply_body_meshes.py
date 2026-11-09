#!/usr/bin/env python3
"""Swap a robot MJCF's per-body meshes for the bodies exported from Blender (export_bodies.py).

Every body that has an exported STL gets it as BOTH its visual and its collision mesh, in the
body frame (geom pos/quat cleared, mesh scale 1). Joints, masses, inertias, actuators, sites and
keyframes are untouched. Writes robot_<name>.xml and scene_<name>.xml next to the source.

  python3 tools/sim/apply_body_meshes.py --robot sim/robot_scs0009_h0.85_e1.xml \
      --scene sim/scene_scs0009_h0.85_e1.xml --meshes sim/assets_v1 --name scs0009_v1
"""
import argparse
import os
import shutil
import xml.etree.ElementTree as ET


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--meshes", required=True)
    ap.add_argument("--name", required=True)
    a = ap.parse_args()
    out_dir = os.path.dirname(os.path.abspath(a.robot))
    tree = ET.parse(a.robot)
    root = tree.getroot()
    meshdir = os.path.join(out_dir, root.find("compiler").get("meshdir", "assets"))
    asset = root.find("asset")
    replaced = []
    for b in root.iter("body"):
        stl = os.path.join(a.meshes, f"{b.get('name')}.stl")
        if not os.path.exists(stl):
            continue
        mname = f"{a.name}_{b.get('name')}"
        shutil.copy2(stl, os.path.join(meshdir, f"{mname}.stl"))
        old = asset.find(f"mesh[@name='{mname}']")
        if old is not None:
            asset.remove(old)
        ET.SubElement(asset, "mesh", name=mname, file=f"{mname}.stl")
        for g in b.findall("geom"):
            if g.get("type") == "mesh":
                g.set("mesh", mname)
                for k in ("pos", "quat"):
                    g.attrib.pop(k, None)
        replaced.append(b.get("name"))
    used = {g.get("mesh") for g in root.iter("geom") if g.get("mesh")}
    for m in list(asset.findall("mesh")):
        if (m.get("name") or os.path.splitext(m.get("file"))[0]) not in used:
            asset.remove(m)
    root.set("model", f"xgoduck_{a.name}")
    robot_out = os.path.join(out_dir, f"robot_{a.name}.xml")
    tree.write(robot_out)
    scene = open(a.scene).read().replace(os.path.basename(a.robot), os.path.basename(robot_out))
    open(os.path.join(out_dir, f"scene_{a.name}.xml"), "w").write(scene)
    print(f"{a.name}: new meshes on {len(replaced)} bodies: {', '.join(replaced)}")


if __name__ == "__main__":
    main()
