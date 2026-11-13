#!/usr/bin/env python3
"""Put the masses / inertias computed from the built parts (tools/blender/body_mass.py) into a robot MJCF, and move sensor
sites to where the parts now are (sites JSON: {site: [body, [x, y, z]]}, body frame, metres). Edits the robot file in place.

  python3 tools/sim/apply_body_inertia.py --robot sim/robot_scs0009_v2.xml --mass sim/body_mass_v2.json --sites sim/sites_v2.json
"""
import argparse
import json
import xml.etree.ElementTree as ET


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--mass", required=True)
    ap.add_argument("--sites")
    a = ap.parse_args()
    tree = ET.parse(a.robot)
    root = tree.getroot()
    bodies = {b.get("name"): b for b in root.iter("body")}
    data = json.load(open(a.mass))["bodies"]
    before = after = 0.0
    for name, d in data.items():
        b = bodies[name]
        inert = b.find("inertial")
        before += float(inert.get("mass"))
        after += d["mass"]
        print(f"  {name:16} {float(inert.get('mass')) * 1000:7.1f} -> {d['mass'] * 1000:7.1f} g")
        inert.attrib.clear()
        inert.set("pos", fmt(d["pos"]))
        inert.set("mass", f"{d['mass']:.9g}")
        inert.set("fullinertia", fmt(d["fullinertia"]))
    missing = [n for n in bodies if n not in data]
    print(f"total {before * 1000:.1f} -> {after * 1000:.1f} g; bodies left as they were: {missing}")
    if a.sites:
        for site, (bname, pos) in json.load(open(a.sites)).items():
            el, parent = None, None
            for b in root.iter("body"):
                for s in b.findall("site"):
                    if s.get("name") == site:
                        el, parent = s, b
            if el is None:
                el = ET.SubElement(bodies[bname], "site", name=site, group="3")
            elif parent is not bodies[bname]:
                parent.remove(el)
                bodies[bname].append(el)
            el.set("pos", fmt(pos))
            el.attrib.pop("quat", None) if site != "mouth_tip" else None
            print(f"  site {site:12} -> {bname} {fmt([p * 1000 for p in pos])} mm")
    tree.write(a.robot)


if __name__ == "__main__":
    main()
