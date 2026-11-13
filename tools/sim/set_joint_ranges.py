#!/usr/bin/env python3
"""Set the joint ranges of a robot MJCF from a ranges file (degrees), and clamp its keyframes into them.

ranges file: {"joints": {"left_knee": [-90, 70], ...}, "servo_torque_nm": 0.1883 (optional: actuator forcerange), "keyframes": {"SIT": {"head_pitch": 80.0}, ...} (optional overrides, deg),
              "notes": {...}}
The keyframes live in the scene file (<key name=... qpos=...>); qpos order = free joint (7) + the robot's hinge joints in
document order. Edits both files in place and prints every change.

  python3 tools/sim/set_joint_ranges.py --robot sim/robot_scs0009_v2.xml --scene sim/scene_scs0009_v2.xml --ranges sim/joint_ranges_v2.json
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True)
    ap.add_argument("--scene")
    ap.add_argument("--ranges", required=True)
    a = ap.parse_args()
    cfg = json.load(open(a.ranges))
    rng = {j: (math.radians(lo), math.radians(hi)) for j, (lo, hi) in cfg["joints"].items()}
    tree = ET.parse(a.robot)
    hinges = []
    for el in tree.getroot().iter("joint"):
        n = el.get("name")
        if n is None or el.get("type", "hinge") != "hinge":
            continue
        hinges.append(n)
        if n in rng:
            old = el.get("range")
            el.set("range", f"{rng[n][0]:.6f} {rng[n][1]:.6f}")
            print(f"  {n:16} {' '.join(f'{math.degrees(float(v)):+7.1f}' for v in old.split()) if old else '-':>17} -> {cfg['joints'][n][0]:+7.1f} {cfg['joints'][n][1]:+7.1f} deg")
    if cfg.get("servo_torque_nm"):                                          # servo supply voltage changed (e.g. 5 V regulator): stall torque
        t = cfg["servo_torque_nm"]
        for el in tree.getroot().iter("position"):
            if el.get("forcerange"):
                el.set("forcerange", f"{-t:.4f} {t:.4f}")
        print(f"  servo torque limit -> {t} N m")
    tree.write(a.robot)
    missing = [j for j in rng if j not in hinges]
    if missing:
        print("  not in the model:", missing)
    if not a.scene:
        return
    st = ET.parse(a.scene)
    for key in st.getroot().iter("key"):
        q = [float(v) for v in key.get("qpos").split()]
        name = key.get("name")
        over = cfg.get("keyframes", {}).get(name, {})
        changes = []
        for i, j in enumerate(hinges):
            k = 7 + i
            v = math.radians(over[j]) if j in over else q[k]
            if j in rng:
                v = min(max(v, rng[j][0]), rng[j][1])
            if abs(v - q[k]) > 1e-9:
                changes.append(f"{j} {math.degrees(q[k]):+.1f} -> {math.degrees(v):+.1f}")
                q[k] = v
        key.set("qpos", " ".join(f"{v:.9g}" for v in q))
        if key.get("ctrl"):
            c = [float(v) for v in key.get("ctrl").split()]
            if len(c) == len(hinges):
                key.set("ctrl", " ".join(f"{v:.9g}" for v in q[7:]) if any(abs(x) > 0 for x in c) else key.get("ctrl"))
        print(f"  key {name}: {'; '.join(changes) if changes else 'unchanged'}")
    st.write(a.scene)


if __name__ == "__main__":
    main()
