#!/usr/bin/env python3
"""Build a uniformly scaled copy of an MJCF robot with a different servo, plus its scene.

Geometry: every body/geom/site/inertial position and every mesh is scaled by --scale; joint
axes, orientations and ranges are unchanged (motion is identical, only smaller).
Mass: per body, the old servos it holds are removed, the rest scales by scale^3 except a
fixed (non-scaling) part such as batteries and boards, and the new servos are added.
Inertia scales by (new mass / old mass) * scale^2.
Servo model (same structure for any servo, so robots can be compared like for like):
  position actuator, kp = stall / sat_err (full torque at sat_err of position error),
  forcerange = ±stall, joint damping = stall / no_load_speed (linear torque-speed line),
  armature (reflected rotor inertia) and frictionloss as given.

  python3 tools/sim/build_scaled_mjcf.py --src robot_allcollisions.xml --scene scene.xml \
      --out projects/xgoduck_scs0009/sim --name scs0009 --scale 0.92 --servo scs0009
"""
import argparse
import json
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET

SERVOS = {
    # stall N·m, no-load rad/s, armature kg·m², frictionloss N·m, mass g, source
    "hls1910": dict(stall=1.04, w0=9.5, armature=0.0018, friction=0.012, mass_g=22.9,
                    source="XGO-Duck sim model chosen_actuator (BAM HLS1910 fit); mass from neck link"),
    "scs0009": dict(stall=0.226, w0=10.47, armature=0.0015, friction=0.010, mass_g=13.2,
                    source="FEETECH SCS0009 spec A/0 @6V (stall, no-load speed, mass); "
                           "armature and friction ESTIMATED - identify on hardware"),
}
# servos held by each body of the XGO-Duck (from the CAD parts list); mouth servo is in the head
SERVOS_PER_BODY = {"trunk_base": 2, "yaw2roll": 1, "bearing_roll": 1, "upper_leg_left": 2, "upper_leg_right": 2,
                   "leg": 1, "leg_2": 1, "neck": 2, "yaw_roll_motion": 1, "jaw_soft": 2}
# mass that does not shrink with the print (battery + boards in the trunk; camera/speaker/etc. in the head), g
FIXED_G = {"trunk_base": 120.0, "jaw_soft": 40.0}
SAT_ERR = 0.052          # rad (3°) of position error for full torque


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


def scale_attr(el, attr, s):
    if el.get(attr):
        el.set(attr, fmt([float(x) * s for x in el.get(attr).split()]))


def root_iter_bodies(path):
    return ET.parse(path).getroot().iter("body")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--servo", default="scs0009", choices=SERVOS)
    ap.add_argument("--old-servo", default="hls1910", choices=SERVOS)
    ap.add_argument("--head-scale", type=float, default=1.0,
                    help="extra scale for the head shell only (bodies in --head-bodies), about the head joint; "
                         "servos and fixed electronics keep their mass")
    ap.add_argument("--head-bodies", default="jaw_soft")
    ap.add_argument("--sat-err-deg", type=float, default=3.0,
                    help="position error at which the servo reaches full torque (sets kp = stall / error); "
                         "Feetech firmware P gain is adjustable, so this is a tuning choice")
    a = ap.parse_args()
    s, new, old = a.scale, SERVOS[a.servo], SERVOS[a.old_servo]
    head = set(a.head_bodies.split(","))
    hs = a.head_scale
    head_meshes = {g.get("mesh") for b in root_iter_bodies(a.src) if b.get("name") in head for g in b.findall("geom") if g.get("mesh")}
    os.makedirs(os.path.join(a.out, "assets"), exist_ok=True)

    tree = ET.parse(a.src)
    root = tree.getroot()
    root.set("model", f"xgoduck_{a.name}")
    src_assets = os.path.join(os.path.dirname(a.src), root.find("compiler").get("meshdir", "assets"))

    for m in root.iter("mesh"):
        shutil.copy2(os.path.join(src_assets, m.get("file")), os.path.join(a.out, "assets", m.get("file")))
        mname = m.get("name") or os.path.splitext(os.path.basename(m.get("file")))[0]
        k = s * (hs if mname in head_meshes else 1.0)
        m.set("scale", fmt([k, k, k]))

    report = []
    for b in root.iter("body"):
        scale_attr(b, "pos", s)                          # joint layout: robot scale only
        kb = s * (hs if b.get("name") in head else 1.0)  # things inside the head body also shrink with it
        for tag in ("geom", "site", "camera"):
            for g in b.findall(tag):
                scale_attr(g, "pos", kb)
                if tag == "geom" and g.get("size") and g.get("type") != "mesh":
                    scale_attr(g, "size", s)
        i = b.find("inertial")
        if i is None:
            continue
        kb = s * (hs if b.get("name") in head else 1.0)
        scale_attr(i, "pos", kb)
        m0 = float(i.get("mass"))
        n = SERVOS_PER_BODY.get(b.get("name"), 0)
        fixed = FIXED_G.get(b.get("name"), 0.0) / 1000
        rest = m0 - n * old["mass_g"] / 1000 - fixed
        m1 = rest * kb ** 3 + fixed + n * new["mass_g"] / 1000
        i.set("mass", f"{m1:.9g}")
        if i.get("fullinertia"):
            i.set("fullinertia", fmt([float(x) * (m1 / m0) * kb * kb for x in i.get("fullinertia").split()]))
        report.append((b.get("name"), m0, m1))

    # servo model on the actuator class
    for d in root.iter("default"):
        if d.get("class") == "chosen_actuator":
            j, p = d.find("joint"), d.find("position")
            j.set("damping", f"{new['stall'] / new['w0']:.6g}")
            j.set("frictionloss", f"{new['friction']:.6g}")
            j.set("armature", f"{new['armature']:.6g}")
            p.set("kp", f"{new['stall'] / math.radians(a.sat_err_deg):.6g}")
            p.set("kv", "0")
            p.set("forcerange", f"{-new['stall']:.6g} {new['stall']:.6g}")
    robot_file = f"robot_{a.name}.xml"
    tree.write(os.path.join(a.out, robot_file))

    # scene: same floor/lights; keyframe root heights scale with the robot
    scene = open(a.scene).read().replace(os.path.basename(a.src), robot_file)

    def scale_key(mt):
        vals = mt.group(2).split()
        vals[2] = f"{float(vals[2]) * s:.6g}"
        return f'{mt.group(1)}"{" ".join(vals)}"'
    scene = re.sub(r'(qpos=)"([^"]*)"', scale_key, scene, flags=re.S)
    open(os.path.join(a.out, f"scene_{a.name}.xml"), "w").write(scene)

    total0, total1 = sum(r[1] for r in report), sum(r[2] for r in report)
    meta = {"scale": s, "head_scale": hs, "servo": a.servo, "servo_model": new, "kp": new["stall"] / math.radians(a.sat_err_deg),
            "sat_err_deg": a.sat_err_deg,
            "fixed_mass_g": FIXED_G, "mass_g": {"original": round(total0 * 1000, 1), "new": round(total1 * 1000, 1)},
            "bodies": {n: [round(m0 * 1000, 1), round(m1 * 1000, 1)] for n, m0, m1 in report}}
    json.dump(meta, open(os.path.join(a.out, f"model_{a.name}.json"), "w"), indent=1)
    print(f"{a.name}: scale {s}, head x{hs}, servo {a.servo}: head {[r[2] for r in report if r[0] in head][0] * 1000:.0f} g, mass {total0 * 1000:.0f} g -> {total1 * 1000:.0f} g; "
          f"kp {new['stall'] / math.radians(a.sat_err_deg):.2f}, forcerange ±{new['stall']}, damping {new['stall'] / new['w0']:.4f}")


if __name__ == "__main__":
    main()
