"""Static joint-torque review: original robot vs scaled robot with replacement servos.
Runs inside Blender on two collections from import_mjcf.py (original at scale 1, target scaled).

Masses: every MJCF body's inertial (mass, CoM) is read from the MJCF. For the target, the old
servos held by each body are removed (ARGS["old_servo_g"] each, at the old servo box centre),
the rest is scaled by s^3 (printed parts), and the new servos are added (ARGS["new_servo_g"],
at the SCS0009 body centre).

Torque per joint = gravity moment about the joint axis of:
  * swing (default): the joint's child subtree
  * stance (leg joints, foot on the ground): everything on the trunk side of the joint
For each pose the max |torque| per joint is reported, as a % of each servo's limit.

ARGS = {"mjcf": "/abs/robot.xml", "original": "xgoduck_original", "target": "duck_scs0009",
        "servos_json": "/abs/original_servos.json", "child_held": ["head_yaw"], "scale": 0.92,
        "old_servo_g": 22.9, "new_servo_g": 13.2, "old_limit_Nm": 1.04,
        "new_stall_Nm": 0.226, "new_rated_Nm": 0.074,
        "legs": {"left": ["left_hip_yaw", ...], "right": [...]}, "foot": {"left": "ankle_left", ...},
        "poses": {"stand": {...}, "sit": {...}}, "sweep": ["neck_pitch", "head_pitch", ...]}
"""
import json
import math
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Quaternion, Vector

args = globals()["ARGS"]
O, T, S = args["original"], args["target"], args["scale"]
G = Vector((0, 0, -9.81))
MM = 0.001

inert = {}
for b in ET.parse(args["mjcf"]).getroot().iter("body"):
    i = b.find("inertial")
    if i is not None:
        inert[b.get("name")] = (float(i.get("mass")), Vector(map(float, (i.get("pos") or "0 0 0").split())))


def bodies(c):
    return {o.name.split(":", 1)[1]: o for o in bpy.data.collections[c].objects if o.type == "EMPTY" and o.name.startswith(c + ":")}


bo, bt = bodies(O), bodies(T)
jo = {e["joint"]: e for e in bo.values() if "joint" in e}
jt = {e["joint"]: e for e in bt.values() if "joint" in e}
q0 = {e.name: e.rotation_quaternion.copy() for e in list(bo.values()) + list(bt.values())}
mesh_owner = {o.name.split(":", 1)[1].split(".")[0]: o.parent for o in bpy.data.collections[O].objects if o.type == "MESH"}

# ---- old servos: holder body + box centre in holder frame (from the original at rest)
bpy.context.view_layer.update()
old_servos = []
for r in json.load(open(args["servos_json"])):
    j = r["joint"]
    holder = mesh_owner.get(r["link"].split(".")[0]) or (jo[j] if j in args.get("child_held", []) else jo[j].parent)
    old_servos.append((holder.name.split(":", 1)[1], holder.matrix_world.inverted() @ Vector(r["centre"])))
new_servos = [(o["holder_body"].split(":", 1)[1], o) for o in bpy.data.collections[T].objects if "servo_joint" in o]
NEW_C = Vector(((-5.95 + 17.35) / 2, 0, -25.25 / 2)) * MM          # SCS0009 body centre, servo frame

mo, mn = args["old_servo_g"] / 1000, args["new_servo_g"] / 1000


def point_masses(which):
    """[(body_name, mass, world CoM)] for the current pose."""
    out = []
    B = bo if which == "o" else bt
    for name, e in B.items():
        if name not in inert:
            continue
        m, c = inert[name]
        mw = e.matrix_world
        if which == "o":
            out.append((name, m, mw @ c))
            continue
        # target: remove old servos held here (in original body frame), scale the rest, add new
        olds = [lc for h, lc in old_servos if h == name]
        m_rest = m - mo * len(olds)
        c_rest = (c * m - sum((lc * mo for lc in olds), Vector())) / m_rest if olds else c
        out.append((name, m_rest * S ** 3, mw @ (c_rest * S)))
    if which == "t":
        for h, s in new_servos:
            out.append((h, mn, s.matrix_world @ NEW_C))
    return out


def subtree(e):
    return {e.name.split(":", 1)[1]} | {o.name.split(":", 1)[1] for o in e.children_recursive if o.type == "EMPTY" and o.name.split(":", 1)[0] == e.name.split(":", 1)[0]}


def torques(which, stance_leg=None):
    J = jo if which == "o" else jt
    pm = point_masses(which)
    out = {}
    for jn, e in J.items():
        p = e.matrix_world.translation
        a = (e.matrix_world.to_3x3() @ Vector(e["joint_axis"])).normalized()
        sub = subtree(e)
        leg = next((side for side, js in args["legs"].items() if jn in js), None)
        use_stance = stance_leg is not None and leg == stance_leg
        sel = [x for x in pm if (x[0] not in sub) == use_stance]
        out[jn] = abs(sum(((c - p).cross(G * m)).dot(a) for _, m, c in sel))
    return out


def pose(q):
    for B in (bo, bt):
        for e in B.values():
            e.rotation_quaternion = q0[e.name]
    for name, ang in q.items():
        for J in (jo, jt):
            e = J[name]
            e.rotation_quaternion = q0[e.name] @ Quaternion(Vector(e["joint_axis"]), ang)
    bpy.context.view_layer.update()


worst = {}                                      # joint -> (orig Nm, new Nm, label)


def record(label, q, stance=None):
    pose(q)
    to, tn = torques("o", stance), torques("t", stance)
    for j in to:
        if j not in worst or tn[j] > worst[j][1]:
            worst[j] = (to[j], tn[j], label)


for name, q in args["poses"].items():
    record(f"{name} (both feet ~ half)", q)
    for side in args["legs"]:
        record(f"{name}, standing on {side}", q, side)
for jn in args.get("sweep", []):                # gravity-loaded upper joints: worst angle in range
    lo, hi = jo[jn]["joint_range"]
    for k in range(25):
        a = lo + (hi - lo) * k / 24
        record(f"{jn} at {math.degrees(a):+.0f}°", {jn: a})
pose({})

mass_o = sum(m for _, m, _ in point_masses("o"))
mass_t = sum(m for _, m, _ in point_masses("t"))
print(f"total mass: original {mass_o * 1000:.0f} g -> new {mass_t * 1000:.0f} g")
L0, LS, LR = args["old_limit_Nm"], args["new_stall_Nm"], args["new_rated_Nm"]
print(f"{'joint':16} {'orig N·m':>8} {'%lim':>5} | {'new N·m':>8} {'%stall':>6} {'%rated':>6}  worst case")
for j in sorted(worst, key=lambda j: -worst[j][1]):
    a, b, lab = worst[j]
    flag = "  !! > stall" if b > LS else ("  ! > rated" if b > LR else "")
    print(f"{j:16} {a:8.3f} {100 * a / L0:4.0f}% | {b:8.3f} {100 * b / LS:5.0f}% {100 * b / LR:5.0f}%  {lab}{flag}")
