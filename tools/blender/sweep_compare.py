"""Sweep every joint through its range on two robots and compare servo self-collisions.
Runs inside Blender. The goal: the new robot must not collide anywhere the original doesn't.

Original servos: boxes from the servos JSON (find_servos.py / fit_servo_boxes.py), rigidly
attached to the body that holds them (parts list) - posed with that body.
New servos: SCS0009 roots from place_servos.py (already parented to their holder body), as
body / ears / swept-horn boxes (same as solve_layout.py).
Poses: each joint alone at N samples across its range (others at zero), plus named keyframes.

ARGS = {"lib": "/abs/tools/blender/servo_geom.py", "original": "xgoduck_original", "target": "duck_scs0009", "json": "/abs/original_servos.json",
        "holder_mesh": {...fitted joints...}, "child_held": ["head_yaw"], "samples": 9,
        "horn_r_mm": 12.3, "clearance_mm": 0.0, "keyframes": {"stand": {joint: rad}, ...},
        "range_limits": {"head_yaw": [-2.618, 2.618]}}   # hardware limits for the new robot
"""
import itertools
import json
import math

import bpy
from mathutils import Matrix, Quaternion, Vector

args = globals()["ARGS"]
exec(open(args["lib"]).read())       # scs_shapes, overlap
O, T = args["original"], args["target"]
MM = 0.001
CL = args.get("clearance_mm", 0.0) / 2000
HR = args.get("horn_r_mm", 12.3) * MM
bpy.context.view_layer.update()


def joints(c):
    return {e["joint"]: e for e in bpy.data.collections[c].objects
            if e.type == "EMPTY" and "joint" in e and e.name.startswith(c + ":")}


jo, jt = joints(O), joints(T)
q0 = {e.name: e.rotation_quaternion.copy() for e in list(jo.values()) + list(jt.values())}

# ---- original servo boxes, attached to their holder bodies
mesh_owner = {o.name.split(":", 1)[1].split(".")[0]: o.parent for o in bpy.data.collections[O].objects if o.type == "MESH"}
orig_boxes = []
for r in json.load(open(args["json"])):
    j = r["joint"]
    holder = mesh_owner.get(r["link"].split(".")[0]) or (jo[j] if j in args.get("child_held", []) else jo[j].parent)
    W = Matrix.Translation(Vector(r["centre"])) @ Matrix((r["long_axis"], r["wide_axis"], r["shaft_axis"])).transposed().to_4x4()
    orig_boxes.append((j, holder, holder.matrix_world.inverted() @ W,
                       Vector((r["long_mm"], r["wide_mm"], r["shaft_len_mm"])) * MM / 2))

new_servos = [o for o in bpy.data.collections[T].objects if "servo_joint" in o]


def orig_obbs():
    out = []
    for j, h, L, half in orig_boxes:
        W = h.matrix_world @ L
        R = W.to_3x3().normalized()
        out.append((j, ("box", W.translation.copy(), [R.col[0], R.col[1], R.col[2]], half + Vector((CL, CL, CL)))))
    return out


def new_obbs():
    return [(f"{s['servo_joint']}.{tag}", shp) for s in new_servos
            for tag, shp in scs_shapes(s.matrix_world, args.get("horn_r_mm", 12.3), args.get("clearance_mm", 0.0))]


def hits(boxes):
    out = []
    for (n1, b1), (n2, b2) in itertools.combinations(boxes, 2):
        if n1.split(".")[0] == n2.split(".")[0]:
            continue
        d = overlap(b1, b2)
        if d:
            out.append((d, f"{n1} x {n2}"))
    return out


def pose(q):
    for name, ang in q.items():
        for J in (jo, jt):
            e = J[name]
            e.rotation_quaternion = q0[e.name] @ Quaternion(Vector(e["joint_axis"]), ang)
    bpy.context.view_layer.update()


def zero():
    for J in (jo, jt):
        for e in J.values():
            e.rotation_quaternion = q0[e.name]
    bpy.context.view_layer.update()


limits = args.get("range_limits", {})
poses = []
N = args.get("samples", 9)
for name, e in sorted(jo.items()):
    lo, hi = e["joint_range"]
    for k in range(N):
        ang = lo + (hi - lo) * k / (N - 1)
        poses.append((f"{name} {math.degrees(ang):+6.1f}°", {name: ang}, name, ang))
for kname, q in args.get("keyframes", {}).items():
    poses.append((f"keyframe {kname}", q, None, None))

worse, both, limited = [], 0, []
for label, q, jn, ang in poses:
    zero()
    pose(q)
    ho, hn = hits(orig_obbs()), hits(new_obbs())
    if jn and jn in limits and not (limits[jn][0] - 1e-6 <= ang <= limits[jn][1] + 1e-6):
        limited.append(label)
        continue
    if hn and not ho:
        worse.append((label, max(hn)[0], max(hn)[1]))
    elif hn and ho:
        both += 1
zero()
print(f"{len(poses)} poses swept. New robot collides where the original does not: {len(worse)}")
for label, d, pair in worse:
    print(f"   {label:28}  {d * 1000:5.2f} mm  {pair}")
print(f"poses where both collide (shared with the original design): {both}")
if limited:
    print(f"poses outside the new robot's hardware range (skipped): {len(limited)} -> {', '.join(limited)}")
