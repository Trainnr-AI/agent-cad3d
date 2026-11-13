"""One line per servo: is it built into its joint the right way round? Runs inside Blender at the zero pose.

For every SCS0009 root (scs_<joint>) it reports
  axis      angle between the servo's shaft (+Z) and the joint axis, and the distance between the two lines (both should be ~0)
  sense     whether a positive joint angle turns the shaft the servo's positive way (+1) or the negative way (-1)
  holder    the printed part(s) the servo's case sits in, and the robot body they belong to
  output    the printed part the horn sits in (largest overlap with the horn), and its body
  bodies    holder body and output body must be the two bodies the joint connects (in either order)
  travel    joint range vs the servo's 300 deg travel, with the horn's zero in the middle
  horn fit  how the horn sits in its pocket: crossing triangles with the output part (press fit expected) and with the holder (none)
The beak servo (scs_mouth) has no robot joint: its output is checked against the jaw's pivot.

ARGS = {"robot": "duck_scs0009", "parts_collections": ["leg_v2", "neck_v2", "head_v2", "head_shell_v2", "body_v2"], "out": "/abs/servo_review.json"}
"""
import json
import math

import bmesh
import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
R = args.get("robot", "duck_scs0009")
bodies = {o.name: o for o in bpy.data.collections[R].objects if o.type == "EMPTY" and o.name.startswith(R + ":")}
joints = {o["joint"]: o for o in bodies.values() if "joint" in o}
parts = [o for c in args["parts_collections"] for o in bpy.data.collections[c].all_objects
         if o.type == "MESH" and "mass_g_solid_PLA" in o.keys() and not o.name.endswith((".x", ".hx", ".rx"))]


def body_of(o):
    p = o.parent
    while p is not None and p.name not in bodies:
        p = p.parent
    return p.name.split(":", 1)[1] if p is not None else None


def tree(o):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bm.transform(o.matrix_world)
    t = BVHTree.FromBMesh(bm)
    P = np.array([v.co[:] for v in bm.verts])
    bm.free()
    return t, P


def near(o, cand, d_max=0.0015):
    """parts within d_max (m) of o's surface, sorted by how many of o's vertices are that close"""
    to, Po = tree(o)
    lo, hi = Po.min(0) - d_max, Po.max(0) + d_max
    res = []
    for c in cand:
        B = np.array([(c.matrix_world @ Vector(b))[:] for b in c.bound_box])
        if (B.min(0) > hi).any() or (B.max(0) < lo).any():
            continue
        tc, _ = tree(c)
        n = sum(1 for p in Po[:: max(1, len(Po) // 1500)] if tc.find_nearest(Vector(p))[3] < d_max)
        cross = len(to.overlap(tc))
        if n or cross:
            res.append((c, n, cross))
    return sorted(res, key=lambda t: -(t[1] + t[2]))


rows = []
for name in sorted([n for n in bpy.data.objects.keys() if n.startswith("scs_") and bpy.data.objects[n].type == "EMPTY" and n.count("_") >= 1]):
    root = bpy.data.objects[name]
    jn = name[4:]
    case, horn = bpy.data.objects.get(name + "_case"), bpy.data.objects.get(name + "_horn_cross")
    if case is None or horn is None:
        continue
    zs = (root.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
    o_s = root.matrix_world.translation
    row = {"servo": name, "joint": jn}
    if jn in joints:
        e = joints[jn]
        za = (e.matrix_world.to_3x3() @ Vector(e["joint_axis"])).normalized()
        o_a = e.matrix_world.translation
        row["axis_deg"] = round(math.degrees(math.acos(max(-1.0, min(1.0, abs(zs.dot(za)))))), 2)
        row["axis_off_mm"] = round((o_s - o_a).cross(za).length * 1000, 2)
        row["sense"] = 1 if zs.dot(za) > 0 else -1
        lo, hi = (math.degrees(a) for a in e["joint_range"])
        row["range_deg"] = [round(lo, 1), round(hi, 1)]
        row["travel_ok"] = max(abs(lo), abs(hi)) <= 150.0
        child = e.name.split(":", 1)[1]
        parent = e.parent.name.split(":", 1)[1] if e.parent and e.parent.name in bodies else None
        row["joint_bodies"] = [parent, child]
    else:
        piv = bpy.data.objects.get("head_jaw_pivot")
        za = (piv.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized() if piv else zs
        row["axis_deg"] = round(math.degrees(math.acos(max(-1.0, min(1.0, abs(zs.dot(za)))))), 2)
        row["axis_off_mm"] = round((o_s - piv.matrix_world.translation).cross(za).length * 1000, 2) if piv else None
        row["sense"] = 1 if zs.dot(za) > 0 else -1
        row["range_deg"] = "jaw (not a robot joint)"
        row["travel_ok"] = True
        row["joint_bodies"] = ["jaw_soft", "jaw (head_jaw_pivot)"]
    hold = near(case, parts)
    outp = near(horn, parts, 0.0006)
    row["holder"] = [(c.name, body_of(c), n, x) for c, n, x in hold[:3]]
    row["output"] = [(c.name, body_of(c), n, x) for c, n, x in outp[:3]]
    hb = hold[0][0] if hold else None
    ob = outp[0][0] if outp else None
    row["holder_body"], row["output_body"] = (body_of(hb) if hb else None), (body_of(ob) if ob else None)
    if jn in joints:
        row["bodies_ok"] = bool(hb and ob) and {row["holder_body"], row["output_body"]} == set(row["joint_bodies"])
    else:
        row["bodies_ok"] = bool(ob) and ob.name == "head_jaw" and row["holder_body"] == "jaw_soft"
    row["horn_in_output_crossing"] = outp[0][2] if outp else 0
    row["horn_vs_holder_crossing"] = next((x for c, n, x in outp if hb and c.name == hb.name), 0)
    rows.append(row)
json.dump(rows, open(args["out"], "w"), indent=1)
print(f"{'servo':22} {'axis':>6} {'off mm':>6} {'sense':>5} {'range deg':>15}  {'holder (body)':36} {'output (body)':36} ok")
for r in rows:
    h = f"{r['holder'][0][0]} ({r['holder_body']})" if r["holder"] else "-"
    o = f"{r['output'][0][0]} ({r['output_body']})" if r["output"] else "-"
    rg = r["range_deg"] if isinstance(r["range_deg"], str) else f"{r['range_deg'][0]}..{r['range_deg'][1]}"
    ok = "ok" if (r["bodies_ok"] and r["axis_deg"] < 0.5 and (r["axis_off_mm"] or 0) < 0.3 and r["travel_ok"]) else "CHECK"
    print(f"{r['servo']:22} {r['axis_deg']:6.2f} {r['axis_off_mm'] or 0:6.2f} {r['sense']:5d} {rg:>15}  {h[:36]:36} {o[:36]:36} {ok}")
