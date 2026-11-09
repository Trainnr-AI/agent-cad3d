"""Attach leg parts to their bodies and sweep the leg joints, reporting real collisions.
Runs inside Blender.

Parts are parented (keeping world pose) to the robot body that carries them; servo cases are
already parented to their holder bodies. For every pose, every pair of objects on DIFFERENT
bodies is tested (triangle BVH); hits are confirmed with an exact intersection volume, so parts
that merely touch are not reported.

ARGS = {"robot": "duck_scs0009", "parts": {"left_thigh": "upper_leg_left", ...},
        "servo_cases": ["scs_left_hip_yaw_case", ...], "joints": ["left_hip_yaw", ...], "samples": 9,
        "min_mm3": 0.05}
"""
import math

import bmesh
import bpy
from mathutils import Quaternion, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
R = args["robot"]
body = lambda n: bpy.data.objects[f"{R}:{n}"]
bpy.context.view_layer.update()

for pname, bname in args["parts"].items():
    p, b = bpy.data.objects[pname], body(bname)
    mw = p.matrix_world.copy()
    p.parent = b
    p.matrix_world = mw

objs = [bpy.data.objects[n] for n in list(args["parts"]) + args["servo_cases"]]


def owner(o):
    while o is not None and not o.name.startswith(R + ":"):
        o = o.parent
    return o


own = {o.name: owner(o) for o in objs}
joints = {e["joint"]: e for e in bpy.data.collections[R].objects if e.type == "EMPTY" and "joint" in e and e.name.startswith(R + ":")}
q0 = {e.name: e.rotation_quaternion.copy() for e in joints.values()}


def tree(o):
    mw = o.matrix_world
    return BVHTree.FromPolygons([mw @ v.co for v in o.data.vertices], [p.vertices[:] for p in o.data.polygons])


def inter_volume(a, b):
    tmp = bpy.data.objects.new("_iv_tmp", a.data.copy())
    bpy.context.scene.collection.objects.link(tmp)
    tmp.matrix_world = a.matrix_world
    m = tmp.modifiers.new("i", "BOOLEAN")
    m.operation, m.object, m.solver = "INTERSECT", b, "EXACT"
    with bpy.context.temp_override(object=tmp, active_object=tmp):
        bpy.ops.object.modifier_apply(modifier=m.name)
    bm = bmesh.new()
    bm.from_mesh(tmp.data)
    bm.transform(tmp.matrix_world)
    v = abs(bm.calc_volume()) * 1e9
    bm.free()
    bpy.data.meshes.remove(tmp.data)
    return v


def check(label):
    bpy.context.view_layer.update()
    trees = {o.name: tree(o) for o in objs}
    hits = []
    for i, a in enumerate(objs):
        for b in objs[i + 1:]:
            if own[a.name] == own[b.name]:
                continue
            if trees[a.name].overlap(trees[b.name]):
                v = inter_volume(a, b)
                if v > args.get("min_mm3", 0.05):
                    hits.append(f"{a.name} x {b.name}: {v:.1f} mm3")
    return hits


results = {"rest": check("rest")}
for jn in args["joints"]:
    e = joints[jn]
    lo, hi = e["joint_range"]
    n = args.get("samples", 9)
    for k in range(n):
        ang = lo + (hi - lo) * k / (n - 1)
        e.rotation_quaternion = q0[e.name] @ Quaternion(Vector(e["joint_axis"]), ang)
        h = check(f"{jn} {math.degrees(ang):+.0f}")
        if h:
            results[f"{jn} {math.degrees(ang):+.0f}°"] = h
    e.rotation_quaternion = q0[e.name]
for label, q in args.get("poses", {}).items():          # combined poses: {joint: rad}
    for jn, ang in q.items():
        e = joints[jn]
        e.rotation_quaternion = q0[e.name] @ Quaternion(Vector(e["joint_axis"]), ang)
    h = check(label)
    if h:
        results[label] = h
    for jn in q:
        joints[jn].rotation_quaternion = q0[joints[jn].name]
bpy.context.view_layer.update()

bad = {k: v for k, v in results.items() if v}
print(f"poses checked: 1 + {len(args['joints'])} x {args.get('samples', 9)} + {len(args.get('poses', {}))} combined; poses with real collisions: {len(bad)}")
for k, v in bad.items():
    print(f"  {k}: " + "; ".join(v[:4]) + (f" (+{len(v) - 4} more)" if len(v) > 4 else ""))
