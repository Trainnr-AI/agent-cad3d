"""Find the smallest UNIFORM scale at which the replacement servos fit without colliding.
Runs inside Blender. Keeps the original proportions exactly (every body offset = s x original).

For each horn radius in ARGS["horn_radii_mm"], binary-searches s and reports the limiting pairs
(which servo boxes - body / ears / horn - touch at the limit). Leaves the target posed at the
last s searched; use ARGS["apply"] = {"s": .., "horn_r_mm": ..} to set a final layout.

ARGS = {"lib": "/abs/tools/blender/servo_geom.py", "original": "xgoduck_original", "target": "duck_scs0009", "clearance_mm": 1.0,
        "horn_radii_mm": [12.3, 9.0, 6.5], "lo": 0.6, "hi": 1.2, "apply": None}
"""
import itertools

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
exec(open(args["lib"]).read())       # scs_shapes, overlap
O, T = args["original"], args["target"]
CL = args.get("clearance_mm", 1.0) / 2000
MM = 0.001
orig = {o.name.split(":", 1)[1]: o for o in bpy.data.collections[O].objects if o.type == "EMPTY" and o.name.startswith(O + ":")}
tgt = {o.name.split(":", 1)[1]: o for o in bpy.data.collections[T].objects if o.type == "EMPTY" and o.name.startswith(T + ":")}
servos = [o for o in bpy.data.collections[T].objects if o.type == "EMPTY" and "M_joint_local" in o]
root_name = next(n for n, o in orig.items() if o.parent is None)
root_z0 = orig[root_name].location.z


def M_local(s):
    v = list(s["M_joint_local"])
    return Matrix([v[0:4], v[4:8], v[8:12], v[12:16]])


def set_scale(s):
    for n, o in tgt.items():
        if n in orig:
            loc = orig[n].location * s
            if orig[n].parent is None:
                loc = Vector((tgt[n].location.x, tgt[n].location.y, root_z0 * s))
            o.location = loc
    bpy.context.view_layer.update()


def obbs(s, hr):
    W = bpy.data.objects[s["joint_body"]].matrix_world @ M_local(s)
    return scs_shapes(W, hr, args.get("clearance_mm", 1.0))


def collisions(hr):
    hits = []
    for s1, s2 in itertools.combinations(servos, 2):
        for b1 in obbs(s1, hr):
            for b2 in obbs(s2, hr):
                d = overlap(b1[1], b2[1])
                if d:
                    hits.append((d, f"{s1.name[4:]}.{b1[0]} x {s2.name[4:]}.{b2[0]}"))
    return sorted(hits, reverse=True)


for hr in args.get("horn_radii_mm", [12.3]):
    lo, hi = args.get("lo", 0.6), args.get("hi", 1.2)
    set_scale(hi)
    if collisions(hr):
        print(f"horn r {hr} mm: still colliding at s={hi}")
        continue
    for _ in range(20):
        mid = (lo + hi) / 2
        set_scale(mid)
        if collisions(hr):
            lo = mid
        else:
            hi = mid
    set_scale(lo)
    lim = collisions(hr)
    print(f"horn r {hr:4.1f} mm: smallest uniform scale s = {hi:.3f}; limiting pairs just below it:")
    for d, pair in lim[:4]:
        print(f"     {d * 1000:5.2f} mm  {pair}")

if args.get("apply"):
    set_scale(args["apply"]["s"])
    for s in servos:
        s.matrix_world = bpy.data.objects[s["joint_body"]].matrix_world @ M_local(s)
    left = collisions(args["apply"]["horn_r_mm"])
    print(f"applied s={args['apply']['s']}: {len(left)} colliding box pairs")
