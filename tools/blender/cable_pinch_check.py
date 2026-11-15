"""Cables against moving parts: as the joints move, is any cable forced through a part? Runs inside Blender (best headless:
it rebuilds the `wiring` collection exactly as wiring.py does, then poses the robot).

  blender -b projects/xgoduck_scs0009/xgoduck_scs0009.blend --python tools/blender/cable_pinch_check.py -- args.json

ARGS = {"wiring": "/abs/tools/blender/wiring.py", "wiring_args": {... the wiring.json build args ...},
        "ranges": "/abs/projects/xgoduck_scs0009/sim/joint_ranges_v2.json", "samples": 13, "depth_mm": 0.5,
        "out": "/abs/result.json"}

Model: wiring.py routes each cable through waypoints, each riding on a part ("owner"). For a pose, every span between two
consecutive waypoints is taken as the STRAIGHT line between them (the least cable that connects them - the same model as
wiring.py's reach check); points 1 mm apart along it (2 mm kept clear at each end, where the cable leaves a plug or a clip) are
tested against every printed part, servo case, bearing, screw and board. A span "is forced through" a part when its points sit
deeper than depth_mm inside it. Spans already inside a part at rest (a cable passing through a slot drawn tight) are judged
only on how much deeper they go. A real cable has slack and bends round a part where the straight line would cross it: a hit
means the cable is pushed by that part there (pinch / chafe risk) - look at it - not that it is cut.
Poses: rest, every joint alone over its servo limits (sim/joint_ranges_v2.json), the STAND / SIT keyframes, walking-range
combinations of both legs, neck + head extremes. The robot is put back at rest at the end, also on an error.
"""
import json
import math
import sys
import time

import bpy
import numpy as np
from mathutils import Quaternion, Vector
from mathutils.bvhtree import BVHTree

args = globals().get("ARGS")
if args is None:
    args = json.load(open(sys.argv[sys.argv.index("--") + 1]))
R = args.get("robot", "duck_scs0009")
DEPTH = args.get("depth_mm", 0.5) / 1000
STEP, END = 0.001, 0.002
SKIPC = {"duck_v2_exploded", "wiring_exploded", "xgoduck_original", "dimensions", "orig_head_parts", "coupon", "wiring",
         "wiring_mesh", "cables", "print_tools", "bay_exploded", "head_exploded", "wiring_bay_exploded"}
RAYS = [Vector(d).normalized() for d in ((0.267, 0.534, 0.802), (-0.577, 0.577, 0.577), (0.408, -0.816, 0.408))]

W = {"ARGS": args["wiring_args"], "__name__": "__main__"}            # rebuild the wiring: its routes are what we test
exec(compile(open(args["wiring"]).read(), args["wiring"], "exec"), W)
routes = W["routes"]                                                    # [(name, [(owner, world point at rest)], limit)]
local = [(n, [(o, bpy.data.objects[o].matrix_world.inverted() @ p) for o, p in pts]) for n, pts, _ in routes]

bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()


def owner(o):
    while o is not None and not o.name.startswith(R + ":"):
        o = o.parent
    return o


class Part:
    def __init__(s, o, body):
        s.name, s.body = o.name, body
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        me.calc_loop_triangles()
        M = np.array(body.matrix_world.inverted() @ o.matrix_world)
        v = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", v)
        v = v.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
        t = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
        me.loop_triangles.foreach_get("vertices", t)
        ev.to_mesh_clear()
        s.tree = BVHTree.FromPolygons([Vector(p) for p in v], t.reshape(-1, 3).tolist())
        s.lo, s.hi = v.min(axis=0), v.max(axis=0)

    def depth(s, p):
        votes = 0
        for d in RAYS:
            n, o = 0, p.copy()
            for _ in range(64):
                loc = s.tree.ray_cast(o, d)[0]
                if loc is None:
                    break
                n += 1
                o = loc + d * 1e-7
            votes += n % 2
        if votes < 2:
            return 0.0
        loc, nrm, _, dist = s.tree.find_nearest(p)                     # and the nearest face must face away from the point:
        return dist if (p - loc).dot(nrm) < 0 else 0.0                  # ray parity alone was fooled by the exact servo meshes


parts = []
for o in bpy.data.objects:
    if o.type != "MESH" or not o.users_collection or {c.name for c in o.users_collection} & SKIPC:
        continue
    if o.name.startswith(R + ":") or not o.visible_get() or "_horn_" in o.name or o.name.endswith("_spline"):
        continue
    b = owner(o)
    if b is not None:
        parts.append(Part(o, b))

joints = {e["joint"]: e for e in bpy.data.collections[R].objects if e.type == "EMPTY" and "joint" in e and e.name.startswith(R + ":")}
q0 = {n: e.rotation_quaternion.copy() for n, e in joints.items()}
rng = json.load(open(args["ranges"]))


def set_pose(q):
    for n, e in joints.items():
        e.rotation_quaternion = q0[n] @ Quaternion(Vector(e["joint_axis"]), math.radians(q.get(n, 0.0)))
    bpy.context.view_layer.update()


poses = [("rest", {})]
n = args.get("samples", 13)
for j, (lo, hi) in rng["joints"].items():
    if j in joints:
        poses += [(f"{j} {lo + (hi - lo) * k / (n - 1):+.0f}", {j: lo + (hi - lo) * k / (n - 1)}) for k in range(n)]
for key in ("STAND", "SIT"):
    if key in rng.get("keyframes", {}):
        poses.append((key, {j: v for j, v in rng["keyframes"][key].items() if j in joints}))
for s_ in (1, -1):                                                      # walking-range combinations, both legs (left signs)
    for yaw in (-6.3, 12.6):
        for roll in (-10.9, 10.9):
            for p_ in (-45.8, 45.8):
                q = {}
                for side, m in (("left", 1), ("right", -1)):
                    q.update({f"{side}_hip_yaw": m * yaw, f"{side}_hip_roll": m * roll, f"{side}_hip_pitch": m * s_ * p_,
                              f"{side}_knee": -m * s_ * max(-55.0, min(55.0, p_ * 1.2)),
                              f"{side}_ankle": m * s_ * max(-23.0, min(23.0, p_ * 0.5))})
                poses.append((f"walk yaw {yaw} roll {roll} pitch {s_ * p_}", q))
for npz in (-80.0, 0.0, 30.0):                                          # neck + head extremes inside the servo limits
    for hp in (-5.0, 65.0):
        for hy in (-150.0, 150.0):
            poses.append((f"neck {npz} head pitch {hp} yaw {hy}", {"neck_pitch": npz, "head_pitch": hp, "head_yaw": hy}))


def spans_world():
    out = []
    for name, pts in local:
        w = [bpy.data.objects[o].matrix_world @ p for o, p in pts]
        for i, (a, b) in enumerate(zip(w, w[1:])):
            L = (b - a).length
            if L <= 2 * END:
                continue
            k = max(1, int((L - 2 * END) / STEP))
            out.append(((name, i), np.array([a + (b - a) * ((END + (L - 2 * END) * t / k) / L) for t in range(k + 1)])))
    return out


def measure():
    mats = {p.body.name: np.array(p.body.matrix_world.inverted()) for p in parts}
    res = {}
    for key, P in spans_world():
        for p in parts:
            M = mats[p.body.name]
            Q = P @ M[:3, :3].T + M[:3, 3]
            sel = Q[np.all((Q >= p.lo) & (Q <= p.hi), axis=1)]
            if not len(sel):
                continue
            d = max(p.depth(Vector(x)) for x in sel)
            if d > 0:
                res[(key, p.name)] = d
    return res


t0 = time.time()
hits = {}
try:
    set_pose({})
    rest = measure()
    for label, q in poses[1:]:
        set_pose(q)
        for (key, part), d in measure().items():
            base = rest.get((key, part), 0.0)
            if d > base + DEPTH:
                name = key[0]
                if d - base > hits.get((name, part), (0, ""))[0]:
                    hits[(name, part)] = (d - base, label, key[1])
finally:
    set_pose({})
rows = sorted(([n, p, round(d * 1000, 2), lab, i] for (n, p), (d, lab, i) in hits.items()), key=lambda r: -r[2])
json.dump({"poses": len(poses), "depth_mm": DEPTH * 1000, "rest_inside": [[k[0][0], k[0][1], k[1], round(v * 1000, 2)] for k, v in rest.items()],
           "forced": rows}, open(args["out"], "w"), indent=1)
print(f"cable pinch check: {len(local)} cables, {len(parts)} parts, {len(poses)} poses, {time.time() - t0:.0f} s; "
      f"{len(rest)} span/part pairs already inside at rest; cable spans forced > {DEPTH * 1000:.1f} mm into a part: {len(rows)}")
for r in rows:
    print(f"  {r[0]:28} span {r[4]:2}  into {r[1]:32} {r[2]:5.2f} mm  at {r[3]}")
