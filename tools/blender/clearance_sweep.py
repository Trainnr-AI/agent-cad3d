"""Pose sweep with a clearance margin: for every pose, every pair of parts on different robot bodies is checked for
surfaces crossing (with penetration depth) AND for gaps smaller than a margin. leg_motion_check.py reports only intersection
volumes > 0.05 mm3, which misses thin touches (the 2026-11-14 independent review found 0.06-0.48 mm ones it had passed).

Runs inside Blender, live (blender_doctor.py) or headless, which keeps the user's session free for long sweeps:
  blender -b projects/xgoduck_scs0009/xgoduck_scs0009.blend --python tools/blender/clearance_sweep.py -- args.json

ARGS = {"robot": "duck_scs0009", "poses": [{"label": "...", "q": {"left_hip_pitch": 45.0, ...}}, ...]   (degrees)
        "margin_mm": 0.5, "depth_mm": 0.05, "out": "/abs/result.json",
        }

Parts: every visible mesh outside the reference / exploded / wiring collections, except the ones that ride with a servo output
or sit inside the case (horns, splines, horn screws, case screws). Each part rides on its nearest `<robot>:` ancestor.
A pair already closer than the margin at rest (mating faces: bearing on boss, servo in its holder) is not judged; the list is
in the output ("rest_pairs"). A pose COLLIDES when surfaces cross with a depth > depth_mm (0.05, as the review), TOUCHES when
they cross less deep or the gap is under the margin. Gaps and depths are point-to-surface, both ways, from every vertex, every triangle centre and
points 0.5 mm apart over large triangles; two edges passing each other can read up to ~0.25 mm larger than they are - keep
the margin >= 0.3 mm. (Fixed 2026-11-15: the first version tested only small triangles and missed contacts on big flat faces.)
Every joint is put back at rest at the end, also on an error.
"""
import json
import math
import sys
import time

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

args = globals().get("ARGS")
if args is None:                                               # headless: blender -b file --python this.py -- args.json
    args = json.load(open(sys.argv[sys.argv.index("--") + 1]))
R = args.get("robot", "duck_scs0009")
MARGIN = args.get("margin_mm", 0.5) / 1000
DEPTH = args.get("depth_mm", 0.05) / 1000
SKIPC = {"duck_v2_exploded", "wiring_exploded", "xgoduck_original", "dimensions", "orig_head_parts", "coupon", "wiring",
         "wiring_mesh", "cables", "print_tools"}
RAYS = [Vector(d).normalized() for d in ((0.267, 0.534, 0.802), (-0.577, 0.577, 0.577), (0.408, -0.816, 0.408))]
CAP = 20000                                                    # points tested per pair and direction (stride above this)
SPACING = 0.0005                                               # surface sample spacing on large triangles (m)


def owner(o):
    while o is not None and not o.name.startswith(R + ":"):
        o = o.parent
    return o


def rides_inside(n):
    return "_horn_" in n or n.endswith("_spline") or n.endswith("_screw_M2x4kit") or n.endswith("_case_screws")


bpy.context.view_layer.update()
joints = {e["joint"]: e for e in bpy.data.collections[R].objects if e.type == "EMPTY" and "joint" in e and e.name.startswith(R + ":")}
q0 = {n: e.rotation_quaternion.copy() for n, e in joints.items()}
dg = bpy.context.evaluated_depsgraph_get()


class Part:
    def __init__(s, o, body):
        s.name, s.body = o.name, body
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        me.calc_loop_triangles()
        to_body = body.matrix_world.inverted() @ o.matrix_world
        v = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", v)
        v = v.reshape(-1, 3)
        M = np.array(to_body)
        s.v = v @ M[:3, :3].T + M[:3, 3]                       # vertices in the body frame
        t = np.empty(len(me.loop_triangles) * 3, dtype=np.int64)
        me.loop_triangles.foreach_get("vertices", t)
        s.t = t.reshape(-1, 3)
        ev.to_mesh_clear()
        s.c = s.v[s.t].mean(axis=1)                            # triangle centres
        tri = s.v[s.t]                                         # + points over every large triangle (printed parts are a few
        edge = np.max(np.linalg.norm(tri - np.roll(tri, 1, axis=1), axis=2), axis=1)   # big flat faces: their vertices and
        extra = []                                             # centres alone miss an edge pressing into the middle of a face)
        for k in np.nonzero(edge > SPACING)[0]:
            n = min(int(math.ceil(edge[k] / SPACING)), 60)
            w = np.array([(i, j, n - i - j) for i in range(n + 1) for j in range(n + 1 - i)], float) / n
            extra.append(w @ tri[k])
        s.c = np.vstack([s.c] + extra) if extra else s.c
        s.tree = BVHTree.FromPolygons([Vector(p) for p in s.v], s.t.tolist())
        s.lo, s.hi = s.v.min(axis=0), s.v.max(axis=0)

    def inside(s, p):
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
        return votes >= 2


parts = []
for o in bpy.data.objects:
    if o.type != "MESH" or not o.users_collection or {c.name for c in o.users_collection} & SKIPC:
        continue
    if o.name.startswith(R + ":") or not o.visible_get() or rides_inside(o.name):
        continue
    b = owner(o)
    if b is None:
        print("not on the robot (skipped):", o.name)
        continue
    parts.append(Part(o, b))

def body_mats():
    return {p.body.name: np.array(p.body.matrix_world) for p in parts}


def near(a, T, lo, hi):
    """a's vertices, triangle centres and surface samples, taken into the other part's frame by T, that lie in its box (lo, hi)."""
    pts = np.vstack([a.v, a.c]) @ T[:3, :3].T + T[:3, 3]
    sel = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
    return sel[:: len(sel) // CAP + 1] if len(sel) > CAP else sel


def one_way(a, b, T, lo, hi, crossing):
    """(min gap, max depth) of a's points against part b. Depth only when the surfaces cross (else nothing is inside)."""
    gap, depth = 1.0, 0.0
    for p in near(a, T, lo, hi):
        p = Vector(p)
        if crossing and b.inside(p):
            depth = max(depth, b.tree.find_nearest(p)[3])
            continue
        hit = b.tree.find_nearest(p, MARGIN)
        if hit[0] is not None:
            gap = min(gap, hit[3])
    return gap, depth


def pair(a, b, Ma, Mb):
    """(gap m, depth m, crossing) for parts a, b with body world matrices Ma, Mb; None if clear by the margin."""
    T = np.linalg.inv(Mb) @ Ma                                  # a's body frame -> b's body frame
    ca = np.array([[x, y, z] for x in (a.lo[0], a.hi[0]) for y in (a.lo[1], a.hi[1]) for z in (a.lo[2], a.hi[2])])
    cb = ca @ T[:3, :3].T + T[:3, 3]
    lo, hi = b.lo - MARGIN, b.hi + MARGIN
    if np.any(cb.min(axis=0) > hi) or np.any(cb.max(axis=0) < lo):
        return None
    pa = a.v @ T[:3, :3].T + T[:3, 3]                           # do the surfaces cross? (a's triangles near b vs b)
    ta_ = pa[a.t]                                               # triangles whose box meets b's box (not only small ones inside it)
    keep = np.all(ta_.min(axis=1) <= hi, axis=1) & np.all(ta_.max(axis=1) >= lo, axis=1)
    if not keep.any():
        return None
    crossing = bool(BVHTree.FromPolygons([Vector(p) for p in pa], a.t[keep].tolist()).overlap(b.tree))
    g1, d1 = one_way(a, b, T, lo, hi, crossing)
    g2, d2 = one_way(b, a, np.linalg.inv(T), a.lo - MARGIN, a.hi + MARGIN, crossing)
    gap, depth = min(g1, g2), max(d1, d2)
    if gap >= MARGIN and not crossing:
        return None
    return (0.0 if crossing else gap), depth, crossing


def set_pose(q):
    for n, e in joints.items():
        e.rotation_quaternion = q0[n] @ Quaternion(Vector(e["joint_axis"]), math.radians(q.get(n, 0.0)))
    bpy.context.view_layer.update()


def check(mats, rest_mats, skip):
    hits = []
    for i, a in enumerate(parts):
        for b in parts[i + 1:]:
            if a.body is b.body or (a.name, b.name) in skip:
                continue
            Ra, Rb = rest_mats[a.body.name], rest_mats[b.body.name]
            Ma, Mb = mats[a.body.name], mats[b.body.name]
            if np.allclose(np.linalg.inv(Mb) @ Ma, np.linalg.inv(Rb) @ Ra, atol=1e-9):
                continue                                        # same relative pose as at rest
            r = pair(a, b, Ma, Mb)
            if r:
                hits.append(r + (a.name, b.name))
    return hits


t0 = time.time()
out = {"what": "clearance_sweep.py", "margin_mm": MARGIN * 1000, "depth_mm": DEPTH * 1000, "poses": []}
try:
    set_pose({})
    rest = body_mats()
    rest_pairs = set()
    for i, a in enumerate(parts):                               # pairs closer than the margin at rest are not judged
        for b in parts[i + 1:]:
            if a.body is not b.body and pair(a, b, rest[a.body.name], rest[b.body.name]):
                rest_pairs.add((a.name, b.name))
    out["rest_pairs"] = sorted(f"{a} x {b}" for a, b in rest_pairs)
    for k, P in enumerate(args["poses"]):
        set_pose(P["q"])
        hits = check(body_mats(), rest, rest_pairs)
        gap = min([h[0] for h in hits], default=None)
        depth = max([h[1] for h in hits], default=0.0)
        crossing = any(h[2] for h in hits)
        out["poses"].append({
            "label": P["label"], "q": P["q"],
            "collides": crossing and depth > DEPTH,
            "touches": bool(hits),
            "min_gap_mm": None if gap is None or gap >= 1.0 else round(gap * 1000, 2),
            "max_depth_mm": round(depth * 1000, 2),
            "pairs": [f"{a} x {b}: " + (f"depth {d * 1000:.2f}" if c or d > 0 else f"gap {g * 1000:.2f}") + " mm"
                      for g, d, c, a, b in sorted(hits, key=lambda h: (-h[1], h[0]))]})
        if (k + 1) % 50 == 0:
            print(f"  {k + 1}/{len(args['poses'])} poses, {time.time() - t0:.0f} s", flush=True)
finally:
    set_pose({})
json.dump(out, open(args["out"], "w"), indent=1)
n = len(out["poses"])
print(f"clearance sweep: {len(parts)} parts, {n} poses, collide {sum(p['collides'] for p in out['poses'])}, "
      f"touch / under {MARGIN * 1000:.2f} mm {sum(p['touches'] for p in out['poses'])}, {time.time() - t0:.0f} s; "
      f"{len(out['rest_pairs'])} pairs closer than the margin at rest not judged", flush=True)
