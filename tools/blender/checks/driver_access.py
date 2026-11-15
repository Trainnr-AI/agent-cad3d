"""Screwdriver access: for every screw, the first part in the way along the screw axis (straight driver, tilt 0) and how far
from the head - a short distance means a short or ball-end driver. Runs inside Blender.
ARGS = {"exclude": ["cable_stub"], "tilts": [0]}
(Started from screw_audit.py, whose docstring follows.)"""
"""Screw audit: for every screw stand-in in the scene, measure how it is held. Runs inside Blender at the zero pose.

Per screw (one row per screw, found from the stand-in's head / shank cylinders):
  thread   the part(s) the thread bites into (material between the pilot and the major diameter) and the length of bite
  clamp    the part(s) the shank passes in a clearance hole (clamped under the head) - none of these may also bite (jacking)
  seat     something under the head (within 0.2 mm), so the head clamps
  tip      the hole goes on at least 0.3 mm past the tip (the screw does not bottom out)
  wall     thinnest material round the bitten hole (radial, from the hole wall to the part's outside)

ARGS = {"skip": ["chip_"], "json_out": "/abs/out.json",
        "thread": {"M2": [0.92, 1.25], "M3": [1.4, 1.8]},     # sample radii: inside the thread (bite) / outside a clearance hole
        "min_bite": {"M2": 2.5, "M3": 2.5}, "min_wall": 1.0}
"""
import json
import math

import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

args = globals().get("ARGS", {})
MM = 0.001
SKIP = tuple(args.get("skip", ["chip_"]))
RAD = args.get("thread", {"M2": [0.92, 1.25], "M3": [1.4, 1.8]})
MIN_BITE = args.get("min_bite", {"M2": 2.5, "M3": 2.5})
MIN_WALL = args.get("min_wall", 1.0)
bpy.context.view_layer.update()


def excluded(o):
    return o.name.startswith(SKIP) or ".x" in o.name or not o.visible_get() or any("explode" in c.name for c in o.users_collection)


BOARDS = tuple(args.get("boards", ["imu", "cam_ov5640_b", "unoq_", "servo_adapter"]))   # PCBs drawn without their holes: clamped, not threaded
screws = [o for o in bpy.data.objects if o.type == "MESH" and ("screw" in o.name.lower()) and not o.name.startswith("scs_")
          and not excluded(o) and not o.name.endswith("_case_screws")]
others = [o for o in bpy.data.objects if o.type == "MESH" and o not in screws and not excluded(o) and "screw" not in o.name.lower()
          and not o.name.startswith(("wire_", "wm_", "_"))]
cache = {}


def tree(o):
    if o.name not in cache:
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.transform(o.matrix_world)
        bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
        lo = Vector([min(v[i] for v in bb) for i in range(3)])
        hi = Vector([max(v[i] for v in bb) for i in range(3)])
        cache[o.name] = (BVHTree.FromBMesh(bm), lo, hi)
        bm.free()
    return cache[o.name]


def inside(t, p):
    d = Vector((0.137, 0.311, 0.940)).normalized()
    n, q = 0, p.copy()
    for _ in range(200):
        h = t.ray_cast(q, d)
        if h[0] is None:
            break
        n += 1
        q = h[0] + d * 1e-7
    return n % 2 == 1


def owners(p, cands):
    out = []
    for o in cands:
        t, lo, hi = tree(o)
        if all(lo[i] - 1e-4 <= p[i] <= hi[i] + 1e-4 for i in range(3)) and inside(t, p):
            out.append(o.name)
    return out


def cylinders(o):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.transform(o.matrix_world)
    seen, cyl = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        st, fs = [f], []
        while st:
            g = st.pop()
            if g.index in seen:
                continue
            seen.add(g.index)
            fs.append(g)
            st += [h for e in g.edges for h in e.link_faces if h.index not in seen]
        vs = {v for g in fs for v in g.verts}
        ce = sum((v.co for v in vs), Vector()) / len(vs)
        caps = [g for g in fs if len(g.verts) > 4]
        ax = max(caps or fs, key=lambda g: g.calc_area()).normal.normalized()
        h = max((v.co - ce).dot(ax) for v in vs) * 2
        r = max(((v.co - ce) - ax * (v.co - ce).dot(ax)).length for v in vs)
        cyl.append((ce, ax, h / MM, r / MM, len(vs)))
    bm.free()
    return cyl



TOOL_R = 1.5                      # driver / key shank radius (a PH0-1 screwdriver ~3 mm, a 1.5-2.5 key smaller)
REACH = 35.0                      # how far out the tool needs a straight line
EXCL = tuple(ARGS.get("exclude", []))
ONLY = tuple(ARGS.get("only", []))
TILTS = ARGS.get("tilts", [0])
others = [o for o in others if not o.name.startswith(EXCL)]
res = []
for s in [s_ for s_ in screws if not ONLY or s_.name.startswith(ONLY)]:
    cyl = cylinders(s)
    big = max(c[3] for c in cyl)
    m3 = any(1.35 < c[3] < 1.6 for c in cyl) and big > 2.5
    r_shank = 1.5 if m3 else 1.0
    shanks = [c for c in cyl if abs(c[3] - r_shank) < 0.12 and c[2] > 1.5]
    heads = [c for c in cyl if c[3] > r_shank + 0.4 and c[4] > 6]
    for hd in heads:
        if not shanks: break
        sh = min(shanks, key=lambda c: (c[0] - hd[0]).length)
        out = hd[1] if (hd[0] - sh[0]).dot(hd[1]) > 0 else -hd[1]
        top = hd[0] + out * hd[2] / 2 * MM
        a0 = Vector((1, 0, 0)) if abs(out.x) < 0.9 else Vector((0, 1, 0)); u = out.cross(a0).normalized(); v = out.cross(u)
        best = None
        for tilt in TILTS:
          for az in ((0,) if tilt == 0 else range(0, 360, 30)):
            dirv = (out * math.cos(math.radians(tilt)) + (u * math.cos(math.radians(az)) + v * math.sin(math.radians(az))) * math.sin(math.radians(tilt))).normalized()
            first = None
            for o in others:
                t, lo, hi = tree(o)
                for rr, n_ in ((0.0, 1), (TOOL_R, 6)):
                    for k in range(n_):
                        p = top + (u * math.cos(k * math.pi / 3) + v * math.sin(k * math.pi / 3)) * rr * MM + dirv * 0.05 * MM
                        h = t.ray_cast(p, dirv, REACH * MM)
                        if h[0] is not None:
                            dd = (h[0] - p).length / MM
                            if first is None or dd < first[0]: first = (dd, o.name)
            cand = (first[0] if first else 999.0, tilt, az, first[1] if first else None)
            if best is None or cand[0] > best[0]: best = cand
        res.append((s.name, [round(x / MM, 1) for x in hd[0]], (best[0], f"{best[3]} (tilt {best[1]} az {best[2]})") if best[0] < 999 else None))
blocked = [r for r in res if r[2] is not None and r[2][0] < REACH]
print(f"driver access: {len(res)} screw heads; a part within {REACH:.0f} mm on the axis: {len(blocked)}")
for n, at, f in sorted(blocked, key=lambda r: r[2][0]):
    print(f"  {n:38} {at}  first in the way: {f[1]} at {f[0]:.1f} mm")
