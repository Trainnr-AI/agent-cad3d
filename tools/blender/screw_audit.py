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
          and not o.name.startswith(("wire_", "wm_", "_", "cable_stub"))]    # cable_stub: stale boxes from the early wiring prototype
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
        caps = [g for g in fs if len(g.verts) > 4]                          # a cylinder's / cone's end cap (n-gon) gives its axis;
        ax = max(caps or fs, key=lambda g: g.calc_area()).normal.normalized()   # (the largest face is a side quad on a long thin shank: M2 > ~8 mm)
        h = max((v.co - ce).dot(ax) for v in vs) * 2
        r = max(((v.co - ce) - ax * (v.co - ce).dot(ax)).length for v in vs)
        cyl.append((ce, ax, h / MM, r / MM, len(vs)))
    bm.free()
    return cyl


rows = []
for s in screws:
    cyl = cylinders(s)
    if not cyl:
        continue
    big = max(c[3] for c in cyl)
    m3 = any(1.35 < c[3] < 1.6 for c in cyl) and big > 2.5            # M3 shank r 1.5
    kind = "M3" if m3 else "M2"
    r_shank = 1.5 if m3 else 1.0
    shanks = [c for c in cyl if abs(c[3] - r_shank) < 0.12 and c[2] > 1.5]
    heads = [c for c in cyl if c[3] > r_shank + 0.4 and c[4] > 6]
    cands = None
    for sh in shanks:
        hd = min(heads, key=lambda c: (c[0] - sh[0]).length) if heads else None
        ax = sh[1]
        if hd is not None and (hd[0] - sh[0]).dot(ax) < 0:
            ax = -ax                                                     # ax: from the tip toward the head
        tip = sh[0] - ax * sh[2] / 2 * MM
        seat = sh[0] + ax * sh[2] / 2 * MM
        if hd is not None:
            seat = hd[0] - ax * hd[2] / 2 * MM
        L = (seat - tip).dot(ax) / MM
        a0 = Vector((1, 0, 0)) if abs(ax.x) < 0.9 else Vector((0, 1, 0))
        u = ax.cross(a0).normalized()
        v = ax.cross(u)
        if cands is None:
            lo = sh[0] - Vector((0.02, 0.02, 0.02))
            hi = sh[0] + Vector((0.02, 0.02, 0.02))
            cands = [o for o in others if all(tree(o)[1][i] < hi[i] and tree(o)[2][i] > lo[i] for i in range(3))]
        r_in, r_out = RAD[kind]
        bite, clamp = {}, {}
        n = max(8, int(L / 0.25))
        for k in range(n):
            t_ = (k + 0.5) / n * L
            p0 = tip + ax * t_ * MM
            for a in range(6):
                d_ = u * math.cos(a * math.pi / 3) + v * math.sin(a * math.pi / 3)
                for nm in owners(p0 + d_ * r_in * MM, cands):
                    bite[nm] = bite.get(nm, 0) + 1
                for nm in owners(p0 + d_ * r_out * MM, cands):
                    clamp[nm] = clamp.get(nm, 0) + 1
        dl = L / n
        bite_mm = {k: round(c / 6 * dl, 2) for k, c in bite.items() if c / 6 * dl > 0.2 and not k.startswith(BOARDS) and "horn_cross" not in k}   # the horn: clamped (its hub hole is drawn small)
        clamp_mm = {k: round(c / 6 * dl, 2) for k, c in clamp.items() if k not in bite_mm and c / 6 * dl > 0.2}
        for k, v_ in list(clamp_mm.items()):                              # a nut: the screw's length inside it is the bite
            if "nut" in k:
                bite_mm[k] = v_
                del clamp_mm[k]
        seat_ok = bool(owners(seat - ax * 0.15 * MM + u * (r_shank + 0.6) * MM, cands) or owners(seat - ax * 0.15 * MM - u * (r_shank + 0.6) * MM, cands))
        tip_free = not any(owners(tip - ax * d * MM, [c for c in cands if c.name in bite_mm]) for d in (0.1, 0.3))
        wall = None
        for nm in bite_mm:
            t, _lo, _hi = tree(bpy.data.objects[nm])
            for k in range(1, 5):
                p0 = tip + ax * (L * k / 5) * MM
                if not inside(t, p0 + u * r_in * MM):
                    continue
                for a in range(16):
                    d_ = u * math.cos(a * math.pi / 8) + v * math.sin(a * math.pi / 8)
                    q = p0 + d_ * r_in * MM
                    h = t.ray_cast(q, d_, 0.03)
                    if h[0] is not None:
                        w = (h[0] - q).length / MM + r_in - (0.8 if kind == "M2" else 1.25)
                        wall = w if wall is None else min(wall, w)
        cap = None                                                         # material left beyond the pilot's end (axially): no breakout
        for nm in bite_mm:
            if "nut" in nm:
                continue
            t, _lo, _hi = tree(bpy.data.objects[nm])
            h = t.ray_cast(tip + ax * 0.05 * MM, -ax, 0.02)                 # from inside the pilot, past the tip: the pilot's end face
            if h[0] is not None:
                h2 = t.ray_cast(h[0] - ax * 1e-6, -ax, 0.05)                # ... then out of the part
                if h2[0] is not None:
                    c_ = (h2[0] - h[0]).length / MM
                    cap = c_ if cap is None else min(cap, c_)
        total_bite = max(bite_mm.values()) if bite_mm else 0.0
        flags = []
        if total_bite < MIN_BITE[kind]:
            flags.append(f"bite {total_bite:.1f} < {MIN_BITE[kind]}")
        if len(bite_mm) > 1 and not any("nut" in k for k in bite_mm):
            flags.append("bites in 2 parts (jacking?)")
        if not seat_ok:
            flags.append("head not seated")
        if not tip_free:
            flags.append("bottoms out")
        if wall is not None and wall < MIN_WALL:
            flags.append(f"wall {wall:.2f}")
        if cap is not None and cap < 0.6:
            flags.append(f"pilot end {cap:.2f} from the outside")
        rows.append(dict(screw=s.name, kind=kind, length=round(L, 2), at=[round(x / MM, 1) for x in tip], axis=[round(c, 4) for c in ax],
                         head=[round(x / MM, 2) for x in (tip + ax * (L + 2.0) * MM)], bite=bite_mm, clamp=clamp_mm,
                         seat=seat_ok, tip_free=tip_free, wall=round(wall, 2) if wall is not None else None,
                         cap=round(cap, 2) if cap is not None else None, flags=flags))
bad = [r for r in rows if r["flags"]]
print(f"screw audit: {len(rows)} screws in {len(screws)} stand-ins; flagged {len(bad)}")
for r in rows:
    print(f"  {'!!' if r['flags'] else 'ok'} {r['screw']:36} {r['kind']} L{r['length']:5.1f} bite {r['bite']} clamp {list(r['clamp'])} "
          f"wall {r['wall']} cap {r['cap']} {'; '.join(r['flags'])}")
if args.get("json_out"):
    json.dump(rows, open(args["json_out"], "w"), indent=1)
