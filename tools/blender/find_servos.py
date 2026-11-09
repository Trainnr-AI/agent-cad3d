"""Find servo bodies inside imported robot link meshes and match each to the joint it drives.
Runs inside Blender on a collection made by import_mjcf.py.

A servo is a connected mesh piece whose bounding box (in the link's local frame) matches
ARGS["servo_mm"] (sorted dims, ± tol). Its shaft axis is the box axis of size ARGS["shaft_len_mm"]
(e.g. 29 for HD-1910). A joint drives the servo whose shaft axis is parallel to the joint
axis and whose box the joint axis passes through.

ARGS = {"collection": "xgoduck_original", "servo_mm": [20, 29, 34], "tol_mm": 1.2,
        "shaft_len_mm": [29, 30], "out_json": "/abs/servos.json"}
Writes a JSON list: link, servo centre/axes in world frame, size, matched joint + where the
joint axis pierces the servo (shaft offset along the long axis).
"""
import json
from collections import deque

import bpy
from mathutils import Vector

args = globals()["ARGS"]
coll = bpy.data.collections[args["collection"]]
want = sorted(args["servo_mm"])
tol = args.get("tol_mm", 1.2)
shaft_lens = args.get("shaft_len_mm", [29, 30])
bpy.context.view_layer.update()


def components(me):
    adj = [[] for _ in me.vertices]
    for e in me.edges:
        a, b = e.vertices
        adj[a].append(b)
        adj[b].append(a)
    # merge coincident vertices (STL import duplicates them)
    key = {}
    for v in me.vertices:
        k = tuple(round(c, 7) for c in v.co)
        if k in key:
            adj[v.index].append(key[k])
            adj[key[k]].append(v.index)
        else:
            key[k] = v.index
    seen = [False] * len(me.vertices)
    for s in range(len(me.vertices)):
        if seen[s]:
            continue
        comp, q = [], deque([s])
        seen[s] = True
        while q:
            i = q.popleft()
            comp.append(i)
            for j in adj[i]:
                if not seen[j]:
                    seen[j] = True
                    q.append(j)
        yield comp


joints = []
for e in coll.objects:
    if e.type == "EMPTY" and "joint" in e:
        mw = e.matrix_world
        joints.append((e["joint"], mw.translation.copy(), (mw.to_3x3() @ Vector(e["joint_axis"])).normalized()))

found = []
for ob in coll.objects:
    if ob.type != "MESH":
        continue
    me = ob.data
    for comp in components(me):
        if len(comp) < 50:
            continue
        cs = [me.vertices[i].co for i in comp]
        # oriented box from principal axes (a box's PCA axes are its edge directions)
        mean = sum(cs, Vector()) / len(cs)
        C = [[sum((c[i] - mean[i]) * (c[j] - mean[j]) for c in cs) for j in range(3)] for i in range(3)]
        import numpy as np
        w, V = np.linalg.eigh(np.array(C))
        pa = [Vector(V[:, k]).normalized() for k in range(3)]
        proj = [[c.dot(a) for c in cs] for a in pa]
        lo_p = [min(p) for p in proj]
        hi_p = [max(p) for p in proj]
        size_mm = [(hi_p[k] - lo_p[k]) * 1000 for k in range(3)]
        if not all(abs(a - b) <= tol for a, b in zip(sorted(size_mm), want)):
            # fallback: plain local-axis box (PCA is ill-defined for some near-symmetric boxes)
            lo = Vector((min(c.x for c in cs), min(c.y for c in cs), min(c.z for c in cs)))
            hi = Vector((max(c.x for c in cs), max(c.y for c in cs), max(c.z for c in cs)))
            aa = [(hi - lo)[k] * 1000 for k in range(3)]
            if all(abs(a - b) <= tol for a, b in zip(sorted(aa), want)):
                pa = [Vector([1 if i == k else 0 for i in range(3)]) for k in range(3)]
                lo_p, hi_p, size_mm = list(lo), list(hi), aa
        if args.get("debug") and all(12 <= d <= 48 for d in size_mm):
            print(f"    cand {ob.name.split(':', 1)[1]:16} {sorted(round(d, 1) for d in size_mm)} verts {len(comp)}")
        if not all(abs(a - b) <= tol for a, b in zip(sorted(size_mm), want)):
            continue
        mw = ob.matrix_world
        centre_local = sum((pa[k] * (lo_p[k] + hi_p[k]) / 2 for k in range(3)), Vector())
        centre = mw @ centre_local
        axes = [(mw.to_3x3() @ pa[k]).normalized() for k in range(3)]
        order = sorted(range(3), key=lambda k: size_mm[k])
        k_shaft = min(range(3), key=lambda k: min(abs(size_mm[k] - L) for L in shaft_lens))
        k_long = max(range(3), key=lambda k: size_mm[k] if k != k_shaft else -1)
        k_wide = [k for k in range(3) if k not in (k_shaft, k_long)][0]
        rec = {"link": ob.name.split(":", 1)[1], "centre": list(centre), "size_mm": size_mm,
               "shaft_axis": list(axes[k_shaft]), "long_axis": list(axes[k_long]), "wide_axis": list(axes[k_wide]),
               "shaft_len_mm": size_mm[k_shaft], "long_mm": size_mm[k_long], "wide_mm": size_mm[k_wide]}
        # match joint: parallel axis passing through the box
        best = None
        for name, p, a in joints:
            if abs(a.dot(axes[k_shaft])) < 0.98:
                continue
            d = centre - p
            perp = d - a * d.dot(a)                      # offset of box centre from the axis line
            off_long = perp.dot(axes[k_long]) * 1000
            off_wide = perp.dot(axes[k_wide]) * 1000
            if abs(off_long) <= size_mm[k_long] / 2 and abs(off_wide) <= size_mm[k_wide] / 2:
                dist = d.length                                  # nearest joint origin (colinear axes!)
                if best is None or dist < best[1]:
                    best = (name, dist, off_long, off_wide)
        if best:
            rec.update({"joint": best[0], "shaft_offset_long_mm": -best[2], "shaft_offset_wide_mm": -best[3]})
        found.append(rec)

json.dump(found, open(args["out_json"], "w"), indent=1)
print(f"{len(found)} servos found")
for r in sorted(found, key=lambda r: r.get("joint", "~")):
    print(f"  {r['link']:16} -> {r.get('joint', 'UNMATCHED'):16} size {[round(s, 1) for s in r['size_mm']]} "
          f"shaft offset along long axis {r.get('shaft_offset_long_mm', float('nan')):6.2f} mm, wide {r.get('shaft_offset_wide_mm', float('nan')):5.2f} mm")
