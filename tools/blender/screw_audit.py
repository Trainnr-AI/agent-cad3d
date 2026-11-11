"""Check every screw of the assembled robot: where its head is, what it clamps, what it threads into, and whether a
screwdriver can reach the head. Runs inside Blender.

A screw dummy is a head cylinder + a shank cylinder (several screws may share one object). For each screw:
  into     the part that contains the thread, 1 mm short of the tip
  clamps   the part just under the head (should differ from `into`; "-" = the head sits on nothing there)
  driver   the first thing a 40 mm driver would meet coming straight onto the head ("free" if nothing)

ARGS = {"lib": "/abs/tools/blender/overlap_sample.py", "collections": ["leg_v2", "neck_v2", "head_v2", "body_v2"],
        "targets": ["leg_v2", "neck_v2", "head_v2", "body_v2", "electronics"], "json_out": "/abs/out.json"}
"""
import json

import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
L = {"ARGS": {"pairs": []}}
exec(open(args["lib"]).read(), L)
bpy.context.view_layer.update()
targets = []
for cn in args.get("targets", []):
    for o in bpy.data.collections[cn].objects:
        if o.type == "MESH" and "screw" not in o.name and not o.hide_get():
            targets.append(o)
targets += [o for o in bpy.data.objects if o.name.startswith("scs_") and o.type == "MESH" and (o.name.endswith("_case") or o.name.endswith("_spline") or "horn_cross" in o.name) and ".x" not in o.name]
info = {}
for o in targets:
    trees, lo, hi = L["parts"](o)
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.transform(o.matrix_world)
    info[o.name] = (trees, lo, hi, BVHTree.FromBMesh(bm))
    bm.free()


def holder(p):
    for n, (trees, lo, hi, _) in info.items():
        if all(lo[i] - 1e-4 <= p[i] <= hi[i] + 1e-4 for i in range(3)) and L["inside"](trees, p):
            return n
    return "-"


def first_hit(p, d, reach=0.040):
    best = (reach, "free")
    for n, (_, lo, hi, tree) in info.items():
        loc, _, _, dist = tree.ray_cast(p, d, reach)
        if loc is not None and dist < best[0]:
            best = (dist, n)
    return best


rows = []
for cn in args.get("collections", []):
    for o in bpy.data.collections[cn].objects:
        if o.type != "MESH" or "screw" not in o.name:
            continue
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.transform(o.matrix_world)
        seen, shells = set(), []
        for f in bm.faces:
            if f.index in seen:
                continue
            stack, vs = [f], set()
            seen.add(f.index)
            while stack:
                g = stack.pop()
                vs.update(v.index for v in g.verts)
                for e in g.edges:
                    for h in e.link_faces:
                        if h.index not in seen:
                            seen.add(h.index)
                            stack.append(h)
            bm.verts.ensure_lookup_table()
            pts = [bm.verts[i].co.copy() for i in vs]
            c = sum(pts, Vector()) / len(pts)
            ext = max((p - c).length for p in pts)
            shells.append((c, ext, pts))
        bm.free()
        # a screw = one long thin shell (shank) + the nearest short wide shell (head)
        dims = []
        for c, ext, pts in shells:
            lo = Vector([min(p[i] for p in pts) for i in range(3)])
            hi = Vector([max(p[i] for p in pts) for i in range(3)])
            d = sorted(hi - lo)
            dims.append((c, d))
        shanks = [(c, d) for c, d in dims if d[2] > 2.2 * d[1]]
        heads = [(c, d) for c, d in dims if d[2] <= 2.2 * d[1]]
        for k, (cs, ds) in enumerate(shanks):
            ch = min(heads, key=lambda h: (h[0] - cs).length)[0]
            axis = (cs - ch).normalized()                       # from the head toward the tip
            length = ds[2]
            tip = cs + axis * (length / 2)
            # sample beside the axis, not on it (the axis itself runs down the hole): thread at r 0.95, head seat at r 1.45
            u = axis.orthogonal().normalized()
            w = axis.cross(u)

            def ring(centre, r):
                found = [holder(centre + (u * cu + w * cw) * r) for cu, cw in ((1, 0), (-1, 0), (0, 1), (0, -1))]
                named = [n for n in found if n != "-"]
                return max(set(named), key=named.count) if named else "-"

            into = ring(tip - axis * 0.0012, 0.00095)
            under = ring(ch + axis * 0.0011, 0.00145)
            dist, blocker = first_hit(ch - axis * 0.0012, -axis)
            ax = max(range(3), key=lambda i: abs(axis[i]))
            dirname = ("+" if axis[ax] > 0 else "-") + "XYZ"[ax]
            rows.append({"screw": o.name + (f" #{k + 1}" if len(shanks) > 1 else ""), "goes": dirname, "len_mm": round(length * 1000, 1), "clamps": under, "into": into,
                         "driver": blocker if blocker == "free" else f"{blocker} at {dist * 1000:.0f} mm"})
flag = lambda r: "CHECK" if (r["into"] == "-" or r["into"] == r["clamps"]) else "ok   "
for r in rows:
    print(f"  {flag(r)} {r['screw']:38} goes {r['goes']}  clamps {r['clamps']:26} into {r['into']:26} driver: {r['driver']}")
print(f"{len(rows)} screws; {sum(1 for r in rows if flag(r) == 'CHECK')} to check")
if args.get("json_out"):
    json.dump(rows, open(args["json_out"], "w"), indent=1)
