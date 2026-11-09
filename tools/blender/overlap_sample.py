"""Overlap volume of two meshes by point sampling (robust where exact booleans fail, e.g. meshes
built from several overlapping closed pieces such as the servo horns). Runs inside Blender.

A point is inside a mesh if it's inside any of its loose parts; each part must be closed.
Inside test per part: ray parity (odd number of surface crossings), majority of three rays.

ARGS = {"pairs": [["scs_head_pitch_horn_cross", "head_pitch_u"], ...], "step_mm": 0.15}
"""
import bmesh
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
STEP = args.get("step_mm", 0.15) / 1000


def parts(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.transform(ob.matrix_world)
    bm.verts.ensure_lookup_table()
    seen, out = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        stack, comp = [f], []
        seen.add(f.index)
        while stack:
            g = stack.pop()
            comp.append(g)
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        stack.append(h)
        verts = sorted({v.index for g in comp for v in g.verts})
        idx = {v: i for i, v in enumerate(verts)}
        out.append(BVHTree.FromPolygons([bm.verts[v].co.copy() for v in verts], [[idx[v.index] for v in g.verts] for g in comp]))
    lo = Vector([min(v.co[i] for v in bm.verts) for i in range(3)])
    hi = Vector([max(v.co[i] for v in bm.verts) for i in range(3)])
    bm.free()
    return out, lo, hi


RAYS = (Vector((0.267, 0.534, 0.802)), Vector((-0.577, 0.577, 0.577)), Vector((0.408, -0.816, 0.408)))


def _crossings(t, p, d):
    n, o = 0, p.copy()
    for _ in range(64):
        loc, _, _, dist = t.ray_cast(o, d)
        if loc is None:
            break
        n += 1
        o = loc + d * 1e-7
    return n


def inside(trees, p):
    """ray-parity test (independent of face normals), majority of three skew rays per part"""
    for t in trees:
        if sum(_crossings(t, p, d) % 2 for d in RAYS) >= 2:
            return True
    return False


for a, b in args["pairs"]:
    ta, la, ha = parts(bpy.data.objects[a])
    tb, lb, hb = parts(bpy.data.objects[b])
    lo = Vector([max(la[i], lb[i]) for i in range(3)])
    hi = Vector([min(ha[i], hb[i]) for i in range(3)])
    n_in = 0
    if all(lo[i] < hi[i] for i in range(3)):
        nx, ny, nz = [max(1, int((hi[i] - lo[i]) / STEP)) for i in range(3)]
        for i in range(nx):
            for j in range(ny):
                for k in range(nz):
                    p = lo + Vector(((i + 0.5) * STEP, (j + 0.5) * STEP, (k + 0.5) * STEP))
                    if inside(ta, p) and inside(tb, p):
                        n_in += 1
    v = n_in * (STEP * 1000) ** 3
    print(f"  {a} ∩ {b}: {v:8.2f} mm³ (sampled at {STEP * 1000:.2f} mm)")
