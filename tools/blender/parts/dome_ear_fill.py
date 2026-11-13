"""Fill the head dome's two side "ear" arches (kept from the original shell; our duck's jaw does not pivot there: user
2026-11-13). Runs inside Blender on the built head_dome, at the zero pose. Idempotent (a filled arch is filled again with
material that is already there).

The patch follows the wall: the dome's outer and inner surfaces round each arch are fitted (quadratic y(x, z)) and the
patch is that slab, 0.15 inside both faces (so where it overlaps the existing wall it adds no surface). Where a part
that does not ride on the head still swings through an arch (right side: the head-pitch U in head roll), the swept
region of those parts over the allowed head poses + `clear` is left open: one profile prism along y (the exact boolean
fails with many-hull cutters).

ARGS = {"robot": "duck_scs0009", "dome": "head_dome", "base": "head_base", "x": [26, 44], "z_top": 217.5,
        "sides": {"L": {"sweep": false}, "R": {"sweep": true}},
        "sweep_parts": ["head_pitch_u", ...], "carry": {"wire_hub_Y": "head_pitch_u", ...},
        "poses": [[yaw_deg, roll_deg], ...], "clear": 1.5}
"""
import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
MM = 0.001
R = args.get("robot", "duck_scs0009")
dome = bpy.data.objects[args.get("dome", "head_dome")]
X0, X1 = args.get("x", [26.0, 44.0])
ZTOP = args.get("z_top", 217.5)
INSET, CLEAR = 0.15, args.get("clear", 1.5)
log = []


def world_tree(o):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.transform(o.matrix_world)
    t = BVHTree.FromBMesh(bm)
    bm.free()
    return t


def shells_manifold(o):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    seen, n = set(), 0
    for f in bm.faces:
        if f.index in seen:
            continue
        n += 1
        st = [f]
        while st:
            g = st.pop()
            if g.index in seen:
                continue
            seen.add(g.index)
            st += [h for e in g.edges for h in e.link_faces if h.index not in seen]
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    v = bm.calc_volume() * 1e9
    bm.free()
    return n, nm, v


def boolean(target, bm_world, op):
    me = bpy.data.meshes.new("_ear_tmp")
    bm_world.transform(target.matrix_world.inverted())
    bm_world.to_mesh(me)
    bm_world.free()
    c = bpy.data.objects.new("_ear_tmp", me)
    bpy.context.scene.collection.objects.link(c)
    c.matrix_world = target.matrix_world.copy()
    m = target.modifiers.new("_ear", "BOOLEAN")
    m.operation, m.object, m.solver = op, c, "EXACT"
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    res = bpy.data.meshes.new_from_object(target.evaluated_get(dg))
    target.modifiers.remove(m)
    b = bmesh.new()
    b.from_mesh(res)
    vol = b.calc_volume() * 1e9
    ok = vol > 0.5 * shells_manifold(target)[2]                   # never write a wiped-out result (shared mesh: copies follow)
    if ok:
        b.to_mesh(target.data)
        target.data.update()
    b.free()
    bpy.data.meshes.remove(res)
    bpy.data.objects.remove(c)
    bpy.data.meshes.remove(me)
    return ok, vol


# ---------------------------------------------------------------- wall surfaces round each arch
td = world_tree(dome)
bb = [dome.matrix_world @ Vector(c) for c in dome.bound_box]
YLO, YHI = min(v.y for v in bb) / MM, max(v.y for v in bb) / MM
ZBOT = min(v.z for v in bb) / MM
YC = (YLO + YHI) / 2


def wall_hits(side):
    """(x, z, y_outer, y_inner) on the wall round the arch (rays along y from outside)."""
    y0, dy = (YHI + 5, -1.0) if side == "L" else (YLO - 5, 1.0)
    pts = []
    for x in np.arange(X0 - 8, X1 + 8.01, 1.0):
        for z in np.arange(ZBOT + 0.6, ZTOP + 5.01, 1.0):
            if X0 - 0.5 < x < X1 + 0.5 and z < ZTOP:              # the arch (and its margin) itself
                continue
            h = td.ray_cast(Vector((x, y0, z)) * MM, Vector((0, dy, 0)), 0.2)
            if h[0] is None:
                continue
            h2 = td.ray_cast(h[0] + Vector((0, dy, 0)) * 1e-6, Vector((0, dy, 0)), 0.2)
            if h2[0] is None:
                continue
            yo, yi = h[0].y / MM, h2[0].y / MM
            if (side == "L" and yi > YC + 20) or (side == "R" and yi < YC - 20):   # the near wall only
                if 0.8 < abs(yo - yi) < 6.0:
                    pts.append((x, z, yo, yi))
    return np.array(pts)


def fit(P, col):
    A = np.c_[np.ones(len(P)), P[:, 0], P[:, 1], P[:, 0] ** 2, P[:, 0] * P[:, 1], P[:, 1] ** 2]
    c, *_ = np.linalg.lstsq(A, P[:, col], rcond=None)
    r = A @ c - P[:, col]
    return c, float(np.sqrt(np.mean(r ** 2))), float(np.max(np.abs(r)))


def ev(c, x, z):
    return c[0] + c[1] * x + c[2] * z + c[3] * x * x + c[4] * x * z + c[5] * z * z


def slab(co, ci, sgn, x0, x1, z0, z1, n=24):
    """closed slab between the fitted outer / inner faces (INSET inside both) over x0..x1, z0..z1 (world mm)."""
    bm = bmesh.new()
    xs, zs = np.linspace(x0, x1, n), np.linspace(z0, z1, n)
    grid = {}
    for face, c, off in (("o", co, -sgn * INSET), ("i", ci, sgn * INSET)):
        for i, x in enumerate(xs):
            for j, z in enumerate(zs):
                grid[face, i, j] = bm.verts.new(Vector((x, ev(c, x, z) + off, z)) * MM)
    for i in range(n - 1):
        for j in range(n - 1):
            q = [grid["o", i, j], grid["o", i + 1, j], grid["o", i + 1, j + 1], grid["o", i, j + 1]]
            bm.faces.new(q)
            bm.faces.new([grid["i", i, j], grid["i", i, j + 1], grid["i", i + 1, j + 1], grid["i", i + 1, j]])
    rim = [(i, 0) for i in range(n - 1)] + [(n - 1, j) for j in range(n - 1)] + [(i, n - 1) for i in range(n - 1, 0, -1)] + [(0, j) for j in range(n - 1, 0, -1)]
    for k in range(len(rim)):
        a, b = rim[k], rim[(k + 1) % len(rim)]
        bm.faces.new([grid["o", a[0], a[1]], grid["i", a[0], a[1]], grid["i", b[0], b[1]], grid["o", b[0], b[1]]])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


# ---------------------------------------------------------------- what still swings through an arch (head frame)
def swept_profile(xs, ylo, yhi):
    """max z (world mm, zero pose) per x column of the sweep parts inside y ylo..yhi, over the allowed head poses."""
    joints = {e["joint"]: e for e in bpy.data.collections[R].objects if e.type == "EMPTY" and "joint" in e and e.name.startswith(R + ":")}
    q0 = {k: (Quaternion(e["q0"]) if "q0" in e else e.rotation_quaternion.copy()) for k, e in joints.items()}
    parts = [bpy.data.objects[n] for n in args["sweep_parts"] if n in bpy.data.objects]
    carry = {bpy.data.objects[n]: bpy.data.objects[a] for n, a in args.get("carry", {}).items() if n in bpy.data.objects}
    bpy.context.view_layer.update()
    rel = {o: a.matrix_world.inverted() @ o.matrix_world for o, a in carry.items()}
    D0 = dome.matrix_world.copy()
    zmax = np.full(len(xs), -1e9)
    for yaw, roll in args["poses"]:
        for jn, ang in (("head_yaw", yaw), ("head_roll", roll)):
            e = joints[jn]
            e.rotation_quaternion = q0[jn] @ Quaternion(Vector(e["joint_axis"]), math.radians(ang))
        bpy.context.view_layer.update()
        to_rest = D0 @ dome.matrix_world.inverted()                    # world now -> world at the zero pose, in the head's frame
        for o in parts + list(carry):
            mw = (carry[o].matrix_world @ rel[o]) if o in carry else o.matrix_world
            M = to_rest @ mw
            vs = [M @ v.co for v in o.data.vertices]
            t = BVHTree.FromPolygons(vs, [p.vertices[:] for p in o.data.polygons])
            for i, x in enumerate(xs):
                for y in np.arange(ylo, yhi + 0.01, 0.5):
                    h = t.ray_cast(Vector((x, y, ZTOP + 30)) * MM, Vector((0, 0, -1)), 0.1)
                    if h[0] is not None:
                        zmax[i] = max(zmax[i], h[0].z / MM)
    for jn in ("head_yaw", "head_roll"):
        joints[jn].rotation_quaternion = q0[jn]
    bpy.context.view_layer.update()
    return zmax


# ---------------------------------------------------------------- per side
for side, cfg in args.get("sides", {"L": {}, "R": {}}).items():
    P = wall_hits(side)
    if len(P) < 30:
        log.append(f"{side}: too few wall hits ({len(P)}) - skipped")
        continue
    co, ro, mo = fit(P, 2)
    ci, ri, mi = fit(P, 3)
    sgn = 1.0 if side == "L" else -1.0                                 # outward = +y on the left
    log.append(f"{side}: {len(P)} wall points; fit rms/max outer {ro:.3f}/{mo:.3f}, inner {ri:.3f}/{mi:.3f} mm; "
               f"wall {abs(ev(co, 35, 207) - ev(ci, 35, 207)):.2f} thick at the arch centre")
    patch = slab(co, ci, sgn, X0, X1, ZBOT - 0.5, ZTOP)
    ok, v = boolean(dome, patch, "UNION")
    log.append(f"{side}: patch {'added' if ok else 'FAILED (not written)'} -> volume {v:.1f}")
    if cfg.get("sweep"):
        xs = np.arange(X0, X1 + 0.01, 0.5)
        ylo, yhi = sorted((ev(co, 35, 207) + sgn * 1.0, ev(ci, 35, 207) - sgn * 1.0))
        zm = swept_profile(xs, ylo - 1.0, yhi + 1.0)
        if np.max(zm) > -1e8:
            top = np.where(zm > -1e8, zm + CLEAR, ZBOT - 1.0)
            for k in range(len(top)):                                    # widen sideways by the clearance as well
                lo_, hi_ = max(0, k - int(CLEAR / 0.5)), min(len(top), k + int(CLEAR / 0.5) + 1)
                top[k] = max(top[lo_:hi_])
            poly = [(xs[0] - 0.01, ZBOT - 2.0)] + [(x, max(t, ZBOT - 1.0)) for x, t in zip(xs, top)] + [(xs[-1] + 0.01, ZBOT - 2.0)]
            bm = bmesh.new()
            ya, yb = ylo - 3.0, yhi + 3.0
            bot = [bm.verts.new(Vector((x, ya, z)) * MM) for x, z in poly]
            topv = [bm.verts.new(Vector((x, yb, z)) * MM) for x, z in poly]
            bm.faces.new(bot)
            bm.faces.new(topv[::-1])
            for k in range(len(poly)):
                bm.faces.new([bot[k], bot[(k + 1) % len(poly)], topv[(k + 1) % len(poly)], topv[k]])
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
            ok, v = boolean(dome, bm, "DIFFERENCE")
            log.append(f"{side}: left open where the U swings (+{CLEAR} clearance): up to z {max(top):.1f} over x "
                       f"{xs[np.argmax(zm > -1e8)]:.1f}..{xs[len(zm) - 1 - np.argmax(zm[::-1] > -1e8)]:.1f} "
                       f"({'cut' if ok else 'FAILED'}; volume {v:.1f})")
base = bpy.data.objects.get(args.get("base", "head_base"))
if base:                                                               # the dome gives way to the base (as head_shell_v2 builds it)
    bb_ = bmesh.new()
    bb_.from_mesh(base.data)
    bb_.transform(base.matrix_world)
    ok, v = boolean(dome, bb_, "DIFFERENCE")
    log.append(f"dome minus base: {'ok' if ok else 'FAILED'} ({v:.1f})")
n, nm, v = shells_manifold(dome)
log.append(f"head_dome: shells {n}, non-manifold edges {nm}, volume {v:.1f} mm3")
print("\n".join(log))
