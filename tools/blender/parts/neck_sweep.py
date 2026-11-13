"""The space the neck and the head-pitch U sweep when the neck leans back, as ONE cutter for a fixed part below it (the Uno Q
cover). Runs inside Blender at the zero pose; the pose is restored afterwards.

The parts swing in the side (x-z) plane, so the cutter is a prism across the target's width: everything above the lowest
height the moving parts reach at each x (over the target's y span), less `clear` mm (grown by `clear` along x as well).
Points: each moving object's surface sampled every ~0.8 mm (own frame, once), carried through every pose
(neck_deg x head_deg grid; the head angle only matters for the parts on the U).
One boolean instead of hundreds of hulls (those broke the exact solver).

Library (body_v2.py):  NS = {"ARGS": {"lib": True}}; exec(open(path).read(), NS)
    poly, (y0, y1) = NS["sweep_profile"](cfg, near_box)          # layout mm
    bm = NS["prism"](poly, y0, y1, to_world)                       # to_world(Vector layout mm) -> world Vector (m)
  cfg = {"cradle": [names riding on the neck], "u": [names riding on the head-pitch U], "neck_deg": [0, 34, 1],
         "head_deg": [-45, 90, 7.5], "clear": 1.5, "dx": 0.5, "keep_rear": true (a piece cut loose at the front is dropped)}
Stand-alone: ARGS = {"lib": False, "target": "uno_cover", "cfg": {...}, "joint_lib": ..., "base_lib": ..., "dims": ...}
  cuts the target in place (body_v2.py with "neck_sweep" in its ARGS makes the same part on a rebuild).
"""
import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

MM = 0.001


def _samples(o, step=0.0008):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    pts = [v.co[:] for v in bm.verts]
    for f in bm.faces:
        a, b, c = (v.co for v in f.verts)
        n = int(max((b - a).length, (c - a).length, (c - b).length) / step)
        if n < 1:
            continue
        for i in range(n + 1):
            for j in range(n + 1 - i):
                pts.append((a + (b - a) * (i / n) + (c - a) * (j / n))[:])
    bm.free()
    P = np.array(pts)
    return np.hstack([P, np.ones((len(P), 1))])


def sweep_profile(cfg, near_box, robot="duck_scs0009"):
    J = {e["joint"]: e for e in bpy.data.objects if e.name.startswith(robot + ":") and e.type == "EMPTY" and "joint" in e and e.users_collection}   # (in the scene: not a stray copy pulled in by a library load)
    q0 = {n: e.rotation_quaternion.copy() for n, e in J.items()}
    trunk = bpy.data.objects[robot + ":trunk_base"]
    Li = np.array((trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)).inverted())
    CL, DX = cfg.get("clear", 1.5), cfg.get("dx", 0.5)
    n0, n1, dn = cfg.get("neck_deg", [0, 34, 1])
    h0, h1, dh = cfg.get("head_deg", [-45, 90, 7.5])
    necks = np.arange(n0, n1 + 1e-6, dn)
    heads = np.arange(h0, h1 + 1e-6, dh)
    bx0, bx1, by0, by1, bz0, bz1 = near_box
    S = {n: (bpy.data.objects[n], _samples(bpy.data.objects[n])) for n in cfg.get("cradle", []) + cfg.get("u", []) if n in bpy.data.objects}
    # every listed object rides on its anchor part (the wiring's hubs and plugs are free objects until the motion check parents them)
    anchors = {"cradle": bpy.data.objects[cfg.get("cradle_anchor", "neck_cradle")], "u": bpy.data.objects[cfg.get("u_anchor", "head_pitch_u")]}
    bpy.context.view_layer.update()
    rel = {n: anchors["u" if n in cfg.get("u", []) else "cradle"].matrix_world.inverted() @ o.matrix_world for n, (o, P) in S.items()}
    xs = np.arange(math.floor(bx0 - 5), math.ceil(bx1 + 5) + DX, DX)
    zmin = np.full(len(xs), np.inf)

    def mark(P):
        P = P[(P[:, 1] > by0 - CL) & (P[:, 1] < by1 + CL) & (P[:, 0] > xs[0]) & (P[:, 0] < xs[-1]) & (P[:, 2] < bz1 + 30)]
        if len(P):
            k = np.clip(np.round((P[:, 0] - xs[0]) / DX).astype(int), 0, len(xs) - 1)
            np.minimum.at(zmin, k, P[:, 2])

    def pose(nd, hd):
        for jn, e in J.items():
            e.rotation_quaternion = q0[jn]
        J["neck_pitch"].rotation_quaternion = q0["neck_pitch"] @ Quaternion(Vector(J["neck_pitch"]["joint_axis"]), math.radians(nd))
        J["head_pitch"].rotation_quaternion = q0["head_pitch"] @ Quaternion(Vector(J["head_pitch"]["joint_axis"]), math.radians(hd))
        bpy.context.view_layer.update()

    try:
        for nd in necks:
            for k, hd in enumerate(heads if cfg.get("u") else [0.0]):
                pose(nd, hd)
                for n, (o, P) in S.items():
                    if n in cfg.get("cradle", []) and k:
                        continue
                    A = anchors["u" if n in cfg.get("u", []) else "cradle"]
                    mark(((Li @ np.array(A.matrix_world @ rel[n]) @ P.T).T[:, :3]) * 1000.0)
    finally:
        for jn, e in J.items():
            e.rotation_quaternion = q0[jn]
        bpy.context.view_layer.update()
    w = int(round(CL / DX))
    zc = np.array([zmin[max(0, i - w): i + w + 1].min() for i in range(len(xs))]) - CL     # grown by CL along x and down
    top = bz1 + 20.0
    keep = zc < top
    if not keep.any():
        return None, (by0 - 2.0, by1 + 2.0)
    i0, i1 = np.nonzero(keep)[0][[0, -1]]
    poly = [(float(xs[i0]), top)] + [(float(xs[i]), float(min(zc[i], top))) for i in range(i0, i1 + 1)] + [(float(xs[i1]), top)]
    return poly, (by0 - 2.0, by1 + 2.0)


def keep_rearmost(o, to_layout):
    """delete every loose piece of o except the one holding its rearmost (lowest layout x) vertex; returns pieces removed"""
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.verts.ensure_lookup_table()
    start = min(bm.verts, key=lambda v: to_layout(o.matrix_world @ v.co).x)
    keep, st = {start.index}, [start]
    while st:
        a = st.pop()
        for e in a.link_edges:
            b = e.other_vert(a)
            if b.index not in keep:
                keep.add(b.index)
                st.append(b)
    drop = [v for v in bm.verts if v.index not in keep]
    bmesh.ops.delete(bm, geom=drop, context="VERTS")
    bm.to_mesh(o.data)
    bm.free()
    return len(drop)


def prism(poly, y0, y1, to_world):
    """closed prism: the x-z polygon `poly` (layout mm) between y0 and y1"""
    bm = bmesh.new()
    A = [bm.verts.new(to_world(Vector((x, y0, z)))) for x, z in poly]
    B = [bm.verts.new(to_world(Vector((x, y1, z)))) for x, z in poly]
    n = len(poly)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((A[i], A[j], B[j], B[i]))
    bm.faces.new(A[::-1])
    bm.faces.new(B)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


args = globals().get("ARGS", {})
if not args.get("lib", True):
    L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
    exec(open(args["joint_lib"]).read(), L)
    R = args.get("robot", "duck_scs0009")
    trunk = bpy.data.objects[R + ":trunk_base"]
    Lw = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
    tgt = bpy.data.objects[args["target"]]
    P = [(Lw.inverted() @ (tgt.matrix_world @ v.co)) / MM for v in tgt.data.vertices]
    box = (min(p.x for p in P), max(p.x for p in P), min(p.y for p in P), max(p.y for p in P), min(p.z for p in P), max(p.z for p in P))
    poly, (y0, y1) = sweep_profile(args["cfg"], box, R)
    if poly is None:
        print(f"{tgt.name}: the sweep does not reach it")
    else:
        print("profile (x, z) every 2 mm:", [(round(x, 1), round(z, 1)) for x, z in poly[1:-1:4]])
        if tgt.data.users > 1:
            tgt.data = tgt.data.copy()
        v0 = L["B"]["_vol"](tgt)
        L["B"]["apply_bool"](tgt, prism(poly, y0, y1, lambda p: Lw @ (p * MM)), "DIFFERENCE", tgt.users_collection[0])
        L["B"]["heal_slivers"](tgt, tgt.name)
        if args["cfg"].get("keep_rear") and L["B"]["shells"](tgt) > 1:
            print(f"  front piece(s) removed: {keep_rearmost(tgt, lambda w: (Lw.inverted() @ w) / MM)} vertices")
        print(f"{tgt.name}: volume {v0:.0f} -> {L['B']['_vol'](tgt):.0f} mm3, shells {L['B']['shells'](tgt)}")
