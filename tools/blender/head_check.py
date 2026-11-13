"""Head v2 fit and motion check (exact triangle tests, no voxels). BVHTree.overlap only proposes candidate pairs
(it over-reports when the parts are not axis-aligned); every candidate is confirmed by an edge-through-triangle test. Runs inside Blender; restores the pose when done.

  1. parts: one solid piece, watertight
  2. rest pose: every contact between head parts and between head and neck-side parts (screws in their holes and the
     roll joint's own interface are listed as intended), smallest gaps for the pairs in ARGS["gaps"]
  3. head roll sweep, 4. jaw sweep, 5. head yaw x roll grid

ARGS = {"robot": "duck_scs0009", "collection": "head_shell_v2", "tests": ["rest", "roll", "jaw", "yawroll"],
        "jaw_deg": 30, "gaps": [["scs_head_roll_case", "head_dome"], ...]}
"""
import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
R = args.get("robot", "duck_scs0009")
TESTS = args.get("tests", ["rest", "roll", "jaw", "yawroll"])
head = bpy.data.objects[R + ":jaw_soft"]
piv = bpy.data.objects.get("head_jaw_pivot")
joints = {e["joint"]: e for e in bpy.data.objects if e.name.startswith(R + ":") and e.type == "EMPTY" and "joint" in e and e.users_collection}   # (in the scene: not a stray copy pulled in by a library load)
q0 = {n: e.rotation_quaternion.copy() for n, e in joints.items()}
SKIP = ("wm_", "wire_", "cable_", R + ":", "xgoduck_original:", "fit_", "orig_", "row_")
SKIPC = ("orig_head_parts", "orig_head_row", "head_fit", "foot_compare", "thigh_compare")


def pose(q):
    for jn, e in joints.items():
        e.rotation_quaternion = q0[jn] @ Quaternion(Vector(e["joint_axis"]), q.get(jn, 0.0))
    bpy.context.view_layer.update()


def under(o, p):
    while o:
        if o == p:
            return True
        o = o.parent
    return False


def usable(o):
    return (o.type == "MESH" and not o.hide_get() and not o.name.startswith(SKIP)
            and not any(c.name in SKIPC or "explod" in c.name or c.name.startswith("sec_") for c in o.users_collection))


def local_bm(o, frame_inv):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)      # BEFORE the frame change: n-gons with holes triangulate wrongly once rotated
    bm.transform(Matrix.Scale(1000, 4) @ frame_inv @ o.matrix_world)
    return bm


def tris(bm):
    return np.array([[v.co[:] for v in f.verts] for f in bm.faces], dtype=np.float64).reshape(-1, 3, 3)


def _seg_tri(p, q, a, b, c):
    d, e1, e2 = q - p, b - a, c - a
    h = np.cross(d, e2)
    det = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(det) > 1e-12
    f = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    s_ = p - a
    u = f * np.einsum("ij,ij->i", s_, h)
    qv = np.cross(s_, e1)
    v = f * np.einsum("ij,ij->i", d, qv)
    t = f * np.einsum("ij,ij->i", e2, qv)
    return ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t >= 0) & (t <= 1)


def crossing(ta, TA, tb, TB):
    """number of triangle pairs of A and B that really cross"""
    ov = ta.overlap(tb)
    if not ov:
        return 0
    ia, ib = np.array([i for i, _ in ov]), np.array([j for _, j in ov])
    A, B = TA[ia], TB[ib]
    hit = np.zeros(len(ov), bool)
    for k in range(3):
        hit |= _seg_tri(A[:, k], A[:, (k + 1) % 3], B[:, 0], B[:, 1], B[:, 2])
        hit |= _seg_tri(B[:, k], B[:, (k + 1) % 3], A[:, 0], A[:, 1], A[:, 2])
    return int(hit.sum())


Hi = head.matrix_world.inverted()
H = [o for o in bpy.context.view_layer.objects if usable(o) and under(o, head)]
JAW = [o for o in H if piv and under(o, piv)]
HS = [o for o in H if o not in JAW]
F = []
for o in bpy.context.view_layer.objects:
    if not usable(o) or under(o, head):
        continue
    P = [Hi @ (o.matrix_world @ Vector(c)) for c in o.bound_box]
    if min(p.length for p in P) < 0.075:                     # near the head (head frame, metres)
        F.append(o)
htree, htri = {}, {}
for o in HS:                                                            # static in the head's frame
    bm_ = local_bm(o, Hi)
    htree[o.name], htri[o.name] = BVHTree.FromBMesh(bm_), tris(bm_)
    bm_.free()
INTENDED = ("screw", "brg", "horn", "spline", "nut")


def intended(a, b):
    return any(k in a or k in b for k in INTENDED)


def contacts_fixed():
    """neck-side parts against the head parts, in the head's frame, at the current pose"""
    out = []
    Hi_ = head.matrix_world.inverted()
    for f in F:
        bm = local_bm(f, Hi_)
        t, tf = BVHTree.FromBMesh(bm), tris(bm)
        bm.free()
        for n, ht in htree.items():
            if crossing(ht, htri[n], t, tf):
                out.append((n, f.name))
        if piv:
            for j in JAW:
                bj = local_bm(j, Hi_)
                if crossing(BVHTree.FromBMesh(bj), tris(bj), t, tf):
                    out.append((j.name, f.name))
                bj.free()
    return out


def fmt(vals):
    vals = sorted(set(vals))
    return f"{vals[0]:g}..{vals[-1]:g} ({len(vals)} poses)" if len(vals) > 1 else f"{vals[0]:g}"


try:
    pose({})
    print(f"head parts {len(H)} (jaw side {len(JAW)}), neck-side parts near the head {len(F)}")
    print("1) printed parts:")
    for n in ("head_base", "head_dome", "head_jaw"):
        o = bpy.data.objects.get(n)
        if not o:
            continue
        bm = bmesh.new()
        bm.from_mesh(o.data)
        nm = sum(1 for e in bm.edges if not e.is_manifold)
        seen, k = set(), 0
        for v in bm.verts:
            if v.index in seen:
                continue
            k += 1
            st = [v]
            seen.add(v.index)
            while st:
                w = st.pop()
                for e in w.link_edges:
                    u = e.other_vert(w)
                    if u.index not in seen:
                        seen.add(u.index)
                        st.append(u)
        P = np.array([(Hi @ (o.matrix_world @ v.co))[:] for v in o.data.vertices]) * 1000
        print(f"   {n}: {bm.calc_volume() * 1e9 * abs(o.matrix_world.determinant()) * 1.24e-3:.1f} g solid PLA, pieces {k}, open/non-manifold edges {nm}, "
              f"size {np.ptp(P[:, 0]):.1f} x {np.ptp(P[:, 1]):.1f} x {np.ptp(P[:, 2]):.1f} mm")
        bm.free()
    if "rest" in TESTS:
        print("2) rest pose contacts:")
        jt, jtri = {}, {}
        for o in JAW:
            bm_ = local_bm(o, Hi)
            jt[o.name], jtri[o.name] = BVHTree.FromBMesh(bm_), tris(bm_)
            bm_.free()
        allt, alltri = dict(htree, **jt), dict(htri, **jtri)
        names = sorted(allt)
        real, okc = [], []
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if a.split("_")[0:2] == b.split("_")[0:2] and a.startswith("scs_"):
                    continue                                   # parts of one servo model
                n = crossing(allt[a], alltri[a], allt[b], alltri[b])
                if n:
                    (okc if intended(a, b) else real).append(f"{a} x {b} ({n})")
        for n, f in contacts_fixed():
            (okc if intended(n, f) else real).append(f"{n} x {f} [neck side]")
        print("   intended (screws in holes, bearing/horn seats):", len(okc))
        for c in okc:
            print("      ", c)
        print("   NOT intended:", len(real))
        for c in real:
            print("      ", c)
        for a, b in args.get("gaps", []):
            oa, ob = bpy.data.objects.get(a), bpy.data.objects.get(b)
            if not oa or not ob:
                continue
            bm = bmesh.new()
            bm.from_mesh(ob.data)
            bm.transform(ob.matrix_world)
            t = BVHTree.FromBMesh(bm)
            d = min(t.find_nearest(oa.matrix_world @ v.co)[3] for v in oa.data.vertices)
            bm.free()
            print(f"   gap {a} -> {b}: {d * 1000:.2f} mm (from {a}'s vertices)")
    if "roll" in TESTS:
        print("3) head roll sweep (1 deg steps):")
        lo, hi = joints["head_roll"]["joint_range"]
        res = {}
        for deg in range(round(math.degrees(lo)), round(math.degrees(hi)) + 1):
            pose({"head_roll": math.radians(deg)})
            for n, f in contacts_fixed():
                if not intended(n, f):
                    res.setdefault((n, f), []).append(deg)
        pose({})
        for (n, f), v in sorted(res.items()):
            print(f"   CONTACT {n} x {f}: roll {fmt(v)}")
        if not res:
            print("   no contact at any angle")
    if "jaw" in TESTS and piv:
        print("4) jaw sweep (positive = open), head level:")
        m0 = piv.matrix_local.copy()
        sgn = args.get("jaw_sign", 1)
        res = {}
        Fl, Ftri = {}, {}
        for f in F:
            bm_ = local_bm(f, Hi)
            Fl[f.name], Ftri[f.name] = BVHTree.FromBMesh(bm_), tris(bm_)
            bm_.free()
        alltri2 = dict(htri, **Ftri)
        for deg in range(0, args.get("jaw_deg", 30) + 1):
            piv.matrix_local = m0 @ Matrix.Rotation(math.radians(sgn * deg), 4, "Z")
            bpy.context.view_layer.update()
            zmin = 1e9
            for j in JAW:
                bj = local_bm(j, Hi)
                tj, tjt = BVHTree.FromBMesh(bj), tris(bj)
                if j.name == "head_jaw":
                    zmin = min((head.matrix_world @ (v.co / 1000)).z for v in bj.verts) * 1000
                for n, t in list(htree.items()) + list(Fl.items()):
                    if not intended(j.name, n) and crossing(tj, tjt, t, alltri2[n]):
                        res.setdefault((j.name, n), []).append(deg)
                bj.free()
            if deg % 10 == 0:
                print(f"   {deg:3d} deg: lowest point of the jaw at world z {zmin:.1f} mm")
        piv.matrix_local = m0
        bpy.context.view_layer.update()
        for (n, f), v in sorted(res.items()):
            print(f"   CONTACT {n} x {f}: jaw {fmt(v)}")
        if not res:
            print("   no contact at any angle")
    if "yawroll" in TESTS:
        print("5) head yaw x head roll:")
        lo, hi = joints["head_yaw"]["joint_range"]
        rolls = (-25, -15, -5, 0, 5, 15, 25)
        grid = {}
        for yd in range(round(math.degrees(lo)), round(math.degrees(hi)) + 1, 10):
            for rd in rolls:
                pose({"head_yaw": math.radians(yd), "head_roll": math.radians(rd)})
                c = [(n, f) for n, f in contacts_fixed() if not intended(n, f)]
                grid[(yd, rd)] = c
        pose({})
        print("   yaw\\roll " + " ".join(f"{r:4d}" for r in rolls) + "   (. free, X contact)")
        for yd in sorted({k[0] for k in grid}):
            print(f"   {yd:5d}    " + " ".join("   ." if not grid[(yd, r)] else "   X" for r in rolls))
        pairs = {}
        for (yd, rd), c in grid.items():
            for p in c:
                pairs.setdefault(p, []).append((yd, rd))
        for p, v in sorted(pairs.items(), key=lambda kv: -len(kv[1]))[:14]:
            print(f"   {p[0]} x {p[1]}: {len(v)} poses, e.g. yaw {v[0][0]} roll {v[0][1]}")
finally:
    pose({})
    if piv and "m0" in dir():
        piv.matrix_local = m0
