"""Rebuild the joints' horn-screw stand-ins (runs inside Blender) as the screw actually used, seated on the part's real counterbore floor.

2026-11-25: the servo's own horn screw (~3.5 mm, user's photos) leaves ~1.3 mm of thread in the shaft (every seat is 2.2 mm from the
head seat to the spline tip); the joints now use a bought M2 x 6 button head (ISO 7380: head Ø3.5 x 1.3 - the same head envelope the
stand-ins always had), 3.8 mm in the shaft, 1.2 short of the hole's bottom. The object names ('<joint>_screw_M2x4kit',
'scs_mouth_horn_screw') are kept: checks and animations refer to them; each carries a 'screw' property with the real part.

The seat is found on the part (the mouth's old stand-in sat 1.1 mm deep in the jaw): from 1.5 mm out on the head side, at r 2.0, the
first part surface along the screw axis = the counterbore floor. Idempotent.

ARGS = {"length": 6.0, "head": [3.5, 1.3], "screw": "M2 x 6 button head (ISO 7380)", "names": null (all)}
"""
import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

args = globals().get("ARGS", {})
MM = 0.001
LEN, (HD, HH) = args.get("length", 6.0), args.get("head", (3.5, 1.3))
S = bpy.data.scenes["Scene"]
names = args.get("names") or sorted(o.name for o in S.objects if o.name.endswith("screw_M2x4kit") or o.name == "scs_mouth_horn_screw")
parts = [o for o in S.objects if o.type == "MESH" and o.name not in names and o.visible_get() and len(o.data.polygons)
         and not o.name.endswith((".x", ".hx")) and not o.name.startswith(("wire_", "scs_", "duck_scs0009", "cable_")) and "screw" not in o.name]
_t = {}


def tree(o):
    if o.name not in _t:
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.transform(o.matrix_world)
        _t[o.name] = BVHTree.FromBMesh(bm)
        bm.free()
    return _t[o.name]


def axis_of(o):
    """(head top centre, unit vector head -> tip) of an axis-aligned stand-in (the wider end is the head)"""
    vs = [o.matrix_world @ v.co for v in o.data.vertices]
    lo = Vector([min(v[i] for v in vs) for i in range(3)])
    hi = Vector([max(v[i] for v in vs) for i in range(3)])
    k = max(range(3), key=lambda i: hi[i] - lo[i])
    c = (lo + hi) / 2
    ax = Vector((0, 0, 0))
    ax[k] = 1.0

    def spread(end):
        e = [v for v in vs if abs(v[k] - end) < 0.05 * MM]
        return max(((v - c) - ax * (v - c).dot(ax)).length for v in e)
    head_hi = spread(hi[k]) > spread(lo[k])
    top = c.copy()
    top[k] = hi[k] if head_hi else lo[k]
    return top, (-ax if head_hi else ax)


def first_part_hit(p, d, maxd=0.012):
    best = (maxd, None)
    for o in parts:
        bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
        if any(p[i] < min(q[i] for q in bb) - maxd or p[i] > max(q[i] for q in bb) + maxd for i in range(3)):
            continue
        h = tree(o).ray_cast(p, d, maxd)
        if h[0] is not None and h[3] < best[0]:
            best = (h[3], o)
    return best


report = []
for n in names:
    o = bpy.data.objects[n]
    top, into = axis_of(o)
    old_seat = top + into * 1.3 * MM if o.get("screw") is None else top + into * o.get("head_h", 1.3) * MM
    out = -into
    u = out.cross(Vector((1, 0, 0)) if abs(out.x) < 0.9 else Vector((0, 1, 0))).normalized()
    p = old_seat + out * 1.5 * MM + u * 2.0 * MM
    d, part = first_part_hit(p, into)
    if part is None:
        report.append(f"{n}: no seat found - left as it was")
        continue
    seat = p + into * d - u * 2.0 * MM
    shift = (seat - old_seat).dot(into) / MM
    bm = bmesh.new()
    q = out.to_track_quat("Z", "Y").to_matrix().to_4x4()
    Mi = o.matrix_world.inverted()
    for r, z0, z1 in ((HD / 2, 0.0, HH), (1.0, -LEN, 0.0)):              # head above the seat, shank below it (along out)
        M = Mi @ Matrix.Translation(seat + out * (z0 + z1) / 2 * MM) @ q
        bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=r * MM, radius2=r * MM, depth=(z1 - z0) * MM, matrix=M)
    me = o.data
    bm.to_mesh(me)
    bm.free()
    me.update()
    o["screw"] = args.get("screw", "M2 x 6 button head (ISO 7380)")
    o["length_mm"], o["head_h"] = LEN, HH
    report.append(f"{n}: seat on {part.name}" + (f" (stand-in moved {shift:+.2f} mm to it)" if abs(shift) > 0.05 else "") +
                  f"; now {o['screw']}, tip {LEN - 2.2:.1f} mm past the spline tip if the seat is 2.2 above it")
print("horn screw stand-ins:\n  " + "\n  ".join(report))
