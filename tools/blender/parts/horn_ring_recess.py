"""Recess for the kit horn's top-face ring, cut IN PLACE into the robot's existing horn-seat parts (runs inside Blender).

The joint library (joint_v2.top_ring_recess) cuts it in every part built from now on; this applies the same cut to the parts already
in the .blend without rebuilding them (the head base / dome cannot be rebuilt - data/build_args/README.md 4b). For each servo horn
screw (the '<joint>_screw_M2x4kit' stand-ins + the mouth's): the part its head sits on is found by a ray along the screw axis, its
floor (counterbore floor -> pocket roof) is measured, and a cylinder TOP_RING_D x (0.1 below the roof .. TOP_RING_H above it) is
subtracted (exact boolean). Seats whose floor is not 0.8-1.2 mm are reported and left alone. Idempotent: the part keeps a flag per
screw. Exploded copies ('.x') keep their own meshes until the exploded views are regenerated.

ARGS = {"d": 5.2, "h": 0.35, "screws": null (all) | [names], "dry_run": false}
"""
import math

import bmesh
import bpy
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

args = globals().get("ARGS", {})
MM = 0.001
D, H = args.get("d", 5.2), args.get("h", 0.35)
S = bpy.data.scenes["Scene"]
dg = bpy.context.evaluated_depsgraph_get()
names = args.get("screws") or sorted(o.name for o in S.objects if o.name.endswith("screw_M2x4kit") or o.name == "scs_mouth_horn_screw")
SKIP = set(names) | {"cutter"}
parts = [o for o in S.objects if o.type == "MESH" and o.name not in SKIP and not o.name.endswith((".x", ".hx"))
         and not o.name.startswith(("wire_", "scs_", "duck_scs0009", "cable_")) and o.visible_get() and len(o.data.polygons)]
_tree = {}


def tree(o):
    if o.name not in _tree:
        bm = bmesh.new()
        bm.from_mesh(o.data)                                       # the mesh itself (parts carry no modifiers; stays valid after a cut)
        bm.transform(o.matrix_world)
        _tree[o.name] = BVHTree.FromBMesh(bm)
        bm.free()
    return _tree[o.name]


def seat_of(o):
    """(head underside centre, unit vector from the head toward the tip): the stand-in is axis-aligned, head 1.3 long"""
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
    into = -ax if head_hi else ax
    top = c.copy()
    top[k] = hi[k] if head_hi else lo[k]
    return top + into * 1.3 * MM, into


def first_hits(p, d, maxd=0.012):
    """all surfaces along the ray p + t d (any part), nearest first: [(dist, part)]"""
    out = []
    for o in parts:
        bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
        if any(p[i] < min(q[i] for q in bb) - maxd or p[i] > max(q[i] for q in bb) + maxd for i in range(3)):
            continue
        t = tree(o)
        s = p
        for _ in range(6):
            h = t.ray_cast(s, d, maxd)
            if h[0] is None:
                break
            dist = (h[0] - p).length
            out.append((dist, o))
            s = h[0] + d * 1e-6
    return sorted(out, key=lambda x: x[0])


def mesh_stats(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    vol, nm = abs(bm.calc_volume()), sum(not e.is_manifold for e in bm.edges)
    seen, shells = set(), 0
    for f in bm.faces:                                            # shells = face islands
        if f.index in seen:
            continue
        shells += 1
        stack = [f]
        seen.add(f.index)
        while stack:
            g = stack.pop()
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        stack.append(h)
    bm.free()
    return vol, nm, shells


SOLVERS = [i.identifier for i in bpy.types.BooleanModifier.bl_rna.properties["solver"].enum_items]
TRY = [s for s in ("MANIFOLD", "EXACT") if s in SOLVERS]
report = []
for n in names:
    scr = bpy.data.objects[n]
    if scr.get("top_ring_recess_done"):
        report.append(f"{n}: already recessed ({scr['top_ring_recess_done']})")
        continue
    seat, into = seat_of(scr)
    out = -into
    u = out.cross(Vector((1, 0, 0)) if abs(out.x) < 0.9 else Vector((0, 1, 0))).normalized()
    # the seat is found on the PART (a stand-in screw can sit off its seat: the mouth's is 1.1 deep in the jaw): from 1.5 mm out on
    # the head side, r 2.0 (on the counterbore floor, outside the Ø2.4 hole), the first part wall 0.8-1.2 thick = the floor
    p = seat + out * 1.5 * MM + u * 2.0 * MM
    hits = first_hits(p, into)
    pick = next(((hits[i][0], hits[i][1], hits[i + 1][0]) for i in range(len(hits) - 1)
                 if hits[i][1] is hits[i + 1][1] and 0.8 <= (hits[i + 1][0] - hits[i][0]) / MM <= 1.2), None)
    if pick is None:
        report.append(f"{n}: no 0.8-1.2 floor found along its axis - skipped ({[(round(d / MM, 2), o.name) for d, o in hits[:4]]})")
        continue
    d0, part, d1 = pick
    floor = (d1 - d0) / MM
    off = (d0 / MM) - 1.5
    if abs(off) > 0.05:
        report.append(f"{n}: NOTE the stand-in screw's head is {off:+.2f} mm off the seat on {part.name} (model display only)")
    roof = p + into * d1 - u * 2.0 * MM                             # pocket roof (underside of the floor), on the axis
    if args.get("dry_run"):
        report.append(f"{n}: would recess {part.name} (floor {floor:.2f} -> {floor - H:.2f})")
        continue
    a, b = roof + into * 0.1 * MM, roof + out * H * MM              # cutter: 0.1 below the roof (in the pocket) .. H above it
    bm = bmesh.new()
    M = Matrix.Translation((a + b) / 2) @ out.to_track_quat("Z", "Y").to_matrix().to_4x4()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=40, radius1=D / 2 * MM, radius2=D / 2 * MM, depth=(b - a).length, matrix=M)
    me = bpy.data.meshes.new("ring_recess_cutter")
    bm.to_mesh(me)
    bm.free()
    cut = bpy.data.objects.new("ring_recess_cutter", me)
    S.collection.objects.link(cut)
    old = part.data
    vol0, nm0, sh0 = mesh_stats(old)
    done = None
    for solver in TRY:
        mod = part.modifiers.new("ring_recess", "BOOLEAN")
        mod.operation, mod.solver, mod.object = "DIFFERENCE", solver, cut
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        new = bpy.data.meshes.new_from_object(part.evaluated_get(dg))
        part.modifiers.remove(mod)
        vol1, nm1, sh1 = mesh_stats(new)
        removed = (vol0 - vol1) * 1e9
        if nm1 <= nm0 and sh1 == sh0 and 2.0 < removed < 12.0:
            done = (solver, new, vol1, nm1, sh1, removed)
            break
        report.append(f"{n}: {solver} solver gave non-manifold {nm1} (was {nm0}), shells {sh1} (was {sh0}), removed {removed:.1f} mm3 - not used")
        bpy.data.meshes.remove(new)
    bpy.data.objects.remove(cut, do_unlink=True)
    bpy.data.meshes.remove(me)
    if done is None:
        report.append(f"{n}: {part.name} NOT recessed (no solver gave a clean result)")
        continue
    solver, new, vol1, nm1, sh1, removed = done
    if not len(new.materials):                                     # new_from_object keeps them; copy only if it did not
        for m in old.materials:
            new.materials.append(m)
    oname = old.name
    old.name = oname + ".pre_ring_recess"                          # the exploded '.x' copies keep this one until regenerated
    new.name = oname
    part.data = new
    _tree.pop(part.name, None)
    after = [h for h in first_hits(p, into) if h[1] is part]
    f2 = ((after[1][0] - after[0][0]) / MM) if len(after) > 1 else float("nan")
    scr["top_ring_recess_done"] = part.name
    report.append(f"{n}: {part.name} recessed ({solver}): floor {floor:.2f} -> {f2:.2f} mm, removed {removed:.1f} mm3, "
                  f"shells {sh1}, non-manifold {nm1} (was {nm0})")
print("horn ring recess (d %.2f, h %.2f):\n  " % (D, H) + "\n  ".join(report))
