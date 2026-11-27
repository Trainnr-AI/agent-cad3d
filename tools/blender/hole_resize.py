"""Resize every screw hole in the printed parts, in place (runs inside Blender at the zero pose).

2026-11-27 (user's printed test, tolerance coupon + the v1 bay): FDM holes come out ~0.3-0.4 small - M2 would not go into its holes,
M3 barely through. New sizes (nominal, in the model): M2 clearance 2.4 -> 2.8, M2 pilot 1.7 -> 2.0, M3 clearance 3.3 -> 3.7,
M3 pilot 2.5 -> 2.9. The builders' constants were changed too (joint_v2.py CLEAR_D / PILOT_D / NUT_HOLE_D / M3_PILOT_D), so a rebuild
gives the same; this tool brings the parts in the current .blend there without rebuilding them (their later in-place fixes stay).

How: tools/blender/screw_audit.py (run first, on the OLD sizes) gives every screw's tip, axis, length, the part(s) its thread bites
(-> pilot) and the part(s) its shank passes (-> clearance). Per printed part (a mesh with `mass_g_solid_PLA` in the given collections)
one cutter = the union of new-size cylinders along those screws:
  pilot:     from the pilot's own end (found by a ray from the tip; blind depth kept) to 0.3 past the head seat
  clearance: from 0.3 before the tip to 0.3 past the head seat
Applied with the Manifold solver (Exact as a fallback), kept only if the part keeps its shell count, gets no new non-manifold edges and
loses a plausible volume. Parts sharing one mesh are cut once (cutters in the mesh's frame, from every object using it). The old mesh
is kept as '<name>.pre_holes' (exploded / animation copies keep it until regenerated). Re-running cuts nothing more (same sizes).

ARGS = {"collections": ["leg_v2", "neck_v2", "head_v2", "head_shell_v2", "body_v2"], "audit": "/abs/tools/blender/screw_audit.py",
        "sizes": {"M2": {"pilot": 2.0, "clear": 2.8}, "M3": {"pilot": 2.9, "clear": 3.7}}, "dry_run": false, "json_out": "/abs/out.json"}
"""
import json
import math

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

args = globals().get("ARGS", {})
MM = 0.001
SIZES = args.get("sizes", {"M2": {"pilot": 2.0, "clear": 2.8}, "M3": {"pilot": 2.9, "clear": 3.7}})
OLD = {"M2": {"pilot": 1.7, "clear": 2.4}, "M3": {"pilot": 2.5, "clear": 3.3}}
TAG = "2026-11-27 " + " ".join(f"{k} clear {v['clear']} pilot {v['pilot']}" for k, v in SIZES.items())
S = bpy.data.scenes.get("Scene") or bpy.context.scene

printed = {}
for c in args.get("collections", ["leg_v2", "neck_v2", "head_v2", "head_shell_v2", "body_v2"]):
    col = bpy.data.collections.get(c)
    if col:
        for o in col.all_objects:
            if o.type == "MESH" and "mass_g_solid_PLA" in o and ".x" not in o.name and not o.name.endswith(".asm"):
                if o.get("holes_resized") and not args.get("force"):
                    continue                    # done: its new pilots would read as clearance holes to the audit - never cut twice
                if args.get("only") and o.name not in args["only"]:
                    continue
                printed[o.name] = o

# ---- 1. the screws (screw_audit on the current, OLD holes)
ns = {"__name__": "screw_audit", "ARGS": {"json_out": args.get("json_out", "/tmp/_holes_audit.json").replace(".json", "_audit.json")}}
exec(open(args["audit"]).read(), ns)
rows = ns["rows"]


def tree(o):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.transform(o.matrix_world)
    t = BVHTree.FromBMesh(bm)
    bm.free()
    return t


_t = {}


def T_(o):
    if o.name not in _t:
        _t[o.name] = tree(o)
    return _t[o.name]


# ---- 2. cutters per mesh (in the mesh's own frame)
cuts = {}          # mesh name -> list of (a_local, b_local, d)
plan = []
for r in rows:
    kind = r["kind"]
    tip, ax, L = Vector(r["at"]) * MM, Vector(r["axis"]).normalized(), r["length"]
    seat = tip + ax * L * MM
    for role, parts in (("pilot", r["bite"]), ("clear", r["clamp"])):
        for pn in parts:
            o = printed.get(pn)
            if o is None:
                continue
            if role == "pilot":
                h = T_(o).ray_cast(tip + ax * 0.05 * MM, -ax, 0.015)        # the pilot's end face, from inside the pilot
                a = (h[0] + ax * 0.01 * MM) if h[0] is not None else tip - ax * 3.0 * MM
            else:
                a = tip - ax * 0.3 * MM
            b = seat + ax * 0.3 * MM
            d = SIZES[kind][role]
            Mi = o.matrix_world.inverted()
            cuts.setdefault(o.data.name, []).append((Mi @ a, Mi @ b, d, o.name))
            plan.append(dict(screw=r["screw"], kind=kind, part=pn, role=role, d_old=OLD[kind][role], d_new=d,
                             length=round((b - a).length / MM, 2), blind=(role == "pilot" and h[0] is not None)))


def drop_slivers(me, max_faces=20, max_vol=0.05):
    """remove stray islands of a few faces and ~no volume (a solver artifact far from the cut, e.g. on the head dome's mesh)"""
    bm = bmesh.new()
    bm.from_mesh(me)
    seen, kill, n = set(), [], 0
    for f in bm.faces:
        if f.index in seen:
            continue
        st, fs = [f], []
        seen.add(f.index)
        while st:
            g = st.pop()
            fs.append(g)
            for e in g.edges:
                for k in e.link_faces:
                    if k.index not in seen:
                        seen.add(k.index)
                        st.append(k)
        if len(fs) < max_faces:
            vol = abs(sum(g.verts[0].co.dot(g.verts[i].co.cross(g.verts[i + 1].co)) for g in fs for i in range(1, len(g.verts) - 1)) / 6) / MM ** 3
            if vol < max_vol:
                kill += fs
                n += 1
    if kill:
        bmesh.ops.delete(bm, geom=list({v for g in kill for v in g.verts}), context="VERTS")
        bm.to_mesh(me)
    bm.free()
    return n


def mesh_stats(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    vol, nm = abs(bm.calc_volume()), sum(not e.is_manifold for e in bm.edges)
    seen, shells = set(), 0
    for f in bm.faces:
        if f.index in seen:
            continue
        shells += 1
        st = [f]
        seen.add(f.index)
        while st:
            g = st.pop()
            for e in g.edges:
                for k in e.link_faces:
                    if k.index not in seen:
                        seen.add(k.index)
                        st.append(k)
    bm.free()
    return vol / MM ** 3, nm, shells


SOLVERS = [i.identifier for i in bpy.types.BooleanModifier.bl_rna.properties["solver"].enum_items]
TRY = [s for s in ("MANIFOLD", "EXACT") if s in SOLVERS]
report, results = [], []
for mname, cl in cuts.items():
    me_old = bpy.data.meshes[mname]
    users = [o for o in printed.values() if o.data == me_old]
    # cutter: one cylinder per hole, merged (duplicates from 2 objects on one mesh land on top of each other: harmless)
    bm = bmesh.new()
    exp = 0.0
    for a, b, d, _on in cl:
        axl = (b - a)
        Mc = Matrix.Translation((a + b) / 2) @ axl.normalized().to_track_quat("Z", "Y").to_matrix().to_4x4()
        bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=d / 2 * MM, radius2=d / 2 * MM, depth=axl.length, matrix=Mc)
        exp += math.pi / 4 * d * d * axl.length / MM                   # upper bound of the volume removed (the whole cylinder)
    cme = bpy.data.meshes.new("holes_cutter")
    bm.to_mesh(cme)
    bm.free()
    cut = bpy.data.objects.new("holes_cutter", cme)
    tmp = bpy.data.objects.new("holes_tmp", me_old)
    S.collection.objects.link(cut)
    S.collection.objects.link(tmp)
    vol0, nm0, sh0 = mesh_stats(me_old)
    done = None
    if not args.get("dry_run"):
        for solver in TRY:
            mod = tmp.modifiers.new("holes", "BOOLEAN")
            mod.operation, mod.solver, mod.object = "DIFFERENCE", solver, cut
            dg = bpy.context.evaluated_depsgraph_get()
            dg.update()
            new = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg))
            tmp.modifiers.remove(mod)
            vol1, nm1, sh1 = mesh_stats(new)
            if sh1 > sh0:
                k = drop_slivers(new)
                if k:
                    report.append(f"{mname}: {solver} left {k} stray sliver island(s) (< 20 faces, < 0.05 mm3) - removed")
                    vol1, nm1, sh1 = mesh_stats(new)
            removed = vol0 - vol1
            if nm1 <= nm0 and sh1 == sh0 and -0.05 <= removed <= exp + 0.5:
                done = (solver, new, vol1, nm1, sh1, removed)
                break
            report.append(f"{mname}: {solver} gave non-manifold {nm1} (was {nm0}), shells {sh1} (was {sh0}), removed {removed:.1f} mm3 - not used")
            bpy.data.meshes.remove(new)
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.objects.remove(cut, do_unlink=True)
    bpy.data.meshes.remove(cme)
    holes = len(cl)
    if args.get("dry_run"):
        results.append(dict(mesh=mname, parts=[o.name for o in users], holes=holes, applied=False))
        continue
    if done is None:
        report.append(f"!! {mname} ({', '.join(o.name for o in users)}): NOT resized ({holes} holes) - no clean boolean")
        results.append(dict(mesh=mname, parts=[o.name for o in users], holes=holes, applied=False))
        continue
    solver, new, vol1, nm1, sh1, removed = done
    if not len(new.materials):
        for m in me_old.materials:
            new.materials.append(m)
    old_name = me_old.name
    me_old.name = old_name + ".pre_holes"
    new.name = old_name
    for o in users:
        o.data = new
        o["holes_resized"] = TAG
        o["volume_mm3"] = vol1
        o["mass_g_solid_PLA"] = round(vol1 / 1000 * 1.24, 2)
    report.append(f"{old_name}: {holes} holes ({solver}), removed {removed:.1f} mm3, shells {sh1}, non-manifold {nm1} (was {nm0})"
                  + ("" if len(users) == 1 else f" - shared by {len(users)}: {[o.name for o in users]}"))
    results.append(dict(mesh=old_name, parts=[o.name for o in users], holes=holes, applied=True, solver=solver, removed_mm3=round(removed, 2)))

n_p = sum(1 for p in plan if p["role"] == "pilot")
n_c = len(plan) - n_p
print(f"hole resize ({TAG}): {len(rows)} screws, {n_p} pilots + {n_c} clearance holes in {len(cuts)} printed meshes"
      + (" [DRY RUN]" if args.get("dry_run") else ""))
for line in report:
    print("  " + line)
unprinted = sorted({pn for r in rows for pn in list(r["bite"]) + list(r["clamp"]) if pn not in printed})
print("  not printed (left as they are):", unprinted)
if args.get("json_out"):
    json.dump(dict(tag=TAG, plan=plan, results=results, report=report), open(args["json_out"], "w"), indent=1)
