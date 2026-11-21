"""Move the dome's two REAR screws from x 5.0 to x 1.5 (runs inside Blender, once: flag "rear_screw_x" on head_dome).

Why: the head-roll servo's right ear-screw head was recessed into the side of the dome's rear-right post (an undercut): the dome
could not come down over it (whole-robot path check, 2026-11-20). A full head_shell_v2 base + dome rebuild no longer gives sound
parts (the original source meshes), so this edits the parts in place, with the exact boolean that keeps shared meshes in step:

  head_dome   new posts Ø4.6 at x 1.5 (they merge with the old ones), their Ø1.6 pilots; a slot down the old right post, Ø4.4 round
              the ear-screw head's path from above it to below the dome's edge (that post carries no screw any more); loose
              slivers dropped
  head_base   new bosses Ø5.6 at x 1.5 + a web from each back to the roll servo's ear pad (they grew from it at x 5.0), under the
              ear-screw heads; their clearance holes + counterbores from below (as head_shell_v2's screw_holes)
  dome_screws_M2x6st   rebuilt: rear pair at x 1.5, front pair unchanged

ARGS = {"robot": "duck_scs0009", "joint_lib": ..., "base_lib": ..., "dims": ..., "x_new": 1.5}
"""
import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
T = bpy.data.objects[R + ":trunk_base"].matrix_world.copy()
ML = T @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
cyl, box, new = L["add_cyl"], L["add_box"], L["new_bm"]
DOME_Z0 = 197.9
X0, X1 = 5.0, args.get("x_new", 1.5)
YS = (-100.03 - 22.2, -100.03 + 22.2)
dome, base = bpy.data.objects["head_dome"], bpy.data.objects["head_base"]
head = bpy.data.objects[R + ":jaw_soft"]


def drop_loose(o, keep_min=2000):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    seen, comps = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        st, fs = [f], []
        seen.add(f.index)
        while st:
            g = st.pop()
            fs.append(g)
            for e in g.edges:
                for h in e.link_faces:
                    if h.index not in seen:
                        seen.add(h.index)
                        st.append(h)
        comps.append(fs)
    comps.sort(key=len, reverse=True)
    kill, gone = set(), 0
    for fs in comps[1:]:
        if len(fs) < keep_min:
            kill |= {v for g in fs for v in g.verts}
            gone += 1
    if kill:
        bmesh.ops.delete(bm, geom=list(kill), context="VERTS")
    bm.to_mesh(o.data)
    bm.free()
    return gone


if dome.get("rear_screw_x") is not None and not args.get("force"):
    print(f"head rear screws: already at x {dome['rear_screw_x']} - nothing done")
else:
    # ---- dome
    for y in YS:
        L["bool_shared"](dome, cyl(new(), X1, y, DOME_Z0 + 0.02, DOME_Z0 + 8.0, 4.6, 28, ML), "UNION")
    ear = bpy.data.objects["head_roll_ear_screws_M2x6st"]
    for seat, o, _h in L["screw_heads"](ear):
        if seat.y * 1000 > -110.0:                                             # only the right one meets a post
            continue
        d, room = L["SOCKET_HEAD"][0] + 0.6, L["SOCKET_HEAD"][1] + 0.3
        bm = new()
        out = bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=d / 2 * MM, radius2=d / 2 * MM, depth=(room + 0.1) * MM)
        Rq = o.to_track_quat("Z", "Y").to_matrix().to_4x4()
        bmesh.ops.transform(bm, matrix=Matrix.Translation(seat + o * (room - 0.1) / 2 * MM) @ Rq, verts=out["verts"])
        top = [v.co.copy() for v in bm.verts]
        bm.free()
        zb = (DOME_Z0 - 3.0) * MM
        bm = new()
        for c in top + [Vector((c.x, c.y, zb)) for c in top]:
            bm.verts.new(c)
        bmesh.ops.convex_hull(bm, input=bm.verts)
        L["bool_shared"](dome, bm, "DIFFERENCE")
    for y in YS:
        L["bool_shared"](dome, cyl(new(), X1, y, DOME_Z0 - 0.5, DOME_Z0 + 4.4, 1.6, 16, ML), "DIFFERENCE")
    w = bmesh.new()                                                          # weld the exact boolean's 0.0001 mm leftovers (no shape change)
    w.from_mesh(dome.data)
    bmesh.ops.remove_doubles(w, verts=w.verts, dist=1e-6)
    bmesh.ops.delete(w, geom=[e for e in w.edges if not e.link_faces], context="EDGES")
    bmesh.ops.delete(w, geom=[v for v in w.verts if not v.link_edges], context="VERTS")
    w.to_mesh(dome.data)
    w.free()
    nd = drop_loose(dome)
    dome["rear_screw_x"] = X1
    # ---- head base
    for y in YS:
        L["bool_shared"](base, cyl(new(), X1, y, 191.4, DOME_Z0 - 0.05, 5.6, 32, ML), "UNION")
        L["bool_shared"](base, box(new(), X1, 8.0, y - 2.0, y + 2.0, 191.4, DOME_Z0 - 0.05, ML), "UNION")
    for h in L["screw_holes"](ML, [(X1, y) for y in YS], 191.3, DOME_Z0 - 0.05 - 191.3, 0.0, +1, csk=False, cbore=DOME_Z0 - 0.05 - 191.3 - 2.0):
        L["bool_shared"](base, h, "DIFFERENCE")
    nb = drop_loose(base)
    # ---- the screws
    old = bpy.data.objects.get("dome_screws_M2x6st")
    fronts = [tuple(head["front_screws"][i:i + 2]) for i in (0, 2)]          # (must be the original 73.06 / -130.9, -69.12: a broken full
    #                                                                          rebuild once overwrote them with 73.75 / -131.2, -68.8)
    if old is not None:
        bpy.data.objects.remove(old, do_unlink=True)
    coll = bpy.data.collections["head_shell_v2"]
    scr = L["pan_screw_dummy"]("dome_screws_M2x6st", [(ML, X1, y, DOME_Z0 - 0.05 - 2.0, +1) for y in YS] + [(ML, x, y, 199.9, +1) for x, y in fronts],
                               coll, L["material"]("hw_steel", (0.6, 0.62, 0.66, 1)))
    mw = scr.matrix_world.copy()
    scr.parent = head
    scr.matrix_world = mw
    for o in (dome, base):
        ok, vol = L["manifold"](o)
        print(f"{o.name}: {vol * 1.24:.1f} g, {'manifold' if ok else 'NOT MANIFOLD'}, {L['shells'](o)} shell(s)")
    print(f"head rear screws: x {X0} -> {X1}; slivers dropped: dome {nd}, base {nb}")

# ---- 2026-11-20, second part (own flags): the dome gives way to the base's new bosses + webs (0.1 clear; the full build cuts the
#      whole base out of the dome), and a notch in its front edge for the tips of the visor's two left screws (camera face: they go
#      into nuts behind the board's left end and come out 1.0 mm past them, into the dome's edge there), open downward
def weld(o):
    w = bmesh.new()
    w.from_mesh(o.data)
    bmesh.ops.remove_doubles(w, verts=w.verts, dist=1e-6)
    bmesh.ops.holes_fill(w, edges=[e for e in w.edges if e.is_boundary], sides=8)   # the exact solver's pin-holes (< 0.7 mm)
    bmesh.ops.delete(w, geom=[e for e in w.edges if not e.link_faces], context="EDGES")
    bmesh.ops.delete(w, geom=[v for v in w.verts if not v.link_edges], context="VERTS")
    w.to_mesh(o.data)
    w.free()


if not dome.get("base_giveway"):
    # at x 1.5 the base's tray rim rises to z ~199.1 (at x 5.0 it was under 197.85): the base's clearance holes go on up through it,
    # and each new dome post starts 0.25 above it (a plain Ø4.8 cut of the post's bottom; intersecting the whole base with the dome
    # wrecked the dome's mesh). The M2x6 then has 2.5 mm of thread in the post.
    for y in YS:
        L["bool_shared"](base, cyl(new(), X1, y, DOME_Z0 - 0.5, 201.0, L["CLEAR_D"], 16, ML), "DIFFERENCE")
    for y in YS:
        L["bool_shared"](dome, cyl(new(), X1, y, DOME_Z0 - 1.0, 199.25, 4.8, 28, ML), "DIFFERENCE")   # 199.35 sat on a dome face (199.32)
    dome["base_giveway"] = True
# (no notch for the visor's left screw tips: their heads sit 0.8 deeper in the visor instead, the tips stop 0.25 short of the dome's edge)
weld(dome)
nd2 = drop_loose(dome)
ok, vol = L["manifold"](dome)
print(f"head_dome after give-way + notch: {vol * 1.24:.1f} g, {'manifold' if ok else 'NOT MANIFOLD'}, {L['shells'](dome)} shell(s), slivers dropped {nd2}")
