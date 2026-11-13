"""Original-style SCS0009 joint (XGO-Duck pattern): one moving plate + a 16x22x4 bearing around
the output, set in the holding link's cap plate. Runs inside Blender.

Frame = SCS0009 servo frame (parts/scs0009.py): origin on the shaft at the case top, +Z out of
the shaft, +X toward the body's far end. Units: metres. All heights below are mm in this frame.

  cap   (holding link)  z 0..CAP_T on the servo's front face; Ø22.1 x 4 bearing seat from the top
                        with a 1 mm lip (Ø18 hole); two pads hang down to the ears (screws from
                        below, M2x6 self-tap, servo kit). The servo goes in from below.
  bearing 16x22x4       z LIP..LIP+4
  link  (moving link)   boss Ø15.95 into the bearing's inner ring (z 0.5..PLATE_Z0), a Ø17.6 shoulder
                        on the inner ring's top face, plate z PLATE_Z0..PLATE_Z1. Inside the boss: the X
                        horn (arms trimmed to TRIM_R) in a crush-rib pocket; M2x6 machine screw in a
                        counterbore into the shaft thread.

Functions are importable (exec this file into a dict) so link builders can place these features
in any servo frame: cap_solid(bm_builder), link_solid(...), etc. Run directly with
ARGS = {"name": "oj", "collection": "orig_joint", "location": (x,y,z), "servo_script": "/abs/scs0009.py"}
to build a demo joint with the servo, bearing and screws.
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

MM = 0.001
# servo (datasheet, joint frame mm)
EAR_X = (-10.55, 21.95)          # ear tips along X
BODY_X = (-5.95, 17.35)
HOLE_XS = (-8.55, 19.95)         # ear holes
EAR_W, EAR_Z = 11.0, (-8.45, -6.85)
BODY_HW = 6.05
# horn (trimmed X horn; NOMINAL widths until measured)
TRIM_R = 6.0                     # arms cut flat at this radius so the horn fits inside the bearing bore
X_W_ROOT, X_W_TIP, X_L_FULL = 5.6, 3.6, 10.5             # untrimmed arm: root width, tip width, length
X_W_TIP_AT_TRIM = X_W_ROOT + (X_W_TIP - X_W_ROOT) * TRIM_R / X_L_FULL
HUB_D, HORN_TOP, HORN_PLATE_T = 6.5, 4.3, 1.6
HORN_CLEAR, RIB_INTERF, RIB_R = 0.20, 0.25, 0.45
# bearing 16x22x4
BRG_ID, BRG_OD, BRG_W = 16.0, 22.0, 4.0
# cap (holding link)
CAP_T = 5.0
LIP = 1.0                        # bearing sits on a 1 mm lip
SEAT_D = BRG_OD + 0.1
LIP_HOLE_D = 18.0
CAP_HW = BODY_HW + 1.6           # cap half-width across the servo
CAP_RING_R = (BRG_OD + 0.1) / 2 + 2.2   # material ring around the bearing seat
PILOT_D = 1.7                    # (2026-11-13: 1.6 -> 1.7, FDM holes print small)
# link (moving)
BOSS_D = BRG_ID - 0.05           # light press into the inner ring
BOSS_Z0 = 0.5
SHOULDER_D = 17.6
PLATE_Z0 = LIP + BRG_W + 0.4     # 5.4
PLATE_T = 4.0
PLATE_Z1 = PLATE_Z0 + PLATE_T    # 9.4
CB_D, CB_Z = 4.2, 6.0            # counterbore for the M2x6 machine screw head
CLEAR_D = 2.4                    # (2026-11-13: 2.3 -> 2.4)


def J(x, y, z):
    return Vector((x * MM, y * MM, z * MM))


def new_bm():
    return bmesh.new()


def add_box(bm, x0, x1, y0, y1, z0, z1, M=None):
    out = bmesh.ops.create_cube(bm, size=1.0)
    for v in out["verts"]:
        v.co = J(x0 if v.co.x < 0 else x1, y0 if v.co.y < 0 else y1, z0 if v.co.z < 0 else z1)
    if M is not None:
        bmesh.ops.transform(bm, matrix=M, verts=out["verts"])
    return bm


def add_cyl(bm, cx, cy, z0, z1, d, segs=48, M=None):
    out = bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=d / 2 * MM, radius2=d / 2 * MM, depth=(z1 - z0) * MM)
    bmesh.ops.translate(bm, verts=out["verts"], vec=J(cx, cy, (z0 + z1) / 2))
    if M is not None:
        bmesh.ops.transform(bm, matrix=M, verts=out["verts"])
    return bm


def add_annulus(bm, r_in, r_out, z0, z1, M=None, segs=64):
    """closed annular prism (axis Z) - e.g. to trim everything outside a boss above a plane"""
    rings = []
    for r, z in ((r_out, z0), (r_out, z1), (r_in, z1), (r_in, z0)):
        rings.append([bm.verts.new(J(r * math.cos(2 * math.pi * i / segs), r * math.sin(2 * math.pi * i / segs), z)) for i in range(segs)])
    for a, b in zip(rings, rings[1:] + rings[:1]):
        for i in range(segs):
            j = (i + 1) % segs
            bm.faces.new((a[i], a[j], b[j], b[i]))
    new_faces = [f for f in bm.faces if any(v in f.verts for v in rings[0])] + [f for f in bm.faces if any(v in f.verts for v in rings[2])]
    bmesh.ops.recalc_face_normals(bm, faces=list(set(new_faces)))
    if M is not None:                      # only the new verts: the bmesh may already hold other (placed) shapes
        bmesh.ops.transform(bm, matrix=M, verts=[v for r in rings for v in r])
    return bm


def add_hull(bm, pts_mm, M=None):
    tmp = bmesh.new()
    for p in pts_mm:
        tmp.verts.new(J(*p))
    bmesh.ops.convex_hull(tmp, input=tmp.verts)
    me = bpy.data.meshes.new("_hull_tmp")
    tmp.to_mesh(me)
    tmp.free()
    if M is not None:
        me.transform(M)
    bm.from_mesh(me)
    bpy.data.meshes.remove(me)
    return bm


def x_pocket_cutters(M=None, z0=-0.5, z1=None):
    """List of bmeshes to subtract: trimmed X arms, tips, hub, plus the rib bumps to ADD back."""
    z1 = HORN_TOP + 0.1 if z1 is None else z1
    c = HORN_CLEAR
    # hub first; every cutter gets slightly different end planes: coincident faces between
    # overlapping cutters make the exact boolean fail (often silently returning an empty mesh)
    cuts = [add_cyl(new_bm(), 0, 0, z0 - 0.07, z1 + 0.05, HUB_D + 2 * c, 40, M)]
    for k in range(4):
        rot = Matrix.Rotation(k * math.pi / 2, 4, "Z")
        Mk = (M @ rot) if M is not None else rot
        quad = [(-0.6, -X_W_ROOT / 2 - c), (TRIM_R + c, -X_W_TIP_AT_TRIM / 2 - c),
                (TRIM_R + c, X_W_TIP_AT_TRIM / 2 + c), (-0.6, X_W_ROOT / 2 + c)]      # flat-ended, like the cut horn
        za, zb = z0 - 0.011 * (k + 1), z1 + 0.013 * (k + 1)
        cuts.append(add_hull(new_bm(), [(x, y, z) for x, y in quad for z in (za, zb)], Mk))
    return cuts


def rib_adds(M=None, z_mouth=0.6, z_top=None):
    z_top = HORN_TOP + 0.5 if z_top is None else z_top
    adds = []
    for k in range(4):
        rot = Matrix.Rotation(k * math.pi / 2, 4, "Z")
        Mk = (M @ rot) if M is not None else rot
        along = 0.65 * TRIM_R
        half_w = (X_W_ROOT + (X_W_TIP_AT_TRIM - X_W_ROOT) * 0.65) / 2
        for side in (-1, 1):
            cy = side * (half_w + HORN_CLEAR + RIB_R - (HORN_CLEAR + RIB_INTERF))
            ring = lambda r, z: [(along + r * math.cos(t), cy + r * math.sin(t), z) for t in (2 * math.pi * i / 16 for i in range(16))]
            adds.append(add_hull(new_bm(), ring(0.1, z_mouth) + ring(RIB_R, z_mouth + 0.6) + ring(RIB_R, z_top), Mk))
    return adds


def cap_features(M, include_plate=True):
    """(solids to add, solids to subtract) for the holding link at servo frame M (4x4, world).
    include_plate=False when a larger joining plate already covers the cap (avoids coincident faces)."""
    # cap plate = servo footprint + a ring around the 16x22x4 bearing seat (the seat is wider than the
    # servo; without the ring it cuts the plate in two)
    pts = [(x, y) for x in EAR_X for y in (-CAP_HW, CAP_HW)] + \
          [(CAP_RING_R * math.cos(2 * math.pi * i / 48), CAP_RING_R * math.sin(2 * math.pi * i / 48)) for i in range(48)]
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (0.0, CAP_T)], M)] if include_plate else []
    for x0, x1 in ((EAR_X[0], BODY_X[0] - 0.2), (BODY_X[1] + 0.2, EAR_X[1])):
        add.append(add_box(new_bm(), x0, x1, -EAR_W / 2, EAR_W / 2, EAR_Z[1], 0.6, M))      # pads down to the ears (top sunk into the plate)
    sub = [add_cyl(new_bm(), 0, 0, LIP, CAP_T + 1, SEAT_D, 64, M),                         # bearing seat
           add_cyl(new_bm(), 0, 0, -1, LIP + 0.01, LIP_HOLE_D, 48, M)]                     # lip hole (horn/boss pass)
    for hx in HOLE_XS:
        sub.append(add_cyl(new_bm(), hx, 0, EAR_Z[1] - 0.5, EAR_Z[1] + 4.4, PILOT_D, 16, M))
    return add, sub


def link_features(M, plate_outline_mm=None):
    """(add, subtract) for the moving link at servo frame M. plate_outline_mm: extra 2D points
    (x, y) in this frame the plate must reach (hulled with the boss disc)."""
    disc = [(BOSS_D / 2 + 3.0) * math.cos(t) for t in ()]   # placeholder (kept simple below)
    r = BOSS_D / 2 + 3.0
    pts = [(r * math.cos(2 * math.pi * i / 48), r * math.sin(2 * math.pi * i / 48)) for i in range(48)]
    pts += list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (PLATE_Z0, PLATE_Z1)], M),
           add_cyl(new_bm(), 0, 0, BOSS_Z0, PLATE_Z0 + 1.0, BOSS_D, 64, M),          # tops sunk well into the
           add_cyl(new_bm(), 0, 0, LIP + BRG_W, PLATE_Z0 + 0.6, SHOULDER_D, 48, M)]  # plate: no near-coincident faces
    sub = x_pocket_cutters(M, z0=BOSS_Z0 - 0.5) + [
        add_cyl(new_bm(), 0, 0, BOSS_Z0 - 1.1, PLATE_Z1 + 1.1, CLEAR_D, 20, M),
        add_cyl(new_bm(), 0, 0, CB_Z, PLATE_Z1 + 0.9, CB_D, 24, M)]
    return add, sub, rib_adds(M, z_mouth=BOSS_Z0 + 0.1)


# ------------------------------------------------------------------ rear pivot (servos without a rear shaft)
# The holding part gets a Ø5 boss under the output shaft (printed with it: coaxial by construction);
# a separate Ø10 pivot stub slides over the boss from OUTSIDE, through the 10x15x3 bearing, held by
# one M2x6 screw. (A pin printed on the holder can't be assembled: the U must slide toward the horn
# to seat the X pocket while the pin would need the bearing arm to slide the opposite way.)
STUB_OD, BOSS_D2, BOSS_L = 10.0, 5.0, 3.0


def rear_boss(M, floor_bottom_z):
    """(add, subs) for the holding part: Ø5 boss below its floor + Ø1.6 pilot through boss and floor"""
    add = add_cyl(new_bm(), 0, 0, floor_bottom_z - BOSS_L, floor_bottom_z + 0.3, BOSS_D2, 32, M)
    sub = add_cyl(new_bm(), 0, 0, floor_bottom_z - BOSS_L - 0.5, floor_bottom_z + 1.4, PILOT_D, 16, M)
    return add, sub


def pivot_stub(name, M, floor_bottom_z, bearing_bottom_z, coll, mat=None):
    """separate Ø10 stub: from 0.2 below the floor to just past the bearing, bore over the boss"""
    z1, z0 = floor_bottom_z - 0.2, bearing_bottom_z - 0.2
    stub = to_object(name, add_cyl(new_bm(), 0, 0, z0, z1, STUB_OD, 48, M), coll, mat)
    apply_bool(stub, add_cyl(new_bm(), 0, 0, floor_bottom_z - BOSS_L - 0.15, z1 + 0.5, BOSS_D2 + 0.2, 32, M), "DIFFERENCE", coll)
    apply_bool(stub, add_cyl(new_bm(), 0, 0, z0 - 0.5, z1, CLEAR_D, 16, M), "DIFFERENCE", coll)
    stub["fastener"] = "pivot stub + M2x6 pan screw"
    return stub


def screw_holes(M, pts_xy, z_face, depth_through, pilot_depth, direction=+1, csk=True, cbore=0.0, cbore_d=3.9):
    """Holes along frame Z: clearance through `depth_through` mm starting at z_face going `direction`
    (countersunk if csk; or with a Ø3.9 counterbore `cbore` mm deep for a pan head), then a Ø1.6 pilot
    `pilot_depth` mm further. Returns list of cutters."""
    cuts = []
    for x, y in pts_xy:
        through = pilot_depth <= 0                      # plain through hole: overshoot the far face
        za, zb = z_face - direction * 0.5, z_face + direction * (depth_through + (0.6 if through else 0.0))
        if depth_through > 0:
            cuts.append(add_cyl(new_bm(), x, y, min(za, zb), max(za, zb), CLEAR_D, 16, M))
        if csk and depth_through > 0:   # 90° countersink, 4.2 at the face
            zc = z_face + direction * 1.0
            tmp = new_bm()
            narrow = CLEAR_D - 0.2                     # narrow end inside the hole: no edge shared with it
            out = bmesh.ops.create_cone(tmp, cap_ends=True, segments=22, radius1=(4.4 if direction > 0 else narrow) / 2 * MM,
                                        radius2=(narrow if direction > 0 else 4.4) / 2 * MM, depth=1.2 * MM)
            bmesh.ops.translate(tmp, verts=out["verts"], vec=J(x, y, (z_face - direction * 0.2 + zc) / 2))
            if M is not None:
                bmesh.ops.transform(tmp, matrix=M, verts=out["verts"])
            cuts.append(tmp)
        if cbore > 0:
            zc0, zc1 = z_face - direction * 0.7, z_face + direction * cbore
            cuts.append(add_cyl(new_bm(), x, y, min(zc0, zc1), max(zc0, zc1), cbore_d, 20, M))
        if not through:
            zp0, zp1 = z_face + direction * (depth_through - 0.4), z_face + direction * (depth_through + pilot_depth)
            cuts.append(add_cyl(new_bm(), x, y, min(zp0, zp1), max(zp0, zp1), PILOT_D, 16, M))
    return cuts


# ------------------------------------------------------------------ object helpers
def to_object(name, bm, coll, mat=None):
    old = bpy.data.objects.get(name)
    if old:
        bpy.data.objects.remove(old, do_unlink=True)
    me = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(me)
    bm.free()
    if mat:
        me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def apply_bool(target, bm, op, coll):
    cutter = to_object("_bool_tmp", bm, coll)
    mod = target.modifiers.new("b", "BOOLEAN")
    mod.operation, mod.object, mod.solver = op, cutter, "EXACT"
    with bpy.context.temp_override(object=target, active_object=target):
        bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


DEBUG = []


def _vol(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    v = bm.calc_volume() * 1e9
    bm.free()
    return v


PROBE = {}   # optional: {"point": world Vector} - after each step record whether it is inside the part


def _inside(ob, p):
    from mathutils.bvhtree import BVHTree
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.transform(ob.matrix_world)
    t = BVHTree.FromBMesh(bm)
    bm.free()
    loc, n, _, _ = t.find_nearest(p)
    return loc is not None and (loc - p).dot(n) > 0


def build_part(name, adds, subs, coll, mat=None, rib_list=()):
    """Union of adds, minus subs, plus ribs (added last so they sit inside pockets).
    Volume is tracked after every step; a step that wipes out the part is reported in DEBUG."""
    part = to_object(name, adds[0], coll, mat)
    steps = [("add", bm) for bm in adds[1:]] + [("sub", bm) for bm in subs] + [("rib", bm) for bm in rib_list]
    for i, (kind, bm) in enumerate(steps):
        before = _vol(part)
        apply_bool(part, bm, "DIFFERENCE" if kind == "sub" else "UNION", coll)
        after = _vol(part)
        if after < 0.5 * before:
            DEBUG.append(f"{name}: step {i} ({kind}) volume {before:.0f} -> {after:.0f} mm3")
        if PROBE.get("steps") == name:                                   # debug: manifold check after every step
            _b = bmesh.new()
            _b.from_mesh(part.data)
            _nm = sum(not e.is_manifold for e in _b.edges)
            _b.free()
            DEBUG.append(f"{name}: step {i} ({kind}) vol {before:.0f}->{after:.0f}, non-manifold edges {_nm}")
        if PROBE.get("point") is not None and PROBE.get("part") == name:
            DEBUG.append(f"{name}: step {i} ({kind}) vol {before:.0f}->{after:.0f}, probe inside={_inside(part, PROBE['point'])}")
    heal_slivers(part, name)
    return part


def heal_slivers(ob, name, max_sides=8):
    """Exact booleans occasionally leave a tiny open triangle where two surfaces meet almost
    tangentially. Fill only small boundary holes (<= max_sides edges); larger gaps are real errors
    and are left for the manifold check to report."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    boundary = [e for e in bm.edges if e.is_boundary]
    if boundary:
        filled = bmesh.ops.holes_fill(bm, edges=boundary, sides=max_sides)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bm.to_mesh(ob.data)
        if filled["faces"]:
            DEBUG.append(f"{name}: healed {len(filled['faces'])} boolean sliver(s)")
    bm.free()


def shells(ob):
    """number of separate solid pieces (a printable part must be exactly 1)"""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    seen, n = set(), 0
    for v in bm.verts:
        if v.index in seen:
            continue
        n += 1
        stack = [v]
        seen.add(v.index)
        while stack:
            w = stack.pop()
            for e in w.link_edges:
                o = e.other_vert(w)
                if o.index not in seen:
                    seen.add(o.index)
                    stack.append(o)
    bm.free()
    return n


def manifold(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    ok = all(e.is_manifold for e in bm.edges)
    vol = bm.calc_volume() * 1e6
    bm.free()
    return ok, vol


def material(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


# ------------------------------------------------------------------ demo
if __name__ == "__main__" or "ARGS" in globals():
    args = globals().get("ARGS", {})
    if args.get("demo", True):
        NAME = args.get("name", "oj")
        coll = bpy.data.collections.get(args.get("collection", "orig_joint")) or bpy.data.collections.new(args.get("collection", "orig_joint"))
        if coll.name not in bpy.context.scene.collection.children:
            bpy.context.scene.collection.children.link(coll)
        for o in [o for o in coll.objects if o.name.startswith(NAME + "_")]:
            bpy.data.objects.remove(o, do_unlink=True)
        M = Matrix.Translation(Vector(args.get("location", (0, 0, 0))))
        ca, cs = cap_features(M)
        cap = build_part(f"{NAME}_cap", ca, cs, coll, material("oj_cap", (0.15, 0.17, 0.22, 1)))
        la, ls, lr = link_features(M, [(-38, -6), (-38, 6)])
        link = build_part(f"{NAME}_link", la, ls, coll, material("oj_link", (0.95, 0.95, 0.93, 1)), lr)
        brg = to_object(f"{NAME}_bearing_16x22x4", add_cyl(new_bm(), 0, 0, LIP, LIP + BRG_W, BRG_OD, 64, M), coll, material("oj_steel", (0.6, 0.62, 0.66, 1)))
        apply_bool(brg, add_cyl(new_bm(), 0, 0, LIP - 1, LIP + BRG_W + 1, BRG_ID, 64, M), "DIFFERENCE", coll)
        if args.get("servo_script"):
            ns = {"ARGS": {"name": f"{NAME}_servo", "collection": coll.name, "location": tuple(M.translation), "horn": "cross"}}
            ns["ARGS"]["horn_trim_r"] = TRIM_R
            exec(open(args["servo_script"]).read(), ns)
        for ob in (cap, link):
            ok, vol = manifold(ob)
            print(f"{ob.name}: {vol * 1.24:.2f} g solid PLA, manifold={ok}")
