"""SCS0009 joint features v2 — exact-servo based. Runs inside Blender; exec this file into a dict.

Two joint types (see projects/xgoduck_scs0009/REVIEW.md, "Joints v2"):
  C  cap (one-sided): stator = cap plate on the tower top + ear pads + saddle walls round the body;
     6702ZZ (15x21x4) bearing in the cap; rotor = Ø14.95 boss in the bearing with the X horn TRIMMED to r = 6
     in a crush-rib pocket; M2x6 clamp screw. (As v1 / the original; the only compact coaxial layout.)
  F  fork (two-sided): stator = floor under the servo bottom + ear pads + saddle walls + captive M3
     nut; rotor = horn plate with the FULL X horn in a pocket (any angle) + separate bearing plate
     with a 693ZZ (3x8x4) on an M3 axle screwed into the nut (2x M2 csk to the rotor).
All heights are mm in the servo/joint frame (parts/scs0009_dims.py J_* values): origin on the shaft
axis at the tower top, +Z out of the shaft, +X toward the connector end. M = 4x4 world matrix of that frame.

ARGS = {"base_lib": "/abs/parts/joint_orig_style.py", "dims": "/abs/parts/scs0009_dims.py"}
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

_args = globals().get("ARGS", {})
B = {"__name__": "joint_lib", "ARGS": {"demo": False}}
exec(open(_args["base_lib"]).read(), B)
exec(open(_args["dims"]).read())
for _k in ("J", "new_bm", "add_box", "add_cyl", "add_annulus", "add_hull", "to_object", "apply_bool", "build_part",
           "heal_slivers", "shells", "manifold", "material", "screw_holes", "DEBUG", "MM"):
    globals()[_k] = B[_k]

# ---- shared: horn pocket
HORN_CLEAR, RIB_INTERF, RIB_R = 0.20, 0.25, 0.5      # loose walls; ribs (1 mm wide: printable) take up the play
CLEAR_D, PILOT_D = 2.8, 2.0                          # M2 clearance; M2 pilot. 2026-11-27 (user's printed test: M2 would not go into
#                                                      the 2.4 / 1.7 holes, M3 barely through 3.3 - FDM holes print ~0.3-0.4 small): 2.4 / 1.7 -> 2.8 / 2.0
#                                                      (2026-11-13 was 2.3 / 1.6 -> 2.4 / 1.7). M2 machine screws thread into PLA, the servo's
#                                                      self-tappers too. Existing parts: tools/blender/hole_resize.py.
M3_CLEAR_D, M3_PILOT_D = 3.7, 2.9                    # M3 clearance / thread-forming pilot (2026-11-27: 3.3 / 2.5 -> 3.7 / 2.9)
HUB_D, HORN_TOP, PLATE_T_HORN = HORN_HUB_D, J_HORN_HUB_Z[1], HORN_PLATE_T
# 2026-11-25 (user's printed coupon + photos): the kit horn's TOP face (the side in our pocket) has a small raised ring round its
# screw hole, so the printed part sat on the ring and the servo's screw no longer reached the shaft. Recess in the pocket roof
# round the centre: Ø5.2 (ring read from the photo as Ø4.5-5, +margin; inside the hub's Ø6.9 opening, so the arms still carry)
# x 0.35 above the roof (the ring's height could not be read; 0.35 + the roof's 0.1 clearance = rings up to 0.45 sit free; the
# screw head (Ø3.5) spans most of it). The floor under the screw head is 0.95 at every seat -> 0.60 (3 layers at 0.2) left.
TOP_RING_D, TOP_RING_H = 5.2, 0.35


def top_ring_recess(M):
    """cutter: recess in the horn pocket's roof for the raised ring round the horn's screw hole (its top face)"""
    return add_cyl(new_bm(), 0, 0, HORN_TOP, HORN_TOP + 0.1 + TOP_RING_H, TOP_RING_D, 40, M)
# ---- type C
BRG_ID, BRG_OD, BRG_W = 15.0, 21.0, 4.0              # 6702ZZ (ISO thin-section 61702): sold everywhere. The original
#                                                      duck's 16x22x4 ("ET2216") is a niche size - not used.
BRG_NAME = "6702ZZ"
TRIM_R = 6.0
CAP_T, LIP, LIP_HOLE_D = 5.0, 1.0, BRG_OD - 4.0       # lip carries the outer ring only
SEAT_D = BRG_OD + 0.1
LIP_SQ = 8.6                                         # half-side of the square lip opening (see cap_features)
CAP_HW = J_BODY_HW + 1.6                             # 7.65: cap half-width = saddle wall outer face
SADDLE_CLEAR, SADDLE_DEPTH = 0.25, 8.0               # walls cover the tower (3.8) + 4.2 of body (clearance 0.15 -> 0.25: print + case tolerance)
SADDLE_T = CAP_HW - 0.1 - (J_BODY_HW + SADDLE_CLEAR)   # 1.35: outer face 0.1 inside the plate edge (no coincident faces)
CAP_RING_R = SEAT_D / 2 + 2.2
BOSS_D, BOSS_Z0, SHOULDER_D = BRG_ID - 0.1, 0.5, BRG_ID + 1.6    # shoulder bears on the inner ring only (boss 14.95 -> 14.9: prints ~0.05 big)
PLATE_Z0 = LIP + BRG_W + 0.4                         # 5.4
PLATE_T = 4.5                                        # 4.5 (not 4.0): = FLOOR_T, see there
PLATE_Z1 = PLATE_Z0 + PLATE_T                        # 9.9
# screws (user 2026-11-13: few screw types, all standard; machine screws from Robu; each servo's packaging has its horn screw)
SOCKET_HEAD = (3.8, 2.0)                             # M2 x 6 socket head (Robu EasyMech: dk 3.7, k 2.0): every M2 screw but the joints'
KIT_SCREW = (2.0, 6.0, 3.5, 1.3)                     # 2026-11-25: joint clamp screw = bought M2 x 6 button head (ISO 7380) - the
#                                                      servo's own (~3.5 mm, user's photo) left ~1.3 mm in the shaft (BOM row 21)
KIT_SEAT = HORN_TOP + 0.1 + 1.0                      # 5.4: joint clamp screw head seat, 1.0 of floor over the horn pocket ->
#                                                      M2 x 6: thread 3.8 into the shaft (spline tip 3.2; the shaft hole ends at -1.8)
CB_D, CB_Z = 4.4, KIT_SEAT                           # counterbore 4.4: the kit screw's pan head may be up to Ø4.0
# ---- type F
FLOOR_T = 4.5                                        # = PLATE_T: a type-C rotor plate and a type-F floor can be one plate.
#                                                      4.5: 693 (4.0) + floor = 8.5 > the M3x8 axle, so its tip stops 0.5 short
#                                                      of the servo bottom (with a 4.0 floor it touched it);
#                                                      holds a captive M3 nut (2.4) under the servo bottom
FLOOR_Z = (J_BOTTOM - FLOOR_T, J_BOTTOM)             # -29.75 .. -25.25
NUT_AF, NUT_T, NUT_HOLE_D = 5.5 + 0.25, 2.4 + 0.2, M3_CLEAR_D
RING_ISLAND_D, RING_RECESS_D, RING_RECESS_H = 4.5, 16.0, 0.5   # floor underside: Ø16 x 0.5 recess round a flush Ø4.5
#                                                      island -> the 693's inner ring bears on the island, the bearing plate
#                                                      clears the floor by 0.5, and the floor's face stays flat (print bed)
B693 = (3.0, 8.0, 4.0)
BP_T = 4.8                                           # bearing plate thickness (seat 4.0 + 0.8 lip = 4 layers:
#                                                      the lip is what keeps the rotor on the bearing in a fall)
BP_INNER_Z = FLOOR_Z[0]                              # bearing plate inner face = floor's nominal underside
BP_SEAT_D, BP_LIP_D = B693[1] + 0.1, 6.0
ARM_Z = (2.6, 6.4)                                   # horn plate of a type-F rotor (2.1 above the horn top)
ARM_R_FULL = HORN_CROSS_L + HORN_CLEAR + 1.8         # 13.94: plate radius that contains the full cross
EAR_SCREW_L = 6.0                                    # M2 self-tap depth into pads (stand-in length)
EAR_PILOT_C = 5.9                                    # type C pilot above the ear top: to 0.6 under the bearing lip (inner ear hole) ->
#                                                      screws up to ~7.2 long through the 1.6 ear (longer: an M2 nut under the head as spacer)
EAR_PILOT_F = 8.5                                    # type F pilot below the ear underside (tall pads): screws up to ~9.8 long


def _rot(M, angle_deg):
    rot = Matrix.Rotation(math.radians(angle_deg), 4, "Z")
    return (M @ rot) if M is not None else rot


def pocket_cutters(M, z0, z1, trim_r=None, angle_deg=0.0):
    """Cutters for an X-horn pocket between z0 and z1 (frame mm): hub + 4 arms (trimmed flat at trim_r,
    or full length with round tips). Each cutter has slightly different end planes (exact boolean)."""
    c = HORN_CLEAR
    L = trim_r if trim_r else HORN_CROSS_STRAIGHT        # end of the straight (tapered) arm
    w_end = HORN_ARM_W_ROOT + (HORN_ARM_W_TIP - HORN_ARM_W_ROOT) * L / HORN_CROSS_STRAIGHT     # the arm tapers to its tip width where its round tip starts (as the horn model)
    Le = L + c if trim_r else L                          # trimmed arm: the same clearance at its flat end as at its sides (horn_check.py)
    cuts = [add_cyl(new_bm(), 0, 0, z0 - 0.07, z1 + 0.05, HUB_D + 2 * c, 40, M)]
    for k in range(4):
        Mk = _rot(M, angle_deg + 90 * k)
        quad = [(-0.6, -HORN_ARM_W_ROOT / 2 - c), (Le, -w_end / 2 - c), (Le, w_end / 2 + c), (-0.6, HORN_ARM_W_ROOT / 2 + c)]
        za, zb = z0 - 0.011 * (k + 1), z1 + 0.013 * (k + 1)
        cuts.append(add_hull(new_bm(), [(x, y, z) for x, y in quad for z in (za, zb)], Mk))
        if not trim_r:                                   # round tip: 0.1 larger than the arm end so no face is tangent to the quad
            cuts.append(add_cyl(new_bm(), L, 0, za - 0.004, zb + 0.004, w_end + 2 * c + 0.2, 24, Mk))
    return cuts


def rib_adds(M, z_mouth, z_top, trim_r=None, angle_deg=0.0):
    """Crush ribs on both walls of each arm pocket: start as a point at the mouth, full radius 0.6 mm in."""
    L = trim_r if trim_r else HORN_CROSS_STRAIGHT
    stations = (0.65,) if trim_r else (0.4, 0.8)
    adds = []
    for k in range(4):
        Mk = _rot(M, angle_deg + 90 * k)
        for f in stations:
            along = f * L
            half_w = (HORN_ARM_W_ROOT + (HORN_ARM_W_TIP - HORN_ARM_W_ROOT) * along / HORN_CROSS_STRAIGHT) / 2
            for side in (-1, 1):
                cy = side * (half_w + RIB_R - RIB_INTERF)
                ring = lambda r, z: [(along + r * math.cos(t), cy + r * math.sin(t), z) for t in (2 * math.pi * i / 16 for i in range(16))]
                adds.append(add_hull(new_bm(), ring(0.1, z_mouth) + ring(RIB_R, z_mouth + 0.6) + ring(RIB_R, z_top), Mk))
    return adds


def combine_cutters(adds, subs=()):
    """Union simple cutter solids (minus optional subtractions) into ONE closed bmesh. Booleans between
    simple convex/annular shapes are robust; many sequential cuts into a finished part are not."""
    tmp_coll = bpy.data.collections.get("_cutters") or bpy.data.collections.new("_cutters")
    if tmp_coll.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(tmp_coll)
    cutter = to_object("_cutter_tmp", adds[0], tmp_coll)
    for p in adds[1:]:
        apply_bool(cutter, p, "UNION", tmp_coll)
    for r in subs:
        apply_bool(cutter, r, "DIFFERENCE", tmp_coll)
    bm = new_bm()
    bm.from_mesh(cutter.data)
    bpy.data.objects.remove(cutter, do_unlink=True)
    return bm


def pocket_cutter(M, z0, z1, trim_r=None, angle_deg=0.0):
    """ONE closed cutter for an X-horn pocket with its crush ribs already subtracted. Cut it once.
    Built in the joint's own frame, then moved: the exact boolean (pocket minus ribs) then does not depend on where the joint is
    (2026-11-13: built in place, the left knee's cutter came out non-manifold at one rib; the right knee's did not)."""
    bm = combine_cutters(pocket_cutters(None, z0, z1, trim_r, angle_deg),
                         rib_adds(None, z0 + 0.6, z1 + 0.4, trim_r, angle_deg))   # mouth 0.1 inside the part's face (z0 is 0.5 outside)
    if M is not None:
        bmesh.ops.transform(bm, matrix=M, verts=list(bm.verts))
    return bm


def saddle_walls(M, z_lo, z_hi):
    """Two walls hugging the servo body sides (SADDLE_CLEAR clearance), full body length."""
    x0, x1 = J_BODY_X[0] - 0.2, J_BODY_X[1] + 0.2
    yi = J_BODY_HW + SADDLE_CLEAR
    return [add_box(new_bm(), x0, x1, s * yi, s * (yi + SADDLE_T), z_lo, z_hi, M) for s in (-1, 1)]


def ear_pads(M, z_lo, z_hi, hw=J_EAR_HW):
    """Pads beyond the body ends under/over the ears (they also stop the body in X). Their outer ends
    sit 0.05 inside the plate's ends: a face flush with the plate's end face breaks the exact boolean."""
    return [add_box(new_bm(), x0, x1, -hw, hw, z_lo, z_hi, M)
            for x0, x1 in ((J_EAR_X[0] + 0.05, J_BODY_X[0] - 0.2), (J_BODY_X[1] + 0.2, J_EAR_X[1] - 0.05))]


CABLE_SIDE = 1                                       # which side (+Y / -Y of the servo frame) the lead leaves a type-F cradle


def cable_channel(M, side=None):
    """Pocket for the lead where it leaves the servo (shaft-end face, at the bottom), inside the shaft-end pad: the lead
    is folded 90 deg in it and leaves SIDEWAYS, then runs along the servo's side. (Straight on, it would run into the
    part that turns on this joint: bearing plate, sole, U.) The pocket is closed toward the ear tip."""
    side = side or CABLE_SIDE
    (x0, x1), (y0, y1), (z0, z1) = J_CABLE
    xa = J_EAR_X[0] + 0.4
    along = add_box(new_bm(), xa, x1, y0 - 0.6, y1 + 0.6, z0 - 0.6, z1 + 1.2, M)
    ys = sorted((0.0, side * (J_EAR_HW + 2.2)))
    out = add_box(new_bm(), xa + 0.05, J_BODY_X[0] - 0.35, ys[0], ys[1], z0 - 0.55, z1 + 1.15, M)
    return combine_cutters([along, out])


# ================================================================== type C
def cap_features(M, include_plate=True, saddle=True, saddle_depth=None):
    """(add, sub) for a type-C stator: cap plate on the tower top (z 0..CAP_T) with the 6702ZZ seat,
    pads hanging to the ear tops (servo screws from below), saddle walls round the body."""
    pts = [(x, y) for x in J_EAR_X for y in (-CAP_HW, CAP_HW)] + \
          [(CAP_RING_R * math.cos(2 * math.pi * i / 48), CAP_RING_R * math.sin(2 * math.pi * i / 48)) for i in range(48)]
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (0.0, CAP_T)], M)] if include_plate else []
    add += ear_pads(M, J_EAR_Z[1], 0.6)
    if saddle:
        add += saddle_walls(M, -(saddle_depth or SADDLE_DEPTH), 0.6)
    # Lip hole: a SQUARE opening (clipped to the seat circle) instead of a round one. The cap prints with the seat open to the
    # bed, so the lip is the seat's ceiling: with a round hole it is a 2 mm ring hanging in the air; with a square one it is
    # four straight 12 mm bridges from wall to wall, which print without support. The outer ring is still carried over 4 arcs
    # of about +-30 deg; the inner ring and the rotor's shoulder (16.6) stay clear (apothem LIP_SQ > LIP_HOLE_D / 2).
    rc = SEAT_D / 2 - 0.05
    sq = [(max(-LIP_SQ, min(LIP_SQ, rc * math.cos(2 * math.pi * i / 96))), max(-LIP_SQ, min(LIP_SQ, rc * math.sin(2 * math.pi * i / 96)))) for i in range(96)]
    sub = [add_cyl(new_bm(), 0, 0, LIP, CAP_T + 1, SEAT_D, 64, M),
           add_hull(new_bm(), [(x, y, z) for x, y in sq for z in (-0.2, LIP + 0.03)], M)]      # 0.2 past the plate's servo face: one layer, no pocket in the pads
    for hx in J_HOLE_X:
        sub.append(add_cyl(new_bm(), hx, 0, J_EAR_Z[1] - 0.5, J_EAR_Z[1] + EAR_PILOT_C, PILOT_D, 16, M))
    return add, sub


def link_features(M, plate_outline_mm=None, angle_deg=0.0):
    """(add, sub, ribs) for a type-C rotor: boss in the bearing, shoulder on the inner ring, plate,
    trimmed X pocket inside the boss, M2x6 counterbored clamp hole."""
    r = BOSS_D / 2 + 3.0
    pts = [(r * math.cos(2 * math.pi * i / 48), r * math.sin(2 * math.pi * i / 48)) for i in range(48)] + list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (PLATE_Z0, PLATE_Z1)], M),
           add_cyl(new_bm(), 0, 0, BOSS_Z0, PLATE_Z0 + 1.0, BOSS_D, 64, M),
           add_cyl(new_bm(), 0, 0, LIP + BRG_W, PLATE_Z0 + 0.6, SHOULDER_D, 48, M)]
    sub = [pocket_cutter(M, BOSS_Z0 - 0.5, HORN_TOP + 0.1, TRIM_R, angle_deg), top_ring_recess(M),
           combine_cutters([add_cyl(new_bm(), 0, 0, BOSS_Z0 - 1.1, PLATE_Z1 + 1.1, CLEAR_D, 20, M),      # clamp hole + counterbore
                            add_cyl(new_bm(), 0, 0, CB_Z, PLATE_Z1 + 0.9, CB_D, 26, M)])]                # as ONE cutter
    return add, sub, []


# ---- bayonet retention for a type-C joint (keeps the rotor on if the horn screw or the horn lets go) -----------------------
# One hook on the stator's cap plate, opposite the rotor's arm, reaches over a 2 mm flange left round the rotor's disc
# (the disc above the flange is turned down to BAY_STEP_R where the hook passes: no added height). The flange has one notch:
# the rotor only goes on at the assembly angle, outside the working range. 0.3 mm axial play: the hook never rubs.
BAY_W = 24.0                                          # hook width, deg (4.9 mm of arc)
BAY_R0, BAY_R1 = BOSS_D / 2 + 3.0 + 0.4, CAP_RING_R - 0.15   # hook post: 10.9 .. 12.6 (inside the cap ring's footprint)
BAY_STEP_R = 9.0                                      # disc radius above the flange
BAY_FLANGE_Z = PLATE_Z0 + 2.0                         # flange top (7.4)
BAY_LIP_Z = (BAY_FLANGE_Z + 0.3, BAY_FLANGE_Z + 1.8)  # lip 7.7 .. 9.2 (below the rotor plate's top, 9.9)


def _revolve(profile_rz, a0, a1, M):
    """a closed (r, z) profile swept from angle a0 to a1 (deg) about Z: ONE closed solid, built directly"""
    bm = new_bm()
    n = max(2, int(math.ceil((a1 - a0) / 6.0)))
    rings = [[bm.verts.new(J(r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), r * math.sin(math.radians(a0 + (a1 - a0) * i / n)), z)) for i in range(n + 1)] for r, z in profile_rz]
    for a, b in zip(rings, rings[1:] + rings[:1]):
        for i in range(n):
            bm.faces.new((a[i], a[i + 1], b[i + 1], b[i]))
    bm.faces.new([r[0] for r in rings])
    bm.faces.new([r[n] for r in rings][::-1])
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    if M is not None:
        bmesh.ops.transform(bm, matrix=M, verts=list(bm.verts))
    return bm


def _sector(r0, r1, a0, a1, z0, z1, M):
    return _revolve([(r1, z0), (r1, z1), (r0, z1), (r0, z0)], a0, a1, M)


def bayonet_hook(M, phi_deg):
    """[solid] to add to the stator: post on the cap plate + lip over the rotor's flange (one L-section solid), centred at angle phi (servo frame)"""
    zb, (zl, zt), rl = CAP_T - 0.3, BAY_LIP_Z, BAY_STEP_R + 0.4
    return [_revolve([(BAY_R0, zb), (BAY_R1, zb), (BAY_R1, zt), (rl, zt), (rl, zl), (BAY_R0, zl)], phi_deg - BAY_W / 2, phi_deg + BAY_W / 2, M)]


def bayonet_rim_cuts(M, phi_deg, lo_deg, hi_deg, assemble_deg, sign=1.0, notch=True):
    """[cutters] for the rotor: the step where the hook passes (working range + the way to the assembly angle) and the notch
    in the flange under the hook at the assembly angle. sign: +1 if the rotor turns about the servo's +Z for a positive joint angle."""
    th = sorted((lo_deg, hi_deg, assemble_deg))
    ra = sorted((phi_deg - sign * th[0], phi_deg - sign * th[-1]))
    m = BAY_W / 2 + 2.0
    na = phi_deg - sign * assemble_deg
    # (the two cutters share no face: different radii, heights and end angles)
    step = _sector(BAY_STEP_R, BAY_R1 + 1.5, ra[0] - m - 1.5, ra[1] + m + 1.5, BAY_FLANGE_Z, PLATE_Z1 + 1.0, M)
    return [step, _sector(BAY_STEP_R - 0.2, BAY_R1 + 2.0, na - m, na + m, PLATE_Z0 - 0.5, BAY_FLANGE_Z + 0.2, M)] if notch else [step]


# ---- keeper clip: the same hook as a separate small print, screwed on AFTER the rotor is in place ------------------------------
# No assembly angle, and both big parts keep their flat print face. One M2x6 self-tapper; a rib under the clip sits in a slot
# through the cap plate (takes the pull-off load in shear and stops the clip turning). The clip prints lying on its side.
KEEP_HW = 3.5                                         # half-width of the foot
KEEP_RIB_X, KEEP_SCREW_X, KEEP_END_X = (13.2, 14.7), 17.2, 19.6
KEEP_SEAT_Z = CAP_T + 1.65                            # screw head seat: 1.6 mm of clip under the head, 4.4 mm of thread in the plate


def keeper_features(M, phi_deg):
    """(add, sub) of the clip, in place on the stator; +X of the clip = direction phi"""
    Mk = _rot(M, phi_deg)
    zb, (zl, zt), rl = CAP_T + 0.09, BAY_LIP_Z, BAY_STEP_R + 0.4
    add = [combine_cutters([_revolve([(BAY_R0, zb), (BAY_R1, zb), (BAY_R1, zt), (rl, zt), (rl, zl), (BAY_R0, zl)], -BAY_W / 2, BAY_W / 2, Mk),
                            add_box(new_bm(), BAY_R0 + 1.2, KEEP_END_X, -KEEP_HW, KEEP_HW, CAP_T + 0.05, zt - 0.2, Mk),
                            add_box(new_bm(), KEEP_RIB_X[0], KEEP_RIB_X[1], -KEEP_HW + 0.02, KEEP_HW - 0.02, CAP_T - 1.5, CAP_T + 0.5, Mk)])]
    sub = [combine_cutters([add_cyl(new_bm(), KEEP_SCREW_X, 0, CAP_T - 1.0, zt + 1.0, CLEAR_D, 20, Mk),
                            add_cyl(new_bm(), KEEP_SCREW_X, 0, KEEP_SEAT_Z, zt + 0.9, CB_D, 26, Mk)])]
    return add, sub


def keeper_seat_cuts(M, phi_deg):
    """[cutters] for the stator's cap plate: the rib's slot (through) and the screw's pilot"""
    Mk = _rot(M, phi_deg)
    return [add_box(new_bm(), KEEP_RIB_X[0] - 0.15, KEEP_RIB_X[1] + 0.15, -KEEP_HW - 0.2, KEEP_HW + 0.2, -1.0, CAP_T + 1.0, Mk),
            add_cyl(new_bm(), KEEP_SCREW_X, 0, CAP_T - 4.5, CAP_T + 0.5, PILOT_D, 14, Mk)]


def keeper_screw(name, M, phi_deg, coll):
    return pan_screw_dummy(name, [(_rot(M, phi_deg), KEEP_SCREW_X, 0, KEEP_SEAT_Z, -1)], coll)


# ================================================================== type F
def floor_features(M, include_floor=True, saddle=True, saddle_top=J_CASE_TOP - 0.2, pad_hw=J_EAR_HW, cable=True):
    """(add, sub) for a type-F stator: floor under the servo bottom with the captive M3 nut, pads up to
    the ear undersides (servo screws from the horn side), saddle walls, cable channel, inner-ring island."""
    add = []
    if include_floor:
        add.append(add_hull(new_bm(), [(x, y, z) for x in J_EAR_X for y in (-CAP_HW, CAP_HW) for z in FLOOR_Z], M))
    add += ear_pads(M, FLOOR_Z[1] - 0.3, J_EAR_Z[0], pad_hw)    # sunk 0.3 into the floor plate: always fused
    if saddle:
        add += saddle_walls(M, FLOOR_Z[1] - 0.3, saddle_top)
    # Ø16 x 0.5 recess in the underside, leaving the Ø4.5 island (an annulus cutter)
    # axle cutter = recess + through hole + hex nut pocket (from the floor top), combined into ONE closed
    # cutter first (simple shapes union robustly), then cut once. Distinct end planes, distinct vertex grids.
    sub = [combine_cutters([add_annulus(new_bm(), RING_ISLAND_D / 2, RING_RECESS_D / 2, FLOOR_Z[0] - 0.5, FLOOR_Z[0] + RING_RECESS_H, M, segs=60),
                            add_cyl(new_bm(), 0, 0, FLOOR_Z[0] - 1.0, FLOOR_Z[1] + 0.5, NUT_HOLE_D, 26, M),
                            add_cyl(new_bm(), 0, 0, FLOOR_Z[1] - NUT_T, FLOOR_Z[1] + 0.7, NUT_AF / math.cos(math.pi / 6), 6, _rot(M, 11.0))])]
    if cable:
        sub.append(cable_channel(M))
    for hx in J_HOLE_X:
        sub.append(add_cyl(new_bm(), hx, 0, J_EAR_Z[0] - EAR_PILOT_F, J_EAR_Z[0] + 0.5, PILOT_D, 16, M))
    return add, sub


def horn_plate_features(M, plate_outline_mm=None, angle_deg=45.0, trim_r=None):
    """(add, sub, ribs) for the horn side of a type-F rotor: plate ARM_Z with the full X pocket, M2 clamp hole."""
    r = (trim_r + HORN_CLEAR + 1.8) if trim_r else ARM_R_FULL
    pts = [(r * math.cos(2 * math.pi * i / 64), r * math.sin(2 * math.pi * i / 64)) for i in range(64)] + list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in ARM_Z], M)]
    sub = [pocket_cutter(M, ARM_Z[0] - 0.5, HORN_TOP + 0.1, trim_r, angle_deg), top_ring_recess(M),
           combine_cutters([add_cyl(new_bm(), 0, 0, ARM_Z[0] - 1, ARM_Z[1] + 1, CLEAR_D, 20, M),          # clamp hole + counterbore
                            add_cyl(new_bm(), 0, 0, KIT_SEAT, ARM_Z[1] + 1, CB_D, 26, M)])]           # (kit horn screw, as type C)
    return add, sub, []


# ================================================================== type C2: the FULL kit horn, uncut (2026-11-13, user)
# The X horn is used as it comes (type C trimmed it to r 6 to fit inside the bearing). The moving part is a plate round the
# whole X, at the horn (as type F's horn plate: ARM_Z, full pocket, crush ribs, kit-screw counterbore), with a boss on top
# running in the 6702ZZ. The bearing sits in a BEARING CAP above that plate (seat open below, lip on top): a separate print
# (ring + arm), screwed with 2 x M2x10 countersunk onto a post of the fixed part just past the servo body's +x end, over the
# +x ear. (One piece does not assemble: the moving plate, wider than the bearing, would be trapped between the base and the
# bridge - assembly_path_check, 2026-11-13.) The fixed part: thin base on the tower top (hole round the horn's hub and the
# servo's boss), ear pads, saddle walls, the post. The moving part needs the sector round the post free over the joint's range.
# Build: servo up into the fixed part + ear screws -> moving part onto the horn + horn screw (from above, open) -> 6702ZZ onto
# the boss -> cap down over it -> 2 cap screws.
C2_BRG_Z = (ARM_Z[1] + 0.4, ARM_Z[1] + 0.4 + BRG_W)          # bearing 6.8 .. 10.8
C2_BRIDGE_Z = (ARM_Z[1] + 0.35, C2_BRG_Z[1] + LIP)           # cap 6.75 .. 11.8 (0.35 over the moving plate)
C2_BASE_Z = (0.0, ARM_Z[0] - 0.8)                            # base on the tower top 0 .. 1.8 (the horn's arms start at 2.7)
C2_SWEEP_R = ARM_R_FULL + 0.35                               # moving plate's radius + clearance (14.29)
C2_RING_R = SEAT_D / 2 + 2.2                                 # cap ring round the bearing seat
C2_LIP_D = 17.8                                              # lip carries the outer ring only (inner ring and shoulder stay clear)
C2_POST_X = (J_BODY_X[1] + 0.25, J_EAR_X[1] - 0.05)          # 17.6 .. 21.9: the post's foot, past the body end, over the +x ear
C2_POST_HW = J_EAR_HW
C2_POST_IN_X = C2_SWEEP_R + 0.3                              # 14.59: above the base the post reaches in to here (outside the moving plate)
C2_CAP_HW = 7.0                                              # cap arm and post top half-width (inside the type-C envelope, CAP_HW 7.65)
C2_CAP_SCREWS = [(18.25, -3.0), (18.25, 3.0)]                # 2 x M2x10 countersunk (Robu EasyMech SS304), down into the post
CSK_HEAD_D, CSK_HEAD_H, CSK_L = 3.8, 0.9, 10.0               # M2 90 deg countersunk head; length overall (head included)
CSK_SEAT = 0.2                                               # head top this far under the cap's top face


def c2_stator_features(M, saddle=True):
    """(add, sub) for a type-C2 fixed part: base on the tower top, ear pads, saddle walls, the post that carries the cap."""
    pts = [(x, y) for x in J_EAR_X for y in (-CAP_HW, CAP_HW)]
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in C2_BASE_Z], M)]
    add += ear_pads(M, J_EAR_Z[1], C2_BASE_Z[1])
    if saddle:
        add += saddle_walls(M, -SADDLE_DEPTH, C2_BASE_Z[1])
    add.append(add_box(new_bm(), C2_POST_X[0], C2_POST_X[1], -C2_POST_HW, C2_POST_HW, J_EAR_Z[1] + 0.02, C2_BASE_Z[1] + 0.3, M))   # post foot
    add.append(add_box(new_bm(), C2_POST_IN_X, C2_POST_X[1] - 0.01, -C2_CAP_HW, C2_CAP_HW, C2_BASE_Z[0] + 0.4, C2_BRIDGE_Z[0], M))  # post, up to the cap
    sub = [add_cyl(new_bm(), 0, 0, C2_BASE_Z[1] + 0.2, C2_BRIDGE_Z[0] + 0.5, 2 * C2_SWEEP_R, 64, M),               # the moving plate's room
           combine_cutters([add_cyl(new_bm(), 0, 0, C2_BASE_Z[0] - 1.0, C2_BASE_Z[1] + 1.0, HUB_D + 0.8, 32, M),     # round the horn's hub
                            add_box(new_bm(), -0.5, BOSS_TONGUE_X1 - SHAFT_X + 0.4, -BOSS_TONGUE_HW - 0.3, BOSS_TONGUE_HW + 0.3,
                                    C2_BASE_Z[0] - 0.5, BOSS_TOP - TOWER_TOP + 0.3, M)])]                       # relief over the servo's boss + tongue
    sub += screw_holes(M, C2_CAP_SCREWS, C2_BRIDGE_Z[0], 0.0, C2_BRIDGE_Z[0] - (C2_BRIDGE_Z[1] - CSK_SEAT - CSK_L) + 0.6, -1)   # cap screw pilots
    for hx in J_HOLE_X:
        sub.append(add_cyl(new_bm(), hx, 0, J_EAR_Z[1] - 0.5, J_EAR_Z[1] + EAR_PILOT_C, PILOT_D, 16, M))
    return add, sub


def _csk_cone(M, x, y, z_lo, d_lo, z_hi, d_hi, segs=24, bm=None):
    """closed cone frustum along frame Z: Ø d_lo at z_lo, Ø d_hi at z_hi (countersinks, countersunk heads); into bm if given"""
    bm = bm if bm is not None else new_bm()
    out = bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=d_lo / 2 * MM, radius2=d_hi / 2 * MM, depth=(z_hi - z_lo) * MM)
    bmesh.ops.translate(bm, verts=out["verts"], vec=J(x, y, (z_lo + z_hi) / 2))
    if M is not None:
        bmesh.ops.transform(bm, matrix=M, verts=out["verts"])
    return bm


def c2_cap_features(M):
    """(add, sub) for a type-C2 bearing cap: ring round the 6702ZZ seat (open below, lip on top) + arm to the post, 2 countersunk
    clearance holes. Prints top face down (lip first; the seat and the countersinks open upward)."""
    z0, z1 = C2_BRIDGE_Z
    add = [add_cyl(new_bm(), 0, 0, z0, z1, 2 * C2_RING_R, 64, M),
           add_box(new_bm(), 0.0, C2_POST_X[1] - 0.01, -C2_CAP_HW, C2_CAP_HW, z0, z1, M)]
    sub = [combine_cutters([add_cyl(new_bm(), 0, 0, z0 - 0.5, C2_BRG_Z[1], SEAT_D, 64, M),                        # seat, open below
                            add_cyl(new_bm(), 0, 0, C2_BRG_Z[1] - 0.03, z1 + 0.5, C2_LIP_D, 48, M)])]            # lip opening
    hs = z1 - CSK_SEAT                                          # head top; 90 deg seat: Ø3.8 there, Ø2.2 at 0.8 below
    for x, y in C2_CAP_SCREWS:
        sub.append(combine_cutters([add_cyl(new_bm(), x, y, z0 - 0.6, z1 - 0.5, CLEAR_D, 16, M),
                                    _csk_cone(M, x, y, hs - (CSK_HEAD_D + 0.1 - 2.2) / 2, 2.2, z1 + 0.3, CSK_HEAD_D + 0.1 + 2 * (z1 + 0.3 - hs))]))
    return add, sub


def c2_rotor_features(M, plate_outline_mm=None, angle_deg=45.0):
    """(add, sub, ribs) for a type-C2 moving part: full-X horn plate (ARM_Z) + shoulder on the inner ring + boss in the bearing."""
    pts = [(ARM_R_FULL * math.cos(2 * math.pi * i / 64), ARM_R_FULL * math.sin(2 * math.pi * i / 64)) for i in range(64)] + list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in ARM_Z], M),
           add_cyl(new_bm(), 0, 0, ARM_Z[1] - 0.02, C2_BRG_Z[0], SHOULDER_D, 48, M),
           add_cyl(new_bm(), 0, 0, ARM_Z[1] - 0.03, C2_BRG_Z[1] + 0.1, BOSS_D, 64, M)]
    top = C2_BRG_Z[1] + 0.1
    sub = [pocket_cutter(M, ARM_Z[0] - 0.5, HORN_TOP + 0.1, None, angle_deg), top_ring_recess(M),
           combine_cutters([add_cyl(new_bm(), 0, 0, ARM_Z[0] - 1, top + 1, CLEAR_D, 20, M),                         # clamp hole +
                            add_cyl(new_bm(), 0, 0, KIT_SEAT, top + 1, CB_D, 26, M)])]                              # counterbore up the boss
    return add, sub, []


def hardware_C2(prefix, M, coll):
    """6702ZZ in the cap, the servo's own horn screw at KIT_SEAT, the ear screws (from below, as type C), the 2 cap screws."""
    steel = material("hw_steel", (0.6, 0.62, 0.66, 1))
    brg = bearing_dummy(f"{prefix}brg_{BRG_NAME}", M, C2_BRG_Z[0], C2_BRG_Z[1], BRG_ID, BRG_OD, RING_T["6702ZZ"], coll, steel)
    scr = new_bm()
    add_cyl(scr, 0, 0, KIT_SEAT, KIT_SEAT + KIT_SCREW[3], KIT_SCREW[2], 20, M)
    add_cyl(scr, 0, 0, KIT_SEAT - KIT_SCREW[1], KIT_SEAT + 0.01, KIT_SCREW[0], 16, M)
    hs = C2_BRIDGE_Z[1] - CSK_SEAT
    cs = new_bm()
    for x, y in C2_CAP_SCREWS:                                 # head (90 deg cone, 0.03 under size: touching, not overlapping) + thread
        _csk_cone(M, x, y, hs - 0.02 - (CSK_HEAD_D - 2.0) / 2, 2.0 - 0.06, hs - 0.02, CSK_HEAD_D - 0.06, bm=cs)
        add_cyl(cs, x, y, hs - CSK_L, hs - (CSK_HEAD_D - 2.0) / 2 - 0.04, 2.0, 16, M)    # (separate shells: parity sampling)
    return [brg, to_object(f"{prefix}screw_M2x4kit", scr, coll, steel), ear_screw_dummy(f"{prefix}ear_screws_M2x6st", M, coll, steel, True),
            to_object(f"{prefix}cap_screws_M2x10csk", cs, coll, steel)]


# ================================================================== type C2M: full horn, the bearing IN THE MOVING PART (2026-11-13)
# For a moving part that is ONE flat plate printed on its far face (the shin: knee plate + ankle floor coplanar, cradle growing
# toward the servo). The plate keeps its plane (PLATE_Z0..PLATE_Z1); round the axis it is a disc down to ARM_Z[0] holding the full
# horn (pocket + ribs), a 1.5 floor, and the 6702ZZ in a seat open to the far face (bearing flush with it; the OUTER ring turns
# with the part). A bolt-on CAP on the far side carries a fixed tubular boss down into the bearing's bore and a shoulder on the
# inner ring, and its own post down to the fixed part's face (the fixed part is printed on that face: nothing may stand on it).
# 2 x M2x6 socket screws (row 19) in clearance holes: the cap centres itself on the bearing, then is clamped (a countersunk head
# would force it onto the printed pilots and side-load the servo shaft). A Ø5.2 hole through the cap reaches the horn screw.
# The fixed part: plate thinned to C2_BASE_Z under the disc (room r C2_SWEEP_R), ear pads, saddle walls, the screws' pilots.
C2M_BRG_Z = (PLATE_Z1 - BRG_W, PLATE_Z1)                     # 5.9 .. 9.9: flush with the far face
C2M_FLOOR_RELIEF = (9.3, C2M_BRG_Z[0] - 0.4)              # floor recess under the inner ring (r <= 8.4) to 5.5: it must not rub; the outer ring (r 9.6+) sits on the land
C2M_CAP_Z = (PLATE_Z1 + 0.3, PLATE_Z1 + 0.3 + 3.1)           # cap plate 10.2 .. 13.3 (knee: the hip-yaw limit leaves 4.4 over the shin
#                                                              face at the cap's middle, less toward its rear - see c2m_cap_features)
C2M_CAP_ARM_HW = 10.0                                         # plate half-width at the post end (the post itself is +-C2_CAP_HW)
C2M_CAP_R = 10.0
C2M_BOSS = (2.6, BOSS_D / 2, C2M_BRG_Z[0] + 0.1)             # tube r 2.6 .. 7.45 down to 6.0 (3.9 of the 4.0 bore); horn screw head r 1.75 inside
C2M_SHOULDER_R = SHOULDER_D / 2                              # 8.3: on the inner ring's face only (outer ring from r 9.6)
C2M_SCREW_BITE = 3.5                                         # M2x6 socket: 3.5 into the fixed part, 2.5 of cap foot clamped
C2M_DRIVE_D = 5.2                                            # through the cap: driver to the horn screw


C2M_ROOM_Z0 = ARM_Z[0] - 1.0                                  # 1.6: base 1.6 thick, 1.0 under the moving disc (the base prints as a ~28 mm
#                                                              bridge over the room: room for the first layer to sag)


def c2m_holder_features(M, face_z=CAP_T, screws=None):
    """(add, sub) for the fixed side of a C2M joint whose plate (face at face_z, printed face down) the caller builds: pads, saddle
    walls; cuts: the disc's room down to the base, hub + boss relief, ear pilots, the cap screws' pilots."""
    add = ear_pads(M, J_EAR_Z[1], 0.6) + saddle_walls(M, -SADDLE_DEPTH, 0.6)
    sub = [add_cyl(new_bm(), 0, 0, C2M_ROOM_Z0, face_z + 0.6, 2 * C2_SWEEP_R, 64, M),
           combine_cutters([add_cyl(new_bm(), 0, 0, C2_BASE_Z[0] - 1.0, C2M_ROOM_Z0 + 1.0, HUB_D + 0.8, 32, M),
                            add_box(new_bm(), -0.5, BOSS_TONGUE_X1 - SHAFT_X + 0.4, -BOSS_TONGUE_HW - 0.3, BOSS_TONGUE_HW + 0.3,
                                    C2_BASE_Z[0] - 0.5, BOSS_TOP - TOWER_TOP + 0.3, M)])]
    tip = face_z - C2M_SCREW_BITE
    for x, y in (screws or C2_CAP_SCREWS):
        sub.append(add_cyl(new_bm(), x, y, tip - 0.6, face_z + 0.4, PILOT_D, 16, M))
    for hx in J_HOLE_X:
        sub.append(add_cyl(new_bm(), hx, 0, J_EAR_Z[1] - 0.5, J_EAR_Z[1] + EAR_PILOT_C, PILOT_D, 16, M))
    return add, sub


def c2m_rotor_features(M, plate_outline_mm=None, angle_deg=45.0):
    """(add, sub, ribs) for a C2M moving plate: disc ARM_Z[0]..PLATE_Z1 round the full horn + the link outline in PLATE_Z0..PLATE_Z1;
    full-X pocket, clamp hole + counterbore, the 6702ZZ seat (open to the far face) with the inner-ring relief."""
    disc = [(ARM_R_FULL * math.cos(2 * math.pi * i / 64), ARM_R_FULL * math.sin(2 * math.pi * i / 64)) for i in range(64)]
    if plate_outline_mm:                                       # disc below the plate (0.03 smaller, top inside it: no coincident faces)
        lo = [(x * (ARM_R_FULL - 0.03) / ARM_R_FULL, y * (ARM_R_FULL - 0.03) / ARM_R_FULL) for x, y in disc]
        add = [add_hull(new_bm(), [(x, y, z) for x, y in disc + list(plate_outline_mm) for z in (PLATE_Z0, PLATE_Z1)], M),
               add_hull(new_bm(), [(x, y, z) for x, y in lo for z in (ARM_Z[0], PLATE_Z0 + 0.6)], M)]
    else:
        add = [add_hull(new_bm(), [(x, y, z) for x, y in disc for z in (ARM_Z[0], PLATE_Z1)], M)]
    sub = [pocket_cutter(M, ARM_Z[0] - 0.5, HORN_TOP + 0.1, None, angle_deg), top_ring_recess(M),
           combine_cutters([add_cyl(new_bm(), 0, 0, ARM_Z[0] - 1.0, KIT_SEAT + 0.3, CLEAR_D, 20, M),
                            add_cyl(new_bm(), 0, 0, KIT_SEAT, C2M_FLOOR_RELIEF[1] + 0.2, CB_D, 26, M),
                            add_cyl(new_bm(), 0, 0, C2M_FLOOR_RELIEF[1], C2M_BRG_Z[0] + 0.3, 2 * C2M_FLOOR_RELIEF[0], 56, M),
                            add_cyl(new_bm(), 0, 0, C2M_BRG_Z[0], PLATE_Z1 + 0.6, SEAT_D, 64, M)])]
    return add, sub, []


def c2m_cap_features(M, foot_z=CAP_T, chamfer=None, screws=None, post_y=None, arm_y=None, disc_r=C2M_CAP_R):
    """(add, sub) for a C2M cap: plate C2M_CAP_Z (disc + arm to the post), the post down to the fixed part's face (foot_z), the tube
    boss + shoulder into the bearing; cuts: driver hole, 2 screw clearance holes + counterbores. Prints top face down.
    chamfer = (sign, y0): 45 deg chamfer off the top on the frame's sign*y side, from y = sign*y0 (knee: the side that swings toward
    the electronics enclosure at the hip-yaw limit; 45 deg prints without support, top face down). It takes a small arc off
    the shoulder's far edge; the rest of the shoulder (about 300 deg) bears on the inner ring.
    screws / post_y / arm_y / disc_r: the post and arm can be off-centre (hip pitch: clear of the body-to-thigh cable) and the disc
    smaller (hip pitch: the yaw connector sweeps 2.4 mm over the L-bracket from r ~11)."""
    screws = screws or C2_CAP_SCREWS
    post_y = post_y or (-C2_CAP_HW, C2_CAP_HW)
    arm_y = arm_y or (-C2M_CAP_ARM_HW, C2M_CAP_ARM_HW)
    z0, z1 = C2M_CAP_Z
    disc = [(disc_r * math.cos(2 * math.pi * i / 48), disc_r * math.sin(2 * math.pi * i / 48)) for i in range(48)]
    add = [add_hull(new_bm(), [(x, y, z) for x, y in disc + [(C2_POST_X[1] - 0.01, y) for y in arm_y] for z in (z0, z1)], M),
           add_box(new_bm(), C2_POST_IN_X, C2_POST_X[1] - 0.02, post_y[0] + 0.01, post_y[1] - 0.01, foot_z + 0.02, z0 + 0.3, M),
           add_cyl(new_bm(), 0, 0, C2M_BOSS[2], z0 + 0.3, 2 * C2M_BOSS[1], 64, M),
           add_cyl(new_bm(), 0, 0, PLATE_Z1 + 0.1, z0 + 0.2, 2 * C2M_SHOULDER_R, 48, M)]     # 0.1 axial play over the inner ring (no preload from print tolerance)
    sub = [add_cyl(new_bm(), 0, 0, C2M_BOSS[2] - 0.5, z1 + 0.5, C2M_DRIVE_D, 32, M)]
    seat = foot_z + 0.02 + (6.0 - C2M_SCREW_BITE)                 # head seat: 2.5 of foot under it
    for x, y in screws:
        sub.append(combine_cutters([add_cyl(new_bm(), x, y, foot_z - 0.5, seat + 0.3, CLEAR_D, 16, M),
                                    add_cyl(new_bm(), x, y, seat, z1 + 0.5, CB_D, 26, M)]))
    if chamfer:
        sg, y0 = chamfer
        prof = [(y0, z1), (y0 + 25.0, z1 - 25.0), (y0 + 25.0, z1 + 3.0), (y0 - 3.0, z1 + 3.0)]     # (y, z) on and beyond the 45 deg plane
        sub.append(add_hull(new_bm(), [(x, sg * y, z) for x in (-15.0, 30.0) for y, z in prof], M))
    return add, sub


def hardware_C2M(prefix, M, coll, foot_z=CAP_T, screws=None):
    """6702ZZ in the moving plate, the servo's own horn screw, the ear screws (from below), the cap's 2 x M2x6 socket screws."""
    steel = material("hw_steel", (0.6, 0.62, 0.66, 1))
    brg = bearing_dummy(f"{prefix}brg_{BRG_NAME}", M, C2M_BRG_Z[0], C2M_BRG_Z[1], BRG_ID, BRG_OD, RING_T["6702ZZ"], coll, steel)
    scr = new_bm()
    add_cyl(scr, 0, 0, KIT_SEAT, KIT_SEAT + KIT_SCREW[3], KIT_SCREW[2], 20, M)
    add_cyl(scr, 0, 0, KIT_SEAT - KIT_SCREW[1], KIT_SEAT + 0.01, KIT_SCREW[0], 16, M)
    seat = foot_z + 0.02 + (6.0 - C2M_SCREW_BITE)
    return [brg, to_object(f"{prefix}screw_M2x4kit", scr, coll, steel), ear_screw_dummy(f"{prefix}ear_screws_M2x6st", M, coll, steel, True),
            pan_screw_dummy(f"{prefix}cap_screws_M2x6", [(M, x, y, seat, -1) for x, y in (screws or C2_CAP_SCREWS)], coll, steel)]


def bearing_plate_features(M, plate_outline_mm=None):
    """(add, sub) for the far side of a type-F rotor: plate with the 693ZZ seat (open to the stator side,
    0.8 lip outside) around the M3 axle. Screws to the rotor are added by the caller."""
    z1, z0 = BP_INNER_Z, BP_INNER_Z - BP_T
    r = BP_SEAT_D / 2 + 2.6
    pts = [(r * math.cos(2 * math.pi * i / 48), r * math.sin(2 * math.pi * i / 48)) for i in range(48)] + list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (z0, z1)], M)]
    sub = [add_cyl(new_bm(), 0, 0, z1 - B693[2], z1 + 1, BP_SEAT_D, 48, M),
           add_cyl(new_bm(), 0, 0, z0 - 1, z1 + 1, BP_LIP_D, 32, M)]
    return add, sub


# ================================================================== hardware dummies (for views and overlap checks)
EAR_SCREW_LEN = 6.0          # M2x6 pan-head self-tapping: 1.6 through the ear + 4.4 into the printed pad
RING_T = {"6702ZZ": 0.9, "693": 0.8}   # ring wall thickness of the bearing dummies (NOMINAL, for views/fit checks)


def bearing_dummy(name, M, z0, z1, d_in, d_out, ring_t, coll, mat):
    """Inner ring, outer ring and a recessed shield between them (three closed shells in one object)."""
    bm = new_bm()
    add_annulus(bm, d_in / 2, d_in / 2 + ring_t, z0, z1, M, segs=64)
    add_annulus(bm, d_out / 2 - ring_t, d_out / 2, z0, z1, M, segs=64)
    add_annulus(bm, d_in / 2 + ring_t + 0.02, d_out / 2 - ring_t - 0.02, z0 + 0.3, z1 - 0.3, M, segs=64)
    return to_object(name, bm, coll, mat)


def ear_screw_dummy(name, M, coll, mat, from_below):
    """The servo's two ear screws. Type C: head under the ear, thread up into the pad; type F: head on the ear, thread down."""
    bm = new_bm()
    for hx in J_HOLE_X:
        if from_below:
            add_cyl(bm, hx, 0, J_EAR_Z[0] - SOCKET_HEAD[1], J_EAR_Z[0], SOCKET_HEAD[0], 20, M)
            add_cyl(bm, hx, 0, J_EAR_Z[0] - 0.01, J_EAR_Z[0] + EAR_SCREW_LEN, 2.0, 16, M)
        else:
            add_cyl(bm, hx, 0, J_EAR_Z[1], J_EAR_Z[1] + SOCKET_HEAD[1], SOCKET_HEAD[0], 20, M)
            add_cyl(bm, hx, 0, J_EAR_Z[1] - EAR_SCREW_LEN, J_EAR_Z[1] + 0.01, 2.0, 16, M)
    return to_object(name, bm, coll, mat)


def pan_screw_dummy(name, placements, coll, mat=None):
    """M2x6 socket-head machine screws (into printed pilots): placements = [(M, x, y, z_seat, direction)], head on z_seat,
    thread 6 mm along direction. (Name kept from the pan-head self-tapper it replaced.)"""
    bm = new_bm()
    for M, x, y, z, d in placements:
        h = SOCKET_HEAD[1]
        add_cyl(bm, x, y, min(z, z - d * h), max(z, z - d * h), SOCKET_HEAD[0], 20, M)
        add_cyl(bm, x, y, min(z - d * 0.01, z + d * EAR_SCREW_LEN), max(z - d * 0.01, z + d * EAR_SCREW_LEN), 2.0, 16, M)
    return to_object(name, bm, coll, mat or material("hw_steel", (0.6, 0.62, 0.66, 1)))


def screw_heads(ob):
    """[(seat, out, head_len)] for each screw of a screw dummy object (world, metres): seat = the head's underside,
    out = away from the thread. Heads/shanks are the dummy's separate cylinders, told apart by diameter."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.transform(ob.matrix_world)
    seen, isl = set(), []
    for f in bm.faces:
        if f.index in seen:
            continue
        st, cur = [f], []
        while st:
            g = st.pop()
            if g.index in seen:
                continue
            seen.add(g.index)
            cur.append(g)
            st += [h for e in g.edges for h in e.link_faces if h.index not in seen]
        isl.append(cur)
    cyl = []
    for fs in isl:
        vs = {v for f in fs for v in f.verts}
        ce = sum((v.co for v in vs), Vector()) / len(vs)
        ax = max(fs, key=lambda f: f.calc_area()).normal.normalized()         # a cap
        h = max((v.co - ce).dot(ax) for v in vs) * 2
        r = max(((v.co - ce) - ax * (v.co - ce).dot(ax)).length for v in vs)
        cyl.append((ce, ax, h, r))
    bm.free()
    heads = [c for c in cyl if c[3] * 2 / MM > 3.0]
    shanks = [c for c in cyl if c[3] * 2 / MM <= 2.4]
    out = []
    for ce, ax, h, r in heads:
        sh = min(shanks, key=lambda s: (s[0] - ce).length)
        o = ax if (ce - sh[0]).dot(ax) > 0 else -ax
        out.append((ce - o * h / 2, o, h / MM))
    return out


def head_room_cut(target, dummies, coll, room=SOCKET_HEAD[1] + 0.3, d=SOCKET_HEAD[0] + 0.6):
    """Cut a Ø d recess, from each head's seat to room above it, into target: room for the screw heads of the given dummy
    objects where a part closes over them. Where the part is farther than room above a head it is untouched."""
    for ob in dummies:
        for seat, o, _h in screw_heads(ob):                                   # one cut per head: neighbouring recesses overlap, and an
            bm = new_bm()                                                     # overlapping cutter makes the exact solver return nothing
            out = bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=d / 2 * MM, radius2=d / 2 * MM, depth=(room + 0.1) * MM)
            R = o.to_track_quat("Z", "Y").to_matrix().to_4x4()
            bmesh.ops.transform(bm, matrix=Matrix.Translation(seat + o * (room - 0.1) / 2 * MM) @ R, verts=out["verts"])
            bool_shared(target, bm, "DIFFERENCE")


def bool_shared(target, bm_world, op):
    """Exact boolean that also works when target's mesh is shared (exploded-view copies follow): evaluate with a modifier and
    write the result into the mesh. A result that loses more than half the volume is NOT written (returns False)."""
    me = bpy.data.meshes.new("_bool_tmp")
    bm_world.transform(target.matrix_world.inverted())
    bm_world.to_mesh(me)
    bm_world.free()
    c = bpy.data.objects.new("_bool_tmp", me)
    bpy.context.scene.collection.objects.link(c)
    c.matrix_world = target.matrix_world.copy()
    m = target.modifiers.new("_b", "BOOLEAN")
    m.operation, m.object, m.solver = op, c, "EXACT"
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    res = bpy.data.meshes.new_from_object(target.evaluated_get(dg))
    target.modifiers.remove(m)
    b0, b1 = bmesh.new(), bmesh.new()
    b0.from_mesh(target.data)
    b1.from_mesh(res)
    ok = b1.calc_volume() > 0.5 * b0.calc_volume()
    if ok:
        b1.to_mesh(target.data)
        target.data.update()
    b0.free()
    b1.free()
    bpy.data.meshes.remove(res)
    bpy.data.objects.remove(c)
    bpy.data.meshes.remove(me)
    return ok


def hardware_C(prefix, M, coll):
    steel = material("hw_steel", (0.6, 0.62, 0.66, 1))
    brg = bearing_dummy(f"{prefix}brg_{BRG_NAME}", M, LIP, LIP + BRG_W, BRG_ID, BRG_OD, RING_T["6702ZZ"], coll, steel)
    scr = new_bm()
    add_cyl(scr, 0, 0, CB_Z, CB_Z + KIT_SCREW[3], KIT_SCREW[2], 20, M)   # the servo's own horn screw in the counterbore
    add_cyl(scr, 0, 0, CB_Z - KIT_SCREW[1], CB_Z + 0.01, KIT_SCREW[0], 16, M)
    return [brg, to_object(f"{prefix}screw_M2x4kit", scr, coll, steel), ear_screw_dummy(f"{prefix}ear_screws_M2x6st", M, coll, steel, True)]


def hardware_F(prefix, M, coll):
    steel = material("hw_steel", (0.6, 0.62, 0.66, 1))
    z1 = BP_INNER_Z
    brg = bearing_dummy(f"{prefix}brg_693", M, z1 - B693[2], z1, B693[0], B693[1], RING_T["693"], coll, steel)
    scr = new_bm()
    add_cyl(scr, 0, 0, z1 - B693[2] - 3.0, z1 - B693[2] + 0.01, 5.5, 24, M)   # M3 socket head outside the bearing
    add_cyl(scr, 0, 0, z1 - B693[2], z1 - B693[2] + 8.0, 3.0, 16, M)         # M3x8 shank: through bearing, island, floor, nut
    nut = to_object(f"{prefix}nut_M3", add_cyl(new_bm(), 0, 0, FLOOR_Z[1] - NUT_T + 0.2, FLOOR_Z[1] - 0.05, 5.5 / math.cos(math.pi / 6), 6, _rot(M, 11.0)), coll, steel)   # turned like its pocket
    apply_bool(nut, add_cyl(new_bm(), 0, 0, FLOOR_Z[1] - NUT_T - 1, FLOOR_Z[1] + 1, 3.0, 16, M), "DIFFERENCE", coll)
    horn_scr = new_bm()
    add_cyl(horn_scr, 0, 0, KIT_SEAT, KIT_SEAT + KIT_SCREW[3], KIT_SCREW[2], 20, M)   # the servo's own horn screw in the counterbore
    add_cyl(horn_scr, 0, 0, KIT_SEAT - KIT_SCREW[1], KIT_SEAT + 0.01, KIT_SCREW[0], 16, M)
    return [brg, to_object(f"{prefix}screw_M3x8", scr, coll, steel), nut, to_object(f"{prefix}screw_M2x4kit", horn_scr, coll, steel),
            ear_screw_dummy(f"{prefix}ear_screws_M2x6st", M, coll, steel, False)]


# ================================================================== horn trimming jig (type C horns)
def trim_jig(name, coll, M, trim_r=TRIM_R):
    """Ø(2 trim_r) block: the full X horn drops in hub-first, its arms stick out through 4 slots in the
    wall; cut them flush with the wall -> every arm ends at trim_r."""
    # 2026-11-13: slots 0.25 per side along the arm's real taper (was 0.1 on a mis-placed taper: 0.06-0.10 at the wall - an FDM
    # slot prints 0.1-0.2 narrow, the horn would not drop in); wall at trim_r - 0.35: with the hub's 0.2 play in its hole and
    # ~0.3 left by a flush cutter, a cut arm ends at 5.45..6.15 < the pocket's end 6.2 (pocket_cutters: L + HORN_CLEAR).
    # Tested with the horn model in the jig: seats (0.02 mm3 = contact), arm slots 0.24 per side, hub 0.20.
    JIG_CLEAR, JIG_IN = 0.25, 0.35
    jig_r = trim_r - JIG_IN
    H_HUB = HORN_HUB_H - HORN_PLATE_T + 0.3           # hub below the arms, +0.3 so the arms sit proud of nothing
    base, top = 1.2, 1.2 + H_HUB + HORN_PLATE_T + 0.2
    body = add_cyl(new_bm(), 0, 0, 0.0, top, 2 * jig_r, 64, M)
    jig = to_object(name, body, coll, material("jig", (0.3, 0.75, 0.35, 1)))
    cuts = [add_cyl(new_bm(), 0, 0, base, top + 1, HUB_D + 0.4, 40, M),                     # the hub centres the horn
            add_cyl(new_bm(), 0, 0, base + H_HUB - 0.31, top + 1.1, 2 * (3.76 + JIG_CLEAR), 48, M)]   # plate level: the plate fills the
    #                                                                               corners between the arms out to r 3.76 (kit photo)
    hw = lambda x: (HORN_ARM_W_ROOT + (HORN_ARM_W_TIP - HORN_ARM_W_ROOT) * max(0.0, x) / HORN_CROSS_STRAIGHT) / 2 + JIG_CLEAR
    for k in range(4):
        Mk = _rot(M, 90 * k)
        xe = jig_r + 3.0
        quad = [(-0.6, -hw(0.0)), (xe, -hw(xe)), (xe, hw(xe)), (-0.6, hw(0.0))]
        cuts.append(add_hull(new_bm(), [(x, y, z) for x, y in quad for z in (base + H_HUB - 0.3, top + 1)], Mk))
    for c in cuts:
        apply_bool(jig, c, "DIFFERENCE", coll)
    jig["purpose"] = f"cut the X horn arms flush with the wall (r {jig_r:.2f}): arms end at {jig_r:.2f}..~{trim_r + 0.2:.1f} mm, pockets end at {trim_r + HORN_CLEAR:.1f}"
    return jig
