"""SCS0009 joint features v2 — exact-servo based. Runs inside Blender; exec this file into a dict.

Two joint types (see projects/xgoduck_scs0009/REVIEW.md, "Joints v2"):
  C  cap (one-sided): stator = cap plate on the tower top + ear pads + saddle walls round the body;
     16x22x4 bearing in the cap; rotor = Ø15.95 boss in the bearing with the X horn TRIMMED to r = 6
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
CLEAR_D, PILOT_D = 2.3, 1.6                          # M2 clearance; M2 self-tap pilot
HUB_D, HORN_TOP, PLATE_T_HORN = HORN_HUB_D, J_HORN_HUB_Z[1], HORN_PLATE_T
# ---- type C
BRG_ID, BRG_OD, BRG_W = 16.0, 22.0, 4.0
TRIM_R = 6.0
CAP_T, LIP, LIP_HOLE_D = 5.0, 1.0, 18.0
SEAT_D = BRG_OD + 0.1
CAP_HW = J_BODY_HW + 1.6                             # 7.65: cap half-width = saddle wall outer face
SADDLE_CLEAR, SADDLE_DEPTH = 0.15, 8.0               # walls cover the tower (3.8) + 4.2 of body
SADDLE_T = CAP_HW - 0.1 - (J_BODY_HW + SADDLE_CLEAR)   # 1.35: outer face 0.1 inside the plate edge (no coincident faces)
CAP_RING_R = SEAT_D / 2 + 2.2
BOSS_D, BOSS_Z0, SHOULDER_D = BRG_ID - 0.05, 0.5, 17.6
PLATE_Z0 = LIP + BRG_W + 0.4                         # 5.4
PLATE_T = 4.0
PLATE_Z1 = PLATE_Z0 + PLATE_T                        # 9.4
CB_D, CB_Z = 4.2, 6.0
# ---- type F
FLOOR_T = 4.0                                        # = PLATE_T: a type-C rotor plate and a type-F floor can be one plate;
#                                                      holds a captive M3 nut (2.4) under the servo bottom
FLOOR_Z = (J_BOTTOM - FLOOR_T, J_BOTTOM)             # -28.85 .. -25.25
NUT_AF, NUT_T, NUT_HOLE_D = 5.5 + 0.25, 2.4 + 0.2, 3.3
RING_ISLAND_D, RING_RECESS_D, RING_RECESS_H = 4.5, 16.0, 0.5   # floor underside: Ø16 x 0.5 recess round a flush Ø4.5
#                                                      island -> the 693's inner ring bears on the island, the bearing plate
#                                                      clears the floor by 0.5, and the floor's face stays flat (print bed)
B693 = (3.0, 8.0, 4.0)
BP_T = 4.2                                           # bearing plate thickness (seat 4.0 + 0.2 lip)
BP_INNER_Z = FLOOR_Z[0]                              # bearing plate inner face = floor's nominal underside
BP_SEAT_D, BP_LIP_D = B693[1] + 0.1, 6.0
ARM_Z = (2.6, 6.4)                                   # horn plate of a type-F rotor (2.1 above the horn top)
ARM_R_FULL = HORN_CROSS_L + HORN_CLEAR + 1.8         # 13.94: plate radius that contains the full cross
EAR_SCREW_L = 6.0                                    # M2 self-tap depth into pads


def _rot(M, angle_deg):
    rot = Matrix.Rotation(math.radians(angle_deg), 4, "Z")
    return (M @ rot) if M is not None else rot


def pocket_cutters(M, z0, z1, trim_r=None, angle_deg=0.0):
    """Cutters for an X-horn pocket between z0 and z1 (frame mm): hub + 4 arms (trimmed flat at trim_r,
    or full length with round tips). Each cutter has slightly different end planes (exact boolean)."""
    c = HORN_CLEAR
    L = trim_r if trim_r else HORN_CROSS_STRAIGHT        # end of the straight (tapered) arm
    w_end = HORN_ARM_W_ROOT + (HORN_ARM_W_TIP - HORN_ARM_W_ROOT) * L / HORN_CROSS_L
    cuts = [add_cyl(new_bm(), 0, 0, z0 - 0.07, z1 + 0.05, HUB_D + 2 * c, 40, M)]
    for k in range(4):
        Mk = _rot(M, angle_deg + 90 * k)
        quad = [(-0.6, -HORN_ARM_W_ROOT / 2 - c), (L, -w_end / 2 - c), (L, w_end / 2 + c), (-0.6, HORN_ARM_W_ROOT / 2 + c)]
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
            half_w = (HORN_ARM_W_ROOT + (HORN_ARM_W_TIP - HORN_ARM_W_ROOT) * along / HORN_CROSS_L) / 2
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
    """ONE closed cutter for an X-horn pocket with its crush ribs already subtracted. Cut it once."""
    return combine_cutters(pocket_cutters(M, z0, z1, trim_r, angle_deg),
                           rib_adds(M, z0 + 0.6, z1 + 0.4, trim_r, angle_deg))   # mouth 0.1 inside the part's face (z0 is 0.5 outside)


def saddle_walls(M, z_lo, z_hi):
    """Two walls hugging the servo body sides (0.15 clearance), full body length."""
    x0, x1 = J_BODY_X[0] - 0.2, J_BODY_X[1] + 0.2
    yi = J_BODY_HW + SADDLE_CLEAR
    return [add_box(new_bm(), x0, x1, s * yi, s * (yi + SADDLE_T), z_lo, z_hi, M) for s in (-1, 1)]


def ear_pads(M, z_lo, z_hi, hw=J_EAR_HW):
    """Pads beyond the body ends under/over the ears (they also stop the body in X). Their outer ends
    sit 0.05 inside the plate's ends: a face flush with the plate's end face breaks the exact boolean."""
    return [add_box(new_bm(), x0, x1, -hw, hw, z_lo, z_hi, M)
            for x0, x1 in ((J_EAR_X[0] + 0.05, J_BODY_X[0] - 0.2), (J_BODY_X[1] + 0.2, J_EAR_X[1] - 0.05))]


def cable_channel(M):
    """Channel for the cable leaving the shaft-end face at the bottom: through the shaft-end pad."""
    (x0, x1), (y0, y1), (z0, z1) = J_CABLE
    return add_box(new_bm(), J_EAR_X[0] - 0.6, x1, y0 - 0.6, y1 + 0.6, z0 - 0.6, z1 + 1.2, M)


# ================================================================== type C
def cap_features(M, include_plate=True, saddle=True):
    """(add, sub) for a type-C stator: cap plate on the tower top (z 0..CAP_T) with the 16x22x4 seat,
    pads hanging to the ear tops (servo screws from below), saddle walls round the body."""
    pts = [(x, y) for x in J_EAR_X for y in (-CAP_HW, CAP_HW)] + \
          [(CAP_RING_R * math.cos(2 * math.pi * i / 48), CAP_RING_R * math.sin(2 * math.pi * i / 48)) for i in range(48)]
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (0.0, CAP_T)], M)] if include_plate else []
    add += ear_pads(M, J_EAR_Z[1], 0.6)
    if saddle:
        add += saddle_walls(M, -SADDLE_DEPTH, 0.6)
    sub = [add_cyl(new_bm(), 0, 0, LIP, CAP_T + 1, SEAT_D, 64, M),
           add_cyl(new_bm(), 0, 0, -1, LIP + 0.01, LIP_HOLE_D, 48, M)]
    for hx in J_HOLE_X:
        sub.append(add_cyl(new_bm(), hx, 0, J_EAR_Z[1] - 0.5, J_EAR_Z[1] + 4.4, PILOT_D, 16, M))
    return add, sub


def link_features(M, plate_outline_mm=None, angle_deg=0.0):
    """(add, sub, ribs) for a type-C rotor: boss in the bearing, shoulder on the inner ring, plate,
    trimmed X pocket inside the boss, M2x6 counterbored clamp hole."""
    r = BOSS_D / 2 + 3.0
    pts = [(r * math.cos(2 * math.pi * i / 48), r * math.sin(2 * math.pi * i / 48)) for i in range(48)] + list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (PLATE_Z0, PLATE_Z1)], M),
           add_cyl(new_bm(), 0, 0, BOSS_Z0, PLATE_Z0 + 1.0, BOSS_D, 64, M),
           add_cyl(new_bm(), 0, 0, LIP + BRG_W, PLATE_Z0 + 0.6, SHOULDER_D, 48, M)]
    sub = [pocket_cutter(M, BOSS_Z0 - 0.5, HORN_TOP + 0.1, TRIM_R, angle_deg),
           add_cyl(new_bm(), 0, 0, BOSS_Z0 - 1.1, PLATE_Z1 + 1.1, CLEAR_D, 20, M),
           add_cyl(new_bm(), 0, 0, CB_Z, PLATE_Z1 + 0.9, CB_D, 24, M)]
    return add, sub, []


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
        sub.append(add_cyl(new_bm(), hx, 0, J_EAR_Z[0] - EAR_SCREW_L, J_EAR_Z[0] + 0.5, PILOT_D, 16, M))
    return add, sub


def horn_plate_features(M, plate_outline_mm=None, angle_deg=45.0, trim_r=None):
    """(add, sub, ribs) for the horn side of a type-F rotor: plate ARM_Z with the full X pocket, M2 clamp hole."""
    r = (trim_r + HORN_CLEAR + 1.8) if trim_r else ARM_R_FULL
    pts = [(r * math.cos(2 * math.pi * i / 64), r * math.sin(2 * math.pi * i / 64)) for i in range(64)] + list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in ARM_Z], M)]
    sub = [pocket_cutter(M, ARM_Z[0] - 0.5, HORN_TOP + 0.1, trim_r, angle_deg),
           add_cyl(new_bm(), 0, 0, ARM_Z[0] - 1, ARM_Z[1] + 1, CLEAR_D, 20, M)]
    return add, sub, []


def bearing_plate_features(M, plate_outline_mm=None):
    """(add, sub) for the far side of a type-F rotor: plate with the 693ZZ seat (open to the stator side,
    0.2 lip outside) around the M3 axle. Screws to the rotor are added by the caller."""
    z1, z0 = BP_INNER_Z, BP_INNER_Z - BP_T
    r = BP_SEAT_D / 2 + 2.6
    pts = [(r * math.cos(2 * math.pi * i / 48), r * math.sin(2 * math.pi * i / 48)) for i in range(48)] + list(plate_outline_mm or [])
    add = [add_hull(new_bm(), [(x, y, z) for x, y in pts for z in (z0, z1)], M)]
    sub = [add_cyl(new_bm(), 0, 0, z1 - B693[2], z1 + 1, BP_SEAT_D, 48, M),
           add_cyl(new_bm(), 0, 0, z0 - 1, z1 + 1, BP_LIP_D, 32, M)]
    return add, sub


# ================================================================== hardware dummies (for views and overlap checks)
def hardware_C(prefix, M, coll):
    steel = material("hw_steel", (0.6, 0.62, 0.66, 1))
    brg = to_object(f"{prefix}brg_16x22x4", add_cyl(new_bm(), 0, 0, LIP, LIP + BRG_W, BRG_OD, 64, M), coll, steel)
    apply_bool(brg, add_cyl(new_bm(), 0, 0, LIP - 1, LIP + BRG_W + 1, BRG_ID, 64, M), "DIFFERENCE", coll)
    scr = new_bm()
    add_cyl(scr, 0, 0, CB_Z, CB_Z + 1.4, 3.6, 20, M)                     # M2x6 pan head in the counterbore
    add_cyl(scr, 0, 0, CB_Z - 6.0, CB_Z + 0.01, 2.0, 16, M)
    return [brg, to_object(f"{prefix}screw_M2x6", scr, coll, steel)]


def hardware_F(prefix, M, coll):
    steel = material("hw_steel", (0.6, 0.62, 0.66, 1))
    z1 = BP_INNER_Z
    brg = to_object(f"{prefix}brg_693", add_cyl(new_bm(), 0, 0, z1 - B693[2], z1, B693[1], 48, M), coll, steel)
    apply_bool(brg, add_cyl(new_bm(), 0, 0, z1 - B693[2] - 1, z1 + 1, B693[0], 32, M), "DIFFERENCE", coll)
    scr = new_bm()
    add_cyl(scr, 0, 0, z1 - B693[2] - 3.0, z1 - B693[2] + 0.01, 5.5, 24, M)   # M3 socket head outside the bearing
    add_cyl(scr, 0, 0, z1 - B693[2], z1 - B693[2] + 8.0, 3.0, 16, M)         # M3x8 shank: through bearing, island, floor, nut
    nut = to_object(f"{prefix}nut_M3", add_cyl(new_bm(), 0, 0, FLOOR_Z[1] - NUT_T + 0.2, FLOOR_Z[1] - 0.05, 5.5 / math.cos(math.pi / 6), 6, M), coll, steel)
    apply_bool(nut, add_cyl(new_bm(), 0, 0, FLOOR_Z[1] - NUT_T - 1, FLOOR_Z[1] + 1, 3.0, 16, M), "DIFFERENCE", coll)
    horn_scr = new_bm()
    add_cyl(horn_scr, 0, 0, ARM_Z[1], ARM_Z[1] + 1.4, 3.6, 20, M)
    add_cyl(horn_scr, 0, 0, ARM_Z[1] - 6.0, ARM_Z[1] + 0.01, 2.0, 16, M)
    return [brg, to_object(f"{prefix}screw_M3x8", scr, coll, steel), nut, to_object(f"{prefix}screw_M2x6", horn_scr, coll, steel)]


# ================================================================== horn trimming jig (type C horns)
def trim_jig(name, coll, M, trim_r=TRIM_R):
    """Ø(2 trim_r) block: the full X horn drops in hub-first, its arms stick out through 4 slots in the
    wall; cut them flush with the wall -> every arm ends at trim_r."""
    H_HUB = HORN_HUB_H - HORN_PLATE_T + 0.3           # hub below the arms, +0.3 so the arms sit proud of nothing
    base, top = 1.2, 1.2 + H_HUB + HORN_PLATE_T + 0.2
    body = add_cyl(new_bm(), 0, 0, 0.0, top, 2 * trim_r, 64, M)
    jig = to_object(name, body, coll, material("jig", (0.3, 0.75, 0.35, 1)))
    cuts = [add_cyl(new_bm(), 0, 0, base, top + 1, HUB_D + 0.3, 40, M)]
    for k in range(4):
        Mk = _rot(M, 90 * k)
        w0, w1 = HORN_ARM_W_ROOT / 2 + 0.1, HORN_ARM_W_TIP / 2 + 0.1
        quad = [(-0.6, -w0), (trim_r + 3, -w1), (trim_r + 3, w1), (-0.6, w0)]
        cuts.append(add_hull(new_bm(), [(x, y, z) for x, y in quad for z in (base + H_HUB - 0.3, top + 1)], Mk))
    for c in cuts:
        apply_bool(jig, c, "DIFFERENCE", coll)
    jig["purpose"] = f"cut the X horn arms flush with the wall: trim radius {trim_r} mm"
    return jig
