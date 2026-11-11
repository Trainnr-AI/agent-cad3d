"""XGO-Duck body v2: one printed frame that holds the two hip-yaw servos, carries the neck, and houses the
off-the-shelf electronics; plus a lid and simple solids for the electronics (for clearance checks).
Runs inside Blender after the servos, legs and neck are built.

  body_frame   THE MOTOR BODY, nothing else: deck with the two hip-yaw caps (6702ZZ seats, pads, saddle walls) -
               replaces hip_yaw_support; neck pedestal + horn plate (full X pocket) - replaces body_neck_mount, moved
               straight under the neck axis so it clears the hip-yaw servos (the old web cut into both of them).
  electronics_enclosure   separate part behind the hips, shaped to what the legs and the head leave free: a keel
               (balance sensor on the floor, servo adapter + regulator on two rails above it) and a pocket above it (battery on its shelf); hangs on the body by a tongue under
               the deck + 3x M2x6 into the deck's rear edge (driven from behind through access holes).
  body_lid     top plate on the enclosure's posts, 4x M2x6: closes the pocket and carries, on stand-offs,
               the Uno Q (it fits nowhere inside: 69 x 53 mm against a 36-44 mm wide free space).
  electronics  collection of boxes: uno_q, battery_3s, servo_adapter, ubec, imu (the bus board is drawn by wiring.py; sizes: see ELEC; the
               battery and regulator sizes are ASSUMED, the others from the makers' pages)

All boxes are given in "layout coordinates" = world mm with the duck at its usual place (trunk at 0, -100, 118.8),
and converted through the trunk frame, so the body follows the robot wherever it stands.

ARGS = {"robot": "duck_scs0009", "collection": "body_v2", "joint_lib": "/abs/parts/joint_v2.py",
        "base_lib": "/abs/parts/joint_orig_style.py", "dims": "/abs/parts/scs0009_dims.py"}
"""
import math

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "body_v2")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
ecoll = bpy.data.collections.get("electronics") or bpy.data.collections.new("electronics")
if ecoll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(ecoll)
for c in (coll, ecoll):
    for o in list(c.objects):
        bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()
trunk = bpy.data.objects[f"{R}:trunk_base"]
Mt = trunk.matrix_world.copy()
HOME = Vector((0.0, -100.0, 118.8))                     # trunk position the layout numbers were taken at
FyL = bpy.data.objects["scs_left_hip_yaw"].matrix_world.copy()
FyR = bpy.data.objects["scs_right_hip_yaw"].matrix_world.copy()
Fn = bpy.data.objects["scs_neck_pitch"].matrix_world.copy()
mat_frame = L["material"]("body_frame", (0.85, 0.75, 0.2, 1))
mat_lid = L["material"]("body_lid", (0.9, 0.82, 0.35, 1))


def box(x0, x1, y0, y1, z0, z1):
    """box given in layout coordinates"""
    return L["add_box"](L["new_bm"](), x0 - HOME.x, x1 - HOME.x, y0 - HOME.y, y1 - HOME.y, z0 - HOME.z, z1 - HOME.z, Mt)


SLOPE = 1.45                              # undersides rise 1.45 per 1 out (55 deg from the bed: inside print_check's 50 deg limit)


def hull(pts):
    """convex solid through points given in layout coordinates"""
    return L["add_hull"](L["new_bm"](), [(x - HOME.x, y - HOME.y, z - HOME.z) for x, y, z in pts], Mt)


def lay(M, p_frame_mm):
    """a servo-frame point -> layout coordinates"""
    return Mt.inverted() @ (M @ (Vector(p_frame_mm) * MM)) / MM + HOME


def to_frame(M, p_layout_mm):
    return (M.inverted() @ (Mt @ ((Vector(p_layout_mm) - HOME) * MM))) / MM


report = []


def finish(name, part):
    ok, vol = L["manifold"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    part["manifold"] = ok
    n = L["shells"](part)
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    return part


# ------------------------------------------------------------------ layout (mm, layout coordinates)
# The envelope comes from tools/blender/swept_space.py (what the legs and the head leave free, 1.5 mm clearance):
#   keel behind the hips   x -52..-6,  z 57..96,   half-width 18   (the feet swing past it when a leg goes back)
#   pocket above it        x -52..-5,  z 99..113 (rear) / 125 (front of x = -33), half-width 22
#   deck                   only the two hip-yaw caps + a centre strip to the neck pedestal (the L-brackets rise beside it)
#   nothing above z ~126 except behind x = 12, and there the head comes down when the neck tilts back
T = 1.6                                   # wall thickness
CY = -100.0                               # centre plane
XR, XK1 = -53.0, -6.0                     # keel / pocket rear face, keel front face
HK, HP = 18.0, 22.0                       # half-widths: keel, pocket
ZB = 57.0                                 # keel underside
z_deck0, z_deck1 = lay(FyL, (0, 0, L["CAP_T"])).z, lay(FyL, (0, 0, 0)).z      # deck = the hip-yaw cap plate (94.2 .. 99.2)
ZP_REAR, ZP_FRONT, XP_STEP = 113.0, 125.0, -36.0
XP1 = -5.0                                # pocket front face (the hip-yaw servos' rear ears are at -3.6)
XD = 43.0                                 # centre strip's front edge (under the neck)
HS = 9.0                                  # centre strip half-width (the L-brackets rise beside it when the hips roll out)
UNO = dict(len=68.85, wid=53.34, pcb=1.6, top=10.0, pins=3.0)
ELEC = {"battery_3s": (40.0, 30.0, 13.0), "servo_adapter": (42.0, 18.5, 33.0), "ubec": (43.0, 8.0, 18.0), "imu": (25.6, 22.7, 4.6)}
LIFT = 1.6                                # the battery shelf sits this much above the deck's top face (room for the adapter standing on its rails)
z_sh = z_deck1 + LIFT
# No electronics on the motor body: the custom bus board is gone (kit hubs only). What the body still offers the wiring:
PORT = dict(y0=CY + 8.6, y1=CY + HK - T - 0.4, z0=86.5, z1=93.0)          # cable port through the keel's front wall, beside the tongue, under the deck
CHEST = dict(x0=XD + 0.3, t=1.6, y0=CY - 1.5, y1=CY + 21.0, z0=None, z1=105.5)    # chest plate standing on the front of the centre strip: seat for ONE kit hub (hub_seats.py); from the deck's underside up, so the body still prints flat on that face
IMU_XY, IMU_HOLES, IMU_BOSS = (-29.5, CY), (10.16, 8.89), 3.4               # balance sensor flat on the keel floor: centre, hole half-pitches, boss height
RAIL_H = 8.6                              # the adapter and the regulator stand on two rails at the keel's ends, above the sensor
Y = lambda h: (CY - h, CY + h)

# ------------------------------------------------------------------ electronics enclosure: simple boxes unioned first
tub = []
# keel: open box (open at the top into the pocket)
(y0, y1) = Y(HK)
tub += [box(XR, XK1, y0, y1, ZB, ZB + T),                                  # floor
        box(XR, XR + T, y0, y1, ZB + 0.3, z_deck1),                        # rear wall
        box(XK1 - T, XK1, y0, y1, ZB + 0.3, z_deck1 - 0.3),                # front wall
        box(XR + 0.3, XK1 - 0.3, y0, y0 + T, ZB + 0.3, z_deck1),           # sides
        box(XR + 0.3, XK1 - 0.3, y1 - T, y1, ZB + 0.3, z_deck1)]
# pocket floor ring: joins the wider pocket to the keel (a 4 mm ledge each side), open in the middle
(p0, p1) = Y(HP)
tub += [box(XR, XP1, p0, y0 + T, z_deck0 + 2.0, z_sh), box(XR, XP1, y1 - T, p1, z_deck0 + 2.0, z_sh)]
# pocket walls: rear part low, front part high
tub += [box(XR, XR + T, p0, p1, z_deck0 + 2.3, ZP_REAR),
        box(XR + 0.3, XP1, p0, p0 + T, z_deck0 + 2.3, ZP_REAR), box(XR + 0.3, XP1, p1 - T, p1, z_deck0 + 2.3, ZP_REAR),
        box(XP_STEP, XP1, p0, p0 + T, ZP_REAR - 0.3, ZP_FRONT), box(XP_STEP, XP1, p1 - T, p1, ZP_REAR - 0.3, ZP_FRONT),
        box(XP1 - T, XP1, p0 + 0.3, p1 - 0.3, z_deck0 + 2.3, ZP_FRONT)]              # front wall (behind the hip-yaw servos)
# printable without supports (enclosure stands on its keel floor): every underside is at 45 deg or steeper
for sgn in (-1, 1):                                                       # under the pocket's ledges
    ya, yb = CY + sgn * (HK - 0.3), CY + sgn * HP
    tub.append(hull([(x, y, z) for x in (XR, XP1) for y, z in ((ya, z_deck0 + 2.3), (yb, z_deck0 + 2.3), (ya, z_deck0 + 2.0 - SLOPE * (HP - HK) - 0.3))]))
enc = list(tub)                                                           # everything so far is the enclosure
tub = []
# ------------------------------------------------------------------ motor body: deck with the two hip-yaw caps (as the old hip
# plate) + a centre strip forward to the neck pedestal. Nothing of the electronics is part of it.
cap_x = sorted((lay(FyL, (L["J_EAR_X"][0], 0, 0)).x, lay(FyL, (L["J_EAR_X"][1], 0, 0)).x))
cap_yL = sorted((lay(FyL, (0, -L["CAP_HW"], 0)).y, lay(FyL, (0, L["CAP_HW"], 0)).y))
cap_yR = sorted((lay(FyR, (0, -L["CAP_HW"], 0)).y, lay(FyR, (0, L["CAP_HW"], 0)).y))
X_JOIN = XP1 + 0.15                                                                 # deck's rear edge: 0.15 behind it starts the enclosure
tub += [box(X_JOIN, cap_x[1], cap_yR[0], cap_yL[1], z_deck0, z_deck1),             # one plate under both servos
        box(cap_x[1] - 0.6, XD, CY - HS, CY + HS, z_deck0, z_deck1)]
# (no ring round the bearing seats beyond this plate: outboard of it the L-bracket's pitch ring rises at roll-out,
#  exactly as with the old hip plate - the seat is open over that arc)
tub.append(box(CHEST["x0"] - 0.6, CHEST["x0"] + CHEST["t"], CHEST["y0"], CHEST["y1"], z_deck0, CHEST["z1"]))        # chest plate (0.6 into the strip's front end)
body_solids = list(tub)
tub = []
# top plate posts: on the pocket's high walls (enclosure)
POSTS = [(XP_STEP + T - 0.3, p0 + T - 0.3), (XP_STEP + T - 0.3, p1 - T - 5.0 + 0.3), (XP1 - T - 5.0 + 0.3, p0 + T - 0.3), (XP1 - T - 5.0 + 0.3, p1 - T - 5.0 + 0.3)]
for px, py in POSTS:
    enc.append(box(px, px + 5.0, py, py + 5.0, ZP_FRONT - 4.6, ZP_FRONT - 0.01))
    yw = p0 + T - 0.3 if py < CY else p1 - T + 0.3                        # corbel under the post, down the side wall
    yo = py + 5.0 if py < CY else py
    enc.append(hull([(x, y, z) for x in (px, px + 5.0) for y, z in ((yw, ZP_FRONT - 4.3), (yo, ZP_FRONT - 4.3), (yw, ZP_FRONT - 4.6 - SLOPE * 5.3))]))      # short: cables pass under them
# how the enclosure hangs on the body: a tongue under the deck (centre, between the hips) + 3x M2x6 through its front wall
# into the deck's rear edge, driven from behind through access holes in the rear wall
TONGUE = (XP1 - 0.3, 3.0, 8.0, 3.0)                                        # from x, to x, half-width, thickness
enc.append(box(TONGUE[0], TONGUE[1], CY - TONGUE[2], CY + TONGUE[2], z_deck0 - 0.2 - TONGUE[3], z_deck0 - 0.2))
enc.append(box(XK1 - 0.3, TONGUE[0] + 0.3, CY - TONGUE[2], CY + TONGUE[2], z_deck0 - 0.2 - TONGUE[3], z_deck0 - 0.2))   # ties the tongue to the keel's front wall
zt = z_deck0 - 0.2 - TONGUE[3]                                             # gusset under the tongue, down the keel's front wall
enc.append(hull([(x, y, z) for y in (CY - TONGUE[2], CY + TONGUE[2]) for x, z in ((XK1 - 0.3, zt + 0.3), (TONGUE[1], zt + 0.3), (XK1 - 0.3, zt - SLOPE * (TONGUE[1] - XK1) - 0.3))]))
JOIN_Y = (-8.0, 0.0, 8.0)
z_join = (z_deck0 + z_deck1) / 2
# neck pedestal: straight under the neck axis, between the horn plate and the bearing plate
WEB_R, WEB_HW = (L["ARM_R_FULL"] + 0.5, L["ARM_R_FULL"] + 6.5), 6.0
ARM, BP_Z = L["ARM_Z"], (L["BP_INNER_Z"] - L["BP_T"], L["BP_INNER_Z"])
wd = (Fn.inverted().to_3x3() @ (Mt.to_3x3() @ Vector((0, 0, -1))))
wd.z = 0
wd.normalize()                                                            # "down" in the neck frame's XY
tn = Vector((-wd.y, wd.x, 0))
web_xy = [tuple((wd * r + tn * s * WEB_HW)[:2]) for r in WEB_R for s in (-1, 1)]
web = L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in web_xy for z in (BP_Z[1], ARM[1] - 0.05)], Fn)
nx = lay(Fn, (0, 0, 0)).x
y_in, y_out = sorted((lay(Fn, (0, 0, BP_Z[1])).y, lay(Fn, (0, 0, ARM[1] - 0.05)).y))
z_web = lay(Fn, (0, 0, 0)).z - WEB_R[0]
# full fork width only near the top; below that the pedestal keeps to the strip's +Y edge (room for the left L-bracket at roll-out)
pedestal = L["combine_cutters"]([box(nx - WEB_HW + 0.2, nx + WEB_HW - 0.2, y_in + 0.2, y_out - 0.2, z_web - 6.0, z_web - 1.0),
                                 box(nx - WEB_HW + 0.25, nx + WEB_HW - 0.25, y_in + 0.25, CY + HS - 0.3, z_deck1 - 0.5, z_web - 5.7)])
ped_w = y_out - 0.2 - (CY + HS - 0.3)                                      # how far the wide top overhangs the narrow base
ped_gusset = hull([(x, y, z) for x in (nx - WEB_HW + 0.25, nx + WEB_HW - 0.25) for y, z in ((CY + HS - 0.6, z_web - 5.7), (y_out - 0.2, z_web - 5.7), (CY + HS - 0.6, z_web - 6.0 - SLOPE * ped_w - 0.3))])
ped_foot = box(nx - WEB_HW + 0.2, nx + WEB_HW - 0.2, y_in + 0.3, CY + HS - 0.35, z_deck0 + 0.02, z_deck1 + 2.0)   # ties the pedestal into the strip
arm_reach = [tuple((wd * r + tn * s * (WEB_HW - 0.4))[:2]) for r in (WEB_R[0] + 0.4, WEB_R[1] - 0.4) for s in (-1, 1)]
a_add, a_sub, a_rib = L["horn_plate_features"](Fn, arm_reach, angle_deg=45.0)
web_screws = [tuple((wd * sum(WEB_R) / 2 + tn * s * 3.5)[:2]) for s in (-1, 1)]
a_sub += L["screw_holes"](Fn, web_screws, BP_Z[1], 0.0, 7.0, +1, csk=False)          # pilots for the bearing plate's screws
# hip-yaw caps
cL, sL = L["cap_features"](FyL, include_plate=False, saddle=True)
cR, sR = L["cap_features"](FyR, include_plate=False, saddle=True)
servo_front = max(lay(FyL, (L["J_BODY_X"][0], 0, 0)).x, lay(FyL, (L["J_BODY_X"][1], 0, 0)).x)
z_holder_top = lay(FyL, (0, 0, -L["SADDLE_DEPTH"])).z          # top of the hip-yaw servo's saddle walls
MX = Mt @ Matrix.Rotation(math.radians(90), 4, "Y")                       # a frame whose +Z points along layout +X (for the horizontal screws)


def xpt(y, z):
    """layout (y, z) of a point on a plane x = const -> (x, y) in MX's own frame (MX: x' = -z, y' = y, z' = x)"""
    return (-(z - HOME.z), y - HOME.y)


cuts = [box(servo_front - 12.0, servo_front + 1.5, y_out - (ARM[1] - ARM[0]) - 0.6, y_out + 2.0, z_holder_top + 2.0, ZP_FRONT + 30.0),   # horn plate, where it nears the left servo
        box(6.5, 23.0, CY - 4.5, CY + 4.5, z_deck0 - 1.0, z_deck1 + 1.0)]      # cable slot through the deck slot between the two servos (clear of the enclosure's tongue)
# the pedestal's front top corner, cut at 45 deg: the cradle's edge passes there when the neck is fully forward
Mc = Mt @ Matrix.Translation(Vector((nx + 3.1 - HOME.x, 0.0, z_web - HOME.z)) * MM) @ Matrix.Rotation(math.radians(45), 4, "Y")
cuts.append(L["add_box"](L["new_bm"](), -12.0, 12.0, y_in - 1.0 - HOME.y, y_out - (ARM[1] - ARM[0]) - 0.35 - HOME.y, 0.0, 14.0, Mc))
for y in JOIN_Y:                                                           # pilots in the deck's rear edge for the enclosure's screws
    cuts.append(L["add_cyl"](L["new_bm"](), *xpt(CY + y, z_join), X_JOIN - HOME.x - 0.5, X_JOIN - HOME.x + 4.6, L["PILOT_D"], 14, MX))
frame = finish("body_frame", L["build_part"]("body_frame", [L["combine_cutters"](body_solids + [pedestal, ped_gusset, ped_foot, web] + cL + cR)] + a_add,   # all simple solids unioned first
                                             a_sub + [L["combine_cutters"](sL), L["combine_cutters"](sR)] + cuts, coll, mat_frame, a_rib))   # each cap's cuts as ONE cutter
enc_cuts = []
for px, py in POSTS:
    enc_cuts.append(L["add_cyl"](L["new_bm"](), px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZP_FRONT - 5.5 - HOME.z, ZP_FRONT + 1.0 - HOME.z, L["PILOT_D"], 14, Mt))
enc_cuts.append(box(XR - 0.5, XR + T + 0.5, CY - 5.3, CY + 5.3, z_sh + 0.0, z_sh + 2.0))            # slot for the battery shelf's tab
for sgn in (-1, 1):                                                        # pilots for the shelf's two screws, down through the ledge and its boss
    enc_cuts.append(L["add_cyl"](L["new_bm"](), XP1 - T - 4.2 - HOME.x, CY + sgn * (HP - T - 2.0) - HOME.y, z_sh - 4.7 - HOME.z, z_sh + 0.5 - HOME.z, L["PILOT_D"], 14, Mt))
enc_cuts.append(box(XP1 - T - 0.5, XP1 + 0.5, CY - 8.5, CY + 8.5, ZP_FRONT - 5.5, ZP_FRONT + 1.0))      # cable notch in the pocket's front wall: the servo adapter's cable to the bus board
enc_cuts.append(box(XK1 - T - 0.5, XK1 + 0.5, PORT["y0"], PORT["y1"], PORT["z0"], PORT["z1"]))       # cable port: the adapter's two cables leave the keel here
for y in JOIN_Y:
    enc_cuts.append(L["add_cyl"](L["new_bm"](), *xpt(CY + y, z_join), XK1 - T - 0.6 - HOME.x, XP1 + 0.6 - HOME.x, L["CLEAR_D"], 16, MX))      # clearance through the front walls (keel + pocket: 2.6 mm)
    enc_cuts.append(L["add_cyl"](L["new_bm"](), *xpt(CY + y, z_join), XR - 0.6 - HOME.x, XR + T + 0.6 - HOME.x, 4.6, 20, MX))                 # driver access through the rear wall
# keel fittings: sensor bosses on the floor, two rails across the ends (adapter + regulator stand on them, above the sensor),
# and a fence post on each rail between the adapter and the regulator
zk0 = ZB + T
for sx in (-1, 1):
    for sy in (-1, 1):
        ix, iy = IMU_XY[0] + sx * IMU_HOLES[0], IMU_XY[1] + sy * IMU_HOLES[1]
        enc.append(L["add_cyl"](L["new_bm"](), ix - HOME.x, iy - HOME.y, zk0 - 0.3 - HOME.z, zk0 + IMU_BOSS - HOME.z, 4.6, 20, Mt))
        enc_cuts.append(L["add_cyl"](L["new_bm"](), ix - HOME.x, iy - HOME.y, zk0 + IMU_BOSS - 4.6 - HOME.z, zk0 + IMU_BOSS + 0.5 - HOME.z, L["PILOT_D"], 14, Mt))
y_ad = CY - HK + T + 0.4 + ELEC["servo_adapter"][1]                          # the adapter's inner face
y_ub = CY + HK - T - 0.4 - ELEC["ubec"][1]                                   # the regulator's inner face
for xa, xb in ((XR + T - 0.3, XR + T + 5.4), (XK1 - T - 5.4, XK1 - T + 0.3)):
    enc.append(box(xa, xb, CY - HK + T - 0.3, CY + HK - T + 0.3, zk0 - 0.3, zk0 + RAIL_H))
    enc.append(box(xa, xb, y_ad + 0.4, y_ub - 0.4, zk0 + RAIL_H - 0.3, zk0 + RAIL_H + 16.0))
mat_enc = L["material"]("electronics_enclosure", (0.35, 0.55, 0.75, 1))
enclosure = finish("electronics_enclosure", L["build_part"]("electronics_enclosure", [L["combine_cutters"](enc)], enc_cuts, coll, mat_enc))
enc_screws = L["pan_screw_dummy"]("enclosure_screws_M2x6st", [(MX, *xpt(CY + y, z_join), XK1 - T - HOME.x, +1) for y in JOIN_Y], coll)   # head on the keel wall's inner face

# ------------------------------------------------------------------ top plate: closes the pocket and carries the Uno Q above the body
LID_T = 2.0
ZL = ZP_FRONT + 0.1
UX0 = XR + 0.2                                                             # Uno Q: from the enclosure's rear face forward (x -52.8 .. 16.1)
UY0 = CY - UNO["wid"] / 2
UNO_HOLES = [(14.0, 2.5), (15.2, 50.7), (66.0, 7.6), (66.0, 35.5)]         # UNO form-factor mounting holes (Ø3.2), from the board's corner - ASSUMED for the Uno Q
STAND = 4.5                                                                # posts under the board (its underside pins clear the top plate's screw heads)
uno_z = ZL + LID_T + STAND
lid_add = [box(XR + 0.3, UX0 + UNO["len"] - 0.5, CY - UNO["wid"] / 2 - 0.5, CY + UNO["wid"] / 2 + 0.5, ZL, ZL + LID_T)]      # as long as the board: also covers the pocket's rear part
for hx, hy in UNO_HOLES:
    lid_add.append(L["add_cyl"](L["new_bm"](), UX0 + hx - HOME.x, UY0 + hy - HOME.y, ZL + LID_T - 0.3 - HOME.z, uno_z - HOME.z, 6.4, 28, Mt))
lid_sub = []
for px, py in POSTS:
    M = Mt @ Matrix.Translation(Vector((px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZL + LID_T - HOME.z)) * MM)
    lid_sub += L["screw_holes"](M, [(0, 0)], 0.0, LID_T, 0.0, -1, csk=False)
for hx, hy in UNO_HOLES:                                                   # M3 thread-forming pilots down the posts and through the plate
    lid_sub.append(L["add_cyl"](L["new_bm"](), UX0 + hx - HOME.x, UY0 + hy - HOME.y, ZL - 1.0 - HOME.z, uno_z + 1.0 - HOME.z, 2.5, 16, Mt))
lid_sub.append(box(XR - 1.0, XR + 9.0, CY - 12.0, CY + 12.0, ZL - 1.0, ZL + LID_T + 1.0))      # wire notch at the rear: battery / IMU / UART wires come up here to the board
lid = finish("body_lid", L["build_part"]("body_lid", [L["combine_cutters"](lid_add)], lid_sub, coll, mat_lid))
us = L["new_bm"]()
for hx, hy in UNO_HOLES:                                                   # M3x8 socket-head screws: through the board (1.6) into the posts (6.4)
    L["add_cyl"](us, UX0 + hx - HOME.x, UY0 + hy - HOME.y, uno_z + UNO["pcb"] - HOME.z, uno_z + UNO["pcb"] + 3.0 - HOME.z, 5.5, 24, Mt)
    L["add_cyl"](us, UX0 + hx - HOME.x, UY0 + hy - HOME.y, uno_z + UNO["pcb"] - 8.0 - HOME.z, uno_z + UNO["pcb"] + 0.01 - HOME.z, 3.0, 16, Mt)
uno_screws = L["to_object"]("uno_screws_M3x8", us, coll, L["material"]("hw_steel", (0.6, 0.62, 0.66, 1)))
lid_screws = L["pan_screw_dummy"]("body_lid_screws_M2x6st", [(Mt @ Matrix.Translation(Vector((px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZL + LID_T - HOME.z)) * MM), 0, 0, 0.0, -1) for px, py in POSTS], coll)

# ------------------------------------------------------------------ electronics (boxes)
def ebox(name, x0, y0, z0, size, rgba):
    bm = box(x0, x0 + size[0], y0, y0 + size[1], z0, z0 + size[2])
    ob = L["to_object"](name, bm, ecoll, L["material"]("elec_" + name.rstrip("_123456"), rgba))
    return ob


elec = []
u = L["new_bm"]()
for b in (box(UX0, UX0 + UNO["len"], CY - UNO["wid"] / 2, CY + UNO["wid"] / 2, uno_z, uno_z + UNO["pcb"]),
          box(UX0 + 2.0, UX0 + UNO["len"] - 2.0, CY - UNO["wid"] / 2 + 1.3, CY + UNO["wid"] / 2 - 1.3, uno_z + UNO["pcb"], uno_z + UNO["pcb"] + UNO["top"]),
          box(UX0 + 19.5, UX0 + 61.5, CY - UNO["wid"] / 2 + 6.5, CY + UNO["wid"] / 2 - 6.5, uno_z - UNO["pins"] + 0.3, uno_z)):      # underside pins: not at the mounting holes
    b.to_mesh(tmp := bpy.data.meshes.new("t"))
    u.from_mesh(tmp)
    bpy.data.meshes.remove(tmp)
elec.append(L["to_object"]("uno_q", u, ecoll, L["material"]("elec_uno", (0.0, 0.45, 0.55, 1))))
zk = ZB + T + RAIL_H                                                        # top of the rails
elec.append(ebox("servo_adapter", XR + T + 0.4, CY - HK + T + 0.4, zk, ELEC["servo_adapter"], (0.1, 0.25, 0.7, 1)))
elec.append(ebox("ubec", XR + T + 0.4, CY + HK - T - 0.4 - ELEC["ubec"][1], zk, ELEC["ubec"], (0.75, 0.1, 0.1, 1)))
SHELF_T = 1.6                                                               # loose shelf on the pocket's ledges: the battery lies on it, above the keel
SH_SCREW = (XP1 - T - 4.2, HP - T - 2.0)                                   # shelf screws: x, |y - CY| (over the ledges, where the legs leave most room)
shelf_add = [box(XR + T + 0.3, XP1 - T - 0.3, CY - HP + T + 0.3, CY + HP - T - 0.3, z_sh + 0.1, z_sh + 0.1 + SHELF_T),
             box(XR + 0.25, XR + T + 0.6, CY - 5.0, CY + 5.0, z_sh + 0.15, z_sh + 0.05 + SHELF_T)]       # tab into the rear wall's slot
BAT = ELEC["battery_3s"]
for sgn in (-1, 1):                                                         # battery seat: a rib each side and two stops at its front end (hook-and-loop patch underneath)
    ya = sorted((CY + sgn * (BAT[1] / 2 + 0.4), CY + sgn * (BAT[1] / 2 + 1.6)))
    shelf_add.append(box(XR + T + 4.0, XR + T + 34.0, ya[0], ya[1], z_sh + SHELF_T - 0.2, z_sh + 0.1 + SHELF_T + 6.0))
    yb = sorted((CY + sgn * 8.6, CY + sgn * 14.0))
    shelf_add.append(box(XR + T + 0.3 + BAT[0] + 0.4, XR + T + 0.3 + BAT[0] + 1.6, yb[0], yb[1], z_sh + SHELF_T - 0.2, z_sh + 0.1 + SHELF_T + 6.0))
shelf_sub = [box(XP1 - T - 7.0, XP1 - T + 1.0, CY - 8.0, CY + 8.0, z_sh - 1.0, z_sh + 3.0)]      # notch at the front: wires from the keel come up here
Msh = [Mt @ Matrix.Translation(Vector((SH_SCREW[0] - HOME.x, CY + sgn * SH_SCREW[1] - HOME.y, z_sh + 0.1 + SHELF_T - HOME.z)) * MM) for sgn in (-1, 1)]
for M in Msh:
    shelf_sub += L["screw_holes"](M, [(0, 0)], 0.0, SHELF_T, 0.0, -1, csk=False)
shelf = finish("battery_shelf", L["build_part"]("battery_shelf", [L["combine_cutters"](shelf_add)], shelf_sub, coll, mat_lid))
shelf_screws = L["pan_screw_dummy"]("shelf_screws_M2x6st", [(M, 0, 0, 0.0, -1) for M in Msh], coll)
elec.append(ebox("battery_3s", XR + T + 0.3, CY - ELEC["battery_3s"][1] / 2, z_sh + 0.1 + SHELF_T + 0.2, ELEC["battery_3s"], (0.15, 0.15, 0.15, 1)))
z_top = z_sh + 0.1 + SHELF_T + 0.2 + ELEC["battery_3s"][2] + 0.3          # top of the battery
# balance sensor: flat on four bosses on the keel floor (low, central, rigid), 4x M2x6 from above before the adapter goes in
elec.append(ebox("imu", IMU_XY[0] - ELEC["imu"][0] / 2, IMU_XY[1] - ELEC["imu"][1] / 2, ZB + T + IMU_BOSS, ELEC["imu"], (0.5, 0.1, 0.6, 1)))
imu_screws = L["pan_screw_dummy"]("imu_screws_M2x6st", [(Mt @ Matrix.Translation(Vector((IMU_XY[0] + sx * IMU_HOLES[0] - HOME.x, IMU_XY[1] + sy * IMU_HOLES[1] - HOME.y, ZB + T + IMU_BOSS + 1.6 - HOME.z)) * MM), 0, 0, 0.0, -1)
                                                          for sx in (-1, 1) for sy in (-1, 1)], coll)
for _n in ("bus_screw_M2x6st",):                                              # left over from the bus board
    _o = bpy.data.objects.get(_n)
    if _o:
        bpy.data.objects.remove(_o, do_unlink=True)
ZT = ZP_FRONT
for o in [frame, enclosure, enc_screws, lid, lid_screws, shelf, shelf_screws, uno_screws, imu_screws] + elec:
    mw = o.matrix_world.copy()
    o.parent = trunk
    o.matrix_world = mw
for n in ("hip_yaw_support", "body_neck_mount"):                             # absorbed into body_frame
    o = bpy.data.objects.get(n)
    if o:
        bpy.data.objects.remove(o, do_unlink=True)
old = bpy.data.objects.get(f"{R}:trunk_base.001")                            # old scaled shell: replaced
if old:
    old.hide_set(True)
    old.hide_render = True
print("body v2: " + " | ".join(report))
print(f"  keel inside {XK1 - XR - 2 * T:.1f} x {2 * HK - 2 * T:.1f} x {z_deck0 - ZB - T:.1f}; pocket inside {XP1 - XR - 2 * T:.1f} x {2 * HP - 2 * T:.1f} x {ZP_REAR - z_deck1:.1f}..{ZP_FRONT - z_deck1:.1f}; Uno Q on top at z {uno_z:.1f}..{uno_z + UNO['pcb'] + UNO['top']:.1f}, x {UX0:.1f}..{UX0 + UNO['len']:.1f}")
for line in L["DEBUG"]:
    print("  !", line)
