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

import bmesh
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
XR, XK1 = -53.0, -6.0                     # keel rear face (the Uno Q lines up with it), keel front face
EXT = args.get("pocket_ext", 0.0)         # battery pocket made this much longer toward the rear (user 2026-11-12: Pro-Range 850 mAh 3S,
XRP = XR - EXT                            #   57 x 31 mm): the keel, the adapter and the Uno Q stay; only the pocket, its shelf and the lid grow
BAT_GAP = args.get("battery_rear_gap", 0.0)   # room behind the battery for its leads (they leave its rear end)
VBAT = args.get("battery_vertical", False)  # user 2026-11-12: battery standing upright at the keel's front (weight low and near the hips), the servo
                                            # adapter standing across the keel behind it, balance sensor on the pocket's rear wall, regulator
                                            # (Mini560 5 V 5 A) and power switch under the lid's rear part. No battery shelf.
EXT_Z0 = args.get("ext_floor_z", 99.0)      # the extension's underside: high, out of the feet's way on a backward step (needs print supports)
HK, HP = args.get("keel_hw", 18.0), 22.0  # half-widths: keel, pocket (18.5 with the upright battery: the adapter stands across the keel)
ZB = 57.0                                 # keel underside
z_deck0, z_deck1 = lay(FyL, (0, 0, L["CAP_T"])).z, lay(FyL, (0, 0, 0)).z      # deck = the hip-yaw cap plate (94.2 .. 99.2)
ZP_REAR, XP_STEP = 113.0, -36.0
ZP_FRONT = args.get("pocket_top", 125.0)  # 2026-11-19: 128 for the 60 mm battery (user: raise the lid + Uno Q 3 mm, inside layout unchanged)
XP1 = -5.0                                # pocket front face (the hip-yaw servos' rear ears are at -3.6)
XD = 43.0                                 # centre strip's front edge (under the neck)
HS = 9.0                                  # centre strip half-width (the L-brackets rise beside it when the hips roll out)
UNO = dict(len=68.58, wid=53.34, pcb=1.6, top=10.0, pins=2.0)           # datasheet ABX00162-ABX00173, section 12 (underside parts under 2 mm)
ELEC = {"battery_3s": tuple(args.get("battery", (40.0, 30.0, 13.0))), "servo_adapter": (42.0, 18.5, 33.0), "ubec": (43.0, 8.0, 18.0), "imu": tuple(args.get("imu", (25.6, 22.7, 4.6)))}
TRAY = args.get("audio_tray", False)      # tray walls round the Uno Q + its cover + the power switch pocket (first made for the audio parts)
AUDIO = TRAY and args.get("audio", True)  # USB sound card + speaker + USB hub in the tray. User 2026-11-12: DROPPED (camera only, its mic) -> "audio": false,
                                          # "board_stand": 4.5 puts the board back down (12.5 mm lower: the neck leans back further)
BAT_Y0 = args.get("battery_y0", None)     # battery's -y face (default: centred). With a narrow block pack it sits to the right, a strip for the switch beside it
LIFT = 1.6                                # the battery shelf sits this much above the deck's top face (room for the adapter standing on its rails)
z_sh = z_deck1 + LIFT
# No electronics on the motor body: the custom bus board is gone (kit hubs only). What the body still offers the wiring:
PORT = dict(y0=CY + 8.6, y1=CY + HK - T - 0.4, z0=86.5, z1=93.0)          # cable port through the keel's front wall, beside the tongue, under the deck
CHEST = dict(x0=XD + 0.3, t=1.6, y0=CY - 4.0 if args.get("tof", False) else CY - 1.5, y1=CY + 22.0 if args.get("tof", False) else CY + 21.0, z0=None, z1=112.8 if args.get("tof", False) else 105.5)    # chest plate standing on the front of the centre strip: seat for ONE kit hub (hub_seats.py); from the deck's underside up, so the body still prints flat on that face
IMU_XY, IMU_HOLES, IMU_BOSS = (-29.5, CY), tuple(args.get("imu_holes", (10.16, 8.89))), 3.4   # 7Semi BNO086: 27.89 x 20.87, holes ESTIMATED 2.6 in from the corners               # balance sensor flat on the keel floor: centre, hole half-pitches, boss height
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
if EXT:
    tub += [box(XRP, XR + 0.3, p0, y0 + T, EXT_Z0, z_sh), box(XRP, XR + 0.3, y1 - T, p1, EXT_Z0, z_sh)]
# pocket walls: rear part low, front part high
tub += [box(XRP, XRP + T, p0, p1, EXT_Z0 if EXT else z_deck0 + 2.3, ZP_FRONT if (EXT or VBAT) else ZP_REAR),
        box(XR + 0.3, XP1, p0, p0 + T, z_deck0 + 2.3, ZP_REAR), box(XR + 0.3, XP1, p1 - T, p1, z_deck0 + 2.3, ZP_REAR),
        box(XP_STEP, XP1, p0, p0 + T, ZP_REAR - 0.3, ZP_FRONT), box(XP_STEP, XP1, p1 - T, p1, ZP_REAR - 0.3, ZP_FRONT),
        box(XP1 - T, XP1, p0 + 0.3, p1 - 0.3, z_deck0 + 2.3, ZP_FRONT)]              # front wall (behind the hip-yaw servos)
if EXT:                                                                   # the extension behind the keel: full-height walls (the lid rests on
    tub += [box(XRP + 0.3, XR + 0.6, p0, p0 + T, EXT_Z0, ZP_FRONT),         # them), all starting at EXT_Z0: nothing lower behind the keel, where
            box(XRP + 0.3, XR + 0.6, p1 - T, p1, EXT_Z0, ZP_FRONT)]         # the feet pass on a backward step
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
if args.get("chest_plate", False) or args.get("tof", False):                                         # off: the chest hub moved to the front of the neck
    tub.append(box(CHEST["x0"] - 0.6, CHEST["x0"] + CHEST["t"], CHEST["y0"], CHEST["y1"], z_deck0, CHEST["z1"]))        # chest plate (0.6 into the strip's front end)
body_solids = list(tub)
tub = []
# top plate posts: on the pocket's high walls (enclosure)
PL_DX = args.get("post_left_rear_dx", 2.6)   # 2026-11-20: the left-rear post (and its corbel) sat over the servo adapter's left edge: moved forward
POSTS = [(XP_STEP + T - 0.3, p0 + T - 0.3), (XP_STEP + T - 0.3 + PL_DX, p1 - T - 5.0 + 0.3), (XP1 - T - 5.0 + 0.3, p0 + T - 0.3), (XP1 - T - 5.0 + 0.3, p1 - T - 5.0 + 0.3)]
if EXT:
    POSTS += [(XRP + T - 0.3, p0 + T - 0.3), (XRP + T - 0.3, p1 - T - 5.0 + 0.3)]
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
# 2026-11-20: the 3 enclosure screws go in VERTICALLY from under the robot (were horizontal, driven inside the bay through access holes
# in its rear wall): through the tongue's gusset (solid from its sloped underside to the tongue's top, z 94.0) into the deck (94.2..99.2);
# M2x10 countersunk, head flush at JOIN_SEAT, 4 mm of thread in the deck, 1 mm of deck left over the tip.
JOIN_V = [(0.5, CY - 5.5), (0.5, CY), (0.5, CY + 5.5)]
JOIN_SEAT = z_deck0 + 0.2 - 10.0 + 3.6                                     # 87.8: tip 3.6 into the deck; the pilot ends 0.45 past it, 0.75 under the deck top
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
YAW_SADDLE = args.get("yaw_saddle_depth", 6.0)       # 8 -> 6 (2026-11-13): the left hip-yaw servo goes in under the neck's horn plate; with 8 mm
#                                                     walls it had only a 0.5 mm window to slide in. 6 still covers the tower (3.8) + 2.2 of body
cL, sL = L["cap_features"](FyL, include_plate=False, saddle=True, saddle_depth=YAW_SADDLE)
cR, sR = L["cap_features"](FyR, include_plate=False, saddle=True, saddle_depth=YAW_SADDLE)
servo_front = max(lay(FyL, (L["J_BODY_X"][0], 0, 0)).x, lay(FyL, (L["J_BODY_X"][1], 0, 0)).x)
z_holder_top = lay(FyL, (0, 0, -YAW_SADDLE)).z          # top of the hip-yaw servo's saddle walls
MX = Mt @ Matrix.Rotation(math.radians(90), 4, "Y")                       # a frame whose +Z points along layout +X (for the horizontal screws)


def xpt(y, z):
    """layout (y, z) of a point on a plane x = const -> (x, y) in MX's own frame (MX: x' = -z, y' = y, z' = x)"""
    return (-(z - HOME.z), y - HOME.y)


cuts = [box(servo_front - 12.0, servo_front + 1.5, y_out - (ARM[1] - ARM[0]) - 0.6, y_out + 2.0, z_holder_top + 2.0, ZP_FRONT + 30.0),   # horn plate, where it nears the left servo
        box(6.5, 23.0, CY - 4.5, CY + 4.5, z_deck0 - 1.0, z_deck1 + 1.0)]      # cable slot through the deck slot between the two servos (clear of the enclosure's tongue)
# above the left hip-yaw servo's front ear tip (2.5 beyond the body end, which the cut above stops at): the servo goes in from the rear,
# lifted 7-8 mm over its pads; the pedestal's rear face hung 0.5 over that ear tip and left a 1 mm window (2026-11-13)
_ex = max(lay(FyL, (x_, 0, 0)).x for x_ in L["J_EAR_X"])
_ey = lay(FyL, (0, 0, 0)).y
_ez = max(lay(FyL, (0, 0, z_)).z for z_ in L["J_EAR_Z"])
cuts.append(box(_ex - 2.0, _ex + 0.8, _ey - L["J_EAR_HW"] - 0.6, _ey + L["J_EAR_HW"] + 0.6, _ez + 3.0, _ez + 10.5))
# the pedestal's front top corner, cut at 45 deg: the cradle's edge passes there when the neck is fully forward
Mc = Mt @ Matrix.Translation(Vector((nx + 3.1 - HOME.x, 0.0, z_web - HOME.z)) * MM) @ Matrix.Rotation(math.radians(45), 4, "Y")
cuts.append(L["add_box"](L["new_bm"](), -12.0, 12.0, y_in - 1.0 - HOME.y, y_out - (ARM[1] - ARM[0]) - 0.35 - HOME.y, 0.0, 14.0, Mc))
for jx, jy in JOIN_V:                                                      # pilots up into the deck for the enclosure's screws (2026-11-20: driven
    cuts.append(L["add_cyl"](L["new_bm"](), jx - HOME.x, jy - HOME.y, z_deck0 - 0.3 - HOME.z, JOIN_SEAT + L["CSK_L"] + 0.45 - HOME.z,   # from below)
                             L["PILOT_D"], 14, Mt))
# 2026-11-15: round the deck's lower OUTER edge (both sides), front part (x -4.9..11.2, where the outer face reaches the bottom): in
# walking (leg back ~46 deg, toed in, rolled out) the hip-roll leads and the body-to-thigh extensions run from the yaw connector under
# the deck up to the thigh and wrap round this edge, every step (cable_pinch_check.py, 2.3 mm). r 1.5: ONE prism along x whose y-z
# profile is the corner minus the quarter circle (a fan of separate wedges broke the exact boolean).
EDGE_R = args.get("cable_edge_r", 1.5)


def x_prism(poly_yz, x0, x1):
    """closed prism along layout x through the y-z polygon (layout mm)"""
    import bmesh
    bm = bmesh.new()
    W = lambda x, y, z: Mt @ ((Vector((x, y, z)) - HOME) * MM)
    A = [bm.verts.new(W(x0, y, z)) for y, z in poly_yz]
    B = [bm.verts.new(W(x1, y, z)) for y, z in poly_yz]
    n = len(poly_yz)
    for i in range(n):
        bm.faces.new((A[i], A[(i + 1) % n], B[(i + 1) % n], B[i]))
    bm.faces.new(A[::-1])
    bm.faces.new(B)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


if EDGE_R:
    for y_e, s_ in ((cap_yL[1], -1.0), (cap_yR[0], +1.0)):                   # s_: from the outer face into the deck
        yc, zc = y_e + s_ * EDGE_R, z_deck0 + EDGE_R
        arc = [(yc - s_ * EDGE_R * math.cos(t), zc - EDGE_R * math.sin(t)) for t in (math.pi / 2 * k / 12 for k in range(13))]
        poly = [(y_e - s_ * 0.4, z_deck0 - 0.4), (y_e - s_ * 0.4, zc)] + arc + [(yc, z_deck0 - 0.4)]   # corner pushed out of the part
        cuts.append(x_prism(poly, -6.0, 11.0))
frame = finish("body_frame", L["build_part"]("body_frame", [L["combine_cutters"](body_solids + [pedestal, ped_gusset, ped_foot, web] + cL + cR)] + a_add,   # all simple solids unioned first
                                             a_sub + [L["combine_cutters"](sL), L["combine_cutters"](sR)] + cuts, coll, mat_frame, a_rib))   # each cap's cuts as ONE cutter
enc_cuts = []
for px, py in POSTS:
    enc_cuts.append(L["add_cyl"](L["new_bm"](), px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZP_FRONT - 5.5 - HOME.z, ZP_FRONT + 1.0 - HOME.z, L["PILOT_D"], 14, Mt))
enc_cuts.append(box(XRP - 0.5, XRP + T + 0.5, CY - 5.3, CY + 5.3, z_sh + 0.0, z_sh + 2.0))            # slot for the battery shelf's tab
for sgn in (-1, 1):                                                        # pilots for the shelf's two screws, down through the ledge and its boss
    enc_cuts.append(L["add_cyl"](L["new_bm"](), XP1 - T - 4.2 - HOME.x, CY + sgn * (HP - T - 2.0) - HOME.y, z_sh - 4.7 - HOME.z, z_sh + 0.5 - HOME.z, L["PILOT_D"], 14, Mt))
enc_cuts.append(box(XP1 - T - 0.5, XP1 + 0.5, CY - 8.5, CY + 8.5, ZP_FRONT - 5.5, ZP_FRONT + 1.0))      # cable notch in the pocket's front wall: the servo adapter's cable to the bus board
if not VBAT:                                                               # (upright bay: no cable uses it - the left-thigh cable goes over the battery)
    enc_cuts.append(box(XK1 - T - 0.5, XK1 + 0.5, PORT["y0"], PORT["y1"], PORT["z0"], PORT["z1"]))   # cable port: the adapter's two cables leave the keel here
for jx, jy in JOIN_V:                                                      # 2026-11-20 (user: screws from the outside): up from UNDER the robot
    # through the tongue's gusset; head flush in a 90 deg countersink. Three separate cutters with clearly different sizes (nearly
    # coincident faces left the enclosure non-manifold): relief from below Ø5.2 to JOIN_SEAT - 0.2, the cone from inside the relief, the
    # clearance hole from inside the cone
    enc_cuts.append(L["add_cyl"](L["new_bm"](), jx - HOME.x, jy - HOME.y, JOIN_SEAT - 12.0 - HOME.z, JOIN_SEAT - 0.2 - HOME.z, 5.2, 32, Mt))
    enc_cuts.append(L["_csk_cone"](Mt, jx - HOME.x, jy - HOME.y, JOIN_SEAT - 0.4 - HOME.z, L["CSK_HEAD_D"] + 0.1 + 0.8,
                                   JOIN_SEAT + (L["CSK_HEAD_D"] + 0.1 - 2.2) / 2 - HOME.z, 2.2))
    enc_cuts.append(L["add_cyl"](L["new_bm"](), jx - HOME.x, jy - HOME.y, JOIN_SEAT + 0.4 - HOME.z, z_deck0 + 0.5 - HOME.z, L["CLEAR_D"], 16, Mt))
# keel fittings: sensor bosses on the floor, two rails across the ends (adapter + regulator stand on them, above the sensor),
# and a fence post on each rail between the adapter and the regulator
zk0 = ZB + T
if VBAT:
    _B = ELEC["battery_3s"]                                                 # (length, width, thickness): standing, x = thickness, z = length
    VB = dict(x1=XK1 - T - 0.3, y0=CY - _B[1] / 2, z0=zk0 + 0.3)
    VB["x0"] = VB["x1"] - _B[2]
    yi = HK - T                                                              # keel inner half-width
    for sgn in (-1, 1):
        for xa in (VB["x0"] + 3.0, VB["x1"] - 5.0):                          # side ribs: the battery centred, 0.3 to each rib
            enc.append(box(xa, xa + 2.0, *sorted((CY + sgn * (_B[1] / 2 + 0.3), CY + sgn * (yi + 0.3))), zk0 - 0.3, zk0 + 34.0))
        enc.append(box(VB["x0"] - 1.6, VB["x0"] - 0.3, *sorted((CY + sgn * 9.0, CY + sgn * 13.0)), zk0 - 0.3, zk0 + 8.0))   # rear stops, low
    # servo adapter standing across the keel (exact board: parts/adapter_mesh, from Waveshare's STEP): solder side toward the rear wall with
    # 4 mm for the soldered power + UART wires, servo sockets toward the battery; long edges in slots between two ribs on each side wall
    ADV = dict(back=XR + T + 4.0, z0=zk0 + 0.3)
    ADV["pcb_x0"], ADV["pcb_x1"] = ADV["back"] + 2.0, ADV["back"] + 3.6
    for sgn in (-1, 1):
        ye = sorted((CY + sgn * (16.5 - 0.8), CY + sgn * (yi + 0.3)))
        # the power edge (+y: DC jack, screw terminal, USB-C stand 0.8 proud, board x 5.2..36.9): its ribs only at the two ends
        for za, zb in ((zk0 - 0.3, ADV["z0"] + 4.7), (ADV["z0"] + 38.0, zk0 + 42.0)):     # (the socket edge too: sockets + header pins reach it)
            enc.append(box(XR + T - 0.3, ADV["pcb_x0"] - 0.15, *ye, za, zb))       # behind the board's edge
            enc.append(box(ADV["pcb_x1"] + 0.15, ADV["pcb_x1"] + 1.4, *ye, za, zb))  # in front of it
    enc_cuts.append(box(ADV["back"] - 0.3, ADV["pcb_x1"] + 12.0, CY + yi - 0.5, CY + yi + 1.0, ADV["z0"] + 4.7, ADV["z0"] + 37.4))   # relief in the left wall for them
    IMU_V = dict(zc=(zk0 + 42.8 + ZP_FRONT - 0.4) / 2, x_face=XR + T + 3.0, yc=CY + args.get("imu_dy", 0.0))   # balance sensor upright on the rear wall, above the adapter
    for sy in (-1, 1):
        for sz in (-1, 1):
            yy, zz = IMU_V["yc"] + sy * IMU_HOLES[0], IMU_V["zc"] + sz * IMU_HOLES[1]
            enc.append(L["add_cyl"](L["new_bm"](), *xpt(yy, zz), XR + T - 0.3 - HOME.x, IMU_V["x_face"] - HOME.x, 4.6, 20, MX))
            # 2026-11-20 (user: nothing screwed from inside the bay): no screws - a peg on each boss goes through the board's mounting
            # hole (the only place a board is sure to be free of parts); the IMU goes straight down, back onto the pegs, a drop of hot
            # glue on each peg tip holds it (the switch plate is also 0.4 in front of its left edge)
            enc.append(L["add_cyl"](L["new_bm"](), *xpt(yy, zz), IMU_V["x_face"] - 0.3 - HOME.x, IMU_V["x_face"] + 1.6 + 0.8 - HOME.x, 1.9, 16, MX))
    # camera USB cradle (2026-11-19, user: rigid right-angle USB-C -> USB-A adapter, mouth down): two ribs on the rear wall's outer face either
    # side of the adapter + the camera's USB-A plug, 1.2 mm proud of them, with a zip-tie slot through each: the plug rests against the wall
    # between them, so a knock goes into the bay, not the Uno Q's USB-C socket (parts/bay_parts.py draws the adapter and plug)
    if args.get("usb_cradle", True):
        _uy, _uw, _ud, _uz0 = args.get("usb_y", -88.9), args.get("usb_cradle_w", 17.0), args.get("usb_cradle_d", 12.3), args.get("usb_cradle_z0", 100.0)
        for sgn in (-1, 1):
            ya_, yb_ = sorted((_uy + sgn * (_uw / 2 + 0.5), _uy + sgn * (_uw / 2 + 0.5 + T)))
            enc.append(box(XR - _ud, XR + 0.3, ya_, yb_, _uz0, ZP_FRONT - 0.5))
            enc_cuts.append(box(XR - _ud + 3.0, XR - 2.0, ya_ - 0.5, yb_ + 0.5, _uz0 + 8.0, _uz0 + 11.0))        # zip-tie slot (3 x 4)
    # battery plug cradle (2026-11-19, user: XT30 outside, rear right): ribs on the rear face either side of the XT30 pair (y -121..-111) and
    # the balance plug beside it (y -110..-104.25), 1.4 mm proud of them, zip-tie slots; the robot's pigtail loops up behind the XT30 (room
    # for it: 4.2 mm behind the pair inside the ribs). Open at the bottom (charger lead into the balance plug) and the top (wires)
    if args.get("plug_cradle", True):
        _cd = 0.6 + 5.2 + 4.2 + 1.4                                           # XT30 + the pigtail behind it + 1.4 proud
        # 2026-11-19: XT30 in the middle (y -110..-100, z 90..116: the right foot reaches y < -111 up to z 109); balance plug at the rear
        # right ABOVE the bay's top (y -121..-115.25, z 128.5..140.9, behind the Uno Q cover), held by a fin on its left (z 124..141, on the
        # rear wall's top 4 mm; the cables round the corner pass under it) + a zip tie. The XT30's right rib from z 109 (IMU pilot below).
        # 2026-11-20: the balance plug's fin ran 0.1 mm into the lid and the Uno Q's board and 1.2 mm through the cover's rear wall (all
        # three overhang the rear face; the fit audit had no such pairs). Now it keeps 0.1 / 0.3 / 0.3 from them, and its joint to the
        # wall reaches down to z 122.1 (just over the battery leads, which lie flat behind the wall at z 118.4..121.6): 1.6 x 5.3 mm.
        FIN = (-114.85, -113.25)
        enc.append(box(XR - 1.3, XR + 0.3, FIN[0], FIN[1], 122.1, 124.35))                                        # root, under the lid's level
        enc.append(box(XR - _cd, XR + 0.1, FIN[0], FIN[1], 124.3, 127.4))                                         # on the wall's outer skin, 0.1 from the lid
        enc.append(box(XR - _cd, XR - 0.1, FIN[0], FIN[1], 127.3, 129.8))                                         # beside the lid / board
        enc.append(box(XR - _cd, XR - 1.6, FIN[0], FIN[1], 129.7, 141.0))                                         # behind the cover
        enc_cuts.append(box(XR - _cd + 3.0, XR - 2.0, FIN[0] - 0.5, FIN[1] + 0.5, 134.0, 137.0))                 # zip-tie slot
        ya_, yb_, z0_, z1_, zs_ = -111.6, -110.5, 109.0, 115.0, 110.5                                            # the XT30's right rib (from z 109:
        enc.append(box(XR - _cd, XR + 0.3, ya_, yb_, z0_, z1_))                                                 #  the IMU pilot below, z 106.7)
        enc_cuts.append(box(XR - _cd + 3.0, XR - 2.0, ya_ - 0.5, yb_ + 0.5, zs_, zs_ + 3.0))                    # zip-tie slot
    # power switch: mini toggle (MTS, 6 A; body 7.8 x 13.2 x 9.5 + pins, 6 mm bushing), body inside above the adapter, lever out through
    # the left side wall (7.4 mm proud). 2026-11-20: its mounting block was part of the enclosure and hung 1.1 mm over the servo adapter -
    # the adapter could not slide down into its slots past it (bay path check). Now a separate SWITCH PLATE (the same block: hex pocket for
    # the M6 nut, Ø6.3 bushing hole in its 1.6 mm floor): on the bench the nut drops into the pocket and the switch screws in by turning its
    # body, wires soldered; in the bay the plate goes down 10 mm in from the wall (a lid post's corbel is above it at the wall), then
    # outward: the lever through a 4.4 x 7 hole in the wall; 2x M2x10 countersunk from outside, heads flush (the enclosure screw).
    TG = dict(xc=args.get("switch_x", -35.2), zc=108.0, y_plate=CY + HP - 11.6)  # plate floor (the switch's mounting face) 11.6 in from the outside
    SWP = dict(x0=TG["xc"] - 6.8, x1=TG["xc"] + 6.8, y0=TG["y_plate"], y1=CY + HP - T - 0.1, z0=101.45, z1=114.8)  # 0.4 from the IMU, 0.5 from the
    #   battery, 0.55 over the adapter, under the post corbel; >= 1.0 mm of plastic round the screw pilots (screw audit)
    SWP_SCREWS = [(TG["xc"] - 4.85, TG["zc"] - 4.65), (TG["xc"] + 4.85, TG["zc"] - 4.65)]   # bottom corners: the wall behind x -36.6 is only 113 high
    MYs = Mt @ Matrix.Rotation(math.radians(-90), 4, "X")                     # +Z along layout +Y
    MYn = Mt @ Matrix.Rotation(math.radians(90), 4, "X")                      # +Z along layout -Y (the plate screws point inward)

    def cyl_y(x, z, y0, y1, d, segs=24):
        """cylinder along layout Y (layout coordinates)"""
        return L["add_cyl"](L["new_bm"](), x - HOME.x, -(z - HOME.z), y0 - HOME.y, y1 - HOME.y, d, segs, MYs)

    enc_cuts.append(box(TG["xc"] - 1.9, TG["xc"] + 1.9, CY + HP - T - 0.5, CY + HP + 0.5, TG["zc"] - 3.5, TG["zc"] + 3.5))   # lever hole (lever Ø3 + its throw)
    _fz = CY + HP - HOME.y                                                     # the wall's outer face (frame z of MYs)
    for sx_, sz_ in SWP_SCREWS:                                                # countersunk clearance holes, heads flush (Ø3.9 at the face)
        enc_cuts.append(L["combine_cutters"]([cyl_y(sx_, sz_, CY + HP - T - 0.6, CY + HP + 0.5, L["CLEAR_D"], 16),
                                              L["_csk_cone"](MYs, sx_ - HOME.x, -(sz_ - HOME.z), _fz - 0.85, 2.2, _fz + 0.3, 4.5)]))
    # 2026-11-20: notches in the pocket's floor ledge over the servo adapter's edges (its left-edge parts and a corner on the right sat
    # under the ledge: it could not slide down into its slots)
    enc_cuts.append(box(-43.85, -32.7, -84.45, -82.6, 95.9, 101.2))
    enc_cuts.append(box(-43.85, -42.2, -116.45, -115.6, 95.9, 101.2))
for sx in (-1, 1) if not VBAT else ():
    for sy in (-1, 1):
        ix, iy = IMU_XY[0] + sx * IMU_HOLES[0], IMU_XY[1] + sy * IMU_HOLES[1]
        enc.append(L["add_cyl"](L["new_bm"](), ix - HOME.x, iy - HOME.y, zk0 - 0.3 - HOME.z, zk0 + IMU_BOSS - HOME.z, 4.6, 20, Mt))
        enc_cuts.append(L["add_cyl"](L["new_bm"](), ix - HOME.x, iy - HOME.y, zk0 + IMU_BOSS - 4.6 - HOME.z, zk0 + IMU_BOSS + 0.5 - HOME.z, L["PILOT_D"], 14, Mt))
y_ad = CY - HK + T + 0.4 + ELEC["servo_adapter"][1]                          # the adapter's inner face
y_ub = CY + HK - T - 0.4 - ELEC["ubec"][1]                                   # the regulator's inner face
for xa, xb in ((XR + T - 0.3, XR + T + 5.4), (XK1 - T - 5.4, XK1 - T + 0.3)) if not VBAT else ():
    enc.append(box(xa, xb, CY - HK + T - 0.3, CY + HK - T + 0.3, zk0 - 0.3, zk0 + RAIL_H))
    enc.append(box(xa, xb, y_ad + 0.4, y_ub - 0.4, zk0 + RAIL_H - 0.3, zk0 + RAIL_H + 16.0))
# servo adapter (Waveshare Bus Servo Adapter (A), 42 x 33 x 1.6 board, parts on one side - product photo): stands on the rails on a
# long edge, parts toward the keel's middle, 2.5 mm off the side wall (solder side), its two short ends in guide slots on the keel's
# end walls (1.5 mm of board in each); the battery shelf above keeps it down. No screws.
AD = dict(len=42.0, h=33.0, pcb=1.6, y0=CY - HK + T + 2.5)
AD["x0"] = (XR + T + XK1 - T) / 2 - AD["len"] / 2
for xa, xb in ((XR + T - 0.3, AD["x0"] + 1.5), (AD["x0"] + AD["len"] - 1.5, XK1 - T + 0.3)) if not VBAT else ():
    enc.append(box(xa, xb, CY - HK + T - 0.3, AD["y0"] - 0.15, zk0 + RAIL_H - 0.3, zk0 + RAIL_H + 30.0))                       # behind the board (fills to the wall)
    enc.append(box(xa, xb, AD["y0"] + AD["pcb"] + 0.15, AD["y0"] + AD["pcb"] + 1.4, zk0 + RAIL_H - 0.3, zk0 + RAIL_H + 30.0))  # in front of it
# keel's lower outer edges: 45 deg chamfer where the knees pass at inward hip yaw + roll (2026-11-13, user: the full-horn knee caps
# came within 0.50 mm of these edges at the walking corner, yaw -6.3 / roll -10.9; with 2.5 mm: 0.97, and roll -12 becomes safe).
# A fillet inside the corner keeps the wall >= 1.3 mm there (the battery stands 1.4 from the wall, 0.3 above the floor: 0.35 clear).
KC = args.get("keel_chamfer", 2.5)
KC_X = tuple(args.get("keel_chamfer_x", (-30.0, -5.0)))                  # along the knees' path only (the servo adapter's slots are behind x -32.8)
KF = args.get("keel_fillet", 1.2)
if KC > 0:
    for sgn in (-1, 1):
        yo, yi = CY + sgn * HK, CY + sgn * (HK - T)                        # outer / inner face of the side wall
        enc_cuts.append(hull([(x, y, z) for x in KC_X for y, z in ((yo + sgn * 1.0, ZB - 1.0), (yo - sgn * (KC + 1.0), ZB - 1.0), (yo + sgn * 1.0, ZB + KC + 1.0))]))
        if KF > 0:
            enc.append(hull([(x, y, z) for x in (KC_X[0], XK1 - 0.3) for y, z in ((yi + sgn * 0.3, ZB + T - 0.3), (yi - sgn * (KF + 0.3), ZB + T - 0.3), (yi + sgn * 0.3, ZB + T + KF + 0.3))]))
mat_enc = L["material"]("electronics_enclosure", (0.35, 0.55, 0.75, 1))
switch_plate = sw_screws = None
enclosure = L["build_part"]("electronics_enclosure", [L["combine_cutters"](enc)], enc_cuts, coll, mat_enc)
_bm = bmesh.new()                                                          # weld coincident vertices the exact boolean can leave (2026-11-20: a
_bm.from_mesh(enclosure.data)                                              # zero-length edge at a front countersink made it non-manifold)
bmesh.ops.remove_doubles(_bm, verts=_bm.verts, dist=1e-7)
_bm.to_mesh(enclosure.data)
_bm.free()
enclosure = finish("electronics_enclosure", enclosure)
if VBAT:                                                                    # the switch plate (2026-11-20) + its 2 countersunk screws
    sw_cuts = [cyl_y(TG["xc"], TG["zc"], TG["y_plate"] + 1.6, SWP["y1"] + 0.5, 8.3 / math.cos(math.pi / 6), 6),   # nut pocket (hex AF 8.3), open to the wall
               cyl_y(TG["xc"], TG["zc"], TG["y_plate"] - 0.5, TG["y_plate"] + 2.1, 6.3)]                         # bushing hole in the 1.6 floor
    sw_cuts += [cyl_y(sx_, sz_, SWP["y1"] - 9.2, SWP["y1"] + 0.5, L["PILOT_D"], 14) for sx_, sz_ in SWP_SCREWS]    # pilots: 8.3 mm of thread
    switch_plate = finish("switch_plate", L["build_part"]("switch_plate", [box(SWP["x0"], SWP["x1"], SWP["y0"], SWP["y1"], SWP["z0"], SWP["z1"])],
                                                          sw_cuts, coll, L["material"]("switch_plate", (0.95, 0.55, 0.15, 1))))
    _ss = L["new_bm"]()
    _f, _hh = -(CY + HP - HOME.y) + 0.05, (L["CSK_HEAD_D"] - 2.0) / 2          # frame of MYn: z = -(y - HOME.y); head top flush with the wall's face
    for sx_, sz_ in SWP_SCREWS:
        L["_csk_cone"](MYn, sx_ - HOME.x, sz_ - HOME.z, _f + 0.02, L["CSK_HEAD_D"] - 0.06, _f + 0.02 + _hh, 2.0 - 0.06, bm=_ss)
        L["add_cyl"](_ss, sx_ - HOME.x, sz_ - HOME.z, _f + _hh + 0.06, _f + L["CSK_L"], 2.0, 16, MYn)
    sw_screws = L["to_object"]("switch_plate_screws_M2x10csk", _ss, coll, L["material"]("hw_steel", (0.6, 0.62, 0.66, 1)))
if VBAT:                                                                    # M2x10 countersunk machine screws (Robu EasyMech, head Ø3.8), head flush with the keel wall's inner face
    _cs = L["new_bm"]()
    f, hh = JOIN_SEAT - HOME.z, (L["CSK_HEAD_D"] - 2.0) / 2               # vertical, head flush with the seat (from below); 90 deg head, 0.9 high
    for jx, jy in JOIN_V:
        L["_csk_cone"](Mt, jx - HOME.x, jy - HOME.y, f + 0.02, L["CSK_HEAD_D"] - 0.06, f + 0.02 + hh, 2.0 - 0.06, bm=_cs)
        L["add_cyl"](_cs, jx - HOME.x, jy - HOME.y, f + hh + 0.06, f + L["CSK_L"], 2.0, 16, Mt)
    enc_screws = L["to_object"]("enclosure_screws_M2x6st", _cs, coll, L["material"]("hw_steel", (0.6, 0.62, 0.66, 1)))   # (object name kept: explode views use it)
else:
    enc_screws = L["pan_screw_dummy"]("enclosure_screws_M2x6st", [(MX, *xpt(CY + y, z_join), XK1 - T - HOME.x, +1) for y in JOIN_Y], coll)   # head on the keel wall's inner face

# ------------------------------------------------------------------ top plate: closes the pocket and carries the Uno Q above the body
LID_T = 2.0
ZL = ZP_FRONT + 0.1
UX0 = XR + 0.2                                                             # Uno Q: from the enclosure's rear face forward (x -52.8 .. 16.1)
UY0 = CY - UNO["wid"] / 2
UNO_HOLES = [(13.97, 2.54), (15.24, 50.80), (66.04, 7.62), (66.04, 35.56)]  # mounting holes (R1.6), from the corner under the USB-C end - datasheet
STAND = args.get("board_stand", 17.0 if TRAY else 4.5)                                              # posts under the board (4.5: its underside parts clear the plate's screw heads; 17: the audio tray)
uno_z = ZL + LID_T + STAND
lid_add = [box(XRP + 0.3, UX0 + UNO["len"] - 0.5, CY - UNO["wid"] / 2 - 0.5, CY + UNO["wid"] / 2 + 0.5, ZL, ZL + LID_T)]      # as long as the board: also covers the pocket's rear part
for hx, hy in UNO_HOLES:
    lid_add.append(L["add_cyl"](L["new_bm"](), UX0 + hx - HOME.x, UY0 + hy - HOME.y, ZL + LID_T - 0.3 - HOME.z, uno_z - HOME.z, 6.4, 28, Mt))
BCY = CY if BAT_Y0 is None else BAT_Y0 + ELEC["battery_3s"][1] / 2          # battery centre line
BAT_X0 = XRP + T + 0.3 + BAT_GAP                                           # battery's rear face
z_bat0 = z_sh + 0.1 + 1.6 + 0.2                                             # battery underside (on the shelf)
lid_sub = []
if not VBAT and (EXT or z_bat0 + ELEC["battery_3s"][2] > ZL - 0.3):                        # (extended pocket: always - the 850 pack's height is not listed; up to ~24 mm fits)                               # a tall block pack reaches into the plate: window over it
    lid_sub.append(box(BAT_X0 + (1.6 if EXT else -0.5), BAT_X0 + ELEC["battery_3s"][0] + 0.6, BCY - ELEC["battery_3s"][1] / 2 - 0.6, BCY + ELEC["battery_3s"][1] / 2 + 0.6, ZL - 1.0, ZL + LID_T + 0.2))
if TRAY:                                                                    # walls round the board's outline, from the plate up to 0.5 under the board
    ux1, uy0, uy1, zw = UX0 + UNO["len"], CY - UNO["wid"] / 2, CY + UNO["wid"] / 2, uno_z - 0.5
    lid_add += [box(UX0, ux1, uy0, uy0 + T, ZL + LID_T - 0.3, zw), box(UX0, ux1, uy1 - T, uy1, ZL + LID_T - 0.3, zw),
                box(UX0, UX0 + T, uy0 + 0.3, uy1 - 0.3, ZL + LID_T - 0.3, zw), box(ux1 - T, ux1, uy0 + 0.3, uy1 - 0.3, ZL + LID_T - 0.3, zw)]
    if AUDIO:
        lid_sub.append(box(UX0 - 1.0, UX0 + T + 1.0, CY + 4.0, CY + 18.0, ZL + LID_T + 5.0, zw + 1.0))            # rear wall: the USB-C cable comes down from the board's socket here
    lid_sub.append(box(ux1 - T - 1.0, ux1 + 1.0, CY - 20.0, CY - 8.0, ZL + LID_T + 2.0, ZL + LID_T + 9.0))    # front wall: cables on toward the neck
    for i in range(6 if AUDIO else 0):                                      # sound holes in the left wall, in front of the speaker
        for k in range(3):
            lid_sub.append(box(UX0 + 22.5 + 4.0 * i, UX0 + 24.5 + 4.0 * i, uy1 - T - 0.5, uy1 + 0.5, ZL + LID_T + 3.0 + 4.0 * k, ZL + LID_T + 5.0 + 4.0 * k))
    for i in range(5 if AUDIO else 0):                                      # slots in the right wall along the sound card (its microphones)
        lid_sub.append(box(UX0 + 12.0 + 9.0 * i, UX0 + 18.0 + 9.0 * i, uy0 - 0.5, uy0 + T + 0.5, ZL + LID_T + 8.0, ZL + LID_T + 9.6))
if TRAY:
    zf = ZL + LID_T                                                         # tray floor
    AUD = dict(x0=UX0 + 8.0, y0=CY - UNO["wid"] / 2 + 9.6, len=53.0, wid=23.5, h=14.6)      # USB sound card, flat
    SPK = dict(x0=UX0 + 21.5, len=25.0, t=4.0, h=15.0)                                        # speaker on edge against the left wall
    SPK["y0"] = uy1 - T - 0.3 - SPK["t"]
    g = 0.25
    lid_add += [] if not AUDIO else [box(AUD["x0"] + 2.0, AUD["x0"] + AUD["len"] - 2.0, AUD["y0"] - g - 1.2, AUD["y0"] - g, zf - 0.3, zf + 3.0),                      # sound card: rail along its right edge
                box(UX0 + 44.0, AUD["x0"] + AUD["len"] - 2.0, AUD["y0"] + AUD["wid"] + g, AUD["y0"] + AUD["wid"] + g + 1.2, zf - 0.3, zf + 3.0),   # rail along its left edge (front part: the battery window is behind)
                box(AUD["x0"] - g - 1.2, AUD["x0"] - g, AUD["y0"] - g - 1.2, AUD["y0"] + 1.6, zf - 0.3, zf + 3.0),                                # rear stop (beside the window)
                box(AUD["x0"] + AUD["len"] + g, AUD["x0"] + AUD["len"] + g + 1.2, AUD["y0"] + 2.0, AUD["y0"] + AUD["wid"] - 2.0, zf - 0.3, zf + 3.0),   # front stop
                box(SPK["x0"] + 1.0, SPK["x0"] + SPK["len"] - 1.0, SPK["y0"] - g - 1.2, SPK["y0"] - g, zf - 0.3, zf + 4.0),                      # speaker: rail on its inner side
                box(SPK["x0"] - g - 1.2, SPK["x0"] - g, SPK["y0"] - g - 1.2, uy1 - T + 0.3, zf - 0.3, zf + 4.0),
                box(SPK["x0"] + SPK["len"] + g, SPK["x0"] + SPK["len"] + g + 1.2, SPK["y0"] - g - 1.2, uy1 - T + 0.3, zf - 0.3, zf + 4.0)]
    # two pilots in the wall + post for the Uno cover's screws (2.2 beside the post's own bore, 5 above the tray floor - or halfway up
    # the low walls when the board sits low)
    _cz = min(5.0, (STAND - 0.5) / 2)
    CVO = 3.1                                                               # 2.2 -> 3.1 (2026-11-13): 1.0 of wall to the post's M3 bore (was 0.15)
    CVS = dict(a=(UX0 + UNO_HOLES[0][0] - CVO, zf + _cz), b=(UX0 + UNO_HOLES[1][0] - CVO, zf + _cz))   # both on the REAR side of their posts: in front, the
    #                                                                         right head sat half over the skirt's opening and the left boss met the board's
    #                                                                         underside parts. b: left side, mirror of a (2026-11-13: it
    #                                                                         came in from the front, where the neck plate left a driver 6.5 mm)
    MYf = Mt @ Matrix.Rotation(math.radians(-90), 4, "X")                   # a frame whose +Z points along layout +Y: (x, y, z) -> frame (x, -z, y)
    lid_add.append(L["add_cyl"](L["new_bm"](), CVS["a"][0] - HOME.x, -(CVS["a"][1] - HOME.z), uy0 - 0.3 - HOME.y, uy0 + 6.8 - HOME.y, 3.7, 20, MYf))   # boss round
    lid_add.append(L["add_cyl"](L["new_bm"](), CVS["b"][0] - HOME.x, -(CVS["b"][1] - HOME.z), uy1 - 6.8 - HOME.y, uy1 + 0.3 - HOME.y, 3.7, 20, MYf))   # each pilot (1 mm wall)
    lid_add += [box(CVS["a"][0] - 1.85, CVS["a"][0] + 1.85, uy0 - 0.3, uy0 + 6.8, zf - 0.3, CVS["a"][1]),                 # ... standing on the tray floor
                box(CVS["b"][0] - 1.85, CVS["b"][0] + 1.85, uy1 - 6.8, uy1 + 0.3, zf - 0.3, CVS["b"][1])]
    lid_sub.append(L["add_cyl"](L["new_bm"](), CVS["a"][0] - HOME.x, -(CVS["a"][1] - HOME.z), uy0 - 0.5 - HOME.y, uy0 + 5.8 - HOME.y, L["PILOT_D"], 14, MYf))
    lid_sub.append(L["add_cyl"](L["new_bm"](), CVS["b"][0] - HOME.x, -(CVS["b"][1] - HOME.z), uy1 - 5.8 - HOME.y, uy1 + 0.5 - HOME.y, L["PILOT_D"], 14, MYf))
    for xa, xb, ya, yb_ in ((UX0 - 6.0, ux1 + 6.0, uy0 - 6.0, uy0 - 0.02), (UX0 - 6.0, ux1 + 6.0, uy1 + 0.02, uy1 + 6.0), (ux1 + 0.02, ux1 + 6.0, uy0 - 6.1, uy1 + 6.1)):
        lid_sub.append(box(xa, xb, ya, yb_, zf + 0.02, uno_z + 1.0))        # the round posts trimmed flush with the tray walls (the cover's skirt slides over)
for px, py in POSTS:
    M = Mt @ Matrix.Translation(Vector((px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZL + LID_T - HOME.z)) * MM)
    lid_sub += L["screw_holes"](M, [(0, 0)], 0.0, LID_T, 0.0, -1, csk=False, cbore=0.7, cbore_d=4.4)   # head 0.7 down: room under the Uno Q for an M2 socket head
    if AUDIO:                                                               # room round the screw's head where the tray's rails pass
        lid_sub.append(L["add_cyl"](L["new_bm"](), px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZL + LID_T - 0.02 - HOME.z, ZL + LID_T + 4.5 - HOME.z, 4.4, 24, Mt))
UNO_M2 = (15.24, 50.80)        # this hole is 2.05 mm from the JDIGITAL header's housing (datasheet sec. 12): any M3 head (dk 5.5) overlaps it
                               # by 0.7 mm -> one M2 x 6 socket (row 19, dk 3.7) there instead (user 2026-11-15; independent review 2026-11-14)
for hx, hy in UNO_HOLES:                                                   # M3 thread-forming pilots down the posts and through the plate (M2 at UNO_M2)
    lid_sub.append(L["add_cyl"](L["new_bm"](), UX0 + hx - HOME.x, UY0 + hy - HOME.y, ZL - 1.0 - HOME.z, uno_z + 1.0 - HOME.z,
                                L["PILOT_D"] if (hx, hy) == UNO_M2 else 2.5, 16, Mt))
lid_sub.append(box((XR - 9.0) if EXT else XR - 1.0, XR + 9.0, CY - 12.0, CY + 12.0, ZL - 1.0, ZL + LID_T + 1.0))      # wire notch at the rear: battery / IMU / UART wires come up here to the board
RG = dict(x0=-40.5, x1=-22.5, y0=CY - 15.0, y1=CY + 15.0)                 # Mini560 30 x 18 (maker's drawing, user 2026-11-19; was 29), long side along y
# 2026-11-18 (independent review: "regulator 0.2 mm from PLA walls ... in a closed bay"): centred between the side walls (was 1 mm
# toward -y: 0.1 mm from the wall there, 2.1 on the other side -> 1.1 / 1.1), and a WINDOW in the lid plate over it: its top (the
# hot side: inductor, IC) faces the air under the Uno Q instead of PLA 0.12 mm away. The board still hangs between the two rails.
REG_WIN = -0.2                                                              # window = the board's outline + 0.2 (its parts run to the long edges: no rim on it)
# 2026-11-19: the Mini560 sits in a SLOT right under the lid plate, component side UP, its parts standing up through the window into the
# air under the Uno Q (was: hanging 0.2 below the plate, 3.6 mm over the battery - the battery's 14 AWG leads could not pass under it).
# Slot = the two side rails, each with a 0.8 mm ledge under the board's long edges; slid in along y on the bench before the lid goes on.
REG = dict(pcb=args.get("reg_pcb", 1.2), parts=args.get("reg_parts", 4.2))  # Mini560: PCB 1.2 + parts 4.2 = 5.4 (listing); parts layout ASSUMED
REG["z1"] = ZL - 0.95                                                       # PCB top 0.95 under the plate's underside: its parts end 1.25 mm under the
#                                                                             Uno Q's underside parts (datasheet: < 2 mm), 7 mm over the battery
REG["z0"] = REG["z1"] - REG["pcb"]
SWV = dict(x0=-48.0, x1=-36.0, y0=CY + 14.0, y1=CY + 20.0)                   # (12 x 6 x 6, slider out of the left side opening) under the lid
if VBAT:
    for x0_, x1_, l0, l1 in ((RG["x0"] - 1.45, RG["x0"] - 0.25, RG["x0"] - 0.3, RG["x0"] + 0.8), (RG["x1"] + 0.25, RG["x1"] + 1.45, RG["x1"] - 0.8, RG["x1"] + 0.3)):
        lid_add.append(box(x0_, x1_, RG["y0"], RG["y1"], REG["z0"] - 1.6, ZL + 0.3))                  # side rail
        lid_add.append(box(l0, l1, RG["y0"], RG["y1"], REG["z0"] - 1.6, REG["z0"] - 0.2))            # ledge under the board's edge (0.2 play)
    lid_sub.append(box(RG["x0"] + REG_WIN, RG["x1"] - REG_WIN, RG["y0"] + REG_WIN, RG["y1"] - REG_WIN, ZL - 1.0, ZL + LID_T + 1.0))
SW_X0 = XRP + T + 0.2                                                       # extended pocket: the power switch hangs under the lid's rear end, in the
if EXT:                                                                     # battery leads' room, slider up through a slot in the lid (reached from outside)
    lid_add += [box(SW_X0 - 0.2, SW_X0 + 6.0, CY + 6.25, CY + 7.45, ZL - 4.0, ZL + 0.3), box(SW_X0 - 0.2, SW_X0 + 6.0, CY - 7.45, CY - 6.25, ZL - 4.0, ZL + 0.3)]
    # side rails + the rear wall locate it (no front rail: the battery's rear top edge is 0.4 behind the switch); a dot of hot glue holds it
    lid_sub.append(box(SW_X0 + 1.5, SW_X0 + 4.5, CY - 3.5, CY + 3.5, ZL - 1.0, ZL + LID_T + 1.0))     # slot for the slider
lid = finish("body_lid", L["build_part"]("body_lid", [L["combine_cutters"](lid_add)], lid_sub, coll, mat_lid))
cover_parts = []
if TRAY and args.get("uno_cover", False):
    # cap over the Uno Q (user: close the board in, with a cut-out over its LED matrix): skirt outside the tray walls, 2x M2x6
    # through the skirt into the tray wall beside a board post (right rear, from the right; front left, from the front)
    CT, CG = 1.2, 0.3
    zc0, zc1 = zw - 4.0, uno_z + UNO["pcb"] + 8.5 + 4.0                     # skirt's lower edge (4 over the tray walls); ceiling 4 above the headers: the header wires need RIGHT-ANGLE housings
    CHF = 5.5                                                               # front top edge cut back at 45 deg: the head-pitch U's hub swings past there
    xo0, xo1, zt_ = UX0 - CG - CT, ux1 + CG + CT, zc1 + CT
    cap = hull([(x, y, z) for y in (uy0 - CG - CT, uy1 + CG + CT) for x, z in ((xo0, zc0), (xo1, zc0), (xo1, zt_ - CHF), (xo1 - CHF, zt_), (xo0, zt_))])
    cap_cut = [hull([(x, y, z) for y in (uy0 - CG, uy1 + CG) for x, z in ((UX0 - CG, zc0 - 1.0), (ux1 + CG, zc0 - 1.0), (ux1 + CG, zt_ - CHF - 0.5), (ux1 + CG - CHF + 0.5, zc1), (UX0 - CG, zc1))]),
               box(UX0 + 30.5, UX0 + 59.5, uy0 + 3.6, uy0 + 21.5, zc1 - 0.5, zc1 + CT + 0.5),                    # window over the LED matrix (board x 31.3..58.8, y 4.4..20.7: scaled from the datasheet drawing)
               box(UX0 - CG - CT - 0.5, UX0 + 1.0, uy0 + 28.8, uy0 + 46.8, zc0 - 1.0, uno_z + UNO["pcb"] + 8.0),   # USB-C -> USB-A adapter at the rear (17 wide + 0.5 each side)
               box(UX0 + 38.5, UX0 + 49.0, uy0 - CG - CT - 0.5, uy0 + 1.0, zc0 - 1.0, uno_z + UNO["pcb"] + 12.5),  # wires: right-angle plug on JANALOG 6-8 (right)
               box(UX0 + 17.0, UX0 + 26.0, uy0 - CG - CT - 0.5, uy0 + 1.0, zc0 - 1.0, uno_z + UNO["pcb"] + 5.0),   # Qwiic plug (right)
               box(UX0 + 35.0, UX0 + 43.0, uy1 - 1.0, uy1 + CG + CT + 0.5, zc0 - 1.0, uno_z + UNO["pcb"] + 11.0),  # light wires (left)
               box(UX0 + 57.0, UX0 + 66.0, uy1 - 1.0, uy1 + CG + CT + 0.5, zc0 - 1.0, uno_z + UNO["pcb"] + 12.5),  # UART: right-angle plug on D0 / D1 (left)
               L["add_cyl"](L["new_bm"](), CVS["a"][0] - HOME.x, -(CVS["a"][1] - HOME.z), uy0 - CG - CT - 0.5 - HOME.y, uy0 - CG + 0.5 - HOME.y, L["CLEAR_D"], 14, MYf),
               L["add_cyl"](L["new_bm"](), CVS["b"][0] - HOME.x, -(CVS["b"][1] - HOME.z), uy1 + CG - 0.5 - HOME.y, uy1 + CG + CT + 0.5 - HOME.y, L["CLEAR_D"], 14, MYf)]
    for hx, hy in [h for h in UNO_HOLES if h != UNO_M2]:                 # 2026-11-15: room round the M3 heads (dk 5.5 + 0.5 all round): they
        cap_cut.append(L["add_cyl"](L["new_bm"](), UX0 + hx - HOME.x, UY0 + hy - HOME.y,       # overhang the board's edge by 0.2 and sat 0.09
                                    uno_z + UNO["pcb"] - 0.6 - HOME.z, uno_z + UNO["pcb"] + 3.5 - HOME.z, 6.5, 28, Mt))   # from the skirt (0.6 under the head's overhang)
    uno_cover = L["build_part"]("uno_cover", [cap], cap_cut, coll, L["material"]("uno_cover", (0.25, 0.3, 0.38, 1)))
    if args.get("neck_sweep"):
        # the neck and the head-pitch U swing down over the board's front when the neck leans back (STAND +20, SIT +29 deg;
        # the board sits 12.5 mm higher for the audio tray): cut the cover above the lowest height they reach, 1.5 mm clear,
        # and keep the rear piece - the cover becomes a hood over the board's rear two thirds (parts/neck_sweep.py)
        import os
        NSW = {"ARGS": {"lib": True}}
        exec(open(args.get("neck_sweep_lib") or os.path.join(os.path.dirname(args["joint_lib"]), "neck_sweep.py")).read(), NSW)
        _P = [Mt.inverted() @ (uno_cover.matrix_world @ v.co) / MM + HOME for v in uno_cover.data.vertices]
        _box = tuple(f(p[i] for p in _P) for i in range(3) for f in (min, max))
        _poly, (_y0, _y1) = NSW["sweep_profile"](args["neck_sweep"], _box, R)
        if _poly is not None:
            L["apply_bool"](uno_cover, NSW["prism"](_poly, _y0, _y1, lambda p: Mt @ ((p - HOME) * MM)), "DIFFERENCE", coll)
            L["heal_slivers"](uno_cover, "uno_cover")
            if args["neck_sweep"].get("keep_rear", True) and L["shells"](uno_cover) > 1:
                NSW["keep_rearmost"](uno_cover, lambda w: Mt.inverted() @ w / MM + HOME)
        uno_cover["neck_sweep_cut"] = str(args["neck_sweep"])
    uno_cover = finish("uno_cover", uno_cover)
    uno_cover["optional"] = True
    cv_scr = L["pan_screw_dummy"]("uno_cover_screws_M2x6st", [(MYf, CVS["a"][0] - HOME.x, -(CVS["a"][1] - HOME.z), uy0 - CG - CT - HOME.y, +1),
                                                              (MYf, CVS["b"][0] - HOME.x, -(CVS["b"][1] - HOME.z), uy1 + CG + CT - HOME.y, -1)], coll)
    cover_parts = [uno_cover, cv_scr]
us = L["new_bm"]()
for hx, hy in [h for h in UNO_HOLES if h != UNO_M2]:                      # M3x8 socket-head screws: through the board (1.6) into the posts (6.4)
    L["add_cyl"](us, UX0 + hx - HOME.x, UY0 + hy - HOME.y, uno_z + UNO["pcb"] - HOME.z, uno_z + UNO["pcb"] + 3.0 - HOME.z, 5.5, 24, Mt)
    L["add_cyl"](us, UX0 + hx - HOME.x, UY0 + hy - HOME.y, uno_z + UNO["pcb"] - 8.0 - HOME.z, uno_z + UNO["pcb"] + 0.01 - HOME.z, 3.0, 16, Mt)
uno_screws = L["to_object"]("uno_screws_M3x8", us, coll, L["material"]("hw_steel", (0.6, 0.62, 0.66, 1)))
uno_screw_m2 = L["pan_screw_dummy"]("uno_screw_M2x6", [(Mt @ Matrix.Translation(Vector((UX0 + UNO_M2[0] - HOME.x, UY0 + UNO_M2[1] - HOME.y,
                                                                                        uno_z + UNO["pcb"] - HOME.z)) * MM), 0, 0, 0.0, -1)], coll)
lid_screws = L["pan_screw_dummy"]("body_lid_screws_M2x6st", [(Mt @ Matrix.Translation(Vector((px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZL + LID_T - HOME.z)) * MM), 0, 0, -0.7, -1) for px, py in POSTS], coll)

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
# servo adapter: board + its parts as boxes. Seen from the keel's middle with the sockets up: the two servo sockets top front, the UART
# header top rear, DC jack bottom front, screw terminal bottom middle, USB-C bottom rear. Part sizes and places: ESTIMATED from the photo.
ax0, ay, az = AD["x0"], AD["y0"] + AD["pcb"], zk
ad = L["new_bm"]()
for b in (box(ax0, ax0 + AD["len"], AD["y0"], ay, az, az + AD["h"]),                       # board
          box(ax0 + 30.0, ax0 + 40.0, ay, ay + 6.0, az + 24.5, az + 32.5),                # servo socket 1 (front)
          box(ax0 + 18.5, ax0 + 28.5, ay, ay + 6.0, az + 24.5, az + 32.5),                # servo socket 2
          box(ax0 + 3.0, ax0 + 10.6, ay, ay + 8.5, az + 28.5, az + 31.0),                 # UART header TX RX GND (rear)
          box(ax0 + 29.5, ax0 + 38.5, ay, ay + 11.0, az, az + 14.0),                      # DC jack
          box(ax0 + 18.5, ax0 + 26.5, ay, ay + 10.0, az + 0.3, az + 8.8),                 # screw terminal DC+ DC-
          box(ax0 + 6.5, ax0 + 15.5, ay, ay + 3.3, az, az + 7.3)):                        # USB-C
    b.to_mesh(tmp := bpy.data.meshes.new("t"))
    ad.from_mesh(tmp)
    bpy.data.meshes.remove(tmp)
if not VBAT:
    adapter_ob = L["to_object"]("servo_adapter", ad, ecoll, L["material"]("elec_servo_adapter", (0.1, 0.25, 0.7, 1)))
    adapter_ob["socket_xyz"] = (ax0 + 29.2, ay + 6.0, az + 28.5)                               # between the two sockets' mouths (layout mm)
    elec.append(adapter_ob)
    elec.append(ebox("ubec", XR + T + 0.4, CY + HK - T - 0.4 - ELEC["ubec"][1], zk, ELEC["ubec"], (0.75, 0.1, 0.1, 1)))
else:
    import bmesh
    import os
    ad.free()
    bpy.ops.wm.stl_import(filepath=args.get("adapter_stl") or os.path.join(os.path.dirname(args["joint_lib"]), "adapter_mesh", "waveshare_bus_servo_adapter_a.stl"))
    _raw = bpy.context.selected_objects[0]
    _bm = bmesh.new()
    _bm.from_mesh(_raw.data)
    bpy.data.objects.remove(_raw, do_unlink=True)
    def _ad_lay(bx, by, bz):                                                  # board frame (STEP, mm: x 0..42, y 0..33, z up from the PCB's top) -> layout
        return Vector((ADV["pcb_x1"] + bz, CY + 16.5 - by, ADV["z0"] + bx))
    for v in _bm.verts:
        v.co = Mt @ ((_ad_lay(*v.co) - HOME) * MM)
    _me = bpy.data.meshes.new("servo_adapter")
    _bm.to_mesh(_me)
    _bm.free()
    _me.materials.append(L["material"]("elec_servo_adapter", (0.1, 0.25, 0.7, 1)))
    adapter_ob = bpy.data.objects.new("servo_adapter", _me)
    ecoll.objects.link(adapter_ob)
    adapter_ob["layout"] = "vertical"
    adapter_ob["sockets_xyz"] = [list(_ad_lay(bx, 30.25, 5.9)) for bx in (10.5, 22.5)]          # the two servo sockets' mouths (plugs enter along +x)
    adapter_ob["term_back_xyz"] = list(_ad_lay(20.8, 2.8, -3.0))                               # screw terminal's pins, on the solder side
    adapter_ob["uart_back_xyz"] = list(_ad_lay(34.0, 20.0, -3.0))                              # UART header's pins, on the solder side
    adapter_ob["gap_x"] = (ADV["pcb_x1"] + 11.0 + VB["x0"]) / 2                                # middle of the cable gap in front of the adapter
    elec.append(adapter_ob)
    _rb = box(RG["x0"], RG["x1"], RG["y0"], RG["y1"], REG["z0"], REG["z1"])                         # Mini560 5 V 5 A: PCB
    _rp = box(RG["x0"] + 0.2, RG["x1"] - 0.2, RG["y0"] + 4.0, RG["y1"] - 4.0, REG["z1"], REG["z1"] + REG["parts"])   # its parts (envelope: to the long edges, clear of the end pads)
    _rp.to_mesh(_tmp := bpy.data.meshes.new("t")); _rb.from_mesh(_tmp); bpy.data.meshes.remove(_tmp)
    elec.append(L["to_object"]("ubec", _rb, ecoll, L["material"]("elec_ubec", (0.75, 0.1, 0.1, 1))))
BAT = ELEC["battery_3s"]
shelf = shelf_screws = None
if not VBAT:
    SHELF_T = 1.6                                                               # loose shelf on the pocket's ledges: the battery lies on it, above the keel
    SH_SCREW = (XP1 - T - 4.2, HP - T - 2.0)                                   # shelf screws: x, |y - CY| (over the ledges, where the legs leave most room)
    shelf_add = [box(XRP + T + 0.3, XP1 - T - 0.3, CY - HP + T + 0.3, CY + HP - T - 0.3, z_sh + 0.1, z_sh + 0.1 + SHELF_T),
                 box(XRP + 0.25, XRP + T + 0.6, CY - 5.0, CY + 5.0, z_sh + 0.15, z_sh + 0.05 + SHELF_T)]       # tab into the rear wall's slot
    BAT = ELEC["battery_3s"]
    for sgn in (-1, 1):                                                         # battery seat: a rib each side and two stops at its front end (hook-and-loop patch underneath)
        ya = sorted((BCY + sgn * (BAT[1] / 2 + 0.4), BCY + sgn * (BAT[1] / 2 + 1.6)))
        shelf_add.append(box(BAT_X0 + 3.7, (BAT_X0 + 11.7 if sgn > 0 else BAT_X0 + 21.7) if TRAY else BAT_X0 + 33.7, ya[0], ya[1], z_sh + SHELF_T - 0.2, z_sh + 0.1 + SHELF_T + 6.0))   # short with the block pack: clear of the lid posts' corbels (and of the USB hub on the left)
        yb = sorted((BCY + sgn * min(8.6, BAT[1] / 2 - 6.0), BCY + sgn * min(14.0, BAT[1] / 2 - 1.0)))
        shelf_add.append(box(BAT_X0 + BAT[0] + 0.4, BAT_X0 + BAT[0] + 1.6, yb[0], yb[1], z_sh + SHELF_T - 0.2, z_sh + 0.1 + SHELF_T + 6.0))
    if TRAY:
        zs_ = z_sh + SHELF_T
        HUB = dict(x0=-35.5, y0=BCY + BAT[1] / 2 + 0.4, len=24.0, t=5.0, h=20.1)                 # USB hub on edge between the battery and the left lid posts
        SWT = dict(x0=XR + T + 1.0, y0=BCY + BAT[1] / 2 + 2.6, len=12.0, wid=6.0, h=6.0)        # power switch at the open rear
        shelf_add += [] if EXT else ([box(-28.5, -13.0, HUB["y0"] + HUB["t"] + 0.25, HUB["y0"] + HUB["t"] + 1.3, zs_ - 0.2, zs_ + 3.0),         # hub: rail on its outer side (between the lid posts)
                       box(HUB["x0"] - 1.4, HUB["x0"] - 0.3, HUB["y0"] - 0.2, HUB["y0"] + HUB["t"] + 1.3, zs_ - 0.2, zs_ + 3.0)] if AUDIO else []) + [  # hub: rear stop
                      box(SWT["x0"] + 0.5, SWT["x0"] + SWT["len"], SWT["y0"] + SWT["wid"] + 0.25, SWT["y0"] + SWT["wid"] + 1.3, zs_ - 0.2, zs_ + 3.0),   # switch: outer rail
                      box(SWT["x0"] + SWT["len"] + 0.25, SWT["x0"] + SWT["len"] + 1.3, SWT["y0"] - 1.3, SWT["y0"] + SWT["wid"] + 1.3, zs_ - 0.2, zs_ + 3.0),   # switch: front stop
                      box(SWT["x0"] + 0.5, SWT["x0"] + SWT["len"] + 1.3, SWT["y0"] - 1.3, SWT["y0"] - 0.25, zs_ - 0.2, zs_ + 3.0)]       # switch: inner rail
    shelf_sub = [box(XP1 - T - 7.0, XP1 - T + 1.0, CY - 8.0, CY + 8.0, z_sh - 1.0, z_sh + 3.0)]      # notch at the front: wires from the keel come up here
    Msh = [Mt @ Matrix.Translation(Vector((SH_SCREW[0] - HOME.x, CY + sgn * SH_SCREW[1] - HOME.y, z_sh + 0.1 + SHELF_T - HOME.z)) * MM) for sgn in (-1, 1)]
    for M in Msh:
        shelf_sub += L["screw_holes"](M, [(0, 0)], 0.0, SHELF_T, 0.0, -1, csk=False)
    shelf = finish("battery_shelf", L["build_part"]("battery_shelf", [L["combine_cutters"](shelf_add)], shelf_sub, coll, mat_lid))
    shelf_screws = L["pan_screw_dummy"]("shelf_screws_M2x6st", [(M, 0, 0, 0.0, -1) for M in Msh], coll)
if VBAT:
    elec.append(ebox("battery_3s", VB["x0"], VB["y0"], VB["z0"], (BAT[2], BAT[1], BAT[0]), (0.15, 0.15, 0.15, 1)))     # standing, leads at the top
else:
    elec.append(ebox("battery_3s", BAT_X0, BCY - ELEC["battery_3s"][1] / 2, z_sh + 0.1 + SHELF_T + 0.2, ELEC["battery_3s"], (0.15, 0.15, 0.15, 1)))
if TRAY:
    # audio + USB + power stand-ins. Sizes: sound card = Waveshare USB TO AUDIO 53 x 23.5 x 14.6 (retailer figures); hub = Adafruit CH334F
    # 2-port 24 x 20.1 x 5; speaker 25 x 15 x 4 and switch 12 x 6 x 6 are NOMINAL (no part chosen).
    zt0 = ZL + LID_T + 0.2
    if AUDIO:
      elec.append(ebox("usb_audio", AUD["x0"], AUD["y0"], zt0, (AUD["len"], AUD["wid"], AUD["h"]), (0.1, 0.45, 0.2, 1)))              # flat in the tray between its rails; the board above keeps it down (0.3)
      elec.append(ebox("speaker", SPK["x0"], SPK["y0"], zt0, (SPK["len"], SPK["t"], SPK["h"]), (0.08, 0.08, 0.08, 1)))                  # on edge against the left wall, behind the sound holes
      elec.append(ebox("usb_hub", HUB["x0"], HUB["y0"], z_sh + 0.1 + SHELF_T + 0.2, (HUB["len"], HUB["t"], HUB["h"]), (0.1, 0.2, 0.6, 1)))   # on edge on the battery shelf, between the battery and the lid posts
    if VBAT:
        elec.append(ebox("power_switch", TG["xc"] - 3.9, TG["y_plate"] - 13.5, TG["zc"] - 6.6, (7.8, 13.5, 13.2), (0.85, 0.2, 0.1, 1)))   # mini toggle behind its plate (body + pins)
    elif EXT:
        elec.append(ebox("power_switch", SW_X0, CY - 6.0, ZL - 6.2, (6.0, 12.0, 6.0), (0.85, 0.2, 0.1, 1)))      # under the lid's rear end (NOMINAL 12 x 6 x 6)
    else:
        elec.append(ebox("power_switch", SWT["x0"], SWT["y0"], z_sh + 0.1 + SHELF_T + 0.2, (SWT["len"], SWT["wid"], SWT["h"]), (0.85, 0.2, 0.1, 1)))   # in its pocket on the shelf, at the open rear
if args.get("tof", False):                                                    # distance sensor on the chest plate's front, on the Qwiic chain
    elec.append(ebox("tof_sensor", CHEST["x0"] + CHEST["t"] + 0.9, (CHEST["y0"] + CHEST["y1"]) / 2 - 12.75, z_deck0 + 0.4, (4.6, 25.5, 17.7), (0.55, 0.1, 0.65, 1)))   # Adafruit VL53L1X STEMMA QT size (25.5 x 17.7 x 4.6, NOMINAL)
z_top = (VB["z0"] + ELEC["battery_3s"][0]) if VBAT else z_sh + 0.1 + 1.6 + 0.2 + ELEC["battery_3s"][2] + 0.3          # top of the battery
# balance sensor: flat on four bosses on the keel floor (low, central, rigid), 4x M2x6 from above before the adapter goes in
if VBAT:
    _ib = box(IMU_V["x_face"], IMU_V["x_face"] + ELEC["imu"][2], IMU_V["yc"] - ELEC["imu"][0] / 2, IMU_V["yc"] + ELEC["imu"][0] / 2,
              IMU_V["zc"] - ELEC["imu"][1] / 2, IMU_V["zc"] + ELEC["imu"][1] / 2)
    _ih = [L["add_cyl"](L["new_bm"](), *xpt(IMU_V["yc"] + sy * IMU_HOLES[0], IMU_V["zc"] + sz * IMU_HOLES[1]), IMU_V["x_face"] - 0.5 - HOME.x,
                        IMU_V["x_face"] + ELEC["imu"][2] + 0.5 - HOME.x, 2.3, 16, MX) for sy in (-1, 1) for sz in (-1, 1)]   # its 4 mounting holes
    elec.append(L["build_part"]("imu", [_ib], _ih, ecoll, L["material"]("elec_imu", (0.5, 0.1, 0.6, 1))))
    imu_screws = None                                                     # 2026-11-20: held on pegs + hot glue (no screws)
else:
    elec.append(ebox("imu", IMU_XY[0] - ELEC["imu"][0] / 2, IMU_XY[1] - ELEC["imu"][1] / 2, ZB + T + IMU_BOSS, ELEC["imu"], (0.5, 0.1, 0.6, 1)))
    imu_screws = L["pan_screw_dummy"]("imu_screws_M2x6st", [(Mt @ Matrix.Translation(Vector((IMU_XY[0] + sx * IMU_HOLES[0] - HOME.x, IMU_XY[1] + sy * IMU_HOLES[1] - HOME.y, ZB + T + IMU_BOSS + 1.6 - HOME.z)) * MM), 0, 0, 0.0, -1)
                                                          for sx in (-1, 1) for sy in (-1, 1)], coll)
for _n in ("bus_screw_M2x6st",):                                              # left over from the bus board
    _o = bpy.data.objects.get(_n)
    if _o:
        bpy.data.objects.remove(_o, do_unlink=True)
ZT = ZP_FRONT
for o in [x for x in [frame, enclosure, enc_screws, switch_plate, sw_screws, lid, lid_screws, shelf, shelf_screws, uno_screws, uno_screw_m2, imu_screws] if x is not None] + elec + cover_parts:
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
print(f"  keel inside {XK1 - XR - 2 * T:.1f} x {2 * HK - 2 * T:.1f} x {z_deck0 - ZB - T:.1f}; pocket inside {XP1 - XRP - 2 * T:.1f} x {2 * HP - 2 * T:.1f} x {ZP_REAR - z_deck1:.1f}..{ZP_FRONT - z_deck1:.1f}; Uno Q on top at z {uno_z:.1f}..{uno_z + UNO['pcb'] + UNO['top']:.1f}, x {UX0:.1f}..{UX0 + UNO['len']:.1f}")
for line in L["DEBUG"]:
    print("  !", line)
