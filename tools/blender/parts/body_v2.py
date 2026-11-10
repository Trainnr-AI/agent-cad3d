"""XGO-Duck body v2: one printed frame that holds the two hip-yaw servos, carries the neck, and houses the
off-the-shelf electronics; plus a lid and simple solids for the electronics (for clearance checks).
Runs inside Blender after the servos, legs and neck are built.

  body_frame   THE MOTOR BODY, nothing else: deck with the two hip-yaw caps (6702ZZ seats, pads, saddle walls) -
               replaces hip_yaw_support; neck pedestal + horn plate (full X pocket) - replaces body_neck_mount, moved
               straight under the neck axis so it clears the hip-yaw servos (the old web cut into both of them).
  electronics_enclosure   separate part behind the hips, shaped to what the legs and the head leave free: a keel
               (servo adapter, regulator) and a pocket above it (battery, a hub); hangs on the body by a tongue under
               the deck + 3x M2x6 into the deck's rear edge (driven from behind through access holes).
  body_lid     top plate on the enclosure's posts, 4x M2x6: closes the pocket, carries the IMU and, on stand-offs,
               the Uno Q (it fits nowhere inside: 69 x 53 mm against a 36-44 mm wide free space).
  electronics  collection of boxes: uno_q, battery_3s, servo_adapter, ubec, imu, hub_1..4 (sizes: see ELEC; the
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
ELEC = {"battery_3s": (42.0, 30.0, 13.0), "servo_adapter": (42.0, 18.5, 33.0), "ubec": (43.0, 8.0, 18.0), "imu": (25.6, 22.7, 4.6), "hub": (6.0, 17.0, 20.0)}
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
tub += [box(XR, XP1, p0, y0 + T, z_deck0 + 2.0, z_deck1), box(XR, XP1, y1 - T, p1, z_deck0 + 2.0, z_deck1)]
# pocket walls: rear part low, front part high
tub += [box(XR, XR + T, p0, p1, z_deck0 + 2.3, ZP_REAR),
        box(XR + 0.3, XP1, p0, p0 + T, z_deck0 + 2.3, ZP_REAR), box(XR + 0.3, XP1, p1 - T, p1, z_deck0 + 2.3, ZP_REAR),
        box(XP_STEP, XP1, p0, p0 + T, ZP_REAR - 0.3, ZP_FRONT), box(XP_STEP, XP1, p1 - T, p1, ZP_REAR - 0.3, ZP_FRONT),
        box(XP_STEP, XP_STEP + T, p0 + 0.3, p1 - 0.3, ZP_REAR - 0.3, ZP_FRONT),      # step wall
        box(XP1 - T, XP1, p0 + 0.3, p1 - 0.3, z_deck0 + 2.3, ZP_FRONT)]              # front wall (behind the hip-yaw servos)
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
ring = [(L["CAP_RING_R"] * math.cos(2 * math.pi * i / 48), L["CAP_RING_R"] * math.sin(2 * math.pi * i / 48)) for i in range(48)]
for F in (FyL, FyR):                                                              # the bearing seat's ring
    tub.append(L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in ring for z in (0.02, L["CAP_T"] - 0.02)], F))
body_solids = list(tub)
tub = []
# top plate posts: on the pocket's high walls (enclosure)
POSTS = [(XP_STEP + T - 0.3, p0 + T - 0.3), (XP_STEP + T - 0.3, p1 - T - 5.0 + 0.3), (XP1 - T - 5.0 + 0.3, p0 + T - 0.3), (XP1 - T - 5.0 + 0.3, p1 - T - 5.0 + 0.3)]
for px, py in POSTS:
    enc.append(box(px, px + 5.0, py, py + 5.0, ZP_FRONT - 9.0, ZP_FRONT - 0.01))
# how the enclosure hangs on the body: a tongue under the deck (centre, between the hips) + 3x M2x6 through its front wall
# into the deck's rear edge, driven from behind through access holes in the rear wall
TONGUE = (XP1 - 0.3, 3.0, 8.0, 3.0)                                        # from x, to x, half-width, thickness
enc.append(box(TONGUE[0], TONGUE[1], CY - TONGUE[2], CY + TONGUE[2], z_deck0 - 0.2 - TONGUE[3], z_deck0 - 0.2))
enc.append(box(XK1 - 0.3, TONGUE[0] + 0.3, CY - TONGUE[2], CY + TONGUE[2], z_deck0 - 0.2 - TONGUE[3], z_deck0 - 0.2))   # ties the tongue to the keel's front wall
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
        box(4.0, 24.0, CY - 4.5, CY + 4.5, z_deck0 - 1.0, z_deck1 + 1.0)]                                                               # cable slot between the two servos
for y in JOIN_Y:                                                           # pilots in the deck's rear edge for the enclosure's screws
    cuts.append(L["add_cyl"](L["new_bm"](), *xpt(CY + y, z_join), X_JOIN - HOME.x - 0.5, X_JOIN - HOME.x + 4.6, L["PILOT_D"], 14, MX))
frame = finish("body_frame", L["build_part"]("body_frame", [L["combine_cutters"](body_solids + [pedestal, ped_foot, web] + cL + cR)] + a_add,   # all simple solids unioned first
                                             a_sub + [L["combine_cutters"](sL), L["combine_cutters"](sR)] + cuts, coll, mat_frame, a_rib))   # each cap's cuts as ONE cutter
enc_cuts = []
for px, py in POSTS:
    enc_cuts.append(L["add_cyl"](L["new_bm"](), px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZP_FRONT - 8.0 - HOME.z, ZP_FRONT + 1.0 - HOME.z, L["PILOT_D"], 14, Mt))
for y in JOIN_Y:
    enc_cuts.append(L["add_cyl"](L["new_bm"](), *xpt(CY + y, z_join), XP1 - T - 0.6 - HOME.x, XP1 + 0.6 - HOME.x, L["CLEAR_D"], 16, MX))      # clearance through the front wall
    enc_cuts.append(L["add_cyl"](L["new_bm"](), *xpt(CY + y, z_join), XR - 0.6 - HOME.x, XR + T + 0.6 - HOME.x, 4.6, 20, MX))                 # driver access through the rear wall
mat_enc = L["material"]("electronics_enclosure", (0.35, 0.55, 0.75, 1))
enclosure = finish("electronics_enclosure", L["build_part"]("electronics_enclosure", [L["combine_cutters"](enc)], enc_cuts, coll, mat_enc))
enc_screws = L["pan_screw_dummy"]("enclosure_screws_M2x6st", [(MX, *xpt(CY + y, z_join), XP1 - T - HOME.x, +1) for y in JOIN_Y], coll)

# ------------------------------------------------------------------ top plate: closes the pocket and carries the Uno Q above the body
LID_T = 2.0
ZL = ZP_FRONT + 0.1
UX0 = XP1 + 3.0 - UNO["len"] + 21.0                                       # Uno Q: over the pocket and the hip-yaw servos (x -50 .. 19)
lid_add = [box(XP_STEP + 0.3, XP1 + 17.0, CY - UNO["wid"] / 2 - 0.5, CY + UNO["wid"] / 2 + 0.5, ZL, ZL + LID_T)]
STAND = 3.5                                                                # stand-offs under the board (clear of its underside pins)
uno_z = ZL + LID_T + STAND
for sx in (XP_STEP + 2.0, XP1 + 13.0):
    for sy in (CY - UNO["wid"] / 2 + 0.2, CY + UNO["wid"] / 2 - 3.2):
        lid_add.append(box(sx, sx + 4.0, sy, sy + 3.0, ZL + LID_T - 0.3, uno_z))
lid_sub = []
for px, py in POSTS:
    M = Mt @ Matrix.Translation(Vector((px + 2.5 - HOME.x, py + 2.5 - HOME.y, ZL + LID_T - HOME.z)) * MM)
    lid_sub += L["screw_holes"](M, [(0, 0)], 0.0, LID_T, 0.0, -1, csk=False)
lid = finish("body_lid", L["build_part"]("body_lid", [L["combine_cutters"](lid_add)], lid_sub, coll, mat_lid))
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
          box(UX0 + 3.0, UX0 + UNO["len"] - 3.0, CY - UNO["wid"] / 2 + 4.5, CY + UNO["wid"] / 2 - 4.5, uno_z - UNO["pins"] + 0.3, uno_z)):
    b.to_mesh(tmp := bpy.data.meshes.new("t"))
    u.from_mesh(tmp)
    bpy.data.meshes.remove(tmp)
elec.append(L["to_object"]("uno_q", u, ecoll, L["material"]("elec_uno", (0.0, 0.45, 0.55, 1))))
zk = ZB + T + 0.5                                                           # keel floor
elec.append(ebox("servo_adapter", XR + T + 0.4, CY - HK + T + 0.4, zk, ELEC["servo_adapter"], (0.1, 0.25, 0.7, 1)))
elec.append(ebox("ubec", XR + T + 0.4, CY + HK - T - 0.4 - ELEC["ubec"][1], zk, ELEC["ubec"], (0.75, 0.1, 0.1, 1)))
elec.append(ebox("battery_3s", XR + T + 0.6, CY - ELEC["battery_3s"][1] / 2, z_deck1 + 0.4, ELEC["battery_3s"], (0.15, 0.15, 0.15, 1)))
elec.append(ebox("imu", XP_STEP + T + 0.6, CY - HP + T + 0.5, ZP_REAR + 1.2, ELEC["imu"], (0.5, 0.1, 0.6, 1)))      # in the pocket's front part, above the battery
for i in range(5):                                                          # hubs: in the slot between the two hip-yaw servos
    elec.append(ebox(f"hub_{i + 1}", -3.0 + i * 6.4, CY - ELEC["hub"][1] / 2, z_deck1 + 2.0, ELEC["hub"], (0.05, 0.5, 0.15, 1)))
elec.append(ebox("hub_6", XP_STEP + T + 0.6, CY + HP - T - 0.5 - 17.0, ZP_REAR + 1.2, (20.0, 17.0, 6.0), (0.05, 0.5, 0.15, 1)))     # flat, beside the IMU
ZT = ZP_FRONT
for o in [frame, enclosure, enc_screws, lid, lid_screws] + elec:
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
