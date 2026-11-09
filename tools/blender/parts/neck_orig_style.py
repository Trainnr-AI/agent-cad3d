"""XGO-Duck neck for SCS0009 servos (no rear shaft, flat bottom), built in place from the layout.
Runs inside Blender after place_servos.py (servo roots scs_neck_pitch / scs_head_pitch / scs_head_yaw,
robot at its zero pose).

  neck_cradle      (replaces the 2x_neck plates)  holds the neck-pitch and head-pitch servos by
                   their ears (pads + M2 self-tap pilots) on one floor plate under their flat
                   bottoms; a Ø10 pin on the floor under each output shaft = each joint's rear pivot.
  body_neck_mount  (replaces the arms on the body) horn arm (trimmed-X-horn crush-rib pocket) + web,
                   web placed in the sector the neck never sweeps. Slides along the shaft onto the horn.
  body_neck_bearing_plate  separate: 10x15x3 seat for the cradle's lower pin; slides on from the
                   other side, 2x M2x6 csk into the web. (A one-piece U can't be assembled: its far
                   arm would have to pass through the servo.)
  head_pitch_u     (replaces head_pitch) horn arm + top plate = head-yaw moving plate (Ø16 boss into the
                   16x22x4 bearing in the yaw block, trimmed-horn pocket).
  head_pitch_bearing_plate  separate, for the upper pin; 2x M2x8 csk down through the top plate.

ARGS = {"robot": "duck_scs0009", "collection": "neck_v1", "joint_lib": "/abs/parts/joint_orig_style.py"}
"""
import math

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_lib", "ARGS": {"demo": False}}
exec(open(args["joint_lib"]).read(), L)
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "neck_v1")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in list(coll.objects):
    bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()
MM = 0.001

Fn = bpy.data.objects["scs_neck_pitch"].matrix_world.copy()
Fh = bpy.data.objects["scs_head_pitch"].matrix_world.copy()
Fy = bpy.data.objects["scs_head_yaw"].matrix_world.copy()
joint = {o["joint"]: o for o in bpy.data.collections[R].objects if o.type == "EMPTY" and "joint" in o and o.name.startswith(R + ":")}

# ---- dimensions in the servo frame (mm)
FLOOR_T = 1.6
Z_BOTTOM = -25.25                       # servo's flat bottom
FLOOR = (Z_BOTTOM - FLOOR_T, Z_BOTTOM)  # cradle floor
PIN_D, PIN_L = 10.0, 3.5
BRG_GAP, BRG_PLATE_T = 0.5, 4.2
BRG = (FLOOR[0] - BRG_GAP - BRG_PLATE_T, FLOOR[0] - BRG_GAP)   # bearing arm z range
ARM = (2.6, 6.4)                        # horn arm z range (horn plate 2.7..4.3 sits in its pocket)
ARM_R = 9.5
PAD_Y = 4.5
PAD_NEAR = (-9.8, -6.15)                # under the ear at the shaft end
PAD_FAR = (17.55, 21.2)                 # under the ear at the connector end
CABLE = (17.4, 21.3, -2.5, 2.5, Z_BOTTOM - 0.05, -21.0)   # notch for the cable leaving the end face
WEB_R = (14.0, 20.0)
WEB_HW = 6.0

mat_cradle = L["material"]("neck_cradle", (0.93, 0.93, 0.9, 1))
mat_mount = L["material"]("neck_mount", (0.16, 0.18, 0.24, 1))
mat_u = L["material"]("neck_u", (0.98, 0.72, 0.15, 1))
report = []


def fr(M, p_mm):
    return (M.inverted() @ (Vector(p_mm) * MM)) / MM


def wr(M, p_mm):
    return (M @ (Vector(p_mm) * MM)) / MM


def disc(cx, cy, r, n=40):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def plate(M, z0, z1, pts):
    return L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in pts for z in (z0, z1)], M)


def finish(name, part):
    ok, vol = L["manifold"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    n = L["shells"](part)
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    return part


def free_dir(e, M, ray_world):
    """Unit vector (in frame M's XY, mm coords) pointing into the sector that `ray_world` (a direction
    rotating with the joint's child) never sweeps over the joint range."""
    a = (e.matrix_world.to_3x3() @ Vector(e["joint_axis"])).normalized()
    lo, hi = e["joint_range"]
    w = Matrix.Rotation((lo + hi) / 2 + math.pi, 3, a) @ ray_world
    d = M.inverted().to_3x3() @ w
    d.z = 0
    return d.normalized(), math.degrees((hi - lo) / 2)


def horn_arm(M, reach_xy):
    """U's horn arm (adds, subs, ribs) in servo frame M: trimmed-X crush-rib pocket + M2x6 screw."""
    adds = [plate(M, ARM[0], ARM[1], disc(0, 0, ARM_R) + reach_xy)]
    subs = L["x_pocket_cutters"](M, z0=ARM[0] - 0.5) + [
        L["add_cyl"](L["new_bm"](), 0, 0, ARM[0] - 1.1, ARM[1] + 1.1, L["CLEAR_D"], 20, M)]
    return adds, subs, L["rib_adds"](M, z_mouth=ARM[0] + 0.1)


def bearing_plate_cuts(M):
    return [L["add_cyl"](L["new_bm"](), 0, 0, BRG[1] - 3.0, BRG[1] + 1.0, 15.2, 64, M),      # 10x15x3 seat
            L["add_cyl"](L["new_bm"](), 0, 0, BRG[0] - 1.0, BRG[1] + 1.2, 12.0, 48, M)]      # through hole (lip)


# =========================================================== neck cradle (moves with the neck)
pH = fr(Fn, wr(Fh, (0, 0, 0)))                          # head-pitch axis in the neck-pitch frame
# floor: servo width (±CAP_HW) plus the ear-pad footprints; a wider floor's side edge clips the
# head-pitch U's top plate at the end of head pitch (±104°)
foot = []
for M in (Fn, Fh):
    for x in (PAD_NEAR[0], PAD_FAR[1]):
        for y in (-PAD_Y, PAD_Y):
            foot.append(tuple(fr(Fn, wr(M, (x, y, 0)))[:2]))
floor = plate(Fn, FLOOR[0], FLOOR[1], disc(0, 0, L["CAP_HW"]) + disc(pH.x, pH.y, L["CAP_HW"]) + foot)
adds, subs = [floor], []
for M in (Fn, Fh):
    for x0, x1 in (PAD_NEAR, PAD_FAR):
        adds.append(L["add_box"](L["new_bm"](), x0, x1, -PAD_Y, PAD_Y, Z_BOTTOM - 0.02, L["EAR_Z"][0], M))
    # rear pivot: Ø10 pin printed with the floor = coaxial with the output shaft by construction
    adds.append(L["add_cyl"](L["new_bm"](), 0, 0, FLOOR[0] - PIN_L, FLOOR[0] + 0.3, PIN_D, 48, M))
    for hx in L["HOLE_XS"]:
        subs.append(L["add_cyl"](L["new_bm"](), hx, 0, L["EAR_Z"][0] - 4.4, L["EAR_Z"][0] + 0.5, L["PILOT_D"], 16, M))
    subs.append(L["add_box"](L["new_bm"](), *CABLE, M))
cradle = finish("neck_cradle", L["build_part"]("neck_cradle", adds, subs, coll, mat_cradle))

# =========================================================== body neck mount (fixed to the body)
# U = horn arm + web (one piece; slides along the shaft onto the horn). The bearing arm is a
# separate plate: it slides on from the other side over the cradle's pin, 2x M2x6 csk into the web.
e_n = joint["neck_pitch"]
neck_ray = (Fn.to_3x3() @ Vector((1, 0, 0))).normalized()          # along the neck (servo +X)
wd, half = free_dir(e_n, Fn, neck_ray)
t = Vector((-wd.y, wd.x, 0))
web_xy = [tuple((wd * r + t * s * WEB_HW)[:2]) for r in WEB_R for s in (-1, 1)]
arm_reach = [tuple((wd * r + t * s * (WEB_HW - 0.4))[:2]) for r in (WEB_R[0] + 0.4, WEB_R[1] - 0.4) for s in (-1, 1)]  # inside the web: no shared faces
web = L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in web_xy for z in (BRG[1], ARM[1] - 0.05)], Fn)
web_screws = [tuple((wd * sum(WEB_R) / 2 + t * s * 3.5)[:2]) for s in (-1, 1)]
a_add, a_sub, a_rib = horn_arm(Fn, arm_reach)
a_sub += L["screw_holes"](Fn, web_screws, BRG[1], 0.0, 7.0, +1, csk=False)            # pilots up into the web
mount = finish("body_neck_mount", L["build_part"]("body_neck_mount", a_add + [web], a_sub, coll, mat_mount, a_rib))
mount_plate = finish("body_neck_bearing_plate", L["build_part"](
    "body_neck_bearing_plate", [plate(Fn, BRG[0], BRG[1], disc(0, 0, ARM_R) + web_xy)],
    bearing_plate_cuts(Fn) + L["screw_holes"](Fn, web_screws, BRG[0], BRG[1] - BRG[0], 0.0, +1), coll, mat_mount))
print(f"neck web: free sector half-width {180 - half:.0f}° each side of the web direction")

# =========================================================== head-pitch U (moves with the head)
# U = horn arm + top plate (= the head-yaw moving plate). Bearing side = separate plate whose
# thickened top block butts against the top plate's underside; 2x M2x8 csk down through the top plate.
# Fy's +Z points DOWN (yaw output faces down) and Fh's +x points down the neck: PLATE_Z0 is the
# top plate's upper face (more negative Fh x), PLATE_Z1 its underside.
top_x = fr(Fh, wr(Fy, (0, 0, L["PLATE_Z0"]))).x                   # upper face, in Fh x
bot_x = fr(Fh, wr(Fy, (0, 0, L["PLATE_Z1"]))).x                   # underside
assert top_x < bot_x
BLOCK = (BRG[0] - 3.0, BRG[1])                                     # block thickened outward by 3 mm
h_add, h_sub, h_rib = horn_arm(Fh, [(bot_x - 1.0, -ARM_R), (bot_x - 1.0, ARM_R)])   # 1 mm into the top plate
corners = [wr(Fh, (top_x, y, z)) for y in (-ARM_R, ARM_R) for z in (ARM[1], BLOCK[0])]
y_add, y_sub, y_rib = L["link_features"](Fy, [tuple(fr(Fy, p)[:2]) for p in corners])
# screws: along Fh -x (world up), through the top plate into the block
blk_z = (BLOCK[0] + BLOCK[1]) / 2
Ms = [Fh @ Matrix.Translation(Vector((top_x, yy, blk_z)) * MM) @ Matrix.Rotation(math.radians(90), 4, "Y") for yy in (-6.0, 6.0)]
# in Ms: local +Z = Fh +x = down, local origin at the top plate's upper face
top_t = bot_x - top_x
h_screws = [c for M in Ms for c in L["screw_holes"](M, [(0, 0)], 0.0, top_t, 0.0, +1)]
u = finish("head_pitch_u", L["build_part"]("head_pitch_u", h_add + y_add, h_sub + y_sub + h_screws, coll, mat_u, h_rib + y_rib))
blk = L["add_box"](L["new_bm"](), bot_x, bot_x + 5.0, -ARM_R, ARM_R, BLOCK[0], BLOCK[1], Fh)
hp_plate = finish("head_pitch_bearing_plate", L["build_part"](
    "head_pitch_bearing_plate", [plate(Fh, BRG[0], BRG[1], disc(0, 0, ARM_R) + [(bot_x + 0.3, -ARM_R), (bot_x + 0.3, ARM_R)]), blk],
    bearing_plate_cuts(Fh) + [c for M in Ms for c in L["screw_holes"](M, [(0, 0)], top_t, 0.0, 5.5, +1, csk=False)], coll, mat_u))

# =========================================================== bearings (visual): 10x15x3 in each bearing plate
steel = L["material"]("oj_steel", (0.6, 0.62, 0.66, 1))
bearings = []
for nm, M, owner in (("brg_neck_pitch", Fn, mount_plate), ("brg_head_pitch", Fh, hp_plate)):
    b = L["to_object"](nm, L["add_cyl"](L["new_bm"](), 0, 0, BRG[1] - 3.0, BRG[1], 15.0, 48, M), coll, steel)
    L["apply_bool"](b, L["add_cyl"](L["new_bm"](), 0, 0, BRG[1] - 4.0, BRG[1] + 1.0, 10.0, 48, M), "DIFFERENCE", coll)
    bearings.append((b, owner))

# =========================================================== parent to bodies so they move with the joints
for part, body in ((cradle, e_n), (mount, e_n.parent), (mount_plate, e_n.parent), (u, joint["head_pitch"]),
                   (hp_plate, joint["head_pitch"])):
    mw = part.matrix_world.copy()
    part.parent = body
    part.matrix_world = mw
for b, owner in bearings:
    mw = b.matrix_world.copy()
    b.parent = owner
    b.matrix_world = mw
for n in ("neck_pitch", "head_pitch"):                              # old scaled shells: replaced
    o = bpy.data.objects.get(f"{R}:{n}")
    if o:
        o.hide_set(True)
        o.hide_render = True
print("neck: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
