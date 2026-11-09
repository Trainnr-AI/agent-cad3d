"""XGO-Duck leg for SCS0009 servos, original-style joints, built in place from the layout.
Runs inside Blender after place_servos.py (needs scs_<side>_<joint> servo roots at the zero pose).

Each part maps to an original XGO-Duck part and is made of joint features
(parts/joint_orig_style.py) placed in the servo frames, joined by flat plates:
  hip_yaw_support   (body plate)        caps for both hip-yaw servos
  yaw_connector     (2x_hip_yaw2rol)    hip-yaw moving plate + hip-roll cap
  l_bracket         (2x_hip_rol_output) hip-roll moving plate + hip-pitch moving plate
  thigh             (thigh plate)       hip-pitch cap + knee cap
  shin              (2x_shank)          knee moving plate + ankle-servo holder (floor, ear pads, Ø10 pin)
  foot              (foot)              ankle horn plate (outboard) + sole
  foot_bearing_plate                    separate: 10x15x3 bearing on the shin's Ø10 pin; flange screws to the sole
Knee/hip joints: 16x22x4 bearing around the output in the cap (as the original). Ankle: two-sided,
as the original foot (its rear stub + Ø10 axis is replaced by a Ø10 pin on the shin).

ARGS = {"side": "left", "collection": "leg_v1", "joint_lib": "/abs/parts/joint_orig_style.py",
        "foot_mesh": "duck_scs0009:left_ankle", "with_support": True}
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_lib", "ARGS": {"demo": False}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
SIDE = args.get("side", "left")
COLL = args.get("collection", "leg_v1")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
prefix = f"{SIDE}_"
for o in [o for o in coll.objects if o.name.startswith(prefix) or (args.get("with_support") and o.name.startswith("hip_yaw_support"))]:
    bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()

F = {j: bpy.data.objects[f"scs_{SIDE}_{j}"].matrix_world.copy() for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")}
mat_dark = L["material"]("leg_dark", (0.16, 0.18, 0.24, 1))      # holding parts (original: black)
mat_white = L["material"]("leg_white", (0.93, 0.93, 0.9, 1))     # moving parts (original: white)
mat_foot = L["material"]("leg_foot", (0.98, 0.72, 0.15, 1))      # foot (original: yellow)


def fr(M, p_world_mm):
    """world point (mm) -> frame coords (mm)"""
    q = M.inverted() @ (Vector(p_world_mm) * MM)
    return q / MM


def wr(M, p_frame_mm):
    """frame point (mm) -> world point (mm)"""
    return (M @ (Vector(p_frame_mm) * MM)) / MM


def plate(M, z0, z1, outline_frame_mm):
    """flat plate in frame M: hull of 2D outline points (mm) between z0 and z1 (mm)"""
    return L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in outline_frame_mm for z in (z0, z1)], M)


def disc_pts(cx, cy, r, n=40):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def proj(M, pts_world_mm):
    """project world points into frame M's XY (drop z)"""
    return [tuple(fr(M, p)[:2]) for p in pts_world_mm]


def box_w(x0, x1, y0, y1, z0, z1):
    return L["add_box"](L["new_bm"](), x0, x1, y0, y1, z0, z1)


def axis_pt(j):
    return wr(F[j], (0, 0, 0))


report = []


def finish(name, part):
    ok, vol = L["manifold"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    part["manifold"] = ok
    n = L["shells"](part)
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    return part


# ------------------------------------------------------------------ thigh = hip-pitch cap + knee cap (one plate)
ca1, cs1 = L["cap_features"](F["hip_pitch"], include_plate=False)
ca2, cs2 = L["cap_features"](F["knee"], include_plate=False)
# two cap plates joined by a narrow bridge (a hull of both would fill the region the shin swings through)
def cap_rect(j, z0=0.0, z1=L["CAP_T"]):
    ring = [(L["CAP_RING_R"] * math.cos(2 * math.pi * i / 48), L["CAP_RING_R"] * math.sin(2 * math.pi * i / 48), 0) for i in range(48)]
    return [wr(F[j], (x, y, 0)) for x in L["EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])] + [wr(F[j], p) for p in ring]


hp_plate = plate(F["hip_pitch"], 0.0, L["CAP_T"], proj(F["hip_pitch"], cap_rect("hip_pitch")))
kn_plate = plate(F["hip_pitch"], 0.02, L["CAP_T"] - 0.02, proj(F["hip_pitch"], cap_rect("knee")))
# bridge: from the knee cap's far (upper) end to the hip-pitch cap's far end, 9 mm wide
kn_top = wr(F["knee"], (L["EAR_X"][1], 0, 0))
hp_far = wr(F["hip_pitch"], (L["EAR_X"][1], 0, 0))
d = (hp_far - kn_top).normalized()
nrm = d.cross(wr(F["hip_pitch"], (0, 0, 1)) - wr(F["hip_pitch"], (0, 0, 0))).normalized()
bridge_pts = [kn_top + nrm * 4.5, kn_top - nrm * 4.5, hp_far + nrm * 4.5, hp_far - nrm * 4.5]
bridge = plate(F["hip_pitch"], 0.04, L["CAP_T"] - 0.04, proj(F["hip_pitch"], bridge_pts))
thigh = finish("thigh", L["build_part"](f"{prefix}thigh", [hp_plate, kn_plate, bridge] + ca1 + ca2, cs1 + cs2, coll, mat_dark))

# ------------------------------------------------------------------ shin = knee moving plate + ankle holder
kn, an = axis_pt("knee"), axis_pt("ankle")
# ankle servo (held here) in its frame: body x -5.95..17.35, bottom at z = -25.25; floor below it
FLOOR, PAD_TOP = 1.6, -25.25
z_body_end = wr(F["ankle"], (L["BODY_X"][1], 0, 0)).z              # top end of the ankle servo (world mm)
shin_stop = z_body_end + 1.2                                         # knee plate stops just above it
la, ls, lr = L["link_features"](F["knee"], proj(F["knee"], [(kn.x - 6, kn.y, shin_stop), (kn.x + 6, kn.y, shin_stop)]))
# floor under the ankle servo (ankle frame), covering body + ears; Ø10 rear pin; ear pads up to the ears
floor = plate(F["ankle"], PAD_TOP - FLOOR, PAD_TOP + 0.01,
              [(x, y) for x in (L["EAR_X"][0], L["EAR_X"][1]) for y in (-L["CAP_HW"], L["CAP_HW"])])
pads = [L["add_box"](L["new_bm"](), x0, x1, -L["EAR_W"] / 2, L["EAR_W"] / 2, PAD_TOP - 0.01, L["EAR_Z"][0], F["ankle"])
        for x0, x1 in ((L["EAR_X"][0], L["BODY_X"][0] - 0.2), (L["BODY_X"][1] + 0.2, L["EAR_X"][1]))]
pin = L["add_cyl"](L["new_bm"](), 0, 0, PAD_TOP - FLOOR - 3.5, PAD_TOP - FLOOR + 0.3, 10.0, 48, F["ankle"])   # rear pivot, coaxial by construction
# joggle: knee plate band (world y of the knee plate) -> floor band, above the ankle servo's top end
kp0 = wr(F["knee"], (0, 0, L["PLATE_Z1"])).y
kp1 = wr(F["knee"], (0, 0, L["PLATE_Z0"])).y
fl0 = wr(F["ankle"], (0, 0, PAD_TOP - FLOOR)).y
y_lo, y_hi = min(kp0, kp1, fl0), max(kp0, kp1, fl0 + FLOOR)
joggle = box_w(kn.x - 6.03, kn.x + 6.02, y_lo + 0.05, y_hi - 0.05, shin_stop - 0.01, shin_stop + 5.0)   # inset: no faces shared with the plate/floor
# the knee plate itself must reach down to the joggle
shin_subs = ls + [L["add_cyl"](L["new_bm"](), hx, 0, PAD_TOP - FLOOR - 1, L["EAR_Z"][0] + 0.5, L["PILOT_D"], 16, F["ankle"])
                  for hx in L["HOLE_XS"]]
shin = finish("shin", L["build_part"](f"{prefix}shin", la + [joggle, floor, pin] + pads, shin_subs, coll, mat_white, lr))

# ------------------------------------------------------------------ foot = horn plate (outboard) + bearing plate (inboard) + sole
fm = bpy.data.objects[args["foot_mesh"]]
fv = [fm.matrix_world @ v.co / MM for v in fm.data.vertices]
zmin = min(v.z for v in fv)
sole_v = [v for v in fv if v.z < zmin + 4.0]
sx0, sx1 = min(v.x for v in sole_v), max(v.x for v in sole_v)
sy0, sy1 = min(v.y for v in sole_v), max(v.y for v in sole_v)
SOLE_T = 5.0
sole = box_w(sx0, sx1, sy0, sy1, zmin, zmin + SOLE_T)
# horn plate (outboard): plain X-pocket arm (no 16x22x4 here: the ankle is two-sided, as the original)
ARM_Z0, ARM_Z1 = 2.6, 6.4
reach = [(an.x - 9, an.y, zmin + SOLE_T - 0.5), (an.x + 9, an.y, zmin + SOLE_T - 0.5)]
horn_plate = plate(F["ankle"], ARM_Z0, ARM_Z1, disc_pts(0, 0, 9.5) + proj(F["ankle"], reach))
foot_subs = L["x_pocket_cutters"](F["ankle"], z0=ARM_Z0 - 0.5) + [
    L["add_cyl"](L["new_bm"](), 0, 0, ARM_Z0 - 1, ARM_Z1 + 1, L["CLEAR_D"], 20, F["ankle"])]
# bearing plate (inboard): 10x15x3 seat riding on the shin's Ø10 pin
BRG_Z1 = PAD_TOP - FLOOR - 0.5            # 0.5 mm below the shin floor
BRG_Z0 = BRG_Z1 - 4.2
# The bearing plate is a SEPARATE part (a one-piece foot can't be assembled: sliding it onto the horn
# would push the inboard plate through the servo). It slides on over the shin's pin from the inside
# and its flange screws down onto the sole (2x M2x6 csk).
top = zmin + SOLE_T
reach_b = [(an.x - 9, an.y, top + 0.3), (an.x + 9, an.y, top + 0.3)]
brg_plate = plate(F["ankle"], BRG_Z0, BRG_Z1, disc_pts(0, 0, 9.5) + proj(F["ankle"], reach_b))
y_a, y_b = wr(F["ankle"], (0, 0, BRG_Z0 - 6.0)).y, wr(F["ankle"], (0, 0, BRG_Z1)).y
flange = box_w(an.x - 9, an.x + 9, min(y_a, y_b), max(y_a, y_b), top, top + 2.5)
y_scr = wr(F["ankle"], (0, 0, BRG_Z0 - 3.0)).y
M_scr = [Matrix.Translation(Vector((an.x + dx, y_scr, top + 2.5)) * MM) for dx in (-5.5, 5.5)]
M_sole = [Matrix.Translation(Vector((an.x + dx, y_scr, top)) * MM) for dx in (-5.5, 5.5)]
foot_subs += [c for M in M_sole for c in L["screw_holes"](M, [(0, 0)], 0.0, 0.0, 4.0, -1, csk=False)]
foot = finish("foot", L["build_part"](f"{prefix}foot", [sole, horn_plate], foot_subs, coll, mat_foot,
                                      L["rib_adds"](F["ankle"], z_mouth=ARM_Z0 + 0.1)))
bp_subs = [L["add_cyl"](L["new_bm"](), 0, 0, BRG_Z1 - 3.0, BRG_Z1 + 1, 15.2, 64, F["ankle"]),
           L["add_cyl"](L["new_bm"](), 0, 0, BRG_Z0 - 1, BRG_Z1 + 1, 12.0, 48, F["ankle"])]
bp_subs += [c for M in M_scr for c in L["screw_holes"](M, [(0, 0)], 0.0, 2.5, 0.0, -1)]
foot_bp = finish("foot_bearing_plate", L["build_part"](f"{prefix}foot_bearing_plate", [brg_plate, flange], bp_subs, coll, mat_foot))
steel = L["material"]("oj_steel", (0.6, 0.62, 0.66, 1))
brg = L["to_object"](f"{prefix}brg_ankle", L["add_cyl"](L["new_bm"](), 0, 0, BRG_Z1 - 3.0, BRG_Z1, 15.0, 48, F["ankle"]), coll, steel)
L["apply_bool"](brg, L["add_cyl"](L["new_bm"](), 0, 0, BRG_Z1 - 4.0, BRG_Z1 + 1.0, 10.0, 48, F["ankle"]), "DIFFERENCE", coll)

# ------------------------------------------------------------------ l_bracket = hip-roll moving plate + hip-pitch moving plate
hr, hp = axis_pt("hip_roll"), axis_pt("hip_pitch")
def far_face(j, coord, toward):
    """world coord of joint j's link-plate face that is farther from `toward` (the other plate's axis)"""
    a = getattr(wr(F[j], (0, 0, L["PLATE_Z0"])), coord)
    b = getattr(wr(F[j], (0, 0, L["PLATE_Z1"])), coord)
    return a if abs(a - toward) > abs(b - toward) else b


corner_x = far_face("hip_roll", "x", axis_pt("hip_pitch").x)
corner_y = far_face("hip_pitch", "y", axis_pt("hip_roll").y)
ra, rs, rr = L["link_features"](F["hip_roll"], proj(F["hip_roll"], [(corner_x, corner_y, hr.z - 7), (corner_x, corner_y, hr.z + 7)]))
pa, ps, pr = L["link_features"](F["hip_pitch"], proj(F["hip_pitch"], [(corner_x, corner_y, hp.z - 7), (corner_x, corner_y, hp.z + 7)]))
lb = finish("l_bracket", L["build_part"](f"{prefix}l_bracket", ra + pa, rs + ps, coll, mat_white, rr + pr))

# ------------------------------------------------------------------ yaw_connector = hip-yaw moving plate + hip-roll cap
hy = axis_pt("hip_yaw")
ya, ys, yr = L["link_features"](F["hip_yaw"], proj(F["hip_yaw"], [(wr(F["hip_roll"], (0, 0, 0)).x + L["CAP_T"], hr.y - 7, 0),
                                                                   (wr(F["hip_roll"], (0, 0, 0)).x + L["CAP_T"], hr.y + 7, 0)]))
rca, rcs = L["cap_features"](F["hip_roll"])
# trim: nothing above this part's own yaw plate top except the bearing boss (the hip-roll cap's
# bearing ring would otherwise rise into the body plate's level and rub it as the hip yaws)
trim = L["add_annulus"](L["new_bm"](), 9.2, 45.0, -20.0, L["PLATE_Z0"], F["hip_yaw"])
yc = finish("yaw_connector", L["build_part"](f"{prefix}yaw_connector", ya + rca, ys + rcs + [trim], coll, mat_dark, yr))

# ------------------------------------------------------------------ hip_yaw_support (body) = both hip-yaw caps
if args.get("with_support"):
    other = "right" if SIDE == "left" else "left"
    Fo = bpy.data.objects[f"scs_{other}_hip_yaw"].matrix_world.copy()
    c1, s1 = L["cap_features"](F["hip_yaw"], include_plate=False)
    c2, s2 = L["cap_features"](Fo, include_plate=False)
    cs = [wr(M, (x, y, 0)) for M in (F["hip_yaw"], Fo) for x in L["EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])]
    sp = plate(F["hip_yaw"], 0.0, L["CAP_T"], proj(F["hip_yaw"], cs))
    finish("hip_yaw_support", L["build_part"]("hip_yaw_support", [sp] + c1 + c2, s1 + s2, coll, mat_dark))

print(f"{SIDE} leg: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
