"""XGO-Duck leg, joints v2 (exact SCS0009): thigh, shin, foot + foot bearing plate for one side.
Runs inside Blender after the servos are placed (scs_<side>_<joint> roots at the zero pose).

  thigh   type-C stator x2 (hip pitch + knee caps with saddle walls, one plate + bridge)
  shin    type-C rotor at the knee (trimmed horn, 16x22x4) + type-F stator for the ankle servo
          (floor with captive M3 nut, pads, saddle walls, cable channel)
  foot    type-F rotor: full-cross horn plate (outboard) + sole
  foot_bearing_plate   693ZZ on an M3x8 axle (inboard), 2x M2 csk into the sole
Hip parts (yaw connector, L-bracket, body plate) are left to leg_orig_style.py for now.

ARGS = {"side": "left", "collection": "leg_v2", "joint_lib": "/abs/parts/joint_v2.py",
        "base_lib": "/abs/parts/joint_orig_style.py", "dims": "/abs/parts/scs0009_dims.py",
        "foot_mesh": "duck_scs0009:left_ankle", "hardware": True, "jig": True}
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
SIDE = args.get("side", "left")
COLL = args.get("collection", "leg_v2")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
prefix = f"{SIDE}_"
for o in [o for o in coll.objects if o.name.startswith(prefix) or o.name.startswith("horn_trim_jig")]:
    bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()

F = {j: bpy.data.objects[f"scs_{SIDE}_{j}"].matrix_world.copy() for j in ("hip_pitch", "knee", "ankle")}
mat_dark = L["material"]("leg_dark", (0.16, 0.18, 0.24, 1))
mat_white = L["material"]("leg_white", (0.93, 0.93, 0.9, 1))
mat_foot = L["material"]("leg_foot", (0.98, 0.72, 0.15, 1))


def fr(M, p_world_mm):
    return (M.inverted() @ (Vector(p_world_mm) * MM)) / MM


def wr(M, p_frame_mm):
    return (M @ (Vector(p_frame_mm) * MM)) / MM


def plate(M, z0, z1, outline_frame_mm):
    return L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in outline_frame_mm for z in (z0, z1)], M)


def disc_pts(cx, cy, r, n=48):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def proj(M, pts_world_mm):
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


# ------------------------------------------------------------------ thigh = hip-pitch cap + knee cap (one plate + bridge)
ca1, cs1 = L["cap_features"](F["hip_pitch"], include_plate=False)
ca2, cs2 = L["cap_features"](F["knee"], include_plate=False)


def cap_rect(j):
    ring = [(L["CAP_RING_R"] * math.cos(2 * math.pi * i / 48), L["CAP_RING_R"] * math.sin(2 * math.pi * i / 48), 0) for i in range(48)]
    return [wr(F[j], (x, y, 0)) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])] + [wr(F[j], p) for p in ring]


hp_plate = plate(F["hip_pitch"], 0.0, L["CAP_T"], proj(F["hip_pitch"], cap_rect("hip_pitch")))
kn_plate = plate(F["hip_pitch"], 0.02, L["CAP_T"] - 0.02, proj(F["hip_pitch"], cap_rect("knee")))
kn_top = wr(F["knee"], (L["J_EAR_X"][1], 0, 0))
hp_far = wr(F["hip_pitch"], (L["J_EAR_X"][1], 0, 0))
d = (hp_far - kn_top).normalized()
nrm = d.cross(wr(F["hip_pitch"], (0, 0, 1)) - wr(F["hip_pitch"], (0, 0, 0))).normalized()
bridge_pts = [kn_top + nrm * 4.5, kn_top - nrm * 4.5, hp_far + nrm * 4.5, hp_far - nrm * 4.5]
bridge = plate(F["hip_pitch"], 0.04, L["CAP_T"] - 0.04, proj(F["hip_pitch"], bridge_pts))
thigh = finish("thigh", L["build_part"](f"{prefix}thigh", [hp_plate, kn_plate, bridge] + ca1 + ca2, cs1 + cs2, coll, mat_dark))

# ------------------------------------------------------------------ shin = ONE 4 mm plate: knee rotor (type C) at the top,
# ankle floor (type F) at the bottom. The ankle servo is slid along its axis (snap_servo_axes.py) so its
# floor lies in the knee plate's plane: the inboard face is flat (print bed), everything grows outboard.
kn, an = axis_pt("knee"), axis_pt("ankle")
kp0, kp1 = wr(F["knee"], (0, 0, L["PLATE_Z0"])).y, wr(F["knee"], (0, 0, L["PLATE_Z1"])).y
fl0, fl1 = wr(F["ankle"], (0, 0, L["FLOOR_Z"][1])).y, wr(F["ankle"], (0, 0, L["FLOOR_Z"][0])).y
if abs(kp0 - fl0) > 0.05 or abs(kp1 - fl1) > 0.05:
    raise RuntimeError(f"shin plates not coplanar: knee plate y {kp0:.2f}..{kp1:.2f}, ankle floor y {fl0:.2f}..{fl1:.2f} "
                       f"-> slide the ankle servo by {kp0 - fl0:+.2f} mm (snap_servo_axes.py axial_mm)")
# ankle cradle footprint (ankle frame x: ear tip to ear tip; y: cap half-width) in the knee frame's plate plane
cradle = [wr(F["ankle"], (x, y, L["FLOOR_Z"][1])) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])]
top_edge = max(p.z for p in cradle)                                  # upper edge of the cradle rectangle (world z)
strip = [(kn.x - 7, kn.y, top_edge - 1.5), (kn.x + 7, kn.y, top_edge - 1.5)]   # 1.5 mm into the cradle plate: fused
la, ls, lr = L["link_features"](F["knee"], proj(F["knee"], strip))   # knee ring + 14 mm strip down to the cradle (20 clips the hip-pitch cradle at knee +90)
cradle_plate = plate(F["knee"], L["PLATE_Z0"] + 0.02, L["PLATE_Z1"] - 0.02, proj(F["knee"], cradle))
fa, fs = L["floor_features"](F["ankle"], include_floor=False, saddle=args.get("ankle_saddle", True), cable=args.get("cable", True))
shin = finish("shin", L["build_part"](f"{prefix}shin", la + [cradle_plate] + fa, ls + fs, coll, mat_white, lr))

# ------------------------------------------------------------------ foot = full-cross horn plate (outboard) + sole
fm = bpy.data.objects[args["foot_mesh"]]
fv = [fm.matrix_world @ v.co / MM for v in fm.data.vertices]
zmin = min(v.z for v in fv)
sole_v = [v for v in fv if v.z < zmin + 4.0]
sx0, sx1 = min(v.x for v in sole_v), max(v.x for v in sole_v)          # fore-aft: the original footprint
# lateral: 1 mm beyond the two plates (bearing plate inboard, horn plate outboard)
BP_Z0, BP_Z1 = L["BP_INNER_Z"] - L["BP_T"], L["BP_INNER_Z"]
ys = [wr(F["ankle"], (0, 0, z)).y for z in (BP_Z0, L["ARM_Z"][1])]
sy0, sy1 = min(ys) - 1.0, max(ys) + 1.0
SOLE_T = 5.0
sole = box_w(sx0, sx1, sy0, sy1, zmin, zmin + SOLE_T)
top = zmin + SOLE_T
reach = [(an.x - 9, an.y, top - 0.5), (an.x + 9, an.y, top - 0.5)]
ha, hs, hr = L["horn_plate_features"](F["ankle"], proj(F["ankle"], reach), angle_deg=45.0)
# bearing plate flange screws (2x M2 csk down into the sole): pilot holes in the sole
y_scr = wr(F["ankle"], (0, 0, BP_Z0 - 3.0)).y
M_sole = [Matrix.Translation(Vector((an.x + dx, y_scr, top)) * MM) for dx in (-5.5, 5.5)]
hs += [c for M in M_sole for c in L["screw_holes"](M, [(0, 0)], 0.0, 0.0, 4.0, -1, csk=False)]
foot = finish("foot", L["build_part"](f"{prefix}foot", [sole] + ha, hs, coll, mat_foot, hr))

# bearing plate (inboard): 693ZZ seat; flange onto the sole
reach_b = [(an.x - 9, an.y, top + 0.3), (an.x + 9, an.y, top + 0.3)]
ba, bs = L["bearing_plate_features"](F["ankle"], proj(F["ankle"], reach_b))
y_a, y_b = wr(F["ankle"], (0, 0, BP_Z0 - 6.0)).y, wr(F["ankle"], (0, 0, BP_Z1)).y
flange = box_w(an.x - 9, an.x + 9, min(y_a, y_b), max(y_a, y_b), top, top + 2.5)
M_scr = [Matrix.Translation(Vector((an.x + dx, y_scr, top + 2.5)) * MM) for dx in (-5.5, 5.5)]
bs += [c for M in M_scr for c in L["screw_holes"](M, [(0, 0)], 0.0, 2.5, 0.0, -1)]
foot_bp = finish("foot_bearing_plate", L["build_part"](f"{prefix}foot_bearing_plate", ba + [flange], bs, coll, mat_foot))

# ------------------------------------------------------------------ hips (type C, as v1 but with the v2 features)
if args.get("hips", True):
    for j in ("hip_yaw", "hip_roll"):
        F[j] = bpy.data.objects[f"scs_{SIDE}_{j}"].matrix_world.copy()
    hr, hp, hy = axis_pt("hip_roll"), axis_pt("hip_pitch"), axis_pt("hip_yaw")

    # l_bracket = hip-roll rotor plate + hip-pitch rotor plate meeting at a corner
    def far_face(j, coord, toward):
        a = getattr(wr(F[j], (0, 0, L["PLATE_Z0"])), coord)
        b = getattr(wr(F[j], (0, 0, L["PLATE_Z1"])), coord)
        return a if abs(a - toward) > abs(b - toward) else b

    corner_x = far_face("hip_roll", "x", hp.x)
    corner_y = far_face("hip_pitch", "y", hr.y)
    ra, rs, rr = L["link_features"](F["hip_roll"], proj(F["hip_roll"], [(corner_x, corner_y, hr.z - 7), (corner_x, corner_y, hr.z + 7)]))
    pa, ps, pr = L["link_features"](F["hip_pitch"], proj(F["hip_pitch"], [(corner_x, corner_y, hp.z - 7), (corner_x, corner_y, hp.z + 7)]))
    finish("l_bracket", L["build_part"](f"{prefix}l_bracket", ra + pa, rs + ps, coll, mat_white, rr + pr))

    # yaw_connector = hip-yaw rotor plate + hip-roll cap (saddle walls optional: they hang beside the yaw servo)
    ya, ys, yr = L["link_features"](F["hip_yaw"], proj(F["hip_yaw"], [(wr(F["hip_roll"], (0, 0, 0)).x + L["CAP_T"], hr.y - 7, 0),
                                                                       (wr(F["hip_roll"], (0, 0, 0)).x + L["CAP_T"], hr.y + 7, 0)]))
    rca, rcs = L["cap_features"](F["hip_roll"], saddle=args.get("roll_saddle", True))
    trim = L["add_annulus"](L["new_bm"](), 9.2, 45.0, -20.0, L["PLATE_Z0"] + 0.01, F["hip_yaw"])   # nothing above the yaw plate but the boss (0.01 into the plate top: no shared plane)
    finish("yaw_connector", L["build_part"](f"{prefix}yaw_connector", ya + rca, ys + rcs + [trim], coll, mat_dark, yr))

    # hip_yaw_support (body plate) = both hip-yaw caps
    if args.get("with_support"):
        other = "right" if SIDE == "left" else "left"
        Fo = bpy.data.objects[f"scs_{other}_hip_yaw"].matrix_world.copy()
        for o in [o for o in coll.objects if o.name.startswith("hip_yaw_support")]:
            bpy.data.objects.remove(o, do_unlink=True)
        c1, s1 = L["cap_features"](F["hip_yaw"], include_plate=False, saddle=args.get("yaw_saddle", True))
        c2, s2 = L["cap_features"](Fo, include_plate=False, saddle=args.get("yaw_saddle", True))
        cs = [wr(M, (x, y, 0)) for M in (F["hip_yaw"], Fo) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])]
        sp = plate(F["hip_yaw"], 0.0, L["CAP_T"], proj(F["hip_yaw"], cs))
        finish("hip_yaw_support", L["build_part"]("hip_yaw_support", [sp] + c1 + c2, s1 + s2, coll, mat_dark))

if args.get("hardware", True):
    L["hardware_C"](f"{prefix}knee_", F["knee"], coll)
    L["hardware_C"](f"{prefix}hip_pitch_", F["hip_pitch"], coll)
    L["hardware_F"](f"{prefix}ankle_", F["ankle"], coll)
    if args.get("hips", True):
        L["hardware_C"](f"{prefix}hip_roll_", F["hip_roll"], coll)
        L["hardware_C"](f"{prefix}hip_yaw_", F["hip_yaw"], coll)
if args.get("jig", True):
    Mj = Matrix.Translation(Vector(args.get("jig_location", (0.30, 0.30, 0.0))))
    L["trim_jig"]("horn_trim_jig_r6", coll, Mj)

print(f"{SIDE} leg v2: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
