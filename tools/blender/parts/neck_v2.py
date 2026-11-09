"""XGO-Duck neck, joints v2 (exact SCS0009). Runs inside Blender after the servos are placed
(scs_neck_pitch / scs_head_pitch / scs_head_yaw roots, robot at its zero pose).

  neck_cradle      type-F stator x2: one 4 mm floor under the neck-pitch and head-pitch servos (flat
                   bottoms), captive M3 nut under each output axis, ear pads (M2 kit screws), cable
                   channels at the shaft ends. No saddle walls (the two servos sit 2 mm apart).
  body_neck_mount  type-F rotor, fixed to the body: full-cross horn plate + web in the sector the
                   neck never sweeps; slides along the shaft onto the horn.
  body_neck_bearing_plate   separate: 693ZZ on the M3x8 axle into the cradle's nut; 2x M2x6 csk into the web.
  head_pitch_u     type-F rotor at head pitch (full-cross horn plate) whose top plate is the head-yaw
                   type-C rotor (Ø15.95 boss into the 16x22x4 in the yaw block, trimmed horn pocket).
  head_pitch_bearing_plate  separate: 693ZZ; 2x M2x8 csk down through the top plate.

ARGS = {"robot": "duck_scs0009", "collection": "neck_v2", "joint_lib": "/abs/parts/joint_v2.py",
        "base_lib": "/abs/parts/joint_orig_style.py", "dims": "/abs/parts/scs0009_dims.py", "hardware": True}
"""
import math

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "neck_v2")
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

FLOOR_Z = L["FLOOR_Z"]                  # (-29.25, -25.25): 4 mm floor under the servo bottoms
ARM = L["ARM_Z"]                        # (2.6, 6.4) horn plate
ARM_R = L["ARM_R_FULL"]                 # 13.94: full cross
BP_Z = (L["BP_INNER_Z"] - L["BP_T"], L["BP_INNER_Z"])   # bearing plate z range
PAD_Y = 4.5                             # narrower pads: a wider floor edge clips the head-pitch U at the end of head pitch
WEB_R = (ARM_R + 0.5, ARM_R + 6.5)
WEB_HW = 6.0

mat_cradle = L["material"]("neck_cradle", (0.93, 0.93, 0.9, 1))
mat_mount = L["material"]("neck_mount", (0.16, 0.18, 0.24, 1))
mat_u = L["material"]("neck_u", (0.98, 0.72, 0.15, 1))
report = []


def fr(M, p_mm):
    return (M.inverted() @ (Vector(p_mm) * MM)) / MM


def wr(M, p_mm):
    return (M @ (Vector(p_mm) * MM)) / MM


def disc(cx, cy, r, n=48):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def plate(M, z0, z1, pts):
    return L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in pts for z in (z0, z1)], M)


def finish(name, part):
    ok, vol = L["manifold"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    part["manifold"] = ok
    n = L["shells"](part)
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    return part


def free_dir(e, M, ray_world):
    """Unit vector (frame M XY) into the sector that `ray_world` (rotating with the joint's child) never sweeps."""
    a = (e.matrix_world.to_3x3() @ Vector(e["joint_axis"])).normalized()
    lo, hi = e["joint_range"]
    w = Matrix.Rotation((lo + hi) / 2 + math.pi, 3, a) @ ray_world
    d = M.inverted().to_3x3() @ w
    d.z = 0
    return d.normalized(), math.degrees((hi - lo) / 2)


# =========================================================== neck cradle (moves with the neck)
pH = fr(Fn, wr(Fh, (0, 0, 0)))                          # head-pitch axis in the neck-pitch frame
foot = []
for M in (Fn, Fh):
    for x in L["J_EAR_X"]:
        for y in (-PAD_Y, PAD_Y):
            foot.append(tuple(fr(Fn, wr(M, (x, y, 0)))[:2]))
floor = plate(Fn, FLOOR_Z[0], FLOOR_Z[1], disc(0, 0, L["CAP_HW"]) + disc(pH.x, pH.y, L["CAP_HW"]) + foot)
adds, subs = [floor], []
for M in (Fn, Fh):
    a, s = L["floor_features"](M, include_floor=False, saddle=False, pad_hw=PAD_Y)
    adds += a
    subs += s
cradle = finish("neck_cradle", L["build_part"]("neck_cradle", adds, subs, coll, mat_cradle))

# =========================================================== body neck mount (fixed to the body)
e_n = joint["neck_pitch"]
neck_ray = (Fn.to_3x3() @ Vector((1, 0, 0))).normalized()          # along the neck (servo +X)
wd, half = free_dir(e_n, Fn, neck_ray)
t = Vector((-wd.y, wd.x, 0))
web_xy = [tuple((wd * r + t * s * WEB_HW)[:2]) for r in WEB_R for s in (-1, 1)]
arm_reach = [tuple((wd * r + t * s * (WEB_HW - 0.4))[:2]) for r in (WEB_R[0] + 0.4, WEB_R[1] - 0.4) for s in (-1, 1)]
web = L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in web_xy for z in (BP_Z[1], ARM[1] - 0.05)], Fn)
web_screws = [tuple((wd * sum(WEB_R) / 2 + t * s * 3.5)[:2]) for s in (-1, 1)]
a_add, a_sub, a_rib = L["horn_plate_features"](Fn, arm_reach, angle_deg=45.0)
a_sub += L["screw_holes"](Fn, web_screws, BP_Z[1], 0.0, 7.0, +1, csk=False)           # pilots up into the web
mount = finish("body_neck_mount", L["build_part"]("body_neck_mount", a_add + [web], a_sub, coll, mat_mount, a_rib))
b_add, b_sub = L["bearing_plate_features"](Fn, web_xy)
b_sub += L["screw_holes"](Fn, web_screws, BP_Z[0], BP_Z[1] - BP_Z[0], 0.0, +1)
mount_plate = finish("body_neck_bearing_plate", L["build_part"]("body_neck_bearing_plate", b_add, b_sub, coll, mat_mount))
print(f"neck web: free sector half-width {180 - half:.0f} deg each side of the web direction")

# =========================================================== head-pitch U (moves with the head)
# Fy's +Z points DOWN (yaw output faces down) and Fh's +x points down the neck: PLATE_Z0 is the
# top plate's upper face (more negative Fh x), PLATE_Z1 its underside.
top_x = fr(Fh, wr(Fy, (0, 0, L["PLATE_Z0"]))).x
bot_x = fr(Fh, wr(Fy, (0, 0, L["PLATE_Z1"]))).x
assert top_x < bot_x
BLOCK = (BP_Z[0] - 3.0, BP_Z[1])                                    # bearing-plate block thickened outward by 3 mm
h_add, h_sub, h_rib = L["horn_plate_features"](Fh, [(bot_x - 1.0, -ARM_R), (bot_x - 1.0, ARM_R)], angle_deg=45.0)
corners = [wr(Fh, (top_x, y, z)) for y in (-ARM_R, ARM_R) for z in (ARM[1], BLOCK[0])]
y_add, y_sub, y_rib = L["link_features"](Fy, [tuple(fr(Fy, p)[:2]) for p in corners])
blk_z = (BLOCK[0] + BLOCK[1]) / 2
Ms = [Fh @ Matrix.Translation(Vector((top_x, yy, blk_z)) * MM) @ Matrix.Rotation(math.radians(90), 4, "Y") for yy in (-6.0, 6.0)]
top_t = bot_x - top_x
h_screws = [c for M in Ms for c in L["screw_holes"](M, [(0, 0)], 0.0, top_t, 0.0, +1)]
u = finish("head_pitch_u", L["build_part"]("head_pitch_u", h_add + y_add, h_sub + y_sub + h_screws, coll, mat_u, h_rib + y_rib))
blk = L["add_box"](L["new_bm"](), bot_x, bot_x + 5.0, -ARM_R, ARM_R, BLOCK[0], BLOCK[1], Fh)
p_add, p_sub = L["bearing_plate_features"](Fh, [(bot_x + 0.3, -ARM_R), (bot_x + 0.3, ARM_R)])
p_sub += [c for M in Ms for c in L["screw_holes"](M, [(0, 0)], top_t, 0.0, 5.5, +1, csk=False)]
hp_plate = finish("head_pitch_bearing_plate", L["build_part"]("head_pitch_bearing_plate", p_add + [blk], p_sub, coll, mat_u))

# =========================================================== hardware dummies + parenting
hw = []
if args.get("hardware", True):
    hw += [(o, e_n.parent) for o in L["hardware_F"]("neck_", Fn, coll) if "brg" in o.name or "M3" in o.name or "M2x6" in o.name]
    hw += [(o, joint["head_pitch"]) for o in L["hardware_F"]("head_pitch_", Fh, coll)]
    hw += [(o, joint["head_yaw"].parent) for o in L["hardware_C"]("head_yaw_", Fy, coll)]
# the neck-pitch nut and M3 axle belong to the cradle (neck), the 693 + horn screw to the body side
for o in list(coll.objects):
    if o.name in ("neck_nut_M3", "neck_screw_M3x8"):
        hw = [(p, b) for p, b in hw if p is not o] + [(o, e_n)]
for part, body in ((cradle, e_n), (mount, e_n.parent), (mount_plate, e_n.parent), (u, joint["head_pitch"]),
                   (hp_plate, joint["head_pitch"])) + tuple(hw):
    mw = part.matrix_world.copy()
    part.parent = body
    part.matrix_world = mw
for n in ("neck_pitch", "head_pitch"):                              # old scaled shells: replaced
    o = bpy.data.objects.get(f"{R}:{n}")
    if o:
        o.hide_set(True)
        o.hide_render = True
print("neck v2: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
