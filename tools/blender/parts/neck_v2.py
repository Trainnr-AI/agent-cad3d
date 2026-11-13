"""XGO-Duck neck, joints v2 (exact SCS0009). Runs inside Blender after the servos are placed
(scs_neck_pitch / scs_head_pitch / scs_head_yaw roots, robot at its zero pose).

  neck_cradle      type-F stator x2: one 4 mm floor under the neck-pitch and head-pitch servos (flat
                   bottoms), captive M3 nut under each output axis, ear pads (M2 kit screws), cable
                   channels at the shaft ends. No saddle walls (the two servos sit 2 mm apart).
  body_neck_mount  type-F rotor, fixed to the body: full-cross horn plate + web in the sector the
                   neck never sweeps; slides along the shaft onto the horn.
  body_neck_bearing_plate   separate: 693ZZ on the M3x8 axle into the cradle's nut; 2x M2x6 pan-head self-tappers (counterbored) into the web.
  head_pitch_u     type-F rotor at head pitch (full-cross horn plate) whose top plate is the head-yaw
                   type-C rotor (Ø15.95 boss into the 16x22x4 in the yaw block, trimmed horn pocket).
  head_pitch_bearing_plate  separate: 693ZZ; 2x M2x6 (counterbored) down through the U's lowered far corner (head rework
                   2026-11-13: the U is trimmed so the head can roll +-10 deg at every yaw - see U_ROLL).

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
mid = (Vector((0, 0, 0)) + Vector((pH.x, pH.y, 0))) / 2
for sy in (-4.4, 4.4):                                   # zip-tie holes: the hub sits on the floor's back between them
    subs.append(L["add_cyl"](L["new_bm"](), mid.x, mid.y + sy, FLOOR_Z[0] - 1.0, FLOOR_Z[1] + 1.0, 3.0, 14, Fn))
cradle = finish("neck_cradle", L["build_part"]("neck_cradle", adds, subs, coll, mat_cradle))

# =========================================================== body neck mount (fixed to the body)
e_n = joint["neck_pitch"]
neck_ray = (Fn.to_3x3() @ Vector((1, 0, 0))).normalized()          # along the neck (servo +X)
wd, half = free_dir(e_n, Fn, neck_ray)
if args.get("web_down", True):       # the web straight under the neck axis: the free-sector middle points at the hip-yaw servos
    wd = Fn.inverted().to_3x3() @ Vector((0, 0, -1))
    wd.z = 0
    wd.normalize()
t = Vector((-wd.y, wd.x, 0))
web_xy = [tuple((wd * r + t * s * WEB_HW)[:2]) for r in WEB_R for s in (-1, 1)]
arm_reach = [tuple((wd * r + t * s * (WEB_HW - 0.4))[:2]) for r in (WEB_R[0] + 0.4, WEB_R[1] - 0.4) for s in (-1, 1)]
web = L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in web_xy for z in (BP_Z[1], ARM[1] - 0.05)], Fn)
web_screws = [tuple((wd * sum(WEB_R) / 2 + t * s * 3.5)[:2]) for s in (-1, 1)]
a_add, a_sub, a_rib = L["horn_plate_features"](Fn, arm_reach, angle_deg=45.0)
a_sub += L["screw_holes"](Fn, web_screws, BP_Z[1], 0.0, 7.0, +1, csk=False)           # pilots up into the web
mount = None
if not args.get("mount_in_body", True):   # default: the mount (horn plate + web) is part of body_frame (parts/body_v2.py)
    mount = finish("body_neck_mount", L["build_part"]("body_neck_mount", a_add + [web], a_sub, coll, mat_mount, a_rib))
b_add, b_sub = L["bearing_plate_features"](Fn, web_xy)
HEAD_SEAT = 2.0                                                     # plastic left under a pan head: M2x6 then bites 4 mm into the next part
b_sub += L["screw_holes"](Fn, web_screws, BP_Z[0], BP_Z[1] - BP_Z[0], 0.0, +1, csk=False, cbore=BP_Z[1] - BP_Z[0] - HEAD_SEAT)
mount_plate = finish("body_neck_bearing_plate", L["build_part"]("body_neck_bearing_plate", b_add, b_sub, coll, mat_mount))
print(f"neck web: free sector half-width {180 - half:.0f} deg each side of the web direction")

# =========================================================== head-pitch U (moves with the head)
# Fy's +Z points DOWN (yaw output faces down) and Fh's +x points down the neck: PLATE_Z0 is the
# top plate's upper face (more negative Fh x), PLATE_Z1 its underside.
top_x = fr(Fh, wr(Fy, (0, 0, L["PLATE_Z0"]))).x
bot_x = fr(Fh, wr(Fy, (0, 0, L["PLATE_Z1"]))).x
assert top_x < bot_x
BLOCK = (BP_Z[0] - 3.0, BP_Z[1])                                    # bearing-plate block thickened outward by 3 mm
# ---- head rework 2026-11-13 (user: head roll +-10 deg at EVERY head yaw, head stays put). Yawed, the U's top-plate corners sit
# under the head's floor 21-32 mm from the yaw axis and roll dips the floor onto them (r x sin roll). So: (1) the U is trimmed by a
# solid of revolution about the yaw axis = everything the head-side parts sweep (roll +-U_ROLL, every yaw), less U_CLEAR; (2) the
# far corner over the bearing plate is lowered: the U takes the bearing plate's old 5 mm top block (DROP) and the bearing plate
# starts DROP lower; its 2 screws go down through the U's lowered corner (HEAD_SEAT under the head) into the plate's new block.
U_ROLL = args.get("u_roll_clear_deg", 0.0)                          # 0 (default): the U as before. 11: trim + lowered corner - NOT USABLE: the U then cannot slide over the head-pitch servo onto its horn (2026-11-13)
U_CLEAR, DROP, U_R0 = 0.6, (5.0 if U_ROLL else 0.0), 15.0
h_add, h_sub, h_rib = L["horn_plate_features"](Fh, [(bot_x - 1.0, -ARM_R), (bot_x - 1.0, ARM_R)], angle_deg=45.0)
corners = [wr(Fh, (top_x, y, z)) for y in (-ARM_R, ARM_R) for z in (ARM[1], BLOCK[0])]
y_add, y_sub, y_rib = L["link_features"](Fy, [tuple(fr(Fy, p)[:2]) for p in corners])
blk_z = (BLOCK[0] + BLOCK[1]) / 2
Ms = [Fh @ Matrix.Translation(Vector((top_x, yy, blk_z)) * MM) @ Matrix.Rotation(math.radians(90), 4, "Y") for yy in (-6.0, 6.0)]
top_t = bot_x - top_x
IFACE = top_t + DROP                                                # U / bearing plate interface, below the screws' frame origin (top face)
h_screws = [c for M in Ms for c in L["screw_holes"](M, [(0, 0)], 0.0, IFACE, 0.0, +1, csk=False, cbore=IFACE - HEAD_SEAT)]
u_cut = None
if U_ROLL:
    import numpy as np
    import bmesh as _bm

    def _body(o):
        while o is not None and not o.name.startswith(R + ":"):
            o = o.parent
        return o
    head_body = bpy.data.objects[R + ":jaw_soft"]                   # the body the head roll turns
    yo = wr(Fy, (0, 0, 0))
    Myaw = Matrix.Translation(yo * MM) @ (Fy.to_3x3() @ Matrix(((1, 0, 0), (0, -1, 0), (0, 0, -1)))).to_4x4()   # z UP the yaw axis
    Mi = Myaw.inverted()
    rax = (Myaw.inverted() @ joint["head_roll"].matrix_world.translation) / MM                       # roll axis point, yaw frame
    rdir = (Mi.to_3x3() @ (joint["head_roll"].matrix_world.to_3x3() @ Vector(joint["head_roll"]["joint_axis"]))).normalized()
    pts = []
    for o in bpy.context.view_layer.objects:
        if o.type != "MESH" or _body(o) != head_body or ".x" in o.name or o.name.startswith("_") or not o.visible_get():
            continue
        b = _bm.new()
        b.from_mesh(o.data)
        _bm.ops.triangulate(b, faces=b.faces[:])
        T = Matrix.Scale(1000, 4) @ Mi @ o.matrix_world
        for f in b.faces:
            a_, b_, c_ = (T @ v.co for v in f.verts)
            n_ = max(1, int(max((b_ - a_).length, (c_ - a_).length, (c_ - b_).length) / 0.6))
            for i in range(n_ + 1):
                for j in range(n_ + 1 - i):
                    pts.append(a_ + (b_ - a_) * (i / n_) + (c_ - a_) * (j / n_))
        b.free()
    if not pts:
        raise RuntimeError("U trim: no head-side parts in the scene (build the head once with u_roll_clear_deg 0, then again)")
    P = np.array([p[:] for p in pts])
    rb = 0.5
    zlow = np.full(int(60 / rb) + 1, 1e9)
    for ang in np.arange(-U_ROLL, U_ROLL + 0.01, 0.5):
        Rm = Matrix.Rotation(math.radians(ang), 3, rdir)
        Q = (P - np.array(rax[:])) @ np.array(Rm).T + np.array(rax[:])
        r = np.hypot(Q[:, 0], Q[:, 1])
        k = np.clip((r / rb).astype(int), 0, len(zlow) - 1)
        np.minimum.at(zlow, k, Q[:, 2])
    zs = np.array([zlow[max(0, i - 2):i + 3].min() for i in range(len(zlow))])     # +-1 mm in radius: no gap between bins
    TOP = 40.0
    prof = [(U_R0, TOP)] + [(i * rb, min(TOP - 0.5, float(zs[i]) - U_CLEAR)) for i in range(int(U_R0 / rb), int(45 / rb) + 1)] + [(45.0, TOP)]
    u_cut = lambda: L["combine_cutters"]([L["_revolve"](prof, -180.0, 172.0, Myaw), L["_revolve"](prof, 166.0, 196.0, Myaw)])
    report.append("U trim: allowed top above the yaw origin r16 %.1f, r20 %.1f, r24 %.1f, r30 %.1f mm" % tuple(dict(prof)[r] for r in (16.0, 20.0, 24.0, 30.0)))
u_adds = h_add + y_add
if DROP:
    u_adds = u_adds + [L["add_box"](L["new_bm"](), bot_x - 0.6, bot_x + DROP, -ARM_R, ARM_R, BLOCK[0], BLOCK[1], Fh)]   # the far corner, lowered
u = finish("head_pitch_u", L["build_part"]("head_pitch_u", u_adds, h_sub + y_sub + h_screws + ([u_cut()] if u_cut else []), coll, mat_u, h_rib + y_rib))
blk = L["add_box"](L["new_bm"](), bot_x + DROP, bot_x + DROP + 5.0, -ARM_R, ARM_R, BLOCK[0], BLOCK[1], Fh)
p_add, p_sub = L["bearing_plate_features"](Fh, [(bot_x + DROP + 0.3, -ARM_R + 0.3), (bot_x + DROP + 0.3, ARM_R - 0.3)])   # corners inside the block: no edge lying on its side face
p_sub += [c for M in Ms for c in L["screw_holes"](M, [(0, 0)], IFACE, 0.0, 4.6 if DROP else 5.5, +1, csk=False)]
if u_cut:
    p_sub.append(u_cut())
hp_plate = finish("head_pitch_bearing_plate", L["build_part"]("head_pitch_bearing_plate", [L["combine_cutters"](p_add + [blk])], p_sub, coll, mat_u))

# =========================================================== hardware dummies + parenting
hw = []
if args.get("hardware", True):
    hw.append((L["pan_screw_dummy"]("neck_plate_screws_M2x6st", [(Fn, x, y, BP_Z[1] - HEAD_SEAT, +1) for x, y in web_screws], coll), e_n.parent))
    hw.append((L["pan_screw_dummy"]("head_pitch_plate_screws_M2x6st", [(M, 0, 0, IFACE - HEAD_SEAT, +1) for M in Ms], coll), joint["head_pitch"]))
    hw += [(o, e_n.parent) for o in L["hardware_F"]("neck_", Fn, coll)]
    hw += [(o, joint["head_pitch"]) for o in L["hardware_F"]("head_pitch_", Fh, coll)]
    hw += [(o, joint["head_yaw"].parent) for o in L["hardware_C"]("head_yaw_", Fy, coll)]
# the neck-pitch nut and M3 axle belong to the cradle (neck), the 693 + horn screw to the body side
for o in list(coll.objects):
    # things fixed to the cradle (neck): both servos' ear screws, both nuts and M3 axles
    if o.name in ("neck_nut_M3", "neck_screw_M3x8", "neck_ear_screws_M2x6st", "head_pitch_nut_M3", "head_pitch_screw_M3x8", "head_pitch_ear_screws_M2x6st"):
        hw = [(p, b) for p, b in hw if p is not o] + [(o, e_n)]
    # head yaw: the servo, its ear screws and the bearing sit in the yaw block; the horn screw turns with the U
    if o.name in ("head_yaw_ear_screws_M2x6st", f"head_yaw_brg_{L['BRG_NAME']}"):
        hw = [(p, b) for p, b in hw if p is not o] + [(o, joint["head_yaw"])]
for part, body in ((cradle, e_n), (mount, e_n.parent), (mount_plate, e_n.parent), (u, joint["head_pitch"]),
                   (hp_plate, joint["head_pitch"])) + tuple(hw):
    if part is None:
        continue
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
