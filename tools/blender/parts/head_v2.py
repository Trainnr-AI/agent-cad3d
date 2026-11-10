"""XGO-Duck head joints v2 (exact SCS0009): the yaw block between the head-pitch U and the head.
Runs inside Blender after the servos are placed (scs_head_yaw, scs_head_roll roots at the zero pose).

  yaw_block   ONE part that turns with head yaw:
              - type-C stator for the head-yaw servo (shaft down): cap plate with the 6702ZZ seat under the
                tower, pads up to the ears, saddle walls. The U's top plate (neck_v2) is its rotor.
              - type-C rotor for the head-roll servo (shaft forward, the servo sits in the head): boss in the
                roll bearing, trimmed-horn pocket, clamp screw. A driver channel through the yaw saddle walls
                lets the roll clamp screw be reached before the yaw servo goes in.
Head roll is a CAP joint here (not a fork): ±25 deg, a 40 g head; one bearing, no rear plate behind the head.
The head-side holder of the roll servo (cap in the head core) is built with the head.

ARGS = {"robot": "duck_scs0009", "collection": "head_v2", "joint_lib": "/abs/parts/joint_v2.py",
        "base_lib": "/abs/parts/joint_orig_style.py", "dims": "/abs/parts/scs0009_dims.py", "hardware": True}
"""
import math

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
R = args.get("robot", "duck_scs0009")
COLL = args.get("collection", "head_v2")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in list(coll.objects):
    bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()
Fy = bpy.data.objects["scs_head_yaw"].matrix_world.copy()
Fr = bpy.data.objects["scs_head_roll"].matrix_world.copy()
joint = {o["joint"]: o for o in bpy.data.collections[R].objects if o.type == "EMPTY" and "joint" in o and o.name.startswith(R + ":")}
mat_block = L["material"]("head_block", (0.25, 0.6, 0.6, 1))


def fr(M, p_mm):
    return (M.inverted() @ (Vector(p_mm) * MM)) / MM


def wr(M, p_mm):
    return (M @ (Vector(p_mm) * MM)) / MM


def proj(M, pts_world_mm):
    return [tuple(fr(M, p)[:2]) for p in pts_world_mm]


report = []


def finish(name, part):
    ok, vol = L["manifold"](part)
    part["mass_g_solid_PLA"] = round(vol * 1.24, 2)
    part["manifold"] = ok
    n = L["shells"](part)
    report.append(f"{name}: {vol * 1.24:.1f} g{'' if ok else ' NOT MANIFOLD'}{'' if n == 1 else f' {n} SEPARATE PIECES'}")
    return part


# ------------------------------------------------------------------ yaw block
HW = 7.0                                                           # half-width of the strip that joins the two plates
roll_axis = wr(Fr, (0, 0, 0))
# the corner where the two plates meet: the roll plate's far face (Fr z = PLATE_Z1) at the yaw cap plate's level
xa, xb = wr(Fr, (0, 0, L["PLATE_Z0"])).x, wr(Fr, (0, 0, L["PLATE_Z1"])).x      # roll plate's two faces (world x)
za, zb = wr(Fy, (0, 0, 0)).z, wr(Fy, (0, 0, L["CAP_T"])).z                      # yaw cap plate's two faces (world z)
z_far = za if abs(za - roll_axis.z) > abs(zb - roll_axis.z) else zb             # the face farther from the roll axis
x_far = xa if abs(xa - wr(Fy, (0, 0, 0)).x) > abs(xb - wr(Fy, (0, 0, 0)).x) else xb   # the face farther from the yaw servo
# yaw cap plate, reaching to the roll plate's far face
ca, cs = L["cap_features"](Fy, include_plate=False, saddle=True)
ring = [(L["CAP_RING_R"] * math.cos(2 * math.pi * i / 48), L["CAP_RING_R"] * math.sin(2 * math.pi * i / 48)) for i in range(48)]
rect = [(x, y) for x in L["J_EAR_X"] for y in (-L["CAP_HW"], L["CAP_HW"])]
reach = proj(Fy, [(x_far, roll_axis.y + s * HW, 0) for s in (-1, 1)])
cap_plate = L["add_hull"](L["new_bm"](), [(x, y, z) for x, y in ring + rect + reach for z in (0.0, L["CAP_T"])], Fy)
# roll rotor plate, reaching down to the yaw cap plate's far face
la, ls, lr = L["link_features"](Fr, proj(Fr, [(0, roll_axis.y + s * HW, z_far) for s in (-1, 1)]))
# driver channel for the roll clamp screw: along the roll axis, through whatever of the yaw cradle is in the way
driver = L["add_cyl"](L["new_bm"](), 0, 0, L["PLATE_Z1"] + 0.5, L["PLATE_Z1"] + 40.0, 4.6, 22, Fr)
block = finish("yaw_block", L["build_part"]("yaw_block", [cap_plate] + ca + la, cs + ls + [driver], coll, mat_block, lr))

# ------------------------------------------------------------------ hardware + parenting
e_yaw, e_roll = joint["head_yaw"], joint["head_roll"]
pairs = [(block, e_yaw)]
if args.get("hardware", True):
    for o in L["hardware_C"]("head_roll_", Fr, coll):
        pairs.append((o, e_yaw if "screw_M2x6" in o.name and "ear" not in o.name else e_roll))   # bearing + ear screws stay with the head
for part, body in pairs:
    mw = part.matrix_world.copy()
    part.parent = body
    part.matrix_world = mw
old = bpy.data.objects.get(f"{R}:head_yaw")                       # old scaled shell: replaced
if old:
    old.hide_set(True)
    old.hide_render = True
print("head v2: " + " | ".join(report))
for line in L["DEBUG"]:
    print("  !", line)
