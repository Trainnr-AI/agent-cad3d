"""Wiring of the duck as cables in place: every servo lead to its hub, the hub-to-hub extension cables, and the
power / signal wires between battery, regulator, servo adapter, Uno Q and balance sensor. Runs inside Blender.

Cables are smooth curves through waypoints; each waypoint belongs to a part ("owner"), so the same wiring can be
drawn on the assembled robot or on an exploded copy of it (ARGS["exploded"]: a collection of '<name>.x' copies,
e.g. from explode_all.py - every waypoint follows its owner's copy). Nothing is checked for collisions here;
the lengths are reported against the kit's cables (lead 150 mm, extension 165 mm).

Bus tree (kit hubs have 3 sockets; the adapter is assumed to offer ONE bus socket - worst case):
  adapter - S1 - S2;  S1 - left chain;  S2 - right chain + head chain
  leg chain:  body hub A (hip yaw) - body hub B (hip roll) - thigh hub A (hip pitch) - thigh hub B (knee, ankle)
  head chain: cradle hub (neck pitch) - cradle hub (head pitch) - U hub (head yaw, head roll)

ARGS = {"kit": "/abs/parts/cable_kit.py", "joint_lib": ..., "base_lib": ..., "dims": ..., "collection": "wiring",
        "exploded": None | "duck_v2_exploded", "scale": 1.0}
"""
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
K = {"ARGS": {"lib": True}, "__name__": "kit"}
exec(open(args["kit"]).read(), K)
L = {"__name__": "joint_v2", "ARGS": {"base_lib": args["base_lib"], "dims": args["dims"]}}
exec(open(args["joint_lib"]).read(), L)
MM = 0.001
EX = bpy.data.collections.get(args["exploded"]) if args.get("exploded") else None
SC = args.get("scale", 1.0)
COLL = args.get("collection", "wiring_exploded" if EX else "wiring")
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
bpy.context.view_layer.update()
(cx0, cx1), (cy0, cy1), (cz0, cz1) = L["J_CABLE"]
m_lead, m_ext = K["WHITE"], K["_mat"]("kit_ext", (0.75, 0.12, 0.1, 1))
m_pwr, m_sig = K["_mat"]("kit_power", (1.0, 0.55, 0.0, 1)), K["_mat"]("kit_signal", (0.2, 0.5, 1.0, 1))


def copy_of(name):
    for o in EX.objects:
        if o.name == name + ".x" or o.name.startswith(name + ".x."):
            return o
    return None


def xf(owner):
    """matrix that carries an assembled-pose world point of `owner` to where that part is drawn"""
    if EX is None:
        return Matrix.Identity(4)
    c = copy_of(owner)
    if c is None:
        return Matrix.Identity(4)
    return c.matrix_world @ bpy.data.objects[owner].matrix_world.inverted()


def W(owner, p):
    return xf(owner) @ Vector(p)


def servo_pt(j, p_frame_mm):
    return bpy.data.objects["scs_" + j].matrix_world @ (Vector(p_frame_mm) * MM)


def exit_pts(j):
    """the lead's first two waypoints: in the slot, and 7 mm straight out of it (both move with the servo)"""
    own = f"scs_{j}_case"
    z = (cz0 + cz1) / 2
    return [(own, servo_pt(j, (cx0, 0, z))), (own, servo_pt(j, (cx0 - 7.0, 0, z)))]


def axis(j):
    return bpy.data.objects["scs_" + j].matrix_world.translation.copy()


hubs = {}


def add_hub(name, owner, M):
    parts = K["hub"]("wire_hub_" + name, xf(owner) @ M, coll)
    hubs[name] = (owner, M.translation.copy())
    return parts


def existing_hub(name, obj):
    o = bpy.data.objects[obj]
    c = sum((o.matrix_world @ Vector(b) for b in o.bound_box), Vector()) / 8
    hubs[name] = (obj, c)


report = []


def cable(name, pts, width, thick, mat, limit=None):
    ob, length = K["ribbon"]("wire_" + name, [W(o, p) for o, p in pts], width * SC, thick * SC, coll, mat)
    if EX is None:
        report.append((name, length, limit))
    return ob


def H(name):
    return hubs[name]


def side_pt(owner, p, d):
    return (owner, Vector(p) + Vector(d) * MM)


# ------------------------------------------------------------------ hubs (13: 6 in the body, 2 per thigh, 2 on the neck cradle, 1 on the U)
for i in range(1, 5):
    existing_hub(f"B{i}", f"hub_{i}")                    # between the hip-yaw servos: left A, left B, right A, right B
existing_hub("S2", "hub_5")                              # splitter 2, also between the hip-yaw servos
existing_hub("S1", "hub_6")                              # splitter 1, in the enclosure's pocket
for s in ("left", "right"):
    a = (bpy.data.objects[f"scs_{s}_hip_pitch"].matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
    mid = (axis(f"{s}_hip_pitch") + axis(f"{s}_knee")) / 2
    for k, name in enumerate(("A", "B")):                 # on the thigh's outer face, between its two servos
        M = Matrix.Translation(mid - a * (0.012 + k * 0.007)) @ a.to_track_quat("Z", "Y").to_matrix().to_4x4()
        add_hub(f"{s[0].upper()}T{name}", f"{s}_thigh", M)
Mn = bpy.data.objects["scs_neck_pitch"].matrix_world
an = (Mn.to_3x3() @ Vector((0, 0, 1))).normalized()
up = (Mn.to_3x3() @ Vector((1, 0, 0))).normalized()
midn = (axis("neck_pitch") + axis("head_pitch")) / 2
for k, name in enumerate(("C1", "C2")):                   # on the neck cradle's back (floor side), between its two servos
    M = Matrix.Translation(midn - an * 0.0335 + up * (0.011 if k else -0.011)) @ an.to_track_quat("Z", "Y").to_matrix().to_4x4()
    add_hub(name, "neck_cradle", M)
ah = (bpy.data.objects["scs_head_pitch"].matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
add_hub("U", "head_pitch_u", Matrix.Translation(axis("head_pitch") + ah * 0.012) @ ah.to_track_quat("Z", "Y").to_matrix().to_4x4())

# ------------------------------------------------------------------ servo leads (150 mm each)
deck_slot = ("body_frame", Vector((0.014, bpy.data.objects["body_frame"].matrix_world.translation.y * 0 + axis("left_hip_yaw").y / 2 + axis("right_hip_yaw").y / 2, axis("left_hip_yaw").z - 0.003)))
for s, (ba, bb) in (("left", ("B1", "B2")), ("right", ("B3", "B4"))):
    T = s[0].upper() + "T"
    cable(f"lead_{s}_hip_yaw", exit_pts(f"{s}_hip_yaw") + [H(ba)], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    cable(f"lead_{s}_hip_roll", exit_pts(f"{s}_hip_roll") + [deck_slot, H(bb)], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    cable(f"lead_{s}_hip_pitch", exit_pts(f"{s}_hip_pitch") + [H(T + "A")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    cable(f"lead_{s}_knee", exit_pts(f"{s}_knee") + [H(T + "B")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    cable(f"lead_{s}_ankle", exit_pts(f"{s}_ankle") + [(f"{s}_thigh", axis(f"{s}_knee")), H(T + "B")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
    cable(f"ext_{s}_body_to_thigh", [H(bb), deck_slot, (f"{s}_yaw_connector", axis(f"{s}_hip_roll")), (f"{s}_l_bracket", axis(f"{s}_hip_pitch")), H(T + "A")], K["EXT_W"], K["EXT_T"], m_ext, 165)
    cable(f"ext_{s}_thigh_A_to_B", [H(T + "A"), H(T + "B")], K["EXT_W"], K["EXT_T"], m_ext, 165)
    cable(f"ext_{s}_body_A_to_B", [H(ba), H(bb)], K["EXT_W"], K["EXT_T"], m_ext, 165)
neck_ax, hp_ax, hy_ax = ("body_frame", axis("neck_pitch")), ("neck_cradle", axis("head_pitch")), ("head_pitch_u", axis("head_yaw"))
cable("lead_neck_pitch", exit_pts("neck_pitch") + [H("C1")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
cable("lead_head_pitch", exit_pts("head_pitch") + [H("C2")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
cable("lead_head_yaw", exit_pts("head_yaw") + [hy_ax, H("U")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
cable("lead_head_roll", exit_pts("head_roll") + [("yaw_block", axis("head_roll")), hy_ax, H("U")], K["LEAD_W"], K["LEAD_T"], m_lead, 150)
cable("ext_cradle_to_U", [H("C2"), hp_ax, H("U")], K["EXT_W"], K["EXT_T"], m_ext, 165)
cable("ext_C1_to_C2", [H("C1"), H("C2")], K["EXT_W"], K["EXT_T"], m_ext, 165)
cable("ext_body_to_cradle", [H("S2"), neck_ax, H("C1")], K["EXT_W"], K["EXT_T"], m_ext, 165)
bay = bpy.data.objects["servo_adapter"]
bc = sum((bay.matrix_world @ Vector(b) for b in bay.bound_box), Vector()) / 8
adapter = ("servo_adapter", bc)
for a, b in (("S1", "B1"), ("S1", "S2"), ("S2", "B3")):
    cable(f"ext_{a}_to_{b}", [H(a), H(b)], K["EXT_W"], K["EXT_T"], m_ext, 165)
cable("ext_adapter_to_S1", [adapter, H("S1")], K["EXT_W"], K["EXT_T"], m_ext, 165)


# ------------------------------------------------------------------ power and signal
def centre(n):
    o = bpy.data.objects[n]
    return (n, sum((o.matrix_world @ Vector(b) for b in o.bound_box), Vector()) / 8)


def top_of(n):
    o = bpy.data.objects[n]
    c = sum((o.matrix_world @ Vector(b) for b in o.bound_box), Vector()) / 8
    return (n, Vector((c.x, c.y, max((o.matrix_world @ Vector(b)).z for b in o.bound_box))))


cable("power_battery_to_regulator", [top_of("battery_3s"), top_of("ubec")], 2.4, 2.4, m_pwr)
cable("power_regulator_to_adapter_6V", [top_of("ubec"), top_of("servo_adapter")], 2.4, 2.4, m_pwr)
cable("power_battery_to_uno_VIN", [top_of("battery_3s"), top_of("uno_q")], 1.8, 1.8, m_pwr)
cable("signal_uno_UART_to_adapter", [top_of("uno_q"), top_of("servo_adapter")], 2.5, 1.0, m_sig)
cable("signal_imu_qwiic_to_uno", [centre("imu"), top_of("uno_q")], 2.0, 1.0, m_sig)
if EX is None:
    print(f"{COLL}: {len(hubs)} hubs, {len(report)} cables")
    for name, length, limit in report:
        if limit:
            print(f"  {'ok  ' if length <= limit else 'LONG'} {name:28} {length:5.0f} mm (kit cable {limit})")
else:
    print(f"{COLL}: wiring drawn on {EX.name}")
