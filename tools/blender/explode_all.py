"""Exploded view of the redesigned duck with every element separate: each link is moved off its joint
along that joint's axis, each cap-joint servo pulled out of its holder, bearings / horns / screws spread
along the axis. Works from the servo frames (scs_* roots), so both legs mirror automatically.
Runs inside Blender; copies are '<name>.x' in ARGS["collection"] (replaced on re-run).

ARGS = {"collection": "duck_v2_exploded", "offset": [0, -0.6, 0.3], "scale": 1.5, "gap": 30,
        "parts": ["legs", "neck"] (default both) or "sides": ["left"], "robot": "duck_scs0009",
        "shells": {"trunk": "duck_scs0009:trunk_base.001", "yaw_block": "duck_scs0009:head_yaw", "head": "duck_scs0009:head_roll"}}
"""
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
G = args.get("gap", 30.0)
COLL = args.get("collection", "duck_v2_exploded")
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
bpy.context.view_layer.update()
Z = Vector((0, 0, 1))


def axis(servo):
    return (bpy.data.objects[servo].matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()


off = {}


def cap_joint(servo, P, h, r, a):
    """holder offset h, rotor offset r, axis a (+ = toward the rotor)"""
    sv = h - a * 0.75 * G
    for part in ("case", "spline", "case_screws"):
        off[f"{servo}_{part}"] = sv
    off[f"{servo}_horn_cross"] = (h + r) / 2 + a * 4
    off[f"{P}brg_6702ZZ"] = h + a * 0.3 * G
    off[f"{P}screw_M2x6"] = r + a * 0.45 * G
    off[f"{P}ear_screws_M2x6st"] = sv - a * 0.45 * G


def fork_joint(servo, P, h, r, bp, a):
    """holder h (servo stays in it), rotor r on the horn side, bearing plate bp on the other side"""
    for part in ("case", "spline", "case_screws"):
        off[f"{servo}_{part}"] = h
    off[f"{servo}_horn_cross"] = (h + r) / 2 + a * 4
    off[f"{P}screw_M2x6"] = r + a * 0.45 * G
    off[f"{P}ear_screws_M2x6st"] = h + a * 0.35 * G
    off[f"{P}nut_M3"] = h + a * 0.3 * G
    off[f"{P}brg_693"] = (h + bp) / 2 - a * 2
    off[f"{P}screw_M3x8"] = bp - a * 0.6 * G


HAS_BODY = bpy.data.objects.get("body_frame") is not None        # body v2: the hip-yaw caps are in the body frame
legs_down = Z * 0 if HAS_BODY else (-Z * (1.6 * G) if "neck" in args.get("parts", ["legs", "neck"]) else Z * 0)
if "legs" in args.get("parts", ["legs", "neck"]):
    off["hip_yaw_support"] = legs_down
    for s in args.get("sides", ["left", "right"]):
        ax = {j: axis(f"scs_{s}_{j}") for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")}
        out = ax["ankle"] * (1 if s == "left" else 1)                 # outboard-ish spread so the two legs part
        b = {"plate": legs_down}
        b["yaw"] = b["plate"] + ax["hip_yaw"] * G * (1.6 if HAS_BODY else 1.0)
        b["lb"] = b["yaw"] + ax["hip_roll"] * G
        b["thigh"] = b["lb"] - ax["hip_pitch"] * G - Z * 12
        b["shin"] = b["thigh"] + ax["knee"] * G - Z * 22
        b["foot"] = b["shin"] + ax["ankle"] * G * 0.8 - Z * 30
        b["bp"] = b["shin"] - ax["ankle"] * G * 0.8 - Z * 30
        for k, n in (("yaw", "yaw_connector"), ("lb", "l_bracket"), ("thigh", "thigh"), ("shin", "shin"), ("foot", "foot"), ("bp", "foot_bearing_plate")):
            off[f"{s}_{n}"] = b[k]
        cap_joint(f"scs_{s}_hip_yaw", f"{s}_hip_yaw_", b["plate"], b["yaw"], ax["hip_yaw"])
        cap_joint(f"scs_{s}_hip_roll", f"{s}_hip_roll_", b["yaw"], b["lb"], ax["hip_roll"])
        cap_joint(f"scs_{s}_hip_pitch", f"{s}_hip_pitch_", b["thigh"], b["lb"], ax["hip_pitch"])
        cap_joint(f"scs_{s}_knee", f"{s}_knee_", b["thigh"], b["shin"], ax["knee"])
        fork_joint(f"scs_{s}_ankle", f"{s}_ankle_", b["shin"], b["foot"], b["bp"], ax["ankle"])
        off[f"{s}_foot_plate_screws_M2x6st"] = b["bp"] + Z * 14
if "neck" in args.get("parts", ["legs", "neck"]):
    sh = args.get("shells", {"trunk": "duck_scs0009:trunk_base.001", "yaw_block": "yaw_block", "head": "duck_scs0009:head_roll"})
    an, ah, ay = axis("scs_neck_pitch"), axis("scs_head_pitch"), axis("scs_head_yaw")
    if not HAS_BODY:
        off[sh["trunk"]] = Z * 0                                      # old scaled shell (only while there is no body v2)
    if HAS_BODY:                                                  # motor body stays; the electronics enclosure comes off the back
        BK = Vector((-2.2 * G, 0, 0))
        off["body_frame"] = Z * 0
        off.update({"electronics_enclosure": BK, "enclosure_screws_M2x6st": BK * 2.0, "servo_adapter": BK + Z * 1.9 * G, "ubec": BK + Z * 1.9 * G,
                    "battery_3s": BK + Z * 2.6 * G, "imu": BK + Z * 3.2 * G, "hub_6": BK + Z * 3.2 * G, "body_lid": BK + Z * 3.9 * G,
                    "body_lid_screws_M2x6st": BK + Z * 4.5 * G, "uno_q": BK + Z * 5.0 * G})
        for i in range(1, 6):
            off[f"hub_{i}"] = Z * 1.5 * G
    cradle = Z * (1.5 * G) + (Vector((1.6 * G, 0, 0)) if HAS_BODY else Z * 0)      # forward, clear of the body's contents
    mount, mplate = Z * (1.2 * G) + an * G, (Z * 0 if HAS_BODY else Z * (1.2 * G)) - an * G
    u, uplate = cradle + Z * (1.6 * G) + ah * G * 0.9, cradle + Z * (1.6 * G) - ah * G * 0.9
    block = u + Z * (1.7 * G)
    head = block + Z * (1.5 * G)
    off.update({"neck_cradle": cradle, "body_neck_mount": mount, "body_neck_bearing_plate": mplate, "head_pitch_u": u,
                "head_pitch_bearing_plate": uplate, sh["yaw_block"]: block, sh["head"]: head})
    fork_joint("scs_neck_pitch", "neck_", cradle, mount, mplate, an)
    fork_joint("scs_head_pitch", "head_pitch_", cradle, u, uplate, ah)
    off["neck_plate_screws_M2x6st"] = mplate - an * 0.5 * G
    off["head_pitch_plate_screws_M2x6st"] = u + Z * 0.5 * G
    # head yaw: the servo sits in the yaw block above, its output points down into the U's top plate
    for part in ("case", "spline", "case_screws"):
        off[f"scs_head_yaw_{part}"] = block
    off["scs_head_yaw_horn_cross"] = (block + u) / 2 + ay * 4
    off["head_yaw_brg_6702ZZ"] = block + ay * 0.45 * G
    off["head_yaw_screw_M2x6"] = u + ay * 0.45 * G
    off["head_yaw_ear_screws_M2x6st"] = block - ay * 0.45 * G
    # head roll: the servo sits in the head, its output points forward into the yaw block
    ar = axis("scs_head_roll")
    for part in ("case", "spline", "case_screws"):
        off[f"scs_head_roll_{part}"] = head
    off["scs_head_roll_horn_cross"] = (head + block) / 2 + ar * 4
    off["head_roll_brg_6702ZZ"] = head + ar * 0.3 * G
    off["head_roll_screw_M2x6"] = block + ar * 0.45 * G
    off["head_roll_ear_screws_M2x6st"] = head - ar * 0.45 * G
p0 = bpy.data.objects[f"{args.get('robot', 'duck_scs0009')}:trunk_base"].matrix_world.translation.copy()
S = args.get("scale", 1.0)
T = Matrix.Translation(Vector(args.get("offset", (0, 0, 0)))) @ Matrix.Scale(S, 4) @ Matrix.Translation(-p0)
n = 0
for name, d in off.items():
    o = bpy.data.objects.get(name)
    if o is None:
        print("  (missing)", name)
        continue
    c = bpy.data.objects.new(name + ".x", o.data)
    coll.objects.link(c)
    c.matrix_world = T @ Matrix.Translation(d / 1000) @ o.matrix_world
    n += 1
print(f"{COLL}: {n} elements")
