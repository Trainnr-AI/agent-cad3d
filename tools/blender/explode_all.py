"""Exploded view of the redesigned duck with every element separate: each link is moved off its joint
along that joint's axis, each cap-joint servo pulled out of its holder, bearings / horns / screws spread
along the axis. Works from the servo frames (scs_* roots), so both legs mirror automatically.
Runs inside Blender; copies are '<name>.x' in ARGS["collection"] (replaced on re-run).

ARGS = {"collection": "duck_v2_exploded", "offset": [0, -0.6, 0.3], "scale": 1.5, "gap": 30, "bay": {name: [x, y, z] mm from the enclosure},
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


SERVO_OUT = args.get("servo_out", {"hip_yaw": 2.2})                # how far (gaps) a cap-joint servo comes out of its holder, per joint


def cap_joint(servo, P, h, r, a):
    """holder offset h, rotor offset r, axis a (+ = toward the rotor)"""
    sv = h - a * next((v for k, v in SERVO_OUT.items() if servo.endswith(k)), 0.75) * G
    for part in ("case", "spline", "case_screws"):
        off[f"{servo}_{part}"] = sv
    off[f"{servo}_horn_cross"] = (h + r) / 2 + a * 4
    off[f"{P}brg_6702ZZ"] = h + a * 0.3 * G
    off[f"{P}screw_M2x4kit"] = r + a * 0.45 * G
    off[f"{P}ear_screws_M2x6st"] = sv - a * 0.45 * G


def c2m_joint(servo, P, h, r, a):
    """full-horn knee (type C2M, 2026-11-13): servo in the holder h, the moving part r carries the 6702ZZ, the bolt-on cap and its
    2 screws come off beyond it, in build order along the axis: horn screw, bearing, cap, cap screws"""
    cap_joint(servo, P, h, r, a)
    off[f"{P}screw_M2x4kit"] = r + a * 0.25 * G
    off[f"{P}brg_6702ZZ"] = r + a * 0.5 * G
    off[f"{P}cap"] = r + a * 0.85 * G
    off[f"{P}cap_screws_M2x6"] = r + a * 1.25 * G


def fork_joint(servo, P, h, r, bp, a):
    """holder h, rotor r on the horn side, bearing plate bp on the other side. The servo comes OUT of its cradle toward the
    horn side (the way it goes in), half-way to the rotor; the caller leaves room for it (2026-11-13: it stayed inside)."""
    sv = h + (r - h) * 0.5
    for part in ("case", "spline", "case_screws"):
        off[f"{servo}_{part}"] = sv
    off[f"{servo}_horn_cross"] = (sv + r) / 2 + a * 4
    off[f"{P}screw_M2x4kit"] = r + a * 0.45 * G
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
        b["foot"] = b["shin"] + ax["ankle"] * G * 2.4 - Z * 30            # room for the ankle servo out of its cradle
        # inboard side of the ankle: only ~22 mm to the mid-plane, so bearing / plate / axle screw are spread by
        # 3 / 9 / 20 mm - with gap-sized steps the two legs' plates and screws crossed over and swapped sides
        low = b["shin"] - Z * 30
        b["bp"] = low - ax["ankle"] * 9
        for k, n in (("yaw", "yaw_connector"), ("lb", "l_bracket"), ("thigh", "thigh"), ("shin", "shin"), ("foot", "foot"), ("bp", "foot_bearing_plate")):
            off[f"{s}_{n}"] = b[k]
        off[f"{s}_thigh_cover"] = b["thigh"] - ax["hip_pitch"] * G * 3.0          # optional cable cover: off the thigh's outer side, beyond the pulled-out servos and hubs
        off[f"{s}_thigh_cover_screws_M2x6st"] = b["thigh"] - ax["hip_pitch"] * G * 4.2
        cap_joint(f"scs_{s}_hip_yaw", f"{s}_hip_yaw_", b["plate"], b["yaw"], ax["hip_yaw"])
        cap_joint(f"scs_{s}_hip_roll", f"{s}_hip_roll_", b["yaw"], b["lb"], ax["hip_roll"])
        cap_joint(f"scs_{s}_hip_pitch", f"{s}_hip_pitch_", b["thigh"], b["lb"], ax["hip_pitch"])
        (c2m_joint if bpy.data.objects.get(f"{s}_knee_cap") else cap_joint)(f"scs_{s}_knee", f"{s}_knee_", b["thigh"], b["shin"], ax["knee"])
        fork_joint(f"scs_{s}_ankle", f"{s}_ankle_", b["shin"], b["foot"], b["bp"], ax["ankle"])
        off[f"{s}_ankle_brg_693"] = low - ax["ankle"] * 3
        off[f"{s}_ankle_screw_M3x8"] = low - ax["ankle"] * 20
        off[f"{s}_foot_plate_screws_M2x6st"] = low - ax["ankle"] * 17    # they go in along the axle direction, like the axle screw
if "neck" in args.get("parts", ["legs", "neck"]):
    sh = args.get("shells", {"trunk": "duck_scs0009:trunk_base.001", "yaw_block": "yaw_block", "head": "duck_scs0009:head_roll"})
    an, ah, ay = axis("scs_neck_pitch"), axis("scs_head_pitch"), axis("scs_head_yaw")
    if not HAS_BODY:
        off[sh["trunk"]] = Z * 0                                      # old scaled shell (only while there is no body v2)
    if HAS_BODY:                                                  # motor body stays; the electronics bay comes off the back, taken apart in build
        BK = Vector((-2.2 * G, 0, 0))                             # order (as data/build_args/bay_explode.json: offsets from the enclosure, mm)
        off["body_frame"] = Z * 0
        bay = args.get("bay", {"electronics_enclosure": [0, 0, 0], "enclosure_screws_M2x6st": [16, 0, 0], "battery_3s": [22, 0, 85],
                               "servo_adapter": [-28, 0, 62], "imu": [-58, 0, 72], "imu_screws_M2x6st": [-70, 0, 72], "ubec": [-4, 0, 118],
                               "power_switch": [-4, 26, 118], "body_lid": [0, 0, 140], "body_lid_screws_M2x6st": [0, 0, 156],
                               "uno_q": [0, 0, 172], "unoq_pcb": [0, 0, 172], "unoq_headers": [0, 0, 172], "unoq_usbc": [0, 0, 172],
                               "unoq_pins": [0, 0, 172], "unoq_bottom_connectors": [0, 0, 172], "unoq_top_parts": [0, 0, 172],
                               "uno_screws_M3x8": [0, 0, 188], "uno_screw_M2x6": [0, 0, 188], "uno_cover": [0, 0, 210], "uno_cover_screws_M2x6st": [0, 0, 250]})
        for n_, v_ in bay.items():
            off[n_] = BK + Vector(v_)
    cradle = Z * (1.5 * G) + (Vector((1.6 * G, 0, 0)) if HAS_BODY else Z * 0)      # forward, clear of the body's contents
    mount, mplate = cradle + an * G * 2.4, (Z * 0 if HAS_BODY else Z * (1.2 * G)) - an * G   # mount: where the neck servo's horn side points (room for it)
    u, uplate = cradle + Z * (1.6 * G) + ah * G * 2.4, cradle + Z * (1.6 * G) - ah * G * 0.9
    block = u + Z * (1.7 * G)
    head = block + Z * (1.5 * G)
    off.update({"neck_cradle": cradle, "body_neck_mount": mount, "body_neck_bearing_plate": mplate, "head_pitch_u": u,
                "head_pitch_bearing_plate": uplate, sh["yaw_block"]: block, sh["head"]: head})
    HEAD_V2 = bpy.data.objects.get("head_base") is not None       # head v2 (parts/head_shell_v2.py) replaces the old one-piece shell
    if HEAD_V2:
        del off[sh["head"]]
    fork_joint("scs_neck_pitch", "neck_", cradle, mount, mplate, an)
    fork_joint("scs_head_pitch", "head_pitch_", cradle, u, uplate, ah)
    off["neck_plate_screws_M2x6st"] = mplate - an * 0.5 * G
    off["head_pitch_plate_screws_M2x6st"] = u + Z * 0.5 * G
    # head yaw: the servo sits in the yaw block above, its output points down into the U's top plate
    hy = block - ay * 0.8 * G                                      # the servo out of the yaw block, the way it goes in (2026-11-13: it stayed in)
    for part in ("case", "spline", "case_screws"):
        off[f"scs_head_yaw_{part}"] = hy
    off["scs_head_yaw_horn_cross"] = (block + u) / 2 + ay * 4
    off["head_yaw_brg_6702ZZ"] = block + ay * 0.45 * G
    off["head_yaw_screw_M2x4kit"] = u + ay * 0.45 * G
    off["head_yaw_ear_screws_M2x6st"] = hy - ay * 0.5 * G
    # head roll: the servo sits in the head, its output points forward into the yaw block
    ar = axis("scs_head_roll")
    for part in ("case", "spline", "case_screws"):
        off[f"scs_head_roll_{part}"] = head
    off["scs_head_roll_horn_cross"] = (head + block) / 2 + ar * 4
    off["head_roll_brg_6702ZZ"] = head + ar * 0.3 * G
    off["head_roll_screw_M2x4kit"] = block + ar * 0.45 * G
    off["head_roll_ear_screws_M2x6st"] = head - ar * 0.45 * G
    if HEAD_V2:                                                    # the head taken apart, as tools/blender/head_explode.py (layout mm, x fwd / y left / z up)
        for n_, d_ in {"head_dome": (0, 0, 62), "dome_screws_M2x6st": (0, 0, -50), "head_base": (0, 0, 0),   # dome last; its front screws come up through the jaw
                       "scs_head_roll_case": (-14, 0, 30), "scs_head_roll_spline": (-14, 0, 30), "scs_head_roll_case_screws": (-14, 0, 30),
                       "head_roll_ear_screws_M2x6st": (-34, 0, 30), "head_roll_brg_6702ZZ": (14, 0, 30), "scs_head_roll_horn_cross": (26, 0, 30),
                       "scs_mouth_case": (0, -22, 30), "scs_mouth_spline": (0, -22, 30), "scs_mouth_case_screws": (0, -22, 30),   # beak servo: in from the
                       "mouth_ear_screws_M2x6st": (0, -44, 30), "scs_mouth_horn_cross": (0, 14, 30), "scs_mouth_horn_screw": (0, 28, 30),  # middle, slid out
                       "head_jaw": (0, 0, -32), "jaw_pivot_screw_M2x6st": (0, 30, 12),                    # jaw up through its slots first
                       "head_face": (62, 0, 40), "cam_ov5640_b": (38, 0, 40), "cam_screws_M2x6st": (20, 0, 40)}.items():   # camera onto the face from behind
            off[n_] = head + Vector(d_)
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
