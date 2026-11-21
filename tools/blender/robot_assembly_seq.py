"""Build the WHOLE-ROBOT assembly sequence (ASSEMBLY.md sections 1-4) for assembly_anim.py, from the model itself (runs inside
Blender, writes JSON; nothing in the scene is changed):

  - servo / part moves use the servo frames' axes and the verified assembly paths (data/check_args/asm.json, segment_paths.json,
    the left hip-yaw servo's L-path), extended for show; every move is path-checked afterwards with anim_to_paths.py +
    assembly_path_check.py (bench sub-assemblies are tested against each other where they are built);
  - screws drive in along their own axes, head out (axes from the screw audit's JSON); their straight paths are the driver lines
    checked by checks/driver_access.py;
  - bearings pressed in on the bench ride with their part; horns ride with the part they are pressed into;
  - the bay is data/build_args/bay_assembly_anim.json, as is;
  - the neck cradle, the head and each leg below the hip are built off the robot (offset), then put on.

ARGS = {"bay": "<abs>/bay_assembly_anim.json", "asm": "<abs>/asm.json", "segments": "<abs>/segment_paths.json",
        "screws": "<abs>/screws.json" (screw audit output with axes), "out": "<abs>/robot_assembly_anim.json"}
"""
import json

import bpy
from mathutils import Vector

args = globals()["ARGS"]
asm = {s["name"]: s for s in json.load(open(args["asm"]))["steps"]}
segs = {p["name"].split(":")[0].split(" ")[0]: p for p in json.load(open(args["segments"]))["paths"]}
SCR = {}
for r in json.load(open(args["screws"])):
    SCR.setdefault(r["screw"], []).append(r)
O = bpy.data.objects


def Z(frame):
    return O[frame].matrix_world.to_3x3().col[2].normalized()


def mm(v):
    return [round(c, 3) for c in v]


def has(n):
    return n in O


def servo_parts(s):
    return [n for n in (f"scs_{s}_case", f"scs_{s}_case_screws", f"scs_{s}_spline") if has(n)]


def move(objs, start, base=Vector(), frames=30, via=None):
    """objs from base + start to base (world mm vectors); via: extra points between"""
    pts = [base + start] + ([base + v for v in via] if via else []) + [base]
    m = {"objects": [n for n in objs if has(n)], "path": [mm(p) for p in pts], "frames": frames}
    if base.length > 1e-9:
        m["hold"] = True
    return m


def screws(names, base=Vector(), out=None, frames=22):
    """screw stand-ins: each screw drives in along its own axis (head outward), at the group's offset `base`"""
    mv = []
    for n in names:
        if not has(n) or n not in SCR:
            continue
        pcs = [{"at": r["at"], "axis": r["axis"], "len": r["length"]} for r in SCR[n]]
        L = max(r["length"] for r in SCR[n])
        mv.append({"objects": [n], "screw_pieces": pcs, "out_mm": out or (L + 8.0), "base": mm(base), "frames": frames,
                   **({"hold": True} if base.length > 1e-9 else {})})
    return mv


def nut(n, screw, base=Vector(), out=12.0):
    """a nut comes onto its screw's tip from the tip side"""
    if not has(n) or screw not in SCR:
        return None
    ax = Vector(SCR[screw][0]["axis"]).normalized()
    return move([n], ax * -out, base)


def appear(objs, base=Vector(), frames=16, drop=25.0):
    """a part placed (on the bench, or as the base of a sub-assembly): comes down from above onto `base`"""
    return move(objs, Vector((0, 0, drop)), base, frames)


steps = []


def step(title, moves, wires=()):
    steps.append({"title": title, "moves": [m for m in moves if m and m.get("objects")], "wires": [w for w in wires if has(w)]})


# ---------------- bearings and horns ride with their parts
printed = ["body_frame", "neck_cradle", "body_neck_bearing_plate", "head_pitch_u", "head_pitch_bearing_plate", "yaw_block", "head_base",
           "left_yaw_connector", "right_yaw_connector", "left_l_bracket", "right_l_bracket", "left_thigh", "right_thigh", "left_shin",
           "right_shin", "left_foot", "right_foot", "left_foot_bearing_plate", "right_foot_bearing_plate", "left_knee_cap", "right_knee_cap"]


def bbox(n):
    P = [O[n].matrix_world @ Vector(c) for c in O[n].bound_box]
    return Vector([min(p[i] for p in P) for i in range(3)]), Vector([max(p[i] for p in P) for i in range(3)])


# the bearings' parts: ASSEMBLY.md section 0 item 5 (pressed in on the bench, before any servo)
host = {"left_hip_yaw_brg_6702ZZ": "body_frame", "right_hip_yaw_brg_6702ZZ": "body_frame",
        "left_hip_roll_brg_6702ZZ": "left_yaw_connector", "right_hip_roll_brg_6702ZZ": "right_yaw_connector",
        "left_hip_pitch_brg_6702ZZ": "left_thigh", "right_hip_pitch_brg_6702ZZ": "right_thigh",
        "head_roll_brg_6702ZZ": "head_base", "head_yaw_brg_6702ZZ": "yaw_block",
        "left_ankle_brg_693": "left_foot_bearing_plate", "right_ankle_brg_693": "right_foot_bearing_plate",
        "neck_brg_693": "body_neck_bearing_plate", "head_pitch_brg_693": "head_pitch_bearing_plate"}   # (knees: their own move, into the shin)
horn_host = {"neck_pitch": "body_frame", "head_pitch": "head_pitch_u", "head_yaw": "head_pitch_u", "head_roll": "yaw_block",
             "left_hip_yaw": "left_yaw_connector", "right_hip_yaw": "right_yaw_connector", "left_hip_roll": "left_l_bracket",
             "right_hip_roll": "right_l_bracket", "left_hip_pitch": "left_l_bracket", "right_hip_pitch": "right_l_bracket",
             "left_knee": "left_shin", "right_knee": "right_shin", "left_ankle": "left_foot", "right_ankle": "right_foot"}


PRESS = {"left_hip_yaw_brg_6702ZZ": (0, 0, -1), "right_hip_yaw_brg_6702ZZ": (0, 0, -1), "left_hip_roll_brg_6702ZZ": (1, 0, 0),
         "right_hip_roll_brg_6702ZZ": (1, 0, 0), "left_hip_pitch_brg_6702ZZ": (0, -1, 0), "right_hip_pitch_brg_6702ZZ": (0, 1, 0),
         "head_roll_brg_6702ZZ": (1, 0, 0), "head_yaw_brg_6702ZZ": (0, 0, -1)}   # path-checked 2026-11-21: the only free side of each seat
#   (the four 693s go in from either side: from the side their plate comes on from)


def brgs_of(h):
    return [b for b, hh in host.items() if hh == h and has(b)]


def press(b, base=Vector(), plate_dir=None):
    """a bearing pressed into its part (outer ring, on the bench) from its seat's open side"""
    d = Vector(PRESS[b]) if b in PRESS else (plate_dir.normalized() if plate_dir is not None else Vector((0, 1, 0)))
    if b not in PRESS:                                              # a 693: along its plate's axis (y)
        d = Vector((0, 1 if d.y >= 0 else -1, 0))
    return move([b], d * 15.0, base, 18)


def bench(host_objs, start, base=Vector(), frames=30, via=None):
    """a part with bearings: put on the bench where its move starts, its bearings pressed in there, then it goes on with them"""
    h = host_objs[0]
    bs = brgs_of(h)
    out = [move(host_objs, Vector((0, 0, 20.0)), base + start, 14)]
    out[0]["hold"] = True
    out += [dict(press(b, base + start, start), hold=True) for b in bs]
    out.append(move(host_objs + bs, start, base, frames, via=via))
    return out


def with_riders(parts):
    """the part + the horns pressed into its pockets (bearings: their own press moves, see bench / press)"""
    out = list(parts)
    for s, h in horn_host.items():
        if h in parts and has(f"scs_{s}_horn_cross"):
            out.append(f"scs_{s}_horn_cross")
    return out


# ---------------- 1. body
zs = Z("scs_right_hip_yaw")
step("BODY 1  Motor body; the two 6702ZZ (orange) pressed into its hip-yaw seats from below\n         (outer ring only; bearings go in before any servo)",
     [appear(with_riders(["body_frame"]), drop=40.0)] + [press(b) for b in brgs_of("body_frame")])
step("BODY 2  Right hip-yaw servo straight down into its saddle;\n         left one lifted ~10 mm, slid in from the side, then down",
     [move(servo_parts("right_hip_yaw"), zs * -40.0),
      move(servo_parts("left_hip_yaw"), Vector((0, 45.0, 10.0)), via=[Vector((0, 0, 10.0))], frames=40)])   # the whole servo's window: lift 9.5-10.5
step("BODY 3  Ear screws (left front ear: M2x6 socket, ball-end key at 20-25 deg)",
     screws(["right_hip_yaw_ear_screws_M2x6st", "left_hip_yaw_ear_screws_M2x6st"]))

# ---------------- 3. legs, steps 1-2 for both legs BEFORE the bay (2026-11-20: with the bay in, the hip-roll servo cannot get past the
#                  battery - whole-robot path check)
for side, sy in (("left", 1.0), ("right", -1.0)):
    S = side.upper()
    zyw, zro = Z(f"scs_{side}_hip_yaw"), Z(f"scs_{side}_hip_roll")
    step(f"{S} LEG 1 (before the bay)  6702ZZ pressed into the yaw connector (bench);\n         yaw connector onto the hip-yaw horn and bearing; horn screw",
         bench(with_riders([f"{side}_yaw_connector"]), zyw * 30.0) + screws([f"{side}_hip_yaw_screw_M2x4kit"]))
    step(f"{S} LEG 2 (before the bay)  Hip-roll servo into the yaw connector, ear screws (an L-key)",
         [move(servo_parts(f"{side}_hip_roll"), zro * -14.0)] + screws([f"{side}_hip_roll_ear_screws_M2x6st"]))   # the verified 14 (the bay is behind it)

# ---------------- bay (section 1 step 4)
bay = json.load(open(args["bay"]))
for st in bay["steps"]:
    t = st["title"]
    steps.append({"title": "BAY " + t, "moves": st["moves"], "wires": st.get("wires", [])})

# ---------------- 2. neck and head
zn = Z("scs_neck_pitch")
NOFF = zn * -60.0                                              # the cradle is built beside the robot, then goes on along the neck axis
step("NECK 1  Both M3 nylon nuts into the cradle's pockets FIRST (the servos cover them),\n        then the neck servo down into the cradle (on the bench), ear screws",
     [appear(["neck_cradle"], NOFF), move(["neck_nut_M3", "head_pitch_nut_M3"], Vector((0, 12.0, 0)), NOFF),     # from +y only, before the servos
      move(servo_parts("neck_pitch"), zn * 30.0, NOFF)] + screws(["neck_ear_screws_M2x6st"], NOFF))
step("NECK 2  Cradle onto the motor body's neck horn plate; horn screw;\n        bearing plate from the other side (2x M2x6), M3x8 axle into its nut",
     [{"objects": [n for n in ["neck_cradle", "neck_nut_M3", "head_pitch_nut_M3"] + servo_parts("neck_pitch") + ["neck_ear_screws_M2x6st"] if has(n)],
       "path": [mm(NOFF), mm(zn * -7.0), [0, 0, 0]], "frames": 40},
      *screws(["neck_screw_M2x4kit"]),
      *bench(with_riders(["body_neck_bearing_plate"]), zn * -30.0),
      *screws(["neck_plate_screws_M2x6st", "neck_screw_M3x8"])])
zp = Z("scs_head_pitch")
step("NECK 3  Head-pitch servo down into the cradle, ear screws",
     [move(servo_parts("head_pitch"), zp * 30.0)] + screws(["head_pitch_ear_screws_M2x6st"]))
HOFF = zp * 75.0                                               # the head is built beside the robot, then goes on along the pitch axis
zr, zy = Z("scs_head_roll"), Z("scs_head_yaw")
step("HEAD 1  Head base; 6702ZZ pressed into its roll bulkhead; head-roll servo into its holder, ear screws",
     [appear(with_riders(["head_base"]), HOFF)] + [dict(press(b, HOFF), hold=True) for b in brgs_of("head_base")]
     + [move(servo_parts("head_roll"), zr * -30.0, HOFF)] + screws(["head_roll_ear_screws_M2x6st"], HOFF))
step("HEAD 2  6702ZZ pressed into the yaw block (bench); yaw block up through the floor opening\n        ~4.8 mm forward, then back onto the roll horn; horn screw",
     bench(with_riders(["yaw_block"]), Vector((4.8, 0, -35.0)), HOFF, frames=40, via=[Vector((4.8, 0, 0))]) + screws(["head_roll_screw_M2x4kit"], HOFF))
step("HEAD 3  Head-yaw servo up into the yaw block, ear screws",
     [move(servo_parts("head_yaw"), zy * -30.0, HOFF)] + screws(["head_yaw_ear_screws_M2x6st"], HOFF))
step("HEAD 4  U onto the head-yaw horn and bearing; horn screw",
     [move(with_riders(["head_pitch_u"]), zy * 25.0, HOFF)] + screws(["head_yaw_screw_M2x4kit"], HOFF))
step("HEAD 5  Jaw straight up through the two bill-plate slots; jaw pivot screw",
     [move(["head_jaw"], Vector((0, 0, -30.0)), HOFF)] + screws(["jaw_pivot_screw_M2x6st"], HOFF))
step("HEAD 6  Beak servo: down ~4.5 mm toward the middle, then out onto the jaw arm;\n        horn screw, ear screws",
     [move(["scs_mouth_case", "scs_mouth_case_screws", "scs_mouth_spline", "scs_mouth_horn_cross"], Vector((0, -4.5, 30.0)), HOFF,
           frames=36, via=[Vector((0, -4.5, 0))])] + screws(["scs_mouth_horn_screw", "mouth_ear_screws_M2x6st"], HOFF))
FOFF = HOFF + Vector((45.0, 0, 0))
step("HEAD 7  2 M2 nuts into their pockets, the camera in from the front, the visor over it;\n        3x M2x10 countersunk from the front (heads flush); then the face onto the head",
     [appear(["head_face"], FOFF), move(["cam_nuts_M2"], Vector((25.0, 0, 0)), FOFF), move(["cam_ov5640_b"], Vector((25.0, 0, 0)), FOFF),
      move(["head_visor"], Vector((25.0, 0, 0)), FOFF)]
     + screws(["cam_screws_M2x6st"], FOFF)
     + [{"objects": ["head_face", "cam_nuts_M2", "cam_ov5640_b", "head_visor", "cam_screws_M2x6st"], "path": [mm(FOFF), mm(HOFF)], "frames": 30, "hold": True}])
step("HEAD 8  Dome last, straight down: 2 rear screws from below,\n        2 front ones up through the jaw's holes",
     [move(["head_dome"], Vector((0, 0, 40.0)), HOFF)] + screws(["dome_screws_M2x6st"], HOFF))
head = ["head_base", "head_roll_brg_6702ZZ", "head_roll_ear_screws_M2x6st", "head_roll_screw_M2x4kit", "yaw_block", "head_yaw_ear_screws_M2x6st",
        "head_yaw_screw_M2x4kit", "head_pitch_u", "head_jaw", "jaw_pivot_screw_M2x6st", "scs_mouth_case", "scs_mouth_case_screws",
        "scs_mouth_spline", "scs_mouth_horn_cross", "scs_mouth_horn_screw", "mouth_ear_screws_M2x6st", "head_face", "head_visor", "cam_nuts_M2", "cam_ov5640_b",
        "cam_screws_M2x6st", "head_dome", "dome_screws_M2x6st"] + servo_parts("head_roll") + servo_parts("head_yaw")
head = list(dict.fromkeys(head + [r for r in with_riders(["head_base", "yaw_block", "head_pitch_u"]) if r not in head]
                         + brgs_of("head_base") + brgs_of("yaw_block")))
step("HEAD 9  Head (with the U) onto the head-pitch servo in the cradle; horn screw;\n        bearing plate, M3x8 axle into its nut",
     [{"objects": [n for n in head if has(n)], "path": [mm(HOFF), mm(zp * 7.0), [0, 0, 0]], "frames": 44},
      *screws(["head_pitch_screw_M2x4kit"]),
      *bench(with_riders(["head_pitch_bearing_plate"]), zp * -30.0),
      *screws(["head_pitch_plate_screws_M2x6st", "head_pitch_screw_M3x8"])])

# ---------------- 3. legs
for side, sy in (("left", 1.0), ("right", -1.0)):
    S = side.upper()
    zyw, zro, zpi, zkn, zan = (Z(f"scs_{side}_{j}") for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"))
    LOFF = zro * 9.0 + Vector((0, sy * 80.0, 0))               # the leg below the hip is built beside the robot, comes in from the side to 9 mm
    #                                                            out along the roll axis, then the verified last 9 mm (from further out along the
    #                                                            axis the knee cap runs into the yaw connector)
    step(f"{S} LEG 3  Off the robot: 6702ZZ pressed into the thigh's hip-pitch seat;\n         hip-pitch and knee servos into the thigh, ear screws",
         [appear([f"{side}_thigh"], LOFF)] + [dict(press(b, LOFF), hold=True) for b in brgs_of(f"{side}_thigh")] + [move(servo_parts(f"{side}_hip_pitch"), zpi * -30.0, LOFF),
          move(servo_parts(f"{side}_knee"), zkn * -30.0, LOFF)]
         + screws([f"{side}_hip_pitch_ear_screws_M2x6st", f"{side}_knee_ear_screws_M2x6st"], LOFF))
    step(f"{S} LEG 4  Knee: full horn in the shin's pocket, shin onto the spline, horn screw;\n         6702ZZ into the shin's seat; cap down, 2x M2x6 (snug last)",
         [move(with_riders([f"{side}_shin"]), zkn * 25.0, LOFF), move([f"{side}_knee_screw_M2x4kit"], zkn * 14.0, LOFF),
          move([f"{side}_knee_brg_6702ZZ"], zkn * 20.0, LOFF), move([f"{side}_knee_cap"], zkn * 20.0, LOFF),
          move([f"{side}_knee_cap_screws_M2x6"], zkn * 14.0, LOFF)])
    step(f"{S} LEG 5  M3 nylon nut into the shin's pocket first; ankle servo into the cradle, ear screws;\n         foot onto the horn, horn screw; bearing plate from the other side, M3x8 axle into the nut",
         [move([f"{side}_ankle_nut_M3"], Vector((0, sy * 12.0, 0)), LOFF),          # the nut FIRST (the servo covers its pocket), from outboard
          move(servo_parts(f"{side}_ankle"), zan * 30.0, LOFF)] + screws([f"{side}_ankle_ear_screws_M2x6st"], LOFF)
         + [move(with_riders([f"{side}_foot"]), zan * 20.0, LOFF)] + screws([f"{side}_ankle_screw_M2x4kit"], LOFF)
         + bench(with_riders([f"{side}_foot_bearing_plate"]), zan * -25.0, LOFF)
         + screws([f"{side}_foot_plate_screws_M2x6st", f"{side}_ankle_screw_M3x8"], LOFF))
    step(f"{S} LEG 6  L-bracket onto the hip-pitch horn and bearing; horn screw",
         [move(with_riders([f"{side}_l_bracket"]), zpi * 25.0, LOFF)] + screws([f"{side}_hip_pitch_screw_M2x4kit"], LOFF))
    leg = [f"{side}_{n}" for n in ("l_bracket", "thigh", "shin", "knee_cap", "knee_cap_screws_M2x6", "knee_brg_6702ZZ", "knee_screw_M2x4kit",
                                   "foot", "foot_bearing_plate", "foot_plate_screws_M2x6st", "ankle_screw_M3x8", "ankle_nut_M3",
                                   "ankle_screw_M2x4kit", "hip_pitch_screw_M2x4kit", "hip_pitch_ear_screws_M2x6st", "knee_ear_screws_M2x6st",
                                   "ankle_ear_screws_M2x6st")]
    leg += servo_parts(f"{side}_hip_pitch") + servo_parts(f"{side}_knee") + servo_parts(f"{side}_ankle")
    leg = list(dict.fromkeys(leg + with_riders([f"{side}_l_bracket", f"{side}_thigh", f"{side}_shin", f"{side}_foot", f"{side}_foot_bearing_plate"])
                             + brgs_of(f"{side}_thigh") + brgs_of(f"{side}_foot_bearing_plate")))
    step(f"{S} LEG 7  Hang the leg on the hip: L-bracket onto the hip-roll horn; horn screw",
         [{"objects": [n for n in leg if has(n)], "path": [mm(LOFF), mm(zro * 9.0), [0, 0, 0]], "frames": 44}]
         + screws([f"{side}_hip_roll_screw_M2x4kit"]))
    step(f"{S} LEG 8  Thigh cover (optional), 2x M2x6",
         [move([f"{side}_thigh_cover"], Vector((0, sy * 30.0, 0)))] + screws([f"{side}_thigh_cover_screws_M2x6st"]))

# ---------------- 4. wiring: the servo chain (the bay's own wires appear in its steps)
shown = {w for st in steps for w in st.get("wires", [])}
bus = [o.name for o in bpy.data.collections["wiring"].objects if o.name not in shown and not o.name.startswith(("wire_power", "wire_signal", "wire_usb"))]
step("WIRING  Servo leads, hubs and extension cables (sleeve + slack loop at the neck)", [], wires=bus)

placed = {n for st in steps for m in st["moves"] for n in m["objects"]} | shown | set(bus)
miss = [o.name for o in bpy.data.scenes["Scene"].objects if o.type == "MESH" and o.visible_get(view_layer=bpy.data.scenes["Scene"].view_layers[0])
        and not any(any(k in c.name for k in ("explode", "wiring", "labels", "section", "original", "dimensions", "assembly")) for c in o.users_collection)
        and o.name not in placed]
lo = Vector((1e9,) * 3); hi = -lo
for n in placed:
    if has(n) and O[n].type == "MESH":
        a, b = bbox(n)
        lo = Vector([min(lo[i], a[i]) for i in range(3)]); hi = Vector([max(hi[i], b[i]) for i in range(3)])
ctr = (lo + hi) / 2 * 1000
out = {"scene": "robot_assembly", "context": [], "against_extra": [], "fps": 24, "hold": 14, "steps": steps, "title_size": 0.042,
       "highlight": [{"contains": "_brg_", "rgba": [1.0, 0.42, 0.0, 1.0]}],                        # bearings in orange (the animation's copies only)
       "camera": {"target_mm": mm(ctr), "dist_mm": round(max(hi - lo) * 1000 * 3.3, 1), "elev_deg": 22, "azim_deg": [-60, 60], "lens": 50},
       "engage_mm": bay.get("engage_mm", {}), "play": False}
json.dump(out, open(args["out"], "w"), indent=1)
print(f"{len(steps)} steps, {sum(len(s['moves']) for s in steps)} moves -> {args['out']}")
print("bearing hosts:", host)
print("not placed by any step:", miss)
