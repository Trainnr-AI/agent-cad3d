"""Compare the kinematics of two robot collections (import_mjcf.py) at the zero pose.
Runs inside Blender.

Checks, joint by joint: same parent/child bodies, same world axis direction (=> same positive
rotation direction), same range; link offsets as a ratio of the original. If servo roots
(place_servos.py) exist in the target, also checks that the servo's output faces the same side
as the original servo's output face, and that the joint range fits the servo's travel.

ARGS = {"original": "xgoduck_original", "target": "duck_scs0009", "servo_travel_deg": 300,
        "servos_json": "/abs/original_servos.json"}
"""
import json
import math

import bpy
from mathutils import Vector

args = globals()["ARGS"]
O, T = args["original"], args["target"]
bpy.context.view_layer.update()


def joints(c):
    return {e["joint"]: e for e in bpy.data.collections[c].objects if e.type == "EMPTY" and "joint" in e and e.name.startswith(c + ":")}


jo, jt = joints(O), joints(T)
travel = math.radians(args.get("servo_travel_deg", 300))
servo_roots = {o["servo_joint"]: o for o in bpy.data.collections[T].objects if "servo_joint" in o}
orig_servos = {r["joint"]: r for r in json.load(open(args["servos_json"]))} if args.get("servos_json") else {}

problems = []
print(f"{'joint':16} {'axis dot':>8} {'range (deg)':>16} {'offset mm orig->new (ratio)':>32}  servo")
for name in sorted(jo):
    a, b = jo[name], jt.get(name)
    if b is None:
        problems.append(f"{name}: missing in target")
        continue
    pa = a.parent.name.split(":")[1] if a.parent else None
    pb = b.parent.name.split(":")[1] if b.parent else None
    if pa != pb:
        problems.append(f"{name}: parent {pa} vs {pb}")
    ax_a = (a.matrix_world.to_3x3() @ Vector(a["joint_axis"])).normalized()
    ax_b = (b.matrix_world.to_3x3() @ Vector(b["joint_axis"])).normalized()
    dot = ax_a.dot(ax_b)
    if dot < 0.9999:
        problems.append(f"{name}: axis differs (dot {dot:.5f})")
    ra, rb = list(a.get("joint_range", [])), list(b.get("joint_range", []))
    if ra != rb:
        problems.append(f"{name}: range {ra} vs {rb}")
    span = (ra[1] - ra[0]) if ra else 0
    lo_d, hi_d = (math.degrees(ra[0]), math.degrees(ra[1])) if ra else (0, 0)
    la, lb = a.location.length * 1000, b.location.length * 1000
    sv = ""
    if name in servo_roots:
        s = servo_roots[name]
        z_s = (s.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()     # SCS0009 output direction
        # original: output face at the joint origin, body on side d -> output points -d*axis
        r = orig_servos.get(name)
        if r:
            c = Vector(r["centre"]) - a.matrix_world.translation
            out_o = -Vector(r["shaft_axis"]) * (1 if c.dot(Vector(r["shaft_axis"])) > 0 else -1)
            same = z_s.dot(out_o) > 0.999
            sv = "output same side" if same else "OUTPUT FLIPPED"
            if not same:
                problems.append(f"{name}: servo output faces the other way")
        # range must fit the servo travel, centred on joint zero (servo neutral 511 = joint 0)
        need = 2 * max(abs(ra[0]), abs(ra[1])) if ra else 0
        if need > travel + 1e-6:
            sv += f"; RANGE {lo_d:+.0f}..{hi_d:+.0f} needs ±{math.degrees(need / 2):.0f}° > servo ±{math.degrees(travel / 2):.0f}°"
            problems.append(f"{name}: range ±{math.degrees(need / 2):.0f}° exceeds servo travel ±{math.degrees(travel / 2):.0f}° (centred)")
        elif span > travel:
            sv += "; span > travel"
    print(f"{name:16} {dot:8.5f} {lo_d:+7.1f}..{hi_d:+6.1f}  {la:6.1f} -> {lb:6.1f} ({lb / la if la else 0:4.2f})   {sv}")

print("\nPROBLEMS:" if problems else "\nno kinematic problems found")
for p in problems:
    print("  -", p)
