"""FDM printability check: overhang area per candidate print orientation. Runs inside Blender.
For each object and each of the 6 axis-aligned "up" directions (or the given one), faces whose
normal points downward more than `max_overhang_deg` from vertical (i.e. the face would need support)
are summed, excluding the first-layer faces (within 0.3 mm of the lowest point). Reports the best
orientation, its unsupported area and the largest offending face.
ARGS = {"objects": ["left_thigh", ...], "max_overhang_deg": 50, "up": {"left_thigh": [0, 0, -1]} (optional)}
"""
import math

import bmesh
import bpy
from mathutils import Vector

args = globals()["ARGS"]
MAXO = math.radians(args.get("max_overhang_deg", 50))
AXES = {"+x": Vector((1, 0, 0)), "-x": Vector((-1, 0, 0)), "+y": Vector((0, 1, 0)), "-y": Vector((0, -1, 0)),
        "+z": Vector((0, 0, 1)), "-z": Vector((0, 0, -1))}


def evaluate(ob, up):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.transform(ob.matrix_world)
    bm.normal_update()
    heights = [v.co.dot(up) for v in bm.verts]
    floor = min(heights) + 0.0003
    area, worst = 0.0, (0.0, None)
    for f in bm.faces:
        if min(v.co.dot(up) for v in f.verts) <= floor:
            continue                                   # first layer
        down = -f.normal.dot(up)                       # 1 = facing straight down
        if down > math.cos(MAXO):
            a = f.calc_area() * 1e6
            area += a
            if a > worst[0]:
                worst = (a, f.calc_center_median())
    bm.free()
    return area, worst


for name in args["objects"]:
    ob = bpy.data.objects[name]
    cands = {"given": Vector(args["up"][name])} if args.get("up", {}).get(name) else AXES
    res = {k: evaluate(ob, v) for k, v in cands.items()}
    best = min(res, key=lambda k: res[k][0])
    others = ", ".join(f"{k}:{res[k][0]:.0f}" for k in res if k != best)
    a, (wa, wc) = res[best]
    print(f"{name}: best up={best}: {a:.0f} mm2 unsupported (largest face {wa:.0f} mm2)   [{others}]")
