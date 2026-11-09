"""Check that parts can be assembled by sliding them along a straight path. Runs inside Blender.

For each step: move `part` from start_mm to 0 (its final pose) along a direction (a servo frame's
+Z unless given), in small increments, and test it against `against` objects (triangle BVH; a hit is
confirmed by an intersection volume via overlap_sample-style ray parity). Contacts allowed at the
final pose (designed press fits) are listed in `allow_final`.

ARGS = {"lib": "/abs/tools/blender/overlap_sample.py",
        "steps": [{"name": "U onto horn", "part": "head_pitch_u", "frame": "scs_head_pitch", "start_mm": 6,
                   "against": ["neck_cradle", "scs_head_pitch_case", "scs_head_pitch_horn_cross"],
                   "allow_final": ["scs_head_pitch_horn_cross"]}, ...], "step_mm": 0.5}
"""
import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
L = {"ARGS": {"pairs": []}}
exec(open(args["lib"]).read(), L)


def tree(o):
    mw = o.matrix_world
    return BVHTree.FromPolygons([mw @ v.co for v in o.data.vertices], [p.vertices[:] for p in o.data.polygons])


def overlap_mm3(a, b, step=0.25):
    ta, la, ha = L["parts"](a)
    tb, lb, hb = L["parts"](b)
    lo = Vector([max(la[i], lb[i]) for i in range(3)])
    hi = Vector([min(ha[i], hb[i]) for i in range(3)])
    if not all(lo[i] < hi[i] for i in range(3)):
        return 0.0
    S = step / 1000
    n = 0
    nx, ny, nz = [max(1, int((hi[i] - lo[i]) / S)) for i in range(3)]
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                p = lo + Vector(((i + 0.5) * S, (j + 0.5) * S, (k + 0.5) * S))
                if L["inside"](ta, p) and L["inside"](tb, p):
                    n += 1
    return n * step ** 3


for st in args["steps"]:
    part = bpy.data.objects[st["part"]]
    d = Vector(st["dir"]) if st.get("dir") else bpy.data.objects[st["frame"]].matrix_world.to_3x3().col[2].normalized()
    mw0 = part.matrix_world.copy()
    worst = []
    k = st["start_mm"]
    inc = args.get("step_mm", 0.5) * (1 if k > 0 else -1)
    while abs(k) > 1e-9:
        part.matrix_world = Matrix.Translation(d * k / 1000) @ mw0
        bpy.context.view_layer.update()
        tp = tree(part)
        for n in st["against"]:
            o = bpy.data.objects[n]
            if tp.overlap(tree(o)):
                v = overlap_mm3(part, o)
                if v > 0.05:
                    worst.append((round(k, 2), n, round(v, 2)))
        k -= inc
        if (inc > 0 and k < 0) or (inc < 0 and k > 0):
            k = 0
    part.matrix_world = mw0
    bpy.context.view_layer.update()
    tp = tree(part)
    final = []
    for n in st["against"]:
        o = bpy.data.objects[n]
        if tp.overlap(tree(o)):
            v = overlap_mm3(part, o)
            if v > 0.05:
                final.append((n, round(v, 2), n in st.get("allow_final", [])))
    blocked = [w for w in worst if w[1] not in st.get("allow_final", []) or abs(w[0]) > st.get("engage_mm", 4.5)]
    status = "OK" if not blocked and all(f[2] for f in final) else "BLOCKED"
    print(f"{status:7} {st['name']}: path {st['start_mm']} -> 0 mm"
          + (f"; blocked at {blocked[:3]}" if blocked else "")
          + (f"; final contacts {final}" if final else ""))
