"""Fit audit: for every listed pair of objects report the overlap volume (ray-parity sampling, see
overlap_sample.py) and, if they don't overlap, the smallest gap between their surfaces; compare with
what the fit is meant to be. Runs inside Blender.

expect:  "clear"   no overlap (<= tol_mm3); the gap is reported (0 = touching)
         "press"   designed interference (horn in its crush-rib pocket): press_mm3[0] <= overlap <= press_mm3[1]
         "thread"  a screw in its pilot hole / thread: some overlap is required
ARGS = {"lib": "/abs/tools/blender/overlap_sample.py", "step_mm": 0.15, "tol_mm3": 0.03, "press_mm3": [0.5, 4.0],
        "checks": [{"group": "left knee", "a": "scs_left_knee_case", "b": "left_thigh", "expect": "clear", "note": "servo in cradle"}, ...],
        "json_out": "/abs/out.json"}
"""
import json

import bmesh
import bpy
from mathutils import Vector

args = globals()["ARGS"]
L = {"ARGS": {"pairs": []}}
exec(open(args["lib"]).read(), L)
STEP = args.get("step_mm", 0.15) / 1000
TOL = args.get("tol_mm3", 0.03)
PRESS = args.get("press_mm3", [0.5, 4.0])
bpy.context.view_layer.update()
cache = {}


def info(name):
    if name not in cache:
        ob = bpy.data.objects[name]
        trees, lo, hi = L["parts"](ob)
        bm = bmesh.new()
        bm.from_mesh(ob.data)
        bm.transform(ob.matrix_world)
        pts = [v.co.copy() for v in bm.verts] + [f.calc_center_median() for f in bm.faces] + [(e.verts[0].co + e.verts[1].co) / 2 for e in bm.edges]
        bm.free()
        cache[name] = (trees, lo, hi, pts)
    return cache[name]


def overlap(a, b):
    ta, la, ha, _ = info(a)
    tb, lb, hb, _ = info(b)
    lo = Vector([max(la[i], lb[i]) for i in range(3)])
    hi = Vector([min(ha[i], hb[i]) for i in range(3)])
    if not all(lo[i] < hi[i] for i in range(3)):
        return 0.0
    n = [max(1, int((hi[i] - lo[i]) / STEP)) for i in range(3)]
    d = [(hi[i] - lo[i]) / n[i] for i in range(3)]
    cnt = 0
    for i in range(n[0]):
        for j in range(n[1]):
            for k in range(n[2]):
                p = lo + Vector(((i + 0.5) * d[0], (j + 0.5) * d[1], (k + 0.5) * d[2]))
                if L["inside"](ta, p) and L["inside"](tb, p):
                    cnt += 1
    return cnt * d[0] * d[1] * d[2] * 1e9


def gap(a, b):
    best = 1e9
    for x, y in ((a, b), (b, a)):
        tx, lo, hi, pts = info(x)
        ty, lo2, hi2, _ = info(y)
        for p in pts:
            if any(p[i] < lo2[i] - 0.003 or p[i] > hi2[i] + 0.003 for i in range(3)):
                continue
            for t in ty:
                r = t.find_nearest(p, min(best, 0.003) if best < 1e8 else 0.003)
                if r[0] is not None and r[3] < best:
                    best = r[3]
    return best * 1000 if best < 1e8 else None


rows, fails, group = [], 0, None
for c in args["checks"]:
    if c["a"] not in bpy.data.objects or c["b"] not in bpy.data.objects:
        rows.append({**c, "result": "MISSING"})
        fails += 1
        print(f"  MISSING  {c['a']} / {c['b']}")
        continue
    v = overlap(c["a"], c["b"])
    g = gap(c["a"], c["b"]) if v <= TOL else None
    e = c.get("expect", "clear")
    ok = (v <= TOL) if e == "clear" else (PRESS[0] <= v <= PRESS[1]) if e == "press" else (v > TOL)
    fails += 0 if ok else 1
    if c.get("group") != group:
        group = c.get("group")
        print(f"\n== {group}")
    gtxt = "" if g is None else f", gap {g:.2f} mm" if g > 0.005 else ", touching"
    print(f"  {'ok  ' if ok else 'FAIL'} {e:6} {c.get('note', ''):44} overlap {v:6.2f} mm3{gtxt}")
    rows.append({**c, "overlap_mm3": round(v, 3), "gap_mm": None if g is None else round(g, 3), "ok": ok})
print(f"\n{len(rows)} fits checked, {fails} not as designed")
if args.get("json_out"):
    json.dump(rows, open(args["json_out"], "w"), indent=1)
