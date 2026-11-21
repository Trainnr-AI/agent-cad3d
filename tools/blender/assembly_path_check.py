"""Check that parts can be assembled by sliding them along a straight path. Runs inside Blender.

For each step: move `part` from start_mm to 0 (its final pose) along a direction (a servo frame's
+Z unless given), in small increments, and test it against `against` objects (triangle BVH; a hit is
confirmed by an intersection volume via overlap_sample-style ray parity). Contacts allowed at the
final pose (designed press fits) are listed in `allow_final`.

ARGS = {"lib": "/abs/tools/blender/overlap_sample.py",
        "steps": [{"name": "U onto horn", "part": "head_pitch_u", "frame": "scs_head_pitch", "start_mm": 6,
                   "against": ["neck_cradle", "scs_head_pitch_case", "scs_head_pitch_horn_cross"],
                   "allow_final": ["scs_head_pitch_horn_cross"]}, ...], "step_mm": 0.5}
Optional per step: "dir": [x, y, z] (world) instead of a frame; "base_mm": [x, y, z] (world mm) - the path ends there instead of
at the final pose (the first leg of a two-leg move, e.g. lower in, then slide onto the bosses); its end is not judged as a final fit.
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


FAST = args.get("fast", False)       # sample only round the crossing faces, cache the trees, stop at the first 0.05 mm3:
_cache = {}                          # a real pass-through is always found there; parts that only touch still read 0


def _geo(o):
    if o.name not in _cache:
        mw = o.matrix_world
        vs = [mw @ v.co for v in o.data.vertices]
        ps = [p.vertices[:] for p in o.data.polygons]
        _cache[o.name] = (vs, ps, BVHTree.FromPolygons(vs, ps), L["parts"](o))
    return _cache[o.name]


def fast_overlap(part, o, off, ext=0.0006, step=0.25):
    """overlap (mm3, lower bound, stops at > 0.05) of `part` moved by `off` (m, world) with `o` at its place"""
    pv, pp, _, (pt, plo, phi) = _geo(part)
    ov, op, ot, (otr, olo, ohi) = _geo(o)
    tp = BVHTree.FromPolygons([v + off for v in pv], pp)
    pairs = tp.overlap(ot)
    if not pairs:
        return 0.0
    pts = [pv[i] + off for a, _ in pairs for i in pp[a]] + [ov[i] for _, b in pairs for i in op[b]]
    lo = Vector([max(min(q[i] for q in pts) - ext, plo[i] + off[i], olo[i]) for i in range(3)])
    hi = Vector([min(max(q[i] for q in pts) + ext, phi[i] + off[i], ohi[i]) for i in range(3)])
    if not all(lo[i] < hi[i] for i in range(3)):
        return 0.0
    S = step / 1000
    n, lim = 0, 0.05 / step ** 3
    nx, ny, nz = [max(1, int((hi[i] - lo[i]) / S)) for i in range(3)]
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                q = lo + Vector(((i + 0.5) * S, (j + 0.5) * S, (k + 0.5) * S))
                if L["inside"](otr, q) and L["inside"](pt, q - off):
                    n += 1
                    if n > lim:
                        return round(n * step ** 3, 3)
    return n * step ** 3


for st in args["steps"]:
    part = bpy.data.objects[st["part"]]
    if FAST:
        d = Vector(st["dir"])
        base = Vector(st.get("base_mm", (0, 0, 0)))
        worst, final = [], []
        k, inc = st["start_mm"], args.get("step_mm", 0.5)
        ks = []
        while k > 1e-9:
            ks.append(k)
            k -= inc
        ks.append(0.0)
        for k in ks:
            off = (base + d * k) / 1000
            for n in st["against"]:
                v = fast_overlap(part, bpy.data.objects[n], off)
                if v > 0.05:
                    if k == 0.0 and not st.get("base_mm"):
                        final.append((n, round(v, 2), n in st.get("allow_final", [])))
                    else:
                        worst.append((round(k, 2), n, round(v, 2)))
        blocked = [w for w in worst if w[1] not in st.get("allow_final", []) or abs(w[0]) > st.get("engage_mm", 4.5)]
        status = "OK" if not blocked and all(f[2] for f in final) else "BLOCKED"
        span = (f"{min(w[0] for w in blocked)}..{max(w[0] for w in blocked)} mm" if blocked else "")
        print(f"{status:7} {st['name']}: path {st['start_mm']} -> " + (f"base {st['base_mm']}" if st.get("base_mm") else "0") + " mm"
              + (f"; blocked {span} at {blocked[:3]}" if blocked else "")
              + (f"; final contacts {final}" if final else ""), flush=True)
        continue
    d = Vector(st["dir"]) if st.get("dir") else bpy.data.objects[st["frame"]].matrix_world.to_3x3().col[2].normalized()
    mw_home = part.matrix_world.copy()
    mw0 = Matrix.Translation(Vector(st.get("base_mm", (0, 0, 0))) / 1000) @ mw_home
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
    if st.get("base_mm"):                                    # a first leg ends in mid-air: its end is checked as a path point
        for n in st["against"]:
            if tp.overlap(tree(bpy.data.objects[n])):
                v = overlap_mm3(part, bpy.data.objects[n])
                if v > 0.05:
                    worst.append((0.0, n, round(v, 2)))
        part.matrix_world = mw_home
        bpy.context.view_layer.update()
        blocked = worst
        print(f"{'OK' if not blocked else 'BLOCKED':7} {st['name']}: path {st['start_mm']} -> base {st['base_mm']} mm"
              + (f"; blocked at {blocked[:3]}" if blocked else ""))
        continue
    for n in st["against"]:
        o = bpy.data.objects[n]
        if tp.overlap(tree(o)):
            v = overlap_mm3(part, o)
            if v > 0.05:
                final.append((n, round(v, 2), n in st.get("allow_final", [])))
    blocked = [w for w in worst if w[1] not in st.get("allow_final", []) or abs(w[0]) > st.get("engage_mm", 4.5)]
    part.matrix_world = mw_home
    bpy.context.view_layer.update()
    status = "OK" if not blocked and all(f[2] for f in final) else "BLOCKED"
    print(f"{status:7} {st['name']}: path {st['start_mm']} -> 0 mm"
          + (f"; blocked at {blocked[:3]}" if blocked else "")
          + (f"; final contacts {final}" if final else ""))
