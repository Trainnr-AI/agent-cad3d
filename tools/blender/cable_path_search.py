"""Cable path search in the head (A* on a 1 mm voxel map; every part grown by the clearance; rest pose, the jaw closed AND fully open).
Headless, the user's Blender is not touched:

  blender -b projects/xgoduck_scs0009/xgoduck_scs0009.blend --python tools/blender/cable_path_search.py -- out.json CLEAR "x,y,z" "x,y,z"

CLEAR = cable radius + margin (mm); the two points = start / goal (layout mm). Prints the simplified path (Douglas-Peucker 1.2 mm) for
wiring.py waypoints; out.json gets the path, out.json.npy the occupancy map. Used 2026-11-19 for the camera cable in the head
(clearance 1.7 for the Ø3.2 cable): REVIEW.md "Camera cable in the head".
"""
import bpy, sys, json, math, heapq
import numpy as np
from mathutils import Vector, Quaternion
from mathutils.bvhtree import BVHTree
out = sys.argv[sys.argv.index("--") + 1]
dg = bpy.context.evaluated_depsgraph_get()
trunk = bpy.data.objects["duck_scs0009:trunk_base"]; Mt = trunk.matrix_world.copy(); HOME = Vector((0, -100, 118.8))
W = lambda p: Mt @ ((Vector(p) - HOME) * 0.001)
skip = ("exploded", "explode", "coupon", "print_tools", "xgoduck_original", "dimensions", "orig_head", "_cutters", "wiring", "cables", "labels")
X0, X1, Y0, Y1, Z0, Z1 = 14, 94, -137, -63, 184, 238
R_CLEAR = float(sys.argv[sys.argv.index('--') + 2]) if len(sys.argv) > sys.argv.index('--') + 2 else 2.0
def lay_bb(o):
    pts = [(Mt.inverted() @ (o.matrix_world @ Vector(c))) * 1000 + HOME for c in o.bound_box]
    return [min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)]
def trees_now():
    dg.update()
    res = []
    for o in bpy.data.objects:
        if o.type != "MESH" or not o.visible_get() or any(s in c.name for c in o.users_collection for s in skip): continue
        lo, hi = lay_bb(o)
        if hi[0] < X0 or lo[0] > X1 or hi[1] < Y0 or lo[1] > Y1 or hi[2] < Z0 or lo[2] > Z1: continue
        try: me = o.evaluated_get(dg).to_mesh()
        except RuntimeError: continue
        if not me or not len(me.polygons): continue
        vs = [(Mt.inverted() @ (o.matrix_world @ v.co)) * 1000 + HOME for v in me.vertices]
        res.append((o.name, BVHTree.FromPolygons(vs, [list(p.vertices) for p in me.polygons])))
        o.evaluated_get(dg).to_mesh_clear()
    return res
nx, ny, nz = X1 - X0, Y1 - Y0, Z1 - Z0
occ = np.zeros((nx, ny, nz), bool)
up = Vector((0.013, 0.007, 1.0)).normalized()
def fill(trees):
    for name, t in trees:
        for i in range(nx):
            for j in range(ny):
                for k in range(nz):
                    if occ[i, j, k]: continue
                    p = Vector((X0 + i + 0.5, Y0 + j + 0.5, Z0 + k + 0.5))
                    if t.find_nearest(p, R_CLEAR)[0] is not None: occ[i, j, k] = True; continue
                    n, q = 0, p
                    while True:
                        h = t.ray_cast(q, up, 300)
                        if h[0] is None: break
                        n += 1; q = h[0] + up * 0.001
                    if n % 2: occ[i, j, k] = True
fill(trees_now())
# jaw fully open (beak 0..+35): turn head_jaw_pivot 35 deg about the mouth servo's shaft (its local Z), the way that lowers the jaw
pv, sv, jaw = bpy.data.objects["head_jaw_pivot"], bpy.data.objects["scs_mouth"], bpy.data.objects["head_jaw"]
from mathutils import Matrix
M0 = pv.matrix_world.copy(); ax = (sv.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized(); c0 = sv.matrix_world.translation.copy()
def jaw_minz():
    dg.update(); return min((jaw.matrix_world @ Vector(c)).z for c in jaw.bound_box)
z_rest = jaw_minz(); jn = None
for sgn in (1, -1):
    R = Matrix.Translation(c0) @ Matrix.Rotation(math.radians(35.0 * sgn), 4, ax) @ Matrix.Translation(-c0)
    pv.matrix_world = R @ M0
    if jaw_minz() < z_rest - 0.002:
        jn = f"jaw open ({'+' if sgn > 0 else '-'}35 about the mouth shaft)"
        fill([(n, t) for n, t in trees_now() if n == "head_jaw"])
        break
pv.matrix_world = M0; dg.update()
print("jaw joint:", jn, "occupied", int(occ.sum()), "of", occ.size)
np.save(out + ".npy", occ)
_a = sys.argv[sys.argv.index('--') + 3:]
start = tuple(float(v) for v in _a[0].split(',')) if _a else (23.5, -118.5, 198.0)
goal = tuple(float(v) for v in _a[1].split(',')) if len(_a) > 1 else (82.5, -131.0, 210.0)
def cell(p): return (int(p[0] - X0), int(p[1] - Y0), int(p[2] - Z0))
s, g = cell(start), cell(goal)
for c in (s, g):
    print("cell", c, "occupied" if occ[c] else "free")
occ[s] = False; occ[g] = False
nb = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)]
dist = {s: 0.0}; prev = {}; pq = [(0.0, s)]
h = lambda c: math.dist(c, g)
while pq:
    f, c = heapq.heappop(pq)
    if c == g: break
    for d in nb:
        n2 = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
        if not (0 <= n2[0] < nx and 0 <= n2[1] < ny and 0 <= n2[2] < nz) or occ[n2]: continue
        nd = dist[c] + math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        if nd < dist.get(n2, 1e9):
            dist[n2] = nd; prev[n2] = c; heapq.heappush(pq, (nd + h(n2), n2))
if g not in prev:
    print("NO PATH"); json.dump({"path": None}, open(out, "w")); raise SystemExit
path = [g]
while path[-1] != s: path.append(prev[path[-1]])
path = [(X0 + c[0] + 0.5, Y0 + c[1] + 0.5, Z0 + c[2] + 0.5) for c in reversed(path)]
def rdp(pts, eps):
    if len(pts) < 3: return pts
    a, b = Vector(pts[0]), Vector(pts[-1]); ab = b - a
    dmax, idx = 0, 0
    for i in range(1, len(pts) - 1):
        p = Vector(pts[i]); d = (ab.cross(p - a)).length / max(ab.length, 1e-9)
        if d > dmax: dmax, idx = d, i
    if dmax > eps: return rdp(pts[:idx + 1], eps)[:-1] + rdp(pts[idx:], eps)
    return [pts[0], pts[-1]]
simple = rdp(path, 1.2)
print("path length %.0f mm, %d waypoints" % (dist[g], len(simple)))
for p in simple: print("  (%.1f, %.1f, %.1f)" % p)
json.dump({"path": simple}, open(out, "w"))
