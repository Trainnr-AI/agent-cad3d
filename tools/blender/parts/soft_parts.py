"""Soft (TPU) parts taken from the original at our scale. Runs inside Blender at the zero pose.

NOTE (2026-11-12): the sole is now PART OF THE FOOT (one printable piece, parts/leg_v2.py "sole_stl", user decision). leg_v2.py takes
sole_matrix() from this file; running this script still makes separate sole objects (for checks), which are not parts to print.

  left_sole / right_sole   the original's foot sole (2x_foot_bottom_tpu.stl, one part printed twice) at 0.92, under each foot. Our foot
                           keeps the original pad's footprint, so the sole fits it; of the two ways round it can lie, the one that
                           does not cut into the foot is taken (reported).

ARGS = {"robot": "duck_scs0009", "collection": "soft_parts", "stl_dir": "/abs/vendor/xgoduck_hardware/structure", "scale": 0.92}
"""
import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

args = globals()["ARGS"]
R = args.get("robot", "duck_scs0009")
S = args.get("scale", 0.92)
COLL = args.get("collection", "soft_parts")
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
trunk = bpy.data.objects[R + ":trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * 0.001)
MLi = ML.inverted()


def lay_pts(o):
    return np.array([((MLi @ o.matrix_world @ v.co) * 1000)[:] for v in o.data.vertices])


def mat(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


bpy.ops.wm.stl_import(filepath=args["stl_dir"] + "/2x_foot_bottom_tpu.stl")
raw = bpy.context.selected_objects[0]
bm0 = bmesh.new()
bm0.from_mesh(raw.data)
bmesh.ops.remove_doubles(bm0, verts=bm0.verts, dist=1e-4)
bpy.data.objects.remove(raw, do_unlink=True)
P = np.array([v.co[:] for v in bm0.verts])
zs, cnt = np.unique(np.round(P[:, 2], 2), return_counts=True)
print("sole STL: %.1f x %.1f x %.2f mm, z levels with most vertices: %s" % (*np.ptp(P, axis=0), [(float(z), int(c)) for z, c in sorted(zip(zs, cnt), key=lambda t: -t[1])[:6]]))
D = [Vector(d).normalized() for d in ((0.31, 0.17, 0.93), (-0.62, 0.71, 0.33), (0.45, -0.83, -0.33))]


def inside(t, p):
    k = 0
    for d in D:
        n, q = 0, Vector(p)
        for _ in range(60):
            h = t.ray_cast(q, d)
            if not h[0]:
                break
            n += 1
            q = h[0] + d * 1e-4
        k += n % 2
    return k >= 2


def sole_matrix(side, orig, down_body, scale, plate_t, R="duck_scs0009"):
    """Layout of the TPU sole (STL mm, z up, plate 0..plate_t) under the original pad, IN mm world: its top face on the pad's underside
    plane, which is the plane that lies on the floor when the original duck stands (down_body: the floor-down direction in the foot's
    body frame at the STAND keyframe, from MuJoCo). Long side along the foot."""
    import numpy as _np
    from mathutils import Matrix as _M, Vector as _V
    fe = bpy.data.objects[f"{R}:ankle_{side}"]
    down = (fe.matrix_world.to_3x3() @ _V(down_body)).normalized()
    P = [orig.matrix_world @ v.co * 1000.0 for v in orig.data.vertices]
    h = max(p.dot(down) for p in P)                                          # the pad's lowest point along "down"
    rim = [p for p in P if p.dot(down) > h - 0.8]                            # the pad's underside outline
    c = _V(((min(p.x for p in rim) + max(p.x for p in rim)) / 2, (min(p.y for p in rim) + max(p.y for p in rim)) / 2, (min(p.z for p in rim) + max(p.z for p in rim)) / 2))
    c = c + down * (h - c.dot(down))                                         # middle of the footprint, on the underside plane
    zs = -down
    ax = _V((0, 1, 0)) if side == "left" else _V((0, -1, 0))                 # where the STL's x goes (as before: +-90 deg about z)
    xs = (ax - zs * ax.dot(zs)).normalized()
    ys = zs.cross(xs)
    Rm = _M((xs, ys, zs)).transposed().to_4x4()
    return _M.Translation(c + down * plate_t * scale) @ Rm @ _M.Scale(scale, 4)


DOWN = args.get("down_body", {"left": [0.9962, -0.0049, 0.0871], "right": [0.9962, 0.0049, 0.0871]})   # MuJoCo, original duck at STAND
PT = 2.5                                                                       # the sole's plate thickness in the STL
for side in ("left", "right"):
    if f"{side}_sole" in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[f"{side}_sole"], do_unlink=True)
    foot = bpy.data.objects[f"{side}_foot"]
    orig = bpy.data.objects[f"{R}:{side}_ankle"]
    Ms = sole_matrix(side, orig, DOWN[side], S, PT, R)
    b = bm0.copy()
    b.transform(Matrix.Scale(0.001, 4) @ Ms)
    me = bpy.data.meshes.new(f"{side}_sole")
    b.to_mesh(me)
    vol = b.calc_volume() * 1e6
    b.free()
    me.materials.append(mat("tpu_sole", (0.9, 0.45, 0.1, 1)))
    o = bpy.data.objects.new(f"{side}_sole", me)
    coll.objects.link(o)
    mw = o.matrix_world.copy()
    o.parent = foot.parent
    o.matrix_world = mw
    o["material"] = "TPU"
    fb = bmesh.new()
    fb.from_mesh(foot.data)
    bmesh.ops.triangulate(fb, faces=fb.faces)
    fb.transform(foot.matrix_world)
    ft = BVHTree.FromBMesh(fb)
    Ps = [o.matrix_world @ v.co for v in o.data.vertices]
    deep = [ft.find_nearest(p)[3] * 1000 for p in Ps[::3] if inside(ft, p)]
    fb.free()
    print(f"{side}_sole: {vol * 1.2:.1f} g TPU; {len(deep)} of {len(Ps[::3])} sample vertices inside the foot, deepest {max(deep) if deep else 0:.2f} mm")
bm0.free()
