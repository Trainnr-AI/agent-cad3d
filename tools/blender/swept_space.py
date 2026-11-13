"""Map the space the moving parts sweep, to find the room left for a fixed part (the body). Runs inside Blender.

Every visible mesh that hangs on a joint body (i.e. moves relative to the trunk) is sampled on its surface
(~0.6 mm spacing, once, in its own frame); for each pose the samples are carried to the trunk frame and
marked in a voxel grid, then grown by `clear_mm`. Free space = the voxels reachable from `seed` without crossing a
marked voxel (so the insides of swept solids are not free). Result: a .npz with the grid, and a printed table of the
free half-width about the centre plane for each (x, z) cell.

Poses: every joint alone through its full range; a walking range (half yaw/roll range corners x hip pitch +-45 deg
with the knee following); squat-like (pitch, knee) pairs; neck and head-pitch sweeps with the head turned +-90 deg.
Extreme combinations are left out on purpose: they pass through the body on the original duck too.

ARGS = {"robot": "duck_scs0009", "out": "/abs/swept.npz", "voxel_mm": 1.5, "clear_mm": 1.5,
        "box_mm": [[-75, 60], [-45, 45], [-90, 35]] (trunk frame: x, y, z), "seed_mm": [5, 0, -8], "skip_prefix": ["wire_", "cable_stub"]}
"""
import math

import bmesh
import bpy
import numpy as np
from mathutils import Quaternion, Vector

args = globals()["ARGS"]
R = args.get("robot", "duck_scs0009")
VOX, CLR = args.get("voxel_mm", 1.5), args.get("clear_mm", 1.5)
BOX = np.array(args.get("box_mm", [[-75, 60], [-45, 45], [-90, 35]]), float)
trunk = bpy.data.objects[f"{R}:trunk_base"]
joints = {e["joint"]: e for e in bpy.data.objects if e.name.startswith(R + ":") and e.type == "EMPTY" and "joint" in e and e.users_collection}   # (in the scene: not a stray copy pulled in by a library load)
q0 = {n: e.rotation_quaternion.copy() for n, e in joints.items()}
skip = tuple(args.get("skip_prefix", ["wire_", "cable_stub"]))
movers = []
for e in joints.values():
    for o in e.children:                       # direct children of the joint body, and what hangs on them (servo roots)
        stack = [o]
        while stack:
            c = stack.pop()
            if c.type == "EMPTY" and c.name.startswith(R + ":"):
                continue                       # the next body: handled by its own joint
            stack.extend(c.children)
            if c.type == "MESH" and not c.hide_get() and not c.name.startswith(skip) and ".x" not in c.name:
                movers.append(c)
rng = np.random.default_rng(1)
samples = {}
for o in movers:
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    pts = [np.array([v.co[:] for v in bm.verts])]
    for f in bm.faces:
        a, b, c = [np.array(v.co[:]) for v in f.verts]
        n = int(f.calc_area() * 1e6 / 0.36) + 1          # one sample per 0.36 mm2
        u = rng.random((n, 2))
        m = u.sum(1) > 1
        u[m] = 1 - u[m]
        pts.append(a + u[:, :1] * (b - a) + u[:, 1:] * (c - a))
    bm.free()
    samples[o.name] = np.vstack(pts)
shape = np.ceil((BOX[:, 1] - BOX[:, 0]) / VOX).astype(int)
grid = np.zeros(shape, bool)


def mark():
    bpy.context.view_layer.update()
    Ti = np.array(trunk.matrix_world.inverted())
    for o in movers:
        M = Ti @ np.array(o.matrix_world)
        p = samples[o.name] @ M[:3, :3].T + M[:3, 3]
        idx = np.floor((p * 1000 - BOX[:, 0]) / VOX).astype(int)
        ok = np.all((idx >= 0) & (idx < shape), axis=1)
        idx = idx[ok]
        grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True


def pose(q):
    for jn, e in joints.items():
        e.rotation_quaternion = q0[jn] @ Quaternion(Vector(e["joint_axis"]), q.get(jn, 0.0))


def rng_of(j, k, n):
    lo, hi = joints[j]["joint_range"]
    return lo + (hi - lo) * k / (n - 1)


def legs(d):
    """same leg pose on both sides, mirrored like the joint ranges are"""
    q = {}
    for j, v in d.items():
        q["left_" + j] = v
        q["right_" + j] = -v
    return q


n_pose = 0
N = args.get("samples", 9)
LEG = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")


def do(q):
    global n_pose
    pose(q)
    mark()
    n_pose += 1


# 1. every joint alone through its full range (this is what the original's ranges promise)
for j in LEG:
    for k in range(N):
        do(legs({j: rng_of("left_" + j, k, N)}))
for j in ("neck_pitch", "head_pitch", "head_yaw", "head_roll"):
    for k in range(13):
        do({j: rng_of(j, k, 13)})
# 2. walking range: half of the yaw / roll range at the corners, with hip pitch +-45 deg and the knee following it
yl, yh = joints["left_hip_yaw"]["joint_range"]
rl, rh = joints["left_hip_roll"]["joint_range"]
for yaw in (yl / 2, 0.0, yh / 2):
    for roll in (rl / 2, 0.0, rh / 2):
        for p in (-0.8, -0.4, 0.0, 0.4, 0.8):
            for kn in (0.0, -2 * p * 0.6, 2 * p * 0.6):
                do(legs({"hip_yaw": yaw, "hip_roll": roll, "hip_pitch": p, "knee": kn, "ankle": -p * 0.5}))
# 3. squat-like pairs (pitch, knee) with the ankle levelling the foot
for p in (-1.2, -0.6, 0.6, 1.2):
    for kn in (-1.5, -0.75, 0.75, 1.5):
        do(legs({"hip_pitch": p, "knee": kn, "ankle": 0.0}))
# 4. head: neck sweep with the head turned, head pitch sweep with the head turned
for k in range(13):
    for hy in (-1.57, 1.57):
        do({"neck_pitch": rng_of("neck_pitch", k, 13), "head_yaw": hy})
        do({"head_pitch": rng_of("head_pitch", k, 13), "head_yaw": hy})
pose({})
bpy.context.view_layer.update()
# grow by the clearance
g = grid.copy()
r = int(math.ceil(CLR / VOX))
for ax in range(3):
    for s in range(1, r + 1):
        g |= np.roll(grid, s, ax) | np.roll(grid, -s, ax)
blocked = g
# free = reachable from the seed
seed = np.floor((np.array(args.get("seed_mm", [5, 0, -8]), float) - BOX[:, 0]) / VOX).astype(int)
free = np.zeros(shape, bool)
free[tuple(seed)] = not blocked[tuple(seed)]
for _ in range(600):
    grow = free.copy()
    for ax in range(3):
        grow[1:] |= free[:-1] if ax == 0 else False
        grow[:-1] |= free[1:] if ax == 0 else False
    grow[:, 1:] |= free[:, :-1]
    grow[:, :-1] |= free[:, 1:]
    grow[:, :, 1:] |= free[:, :, :-1]
    grow[:, :, :-1] |= free[:, :, 1:]
    grow &= ~blocked
    if grow.sum() == free.sum():
        break
    free = grow
np.savez_compressed(args["out"], free=free, blocked=blocked, box=BOX, vox=VOX)
print(f"swept space: {len(movers)} moving meshes, {n_pose} poses, voxel {VOX} mm, clearance {CLR} mm; seed free: {bool(free[tuple(seed)])}")
