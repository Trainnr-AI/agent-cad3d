"""Grow link offsets just enough that no two servos collide. Runs inside Blender.

Works on a robot collection built by import_mjcf.py (any scale) with servos placed by
place_servos.py (each servo root stores `joint_body` and `M_joint_local`). Every servo is
modelled as oriented boxes in its own frame (SCS0009 frame: origin on the shaft at the case
top, +Z out of the shaft, +X toward the far end):
  body  23.3 x 12.1 x 25.25 (x from -5.95 to 17.35, z from -25.25 to 0)
  ears  32.5 x 11 x 1.6     (x from -10.55 to 21.95, z from -8.45 to -6.85)
  horn  swept disc of the X horn, as a 2R x 2R x 4 box above the case (z 0.3 .. 4.3)
inflated by ARGS["clearance_mm"]/2 on every side.

Rules, so the robot keeps its shape:
  * A collision is fixed by moving the body offset (`location` of a body empty = MJCF pos) of
    the deeper "anchor" body on the kinematic path between the two servos.
  * The offset GROWS ALONG ITS OWN DIRECTION (the link gets longer, never crooked or shorter)
    when that direction can separate the pair; otherwise the minimum-translation direction is used.
  * Only the left side + centre are solved; the right side is re-mirrored from the left
    (about the root body's XZ plane) after every move, so the robot stays symmetric.

ARGS = {"collection": "duck_scs0009", "clearance_mm": 1.0, "horn_r_mm": 12.3, "max_iter": 80,
        "root": "trunk_base", "mirror_pairs": [["yaw2roll", "bearing_roll"], ...],
        "right_servo_prefix": ["scs_left_", "scs_right_"]}
"""
import itertools

import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
C = args["collection"]
coll = bpy.data.collections[C]
CL = args.get("clearance_mm", 1.0) / 2000
HR = args.get("horn_r_mm", 12.3) / 1000
MM = 0.001
BOXES = [  # (min xyz, max xyz) in servo frame, metres
    (Vector((-5.95, -6.05, -25.25)) * MM, Vector((17.35, 6.05, 0.0)) * MM),
    (Vector((-10.55, -5.5, -8.45)) * MM, Vector((21.95, 5.5, -6.85)) * MM),
    (Vector((-HR, -HR, 0.3 * MM)), Vector((HR, HR, 4.3 * MM))),
]


def body(n):
    return bpy.data.objects[f"{C}:{n}"]


root = body(args.get("root", "trunk_base"))
pairs = [(body(l), body(r)) for l, r in args.get("mirror_pairs", [])]
right_bodies = {r for _, r in pairs}
lp, rp = args.get("right_servo_prefix", ["scs_left_", "scs_right_"])
servos = [o for o in coll.objects if o.type == "EMPTY" and "M_joint_local" in o]
solve_set = [s for s in servos if not s.name.startswith(rp)]
bodies = [o for o in coll.objects if o.type == "EMPTY" and o.name.startswith(f"{C}:")]
start = {o.name: o.location.copy() for o in bodies}


def M_local(s):
    v = list(s["M_joint_local"])
    return Matrix([v[0:4], v[4:8], v[8:12], v[12:16]])


def obbs(s):
    W = bpy.data.objects[s["joint_body"]].matrix_world @ M_local(s)
    R = W.to_3x3().normalized()
    out = []
    for lo, hi in BOXES:
        c = W @ ((lo + hi) / 2)
        h = (hi - lo) / 2 + Vector((CL, CL, CL))
        out.append((c, [R.col[0], R.col[1], R.col[2]], h))
    return out


def sat(A, B):
    """(depth, unit axis from A to B) if the boxes overlap, else None."""
    ca, ua, ha = A
    cb, ub, hb = B
    d = cb - ca
    best = None
    for ax in ua + ub + [x.cross(y) for x in ua for y in ub]:
        if ax.length < 1e-9:
            continue
        ax = ax.normalized()
        ra = sum(abs(ax.dot(u)) * h for u, h in zip(ua, ha))
        rb = sum(abs(ax.dot(u)) * h for u, h in zip(ub, hb))
        overlap = ra + rb - abs(d.dot(ax))
        if overlap <= 0:
            return None
        if best is None or overlap < best[0]:
            best = (overlap, ax if d.dot(ax) >= 0 else -ax)
    return best


def collide(s1, s2):
    worst = None
    for b1 in obbs(s1):
        for b2 in obbs(s2):
            r = sat(b1, b2)
            if r and (worst is None or r[0] > worst[0]):
                worst = r
    return worst


def chain(b):
    out = []
    while b is not None:
        out.append(b)
        b = b.parent
    return out


def remirror():
    bpy.context.view_layer.update()
    T = root.matrix_world
    Ti = T.inverted()
    for l, r in pairs:                     # pairs are listed parents-first
        p = Ti @ l.matrix_world.translation
        p.y = -p.y
        r.location = r.parent.matrix_world.inverted() @ (T @ p)
        bpy.context.view_layer.update()


def move(b, world_move):
    """Grow b's offset along its own direction if that separates; else move along world_move."""
    P = b.parent.matrix_world.to_3x3().normalized() if b.parent else Matrix.Identity(3)
    local_move = P.inverted() @ world_move
    u = b.location.normalized() if b.location.length > 2 * MM else None
    need = world_move.length
    if u is not None:
        along = (P @ u).normalized().dot(world_move.normalized())
        if along > 0.3:                    # only ever lengthen; shrinking can flip a link through zero
            b.location += u * (need / along)
            return "grow"
    b.location += local_move
    return "shift"


remirror()
for it in range(args.get("max_iter", 80)):
    bpy.context.view_layer.update()
    worst = None
    for s1, s2 in itertools.combinations(servos, 2):
        if s1 not in solve_set and s2 not in solve_set:
            continue
        r = collide(s1, s2)
        if r and (worst is None or r[0] > worst[0]):
            worst = (r[0], r[1], s1, s2)
    if worst is None or worst[0] < 1e-5:
        print(f"clear after {it} iterations")
        break
    depth, axis, s1, s2 = worst
    j1, j2 = bpy.data.objects[s1["joint_body"]], bpy.data.objects[s2["joint_body"]]
    c1, c2 = chain(j1), chain(j2)
    if j2 in c1:
        target, wm = j1, -axis * depth
    elif j1 in c2:
        target, wm = j2, axis * depth
    else:
        common = next(b for b in c1 if b in c2)
        b1, b2 = c1[c1.index(common) - 1], c2[c2.index(common) - 1]
        # move the left/centre one; the mirror moves its twin by the same amount
        if b1 in right_bodies:
            target, wm = b2, axis * depth / 2
        else:
            target, wm = b1, -axis * depth / (2 if b2 in right_bodies else 1)
    if target in right_bodies:              # never edit the right side directly
        target = next(l for l, r in pairs if r == target)
        wm = Vector((wm.x, -wm.y, wm.z))
    how = move(target, wm)
    remirror()
    print(f"  it {it:2d}: {s1.name} x {s2.name}  depth {depth * 1000:5.2f} mm  -> {how} {target.name.split(':')[1]}")
else:
    print("WARNING: not clear after max_iter")

bpy.context.view_layer.update()
for s in servos:
    s.matrix_world = bpy.data.objects[s["joint_body"]].matrix_world @ M_local(s)

print("changed offsets (mm): body  |scaled| -> |solved|  (scaled -> solved)")
for o in bodies:
    if (o.location - start[o.name]).length > 1e-6:
        a, b = start[o.name] * 1000, o.location * 1000
        print(f"  {o.name.split(':')[1]:16} {a.length:5.1f} -> {b.length:5.1f}   ({a.x:6.1f},{a.y:6.1f},{a.z:6.1f}) -> ({b.x:6.1f},{b.y:6.1f},{b.z:6.1f})")
