"""Exploded view of a whole robot: every link pushed out along the kinematic tree. Runs inside Blender.

Makes linked copies (shared mesh data) of all meshes under the robot collection in a new
collection, placed at ARGS["offset"]. Each body moves away from its parent along its own
offset direction by ARGS["link_gap_mm"] (accumulating down the chain); each servo (place_servos.py
roots) is pulled ARGS["servo_gap_mm"] off its holder along its shaft axis (-Z of the servo frame,
i.e. out of the link it sits in). Re-running replaces the collection.

ARGS = {"robot": "duck_scs0009", "collection": "duck_exploded", "offset": [0, -0.32, 0],
        "link_gap_mm": 22, "servo_gap_mm": 0, "include_prefixes": ["duck_scs0009:", "scs_"],
        "extra_collections": ["leg_v1"]}   # other collections whose objects are parented to robot bodies
"""
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
R = args["robot"]
src = bpy.data.collections[R]
COLL = args.get("collection", f"{R}_exploded")
OFF = Vector(args.get("offset", (0, -0.32, 0)))
GAP = args.get("link_gap_mm", 22) / 1000
SG = args.get("servo_gap_mm", 0) / 1000
bpy.context.view_layer.update()

old = bpy.data.collections.get(COLL)
if old:
    for ob in list(old.all_objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)

bodies = {o for o in src.objects if o.type == "EMPTY" and o.name.startswith(R + ":")}


def body_of(o):
    while o is not None and o not in bodies:
        o = o.parent
    return o


shift = {}


def body_shift(b):
    if b in shift:
        return shift[b]
    if b.parent is None or b.parent not in bodies:
        shift[b] = Vector()
    else:
        d = b.matrix_world.translation - b.parent.matrix_world.translation
        step = d.normalized() * GAP if d.length > 1e-6 else Vector()
        shift[b] = body_shift(b.parent) + step
    return shift[b]


n = 0
pool = list(src.all_objects) + [o for c in args.get("extra_collections", []) for o in bpy.data.collections[c].all_objects]
for o in pool:
    if o.type != "MESH" or not any(o.name.startswith(p) for p in args.get("include_prefixes", [""])):
        continue
    if o.name in args.get("extra_offsets_mm", {}):          # per-object extra push (world mm)
        extra = Vector(args["extra_offsets_mm"][o.name]) / 1000
    else:
        extra = Vector()
    b = body_of(o)
    move = body_shift(b) if b else Vector()
    sroot = o
    while sroot is not None and "servo_joint" not in sroot:
        sroot = sroot.parent
    if sroot is not None and SG:
        move = move - (sroot.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized() * SG
    cp = bpy.data.objects.new(o.name + ".x", o.data)
    coll.objects.link(cp)
    cp.matrix_world = Matrix.Translation(OFF + move + extra) @ o.matrix_world
    cp.display_type = o.display_type
    n += 1
print(f"{COLL}: {n} parts exploded along {len(bodies)} bodies")
