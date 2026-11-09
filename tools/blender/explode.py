"""Build an exploded view of an assembly. Runs inside Blender.

Reads `explode_mm` / `asm_step` / `caption` custom props (set by part scripts such as
parts/scs0009_joint.py) on objects under ARGS["root"], and creates linked copies offset
along the root's local Z axis in a new collection, plus an assembly axis line and
per-step captions. Re-running replaces the exploded collection.

ARGS = {"root": "kit_joint", "collection": "joint_kit_exploded", "offset": (0, -0.08, 0),
        "label_side": (0, 0.03, 0), "label_size": 0.004, "label_rotation": (1.5708, 0, 1.5708)}
"""
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
root = bpy.data.objects[args["root"]]
COLL = args.get("collection", f"{root.name}_exploded")
OFFSET = Vector(args.get("offset", (0, -0.08, 0)))
SIDE = Vector(args.get("label_side", (0, 0.03, 0)))

old = bpy.data.collections.get(COLL)
if old:
    for ob in list(old.all_objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)

bpy.context.view_layer.update()
axis = (root.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()


def explode_info(ob):
    while ob is not None and ob != root:
        if "explode_mm" in ob:
            return ob["explode_mm"], ob.get("asm_step", 0), ob.get("caption", "")
        ob = ob.parent
    return None


steps = {}
zs = []
base0 = root.matrix_world.translation + OFFSET
for ob in root.children_recursive:
    if ob.type != "MESH":
        continue
    info = explode_info(ob)
    if info is None:
        continue
    off_mm, step, cap = info
    cp = bpy.data.objects.new(f"{ob.name}.x", ob.data)
    coll.objects.link(cp)
    cp.matrix_world = Matrix.Translation(OFFSET + axis * off_mm * 0.001) @ ob.matrix_world
    zs.append(off_mm)
    centre = sum((cp.matrix_world @ Vector(c) for c in ob.bound_box), Vector()) / 8
    entry = steps.setdefault(step, ["", []])
    if cap:                                   # objects without a caption still move, but don't label
        entry[0] = cap
        entry[1].append((centre - base0).dot(axis))

# assembly axis line
base = root.matrix_world.translation + OFFSET
curve = bpy.data.curves.new(f"{COLL}_axis", "CURVE")
curve.dimensions = "3D"
sp = curve.splines.new("POLY")
sp.points.add(1)
lo, hi = (min(zs) - 12) * 0.001, (max(zs) + 12) * 0.001
sp.points[0].co = (*(base + axis * lo), 1)
sp.points[1].co = (*(base + axis * hi), 1)
curve.bevel_depth = 0.00015
line = bpy.data.objects.new(f"{COLL}_axis", curve)
mat = bpy.data.materials.get("explode_axis") or bpy.data.materials.new("explode_axis")
mat.diffuse_color = (0.9, 0.25, 0.2, 1)
curve.materials.append(mat)
coll.objects.link(line)

# captions, one per step, stacked beside the part they describe
for step, (cap, heights) in sorted(steps.items()):
    if not cap:
        continue
    h = sum(heights) / len(heights) if heights else 0.0
    txt = bpy.data.curves.new(f"{COLL}_cap{step}", "FONT")
    txt.body = cap
    txt.size = args.get("label_size", 0.004)
    txt.align_x = "LEFT"
    ob = bpy.data.objects.new(f"{COLL}_cap{step}", txt)
    coll.objects.link(ob)
    ob.location = base + axis * h + SIDE
    ob.rotation_euler = args.get("label_rotation", (1.5707963, 0, 1.5707963))
print(f"{COLL}: {len(zs)} parts in {len(steps)} steps, spread {min(zs)}..{max(zs)} mm")
