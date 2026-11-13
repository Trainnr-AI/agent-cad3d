"""Hand-placed exploded view of a few objects: each copy is moved by its own offset (world mm), the
whole group scaled about a frame's origin and placed at ARGS["offset"]. Copies are '<name>.x' in a
new collection (re-running replaces it); originals are untouched. Runs inside Blender.

ARGS = {"collection": "ankle_assembly", "frame": "scs_left_ankle", "offset": [0, -0.9, 0.05], "scale": 3.0,
        "objects": {"left_foot": [0, 0, -14], "left_foot_bearing_plate": [0, -14, 0], ...}}
"""
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
COLL = args["collection"]
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
bpy.context.view_layer.update()
p0 = bpy.data.objects[args["frame"]].matrix_world.translation.copy()
S = args.get("scale", 1.0)
T = Matrix.Translation(Vector(args.get("offset", (0, 0, 0))) + p0) @ Matrix.Scale(S, 4) @ Matrix.Translation(-p0)
for name, d in args["objects"].items():
    o = bpy.data.objects[name]
    c = bpy.data.objects.new(name + ".x", o.data)
    coll.objects.link(c)
    c.matrix_world = T @ Matrix.Translation(Vector(d) / 1000) @ o.matrix_world
print(f"{COLL}: {len(args['objects'])} objects")
