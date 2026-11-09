"""Add (or update) a 3D text label. Runs inside Blender.

ARGS = {"name": "lbl_x", "text": "...", "location": (x, y, z), "size": 0.01, "collection": "labels"}
"""
import bpy

args = globals()["ARGS"]
coll = bpy.data.collections.get(args.get("collection", "labels"))
if coll is None:
    coll = bpy.data.collections.new(args.get("collection", "labels"))
    bpy.context.scene.collection.children.link(coll)
ob = bpy.data.objects.get(args["name"])
if ob is None:
    ob = bpy.data.objects.new(args["name"], bpy.data.curves.new(args["name"], "FONT"))
    coll.objects.link(ob)
ob.data.body = args["text"]
ob.data.size = args.get("size", 0.01)
ob.data.align_x = "CENTER"
ob.location = args["location"]
ob.rotation_euler = args.get("rotation", (1.5707963, 0, 0))   # upright, facing -Y
print("label", ob.name)
