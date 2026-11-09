"""Import an MJCF robot into the live Blender scene as a posable rig.

Runs inside Blender. Set ARGS before executing, e.g. via scripts/blender_doctor.py:
    ARGS = {"mjcf": "/abs/path/robot.xml", "collection": "xgoduck_original",
            "offset": (0, 0, 0), "color": (0.9, 0.9, 0.9, 1), "scale": 1.0}

Each MJCF body becomes an Empty named `<collection>:<body>`, parented like the kinematic
tree. For a hinge joint, rotate that Empty about its local joint axis (stored as the
custom properties `joint`, `joint_axis`, `joint_range`) to pose the robot. Visual meshes
are children of their body Empty. Re-running replaces the collection.
"""
import os
import xml.etree.ElementTree as ET

import bpy
from mathutils import Quaternion, Vector

args = globals().get("ARGS", {})
MJCF = args["mjcf"]
COLL = args.get("collection", os.path.splitext(os.path.basename(MJCF))[0])
OFFSET = Vector(args.get("offset", (0, 0, 0)))
COLOR = args.get("color", (0.85, 0.85, 0.85, 1.0))
SCALE = float(args.get("scale", 1.0))   # uniform: body offsets and visual meshes

root = ET.parse(MJCF).getroot()
meshdir = os.path.join(os.path.dirname(MJCF), root.find("compiler").get("meshdir", ""))
mesh_files = {
    m.get("name") or os.path.splitext(os.path.basename(m.get("file")))[0]: os.path.join(meshdir, m.get("file"))
    for m in root.iter("mesh")
}


def vec(s, default):
    return Vector(map(float, s.split())) if s else Vector(default)


# Fresh collection
old = bpy.data.collections.get(COLL)
if old:
    for ob in list(old.all_objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)

mat = bpy.data.materials.get(f"{COLL}_mat") or bpy.data.materials.new(f"{COLL}_mat")
mat.diffuse_color = COLOR

loaded = {}


def load_mesh(name):
    if name not in loaded:
        before = set(bpy.data.objects)
        bpy.ops.wm.stl_import(filepath=mesh_files[name])
        ob = (set(bpy.data.objects) - before).pop()
        me = ob.data
        me.name = f"{COLL}:{name}"
        bpy.data.objects.remove(ob, do_unlink=True)
        if not me.materials:
            me.materials.append(mat)
        loaded[name] = me
    return loaded[name]


def build(body_el, parent_empty):
    name = body_el.get("name")
    e = bpy.data.objects.new(f"{COLL}:{name}", None)
    e.empty_display_type = "ARROWS"
    e.empty_display_size = 0.01
    coll.objects.link(e)
    e.parent = parent_empty
    e.rotation_mode = "QUATERNION"
    e.location = vec(body_el.get("pos"), (0, 0, 0)) * SCALE
    e.rotation_quaternion = Quaternion(vec(body_el.get("quat"), (1, 0, 0, 0)))
    if parent_empty is None:
        e.location += OFFSET
    j = body_el.find("joint")
    if j is not None and j.get("type", "hinge") == "hinge":
        e["joint"] = j.get("name")
        e["joint_axis"] = list(vec(j.get("axis"), (0, 0, 1)))
        if j.get("range"):
            e["joint_range"] = [float(x) for x in j.get("range").split()]
    for g in body_el.findall("geom"):
        if g.get("type") == "mesh" and g.get("class", "visual") == "visual":
            ob = bpy.data.objects.new(f"{COLL}:{g.get('mesh')}", load_mesh(g.get("mesh")))
            coll.objects.link(ob)
            ob.parent = e
            ob.rotation_mode = "QUATERNION"
            ob.location = vec(g.get("pos"), (0, 0, 0)) * SCALE
            ob.scale = (SCALE, SCALE, SCALE)
            ob.rotation_quaternion = Quaternion(vec(g.get("quat"), (1, 0, 0, 0)))
    for child in body_el.findall("body"):
        build(child, e)


for top in root.find("worldbody").findall("body"):
    build(top, None)

bpy.context.view_layer.update()
pts = [ob.matrix_world @ Vector(c) for ob in coll.objects if ob.type == "MESH" for c in ob.bound_box]
lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
joints = [ob["joint"] for ob in coll.objects if "joint" in ob]
print(f"{COLL}: {len(joints)} joints, {len(loaded)} meshes, bbox size (m) {tuple(round(v, 4) for v in hi - lo)}, min {tuple(round(v, 4) for v in lo)}")
