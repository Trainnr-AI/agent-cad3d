"""Render a framed close-up of some objects from a given angle. Runs inside Blender.

Uses a dedicated camera ("agent_cam") and a workbench render, so it never moves the user's 3D
view and doesn't depend on the window repainting (screenshots can return stale frames while
Blender is in the background). Temporarily hides other objects (isolate) and restores everything.

ARGS = {"collection": "joint_kit" (or "objects": [names]), "path": "/abs/out.png",
        "elev_deg": 62, "azim_deg": 125, "pad": 1.1, "isolate": True, "hide_prefix": [],
        "size": [1600, 1000], "also": ["labels_collection"]}
elev/azim follow Blender's view rotation convention: elev 0 = looking straight down,
90 = horizontal; azim rotates about world Z.
"""
import math

import bpy
from mathutils import Euler, Vector

args = globals()["ARGS"]
sc = bpy.context.scene
obs = [bpy.data.objects[n] for n in args["objects"]] if args.get("objects") else list(bpy.data.collections[args["collection"]].all_objects)
for extra in args.get("also", []):                     # further collections to frame with it (e.g. its labels)
    obs += list(bpy.data.collections[extra].all_objects)
obs_set = set(obs)

changed = []                                   # (object, previous hide_render)
for o in sc.objects:
    hide = (args.get("isolate") and o not in obs_set) or any(o.name.startswith(p) for p in args.get("hide_prefix", [])) \
        or not o.visible_get()
    if hide != o.hide_render:
        changed.append((o, o.hide_render))
        o.hide_render = hide
bpy.context.view_layer.update()

pts = [o.matrix_world @ Vector(c) for o in obs if not o.hide_render and o.type in {"MESH", "CURVE", "FONT"} for c in o.bound_box]
centre = sum(pts, Vector()) / len(pts)
radius = max((p - centre).length for p in pts)

cam = bpy.data.objects.get("agent_cam")
if cam is None:
    cam = bpy.data.objects.new("agent_cam", bpy.data.cameras.new("agent_cam"))
    sc.collection.objects.link(cam)
cam.data.lens, cam.data.sensor_width = 50, 36
cam.data.clip_start, cam.data.clip_end = 0.001, 50
rot = Euler((math.radians(args.get("elev_deg", 62)), 0, math.radians(args.get("azim_deg", 125))), "XYZ")
W, H = args.get("size", [1600, 1000])
fov = 2 * math.atan(cam.data.sensor_width / 2 / cam.data.lens) * min(1.0, H / W)
dist = radius / math.sin(fov / 2) * args.get("pad", 1.1)
cam.rotation_euler = rot
cam.location = centre + rot.to_matrix() @ Vector((0, 0, dist))

keep = (sc.camera, sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage,
        sc.render.engine, sc.display.shading.color_type)
sc.camera = cam
sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = W, H, 100
sc.render.engine = "BLENDER_WORKBENCH"
sc.display.shading.color_type = "MATERIAL"
bpy.ops.render.render(write_still=False)
bpy.data.images["Render Result"].save_render(filepath=args["path"])

(sc.camera, sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage,
 sc.render.engine, sc.display.shading.color_type) = keep
for o, prev in changed:
    o.hide_render = prev
print("saved", args["path"])
