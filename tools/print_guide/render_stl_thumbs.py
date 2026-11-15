"""Thumbnails of STL files as they sit on the print bed (Workbench render, isometric from above, grey bed), for the print guides.
Runs in a clean, headless Blender (no project file is opened or touched):

  blender -b --factory-startup --python tools/print_guide/render_stl_thumbs.py -- <stl_dir> <out_dir>
"""
import glob
import math
import os
import sys

import bpy
from mathutils import Vector

src, dst = sys.argv[sys.argv.index("--") + 1:][:2]
os.makedirs(dst, exist_ok=True)
sc = bpy.context.scene
for engine in ("BLENDER_WORKBENCH", "BLENDER_WORKBENCH_NEXT"):
    try:
        sc.render.engine = engine
        break
    except TypeError:
        pass
sh = sc.display.shading
sh.light, sh.color_type = "STUDIO", "OBJECT"
sh.show_shadows, sh.show_cavity = True, True
sc.render.resolution_x, sc.render.resolution_y = 900, 700
sc.render.film_transparent = False
sc.world.color = (1, 1, 1)
sc.view_settings.view_transform = "Standard"

for f in sorted(glob.glob(os.path.join(src, "*.stl"))):
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.ops.wm.stl_import(filepath=f, global_scale=0.001)        # files are in mm
    part = bpy.context.selected_objects[0]
    part.color = (0.86, 0.66, 0.25, 1)
    pts = [part.matrix_world @ Vector(c) for c in part.bound_box]
    lo = Vector([min(p[i] for p in pts) for i in range(3)])
    hi = Vector([max(p[i] for p in pts) for i in range(3)])
    size = max(hi - lo)
    ctr = (lo + hi) / 2
    bpy.ops.mesh.primitive_plane_add(size=size * 1.8, location=(ctr.x, ctr.y, lo.z - 0.00005))
    bpy.context.active_object.color = (0.80, 0.82, 0.86, 1)
    cam_data = bpy.data.cameras.new("cam")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = size * 1.9
    cam_data.clip_start, cam_data.clip_end = size * 0.01, size * 20   # parts are ~0.01-0.08 m: the default 0.1 m near clip cut them
    cam = bpy.data.objects.new("cam", cam_data)
    sc.collection.objects.link(cam)
    d = Vector((math.cos(math.radians(-55)) * math.cos(math.radians(32)), math.sin(math.radians(-55)) * math.cos(math.radians(32)),
                math.sin(math.radians(32))))
    cam.location = ctr + d * size * 4
    cam.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
    sc.camera = cam
    sc.render.filepath = os.path.join(dst, os.path.basename(f)[:-4] + ".png")
    bpy.ops.render.render(write_still=True)
    print("rendered", os.path.basename(f))
