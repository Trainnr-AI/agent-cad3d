"""Frame objects in every 3D viewport. Runs inside Blender.

ARGS = {"collections": ["name", ...]}   # omit to frame everything; "shading": "SOLID"|"MATERIAL"
"""
import bpy

args = globals().get("ARGS", {})
names = args.get("collections")
obs = [o for c in names for o in bpy.data.collections[c].all_objects] if names else list(bpy.context.scene.objects)

for area in bpy.context.screen.areas:
    if area.type != "VIEW_3D":
        continue
    region = next(r for r in area.regions if r.type == "WINDOW")
    space = area.spaces.active
    space.shading.type = args.get("shading", "SOLID")
    space.shading.color_type = "MATERIAL"
    space.clip_start = 0.0005
    with bpy.context.temp_override(area=area, region=region):
        bpy.ops.object.select_all(action="DESELECT")
        for o in obs:
            if o.visible_get():
                o.select_set(True)
        bpy.ops.view3d.view_selected()
        bpy.ops.object.select_all(action="DESELECT")
print(f"framed {len(obs)} objects")
