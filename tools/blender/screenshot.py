"""Save a screenshot of the largest 3D viewport. Runs inside Blender.

ARGS = {"path": "/abs/out.png", "view": "FRONT"|"RIGHT"|"TOP"|"LEFT"|"BACK"|None, "persp": True}
"""
import bpy

args = globals().get("ARGS", {})
area = max((a for a in bpy.context.screen.areas if a.type == "VIEW_3D"), key=lambda a: a.width * a.height)
region = next(r for r in area.regions if r.type == "WINDOW")
with bpy.context.temp_override(area=area, region=region):
    if args.get("view"):
        bpy.ops.view3d.view_axis(type=args["view"])
        if args.get("persp", False):
            bpy.ops.view3d.view_persportho()
    bpy.ops.screen.screenshot_area(filepath=args["path"])
print("saved", args["path"])
