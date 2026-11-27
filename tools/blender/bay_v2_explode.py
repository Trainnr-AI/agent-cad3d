"""Exploded copy of the bay v2 assembly (runs inside Blender): collection `bay_v2_explode`, linked copies of `bay_v2` (built by
parts/bay_v2.py) moved apart in the order they go in, the whole set shifted aside (`at`, layout mm) so it does not overlap the robot.
Wires are left out (they would float between moved parts) unless ARGS["wires"]. Re-running replaces the collection.
ARGS = {"at": [0, 260, 0], "wires": false}
"""
import bpy
from mathutils import Matrix, Vector

args = globals().get("ARGS", {})
MM = 0.001
src, name = bpy.data.collections["bay_v2"], "bay_v2_explode"
old = bpy.data.collections.get(name)
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
coll = old or bpy.data.collections.new(name)
MAIN = bpy.data.scenes.get("Scene") or bpy.context.window_manager.windows[0].scene   # the model's scene, never an animation scene
if coll.name not in MAIN.collection.children:
    MAIN.collection.children.link(coll)
for _sc in bpy.data.scenes:                                          # (an animation scene that was active once may hold it: unlink)
    if _sc != MAIN and coll.name in _sc.collection.children:
        _sc.collection.children.unlink(coll)
trunk = bpy.data.objects["duck_scs0009:trunk_base"]
ML = trunk.matrix_world @ Matrix.Translation(Vector((0, 100, -118.8)) * MM)
AT = Vector(args.get("at", (0, 260, 0)))
GROUPS = [  # (name prefixes, offset layout mm) - first match wins
    (("bay_v2_lid",), (0, 0, 85)),
    (("v2_bay_usb_adapter_L", "v2_bay_usb_a_plug"), (25, 80, 20)),
    (("v2_unoq_", "v2_plug_"), (0, 80, 20)),
    (("v2_battery_3s",), (0, 0, 80)),
    (("v2_ubec",), (-30, 0, 55)),                                     # UBEC lifted out of its pod (2026-11-28)
    (("v2_splices_x3",), (0, -20, 40)),
    (("v2_switch_",), (0, -45, 0)),
    (("v2_bay_xt30_pair", "v2_bay_balance_plug"), (-30, 0, 0)),
    (("v2_imu", "v2_bay_imu_qwiic"), (-25, 0, 45)),
    (("v2_servo_adapter",), (0, 0, 0)),
    (("bay_v2_container",), (0, 0, 0)),
]
n = 0
for o in src.objects:
    if o.type == "CURVE" and not args.get("wires"):
        continue
    off = next((Vector(d) for pre, d in GROUPS if o.name.startswith(pre)), None)
    if off is None:
        continue
    c = o.copy()
    c.name = "x_" + o.name
    coll.objects.link(c)
    c.matrix_world = Matrix.Translation(ML.to_3x3() @ ((off + AT) * MM)) @ o.matrix_world
    n += 1
print("exploded copies:", n, "at", tuple(AT))
