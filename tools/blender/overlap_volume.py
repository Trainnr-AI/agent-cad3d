"""Volume shared by two mesh objects (boolean INTERSECT on temporary copies). Runs inside Blender.
Tells real penetration (volume > 0) apart from parts that only touch (volume ~ 0).

ARGS = {"pairs": [["dm_servo_case", "dm_holder"], ...]}
"""
import bmesh
import bpy

args = globals()["ARGS"]


def volume(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.transform(ob.matrix_world)
    v = abs(bm.calc_volume())
    bm.free()
    return v


for a_name, b_name in args["pairs"]:
    a, b = bpy.data.objects[a_name], bpy.data.objects[b_name]
    tmp = bpy.data.objects.new("_ovl_tmp", a.data.copy())
    bpy.context.scene.collection.objects.link(tmp)
    tmp.matrix_world = a.matrix_world
    mod = tmp.modifiers.new("i", "BOOLEAN")
    mod.operation, mod.object, mod.solver = "INTERSECT", b, "EXACT"
    with bpy.context.temp_override(object=tmp, active_object=tmp):
        bpy.ops.object.modifier_apply(modifier=mod.name)
    v = volume(tmp) * 1e9
    bpy.data.meshes.remove(tmp.data)
    print(f"  {a_name} ∩ {b_name}: {v:8.3f} mm³ {'(touching only)' if v < 0.05 else '<-- REAL OVERLAP'}")
