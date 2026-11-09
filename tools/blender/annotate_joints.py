"""Draw a coloured arrow along each listed joint's axis, with a label. Runs inside Blender.

ARGS = {"robot": "duck_scs0009", "collection": "neck_explain",
        "joints": [{"joint": "neck_pitch", "label": "1 Neck pitch (ID30)", "color": [0.9, 0.2, 0.2, 1]}, ...],
        "length_mm": 46, "label_offset_mm": [0, 0, 6], "wire": ["duck_scs0009:head_roll"]}
Arrows follow the joint (parented to the joint's body). Objects named in "wire" are drawn as
wireframe so the servos inside are visible (run again with "wire": [] and "restore": [...] to undo).
"""
import bmesh
import bpy
from mathutils import Matrix, Vector

args = globals()["ARGS"]
R = args["robot"]
coll = bpy.data.collections.get(args["collection"]) or bpy.data.collections.new(args["collection"])
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in list(coll.objects):
    bpy.data.objects.remove(o, do_unlink=True)
bpy.context.view_layer.update()
MM = 0.001
L = args.get("length_mm", 46) * MM

for spec in args["joints"]:
    e = next(o for o in bpy.data.collections[R].objects if o.get("joint") == spec["joint"] and o.name.startswith(R + ":"))
    mat = bpy.data.materials.get(f"arrow_{spec['joint']}") or bpy.data.materials.new(f"arrow_{spec['joint']}")
    mat.diffuse_color = spec["color"]
    bm = bmesh.new()
    shaft = bmesh.ops.create_cone(bm, cap_ends=True, segments=16, radius1=0.0012, radius2=0.0012, depth=L)
    tip = bmesh.ops.create_cone(bm, cap_ends=True, segments=16, radius1=0.0032, radius2=0.0, depth=0.008)
    bmesh.ops.translate(bm, verts=tip["verts"], vec=Vector((0, 0, L / 2 + 0.004)))
    me = bpy.data.meshes.new(f"arrow_{spec['joint']}")
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    arrow = bpy.data.objects.new(f"arrow_{spec['joint']}", me)
    coll.objects.link(arrow)
    a = Vector(e["joint_axis"]).normalized()                        # axis in the joint body's frame
    rot = a.to_track_quat("Z", "Y").to_matrix().to_4x4()
    arrow.parent = e
    arrow.matrix_parent_inverse.identity()
    arrow.matrix_basis = rot
    txt = bpy.data.curves.new(f"lbl_{spec['joint']}", "FONT")
    txt.body = spec["label"]
    txt.size = args.get("label_size", 0.0075)
    txt.align_x = "LEFT"
    lab = bpy.data.objects.new(f"lbl_{spec['joint']}", txt)
    coll.objects.link(lab)
    txt.materials.append(mat)
    bpy.context.view_layer.update()
    if args.get("label_at") == "joint":           # label beside the joint origin (good for side views)
        base = e.matrix_world.translation.copy()
    else:
        base = arrow.matrix_world @ Vector((0, 0, L / 2 + 0.009))
    lab.location = base + Vector(spec.get("label_offset_mm", args.get("label_offset_mm", [0, 0, 4]))) * MM
    # colour this joint's servo to match (original material kept in a custom prop for restoring)
    servo = bpy.data.objects.get(f"scs_{spec['joint']}_case")
    if servo is not None and args.get("color_servos", True):
        if "orig_mat" not in servo:
            servo["orig_mat"] = servo.data.materials[0].name if servo.data.materials else ""
        servo.data.materials.clear()
        servo.data.materials.append(mat)
    lab.rotation_euler = args.get("label_rotation", (1.5707963, 0, 0))   # readable from the side (-Y)

for n in args.get("wire", []):
    o = bpy.data.objects[n]
    o.display_type = "WIRE"
for n in args.get("restore", []):
    bpy.data.objects[n].display_type = "TEXTURED"
if args.get("restore_servo_colors"):
    for o in bpy.data.objects:
        if "orig_mat" in o and o.type == "MESH":
            o.data.materials.clear()
            if o["orig_mat"]:
                o.data.materials.append(bpy.data.materials[o["orig_mat"]])
            del o["orig_mat"]
print(f"{len(args['joints'])} joint arrows in {coll.name}")
