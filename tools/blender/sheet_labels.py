"""Labels with leader lines for an exploded sheet (runs inside Blender). Each label is a 3D text facing -Y, placed in a column left or
right of the sheet at its target's height, with a thin leader line from the text to the target's nearest bounding-box side.
Re-running replaces the collection.

ARGS = {"collection": "bay_labels", "sheet": "bay_exploded" (collection the targets are in), "size_mm": 5.0, "gap_mm": 40.0,
        "labels": [{"target": "electronics_enclosure.x.001", "text": "1  ...", "side": "L" | "R", "dz_mm": 0.0}, ...]}
"""
import bpy
from mathutils import Vector

args = globals()["ARGS"]
COLL = args["collection"]
old = bpy.data.collections.get(COLL)
if old:
    for o in list(old.objects):
        d = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if d is not None and d.users == 0:
            (bpy.data.curves if isinstance(d, bpy.types.Curve) else bpy.data.meshes).remove(d)
    bpy.data.collections.remove(old)
coll = bpy.data.collections.new(COLL)
bpy.context.scene.collection.children.link(coll)
MM = 0.001
sheet = [o for o in bpy.data.collections[args["sheet"]].objects if o.visible_get()]


def bbox(objs):
    pts = [o.matrix_world @ Vector(c) for o in objs for c in o.bound_box]
    return Vector([min(p[i] for p in pts) for i in range(3)]), Vector([max(p[i] for p in pts) for i in range(3)])


slo, shi = bbox(sheet)
gap, size = args.get("gap_mm", 40.0) * MM, args.get("size_mm", 5.0) * MM
y_front = slo.y - 5 * MM                                                     # labels a little in front of the sheet (toward the camera at -Y)
mat_txt = bpy.data.materials.get("label_text") or bpy.data.materials.new("label_text")
mat_txt.diffuse_color = (0.95, 0.95, 0.95, 1)
mat_ln = bpy.data.materials.get("label_line") or bpy.data.materials.new("label_line")
mat_ln.diffuse_color = (0.9, 0.85, 0.3, 1)
for i, lab in enumerate(args["labels"]):
    t = bpy.data.objects[lab["target"]]
    lo, hi = bbox([t])
    z = (lo.z + hi.z) / 2 + lab.get("dz_mm", 0.0) * MM
    left = lab.get("side", "L") == "L"
    x_txt = slo.x - gap if left else shi.x + gap
    cu = bpy.data.curves.new(f"lbl_{i:02d}", "FONT")
    cu.body = lab["text"]
    cu.size = size
    cu.align_x = "RIGHT" if left else "LEFT"
    cu.align_y = "CENTER"
    ob = bpy.data.objects.new(f"lbl_{i:02d}", cu)
    ob.location = (x_txt, y_front, z)
    ob.rotation_euler = (1.5707963, 0, 0)
    ob.data.materials.append(mat_txt)
    ob.color = (0.95, 0.95, 0.95, 1)
    coll.objects.link(ob)
    # leader: from just beside the text to the target's near side, at the same height
    x0 = x_txt + (2 * MM if left else -2 * MM)
    x1 = lo.x if left else hi.x
    me = bpy.data.meshes.new(f"lbl_line_{i:02d}")
    a, b = Vector((x0, y_front, z)), Vector((x1, (lo.y + hi.y) / 2, z))
    d = (b - a).normalized()
    side = Vector((0, 0, 0.25 * MM))
    vs = [a - side, a + side, b + side, b - side]
    me.from_pydata(vs, [], [(0, 1, 2, 3)])
    ln = bpy.data.objects.new(f"lbl_line_{i:02d}", me)
    ln.data.materials.append(mat_ln)
    ln.color = (0.9, 0.85, 0.3, 1)
    coll.objects.link(ln)
print(f"{COLL}: {len(args['labels'])} labels")
