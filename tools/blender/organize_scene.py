"""Tidy the scene into a few clearly separated, labelled groups; archive (hide) everything else.
Runs inside Blender. Nothing is deleted: archived collections are moved under one parent
collection and excluded from the view layer (tick them back on in the Outliner to see them).

ARGS = {
  "show": [  # in display order along -Y, each placed at its own y
    {"collection": "xgoduck_original", "root": "xgoduck_original:trunk_base", "y": 0.12, "label": "Original XGO-Duck (HD-1910)"},
    {"collection": "duck_scs0009", "root": "duck_scs0009:trunk_base", "y": -0.12, "label": "New duck: SCS0009, 0.92x",
     "also": ["leg_v1"], "hide_prefix": ["duck_scs0009:left_", "duck_scs0009:right_"]},
    {"collection": "duck_v1_exploded", "y": -0.40, "label": "New duck - exploded"}],
  "archive_name": "zz_archive (hidden)"}
"""
import bpy
from mathutils import Vector

args = globals()["ARGS"]
scene = bpy.context.scene
keep = set()
for item in args["show"]:
    keep.add(item["collection"])
    keep.update(item.get("also", []))
keep.add("labels_main")

# ---- archive everything else that sits directly under the scene collection
arch_name = args.get("archive_name", "zz_archive (hidden)")
arch = bpy.data.collections.get(arch_name) or bpy.data.collections.new(arch_name)
if arch.name not in scene.collection.children:
    scene.collection.children.link(arch)
for c in list(scene.collection.children):
    if c.name in keep or c == arch:
        continue
    scene.collection.children.unlink(c)
    arch.children.link(c)
# loose objects in the scene root (Cube, Light, Camera, agent_cam): hide the helpers, keep the light
for o in scene.collection.objects:
    if o.type in {"MESH", "CAMERA"}:
        o.hide_set(True)
        o.hide_render = True


def layer_coll(name, lc=None):
    lc = lc or bpy.context.view_layer.layer_collection
    if lc.name == name:
        return lc
    for ch in lc.children:
        r = layer_coll(name, ch)
        if r:
            return r


layer_coll(arch.name).exclude = True
bpy.context.view_layer.update()

# ---- place the shown groups along Y and label them
labels = bpy.data.collections.get("labels_main") or bpy.data.collections.new("labels_main")
if labels.name not in scene.collection.children:
    scene.collection.children.link(labels)
for o in list(labels.objects):
    bpy.data.objects.remove(o, do_unlink=True)


def bbox(objs):
    pts = [o.matrix_world @ Vector(c) for o in objs if o.type == "MESH" and o.visible_get() for c in o.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return lo, hi


for item in args["show"]:
    coll = bpy.data.collections[item["collection"]]
    layer_coll(coll.name).exclude = False
    for c in item.get("also", []):
        layer_coll(c).exclude = False
    for o in coll.all_objects:
        if any(o.name.startswith(p) for p in item.get("hide_prefix", [])):
            o.hide_set(True)
            o.hide_render = True
    objs = list(coll.all_objects) + [o for c in item.get("also", []) for o in bpy.data.collections[c].all_objects]
    bpy.context.view_layer.update()
    lo, hi = bbox(objs)
    dy = item["y"] - (lo.y + hi.y) / 2
    if item.get("root"):                       # posable robot: move its root body (children follow)
        bpy.data.objects[item["root"]].location.y += dy
    else:                                      # loose copies: move every object
        for o in coll.all_objects:
            if o.parent is None:
                o.location.y += dy
    bpy.context.view_layer.update()
    lo, hi = bbox(objs)
    txt = bpy.data.curves.new(f"lbl_{coll.name}", "FONT")
    txt.body = item["label"]
    txt.size = 0.012
    txt.align_x = "CENTER"
    lab = bpy.data.objects.new(f"lbl_{coll.name}", txt)
    labels.objects.link(lab)
    lab.location = ((lo.x + hi.x) / 2, (lo.y + hi.y) / 2, hi.z + 0.02)
    lab.rotation_euler = (1.5707963, 0, 1.5707963)        # readable from the front (+X)
    print(f"{item['label']}: y {(lo.y + hi.y) / 2:+.3f} m, size {tuple(round((hi - lo)[i] * 1000) for i in range(3))} mm")

print("archived:", ", ".join(c.name for c in arch.children))
