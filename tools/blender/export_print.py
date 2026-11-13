"""Export one binary STL per PRINTED part (mm, world orientation as modelled) + a manifest. Runs inside Blender.

A printed part = a mesh carrying the `mass_g_solid_PLA` property (set by the part builders) in the given collections, plus any
extra names. Refuses to write a part that is not one closed piece (non-manifold edges or several shells) - the slicer would guess.
ARGS = {"collections": ["leg_v2", "neck_v2", "head_v2", "head_shell_v2", "body_v2"], "extra": ["horn_trim_jig_r6"],
        "out_dir": "/abs/exports/xgoduck_scs0009/print"}
"""
import json
import os
import struct

import bmesh
import bpy

args = globals()["ARGS"]
out = args["out_dir"]
os.makedirs(out, exist_ok=True)
names = []
for c in args["collections"]:
    col = bpy.data.collections.get(c)
    if col:
        names += [o.name for o in col.objects if o.type == "MESH" and "mass_g_solid_PLA" in o]
names += [n for n in args.get("extra", []) if n in bpy.data.objects]
manifest, refused = [], []
for n in sorted(set(names)):
    o = bpy.data.objects[n]
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.transform(o.matrix_world)
    nm = sum(not e.is_manifold for e in bm.edges)
    seen, shells = set(), 0
    for v in bm.verts:
        if v.index in seen:
            continue
        shells += 1
        st = [v]
        while st:
            w = st.pop()
            if w.index in seen:
                continue
            seen.add(w.index)
            st += [e.other_vert(w) for e in w.link_edges]
    if nm or shells != 1:
        refused.append(f"{n}: {nm} non-manifold edges, {shells} pieces")
        bm.free()
        continue
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.normal_update()
    path = os.path.join(out, f"{n}.stl")
    with open(path, "wb") as f:
        f.write(b"nanoduck printed part (mm)".ljust(80, b" "))
        f.write(struct.pack("<I", len(bm.faces)))
        for t in bm.faces:
            f.write(struct.pack("<3f", *t.normal))
            for v in t.verts:
                f.write(struct.pack("<3f", *(c * 1000.0 for c in v.co)))
            f.write(b"\0\0")
    manifest.append({"part": n, "file": os.path.basename(path), "triangles": len(bm.faces),
                     "volume_mm3": round(bm.calc_volume() * 1e9, 1), "mass_g_solid_PLA": o.get("mass_g_solid_PLA")})
    bm.free()
json.dump({"units": "mm", "parts": manifest, "refused": refused}, open(os.path.join(out, "manifest.json"), "w"), indent=1)
print(f"exported {len(manifest)} printed parts to {out}; refused {len(refused)}: {refused}")
