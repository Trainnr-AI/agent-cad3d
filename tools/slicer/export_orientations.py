"""Export every printed part in each of the 6 axis orientations (that model axis up), on the bed, via export_print.py.
Runs inside Blender. ARGS = {"collections": [...], "out_dir": ".../orient", "export_tool": ".../tools/blender/export_print.py",
"orientations": ["px","mx","py","my","pz","mz"] (optional)}. Writes <out_dir>/<tag>/<part>.stl + manifest.json.
"""
import bpy

A = ARGS  # noqa: F821
TAG = {"px": "+x", "mx": "-x", "py": "+y", "my": "-y", "pz": "+z", "mz": "-z"}
names = set()
for c in A["collections"]:
    col = bpy.data.collections.get(c)
    if col:
        names |= {o.name for o in col.objects if o.type == "MESH" and "mass_g_solid_PLA" in o}
names -= set(A.get("exclude", []))
for tag in A.get("orientations", list(TAG)):
    g = {"ARGS": {"collections": A["collections"], "exclude": A.get("exclude", []), "out_dir": f"{A['out_dir']}/{tag}",
                  "print_up": {n: TAG[tag] for n in names}, "on_bed": True}}
    exec(compile(open(A["export_tool"]).read(), A["export_tool"], "exec"), g)
