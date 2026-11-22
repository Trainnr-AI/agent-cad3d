"""Totals of a sliced project (all plates): print time, grams, extruded volume, support, plus the solid model volume from the
export manifest. Plain Python.  python3 tools/slicer/summarize.py <project_slice_dir> <manifest.json> [out.json]"""
import glob
import json
import sys

d, man = sys.argv[1], json.load(open(sys.argv[2]))["parts"]
t = g = v = sv = sg = 0.0
plates = []
for f in sorted(glob.glob(f"{d}/plate_*.statistics.json")):
    s = json.load(open(f))
    su = s["summary"]
    t, g, v = t + su["time"], g + su["weight"], v + su["volume"]
    sv += sum(o["support_volume"] for o in s["objects"])
    sg += sum(o["support_weight"] for o in s["objects"])
    plates.append({"plate": f.split("/")[-1].split(".")[0], "hours": round(su["time"] / 3600, 2), "g": round(su["weight"], 1),
                   "objects": [o["name"] for o in s["objects"]]})
res = {"parts": len(man), "plates": plates, "hours": round(t / 3600, 1), "grams": round(g), "support_g": round(sg),
       "extruded_cm3": round(v / 1000, 1), "support_cm3": round(sv / 1000, 1),
       "solid_model_cm3": round(sum(p["volume_mm3"] for p in man) / 1000, 1)}
print(json.dumps(res, indent=1))
if len(sys.argv) > 3:
    json.dump(res, open(sys.argv[3], "w"), indent=1)
