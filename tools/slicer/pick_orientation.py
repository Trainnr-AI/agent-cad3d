"""Pick each part's print orientation from the 6-orientation slices: fewest support contacts on FIT surfaces (next to a bearing,
servo case, horn, nut, board), then fewest on other mating faces, then least support. Plain Python.

python3 tools/slicer/pick_orientation.py <slicing.json> <contacts.json> <oslice_dir> <out_dir>
Writes <out_dir>/orientation_scores.json (all 6 per part), chosen_orientation.json ({part: tag}) and orientation_table.md.
`orientation_overrides` in slicing.json wins over the score (give the reason next to it in that file).
Orientation tags: px = model +X points up when printing, mx = -X up, ... (the STL is turned so that axis becomes +Z).
"""
import json
import os
import re
import sys

cfg = json.load(open(sys.argv[1]))
C = json.load(open(sys.argv[2]))
SL, OUT = sys.argv[3], sys.argv[4]
os.makedirs(OUT, exist_ok=True)
FIT = re.compile(cfg.get("fit_regex", r"_brg_|horn|_case|spline|nut|pcb|imu|cam_ov5640|ubec"))
OVR = {k: v for k, v in cfg.get("orientation_overrides", {}).items() if not k.startswith("_")}
TAGS = ["px", "mx", "py", "my", "pz", "mz"]


def kind(names):
    k = "free"
    for n in names.split(","):
        if n == "-":
            continue
        if FIT.search(n):
            return "fit"
        k = "mate"
    return k


parts = sorted({k.split("/")[1] for k in C})
scores, chosen, lines = {}, {}, []
for p in parts:
    rows = []
    for t in TAGS:
        if f"{t}/{p}" not in C:
            continue
        r = C[f"{t}/{p}"]
        st = json.load(open(f"{SL}/{t}/{p}/plate_1.statistics.json"))
        n = {"fit": 0, "mate": 0}
        what = {}
        for k, g in r["groups"].items():
            kk = kind(k)
            if kk != "free":
                n[kk] += g["n"]
                what[k] = g["n"]
        rows.append({"tag": t, "fit_contacts": n["fit"], "mate_contacts": n["mate"],
                     "support_g": round(st["objects"][0]["support_weight"], 2), "part_g": round(st["summary"]["weight"], 1),
                     "minutes": round(st["summary"]["time"] / 60), "align_err_mm": r["bbox_fit_err_mm"], "touches": what,
                     "trusted": r["bbox_fit_err_mm"] < 1.5})
    # untrusted alignment (contacts misplaced) ranks after every trusted one
    rows.sort(key=lambda x: (not x["trusted"], x["fit_contacts"], x["mate_contacts"], x["support_g"]))
    scores[p] = rows
    pick = OVR.get(p, rows[0]["tag"])
    chosen[p] = pick
    b = next(x for x in rows if x["tag"] == pick)
    top = "; ".join(f"{k} {v}" for k, v in sorted(b["touches"].items(), key=lambda kv: -kv[1])[:3])
    lines.append(f"| {p} | {pick}{' (override)' if p in OVR else ''} | {b['fit_contacts']} | {b['mate_contacts']} | "
                 f"{b['support_g']} | {b['part_g']} | {b['minutes']} | {top} |")
json.dump(scores, open(f"{OUT}/orientation_scores.json", "w"), indent=1)
json.dump(chosen, open(f"{OUT}/chosen_orientation.json", "w"), indent=1)
hdr = ("| part | up | fit-surface contacts | other mating contacts | support g | g (alone) | min (alone) | touches |\n"
       "|---|---|---|---|---|---|---|---|")
open(f"{OUT}/orientation_table.md", "w").write(hdr + "\n" + "\n".join(lines) + "\n")
print(hdr)
print("\n".join(lines))
