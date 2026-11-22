"""Read sliced G-code (one part per slice dir): the support-interface extrusion points (x, y, z top of the interface layer, ~1 mm
apart) and the part's footprint from its WALLS above the first layer (bed coordinates, mm). Plain Python.

python3 tools/slicer/gcode_support_pts.py <out.json> <slice_dir>...
Keys in out.json = the last two path parts of each slice dir ("<orientation>/<part>").

Footprint from walls only: brim lines and tree-support fill are labelled as ordinary features (Top surface, infill) in the G-code,
so a footprint over all features is too big; walls trace the part itself (path centres: ~0.4 mm inside the mesh per side).
"""
import json
import math
import re
import sys

WALLS = ("Outer wall", "Inner wall", "Overhang wall")
out = {}
for d in sys.argv[2:]:
    z, feat, x, y = 0.0, None, None, None
    pts, wall = [], [1e9, 1e9, -1e9, -1e9]
    for ln in open(d.rstrip("/") + "/plate_1.gcode"):
        if ln.startswith("; Z_HEIGHT:"):
            z = float(ln.split(":")[1])
            continue
        if ln.startswith("; FEATURE:"):
            feat = ln.split(":", 1)[1].strip()
            continue
        if ln[:3] in ("G1 ", "G0 ", "G2 ", "G3 "):
            m = {k: float(v) for k, v in re.findall(r"([XYE])(-?[\d.]+)", ln)}
            nx, ny = m.get("X", x), m.get("Y", y)
            if m.get("E", 0) > 0 and x is not None and nx is not None:
                if feat == "Support interface":
                    n = max(1, int(math.hypot(nx - x, ny - y) / 1.0))
                    pts += [(round(x + (nx - x) * i / n, 3), round(y + (ny - y) * i / n, 3), z) for i in range(n + 1)]
                elif z > 0.3 and feat in WALLS:
                    wall = [min(wall[0], nx), min(wall[1], ny), max(wall[2], nx), max(wall[3], ny)]
            x, y = nx, ny
    out["/".join(d.rstrip("/").split("/")[-2:])] = {"wall_bbox": wall, "pts": pts}
json.dump(out, open(sys.argv[1], "w"))
print(f"{len(out)} slices read, {sum(len(v['pts']) for v in out.values())} support-interface points")
