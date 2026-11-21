"""Turn an assembly animation's sequence (data/build_args/*_assembly_anim.json, for assembly_anim.py) into assembly_path_check
steps, so the moves shown are the moves tested. Plain Python (no Blender).

  python3 tools/blender/anim_to_paths.py <anim.json> <out_paths.json> [--allow allow.json]

Every leg of every move is a step; it is tested against everything placed so far (the anim's "context" + "against_extra" + the
parts moved before), each group at the offset it has at that moment (a part riding on another one that is still in the air,
e.g. the regulator sliding into the lid before the lid goes down, is tested against the lid where the lid is). Wires are not tested.
allow.json: {"part": ["object it may touch at its final place", ...]} (designed fits: screw threads in their pilots etc.).
"""
import json
import sys
from math import sqrt

anim = json.load(open(sys.argv[1]))
allow = json.load(open(sys.argv[sys.argv.index("--allow") + 1])) if "--allow" in sys.argv else {}
engage = anim.get("engage_mm", {})
pos = {n: (0.0, 0.0, 0.0) for n in anim.get("context", []) + anim.get("against_extra", [])}   # object -> its current offset (mm)
steps = []
sub = lambda a, b: tuple(a[i] - b[i] for i in range(3))
for si, st in enumerate(anim["steps"]):
    for mv in st.get("moves", []):
        moving = mv["objects"]
        if mv.get("split_axis") is not None or mv.get("screw_pieces"):   # screws: each drives in along its own axis = the driver line,
            for n in moving:                                              # checked by checks/driver_access.py; here only their place
                pos[n] = tuple(float(c) for c in mv.get("base", (0.0, 0.0, 0.0)))
            continue
        path = [tuple(p) for p in mv["path"]]
        if any(abs(c) > 1e-9 for c in path[-1]) and not mv.get("hold"):
            path.append((0.0, 0.0, 0.0))
        groups = {}
        for n, o in pos.items():
            if n not in moving:
                groups.setdefault(o, []).append(n)
        for n in moving:
            for k in range(len(path) - 1):
                a, b = path[k], path[k + 1]
                d = sub(a, b)
                L = sqrt(sum(c * c for c in d))
                if L < 1e-9:
                    continue
                for off, against in groups.items():
                    base = sub(b, off)
                    s = {"name": f"bay {si + 1}: {n} leg {k + 1}/{len(path) - 1}" + (f" (vs parts at {list(off)})" if any(off) else ""),
                         "part": n, "dir": [c / L for c in d], "start_mm": round(L, 3), "against": sorted(against),
                         "allow_final": allow.get(n, []), "engage_mm": engage.get(n, 4.5)}
                    if any(abs(c) > 1e-9 for c in base):
                        s["base_mm"] = list(base)
                    steps.append(s)
        for n in moving:
            pos[n] = tuple(float(c) for c in path[-1]) if path else (0.0, 0.0, 0.0)
json.dump({"lib": "<repo>/tools/blender/overlap_sample.py", "step_mm": 0.5, "steps": steps}, open(sys.argv[2], "w"), indent=1)
print(f"{len(steps)} path steps -> {sys.argv[2]}")
