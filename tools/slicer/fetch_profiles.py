"""Fetch OrcaSlicer system profiles (machine / process / filament) from the OrcaSlicer GitHub repo at a pinned tag, flatten their
`inherits` chains and apply our overrides, so the CLI can load them. Plain Python (no Blender).

Why: the profiles bundled in the app are packed (profiles/BBL.opc) and the CLI cannot resolve a preset that still `inherits`
("can not resolve preset ... <datadir>/system"), so every file must be complete and keep `from: system`.

python3 tools/slicer/fetch_profiles.py <slicing.json> <out_dir>
  slicing.json: {"orca_tag": "v2.5.2", "vendor": "BBL", "machine": "...", "process": "...", "filament": "...",
                 "process_overrides": {...}}
Writes <out_dir>/machine.json, process_support.json, process_nosupport.json, filament.json.
"""
import json
import os
import sys
import urllib.parse
import urllib.request

cfg = json.load(open(sys.argv[1]))
out = sys.argv[2]
os.makedirs(out, exist_ok=True)
BASE = f"https://raw.githubusercontent.com/OrcaSlicer/OrcaSlicer/{cfg['orca_tag']}/resources/profiles/{cfg['vendor']}/"


def get(kind, name):
    with urllib.request.urlopen(BASE + kind + "/" + urllib.parse.quote(name) + ".json", timeout=30) as r:
        return json.load(r)


def flat(kind, name):
    d = get(kind, name)
    chain = [d]
    while d.get("inherits"):
        d = get(kind, d["inherits"])
        chain.append(d)
    merged = {}
    for c in reversed(chain):
        merged.update(c)
    merged.pop("inherits", None)
    merged["name"] = name
    merged["from"] = "system"
    print(f"{kind}: {' <- '.join(c.get('name', '?') for c in chain)}")
    return merged


json.dump(flat("machine", cfg["machine"]), open(os.path.join(out, "machine.json"), "w"), indent=1)
json.dump(flat("filament", cfg["filament"]), open(os.path.join(out, "filament.json"), "w"), indent=1)
proc = flat("process", cfg["process"])
proc.update({k: v for k, v in cfg.get("process_overrides", {}).items() if not k.startswith("_")})
for sup in ("1", "0"):
    p = dict(proc, enable_support=sup, name=proc["name"] + (" + support" if sup == "1" else " no support"))
    json.dump(p, open(os.path.join(out, "process_support.json" if sup == "1" else "process_nosupport.json"), "w"), indent=1)
print("profiles written to", out)
