# tools/slicer - slice the printed parts with OrcaSlicer before printing

Answers: how long, how many grams / cm3, where the supports go, and whether support lands on a surface that must fit (bearing
seat, servo pocket, horn pocket, nut trap, board slot) - and picks each part's print orientation from that. One command:

```
scripts/setup_orca.sh                                                   # once: OrcaSlicer 2.5.2
tools/slicer/slice_project.sh projects/xgoduck_scs0009/data/build_args/slicing.json   # ~20 min, output in exports/<project>/slicing/
open -a OrcaSlicer exports/xgoduck_scs0009/slicing/project/xgoduck_scs0009_parts.3mf
```

Works on a copy of the .blend in headless Blender (the live session is never touched). The Claude skill
`.claude/skills/slice-print/SKILL.md` describes how to run and report it.

| Script | Runs in | Does |
|---|---|---|
| `slice_project.sh` | shell | the whole pipeline below |
| `fetch_profiles.py` | python | machine / process / filament profiles from the OrcaSlicer repo at the pinned tag, `inherits` flattened, overrides applied (the app's own profiles are packed and unusable by the CLI) |
| `orca.sh` | shell | one OrcaSlicer CLI slice with those profiles (`support` / `nosupport`), statistics JSON |
| `export_orientations.py` | Blender | every printed part (mesh with `mass_g_solid_PLA`) as STL in all 6 axis orientations, on the bed (uses `tools/blender/export_print.py`, which refuses parts that are not one closed piece) |
| `gcode_support_pts.py` | python | support-interface points and the part's wall footprint from the G-code |
| `support_contacts.py` | Blender | puts each support point back on the model and names the parts its contact surface mates with (`details`: shoulder vs bore, radius from the bearing axis) |
| `pick_orientation.py` | python | per part: fewest fit-surface contacts, then other mating faces, then least support; `orientation_overrides` win |
| `summarize.py` | python | hours, grams, extruded / support / solid cm3 of the sliced project |
| `run_headless.py` | Blender | runs any ARGS-style tool script with `Blender -b copy.blend --python run_headless.py -- script.py '<json>'` |

Orientation tags: `px` = the model's +X points up when printing, `mx` = -X up, `py`, `my`, `pz`, `mz` likewise.

What it cannot tell: whether fits are right (that depends on the printer - the test coupon decides), how cleanly support snaps
off, strength.
