---
name: slice-print
description: Slice a project's printed parts with OrcaSlicer before printing - time, grams, volume, supports, and whether support lands on bearing seats / servo pockets / horn pockets; picks each part's print orientation. Use when asked to "slice", "test the print", "how long / how much plastic", "print orientation", "where are the supports", or before sending parts to a printing service.
---

# Slice and check prints (OrcaSlicer CLI)

Everything runs on a **copy** of the project's .blend in headless Blender, so the user's live session is never touched and it
works when Blender is closed. Tools are in `tools/slicer/` (see its README for each script).

## Run

1. `scripts/setup_orca.sh` once (macOS: brew cask; Linux/WSL: AppImage to `~/.local/bin`).
2. Check the project's config, e.g. `projects/xgoduck_scs0009/data/build_args/slicing.json`: printer / process / filament profile
   names (OrcaSlicer system profile names, at the pinned `orca_tag`), `process_overrides` (must match what the print service
   will use; for the nanoduck they are the test-print guide's settings), `orientation_overrides` (each with its reason in `_why`).
   Ask the user which printer and material the service uses if it is not in the config; otherwise keep the config.
3. `tools/slicer/slice_project.sh <slicing.json>` - about 20 min for 29 parts. Run it in the background and tell the user the
   duration up front. Output (git-ignored) in `exports/<project>/slicing/`:
   - `project/<project>_parts.3mf` - the chosen orientations on plates, sliced: `open -a OrcaSlicer <file>` to show the user
     (Preview tab + layer slider shows supports and bridges);
   - `summary.json` - hours, grams, extruded / support / solid cm3, plates;
   - `orientation_table.md`, `orientation_scores.json` - all 6 orientations per part, with what the supports touch;
   - `chosen_contacts.json` (`fit_faces`: axial face = the shoulder a bearing sits on, radial face = the bore / a boss; `r_*` =
     radius from the bearing axis, so r 7.4-7.5 on a 6702 = the Ø15 boss through it, r 10.5 = the Ø21 bore).
4. Report: totals; parts that print clean; every remaining support contact on a fit surface, by part and surface; what was
   NOT checked (fit tolerance = only a test print; support removal). Do not change the design without asking.
5. If the chosen orientations or results change, copy `orientation_table.md`, `chosen_orientation.json` and `summary.json` to
   `projects/<project>/print/slicing/` and update the README there.

## Gotchas (learned the hard way)

- The app's bundled profiles are packed (`*.opc`); `fetch_profiles.py` downloads the JSONs from GitHub at the pinned tag and
  flattens `inherits` - a profile that still inherits fails with "can not resolve preset ... datadir/system".
- `--export-3mf` takes a bare filename (relative to `--outputdir`). `--strict` only exits 156 on macOS (no warning text).
- G-code features: brim and tree-support fill are labelled as ordinary features, so the part footprint comes from walls only.
  `bbox_fit_err_mm` ~0.84 is normal; >1.5 means that slice's contacts are misplaced (ranked last automatically).
- "Potentially lost branch!" in a log = a tree-support branch OrcaSlicer is unsure about; look at that part in the GUI.
- Least support is NOT the best orientation: it put support into bearing seats. Always rank by fit-surface contacts.
- Scripts sent to a LIVE Blender take paths only from ARGS, never sys.argv (that once overwrote the Blender binary).
