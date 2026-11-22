#!/usr/bin/env bash
# Whole slicing check for a project, end to end, on a COPY of the .blend (the live Blender session is never touched):
#   1 profiles  2 export each printed part in 6 orientations  3 slice them all  4 map supports back onto the model
#   5 pick each part's orientation  6 slice the chosen set as one project (.3mf, plates) + totals  7 fit-face details
#
#   tools/slicer/slice_project.sh <slicing.json> [out_dir]
# slicing.json (e.g. projects/xgoduck_scs0009/data/build_args/slicing.json) names the .blend, the collections, the printer /
# process / filament profiles and overrides. out_dir defaults to exports/<project>/slicing/ (git-ignored). ~6 min for 29 parts.
# Env: BLENDER (binary), ORCA (binary), JOBS (parallel slices, default 6).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
CFG=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
T=$ROOT/tools/slicer
val() { python3 -c "import json,sys; print(json.load(open('$CFG'))[sys.argv[1]])" "$1"; }
PROJ=$(val project)
OUT=${2:-$ROOT/exports/$PROJ/slicing}
BLEND=$ROOT/$(val blend)
COLS=$(python3 -c "import json; print(json.dumps(json.load(open('$CFG'))['collections']))")
JOBS=${JOBS:-6}
if [ -z "${BLENDER:-}" ]; then
  for c in /Applications/Blender.app/Contents/MacOS/Blender "$(command -v blender 2>/dev/null)"; do
    [ -n "$c" ] && [ -x "$c" ] && BLENDER=$c && break
  done
fi
mkdir -p "$OUT"
echo "== 1 profiles"; python3 "$T/fetch_profiles.py" "$CFG" "$OUT/profiles"
echo "== 2 export (copy of $BLEND)"
cp "$BLEND" "$OUT/model_copy.blend"
"$BLENDER" -b "$OUT/model_copy.blend" --python "$ROOT/tools/blender/run_headless.py" -- "$T/export_orientations.py" \
  "{\"collections\": $COLS, \"out_dir\": \"$OUT/orient\", \"export_tool\": \"$ROOT/tools/blender/export_print.py\"}" 2>&1 | grep -E "exported|Error"
echo "== 3 slice every part x 6 orientations ($JOBS at a time)"
ls "$OUT"/orient/*/*.stl | xargs -P "$JOBS" -I{} sh -c \
  'f="{}"; tag=$(basename "$(dirname "$f")"); n=$(basename "$f" .stl); "$0" "$1/profiles" support "$1/oslice/$tag/$n" "$f" || echo "FAILED $tag/$n"' \
  "$T/orca.sh" "$OUT"
echo "== 4 support contacts"
(cd "$OUT/oslice" && python3 "$T/gcode_support_pts.py" "$OUT/orient_pts.json" */*)
"$BLENDER" -b "$OUT/model_copy.blend" --python "$ROOT/tools/blender/run_headless.py" -- "$T/support_contacts.py" \
  "{\"pts\": \"$OUT/orient_pts.json\", \"out\": \"$OUT/orient_contacts.json\"}" > "$OUT/orient_contacts.log" 2>&1
tail -1 "$OUT/orient_contacts.log"
echo "== 5 pick orientations"
python3 "$T/pick_orientation.py" "$CFG" "$OUT/orient_contacts.json" "$OUT/oslice" "$OUT"
echo "== 6 project slice"
rm -rf "$OUT/chosen" "$OUT/project"; mkdir -p "$OUT/chosen"
python3 -c "
import json, shutil
for p, t in json.load(open('$OUT/chosen_orientation.json')).items():
    shutil.copy('$OUT/orient/' + t + '/' + p + '.stl', '$OUT/chosen/' + p + '.stl')"
"$T/orca.sh" "$OUT/profiles" support "$OUT/project" "$OUT"/chosen/*.stl -- --export-3mf "${PROJ}_parts.3mf" || echo "project slice exit $?"
python3 "$T/summarize.py" "$OUT/project" "$OUT/orient/pz/manifest.json" "$OUT/summary.json"
echo "== 7 fit-face details for the chosen orientations"
python3 -c "
import json
P = json.load(open('$OUT/orient_pts.json')); c = json.load(open('$OUT/chosen_orientation.json'))
json.dump({t + '/' + p: P[t + '/' + p] for p, t in c.items()}, open('$OUT/chosen_pts.json', 'w'))"
"$BLENDER" -b "$OUT/model_copy.blend" --python "$ROOT/tools/blender/run_headless.py" -- "$T/support_contacts.py" \
  "{\"pts\": \"$OUT/chosen_pts.json\", \"out\": \"$OUT/chosen_contacts.json\", \"details\": true}" > "$OUT/chosen_contacts.log" 2>&1
python3 -c "
import json
for k, v in json.load(open('$OUT/chosen_contacts.json')).items():
    for m, e in v.get('fit_faces', {}).items(): print(f'{k:32s} {m:34s} {e}')"
echo "done: $OUT (open $OUT/project/${PROJ}_parts.3mf in OrcaSlicer)"
