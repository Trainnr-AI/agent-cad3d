#!/usr/bin/env bash
# Slice STLs with the OrcaSlicer CLI and the flattened profiles from fetch_profiles.py.
#   tools/slicer/orca.sh <profiles_dir> <support|nosupport> <out_dir> <file.stl>... [-- extra orca args]
# Writes <out_dir>/plate_N.gcode + plate_N.statistics.json (time, grams, per-object support) and log.txt.
# Finds the binary in $ORCA, then the macOS app, then orca-slicer / an AppImage on PATH.
set -uo pipefail
P=$1; MODE=$2; OUT=$3; shift 3
ORCA=${ORCA:-}
if [ -z "$ORCA" ]; then
  for c in /Applications/OrcaSlicer.app/Contents/MacOS/OrcaSlicer "$(command -v orca-slicer 2>/dev/null)" \
           "$(ls "$HOME"/.local/bin/OrcaSlicer*.AppImage 2>/dev/null | head -1)"; do
    [ -n "$c" ] && [ -x "$c" ] && ORCA=$c && break
  done
fi
[ -z "$ORCA" ] && { echo "OrcaSlicer not found: run scripts/setup_orca.sh or set ORCA=/path/to/binary" >&2; exit 2; }
mkdir -p "$OUT" "$P/datadir"
FILES=(); EXTRA=()
while [ $# -gt 0 ]; do [ "$1" = "--" ] && { shift; EXTRA=("$@"); break; }; FILES+=("$1"); shift; done
# APPIMAGE_EXTRACT_AND_RUN: AppImages under WSL/containers often lack FUSE
APPIMAGE_EXTRACT_AND_RUN=1 "$ORCA" --datadir "$P/datadir" \
  --load-settings "$P/machine.json;$P/process_$MODE.json" --load-filaments "$P/filament.json" \
  --arrange 1 --orient 0 --slice 0 --export-statistics --outputdir "$OUT" ${EXTRA[@]+"${EXTRA[@]}"} "${FILES[@]}" > "$OUT/log.txt" 2>&1
rc=$?
echo "$rc" > "$OUT/exit_code"
exit $rc
