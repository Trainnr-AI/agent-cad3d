#!/usr/bin/env bash
# Full design check of the nanoduck, headless (the live Blender is not touched), all checks in parallel (~15-20 min, 12 Blenders).
#   scripts/full_check.sh [out_dir]         (default out_dir: /tmp/nanoduck_check)
# Reads the SAVED .blend - save in Blender first. Arguments: projects/xgoduck_scs0009/data/check_args/ (README there).
set -u
cd "$(dirname "$0")/.."
OUT="${1:-/tmp/nanoduck_check}"
mkdir -p "$OUT"
BLEND=projects/xgoduck_scs0009/xgoduck_scs0009.blend
A=projects/xgoduck_scs0009/data/check_args
T=tools/blender
run() {   # name tool args.json
  blender -b "$BLEND" --python $T/run_headless.py -- "$2" "$3" "$OUT" > "$OUT/$1.log" 2>&1
  echo "$1 done $(date +%H:%M)"
}
none="$OUT/none.json"; echo '{}' > "$none"
( run rest_pose $T/checks/rest_pose_check.py "$none"; run printed_parts $T/checks/printed_parts_check.py "$none"
  run print $T/print_check.py $A/print.json; run hip_yaw_path $T/checks/hip_yaw_servo_path.py "$none"
  run segment_paths $T/checks/segment_path_check.py $A/segment_paths.json ) &
run horn $T/horn_check.py $A/horn.json &
run fit $T/fit_audit.py $A/fit.json &
run asm $T/assembly_path_check.py $A/asm.json &
run screws $T/screw_audit.py $A/screws.json &
run driver $T/checks/driver_access.py $A/driver.json &
run motion $T/leg_motion_check.py $A/motion.json &
run cable_reach $T/wiring.py $A/cable_reach.json &
run cable_pinch $T/cable_pinch_check.py $A/cable_pinch.json &
wait
echo "=== summary ($OUT)"
grep -h "joints differing\|printed parts checked" "$OUT"/rest_pose.log "$OUT"/printed_parts.log
grep -h "fits checked\|FAIL" "$OUT/fit.log"
echo "assembly paths: $(grep -c '^OK' "$OUT/asm.log") OK; not OK:"; grep "^BLOCKED\|^FAIL" "$OUT/asm.log" | cut -c1-160
grep -h "^OK\|^BLOCKED" "$OUT/segment_paths.log"
grep -h "screw audit" "$OUT/screws.log"
grep -h "poses checked" "$OUT/motion.log"
grep -h "too short" "$OUT/cable_reach.log"
grep -h "cable pinch check" "$OUT/cable_pinch.log"
echo "(hip-yaw servo L-path, horns, driver access, print: see their logs)"
