#!/usr/bin/env bash
# Fetch the upstream XGO-Duck sources into vendor/ (git-ignored) at the exact commits this repo
# was built from. Needed to rebuild the simulation models / re-run the Blender pipeline; the
# generated models in projects/ are committed and don't need it.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/vendor"

fetch() {   # fetch <repo> <commit> <dir> [sparse paths...]
  local repo=$1 sha=$2 name=$3 dir="$ROOT/vendor/$3"; shift 3
  if [ -d "$dir/.git" ] && [ "$(git -C "$dir" rev-parse HEAD)" = "$sha" ]; then
    echo "[vendor] $name already at ${sha:0:10}"; return
  fi
  rm -rf "$dir"
  git init -q "$dir"
  git -C "$dir" remote add origin "https://github.com/$repo"
  if [ $# -gt 0 ]; then
    git -C "$dir" sparse-checkout set --no-cone "$@"
  fi
  git -C "$dir" fetch -q --depth 1 origin "$sha"
  git -C "$dir" checkout -q FETCH_HEAD
  echo "[vendor] $name @ ${sha:0:10}"
}

# hardware: STLs, BOM, assembly guide (verified 2026-11-09 against the files used)
fetch LuwuDynamics/xgoduck_hardware e1d5aa4317fe20d3fc7b899ea5bb3365e04d827f xgoduck_hardware
# RL repo, only the XGO-Duck robot model (the commit BEFORE a4c78aa, which changed robot_allcollisions.xml)
fetch LuwuDynamics/xgoduck_rl fa621fc77ad9a05d69da13270e86118bdd3603fe xgoduck_rl \
  "/src/mjlab_microduck/robot/xgoduck/" "/src/mjlab_microduck/robot/xgoduck_constants.py" "/LICENSE*" "/README.md"

# Feetech SCS0009 CAD (Pro/E assembly SCS009-20201118-S, STEP export 2023-01-10) as distributed by a reseller.
# tools/cad/scs0009_from_step.py turns it into tools/blender/parts/scs0009_mesh/*.stl (committed).
STEP_URL='https://cdn.shopify.com/s/files/1/0673/6848/5000/files/SCS009-20230110-S.stp?v=1761661153'
STEP_SHA=cdaa6e070a9742c7aecc7143849469760238aa589e1045fe844a8c161eb45167
STEP_OUT="$ROOT/vendor/feetech/SCS009-20230110-S.stp"
if [ -f "$STEP_OUT" ] && echo "$STEP_SHA  $STEP_OUT" | sha256sum -c --quiet - 2>/dev/null; then
  echo "[vendor] feetech SCS0009 STEP already present"
else
  mkdir -p "$ROOT/vendor/feetech"
  curl -fsSL -o "$STEP_OUT" "$STEP_URL"
  echo "$STEP_SHA  $STEP_OUT" | sha256sum -c --quiet - || { echo "[vendor] SCS0009 STEP checksum mismatch"; exit 1; }
  echo "[vendor] feetech SCS0009 STEP fetched"
fi
