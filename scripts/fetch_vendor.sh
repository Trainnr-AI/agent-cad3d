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
