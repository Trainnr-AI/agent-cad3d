#!/usr/bin/env bash
# One-shot setup: Blender + MCP for Blender addon, wired to this repo's .mcp.json.
set -euo pipefail

MCP_PKG="mcp-for-blender==2.2.7"   # keep in sync with .mcp.json
BLENDER_VERSION="5.2.3"              # the version this repo's .blend files were made with
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

say() { printf '\033[1;36m[agent-cad3d]\033[0m %s\n' "$*"; }

# 1. uv (runs the MCP server)
if ! command -v uvx >/dev/null 2>&1; then
  say "Installing uv..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi

# 2. Blender
find_blender() {
  if command -v blender >/dev/null 2>&1; then command -v blender; return; fi
  for p in /Applications/Blender.app/Contents/MacOS/Blender "$HOME/Applications/Blender.app/Contents/MacOS/Blender" \
           "$HOME/.local/bin/blender"; do
    [ -x "$p" ] && { echo "$p"; return; }
  done
}
BLENDER="$(find_blender || true)"
if [ -z "$BLENDER" ]; then
  if [[ "$(uname)" == "Darwin" ]] && command -v brew >/dev/null 2>&1; then
    say "Installing Blender via Homebrew..."
    brew install --cask blender
    BLENDER="$(find_blender)"
  elif [[ "$(uname)" == "Linux" ]]; then
    # Linux / WSL2: official tarball into ~/.local/opt (no root needed). WSL needs WSLg for the window.
    MAJ="${BLENDER_VERSION%.*}"
    URL="https://download.blender.org/release/Blender${MAJ}/blender-${BLENDER_VERSION}-linux-x64.tar.xz"
    say "Installing Blender ${BLENDER_VERSION} from $URL"
    mkdir -p "$HOME/.local/opt" "$HOME/.local/bin"
    curl -fL "$URL" | tar -xJ -C "$HOME/.local/opt"
    ln -sf "$HOME/.local/opt/blender-${BLENDER_VERSION}-linux-x64/blender" "$HOME/.local/bin/blender"
    BLENDER="$HOME/.local/bin/blender"
    say "If Blender fails to start, install its libraries:"
    say "  sudo apt install -y libxi6 libxkbcommon0 libxrender1 libgl1 libsm6 libxxf86vm1 libxfixes3 libegl1"
  else
    say "Blender not found. Install it from https://www.blender.org/download/ and re-run."
    exit 1
  fi
fi
say "Blender: $("$BLENDER" --version 2>/dev/null | head -1)"

# 3. Locate (or create) Blender's user addons dir for this version
VER="$("$BLENDER" --version 2>/dev/null | head -1 | sed -E 's/Blender ([0-9]+\.[0-9]+).*/\1/')"
if [[ "$(uname)" == "Darwin" ]]; then
  ADDONS_DIR="$HOME/Library/Application Support/Blender/$VER/scripts/addons"
else
  ADDONS_DIR="$HOME/.config/blender/$VER/scripts/addons"
fi
mkdir -p "$ADDONS_DIR"

# 4. Install + enable the addon
say "Installing MCP for Blender addon into $ADDONS_DIR"
BLENDERMCP_ADDONS_DIR="$ADDONS_DIR" uvx "$MCP_PKG" install-addon
# No --factory-startup here: it would save factory defaults over the user's existing prefs
"$BLENDER" --background --python "$REPO_ROOT/scripts/enable_addon.py"

# 5. Upstream XGO-Duck sources (git-ignored) at the pinned commits
"$REPO_ROOT/scripts/fetch_vendor.sh"

say "Done. Next:"
say "  1. Open the project file (its MCP port 9877 is saved in it; the addon auto-starts):"
say "       $BLENDER $REPO_ROOT/projects/xgoduck_scs0009/xgoduck_scs0009.blend"
say "     Check: python3 scripts/blender_doctor.py   (9876 is left free for the trainnr Studio viewer)"
say "  2. Start Claude Code in $REPO_ROOT and approve the 'blender' MCP server."
say "  3. Read docs/HANDOFF.md for where the project stands."
