#!/usr/bin/env bash
# Install OrcaSlicer (pinned 2.5.2) for tools/slicer/. Idempotent.
# macOS: Homebrew cask. Linux / WSL: the official AppImage into ~/.local/bin (run with APPIMAGE_EXTRACT_AND_RUN=1 when FUSE is
# missing, which tools/slicer/orca.sh does).
set -euo pipefail
VER=2.5.2
if [ "$(uname)" = "Darwin" ]; then
  [ -x /Applications/OrcaSlicer.app/Contents/MacOS/OrcaSlicer ] && { echo "OrcaSlicer already installed"; exit 0; }
  brew install --cask orcaslicer
else
  DEST=$HOME/.local/bin/OrcaSlicer_V$VER.AppImage
  [ -x "$DEST" ] && { echo "OrcaSlicer already at $DEST"; exit 0; }
  case "$(uname -m)" in
    aarch64|arm64) A=OrcaSlicer_Linux_AppImage_Ubuntu2404_aarch64_V$VER.AppImage ;;
    *) A=OrcaSlicer_Linux_AppImage_Ubuntu2404_V$VER.AppImage ;;
  esac
  mkdir -p "$HOME/.local/bin"
  curl -fL -o "$DEST" "https://github.com/OrcaSlicer/OrcaSlicer/releases/download/v$VER/$A"
  chmod +x "$DEST"
  echo "installed $DEST (headless use may need: sudo apt install libwebkit2gtk-4.1-0 libgl1 xvfb)"
fi
