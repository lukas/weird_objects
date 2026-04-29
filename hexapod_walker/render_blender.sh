#!/usr/bin/env bash
# Wrapper that:
#   1. Locates Blender (or tells you how to install it).
#   2. Builds assembly/{frame,motors,battery,soft,rider,full}.stl via
#      build_full_assembly.py if any of the category STLs are missing
#      or build_full_assembly.py is newer than them.
#   3. Runs render_blender.py inside Blender to produce a Cycles render.
#
# Usage (from anywhere -- the script cd's into hexapod_walker/ first):
#   ./render_blender.sh                                   # default render
#   ./render_blender.sh --samples 256                     # higher quality
#   ./render_blender.sh --device METAL                    # GPU (Apple Silicon)
#   ./render_blender.sh --out renders/walker_3q.png       # custom output
#   ./render_blender.sh --camera-azimuth-deg 90           # straight from side

set -euo pipefail

WALKER_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$WALKER_DIR/.." && pwd)"
cd "$WALKER_DIR"

# ---- locate Blender --------------------------------------------------------
BLENDER=""
for cand in \
    "$(command -v blender 2>/dev/null || true)" \
    "/Applications/Blender.app/Contents/MacOS/Blender" \
    "/opt/homebrew/bin/blender" \
    "/usr/local/bin/blender"; do
    if [[ -x "$cand" ]]; then
        BLENDER="$cand"
        break
    fi
done

if [[ -z "$BLENDER" ]]; then
    cat <<'EOF' >&2
Blender was not found.

Install on macOS with Homebrew:
    brew install --cask blender

Or download from: https://www.blender.org/download/

After installing, re-run this script.
EOF
    exit 1
fi

echo "[render_blender] using Blender at: $BLENDER"
"$BLENDER" --version | head -1

# ---- step 1: build assembly STLs if missing or stale ---------------------
ASSETS="assembly"
NEEDED=(frame.stl motors.stl battery.stl soft.stl rider.stl)
REBUILD=0
for f in "${NEEDED[@]}"; do
    if [[ ! -f "$ASSETS/$f" ]]; then
        REBUILD=1
        echo "[render_blender] missing $ASSETS/$f"
        break
    fi
    if [[ "build_full_assembly.py" -nt "$ASSETS/$f" \
       || "hexapod_walker.py"      -nt "$ASSETS/$f" ]]; then
        REBUILD=1
        echo "[render_blender] $ASSETS/$f is older than its source script"
        break
    fi
done

if [[ $REBUILD -eq 1 ]]; then
    echo "[render_blender] (re)building assembly STLs ..."
    "$ROOT_DIR/run.sh" hexapod_walker/build_full_assembly.py
else
    echo "[render_blender] reusing cached $ASSETS/*.stl"
fi

# ---- step 2: render --------------------------------------------------------
echo "[render_blender] launching Blender Cycles render ..."
"$BLENDER" --background --factory-startup \
    --python render_blender.py -- "$@"
