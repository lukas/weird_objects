#!/usr/bin/env bash
# Wrapper that:
#   1. Locates Blender (or tells you how to install it).
#   2. Builds blender_assets/{panels.ply, led_positions.json} via the
#      simulator (only if the cache is missing or stale).
#   3. Ensures all_polyhedra_31.stl exists (rebuilds it if not).
#   4. Runs render_blender.py inside Blender to produce a Cycles render.
#
# Usage (from anywhere — the script cd's into chandelier/ first):
#   ./render_blender.sh                         # default render
#   ./render_blender.sh --samples 256           # higher quality
#   ./render_blender.sh --device METAL          # GPU (Apple Silicon)
#   ./render_blender.sh --projection            # moody projection preset
#   ./render_blender.sh --out renders/foo.png   # custom output

set -euo pipefail

CHAN_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$CHAN_DIR/.." && pwd)"
cd "$CHAN_DIR"

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

# ---- step 1: build panels + LED positions ----------------------------------
ASSETS="blender_assets"
if [[ ! -f "$ASSETS/panels.ply" || ! -f "$ASSETS/led_positions.json" \
      || "all_polyhedra.py" -nt "$ASSETS/panels.ply" ]]; then
    echo "[render_blender] building panels + LED metadata ..."
    "$ROOT_DIR/run.sh" chandelier/simulate_chandelier.py --export-blender "$ASSETS"
else
    echo "[render_blender] reusing cached $ASSETS/{panels.ply,led_positions.json}"
fi

# ---- step 2: ensure STL(s) exist -------------------------------------------
# Detect whether the caller wants the minimalist build so we can ensure
# the right STL set is on disk before launching Blender.
WANT_MINIMALIST=0
for arg in "$@"; do
    if [[ "$arg" == "--minimalist" ]]; then
        WANT_MINIMALIST=1
        break
    fi
done

if [[ $WANT_MINIMALIST -eq 1 ]]; then
    if [[ ! -f "chandelier_metal_minimal.stl" ]]; then
        echo "[render_blender] minimalist metal STL missing; rebuilding chandelier ..."
        "$ROOT_DIR/run.sh" chandelier/all_polyhedra.py
    fi
    if [[ ! -f "chandelier_minimalist_polyhedra.stl" \
          || ! -f "chandelier_minimalist_cables.stl" \
          || "all_polyhedra.py" -nt "chandelier_minimalist_polyhedra.stl" ]]; then
        echo "[render_blender] building minimalist visualization STLs ..."
        "$ROOT_DIR/run.sh" chandelier/build_minimalist_render.py
    fi
else
    if [[ ! -f "all_polyhedra_31.stl" ]]; then
        echo "[render_blender] STL missing; building it ..."
        "$ROOT_DIR/run.sh" chandelier/all_polyhedra.py
    fi
fi

# ---- step 3: render --------------------------------------------------------
echo "[render_blender] launching Blender Cycles render ..."
"$BLENDER" --background --factory-startup \
    --python render_blender.py -- "$@"
