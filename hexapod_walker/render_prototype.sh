#!/usr/bin/env bash
# Wrapper for rendering the PROTOTYPE (hobby-servo) hexapod walker.
# Mirrors render_blender.sh but:
#   - sources its STLs from prototype_assembly/ (built by
#     build_prototype_assembly.py)
#   - uses a closer camera + lower viewpoint suited to a 60 cm robot
#     instead of a 4 m one
#   - skips the rider material (the prototype has no rider)
#
# Usage (from anywhere):
#   ./render_prototype.sh                                   # default render
#   ./render_prototype.sh --samples 256                     # higher quality
#   ./render_prototype.sh --device METAL                    # GPU on Apple Silicon
#   ./render_prototype.sh --out renders/prototype_side.png  # custom output
#   ./render_prototype.sh --camera-azimuth-deg 90           # straight from side

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

echo "[render_prototype] using Blender at: $BLENDER"
"$BLENDER" --version | head -1

# ---- step 1: build prototype assembly STLs if missing or stale ------------
ASSETS="prototype_assembly"
NEEDED=(frame.stl motors.stl battery.stl soft.stl)
REBUILD=0
for f in "${NEEDED[@]}"; do
    if [[ ! -f "$ASSETS/$f" ]]; then
        REBUILD=1
        echo "[render_prototype] missing $ASSETS/$f"
        break
    fi
    if [[ "build_prototype_assembly.py" -nt "$ASSETS/$f" \
       || "hexapod_prototype.py"        -nt "$ASSETS/$f" ]]; then
        REBUILD=1
        echo "[render_prototype] $ASSETS/$f is older than its source script"
        break
    fi
done

if [[ $REBUILD -eq 1 ]]; then
    echo "[render_prototype] (re)building prototype assembly STLs ..."
    "$ROOT_DIR/run.sh" hexapod_walker/build_prototype_assembly.py
else
    echo "[render_prototype] reusing cached $ASSETS/*.stl"
fi

# ---- step 2: render --------------------------------------------------------
# Defaults tuned for a ~60 cm robot:
#   - camera 1.2 m away at 0.45 m eye height
#   - shorter focal length (35 mm) for a wider 3/4 view
#   - frame_color is more colourful (printed PLA) than brushed Al
#   - motor_color closer to dark grey (hobby servo plastic) than anodized Al
#   - battery_color brighter (fluorescent LiPo wraps tend to be vivid)
#   - soft_color picks a black rubber tone like inner-tube tread
#   - default output renders/prototype.png
DEFAULTS=(
    --assets        prototype_assembly
    --out           renders/prototype.png
    --camera-distance 0.85
    --camera-height   0.32
    --camera-azimuth-deg 35
    --camera-pitch-deg   -10
    --camera-target-elev 0.06
    --lens          35
    --sun-strength  4.0
    --sky-strength  1.0
    --frame-color   "#dfe4e7"
    --motor-color   "#2a2c30"
    --battery-color "#262638"
    --soft-color    "#181818"
    --ground        "#cfcdc6"
)

echo "[render_prototype] launching Blender Cycles render ..."
"$BLENDER" --background --factory-startup \
    --python render_blender.py -- "${DEFAULTS[@]}" "$@"
