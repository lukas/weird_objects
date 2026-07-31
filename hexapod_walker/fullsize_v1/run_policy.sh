#!/usr/bin/env bash
# Tiny wrapper: launch a trained walker policy in the MuJoCo viewer.
#
#   ./hexapod_walker/fullsize_v1/run_policy.sh v2          # default vx=0.3, light obstacles
#   ./hexapod_walker/fullsize_v1/run_policy.sh v5 0.4      # vx=0.4
#   ./hexapod_walker/fullsize_v1/run_policy.sh v2 0.3 0    # vx=0.3, no obstacles
#
# Available tags: whichever folders exist under hexapod_walker/fullsize_v1/policies/
# that contain a <tag>.zip file (e.g. walker_v2/walker_v2.zip -> tag "v2").

set -euo pipefail

TAG="${1:-v2}"
VX="${2:-0.3}"
OBSTACLES="${3:-12}"

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
POLICY="$ROOT/hexapod_walker/fullsize_v1/policies/walker_${TAG}/walker_${TAG}.zip"

if [ ! -f "$POLICY" ]; then
    echo "No trained policy at: $POLICY" >&2
    echo "Available tags:" >&2
    ls -1 "$ROOT/hexapod_walker/fullsize_v1/policies" 2>/dev/null \
        | sed -n 's/^walker_//p' >&2
    exit 1
fi

cd "$ROOT"
exec ./.venv/bin/mjpython hexapod_walker/fullsize_v1/rollout_walker.py \
    --policy "$POLICY" \
    --vx "$VX" \
    --duration 600 \
    --obstacles "$OBSTACLES" \
    --terrain-seed 42 --obstacle-seed 7
