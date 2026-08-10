#!/usr/bin/env bash
# One command: BOTH champions in one sim - stand up (7), walk (I/K/J/L),
# stop (space), sit (8). OpenCV window: every key is ours, none of
# MuJoCo's built-in letter toggles exist here.
#
#   ./sim_play.sh
#   ./sim_play.sh --stance other_stance.zip --walk other_walk.zip
#
# Plain python on purpose (cv2 render path) - mjpython segfaults
# intermittently on macOS and its viewer grabs letter keys.
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=../../.venv
[ -x "$VENV/bin/python" ] || {
    echo "missing $VENV/bin/python - restore the venv:" >&2
    echo "  cd ~/weird_objects && uv pip install ftservo-python-sdk mujoco stable-baselines3 gymnasium pyyaml opencv-python trimesh" >&2
    exit 1
}

exec "$VENV/bin/python" -m rl_move.sim.play "$@"
