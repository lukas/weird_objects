#!/usr/bin/env bash
# One command: drive the NO-SLIP scripted gait in sim (OpenCV window).
# I/K = fwd/back, J/L = strafe, U/O = turn, 1/2/3 = gait period,
# 4/5/6 = alpha 0/0.5/1 (body-motion overlap), Space = stop, R = reset,
# Q = quit.
#
#   ./sim_noslip.sh
#
# Deliberately plain python, NOT mjpython: renders via cv2 because
# mjpython's viewer segfaults intermittently on macOS.
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=../../.venv
[ -x "$VENV/bin/python" ] || {
    echo "missing $VENV/bin/python - restore the venv:" >&2
    echo "  cd ~/weird_objects && uv pip install ftservo-python-sdk mujoco stable-baselines3 gymnasium pyyaml opencv-python trimesh" >&2
    exit 1
}

exec "$VENV/bin/python" -m rl_move.sim.drive_noslip "$@"
