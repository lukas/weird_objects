#!/usr/bin/env bash
# One command: drive the WALK champion in sim (OpenCV window, HUD built in).
# I/K = faster/slower, J/L = strafe, Space = stop, R = reset, Q = quit.
#
#   ./sim_walk.sh                  # champion (ppo_goal_cw_walk_longdist_r2)
#   ./sim_walk.sh path/to/other.zip
#
# Deliberately plain python, NOT mjpython: drive_policy renders via cv2
# because mjpython's viewer segfaults intermittently on macOS.
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=../../.venv
POLICY="${1:-rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip}"

[ -x "$VENV/bin/python" ] || {
    echo "missing $VENV/bin/python - restore the venv:" >&2
    echo "  cd ~/weird_objects && uv pip install ftservo-python-sdk mujoco stable-baselines3 gymnasium pyyaml opencv-python trimesh" >&2
    exit 1
}
[ -f "$POLICY" ] || {
    echo "missing $POLICY - pull it from a pod:" >&2
    echo "  KUBECONFIG=~/.kube/coreweave.yaml kubectl cp hexapod-mjx-train-0:/workspace/prototype_sts3215/$POLICY $POLICY" >&2
    exit 1
}

exec "$VENV/bin/python" -m rl_move.sim.drive_policy "$POLICY"
