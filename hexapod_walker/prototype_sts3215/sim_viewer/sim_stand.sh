#!/usr/bin/env bash
# One command: MuJoCo viewer with the STANCE champion (stand up / sit down).
# Keys are shown IN the viewer window. 7 = stand up, 8 = sit down.
#
#   ./sim_stand.sh                 # champion (ppo_goal_cw_stance_dr10)
#   ./sim_stand.sh path/to/other.zip
#
# Gotcha this script exists to encode: stance checkpoints need
# `--task joint_goal` (obs 68 / act 18). The default --task goal is the
# 56-obs body-offset env and fails with an observation-shape error.
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=../../.venv
POLICY="${1:-rl_move/sim/policies/ppo_goal_cw_stance_dr10.zip}"

[ -x "$VENV/bin/mjpython" ] || {
    echo "missing $VENV/bin/mjpython - restore the venv:" >&2
    echo "  cd ~/weird_objects && uv pip install ftservo-python-sdk mujoco stable-baselines3 gymnasium pyyaml opencv-python trimesh" >&2
    exit 1
}
[ -f "$POLICY" ] || {
    echo "missing $POLICY - pull it from a pod:" >&2
    echo "  KUBECONFIG=~/.kube/coreweave.yaml kubectl cp hexapod-sweep-friction:/workspace/prototype_sts3215/$POLICY $POLICY" >&2
    exit 1
}

exec "$VENV/bin/mjpython" -m rl_move.sim.view --task joint_goal --policy "$POLICY"
