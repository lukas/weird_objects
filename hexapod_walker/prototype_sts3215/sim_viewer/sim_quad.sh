#!/usr/bin/env bash
# Interactive playground for the tip-back QUAD walk (the webui Quad
# tab's gait, motor_setup/quad_walk.py) in the fitted servo twin.
# 7/W rear up, W walk, Space stop, 8 sit down, 9 reset, -/= speed,
# P forward shove, drag to orbit, Z/X zoom, Q quit. Keys are drawn in
# the window. OpenCV window: no MuJoCo letter-toggle collisions.
#
#   ./sim_quad.sh
#
# Plain python on purpose (cv2 render path) - mjpython segfaults
# intermittently on macOS and its viewer grabs letter keys.
set -euo pipefail
cd "$(dirname "$0")/.."

# repo-root venv; in an agent worktree fall back to the active venv or
# the MAIN checkout's (worktrees share it — see .cursor rules).
PY=""
for cand in ../../.venv/bin/python "${VIRTUAL_ENV:-/nonexistent}/bin/python" \
            "$HOME/weird_objects/.venv/bin/python"; do
    [ -x "$cand" ] && { PY="$cand"; break; }
done
[ -n "$PY" ] || {
    echo "no venv python found - restore the venv:" >&2
    echo "  cd ~/weird_objects && uv pip install ftservo-python-sdk mujoco stable-baselines3 gymnasium pyyaml opencv-python trimesh pygame-ce" >&2
    exit 1
}

exec "$PY" -m rl_move.sim.quad_play "$@"
