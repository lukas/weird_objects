#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

POLICY_DIR="${POLICY_DIR:-rl_move/sim/policies}"
exec python3 -m rl_move.sim.web_server \
  --http-port "${SIM_WEB_PORT:-8898}" \
  --policy-dir "$POLICY_DIR" \
  "$@"
