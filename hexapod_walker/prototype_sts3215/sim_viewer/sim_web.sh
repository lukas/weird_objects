#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ -z "${VENV:-}" ]]; then
  if [[ -x ".venv/bin/mjpython" || -x ".venv/bin/python" ]]; then
    VENV=".venv"
  else
    VENV="../../.venv"
  fi
fi
POLICY_DIR="${POLICY_DIR:-rl_move/sim/policies}"
# macOS ships bash 3.2, where "${EXTRA[@]}" on an empty array trips `set -u`
# — prepend to "$@" instead of using an array.
if [[ "${SIM_WEB_VIEWER:-1}" != "0" ]]; then
  PY="$VENV/bin/mjpython"
  set -- --viewer "$@"
else
  PY="$VENV/bin/python"
fi

[ -x "$PY" ] || {
  echo "missing $PY - restore the prototype venv" >&2
  exit 1
}

exec "$PY" -m rl_move.sim.web_server \
  --http-port "${SIM_WEB_PORT:-8898}" \
  --policy-dir "$POLICY_DIR" \
  "$@"
