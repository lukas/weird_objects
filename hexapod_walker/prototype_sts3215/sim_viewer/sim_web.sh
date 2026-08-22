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
# Phase-clock obs ON by default (SIM_WEB_PHASE=0 disables): the frontier
# walk line (phasedir*, obs 74) is invisible to the picker without it,
# which silently dropped curated champions on restart (08-22). 72-obs
# policies are unaffected — they read the first 72 dims either way.
if [[ "${SIM_WEB_PHASE:-1}" != "0" ]]; then
  set -- --phase-obs "$@"
fi
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
