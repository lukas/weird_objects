#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

UV_BIN="${UV:-$(command -v uv || true)}"
if [[ -z "$UV_BIN" && -x /opt/homebrew/bin/uv ]]; then
  UV_BIN=/opt/homebrew/bin/uv
fi
[ -n "$UV_BIN" ] || {
  echo "uv not found - install uv or set UV=/path/to/uv" >&2
  exit 1
}
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
  PY=( "$UV_BIN" run mjpython )
  set -- --viewer "$@"
else
  PY=( "$UV_BIN" run python )
fi

exec "${PY[@]}" -m rl_move.sim.web_server \
  --http-port "${SIM_WEB_PORT:-8898}" \
  --policy-dir "$POLICY_DIR" \
  "$@"
