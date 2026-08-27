#!/usr/bin/env bash
# Canonical local Mac launcher for the hexapod web hub on :8898.
#
# This runs on Lukas's Mac, not on the Uno Q. It serves the shared web UI
# locally and proxies the robot target to the board's :8080 web service.
set -euo pipefail

SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LABEL="${HEXAPOD_WEB8898_LABEL:-com.lukas.hexapod-web-8898}"
LEGACY_LABEL="${HEXAPOD_WEB8898_LEGACY_LABEL:-com.lukas.hexapod-sim-web-8898}"
PORT="${SIM_WEB_PORT:-8898}"
BIND="${SIM_WEB_BIND:-127.0.0.1}"
TARGET="${SIM_WEB_TARGET:-robot}"
PHASE="${SIM_WEB_PHASE:-1}"
LOG="${HEXAPOD_WEB8898_LOG:-/tmp/hexapod_web_8898.log}"
POLICY_DIR="${POLICY_DIR:-$ROOT/rl_move/sim/policies}"
UV_BIN="${UV:-$(command -v uv || true)}"

if [[ -z "$UV_BIN" ]]; then
  if [[ -x /opt/homebrew/bin/uv ]]; then
    UV_BIN=/opt/homebrew/bin/uv
  else
    echo "uv not found. Install uv or set UV=/path/to/uv." >&2
    exit 127
  fi
fi

usage() {
  cat <<EOF
Usage: sim_viewer/hexapod_web_8898.sh <command>

Commands:
  start       Start the local Mac web hub on http://localhost:${PORT}/rl
  stop        Stop the launchctl job and any stale hub on port ${PORT}
  restart     Stop, then start
  status      Show launchctl, port, and HTTP health
  logs        Tail ${LOG}
  foreground  Run in the foreground with uv run

Environment:
  HEXAPOD_HOST=http://<robot-ip-or-name>:8080   robot target (default: resolve hexapod.local)
  SIM_WEB_BIND=${BIND}                          bind address
  SIM_WEB_PORT=${PORT}                          local port
  SIM_WEB_TARGET=${TARGET}                      sim, robot, or both
  POLICY_DIR=${POLICY_DIR}                      policy cache
EOF
}

normalize_url() {
  local url="$1"
  if [[ "$url" != http://* && "$url" != https://* ]]; then
    url="http://$url"
  fi
  printf '%s\n' "$url"
}

robot_url() {
  if [[ -n "${HEXAPOD_HOST:-}" ]]; then
    normalize_url "$HEXAPOD_HOST"
    return
  fi
  if [[ -n "${ROBOT_URL:-}" ]]; then
    normalize_url "$ROBOT_URL"
    return
  fi

  local ip=""
  ip="$(bash "$ROOT/linux_control/dev_loop.sh" resolve 2>/dev/null || true)"
  if [[ -n "$ip" ]]; then
    printf 'http://%s:8080\n' "$ip"
  else
    printf 'http://hexapod.local:8080\n'
  fi
}

listener_pid() {
  lsof -tiTCP:"$PORT" -sTCP:LISTEN 2>/dev/null | head -n 1 || true
}

listener_command() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  ps -p "$pid" -o command= 2>/dev/null || true
}

wait_ready() {
  local url="http://${BIND}:${PORT}/api/ping"
  local i
  for i in {1..80}; do
    if curl -fsS -m 1 "$url" >/dev/null 2>&1; then
      echo "ready: http://localhost:${PORT}/rl"
      curl -fsS -m 2 "$url" || true
      echo
      return 0
    fi
    sleep 0.25
  done
  echo "local web hub did not become ready. Recent log:" >&2
  tail -80 "$LOG" >&2 2>/dev/null || true
  return 1
}

stop_port_if_ours() {
  local pid cmd
  pid="$(listener_pid)"
  [[ -n "$pid" ]] || return 0
  cmd="$(listener_command "$pid")"
  if [[ "$cmd" == *"rl_move.sim.web_server"* && "$cmd" == *"--http-port $PORT"* ]]; then
    /bin/kill "$pid" 2>/dev/null || true
  else
    echo "port ${PORT} is in use by another process:" >&2
    echo "  $cmd" >&2
    return 1
  fi
}

serve() {
  local bind="$1" port="$2" policy_dir="$3" url="$4" target="$5" phase="$6" log="$7"
  cd "$ROOT"
  exec >>"$log" 2>&1
  echo "[$(date -u '+%Y-%m-%dT%H:%M:%SZ')] starting hexapod web hub on ${bind}:${port}"
  echo "robot target: ${url}"
  local args=(
    run python -m rl_move.sim.web_server
    --bind "$bind"
    --http-port "$port"
    --policy-dir "$policy_dir"
    --robot-url "$url"
    --target "$target"
  )
  if [[ "$phase" != "0" ]]; then
    args+=(--phase-obs)
  fi
  exec "$UV_BIN" "${args[@]}"
}

start() {
  local url pid
  url="$(robot_url)"
  if pid="$(listener_pid)" && [[ -n "$pid" ]]; then
    echo "already listening on :${PORT}:"
    listener_command "$pid"
    wait_ready
    return
  fi

  launchctl remove "$LEGACY_LABEL" >/dev/null 2>&1 || true
  launchctl remove "$LABEL" >/dev/null 2>&1 || true
  echo "starting $LABEL with uv run..."
  launchctl submit -l "$LABEL" -- /bin/bash "$SELF" serve \
    "$BIND" "$PORT" "$POLICY_DIR" "$url" "$TARGET" "$PHASE" "$LOG"
  wait_ready
}

stop() {
  launchctl remove "$LEGACY_LABEL" >/dev/null 2>&1 || true
  launchctl remove "$LABEL" >/dev/null 2>&1 || true
  stop_port_if_ours || return
  echo "stopped $LABEL"
}

status() {
  echo "label: $LABEL"
  launchctl print "gui/$(id -u)/$LABEL" 2>/dev/null | sed -n '1,70p' || true
  local pid
  pid="$(listener_pid)"
  if [[ -n "$pid" ]]; then
    echo
    echo "port ${PORT}:"
    lsof -nP -iTCP:"$PORT" -sTCP:LISTEN || true
    echo
    curl -fsS -m 2 "http://localhost:${PORT}/api/ping" || true
    echo
    echo "url: http://localhost:${PORT}/rl"
  else
    echo "port ${PORT}: not listening"
  fi
}

logs() {
  tail -n "${N:-120}" -f "$LOG"
}

foreground() {
  local url
  url="$(robot_url)"
  cd "$ROOT"
  local args=(
    run python -m rl_move.sim.web_server
    --bind "$BIND"
    --http-port "$PORT"
    --policy-dir "$POLICY_DIR"
    --robot-url "$url"
    --target "$TARGET"
  )
  if [[ "$PHASE" != "0" ]]; then
    args+=(--phase-obs)
  fi
  exec "$UV_BIN" "${args[@]}"
}

cmd="${1:-status}"
shift || true
case "$cmd" in
  start) start "$@" ;;
  stop) stop "$@" ;;
  restart) stop "$@" || true; start "$@" ;;
  status) status "$@" ;;
  logs) logs "$@" ;;
  foreground) foreground "$@" ;;
  serve) serve "$@" ;;
  help|-h|--help) usage ;;
  *)
    echo "unknown command: $cmd" >&2
    usage >&2
    exit 2
    ;;
esac
