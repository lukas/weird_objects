#!/usr/bin/env bash
# Fast local helpers for the robot web/control dev loop.
#
# Usage:
#   source linux_control/dev_loop.sh
#   hex_check
#   hex_deploy
#   hex_deploy_fast
#
# Or run as a command:
#   linux_control/dev_loop.sh check
#   linux_control/dev_loop.sh deploy
#   linux_control/dev_loop.sh deploy-fast
if [ -n "${BASH_VERSION:-}" ]; then
  _HEX_SCRIPT="${BASH_SOURCE[0]}"
  if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    _HEX_EXECUTED=1
  else
    _HEX_EXECUTED=0
  fi
elif [ -n "${ZSH_VERSION:-}" ]; then
  _HEX_SCRIPT="${(%):-%x}"
  case "${ZSH_EVAL_CONTEXT:-}" in
    *:file) _HEX_EXECUTED=0 ;;
    *) _HEX_EXECUTED=1 ;;
  esac
else
  _HEX_SCRIPT="$0"
  _HEX_EXECUTED=1
fi

if [ "$_HEX_EXECUTED" = "1" ]; then
  set -euo pipefail
fi

HEX_LC_DIR="$(cd "$(dirname "$_HEX_SCRIPT")" && pwd)"
HEX_ROOT="$(cd "$HEX_LC_DIR/.." && pwd)"
HEXAPOD_HOST="${HEXAPOD_HOST:-http://hexapod.local:8080}"
HEXAPOD_CURL_TIMEOUT="${HEXAPOD_CURL_TIMEOUT:-2}"
HEXAPOD_SSH="${HEXAPOD_SSH:-arduino@hexapod.local}"
HEXAPOD_SSH_HOSTKEY_ALIAS="${HEXAPOD_SSH_HOSTKEY_ALIAS:-hexapod.local}"
HEX_REMOTE_ROOT="${HEX_REMOTE_ROOT:-/home/arduino/hexapod_sts}"
HEXAPOD_MDNS_NAME="${HEXAPOD_MDNS_NAME:-hexapod.local}"
HEXAPOD_CACHE_DIR="${HEXAPOD_CACHE_DIR:-$HOME/.hexapod}"
HEXAPOD_IP_CACHE="${HEXAPOD_IP_CACHE:-$HEXAPOD_CACHE_DIR/last_ip}"

hex_note() {
  printf '>> %s\n' "$*"
}

hex_py() {
  if ! command -v uv >/dev/null 2>&1; then
    echo "uv is required for local Python commands; install uv first" >&2
    return 127
  fi
  (cd "$HEX_ROOT" && uv run python "$@")
}

hex_py_syntax() {
  hex_note "uv python syntax check (no imports, no hardware)"
  hex_py - "$HEX_ROOT" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
files = [
    "linux_control/web_drive.py",
    "linux_control/bench_api.py",
    "linux_control/drive_controller.py",
    "linux_control/cpg_controller_loader.py",
    "linux_control/mcu_feetech_bus.py",
    "linux_control/event_log.py",
    "linux_control/status_display.py",
    "linux_control/deploy_status_display.py",
    "linux_control/video_contact_sheet.py",
    "linux_control/safe_zero.py",
    "linux_control/pinned_tip.py",
    "linux_control/test_calibration_checkup.py",
    "linux_control/plant_calibrate.py",
    "linux_control/geometry_plant.py",
    "linux_control/test_geometry_sweep_fit.py",
    "linux_control/imu_calibrate.py",
    "linux_control/bus_bench.py",
    "linux_control/sysid_protocol.py",
    "linux_control/sysid_runner.py",
    "linux_control/urt2_setup/inplace_demos.py",
    "linux_control/urt2_setup/motion_telemetry.py",
    "motor_setup/inplace_demos.py",
    "motor_setup/motion_telemetry.py",
    "motor_setup/dance_script.py",
    "motor_setup/quad_walk.py",
]

ok = True
for rel in files:
    path = root / rel
    try:
        compile(path.read_text(), str(path), "exec")
    except Exception as exc:
        ok = False
        print(f"FAIL {rel}: {exc}", file=sys.stderr)
if not ok:
    sys.exit(1)
print(f"OK {len(files)} files")
PY
}

hex_js_syntax() {
  if ! command -v node >/dev/null 2>&1; then
    hex_note "node not found; skipping webui/app.js syntax check"
    return 0
  fi
  hex_note "web UI syntax check"
  node --check "$HEX_LC_DIR/webui/app.js"
}

hex_diff_check() {
  hex_note "git whitespace/conflict-marker check"
  git -C "$HEX_ROOT" diff --check -- \
    linux_control motor_setup Makefile README.md
  git -C "$HEX_ROOT" diff --cached --check -- \
    linux_control motor_setup Makefile README.md
}

hex_cache_ip() {
  local ip="$1"
  [ -n "$ip" ] || return 1
  mkdir -p "$HEXAPOD_CACHE_DIR"
  printf '%s\n' "$ip" > "$HEXAPOD_IP_CACHE"
}

hex_cached_ip() {
  [ -s "$HEXAPOD_IP_CACHE" ] || return 1
  head -n 1 "$HEXAPOD_IP_CACHE"
}

hex_is_mdns_default() {
  case "$1" in
    *"$HEXAPOD_MDNS_NAME"*) return 0 ;;
    *) return 1 ;;
  esac
}

hex_url_for_ip() {
  local url="$1" ip="$2"
  printf '%s\n' "${url//$HEXAPOD_MDNS_NAME/$ip}"
}

hex_ssh_for_ip() {
  local ssh_target="$1" ip="$2"
  printf '%s\n' "${ssh_target//$HEXAPOD_MDNS_NAME/$ip}"
}

hex_resolve_active() {
  local name="${1:-$HEXAPOD_MDNS_NAME}"
  hex_py - "$name" <<'PY'
import re
import socket
import subprocess
import sys
import time

name = sys.argv[1].rstrip(".")

try:
    socket.setdefaulttimeout(1.5)
    print(socket.gethostbyname(name))
    raise SystemExit(0)
except Exception:
    pass

try:
    proc = subprocess.Popen(
        ["dns-sd", "-G", "v4", name],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
except Exception:
    raise SystemExit(1)

ip = None
deadline = time.time() + 3.0
try:
    while time.time() < deadline:
        line = proc.stdout.readline() if proc.stdout else ""
        if not line:
            time.sleep(0.05)
            continue
        match = re.search(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b", line)
        if match and match.group(1) != "0.0.0.0":
            ip = match.group(1)
            break
finally:
    proc.terminate()
    try:
        proc.wait(timeout=0.5)
    except Exception:
        proc.kill()

if not ip:
    raise SystemExit(1)
print(ip)
PY
}

hex_resolve() {
  local name="${1:-$HEXAPOD_MDNS_NAME}" ip cached
  if ip="$(hex_resolve_active "$name")"; then
    hex_cache_ip "$ip" || true
    echo "$ip"
    return 0
  fi
  if cached="$(hex_cached_ip)"; then
    echo "$cached"
    return 0
  fi
  return 1
}

hex_preferred_http_url() {
  local url="${1:-$HEXAPOD_HOST}" cached ip
  if ! hex_is_mdns_default "$url"; then
    echo "$url"
    return 0
  fi
  if cached="$(hex_cached_ip)"; then
    echo "$(hex_url_for_ip "$url" "$cached")"
    return 0
  fi
  if ip="$(hex_resolve_active "$HEXAPOD_MDNS_NAME" 2>/dev/null)"; then
    hex_cache_ip "$ip" || true
    echo "$(hex_url_for_ip "$url" "$ip")"
    return 0
  fi
  echo "$url"
}

hex_preferred_ssh_target() {
  local target="${1:-$HEXAPOD_SSH}" cached ip
  if ! hex_is_mdns_default "$target"; then
    echo "$target"
    return 0
  fi
  if cached="$(hex_cached_ip)"; then
    echo "$(hex_ssh_for_ip "$target" "$cached")"
    return 0
  fi
  if ip="$(hex_resolve_active "$HEXAPOD_MDNS_NAME" 2>/dev/null)"; then
    hex_cache_ip "$ip" || true
    echo "$(hex_ssh_for_ip "$target" "$ip")"
    return 0
  fi
  echo "$target"
}

hex_check() {
  hex_py_syntax || return
  hex_js_syntax || return
  hex_diff_check
}

hex_unit_check() {
  hex_check || return
  hex_note "off-robot unit tests (fake buses only)"
  (
    hex_py linux_control/test_calibration_checkup.py &&
    hex_py linux_control/test_mcu_stream.py &&
    hex_py linux_control/test_geometry_sweep_fit.py &&
    hex_py linux_control/test_quad_pitch_trim.py &&
    hex_py linux_control/test_safe_zero.py &&
    hex_py linux_control/test_pinned_tip.py
  )
}

hex_status() {
  local host_url ip robot_json
  host_url="$(hex_preferred_http_url "$HEXAPOD_HOST")"
  hex_note "robot HTTP health (read-only)"
  if ! curl -fsS -m "$HEXAPOD_CURL_TIMEOUT" "$host_url/api/ping"; then
    if hex_is_mdns_default "$HEXAPOD_HOST" \
        && ip="$(hex_resolve_active "$HEXAPOD_MDNS_NAME" 2>/dev/null)"; then
      hex_cache_ip "$ip" || true
      host_url="$(hex_url_for_ip "$HEXAPOD_HOST" "$ip")"
      echo >&2
      hex_note "retrying with fresh mDNS address: $host_url"
      if ! curl -fsS -m "$HEXAPOD_CURL_TIMEOUT" "$host_url/api/ping"; then
        echo >&2
        echo "!! resolved $ip, but HTTP still failed" >&2
        return 1
      fi
    else
      echo >&2
      echo "!! could not reach $HEXAPOD_HOST" >&2
      echo "   Try: HEXAPOD_HOST=http://<board-ip>:8080 make robot-status" >&2
      echo "   or use adb forwarding, then: HEXAPOD_HOST=http://127.0.0.1:8080 make robot-status" >&2
      echo "   Cached IP file: $HEXAPOD_IP_CACHE" >&2
      echo "   To refresh mDNS/cache: make robot-resolve" >&2
      return 1
    fi
  fi
  echo
  if ! robot_json="$(curl -fsS -m "$HEXAPOD_CURL_TIMEOUT" \
      "$host_url/api/robot")"; then
    echo >&2
    echo "!! /api/robot failed after /api/ping succeeded ($host_url)" >&2
    return 1
  fi
  hex_py -c '
import json
import sys

d = json.loads(sys.stdin.read())
servo = d.get("servo") or {}
demo = d.get("demo") or {}
print(
    "activity={activity} mode={mode} armed={armed} "
    "demo={demo_name}/{demo_running} servos={live}/{expected} "
    "imu_ok={imu_ok} drive={drive_status}".format(
        activity=d.get("activity"),
        mode=d.get("mode"),
        armed=d.get("armed"),
        demo_name=demo.get("name"),
        demo_running=demo.get("running"),
        live=servo.get("live"),
        expected=servo.get("expected"),
        imu_ok=servo.get("imu_ok"),
        drive_status=d.get("drive_status"),
    )
)
' <<<"$robot_json"
}

hex_remote_compile() {
  local ssh_target
  ssh_target="$(hex_preferred_ssh_target "$HEXAPOD_SSH")"
  hex_note "remote syntax check (read-only except pycache)"
  ssh -o BatchMode=yes \
    -o ConnectTimeout="${HEXAPOD_SSH_TIMEOUT:-10}" \
    -o StrictHostKeyChecking=accept-new \
    -o HostKeyAlias="$HEXAPOD_SSH_HOSTKEY_ALIAS" \
    "$ssh_target" \
    "cd '$HEX_REMOTE_ROOT' && /home/arduino/.local/bin/uv run python -m py_compile \
      linux_control/web_drive.py \
      linux_control/bench_api.py \
      linux_control/drive_controller.py \
      linux_control/cpg_controller_loader.py \
      linux_control/urt2_setup/inplace_demos.py \
      linux_control/urt2_setup/motion_telemetry.py \
      motor_setup/inplace_demos.py \
      motor_setup/motion_telemetry.py"
}

hex_deploy() {
  hex_check || return
  hex_note "deploy over SSH using existing global deploy lock"
  HEXAPOD_HOST="$(hex_preferred_http_url "$HEXAPOD_HOST")" \
    HEXAPOD_SSH="$(hex_preferred_ssh_target "$HEXAPOD_SSH")" \
    HEXAPOD_SSH_HOSTKEY_ALIAS="$HEXAPOD_SSH_HOSTKEY_ALIAS" \
    HEXAPOD_IP_CACHE="$HEXAPOD_IP_CACHE" \
    "$HEX_LC_DIR/deploy_ssh.sh" || return
  hex_remote_compile || return
  hex_status
}

hex_deploy_fast() {
  hex_check || return
  hex_note "fast deploy over SSH (skip remote compile/status; deploy waits for /api/ping)"
  HEXAPOD_HOST="$(hex_preferred_http_url "$HEXAPOD_HOST")" \
    HEXAPOD_SSH="$(hex_preferred_ssh_target "$HEXAPOD_SSH")" \
    HEXAPOD_SSH_HOSTKEY_ALIAS="$HEXAPOD_SSH_HOSTKEY_ALIAS" \
    HEXAPOD_IP_CACHE="$HEXAPOD_IP_CACHE" \
    "$HEX_LC_DIR/deploy_ssh.sh"
}

hex_commit_push() {
  if [ "$#" -lt 2 ]; then
    echo "usage: hex_commit_push 'commit message' path [path ...]" >&2
    return 2
  fi
  local msg="$1"
  shift
  hex_check || return
  git -C "$HEX_ROOT" add -- "$@" || return
  git -C "$HEX_ROOT" diff --cached --stat || return
  git -C "$HEX_ROOT" diff --cached --check || return
  git -C "$HEX_ROOT" commit -m "$msg" || return
  git -C "$HEX_ROOT" push origin "$(git -C "$HEX_ROOT" branch --show-current)"
}

hex_help() {
  cat <<'EOF'
Fast hexapod dev-loop helpers:
  hex_check          Local syntax + JS + git diff hygiene. No robot access.
  hex_unit_check     hex_check plus fake-bus/off-robot unit tests.
  hex_resolve        Refresh/print cached robot IP for hexapod.local.
  hex_status         Read-only /api/ping + compact /api/robot summary.
  hex_deploy         hex_check, deploy_ssh.sh, remote compile, hex_status.
  hex_deploy_fast    hex_check, deploy_ssh.sh only; deploy waits for /api/ping.
  hex_commit_push    Commit/push explicit paths only:
                    hex_commit_push "message" path [path ...]

Environment:
  HEXAPOD_HOST       default http://hexapod.local:8080
  HEXAPOD_SSH        default arduino@hexapod.local
  HEXAPOD_SSH_HOSTKEY_ALIAS default hexapod.local
  HEX_REMOTE_ROOT    default /home/arduino/hexapod_sts
  HEXAPOD_IP_CACHE   default ~/.hexapod/last_ip
  Local and Uno Q Python commands use uv run python.
EOF
}

if [ "$_HEX_EXECUTED" = "1" ]; then
  cmd="${1:-help}"
  shift || true
  case "$cmd" in
    check) hex_check "$@" ;;
    unit-check) hex_unit_check "$@" ;;
    resolve) hex_resolve "$@" ;;
    status) hex_status "$@" ;;
    deploy) hex_deploy "$@" ;;
    deploy-fast) hex_deploy_fast "$@" ;;
    remote-compile) hex_remote_compile "$@" ;;
    commit-push) hex_commit_push "$@" ;;
    help|-h|--help) hex_help ;;
    *)
      echo "unknown command: $cmd" >&2
      hex_help >&2
      exit 2
      ;;
  esac
fi
