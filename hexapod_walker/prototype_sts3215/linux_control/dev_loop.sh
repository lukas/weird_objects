#!/usr/bin/env bash
# Fast local helpers for the robot web/control dev loop.
#
# Usage:
#   source linux_control/dev_loop.sh
#   hex_check
#   hex_deploy
#
# Or run as a command:
#   linux_control/dev_loop.sh check
#   linux_control/dev_loop.sh deploy
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

hex_note() {
  printf '>> %s\n' "$*"
}

hex_py_syntax() {
  hex_note "python syntax check (no imports, no hardware)"
  python3 - "$HEX_ROOT" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
files = [
    "linux_control/web_drive.py",
    "linux_control/bench_api.py",
    "linux_control/drive_controller.py",
    "linux_control/mcu_feetech_bus.py",
    "linux_control/event_log.py",
    "linux_control/status_display.py",
    "linux_control/safe_zero.py",
    "linux_control/pinned_tip.py",
    "linux_control/plant_calibrate.py",
    "linux_control/geometry_plant.py",
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

hex_resolve() {
  local name="${1:-hexapod.local}"
  python3 - "$name" <<'PY'
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

hex_check() {
  hex_py_syntax || return
  hex_js_syntax || return
  hex_diff_check
}

hex_unit_check() {
  hex_check || return
  hex_note "off-robot unit tests (fake buses only)"
  (
    cd "$HEX_LC_DIR"
    python3 test_mcu_stream.py &&
    python3 test_quad_pitch_trim.py &&
    python3 test_safe_zero.py &&
    python3 test_pinned_tip.py
  )
}

hex_status() {
  local host_url ip robot_json
  host_url="$HEXAPOD_HOST"
  hex_note "robot HTTP health (read-only)"
  if ! curl -fsS -m "$HEXAPOD_CURL_TIMEOUT" "$host_url/api/ping"; then
    if [ "${HEXAPOD_HOST#*hexapod.local}" != "$HEXAPOD_HOST" ] \
        && ip="$(hex_resolve hexapod.local 2>/dev/null)"; then
      host_url="${HEXAPOD_HOST/hexapod.local/$ip}"
      echo >&2
      hex_note "retrying with current mDNS address: $host_url"
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
      echo "   To print the current mDNS answer: make robot-resolve" >&2
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
  python3 -c '
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
  hex_note "remote syntax check (read-only except pycache)"
  ssh -o BatchMode=yes \
    -o ConnectTimeout="${HEXAPOD_SSH_TIMEOUT:-10}" \
    -o StrictHostKeyChecking=accept-new \
    -o HostKeyAlias="$HEXAPOD_SSH_HOSTKEY_ALIAS" \
    "$HEXAPOD_SSH" \
    "cd '$HEX_REMOTE_ROOT' && python3 -m py_compile \
      linux_control/web_drive.py \
      linux_control/bench_api.py \
      linux_control/drive_controller.py \
      linux_control/urt2_setup/inplace_demos.py \
      linux_control/urt2_setup/motion_telemetry.py \
      motor_setup/inplace_demos.py \
      motor_setup/motion_telemetry.py"
}

hex_deploy() {
  hex_check || return
  hex_note "deploy over SSH using existing global deploy lock"
  HEXAPOD_HOST="$HEXAPOD_HOST" HEXAPOD_SSH="$HEXAPOD_SSH" \
    HEXAPOD_SSH_HOSTKEY_ALIAS="$HEXAPOD_SSH_HOSTKEY_ALIAS" \
    "$HEX_LC_DIR/deploy_ssh.sh" || return
  hex_remote_compile || return
  hex_status
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
  hex_resolve        Print current mDNS IPv4 answer for hexapod.local.
  hex_status         Read-only /api/ping + compact /api/robot summary.
  hex_deploy         hex_check, deploy_ssh.sh, remote compile, hex_status.
  hex_commit_push    Commit/push explicit paths only:
                    hex_commit_push "message" path [path ...]

Environment:
  HEXAPOD_HOST       default http://hexapod.local:8080
  HEXAPOD_SSH        default arduino@hexapod.local
  HEXAPOD_SSH_HOSTKEY_ALIAS default hexapod.local
  HEX_REMOTE_ROOT    default /home/arduino/hexapod_sts
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
