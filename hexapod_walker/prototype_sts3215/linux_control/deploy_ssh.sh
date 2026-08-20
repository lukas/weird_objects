#!/usr/bin/env bash
# Deploy linux_control to the Uno Q over SSH (key auth) — the no-USB
# twin of deploy_adb.sh (same file list; keep the two in sync). Used
# when the board is only reachable over the network.
#
#   ./deploy_ssh.sh              # push + restart web UI
#   ./deploy_ssh.sh --stop       # stop the web server on the board
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)"
HOST="${HEXAPOD_SSH:-arduino@hexapod.local}"
REMOTE="/home/arduino/hexapod_sts"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST")

# Serialize deploys across workspaces (lock ~/.hexapod/deploy.lock,
# history ~/.hexapod/deploy.log — see deploy_lock.sh).
source "$SRC/deploy_lock.sh"
deploy_lock_acquire "ssh ${1:-push+restart}"

if [ "${1:-}" = "--stop" ]; then
  "${SSH[@]}" 'pkill -f "[p]ython3 .*web_drive.py" || true'
  echo ">> stopped"
  exit 0
fi

# Stage the EXACT remote layout locally, then ship it as ONE tar over
# one ssh connection (the old per-file scp made ~40 round trips and a
# deploy took ~4 minutes; this takes seconds).
echo ">> staging deploy tree"
STAGE="$(mktemp -d /tmp/hexapod_deploy.XXXXXX)"
trap 'rm -rf "$STAGE"; deploy_lock_release' EXIT
mkdir -p "$STAGE/linux_control" "$STAGE/motor_setup" \
  "$STAGE/urt2_setup" "$STAGE/rl_move/sim"

cp "$SRC/tripod_gait.py" "$SRC/drive_controller.py" \
  "$SRC/mcu_feetech_bus.py" "$SRC/bench_api.py" "$SRC/web_drive.py" \
  "$SRC/xbox_drive.py" "$SRC/joint_calibrate.py" \
  "$SRC/plant_calibrate.py" "$SRC/imu_calibrate.py" \
  "$SRC/event_log.py" "$SRC/status_display.py" "$SRC/servo_watch.py" \
  "$SRC/mpu_probe.py" "$SRC/rl_policy.py" "$SRC/safe_zero.py" \
  "$SRC/pinned_tip.py" "$SRC/noslip_gait.py" \
  "$SRC/sysid_protocol.py" "$SRC/sysid_runner.py" "$SRC/bus_bench.py" \
  "$SRC/rl_policy_weights.json" "$SRC/rl_walk_weights.json" \
  "$SRC/standup_modes.json" \
  "$STAGE/linux_control/"
cp -R "$SRC/webui" "$SRC/policies" "$SRC/vendor" "$STAGE/linux_control/"

# rl_move core (numpy-only) + rot-60 canonicalizer + sagittal mirror —
# same list as deploy_adb.sh.
for f in __init__.py env.py robot_state.py attitude.py safety.py \
         config.py config.yaml body_ik.py control_loop.py logger.py \
         np_policy.py; do
  cp "$SRC/../rl_move/$f" "$STAGE/rl_move/"
done
for f in __init__.py rot60.py mirror.py; do
  cp "$SRC/../rl_move/sim/$f" "$STAGE/rl_move/sim/"
done

# Setup bundle + canonical motor_setup copies.
cp -R "$SRC/urt2_setup/." "$STAGE/urt2_setup/"
mkdir -p "$STAGE/linux_control/urt2_setup"
cp -R "$SRC/urt2_setup/." "$STAGE/linux_control/urt2_setup/"
# urt2_setup sits BEFORE motor_setup on the service PYTHONPATH, so a stale
# bundle copy silently shadows the canonical file (bit us 2026-08-19:
# demos ran an old inplace_demos.py for two test rounds). Overwrite the
# shared modules in BOTH urt2_setup dirs from motor_setup so the deployed
# tree can never disagree with the canonical copies.
for f in feetech_bus.py urt2_bench.py inplace_demos.py quad_walk.py \
         dance_script.py motion_telemetry.py motor_setup_registry.json; do
  cp "$SRC/../motor_setup/$f" "$STAGE/motor_setup/"
  cp "$SRC/../motor_setup/$f" "$STAGE/urt2_setup/"
  cp "$SRC/../motor_setup/$f" "$STAGE/linux_control/urt2_setup/"
done
touch "$STAGE/motor_setup/__init__.py" "$STAGE/linux_control/__init__.py"

echo ">> pushing code + vendored SDK -> $HOST:$REMOTE (single tar|ssh)"
# COPYFILE_DISABLE: keep macOS bsdtar from tucking ._* AppleDouble files
# into the stream (GNU tar on the board would extract them as junk).
COPYFILE_DISABLE=1 tar -C "$STAGE" -czf - . \
  | "${SSH[@]}" "mkdir -p '$REMOTE' && tar -xzf - -C '$REMOTE'"

echo ">> restarting web_drive.py"
if "${SSH[@]}" 'systemctl is-enabled hexapod-web.service >/dev/null 2>&1'; then
  "${SSH[@]}" 'echo arduino | sudo -S systemctl restart hexapod-web.service' \
    >/dev/null
  sleep 5
  "${SSH[@]}" 'systemctl --no-pager -l status hexapod-web.service \
    | head -5 || true'
else
  "${SSH[@]}" "pkill -f '[p]ython3 .*web_drive.py' || true" || true
  "${SSH[@]}" "sh -c 'cd \"$REMOTE/linux_control\" && \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=\"$REMOTE/linux_control/vendor:$REMOTE/urt2_setup:$REMOTE/motor_setup:$REMOTE/linux_control\" \
    nohup python3 web_drive.py --port mcu --http-port 8080 \
    --https-port 8443 > \"$REMOTE/web_drive.log\" 2>&1 < /dev/null &'"
  sleep 5
fi

echo ">> verify over HTTP"
curl -s -m 8 http://hexapod.local:8080/api/ping || true
echo
echo ">> done"
