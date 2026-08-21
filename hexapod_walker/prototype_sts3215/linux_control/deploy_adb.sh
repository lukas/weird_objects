#!/usr/bin/env bash
# Deploy STS3215 linux_control to the Uno Q over USB adb and start the web UI.
#
#   ./deploy_adb.sh              # push + start (dry-run if no URT-2 yet)
#   ./deploy_adb.sh --bus        # require a real Feetech bus adapter
#   ./deploy_adb.sh --stop       # stop the web server on the board
#
# Then on the Mac:
#   adb forward tcp:8080 tcp:8080
#   adb forward tcp:8443 tcp:8443
#   open http://127.0.0.1:8080
#   # Xbox on the Mac → https://127.0.0.1:8443  (accept cert warning)
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)"
REMOTE="/home/arduino/hexapod_sts"
MODE="start"
NEED_BUS=0
for a in "$@"; do
  case "$a" in
    --stop) MODE="stop" ;;
    --bus) NEED_BUS=1 ;;
  esac
done

# Serialize deploys across workspaces (lock ~/.hexapod/deploy.lock,
# history ~/.hexapod/deploy.log — see deploy_lock.sh).
source "$SRC/deploy_lock.sh"
deploy_lock_acquire "adb $MODE"

echo ">> waiting for Uno Q over adb ..."
adb wait-for-device
adb shell 'echo connected as $(whoami) on $(hostname)'

if [ "$MODE" = "stop" ]; then
  adb shell 'pkill -f "[p]ython3 .*web_drive.py" || true'
  echo ">> stopped"
  exit 0
fi

echo ">> staging motor-setup deps into linux_control/urt2_setup"
"$SRC/deploy_urt2_setup.sh" >/dev/null || true
# Also keep a local urt2_setup stage next to web_drive for PYTHONPATH.

echo ">> pushing code + vendored SDK → $REMOTE"
adb shell "mkdir -p '$REMOTE/linux_control' '$REMOTE/motor_setup' '$REMOTE/urt2_setup'"
adb push "$SRC/tripod_gait.py" "$REMOTE/linux_control/"
adb push "$SRC/drive_controller.py" "$REMOTE/linux_control/"
adb push "$SRC/mcu_feetech_bus.py" "$REMOTE/linux_control/"
adb push "$SRC/bench_api.py" "$REMOTE/linux_control/"
adb push "$SRC/web_drive.py" "$REMOTE/linux_control/"
adb push "$SRC/webui" "$REMOTE/linux_control/"
adb push "$SRC/xbox_drive.py" "$REMOTE/linux_control/"
adb push "$SRC/joint_calibrate.py" "$REMOTE/linux_control/"
adb push "$SRC/plant_calibrate.py" "$REMOTE/linux_control/"
adb push "$SRC/geometry_plant.py" "$REMOTE/linux_control/"
adb push "$SRC/imu_calibrate.py" "$REMOTE/linux_control/"
adb push "$SRC/event_log.py" "$REMOTE/linux_control/"
adb push "$SRC/status_display.py" "$REMOTE/linux_control/"
adb push "$SRC/servo_watch.py" "$REMOTE/linux_control/"
adb push "$SRC/mpu_probe.py" "$REMOTE/linux_control/"
adb push "$SRC/rl_policy.py" "$REMOTE/linux_control/"
adb push "$SRC/safe_zero.py" "$REMOTE/linux_control/"
adb push "$SRC/pinned_tip.py" "$REMOTE/linux_control/"
adb push "$SRC/noslip_gait.py" "$REMOTE/linux_control/"
adb push "$SRC/sysid_protocol.py" "$REMOTE/linux_control/"
adb push "$SRC/sysid_runner.py" "$REMOTE/linux_control/"
adb push "$SRC/bus_bench.py" "$REMOTE/linux_control/"
adb push "$SRC/rl_policy_weights.json" "$REMOTE/linux_control/"
adb push "$SRC/rl_walk_weights.json" "$REMOTE/linux_control/"
# Swappable policy registry (bench_api rl_policies/rl_policy_select):
# every exported candidate ships so the operator can A/B on the bench.
adb push "$SRC/policies" "$REMOTE/linux_control/"
# Stand-up lab: baked keyframes from rl_move/sim/compare_standup.py --export.
adb push "$SRC/standup_modes.json" "$REMOTE/linux_control/"
adb push "$SRC/vendor" "$REMOTE/linux_control/"
# rl_move core (numpy-only): obs builder, state estimator, safety layer —
# imported by rl_policy.py for the RL stand/lower buttons.
adb shell "mkdir -p '$REMOTE/rl_move'"
for f in __init__.py env.py robot_state.py attitude.py safety.py \
         config.py config.yaml body_ik.py control_loop.py logger.py \
         np_policy.py; do
  adb push "$SRC/../rl_move/$f" "$REMOTE/rl_move/"
done
# rot-60 canonicalizer + sagittal mirror (numpy-only; sim/__init__.py
# is a bare docstring) — full-circle walk headings (RL_PLAN queue 2.1)
# and turn= chirality selection (TURN.md deploy port) for rl_policy.py.
adb shell "mkdir -p '$REMOTE/rl_move/sim'"
for f in __init__.py rot60.py mirror.py; do
  adb push "$SRC/../rl_move/sim/$f" "$REMOTE/rl_move/sim/"
done
# Full setup bundle (demos + bench helpers) for Motors/Demos tabs.
adb push "$SRC/urt2_setup/." "$REMOTE/urt2_setup/"
adb push "$SRC/urt2_setup/." "$REMOTE/linux_control/urt2_setup/"
# Canonical motor_setup copies (feetech + friends).
for f in feetech_bus.py urt2_bench.py inplace_demos.py quad_walk.py \
         motion_telemetry.py motor_setup_registry.json; do
  adb push "$SRC/../motor_setup/$f" "$REMOTE/motor_setup/"
done
adb shell "touch '$REMOTE/motor_setup/__init__.py' '$REMOTE/linux_control/__init__.py'"

BUS_ARGS=""
DRY=""
if [ "$NEED_BUS" -eq 0 ]; then
  # Prefer MCU UART bridge (FE-URT on D0/D1); else USB URT; else dry-run.
  if adb shell 'test -e /dev/ttyHS1' >/dev/null 2>&1; then
    echo ">> using MCU Feetech bridge (/dev/ttyHS1)"
    BUS_ARGS="--port mcu"
  elif adb shell 'ls /dev/ttyUSB* /dev/ttyCH343USB* 2>/dev/null | head -1' | grep -q .; then
    echo ">> using USB Feetech adapter"
  else
    echo ">> no bus yet — starting with --dry-run"
    echo "   (flash feetech_bridge + wire URT UART→D0/D1, or plug USB URT)"
    DRY="--dry-run"
  fi
else
  BUS_ARGS="--port mcu"
fi

echo ">> restarting web_drive.py"
# Prefer the boot-enabled systemd unit when present.
if adb shell 'systemctl is-enabled hexapod-web.service >/dev/null 2>&1'; then
  adb shell 'echo arduino | sudo -S systemctl restart hexapod-web.service' >/dev/null
  sleep 5
  adb shell 'systemctl --no-pager -l status hexapod-web.service | head -20 || true'
else
  adb shell "pkill -f '[p]ython3 .*web_drive.py' || true" >/dev/null || true
  # Detach cleanly — a bare `adb shell '... &'` can hang until the child exits.
  adb shell "sh -c 'cd \"$REMOTE/linux_control\" && \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=\"$REMOTE/linux_control/vendor:$REMOTE/urt2_setup:$REMOTE/motor_setup:$REMOTE/linux_control\" \
    nohup python3 web_drive.py $DRY $BUS_ARGS --http-port 8080 --https-port 8443 \
    >/tmp/hexapod_web.log 2>&1 </dev/null & echo started_pid=\$!'"
  sleep 1.5
  adb shell 'tail -n 30 /tmp/hexapod_web.log || true'
fi

adb forward --remove-all >/dev/null 2>&1 || true
adb forward tcp:8080 tcp:8080
adb forward tcp:8443 tcp:8443

echo ">> log / listen:"
adb shell 'journalctl -u hexapod-web -n 20 --no-pager 2>/dev/null || tail -n 30 /tmp/hexapod_web.log || true'
adb shell 'ss -tln 2>/dev/null | grep -E ":8080|:8443" || netstat -tln 2>/dev/null | grep -E ":8080|:8443" || true'
echo
echo ">> open on this Mac:"
echo "     http://127.0.0.1:8080"
echo "     https://127.0.0.1:8443   (Xbox / Gamepad API — accept cert warning)"
echo ">> live log:  adb shell 'tail -f /tmp/hexapod_web.log'"
