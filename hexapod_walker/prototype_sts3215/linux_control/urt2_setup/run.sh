#!/usr/bin/env bash
# Run motor setup on the Uno Q (MCU UART bridge) or a laptop (USB URT).
#
#   ./run.sh                 # interactive menu
#   ./run.sh --status
#   ./run.sh --port mcu
#   ./run.sh --port /dev/ttyUSB0
#
# On the Uno Q, web_drive owns /dev/ttyHS1 — this script stops it for the
# session and restarts it afterward.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
LC="$(cd "$HERE/.." && pwd)"
REMOTE_HOME="${HOME}/hexapod_sts/linux_control"
export PYTHONPATH="${HERE}/vendor:${HERE}:${LC}:${REMOTE_HOME}:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
export HEXAPOD_BUS_PORT="${HEXAPOD_BUS_PORT:-mcu}"

WEB_WAS_RUNNING=0
restart_web() {
  if [ "$WEB_WAS_RUNNING" -eq 1 ]; then
    echo ">> restarting web_drive on MCU bridge ..."
    (
      cd "$LC"
      PYTHONUNBUFFERED=1 \
      PYTHONPATH="${LC}/vendor:${HERE}:${LC}/../motor_setup:${LC}:${PYTHONPATH:-}" \
      nohup python3 web_drive.py --port mcu --http-port 8080 --https-port 8443 \
        >/tmp/hexapod_web.log 2>&1 </dev/null &
    ) || true
  fi
}
trap restart_web EXIT

if pgrep -f '[p]ython3 .*web_drive.py' >/dev/null 2>&1; then
  WEB_WAS_RUNNING=1
  echo ">> stopping web_drive (needs exclusive bus) ..."
  pkill -f '[p]ython3 .*web_drive.py' || true
  sleep 0.4
fi

exec python3 "${HERE}/urt2_motor_setup.py" "$@"
