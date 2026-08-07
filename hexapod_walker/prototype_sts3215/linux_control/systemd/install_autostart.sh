#!/usr/bin/env bash
# Install / enable hexapod-web.service on the Uno Q (run ON the board).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
UNIT_SRC="$HERE/hexapod-web.service"
UNIT_DST=/etc/systemd/system/hexapod-web.service
MCU_UNIT_SRC="$HERE/hexapod-mcu-ready.service"
MCU_UNIT_DST=/etc/systemd/system/hexapod-mcu-ready.service
SUDOERS_DST=/etc/sudoers.d/40-hexapod-web

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "missing $UNIT_SRC" >&2
  exit 1
fi

# Kill any hand-started copy so systemd owns the ports.
pkill -f '[p]ython3 .*web_drive.py' 2>/dev/null || true
sleep 1

echo ">> install unit → $UNIT_DST"
sudo cp "$UNIT_SRC" "$UNIT_DST"
sudo chmod 644 "$UNIT_DST"

if [[ -f "$MCU_UNIT_SRC" ]]; then
  echo ">> install unit → $MCU_UNIT_DST"
  sudo cp "$MCU_UNIT_SRC" "$MCU_UNIT_DST"
  sudo chmod 644 "$MCU_UNIT_DST"
  sudo systemctl enable hexapod-mcu-ready.service
fi

# Allow web_drive (via sudo -n) to reclaim / release the MCU UART.
echo ">> sudoers for arduino-router stop/start"
sudo tee "$SUDOERS_DST" >/dev/null <<'EOF'
# Hexapod web_drive claims /dev/ttyHS1 from arduino-router.
arduino ALL=(root) NOPASSWD: /bin/systemctl stop arduino-router, /bin/systemctl start arduino-router, /usr/bin/systemctl stop arduino-router, /usr/bin/systemctl start arduino-router
EOF
sudo chmod 440 "$SUDOERS_DST"
sudo visudo -cf "$SUDOERS_DST"

echo ">> enable + start hexapod-web"
sudo systemctl daemon-reload
sudo systemctl enable hexapod-web.service
sudo systemctl restart hexapod-web.service
sleep 5
sudo systemctl --no-pager --full status hexapod-web.service || true
curl -s -m 3 http://127.0.0.1:8080/api/ping && echo
echo "ok — hexapod-web enabled on boot"
