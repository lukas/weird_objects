#!/usr/bin/env bash
# Compile + flash feetech_bridge on the Uno Q over SSH (no USB needed).
#
#   ./flash_feetech_bridge.sh
#   ./flash_feetech_bridge.sh arduino@hexapod.local
set -euo pipefail

BOARD="${1:-arduino@hexapod.local}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SKETCH="$HERE/feetech_bridge"
REMOTE_DIR="~/feetech_bridge"
PASS="${HEXAPOD_SSH_PASSWORD:-arduino}"

ssh_pw() {
  if command -v sshpass >/dev/null 2>&1; then
    sshpass -p "$PASS" ssh -o PreferredAuthentications=password \
      -o PubkeyAuthentication=no "$@"
  else
    ssh "$@"
  fi
}
scp_pw() {
  if command -v sshpass >/dev/null 2>&1; then
    sshpass -p "$PASS" scp -o PreferredAuthentications=password \
      -o PubkeyAuthentication=no "$@"
  else
    scp "$@"
  fi
}

echo ">> push sketch to $BOARD"
ssh_pw "$BOARD" "mkdir -p $REMOTE_DIR"
scp_pw "$SKETCH/feetech_bridge.ino" "$SKETCH/st7789_tft.h" "$BOARD:$REMOTE_DIR/"

echo ">> ensure FTServo library"
ssh_pw "$BOARD" 'arduino-cli lib install FTServo >/dev/null 2>&1 || true'

echo ">> compile + flash via local remoteocd"
ssh_pw "$BOARD" 'bash -s' <<'EOF'
set -e
rm -rf /tmp/feetech_bridge_build
arduino-cli compile -b arduino:zephyr:unoq --build-path /tmp/feetech_bridge_build ~/feetech_bridge
REMOTEOCD=~/.arduino15/packages/arduino/tools/remoteocd/0.0.4-rc.4/remoteocd
FLASH_CFG=~/.arduino15/packages/arduino/hardware/zephyr/0.51.0/variants/arduino_uno_q_stm32u585xx/flash_sketch.cfg
"$REMOTEOCD" upload -f "$FLASH_CFG" /tmp/feetech_bridge_build/feetech_bridge.ino.elf-zsk.bin
echo ">> flashed feetech_bridge"
EOF

echo ">> done"
