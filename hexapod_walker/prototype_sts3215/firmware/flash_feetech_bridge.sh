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

# A firmware flash mid-deploy is the worst interleave — take the same
# global lock the deploy scripts use (~/.hexapod/deploy.lock).
source "$HERE/../linux_control/deploy_lock.sh"
deploy_lock_acquire "flash feetech_bridge"

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

# The Arduino-Zephyr core's Serial ring buffers default to 64 bytes —
# smaller than one 113-byte binary 'W'/'S' frame, so bytes get dropped
# whenever a frame lands while the MCU is inside a servo/IMU
# transaction (measured 2026-08-19). The size is compiled into the
# sketch (llext), so patching the variant's generated config is safe
# and takes effect on the next compile. Idempotent; re-applies after
# core updates.
echo ">> ensure 1024-byte Arduino serial ring buffers"
ssh_pw "$BOARD" 'sed -i \
  "s/#define CONFIG_ARDUINO_API_SERIAL_BUFFER_SIZE 64$/#define CONFIG_ARDUINO_API_SERIAL_BUFFER_SIZE 1024/" \
  ~/.arduino15/packages/arduino/hardware/zephyr/*/variants/arduino_uno_q_stm32u585xx/llext-edk/include/zephyr/include/generated/zephyr/autoconf.h
  grep -h "ARDUINO_API_SERIAL_BUFFER_SIZE" \
  ~/.arduino15/packages/arduino/hardware/zephyr/*/variants/arduino_uno_q_stm32u585xx/llext-edk/include/zephyr/include/generated/zephyr/autoconf.h'

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
