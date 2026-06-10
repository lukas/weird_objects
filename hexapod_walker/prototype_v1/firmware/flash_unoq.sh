#!/usr/bin/env bash
# Flash an Arduino UNO Q sketch over USB when available, else over SSH.
#
# USB path:  normal `arduino-cli compile --upload` (needs the board's
#            /dev/cu.usbmodem* device to be enumerated).
# SSH path:  on the UNO Q, USB is only a tunnel for adb to copy the binary
#            to the board's onboard Linux processor -- the actual flash is
#            done by OpenOCD running ON the board (linuxgpiod SWD driver).
#            So when USB is down we compile here, scp the signed binary to
#            the board, and run the board's own `remoteocd` in local mode.
#
# Usage:
#   ./flash_unoq.sh [sketch_dir] [usb|ssh|auto] [board_host]
#
# Defaults: sketch_dir=prototype_servo_test  transport=auto
#           board_host=arduino@192.168.0.192
# Requires: host arduino-cli with the arduino:zephyr core; for SSH mode,
#           passwordless key auth to the board.
set -euo pipefail

SKETCH="${1:-prototype_servo_test}"
TRANSPORT="${2:-auto}"
BOARD="${3:-arduino@192.168.0.192}"
FQBN="arduino:zephyr:unoq"
HERE="$(cd "$(dirname "$0")" && pwd)"
SKETCH_DIR="$HERE/$SKETCH"
SKETCH_NAME="$(basename "$SKETCH_DIR")"
BUILD_DIR="/tmp/${SKETCH_NAME}_unoq_build"
ZSK="$BUILD_DIR/${SKETCH_NAME}.ino.elf-zsk.bin"

# Board-side paths (fixed by the installed zephyr core layout).
REMOTEOCD='~/.arduino15/packages/arduino/tools/remoteocd/0.1.1/remoteocd'
FLASH_CFG='~/.arduino15/packages/arduino/hardware/zephyr/0.55.2/variants/arduino_uno_q_stm32u585xx/flash_sketch.cfg'

usb_port() { ls /dev/cu.usbmodem* 2>/dev/null | head -1 || true; }

flash_usb() {
  local port="$1"
  echo ">> USB flash via $port ..."
  arduino-cli compile --upload -b "$FQBN" -p "$port" "$SKETCH_DIR"
}

flash_ssh() {
  echo ">> compiling $SKETCH_NAME on this Mac ..."
  rm -rf "$BUILD_DIR"
  arduino-cli compile -b "$FQBN" --build-path "$BUILD_DIR" "$SKETCH_DIR" >/dev/null
  [ -f "$ZSK" ] || { echo "ERROR: signed binary not found: $ZSK" >&2; exit 1; }
  echo "   built $(basename "$ZSK") ($(wc -c <"$ZSK") bytes)"
  echo ">> copying binary to $BOARD ..."
  scp -q "$ZSK" "$BOARD:~/${SKETCH_NAME}.elf-zsk.bin"
  echo ">> flashing on the board via its local OpenOCD (no USB) ..."
  ssh -o BatchMode=yes "$BOARD" \
    "$REMOTEOCD upload -f $FLASH_CFG ~/${SKETCH_NAME}.elf-zsk.bin" 2>&1 \
    | grep -E "Examination succeed|idcode|write_image|Padding|shutdown|Error|error" || true
}

case "$TRANSPORT" in
  usb)
    PORT="$(usb_port)"
    [ -n "$PORT" ] || { echo "ERROR: no /dev/cu.usbmodem* device found" >&2; exit 1; }
    flash_usb "$PORT"
    ;;
  ssh)
    flash_ssh
    ;;
  auto)
    PORT="$(usb_port)"
    if [ -n "$PORT" ]; then
      if ! flash_usb "$PORT"; then
        echo ">> USB upload failed -- falling back to SSH ..."
        flash_ssh
      fi
    else
      echo ">> no USB device -- using SSH ..."
      flash_ssh
    fi
    ;;
  *)
    echo "ERROR: transport must be usb, ssh, or auto (got '$TRANSPORT')" >&2
    exit 1
    ;;
esac

echo ">> done. The STM32 was reset into the new sketch."
