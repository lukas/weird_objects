#!/usr/bin/env bash
# Boot the Uno Q MCU now: ready poke + SWD reset via remoteocd (~5 s).
# Used by hexapod-mcu-ready.service at boot and available for recovery.
set -u
gpioset -c /dev/gpiochip1 -t0 37=0 2>/dev/null || true
OCD=$(ls -d /home/arduino/.arduino15/packages/arduino/tools/remoteocd/*/remoteocd 2>/dev/null | tail -1)
CFG=/home/arduino/hexapod_sts/linux_control/mcu_reset.cfg
if [ -n "${OCD:-}" ] && [ -x "$OCD" ] && [ -f "$CFG" ]; then
  "$OCD" upload -f "$CFG" /dev/null
  rc=$?
  # Running as root leaves root-owned scratch that breaks later
  # arduino-user sketch flashes ("chmod: operation not permitted").
  rm -rf /tmp/remoteocd
  exit $rc
fi
echo "mcu_boot: remoteocd or mcu_reset.cfg missing — skipped SWD reset" >&2
exit 0
