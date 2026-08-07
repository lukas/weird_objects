#!/usr/bin/env bash
# Sync motor_setup + vendored SDK to the Uno Q and install ~/hexapod_sts/urt2_setup.
#
#   ./deploy_urt2_setup.sh           # push over adb
#   ./deploy_urt2_setup.sh --ssh     # push over ssh hexapod (Wi‑Fi)
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_MS="$HERE/../motor_setup"
STAGE="$HERE/urt2_setup"
REMOTE_DIR="/home/arduino/hexapod_sts/urt2_setup"
USE_SSH=0
for a in "$@"; do
  case "$a" in --ssh) USE_SSH=1 ;; esac
done

echo ">> staging $STAGE"
mkdir -p "$STAGE/vendor"
# Fresh copies from canonical motor_setup (keep run.sh / README).
for f in urt2_motor_setup.py feetech_bus.py urt2_bench.py \
         inplace_demos.py motion_telemetry.py motor_setup_registry.json; do
  cp "$REPO_MS/$f" "$STAGE/$f"
done
# Ensure executable bit on the entrypoint.
chmod +x "$STAGE/urt2_motor_setup.py" "$STAGE/run.sh"
# Vendored pyserial + scservo_sdk (offline Uno Q — no pip/network needed).
rsync -a --delete "$HERE/vendor/" "$STAGE/vendor/"
# Tiny package marker.
: > "$STAGE/__init__.py"

# Patch the staged docstring to mention Uno Q run path (optional clarity).
# (file content already has working default_port from motor_setup)

push_adb() {
  echo ">> waiting for adb ..."
  adb wait-for-device
  adb shell "mkdir -p '$REMOTE_DIR'"
  adb push "$STAGE/." "$REMOTE_DIR/"
  adb shell "chmod +x '$REMOTE_DIR/run.sh' '$REMOTE_DIR/urt2_motor_setup.py'"
  adb shell "ls -la '$REMOTE_DIR' | head -30"
  echo
  echo ">> on the board:"
  echo "     cd $REMOTE_DIR && ./run.sh"
  echo "   or from Mac:"
  echo "     adb shell \"cd $REMOTE_DIR && ./run.sh\""
}

push_ssh() {
  echo ">> scp → hexapod:$REMOTE_DIR"
  ssh -o ConnectTimeout=5 hexapod "mkdir -p '$REMOTE_DIR'"
  scp -r "$STAGE/"* "hexapod:$REMOTE_DIR/"
  ssh hexapod "chmod +x '$REMOTE_DIR/run.sh' '$REMOTE_DIR/urt2_motor_setup.py' && ls -la '$REMOTE_DIR' | head -30"
  echo
  echo ">> ssh hexapod"
  echo "     cd $REMOTE_DIR && ./run.sh"
}

if [ "$USE_SSH" -eq 1 ]; then
  push_ssh
else
  push_adb
fi

# Show bus devices on the board if reachable.
if adb devices 2>/dev/null | grep -q 'device$'; then
  echo ">> serial devices on board:"
  adb shell 'ls -la /dev/ttyUSB* /dev/ttyCH343USB* /dev/ttyACM* 2>/dev/null || echo "(none yet — plug URT-2 into OTG)"'
fi
