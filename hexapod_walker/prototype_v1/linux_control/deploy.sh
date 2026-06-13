#!/usr/bin/env bash
# Deploy the Linux-side control scripts to the robot's onboard Linux and
# restart the web control-panel service in one step.
#
#   ./deploy.sh                      # uses the default board below
#   ./deploy.sh arduino@10.0.0.42    # override the ssh target
#   BOARD=pi@hex.local ./deploy.sh   # or via env var
#
# Copies xbox_drive.py + web_drive.py (and the systemd unit) to the board,
# then restarts the hexapod-web user service so the new web_drive.py is live.
set -euo pipefail

BOARD="${1:-${BOARD:-arduino@192.168.0.192}}"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
XDG="XDG_RUNTIME_DIR=/run/user/1000"   # needed for `systemctl --user` over ssh

echo ">> deploying to $BOARD"
scp "$SRC_DIR/web_drive.py" "$SRC_DIR/xbox_drive.py" "$BOARD:~/"
scp "$SRC_DIR/hexapod-web.service" "$BOARD:/tmp/hexapod-web.service"

echo ">> refreshing the systemd unit + restarting the web panel"
ssh "$BOARD" "mkdir -p ~/.config/systemd/user \
  && cp /tmp/hexapod-web.service ~/.config/systemd/user/hexapod-web.service \
  && $XDG systemctl --user daemon-reload \
  && $XDG systemctl --user restart hexapod-web.service \
  && $XDG systemctl --user is-active hexapod-web.service"

# Report the URL using the host from the ssh target (skips docker/bridge IPs).
HOST="${BOARD##*@}"
echo ">> done. control panel: http://${HOST}:8080"
# Report whichever HTTPS port actually bound (443 if privileged, else 8443).
HTTPS_PORT="$(ssh "$BOARD" 'ss -tln 2>/dev/null | grep -oE ":(443|8443) " | tr -d ": " | head -1' 2>/dev/null || true)"
if [ "$HTTPS_PORT" = "443" ]; then
  echo ">>       Xbox controller (HTTPS): https://${HOST}  (accept the cert warning)"
else
  echo ">>       Xbox controller (HTTPS): https://${HOST}:${HTTPS_PORT:-8443}  (accept the cert warning)"
fi
