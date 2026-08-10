#!/usr/bin/env bash
# Join the Uno Q to WiFi over USB adb — and remember it.
#
#   ./wifi_join.sh                    # AUTO: join the network this Mac knows
#   ./wifi_join.sh --list             # networks in the air + saved on board
#   ./wifi_join.sh --status           # current connection + board IP
#   ./wifi_join.sh <SSID>             # prompt for password (hidden), join + save
#   ./wifi_join.sh <SSID> <password>  # non-interactive join + save
#   ./wifi_join.sh <SSID> --saved     # re-join a network the board already knows
#
# AUTO mode: the board scans the air, we intersect the visible SSIDs with
# the WiFi passwords saved in this Mac's System keychain, pick the
# strongest match, pull its password (macOS will pop an Allow dialog),
# and join. NetworkManager persists every successful join with
# autoconnect, so the board rejoins known networks on boot by itself.
set -euo pipefail

WIFI_DEV="wlan0"

# nmcli may need root depending on polkit; mirror deploy_adb.sh's sudo trick.
nm() {
  local q
  q=$(printf ' %q' "$@")
  adb shell "nmcli$q 2>/dev/null || echo arduino | sudo -S nmcli$q"
}

join() { # $1=ssid $2=password
  echo ">> joining '$1' (will be saved + autoconnect)"
  nm dev wifi connect "$1" password "$2" ifname "$WIFI_DEV"
  echo ">> connected; board address:"
  nm -t -f IP4.ADDRESS dev show "$WIFI_DEV" | cut -d: -f2
}

echo ">> waiting for Uno Q over adb ..."
adb wait-for-device

case "${1:-}" in
  "")
    echo ">> auto: scanning from the board ..."
    nm dev wifi rescan >/dev/null 2>&1 || true
    sleep 3
    # nmcli lists strongest first; de-dup, unescape \:, drop hidden (--).
    SSIDS=$(nm -t -f SSID dev wifi list | sed 's/\\:/:/g' | awk '!seen[$0]++ && $0 != "" && $0 != "--"')
    PICK=""
    while IFS= read -r ssid; do
      if security find-generic-password -D "AirPort network password" \
           -a "$ssid" /Library/Keychains/System.keychain >/dev/null 2>&1; then
        PICK="$ssid"; break
      fi
    done <<< "$SSIDS"
    if [ -z "$PICK" ]; then
      echo "!! none of the visible networks are saved on this Mac:"
      echo "$SSIDS" | sed 's/^/     /'
      echo "   run:  $0 <SSID>   to join one with a password"
      exit 1
    fi
    echo ">> Mac knows '$PICK' — fetching password (approve the macOS dialog)"
    PASS=$(security find-generic-password -D "AirPort network password" \
             -wa "$PICK" /Library/Keychains/System.keychain)
    join "$PICK" "$PASS"
    ;;
  --list)
    echo ">> networks in the air:"
    nm dev wifi rescan >/dev/null 2>&1 || true
    sleep 3
    nm -f SSID,SIGNAL,SECURITY dev wifi list
    echo
    echo ">> saved on the board (autoconnect on boot):"
    nm -t -f NAME,TYPE connection show | grep 802-11-wireless | cut -d: -f1
    ;;
  --status)
    nm -f GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS dev show "$WIFI_DEV"
    ;;
  *)
    SSID="$1"
    if [ "${2:-}" = "--saved" ]; then
      echo ">> re-joining saved network '$SSID'"
      nm connection up "$SSID"
      nm -t -f IP4.ADDRESS dev show "$WIFI_DEV" | cut -d: -f2
    else
      PASS="${2:-}"
      if [ -z "$PASS" ]; then
        read -r -s -p "Password for '$SSID': " PASS
        echo
      fi
      join "$SSID" "$PASS"
    fi
    ;;
esac
