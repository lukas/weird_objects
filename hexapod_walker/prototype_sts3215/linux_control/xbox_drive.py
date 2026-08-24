#!/usr/bin/env python3
"""Bluetooth Xbox → STS3215 drive (runs on Uno Q Linux).

Reads ``/dev/input/event*`` (stdlib only). Joysticks feed ``DriveController``;
face / shoulder chords hit the local web bench API when ``web_drive`` is up
(``http://127.0.0.1:8080``), otherwise only walk/turn/sit/stand via the bus.

  sudo /home/arduino/.local/bin/uv run python xbox_drive.py
  uv run python xbox_drive.py --list

Mapping
-------
- Left stick  → walk (vx / vy)
- Right stick → turn (omega)
- X alone     → sit zero
- Y alone     → stand zero
- A alone     → set HERE as zero (bench API)
- B alone     → stop demo
- Hold LB/LT/RB/RT + tap X/Y/A/B → 16 demos (LB easiest → RT hardest)
"""
from __future__ import annotations

import argparse
import fcntl
import glob
import json
import os
import select
import struct
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from drive_controller import DriveController  # noqa: E402

_EV_FMT = "llHHi"
_EV_SIZE = struct.calcsize(_EV_FMT)
EV_KEY, EV_ABS = 0x01, 0x03
ABS_X, ABS_Y, ABS_RX, ABS_Z, ABS_RZ = 0x00, 0x01, 0x03, 0x02, 0x05
BTN_A, BTN_B, BTN_X, BTN_Y = 0x130, 0x131, 0x133, 0x134
BTN_TL, BTN_TR = 0x136, 0x137          # LB / RB
BTN_TL2, BTN_TR2 = 0x138, 0x139        # LT / RT (some pads)
BTN_SELECT, BTN_MODE = 0x13a, 0x13c

# Face order within each modifier row: X, Y, A, B
PAD_DEMOS = {
    "LB": ["breathe", "heartbeat", "twinkle", "shimmy"],
    "LT": ["ripple", "conductor", "arms_up", "walk"],
    "RB": ["walk_spin", "plant_bounce", "plant_gallop", "plant_tripod"],
    "RT": ["walk_oval", "plant_star", "plant_stomp", "rise_show"],
}
FACE_ORDER = (BTN_X, BTN_Y, BTN_A, BTN_B)
FACE_NAME = {BTN_X: "X", BTN_Y: "Y", BTN_A: "A", BTN_B: "B"}


def _ioc(direction, typ, nr, size):
    return (direction << 30) | (size << 16) | (typ << 8) | nr


def device_name(fd):
    buf = bytearray(256)
    try:
        fcntl.ioctl(fd, _ioc(2, ord("E"), 0x06, len(buf)), buf)
    except OSError:
        return ""
    return buf.split(b"\x00", 1)[0].decode("utf-8", "replace")


def abs_info(fd, axis):
    buf = bytearray(24)
    try:
        fcntl.ioctl(fd, _ioc(2, ord("E"), 0x40 + axis, 24), buf)
    except OSError:
        return None
    _v, minimum, maximum, _f, flat, _r = struct.unpack("iiiiii", bytes(buf))
    if maximum == minimum:
        return None
    return minimum, maximum, flat


_HINTS = ("xbox", "x-box", "microsoft", "controller", "gamepad", "wireless")


def list_devices():
    out = []
    for path in sorted(glob.glob("/dev/input/event*")):
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            out.append((path, f"<cannot open: {e}>", False))
            continue
        try:
            name = device_name(fd)
            hint = any(h in name.lower() for h in _HINTS)
            out.append((path, name or "?", hint))
        finally:
            os.close(fd)
    return out


def pick_device(explicit=None):
    if explicit:
        return explicit
    for path, name, hint in list_devices():
        if hint:
            print(f"[xbox] using {path}  ({name})")
            return path
    return None


def norm_axis(fd, axis, value):
    info = abs_info(fd, axis)
    if not info:
        return 0.0
    lo, hi, flat = info
    mid = (lo + hi) / 2.0
    span = (hi - lo) / 2.0 or 1.0
    v = (value - mid) / span
    if abs(v) < max(0.12, (flat or 0) / span):
        return 0.0
    return max(-1.0, min(1.0, v))


def norm_trigger(fd, axis, value):
    """0..1 trigger; many Xbox pads report 0..255 or -32768..32767."""
    info = abs_info(fd, axis)
    if not info:
        return 0.0
    lo, hi, _flat = info
    if hi == lo:
        return 0.0
    v = (value - lo) / float(hi - lo)
    return max(0.0, min(1.0, v))


def bench_post(path: str, body: dict | None = None, *, base: str) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(
        base.rstrip("/") + path,
        data=data,
        headers={"Content-Type": "application/json"} if data else {},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            return json.loads(r.read().decode() or "{}")
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
        return {"ok": False, "error": str(e)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--device", default=None)
    ap.add_argument("--port", default=None, help="Feetech bus port")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--max-vx", type=float, default=40.0, help="mm/s")
    ap.add_argument("--max-omega", type=float, default=0.7)
    ap.add_argument("--api", default="http://127.0.0.1:8080",
                    help="web_drive bench API base")
    args = ap.parse_args()

    if args.list:
        for path, name, hint in list_devices():
            mark = "  <-- gamepad?" if hint else ""
            print(f"  {path:22}  {name}{mark}")
        return

    drive = DriveController(port=args.port, dry_run=args.dry_run)
    drive.start()
    drive.handle("ARM")
    drive.handle("P")

    if args.selftest:
        print("[xbox] selftest: forward 2s, turn 2s, stop")
        drive.handle(f"J {args.max_vx:.0f} 0 0 0")
        time.sleep(2)
        drive.handle(f"J 0 0 {args.max_omega:.3f} 0")
        time.sleep(2)
        drive.handle("J 0 0 0 0")
        drive.handle("SETTLE")
        time.sleep(4.5)
        drive.close()
        return

    path = pick_device(args.device)
    if not path:
        print("No Xbox/gamepad found. Pair over bluetoothctl, then --list.")
        drive.close()
        return

    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    print("[xbox] armed + standing.")
    print("  sticks=walk/turn  X=sit Y=stand A=set-zero B=stop")
    print("  hold LB/LT/RB/RT + X/Y/A/B = 16 demos  |  Back/Xbox quits")
    axes = {ABS_X: 0.0, ABS_Y: 0.0, ABS_RX: 0.0, ABS_Z: 0.0, ABS_RZ: 0.0}
    buttons: dict[int, int] = {}
    last_j = ""

    def mod_held() -> str | None:
        if buttons.get(BTN_TL):
            return "LB"
        if buttons.get(BTN_TL2) or axes.get(ABS_Z, 0.0) > 0.35:
            return "LT"
        if buttons.get(BTN_TR):
            return "RB"
        if buttons.get(BTN_TR2) or axes.get(ABS_RZ, 0.0) > 0.35:
            return "RT"
        return None

    try:
        while True:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                data = os.read(fd, _EV_SIZE * 32)
                for off in range(0, len(data) - _EV_SIZE + 1, _EV_SIZE):
                    _s, _u, typ, code, value = struct.unpack_from(
                        _EV_FMT, data, off)
                    if typ == EV_ABS and code in axes:
                        if code in (ABS_Z, ABS_RZ):
                            axes[code] = norm_trigger(fd, code, value)
                        else:
                            axes[code] = norm_axis(fd, code, value)
                    elif typ == EV_KEY:
                        prev = buttons.get(code, 0)
                        buttons[code] = value
                        if value and not prev:
                            mod = mod_held()
                            if code in FACE_ORDER and mod:
                                fi = FACE_ORDER.index(code)
                                name = PAD_DEMOS[mod][fi]
                                print(f"[xbox] {mod}+{FACE_NAME[code]} → {name}")
                                j = bench_post(
                                    "/api/demo",
                                    {"name": name, "speed": 1.0},
                                    base=args.api)
                                if not j.get("ok"):
                                    print(f"  demo fail: {j.get('error')}")
                            elif code == BTN_X and not mod:
                                print("[xbox] X → sit")
                                j = bench_post("/api/zero", {"pose": "sit"},
                                               base=args.api)
                                if not j.get("ok"):
                                    drive.handle("C")
                            elif code == BTN_Y and not mod:
                                print("[xbox] Y → stand")
                                j = bench_post("/api/zero", {"pose": "stand"},
                                               base=args.api)
                                if not j.get("ok"):
                                    drive.handle("P")
                            elif code == BTN_A and not mod:
                                print("[xbox] A → set-zero-here")
                                j = bench_post("/api/set_zero", base=args.api)
                                print(f"  {j}")
                            elif code == BTN_B and not mod:
                                print("[xbox] B → stop demo")
                                bench_post("/api/demo/stop", base=args.api)
                            elif code in (BTN_SELECT, BTN_MODE):
                                drive.handle("SETTLE")
                                time.sleep(4.5)
                                raise SystemExit
            x = axes[ABS_X]
            y = -axes[ABS_Y]
            t = axes[ABS_RX]
            vx = y * args.max_vx
            vy = -x * (args.max_vx * 0.73)
            om = -t * args.max_omega
            line = f"J {vx:.0f} {vy:.0f} {om:.3f} 0"
            if line != last_j:
                drive.handle(line)
                last_j = line
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            os.close(fd)
        except OSError:
            pass
        drive.handle("SETTLE")
        time.sleep(4.5)
        drive.close()


if __name__ == "__main__":
    main()
