#!/usr/bin/env python3
"""
Xbox (Bluetooth) gamepad -> hexapod drive bridge.

Runs on the robot's onboard Linux side (the Arduino UNO Q's Qualcomm
processor).  It reads a paired Bluetooth Xbox controller straight from the
Linux input layer (no external Python packages -- pure stdlib, parses
/dev/input/eventN itself) and translates sticks/buttons into the firmware's
serial command language, which it sends to the STM32 sketch through the
on-board arduino-router monitor port (TCP 127.0.0.1:7500 -- the same pipe the
interactive `socat tcp:127.0.0.1:7500 -` session uses).

The matching firmware is firmware/prototype_servo_test/prototype_servo_test.ino.
It must be the build that has the live-drive `J vx vy omega [gait]` command
(added Jun 2026 alongside the gait selector + dances).

------------------------------------------------------------------ controls --
  Left stick            drive: up/down = forward/back, left/right = strafe
  Right stick (X)       turn left / right
  Right stick CLICK     cycle gait: tripod -> ripple -> wave -> tetrapod
  Right trigger (RT)    speed boost while driving (also a dance-bank modifier)
  D-pad alone           COM trim nudge: Up/Down = lean fwd/back, L/R = lean L/R
  Modifier + D-pad      dance (D-pad U/R/D/L picks one of four in the bank):
      LB -> 1-4   RB -> 5-8   LT -> 9-12   RT -> 13-16   LB+RB -> 17-20
                                 (20 dances total -- see firmware help)
  A                     stand / park (planted stance, stops driving)
  B                     relax all motors (limp)
  X                     centre every joint
  Y                     legs up over the body
  Start                 arm (begin driving); pushing a stick also arms
  Back / Xbox(Guide)    quit this program (parks the robot first)

Just pushing the left stick auto-arms and drives.  A centred stick makes the
robot stand in place (the firmware fades the foot lift to zero at ~0 speed),
so steering is instant -- it doesn't march on the spot.  A dance plays until
you grab a stick again, which resumes walking.

------------------------------------------------------------------- running --
  # one-time: pair the controller (see linux_control/README.md), then:
  sudo python3 xbox_drive.py            # sudo: reading /dev/input needs it
  python3 xbox_drive.py --list          # list input devices + pick one
  python3 xbox_drive.py --selftest      # no gamepad: scripted drive demo
"""

import argparse
import fcntl
import glob
import os
import select
import signal
import socket
import struct
import sys
import time

# ----------------------------------------------------------------- evdev raw
# struct input_event { struct timeval time; __u16 type, code; __s32 value; }
# On 64-bit Linux timeval is two longs, so the record is 'llHHi' = 24 bytes.
_EV_FMT = "llHHi"
_EV_SIZE = struct.calcsize(_EV_FMT)

EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03

# Absolute axes (linux/input-event-codes.h)
ABS_X, ABS_Y, ABS_Z = 0x00, 0x01, 0x02
ABS_RX, ABS_RY, ABS_RZ = 0x03, 0x04, 0x05
ABS_HAT0X, ABS_HAT0Y = 0x10, 0x11

# Gamepad buttons (standard kernel mapping; Xbox face buttons)
BTN_A, BTN_B, BTN_X, BTN_Y = 0x130, 0x131, 0x133, 0x134
BTN_TL, BTN_TR = 0x136, 0x137          # LB / RB
BTN_SELECT, BTN_START, BTN_MODE = 0x13a, 0x13b, 0x13c
BTN_THUMBL, BTN_THUMBR = 0x13d, 0x13e

# ioctl helpers (asm-generic _IOC encoding; _IOC_READ = 2)
def _ioc(direction, typ, nr, size):
    return (direction << 30) | (size << 16) | (typ << 8) | nr

def _eviocgname(length):
    return _ioc(2, ord("E"), 0x06, length)

def _eviocgabs(axis):
    return _ioc(2, ord("E"), 0x40 + axis, 24)   # sizeof(struct input_absinfo)


def device_name(fd):
    buf = bytearray(256)
    try:
        fcntl.ioctl(fd, _eviocgname(len(buf)), buf)
    except OSError:
        return ""
    return buf.split(b"\x00", 1)[0].decode("utf-8", "replace")


def abs_info(fd, axis):
    """Return (min, max, flat) for an absolute axis, or None if absent."""
    buf = bytearray(24)
    try:
        fcntl.ioctl(fd, _eviocgabs(axis), buf)
    except OSError:
        return None
    value, minimum, maximum, fuzz, flat, res = struct.unpack("iiiiii", bytes(buf))
    if maximum == minimum:
        return None
    return (minimum, maximum, flat)


_GAMEPAD_HINTS = ("xbox", "x-box", "microsoft", "controller", "gamepad",
                  "wireless", "8bitdo", "pro controller")


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
            is_pad = abs_info(fd, ABS_X) is not None
        finally:
            os.close(fd)
        out.append((path, name, is_pad))
    return out


def find_gamepad():
    """Pick the first input device that looks like a gamepad (has ABS_X and a
    gamepad-ish name)."""
    candidates = []
    for path in sorted(glob.glob("/dev/input/event*")):
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            name = device_name(fd)
            has_stick = abs_info(fd, ABS_X) is not None
        finally:
            os.close(fd)
        if not has_stick:
            continue
        score = 1 if any(h in name.lower() for h in _GAMEPAD_HINTS) else 0
        candidates.append((score, path, name))
    if not candidates:
        return None, None
    candidates.sort(reverse=True)        # prefer name-matched, then lowest event#
    _, path, name = candidates[0]
    return path, name


# --------------------------------------------------------------- serial link
class Link:
    """Line-oriented sender to the firmware over the router monitor TCP port."""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.sock = None

    def _connect(self):
        try:
            s = socket.create_connection((self.host, self.port), timeout=3.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock = s
            # Drain + discard firmware output so the recv buffer never fills
            # and stalls the link (we only ever send).
            import threading
            threading.Thread(target=self._drain, args=(s,), daemon=True).start()
            print(f"[link] connected to {self.host}:{self.port}")
        except OSError as e:
            self.sock = None
            print(f"[link] connect failed: {e}")

    def _drain(self, s):
        try:
            while True:
                if not s.recv(4096):
                    break
        except OSError:
            pass

    def send(self, line):
        if self.sock is None:
            self._connect()
            if self.sock is None:
                return False
        try:
            self.sock.sendall((line + "\n").encode("ascii"))
            return True
        except OSError as e:
            print(f"[link] send failed ({e}); will reconnect")
            try:
                self.sock.close()
            finally:
                self.sock = None
            return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None


# ---------------------------------------------------------------- axis maths
def norm_axis(raw, calib, dead=0.12):
    """Normalise a stick axis to [-1, 1] with a centred dead-zone."""
    if calib is None:
        return 0.0
    lo, hi, flat = calib
    centre = (lo + hi) / 2.0
    half = (hi - lo) / 2.0
    if half <= 0:
        return 0.0
    v = (raw - centre) / half           # -1 .. 1
    # combine the kernel's suggested flat dead-zone with our own
    dz = max(dead, (flat / half) if half else 0.0)
    if abs(v) <= dz:
        return 0.0
    v = (abs(v) - dz) / (1.0 - dz) * (1.0 if v > 0 else -1.0)
    return max(-1.0, min(1.0, v))


def expo(v, amount=0.35):
    """Soften around centre for finer low-speed control."""
    return (1.0 - amount) * v + amount * v * v * v


def norm_trigger(raw, calib):
    if calib is None:
        return 0.0
    lo, hi, _ = calib
    if hi <= lo:
        return 0.0
    return max(0.0, min(1.0, (raw - lo) / (hi - lo)))


GAIT_NAMES = ("tripod", "ripple", "wave", "tetrapod")


def run(args):
    link = Link(args.host, args.port)

    if args.selftest:
        return selftest(link, args)

    if args.device:
        dev_path, dev_name = args.device, "(forced)"
    else:
        dev_path, dev_name = find_gamepad()
    if not dev_path:
        print("No gamepad found. Pair it first (see README.md), then try "
              "`--list` or pass `--device /dev/input/eventN`.", file=sys.stderr)
        return 2
    print(f"[pad] using {dev_path}  ({dev_name})")

    try:
        fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        print(f"Cannot open {dev_path}: {e}\n"
              f"Reading /dev/input needs permission -- run with sudo or add "
              f"your user to the 'input' group.", file=sys.stderr)
        return 2

    calib = {ax: abs_info(fd, ax) for ax in
             (ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ)}
    raw = {ax: ((calib[ax][0] + calib[ax][1]) // 2 if calib[ax] else 0)
           for ax in calib}
    raw[ABS_HAT0X] = 0
    raw[ABS_HAT0Y] = 0

    armed = False
    dance_pause = False
    gait = 0
    quit_flag = {"v": False}
    last_line = ""        # last J packet sent (dedup -- don't flood the bridge)
    last_send = 0.0
    last_status = ""
    held = set()          # currently-held buttons (for the LB/RB dance modifiers)
    com_x = 0.0           # COM lean trim (mm) nudged by the bare D-pad
    com_y = 0.0

    def status(msg):
        nonlocal last_status
        if msg != last_status:
            print("[drive] " + msg)
            last_status = msg

    def on_button(code):
        nonlocal armed, dance_pause, gait, last_line
        last_line = ""        # force the next J to resend after any button
        if code == BTN_START:
            armed, dance_pause = True, False
            status("armed (driving)")
        elif code == BTN_A:
            armed = False
            link.send("P")
            status("parked (stand)")
        elif code == BTN_B:
            armed, dance_pause = False, False
            link.send("X")
            status("motors relaxed")
        elif code == BTN_X:
            link.send("C")
            status("centred")
        elif code == BTN_Y:
            armed = False
            link.send("U")
            status("legs up")
        elif code in (BTN_THUMBR, BTN_THUMBL):    # stick click: cycle gait
            gait = (gait + 1) % 4
            status(f"gait -> {GAIT_NAMES[gait]}")
        elif code in (BTN_SELECT, BTN_MODE):
            quit_flag["v"] = True
        # NB: BTN_TL/BTN_TR (LB/RB) are now HELD modifiers for the dance banks,
        # so they do nothing on their own press edge (see the dispatch below).

    def on_dance(cmd, label):
        nonlocal dance_pause, last_line
        link.send(cmd)
        dance_pause = True              # stop streaming J so it isn't cancelled
        last_line = ""                  # force resend when driving resumes
        status(f"dance: {label} (push a stick to resume driving)")

    signal.signal(signal.SIGINT, lambda *_: quit_flag.__setitem__("v", True))

    print("Ready. Push the left stick (or press Start) to drive. "
          "Back/Xbox button or Ctrl-C quits.")

    while not quit_flag["v"]:
        r, _, _ = select.select([fd], [], [], 0.02)
        if r:
            try:
                data = os.read(fd, _EV_SIZE * 64)
            except BlockingIOError:
                data = b""
            except OSError as e:
                print(f"[pad] read error: {e}", file=sys.stderr)
                break
            for off in range(0, len(data) - _EV_SIZE + 1, _EV_SIZE):
                _, _, etype, code, value = struct.unpack(
                    _EV_FMT, data[off:off + _EV_SIZE])
                if etype == EV_ABS:
                    raw[code] = value
                elif etype == EV_KEY:
                    if value == 1:
                        held.add(code)
                        on_button(code)
                    elif value == 0:
                        held.discard(code)

            # D-pad -> dance (if a shoulder/trigger modifier is held) or COM trim
            # nudge (if not).  Banks: LB=1-4 RB=5-8 LT=9-12 RT=13-16 LB+RB=17-20.
            hx, hy = raw.get(ABS_HAT0X, 0), raw.get(ABS_HAT0Y, 0)
            lb = BTN_TL in held
            rb = BTN_TR in held
            lt = raw.get(ABS_Z, 0) > 100
            rt = raw.get(ABS_RZ, 0) > 100
            if lb and rb: bank = 4
            elif lb:      bank = 0
            elif rb:      bank = 1
            elif lt:      bank = 2
            elif rt:      bank = 3
            else:         bank = -1
            DANCE_NAMES = {1: "wave", 2: "say hi", 3: "hula", 4: "twist",
                           5: "march", 6: "boogie", 7: "pinwheel", 8: "freakout",
                           9: "pogo", 10: "cancan", 11: "corkscrew", 12: "shimmy",
                           13: "twist-stomp", 14: "tippy-taps", 15: "disco point",
                           16: "rave", 17: "breathe", 18: "sway", 19: "nod",
                           20: "slow wave"}
            d = 0
            if hy < 0:   d = 1       # up
            elif hx > 0: d = 2       # right
            elif hy > 0: d = 3       # down
            elif hx < 0: d = 4       # left
            if d:
                if bank >= 0:
                    n = bank * 4 + d
                    on_dance(f"M{n}", DANCE_NAMES[n])
                else:                       # bare D-pad = COM trim nudge
                    if   d == 1: com_x = min(25.0, com_x + 2.0)   # lean fwd
                    elif d == 3: com_x = max(-25.0, com_x - 2.0)  # lean back
                    elif d == 4: com_y = min(25.0, com_y + 2.0)   # lean left
                    elif d == 2: com_y = max(-25.0, com_y - 2.0)  # lean right
                    link.send(f"E {com_x:.0f} {com_y:.0f}")
                    last_line = ""
                    status(f"trim X={com_x:.0f} Y={com_y:.0f} mm")
            raw[ABS_HAT0X] = 0          # consume so it fires once per press
            raw[ABS_HAT0Y] = 0

        # ---- compute drive vector from the sticks
        ly = norm_axis(raw[ABS_Y], calib[ABS_Y])
        lx = norm_axis(raw[ABS_X], calib[ABS_X])
        rx = norm_axis(raw[ABS_RX], calib[ABS_RX])
        rt = norm_trigger(raw[ABS_RZ], calib[ABS_RZ])

        moving = (abs(ly) + abs(lx) + abs(rx)) > 0.02
        if moving:
            armed = True
            dance_pause = False

        boost = 1.0 + args.turbo * rt
        vx = -expo(ly) * args.max_vx * boost      # stick up = forward
        vy = -expo(lx) * args.max_vy * boost      # stick right = strafe right
        omega = -expo(rx) * args.max_omega        # stick right = turn right

        now = time.monotonic()
        if armed and not dance_pause:
            line = f"J {vx:.0f} {vy:.0f} {omega:.3f} {gait}"
            if line != last_line or now - last_send > 0.3:   # change or ~3 Hz beat
                link.send(line)
                last_line = line
                last_send = now

    # park on exit
    link.send("P")
    time.sleep(0.1)
    link.close()
    try:
        os.close(fd)
    except OSError:
        pass
    print("\nbye -- robot parked in stance.")
    return 0


def selftest(link, args):
    """No gamepad needed: drive a short scripted sequence to prove the pipe
    (board -> router -> STM32) works end to end."""
    print("[selftest] running a scripted drive demo over the serial pipe...")
    seq = [
        ("P", 1.5, "stand"),
        ("J 40 0 0 0", 3.0, "tripod forward 40 mm/s"),
        ("J 40 0 0.4 1", 3.0, "ripple forward + turn left"),
        ("J 0 35 0 2", 3.0, "wave strafe left"),
        ("J 0 0 0 0", 1.0, "stand in place"),
        ("M1", 3.0, "dance 1: stadium wave"),
        ("M5", 3.0, "dance 5: tripod march"),
        ("M8", 3.0, "dance 8: freakout"),
        ("P", 1.0, "stand"),
        ("X", 0.2, "relax"),
    ]
    for cmd, hold, label in seq:
        ok = link.send(cmd)
        print(f"  -> {cmd:<14} {'(sent)' if ok else '(FAILED)'}  {label}")
        if cmd.startswith("J"):
            # stream it like the real driver so the gait keeps its velocities
            t_end = time.monotonic() + hold
            while time.monotonic() < t_end:
                link.send(cmd)
                time.sleep(0.05)
        else:
            time.sleep(hold)
    link.close()
    print("[selftest] done.")
    return 0


def main():
    ap = argparse.ArgumentParser(description="Xbox gamepad -> hexapod drive bridge")
    ap.add_argument("--host", default="127.0.0.1",
                    help="firmware serial bridge host (default 127.0.0.1)")
    ap.add_argument("--port", type=int, default=7500,
                    help="arduino-router monitor TCP port (default 7500)")
    ap.add_argument("--device", default=None,
                    help="force a specific /dev/input/eventN")
    ap.add_argument("--list", action="store_true",
                    help="list input devices and exit")
    ap.add_argument("--selftest", action="store_true",
                    help="no gamepad: run a scripted drive demo")
    ap.add_argument("--max-vx", type=float, default=55.0, dest="max_vx",
                    help="max forward speed mm/s (default 55)")
    ap.add_argument("--max-vy", type=float, default=40.0, dest="max_vy",
                    help="max strafe speed mm/s (default 40)")
    ap.add_argument("--max-omega", type=float, default=0.8, dest="max_omega",
                    help="max turn rate rad/s (default 0.8)")
    ap.add_argument("--turbo", type=float, default=1.0,
                    help="extra speed multiplier at full RT (default 1.0 = up to 2x)")
    args = ap.parse_args()

    if args.list:
        for path, name, is_pad in list_devices():
            tag = "  <-- gamepad?" if is_pad else ""
            print(f"{path:<22} {name}{tag}")
        return 0
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
