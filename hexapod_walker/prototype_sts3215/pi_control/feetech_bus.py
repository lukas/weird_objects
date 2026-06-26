#!/usr/bin/env python3
"""Arduino Uno Q direct driver for the FEETECH STS3215 serial-bus servos.

June 2026 redesign: the prototype dropped the Arduino Mega + 2x PCA9685
PWM stack (and the AS5600 add-on encoders) in favour of 18x FEETECH
STS3215 (ST-3215-C018, 12 V / 30 kg-cm) smart serial-bus servos.  Each
servo carries its own 12-bit magnetic encoder and reports position,
speed, load, voltage, current and temperature back over the bus, so the
robot gets closed-loop joint feedback "for free" with NO external
sensors and NO PWM driver boards.

Topology (see ../firmware/WIRING.md):

    Arduino Uno Q UART --half-duplex TTL bus, 1 Mbps-->
                  servo 1 --> servo 2 --> ... --> servo 18
    12 V (3S LiPo / bench supply) injected on the bus V+ rail per leg,
    common ground with the Uno Q.  The Uno Q (on-board Linux SoC + MCU)
    runs this file AND drives the half-duplex STS3215 bus directly: it
    replaces BOTH the Raspberry Pi and the separate USB->TTL bus adapter
    (no FE-URT-1 / Waveshare adapter).  This file is the ONLY controller
    -- the Uno Q talks the STS protocol directly; there is no other
    microcontroller in the loop.

Joint model (unchanged from the old Arduino bridge so poses, the MuJoCo
sim and the docs all still line up):

    joint = leg * 3 + axis          leg = 0..5
    axis 0 = yaw, axis 1 = hip pitch, axis 2 = knee pitch
    logical joint j  ->  servo ID (j + 1)   [IDs 1..18, set once with `setid`]

Per-axis SAFE software limits (MUST match mujoco_prototype._leg_xml and
check_workspace_self_collision in _verify_prototype.py):

    yaw:        -35 .. +35 deg
    hip pitch:  -80 .. +30 deg
    knee pitch: -20 .. +80 deg

Trims: the old Arduino bridge used to persist per-joint trims in
EEPROM.  Trims now live in a JSON file next to this script
(``feetech_trims.json``) on the Uno Q and are applied in software
before the angle->step conversion.

Examples
--------

    # One-time per servo (only ONE servo on the bus at a time!):
    python feetech_bus.py --port /dev/ttyACM0 setid --from 1 --to 5

    # Discover everything currently on the bus
    python feetech_bus.py --port /dev/ttyACM0 scan

    # Centre every joint (0 deg)
    python feetech_bus.py --port /dev/ttyACM0 centre

    # Move joint 0 to +20 deg, eased
    python feetech_bus.py --port /dev/ttyACM0 joint 0 20 --sweep

    # Standing tripod pose on all 18 joints
    python feetech_bus.py --port /dev/ttyACM0 stance

    # Live feedback table (the whole point of the swap)
    python feetech_bus.py --port /dev/ttyACM0 feedback --watch

    # Limp the robot (torque off) so you can pose it by hand
    python feetech_bus.py --port /dev/ttyACM0 relax
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Joint model (mirrors firmware/prototype_servo_bridge.ino and mujoco_prototype)
# ---------------------------------------------------------------------------

N_JOINTS = 18

# Per-axis safe limits, axis = joint % 3 (0 yaw, 1 hip, 2 knee).
AXIS_LIMITS_DEG = {
    0: (-35.0, 35.0),    # yaw
    1: (-80.0, 30.0),    # hip pitch
    2: (-20.0, 80.0),    # knee pitch
}
TRIM_LIMIT_DEG = 30.0

# STS3215: 0..4096 counts over a full 360 deg, neutral ("centre") = 2048.
STS_COUNTS_PER_REV = 4096
STS_CENTRE_COUNT = 2048
COUNTS_PER_DEG = STS_COUNTS_PER_REV / 360.0   # 11.3778 counts/deg

# Per-joint mechanical direction.  +1 means servo CW (increasing count) is
# the positive joint direction; -1 flips it.  Left/right legs and the
# yaw/hip/knee horn orientations may need individual flips -- calibrate on
# the bench with `feedback` + `joint`, then set these.  Default +1.
JOINT_SIGN = [1] * N_JOINTS

# Default servo profile: moderate speed + gentle acceleration so a freshly
# powered, possibly-misaligned leg eases to target instead of slamming.
DEFAULT_SPEED = 1500    # steps/s (0 = max)
DEFAULT_ACC = 30        # 0..255, *100 steps/s^2

BAUD_DEFAULT = 1_000_000   # STS3215 factory default

TRIM_PATH = Path(__file__).resolve().parent / "feetech_trims.json"


def axis_of(joint: int) -> int:
    return joint % 3


def clampf(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def joint_limits(joint: int) -> tuple[float, float]:
    return AXIS_LIMITS_DEG[axis_of(joint)]


def joint_to_servo_id(joint: int) -> int:
    """Logical joint 0..17 -> servo bus ID 1..18."""
    return joint + 1


def standing_pose_degrees() -> list[float]:
    """Neutral tripod standing pose (yaw 0, hip -25, knee +60 per leg)."""
    out: list[float] = []
    for _leg in range(6):
        out.extend([0.0, -25.0, 60.0])
    return out


def deg_to_count(joint: int, deg: float, trim: float) -> int:
    """Convert a joint angle (deg) + trim to an STS3215 goal count.

    Applies the per-joint sign, clamps to the axis limit, then maps
    0 deg -> centre count (2048).
    """
    lo, hi = joint_limits(joint)
    corrected = clampf(deg + trim, lo, hi)
    count = STS_CENTRE_COUNT + JOINT_SIGN[joint] * corrected * COUNTS_PER_DEG
    return int(round(clampf(count, 0, STS_COUNTS_PER_REV - 1)))


def count_to_deg(joint: int, count: int) -> float:
    """Inverse of deg_to_count (ignoring trim) -- for feedback display."""
    return JOINT_SIGN[joint] * (count - STS_CENTRE_COUNT) / COUNTS_PER_DEG


# ---------------------------------------------------------------------------
# Trim persistence (replaces the Arduino's EEPROM block)
# ---------------------------------------------------------------------------

def load_trims() -> list[float]:
    if TRIM_PATH.exists():
        try:
            data = json.loads(TRIM_PATH.read_text())
            trims = [float(data.get(str(i), 0.0)) for i in range(N_JOINTS)]
            return [clampf(t, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG) for t in trims]
        except (ValueError, OSError):
            pass
    return [0.0] * N_JOINTS


def save_trims(trims: list[float]) -> None:
    TRIM_PATH.write_text(json.dumps({str(i): round(t, 3)
                                     for i, t in enumerate(trims)}, indent=2))


# ---------------------------------------------------------------------------
# Bus driver (thin wrapper over the official feetech-servo-sdk `sts` class)
# ---------------------------------------------------------------------------

def _import_sdk():
    try:
        import scservo_sdk as scs
        return scs
    except ImportError as exc:
        raise SystemExit(
            "feetech-servo-sdk is required.  Install on the Uno Q "
            "(or your laptop) with:\n"
            "  python -m pip install feetech-servo-sdk\n"
            "(provides the `scservo_sdk` module used here)."
        ) from exc


# STS/SMS control-table addresses used for the generic feedback reads that
# the high-level `sts` helper doesn't wrap directly.
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMP = 63
ADDR_PRESENT_CURRENT = 69


class FeetechBus:
    def __init__(self, port: str, baud: int = BAUD_DEFAULT):
        scs = _import_sdk()
        self.scs = scs
        self.port = scs.PortHandler(port)
        if not self.port.openPort():
            raise SystemExit(f"Failed to open bus port {port!r}")
        if not self.port.setBaudRate(baud):
            raise SystemExit(f"Failed to set baud {baud} on {port!r}")
        self.pkt = scs.sts(self.port)
        self.trims = load_trims()

    # -- low level ---------------------------------------------------------
    def ping(self, sid: int) -> bool:
        _model, result, _err = self.pkt.ping(sid)
        return result == self.scs.COMM_SUCCESS

    def scan(self, id_range=range(1, 31)) -> list[int]:
        return [sid for sid in id_range if self.ping(sid)]

    def torque(self, sid: int, on: bool) -> None:
        self.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def set_id(self, old_id: int, new_id: int) -> None:
        # STS: unlock EEPROM (lock reg 55 = 0), write ID (reg 5), relock.
        self.pkt.unLockEprom(old_id)
        self.pkt.write1ByteTxRx(old_id, 5, new_id)
        self.pkt.LockEprom(new_id)

    # -- motion ------------------------------------------------------------
    def write_joint(self, joint: int, deg: float,
                    speed: int = DEFAULT_SPEED, acc: int = DEFAULT_ACC) -> None:
        count = deg_to_count(joint, deg, self.trims[joint])
        self.pkt.WritePosEx(joint_to_servo_id(joint), count, speed, acc)

    def write_all(self, degrees, speed: int = DEFAULT_SPEED,
                  acc: int = DEFAULT_ACC) -> None:
        """Group sync-write all joints so the whole robot moves in one packet."""
        for joint, deg in enumerate(degrees):
            count = deg_to_count(joint, deg, self.trims[joint])
            self.pkt.SyncWritePosEx(joint_to_servo_id(joint), count, speed, acc)
        self.pkt.groupSyncWrite.txPacket()
        self.pkt.groupSyncWrite.clearParam()

    def centre(self, **kw) -> None:
        self.write_all([0.0] * N_JOINTS, **kw)

    # -- feedback (the reason for the swap) --------------------------------
    def read_position_deg(self, joint: int) -> float | None:
        pos, result, _err = self.pkt.ReadPos(joint_to_servo_id(joint))
        if result != self.scs.COMM_SUCCESS:
            return None
        return count_to_deg(joint, pos)

    def read_feedback(self, joint: int) -> dict | None:
        sid = joint_to_servo_id(joint)
        pos, result, _err = self.pkt.ReadPos(sid)
        if result != self.scs.COMM_SUCCESS:
            return None
        load, _r, _e = self.pkt.read2ByteTxRx(sid, ADDR_PRESENT_LOAD)
        volt, _r, _e = self.pkt.read1ByteTxRx(sid, ADDR_PRESENT_VOLTAGE)
        temp, _r, _e = self.pkt.read1ByteTxRx(sid, ADDR_PRESENT_TEMP)
        cur, _r, _e = self.pkt.read2ByteTxRx(sid, ADDR_PRESENT_CURRENT)
        # STS load: bit 10 = direction, lower 10 bits = magnitude (0.1%).
        load_pct = (load & 0x3FF) / 10.0
        return {
            "joint": joint,
            "id": sid,
            "deg": count_to_deg(joint, pos),
            "load_pct": load_pct,
            "volt": volt / 10.0,        # 0.1 V units
            "temp_c": temp,             # deg C
            "current_a": cur * 0.0065,  # ~6.5 mA/LSB
        }

    def close(self) -> None:
        self.port.closePort()


# ---------------------------------------------------------------------------
# CLI commands
# ---------------------------------------------------------------------------

def cmd_scan(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        found = bus.scan()
        print(f"Found {len(found)} servo(s): {found}")
        if found != list(range(1, N_JOINTS + 1)):
            print(f"  expected IDs 1..{N_JOINTS} for a fully-built robot.")
    finally:
        bus.close()


def cmd_setid(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        if not bus.ping(args.frm):
            raise SystemExit(f"No servo answered at ID {args.frm}. "
                             "Only ONE servo may be on the bus when re-IDing.")
        bus.set_id(args.frm, args.to)
        ok = bus.ping(args.to)
        print(f"ID {args.frm} -> {args.to}: {'OK' if ok else 'FAILED'}")
    finally:
        bus.close()


def cmd_centre(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        bus.centre()
        print("OK centre")
    finally:
        bus.close()


def cmd_joint(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        if args.sweep:
            here = bus.read_position_deg(args.joint) or 0.0
            steps = max(2, int(abs(args.deg - here) / 2.0))
            for k in range(steps + 1):
                u = k / steps
                bus.write_joint(args.joint, here + (args.deg - here) * u)
                time.sleep(0.04)
        else:
            bus.write_joint(args.joint, args.deg)
        print(f"OK joint {args.joint} -> {args.deg:.2f} deg")
    finally:
        bus.close()


def cmd_wiggle(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        for deg in (0, +15, 0, -15, 0, +30, 0, -30, 0):
            lo, hi = joint_limits(args.joint)
            bus.write_joint(args.joint, clampf(deg, lo, hi))
            time.sleep(args.delay)
        print(f"OK wiggle joint {args.joint}")
    finally:
        bus.close()


def cmd_stance(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        bus.write_all(standing_pose_degrees())
        print("OK stance")
    finally:
        bus.close()


def cmd_relax(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        targets = [args.joint] if args.joint is not None else range(N_JOINTS)
        for j in targets:
            bus.torque(joint_to_servo_id(j), False)
        print(f"OK relax ({'joint ' + str(args.joint) if args.joint is not None else 'all'})")
    finally:
        bus.close()


def cmd_hold(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        targets = [args.joint] if args.joint is not None else range(N_JOINTS)
        for j in targets:
            bus.torque(joint_to_servo_id(j), True)
        print(f"OK hold ({'joint ' + str(args.joint) if args.joint is not None else 'all'})")
    finally:
        bus.close()


def cmd_feedback(args):
    bus = FeetechBus(args.port, args.baud)
    hdr = f"{'jnt':>3} {'id':>3} {'deg':>8} {'load%':>7} {'volt':>6} {'temp':>5} {'amp':>6}"
    try:
        while True:
            print(hdr)
            print("-" * len(hdr))
            for j in range(N_JOINTS):
                fb = bus.read_feedback(j)
                if fb is None:
                    print(f"{j:>3} {joint_to_servo_id(j):>3} {'--- no reply ---':>30}")
                    continue
                print(f"{fb['joint']:>3} {fb['id']:>3} {fb['deg']:>8.2f} "
                      f"{fb['load_pct']:>7.1f} {fb['volt']:>6.1f} "
                      f"{fb['temp_c']:>5d} {fb['current_a']:>6.2f}")
            if not args.watch:
                break
            time.sleep(args.interval)
            print()
    finally:
        bus.close()


def cmd_trim(args):
    bus = FeetechBus(args.port, args.baud)
    try:
        bus.trims[args.joint] = clampf(args.deg, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG)
        save_trims(bus.trims)
        bus.write_joint(args.joint, 0.0)   # re-apply at neutral to see effect
        print(f"OK trim joint {args.joint} = {bus.trims[args.joint]:.2f} deg "
              f"(saved to {TRIM_PATH.name})")
        print("TRIM " + " ".join(f"{t:.2f}" for t in bus.trims))
    finally:
        bus.close()


def _default_port() -> str:
    import glob
    for pattern in ("/dev/ttyACM0", "/dev/ttyUSB0",
                    "/dev/cu.usbserial*", "/dev/cu.usbmodem*"):
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return "/dev/ttyUSB0"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=_default_port(),
                    help="Bus adapter serial port (e.g. /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan").set_defaults(func=cmd_scan)

    p = sub.add_parser("setid", help="re-ID the ONLY servo on the bus")
    p.add_argument("--from", dest="frm", type=int, required=True)
    p.add_argument("--to", type=int, required=True)
    p.set_defaults(func=cmd_setid)

    sub.add_parser("centre").set_defaults(func=cmd_centre)

    p = sub.add_parser("joint")
    p.add_argument("joint", type=int)
    p.add_argument("deg", type=float)
    p.add_argument("--sweep", action="store_true")
    p.set_defaults(func=cmd_joint)

    p = sub.add_parser("wiggle")
    p.add_argument("--joint", type=int, default=0)
    p.add_argument("--delay", type=float, default=0.35)
    p.set_defaults(func=cmd_wiggle)

    sub.add_parser("stance").set_defaults(func=cmd_stance)

    p = sub.add_parser("relax", help="torque OFF (limp) so you can pose by hand")
    p.add_argument("--joint", type=int, default=None)
    p.set_defaults(func=cmd_relax)

    p = sub.add_parser("hold", help="torque ON (hold position)")
    p.add_argument("--joint", type=int, default=None)
    p.set_defaults(func=cmd_hold)

    p = sub.add_parser("feedback", help="print position/load/volt/temp/current")
    p.add_argument("--watch", action="store_true")
    p.add_argument("--interval", type=float, default=0.5)
    p.set_defaults(func=cmd_feedback)

    p = sub.add_parser("trim")
    p.add_argument("joint", type=int)
    p.add_argument("deg", type=float)
    p.set_defaults(func=cmd_trim)

    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
