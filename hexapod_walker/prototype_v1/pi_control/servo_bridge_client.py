#!/usr/bin/env python3
"""Raspberry Pi / laptop client for the prototype Arduino servo bridge.

Examples
--------

    # Centre all joints
    python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 centre

    # Move joint 0 to +20 degrees, slowly
    python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 joint 0 20 --sweep

    # Wiggle one DS3225 on channel 0 to verify direction / range
    python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 0

    # Send a standing-pose vector to all 18 joints
    python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 stance
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def _import_serial():
    try:
        import serial
        return serial
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required. Install on the Pi/laptop with:\n"
            "  python -m pip install pyserial"
        ) from exc


class ServoBridge:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        serial = _import_serial()
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # Arduino resets on serial-open. Give it time to boot and print help.
        time.sleep(2.0)
        self._drain()

    def _drain(self):
        while self.ser.in_waiting:
            self.ser.readline()

    def command(self, line: str, *, expect_ok: bool = True) -> str:
        self.ser.write((line.strip() + "\n").encode("ascii"))
        self.ser.flush()
        reply = self.ser.readline().decode("ascii", errors="replace").strip()
        if expect_ok and not (reply.startswith("OK") or reply.startswith("TRIM")):
            raise RuntimeError(f"Arduino rejected {line!r}: {reply!r}")
        return reply

    def centre(self):
        return self.command("C")

    def set_joint(self, joint: int, deg: float):
        return self.command(f"J {int(joint)} {float(deg):.3f}")

    def set_all(self, degrees):
        vals = " ".join(f"{float(x):.3f}" for x in degrees)
        return self.command(f"A {vals}")

    def set_trim(self, joint: int, deg: float):
        return self.command(f"T {int(joint)} {float(deg):.3f}")

    def print_trims(self):
        return self.command("P")

    def close(self):
        self.ser.close()


def standing_pose_degrees() -> list[float]:
    """Neutral tripod standing pose in the Arduino joint order.

    yaw = 0, hip = -25, knee = +60 for all 6 legs, matching
    hexapod_prototype.py / mujoco_prototype.py.
    """
    out = []
    for _leg in range(6):
        out.extend([0.0, -25.0, 60.0])
    return out


def cmd_centre(args):
    br = ServoBridge(args.port, args.baud)
    try:
        print(br.centre())
    finally:
        br.close()


def cmd_joint(args):
    br = ServoBridge(args.port, args.baud)
    try:
        if args.sweep:
            start = 0.0
            steps = max(2, int(abs(args.deg - start) / 2.0))
            for k in range(steps + 1):
                u = k / steps
                deg = start + (args.deg - start) * u
                print(br.set_joint(args.joint, deg))
                time.sleep(0.04)
        else:
            print(br.set_joint(args.joint, args.deg))
    finally:
        br.close()


def cmd_wiggle(args):
    br = ServoBridge(args.port, args.baud)
    try:
        seq = [0, +15, 0, -15, 0, +30, 0, -30, 0]
        for deg in seq:
            print(br.set_joint(args.joint, deg))
            time.sleep(args.delay)
    finally:
        br.close()


def cmd_stance(args):
    br = ServoBridge(args.port, args.baud)
    try:
        print(br.set_all(standing_pose_degrees()))
    finally:
        br.close()


def cmd_trim(args):
    br = ServoBridge(args.port, args.baud)
    try:
        print(br.set_trim(args.joint, args.deg))
        print(br.print_trims())
    finally:
        br.close()


def _default_port() -> str:
    candidates = (
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
    )
    import glob
    for pattern in candidates:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return "/dev/ttyACM0"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=_default_port(),
                    help="Serial port, e.g. /dev/ttyACM0 on Pi/Linux or /dev/cu.usbmodem... on macOS")
    ap.add_argument("--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("centre")
    p.set_defaults(func=cmd_centre)

    p = sub.add_parser("joint")
    p.add_argument("joint", type=int)
    p.add_argument("deg", type=float)
    p.add_argument("--sweep", action="store_true")
    p.set_defaults(func=cmd_joint)

    p = sub.add_parser("wiggle")
    p.add_argument("--joint", type=int, default=0)
    p.add_argument("--delay", type=float, default=0.35)
    p.set_defaults(func=cmd_wiggle)

    p = sub.add_parser("stance")
    p.set_defaults(func=cmd_stance)

    p = sub.add_parser("trim")
    p.add_argument("joint", type=int)
    p.add_argument("deg", type=float)
    p.set_defaults(func=cmd_trim)

    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
