#!/usr/bin/env python3
"""Probe the MPU-6050 through feetech_bridge on the Uno Q MCU.

Header SDA/SCL (D20/D21) are STM32 Wire pins — Linux ``i2cdetect`` will
never see 0x68 there. Talk to the sketch instead:

    uv run python mpu_probe.py            # on the Uno Q

Protocol (ASCII on /dev/ttyHS1 @ 921600, stop arduino-router first):
    HELLO → HELLO feetech_bridge
    I2CSCAN → OK 0x68,...
    IMU → OK 0x68
    IMUR → OK ax ay az gx gy gz temp_raw
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

MCU_PORT_DEFAULT = "/dev/ttyHS1"
MCU_BAUD = 921_600  # match feetech_bridge HOST_BAUD


def _sudo(cmd: list[str]) -> bool:
    try:
        r = subprocess.run(
            ["sudo", "-n"] + cmd,
            capture_output=True, text=True, timeout=8,
        )
        if r.returncode == 0:
            return True
    except (OSError, subprocess.TimeoutExpired):
        pass
    pw = os.environ.get("HEXAPOD_SUDO_PASSWORD", "arduino")
    try:
        r = subprocess.run(
            ["sudo", "-S"] + cmd,
            input=pw + "\n",
            capture_output=True, text=True, timeout=8,
        )
        return r.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def claim_mcu_port(port: str) -> str:
    if not Path(port).exists():
        raise SystemExit(f"MCU bridge port {port!r} not found")
    _sudo(["systemctl", "stop", "arduino-router"])
    time.sleep(0.15)
    return port


def _readline(ser, timeout: float = 1.5) -> str:
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            if b"\n" in buf:
                line, _, _rest = buf.partition(b"\n")
                return line.decode("ascii", errors="replace").strip()
        else:
            time.sleep(0.01)
    return buf.decode("ascii", errors="replace").strip()


def _drain(ser, timeout: float = 0.8) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = ser.read(128)
        if not chunk:
            time.sleep(0.02)


def cmd(ser, line: str, timeout: float = 2.0) -> str:
    ser.reset_input_buffer()
    ser.write((line.strip() + "\n").encode("ascii"))
    ser.flush()
    return _readline(ser, timeout=timeout)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=MCU_PORT_DEFAULT)
    ap.add_argument("--watch", type=float, default=0.0,
                    help="re-read IMUR every N seconds (0 = once)")
    ap.add_argument("--restore-router", action="store_true",
                    help="restart arduino-router on exit")
    args = ap.parse_args()

    import serial

    port = claim_mcu_port(args.port)
    ser = serial.Serial(port, MCU_BAUD, timeout=0.05, write_timeout=1.0)
    try:
        _drain(ser)
        hello = cmd(ser, "HELLO")
        print(f"HELLO → {hello}")
        if "feetech_bridge" not in hello:
            print("ERR: expected feetech_bridge (flash firmware/feetech_bridge)",
                  file=sys.stderr)
            return 2

        scan = cmd(ser, "I2CSCAN", timeout=3.0)
        print(f"I2CSCAN → {scan}")

        imu = cmd(ser, "IMU", timeout=2.0)
        print(f"IMU → {imu}")
        if not imu.startswith("OK"):
            print("MPU not answering on Wire (SDA/SCL). Check 3V3/GND/"
                  "SDA→SDA/SCL→SCL.", file=sys.stderr)
            return 1

        def once() -> str:
            return cmd(ser, "IMUR", timeout=1.5)

        line = once()
        print(f"IMUR → {line}")
        if not line.startswith("OK"):
            return 1

        parts = line.split()
        if len(parts) >= 8:
            ax, ay, az = int(parts[1]), int(parts[2]), int(parts[3])
            gx, gy, gz = int(parts[4]), int(parts[5]), int(parts[6])
            temp_raw = int(parts[7])
            temp_c = temp_raw / 340.0 + 36.53
            print(f"  accel LSB: ({ax}, {ay}, {az})  "
                  f"~g: ({ax/16384:.2f}, {ay/16384:.2f}, {az/16384:.2f})")
            print(f"  gyro  LSB: ({gx}, {gy}, {gz})  "
                  f"~dps: ({gx/131:.1f}, {gy/131:.1f}, {gz/131:.1f})")
            print(f"  temp: {temp_c:.1f} °C")

        if args.watch > 0:
            print(f"--watch {args.watch}s (Ctrl-C to stop)")
            while True:
                time.sleep(args.watch)
                print(once())
        return 0
    finally:
        ser.close()
        if args.restore_router:
            _sudo(["systemctl", "start", "arduino-router"])


if __name__ == "__main__":
    raise SystemExit(main())
