#!/usr/bin/env python3
"""FeetechBus-compatible driver over the Uno Q MCU UART bridge.

Talks to ``firmware/feetech_bridge`` on ``/dev/ttyHS1`` (STM32 LPUART1).
The MCU owns D0/D1 → FE-URT UART (1 Mbps). Stops ``arduino-router`` while
open so this process can own the port.

Used by ``web_drive`` / ``DriveController`` and by ``urt2_motor_setup``.

    from mcu_feetech_bus import open_feetech_bus
    bus, port = open_feetech_bus()          # MCU preferred, else USB
    bus, port = open_feetech_bus("mcu")     # force MCU bridge

STREAM mode (2026-08-19, the 50-100 Hz feedback upgrade): on open, the
driver sends ``STREAM 1``. New firmware then free-runs the servo bus
itself (pos+speed sync-read + IMU every pass, full current/load/volt/
temp state at ~10 Hz) and serves ``read_all_positions`` /
``read_all_feedback`` / ``read_imu`` from RAM caches with no servo-bus
wait — each call collapses to one short host<->MCU UART round trip.
``step_all()`` goes further: SyncWrite 18 goals AND get the freshest
state snapshot back in a SINGLE round trip (the whole bus cost of a
control tick). Old firmware answers ERR and everything falls back to
the legacy synchronous paths. Opt out with ``HEXAPOD_NO_STREAM=1``.
"""
from __future__ import annotations

import glob
import json
import os
import struct
import subprocess
import threading
import time
from pathlib import Path

from feetech_bus import (  # noqa: E402
    ADDR_PRESENT_CURRENT,
    ADDR_PRESENT_LOAD,
    ADDR_PRESENT_SPEED,
    ADDR_PRESENT_TEMP,
    ADDR_PRESENT_VOLTAGE,
    ADDR_TORQUE_ENABLE,
    BAUD_DEFAULT,
    FeetechBus,
    N_JOINTS,
    count_to_deg,
    deg_to_count,
    joint_to_servo_id,
    load_trims,
    normalize_acc,
    normalize_speed,
)

MCU_PORT_DEFAULT = "/dev/ttyHS1"
# Prefer firmware HOST_BAUD (921600); fall back if an older sketch is flashed.
MCU_BAUD = 921_600
MCU_BAUD_CANDIDATES = (921_600, 115_200)
HELLO_TOKEN = "HELLO feetech_bridge"
COMM_SUCCESS = 0
COMM_FAIL = 1

# 's' snapshot reply: fixed header (seq u16, pos_age u16, imu_age u16,
# imu 7×i16) then 6 bytes per servo (id, ok, pos i16, spd i16).
SNAP_HEAD_LEN = 20
SNAP_REC_LEN = 6
SNAP_AGE_INVALID = 0xFFFF


def encode_sync_frame(cmd: int, items: list[tuple[int, int, int, int]]
                      ) -> bytes:
    """Binary ``A5 5A <cmd> n {id,pos,spd,acc}×n xor`` frame ('W' / 'S')."""
    n = min(len(items), 18)
    payload = bytearray([0xA5, 0x5A, cmd & 0xFF, n])
    x = (cmd & 0xFF) ^ n
    for sid, pos, speed, acc in items[:n]:
        chunk = struct.pack("<BhHB", sid & 0xFF, int(pos),
                            int(speed) & 0xFFFF, int(acc) & 0xFF)
        payload.extend(chunk)
        for b in chunk:
            x ^= b
    payload.append(x)
    return bytes(payload)


def parse_snapshot_payload(rn: int, payload: bytes) -> dict:
    """Decode the 's' reply payload (header + ``rn`` servo records).

    Returns raw integer fields; unit conversion happens in
    ``McuFeetechBus.step_all`` (needs trims / IMU calib).
    """
    seq, pos_age, imu_age = struct.unpack_from("<HHH", payload, 0)
    imu_raw = struct.unpack_from("<7h", payload, 6)
    servos = []
    for k in range(rn):
        sid, ok, pos, spd = struct.unpack_from(
            "<BBhh", payload, SNAP_HEAD_LEN + k * SNAP_REC_LEN)
        servos.append({"id": sid, "ok": bool(ok),
                       "pos_counts": pos, "spd_counts_s": spd})
    return {
        "seq": seq,
        "pos_age_ms": pos_age,
        "imu_age_ms": imu_age,
        "imu_raw": imu_raw,
        "servos": servos,
    }


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


def stop_arduino_router() -> bool:
    return _sudo(["systemctl", "stop", "arduino-router"])


def start_arduino_router() -> bool:
    return _sudo(["systemctl", "start", "arduino-router"])


def mcu_reset() -> bool:
    """Hard-boot the MCU: ready poke + SWD reset via remoteocd (~5 s).

    After an SoC (re)boot the STM32 does NOT auto-run its sketch — stock
    boot relies on arduino-router bringing it up ~80 s in. The reset-only
    remoteocd recipe (mcu_reset.cfg) starts the flashed sketch immediately;
    verified 2026-08-07 (the gpiochip1 line-37 poke alone is NOT enough).
    """
    _sudo(["gpioset", "-c", "/dev/gpiochip1", "-t0", "37=0"])
    ocd = sorted(glob.glob(
        "/home/arduino/.arduino15/packages/arduino/tools/remoteocd/"
        "*/remoteocd"))
    cfg = Path(__file__).resolve().parent / "mcu_reset.cfg"
    if not ocd or not cfg.is_file():
        return False
    try:
        r = subprocess.run(
            [ocd[-1], "upload", "-f", str(cfg), "/dev/null"],
            capture_output=True, text=True, timeout=30)
        return r.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def claim_mcu_port(port: str = MCU_PORT_DEFAULT) -> str:
    if not Path(port).exists():
        raise SystemExit(f"MCU bridge port {port!r} not found")
    stop_arduino_router()
    # Give the router a moment to actually release the UART — opening the
    # port too early was part of the boot-time first-HELLO failure.
    time.sleep(0.5)
    return port


def find_mcu_port() -> str | None:
    env = os.environ.get("HEXAPOD_BUS_PORT")
    if env in ("mcu", "MCU", "bridge", "ttyHS1"):
        return MCU_PORT_DEFAULT if Path(MCU_PORT_DEFAULT).exists() else None
    if env and Path(env).exists() and "ttyHS" in env:
        return env
    if Path(MCU_PORT_DEFAULT).exists():
        return MCU_PORT_DEFAULT
    hits = sorted(glob.glob("/dev/ttyHS*"))
    return hits[0] if hits else None


def find_usb_bus_port(explicit: str | None = None) -> str | None:
    if explicit and explicit not in ("mcu", "MCU", "bridge", "ttyHS1"):
        if str(explicit).startswith("/dev/ttyHS"):
            return None
        return explicit
    env = os.environ.get("HEXAPOD_BUS_PORT")
    if env and env not in ("mcu", "MCU", "bridge", "ttyHS1") \
            and not str(env).startswith("/dev/ttyHS"):
        return env
    candidates = []
    for pat in ("/dev/ttyUSB*", "/dev/ttyCH343USB*", "/dev/ttyACM*",
                "/dev/cu.usbserial*", "/dev/cu.usbmodem*"):
        candidates.extend(sorted(glob.glob(pat)))
    ranked = sorted(
        candidates,
        key=lambda p: (0 if ("CH343" in p or "USB" in p.upper()
                             or "usbserial" in p) else 1, p),
    )
    return ranked[0] if ranked else None


def probe_mcu_bridge(port: str = MCU_PORT_DEFAULT, *,
                     timeout: float = 1.2) -> bool:
    if not Path(port).exists():
        return False
    try:
        claim_mcu_port(port)
        import serial
        ser = serial.Serial(port, MCU_BAUD, timeout=timeout, write_timeout=1.0)
        try:
            ser.reset_input_buffer()
            ser.write(b"HELLO\n")
            ser.flush()
            deadline = time.monotonic() + timeout
            buf = b""
            while time.monotonic() < deadline:
                chunk = ser.read(64)
                if chunk:
                    buf += chunk
                    if HELLO_TOKEN.encode() in buf or b"HELLO" in buf:
                        return True
                else:
                    time.sleep(0.02)
            return False
        finally:
            ser.close()
    except Exception:
        return False


def open_feetech_bus(port: str | None = None, *, baud: int = BAUD_DEFAULT):
    """Open MCU bridge (preferred on Uno Q) or USB URT ``FeetechBus``.

    Returns ``(bus, port_name)``.
    """
    explicit = port or os.environ.get("HEXAPOD_BUS_PORT")
    force_mcu = explicit in ("mcu", "MCU", "bridge", "ttyHS1") or (
        explicit is not None and str(explicit).startswith("/dev/ttyHS"))
    force_usb = bool(explicit) and not force_mcu

    if not force_usb:
        mcu = explicit if (force_mcu and explicit
                           and str(explicit).startswith("/dev/")) \
            else find_mcu_port()
        if mcu and (force_mcu or probe_mcu_bridge(mcu)):
            print(f"[bus] MCU Feetech bridge on {mcu}")
            return McuFeetechBus(mcu), mcu

    usb = find_usb_bus_port(None if force_mcu else explicit)
    if usb:
        print(f"[bus] USB Feetech on {usb} @ {baud}")
        return FeetechBus(usb, baud), usb

    raise SystemExit(
        "No Feetech bus found.  Wire FE-URT UART→D0/D1 and flash "
        "firmware/feetech_bridge, or plug a USB URT-2 "
        "(or set HEXAPOD_BUS_PORT=mcu|/dev/ttyUSB0).")


class _FakePort:
    """Stand-in for scservo PortHandler (baud-swap recovery is MCU-side)."""

    def __init__(self, bus: "McuFeetechBus"):
        self._bus = bus

    def setBaudRate(self, baud: int) -> bool:
        # Host↔MCU link baud; servo-bus baud changes aren't wired yet.
        return baud in (MCU_BAUD, 115_200, BAUD_DEFAULT, 1_000_000)

    def closePort(self) -> None:
        pass

    def isOpen(self) -> bool:
        return True


class _GroupSyncWrite:
    def __init__(self, bus: "McuFeetechBus"):
        self._bus = bus

    def txPacket(self):
        self._bus._flush_sync()

    def clearParam(self):
        self._bus._pending.clear()


class _PktProxy:
    def __init__(self, bus: "McuFeetechBus"):
        self._bus = bus
        self.groupSyncWrite = _GroupSyncWrite(bus)

    def SyncWritePosEx(self, sid, pos, speed, acc) -> None:
        self._bus._pending.append(
            (int(sid), int(pos), int(speed), int(acc)))

    def WritePosEx(self, sid, pos, speed, acc=0):
        line = self._bus._transact(
            f"WP {int(sid)} {int(pos)} {int(speed)} {int(acc)}", timeout=0.6)
        ok = bool(line and line.startswith("OK"))
        return 0 if ok else COMM_FAIL

    def write1ByteTxRx(self, sid, addr, val):
        line = self._bus._transact(
            f"W1 {int(sid)} {int(addr)} {int(val)}", timeout=0.5)
        ok = bool(line and line.startswith("OK"))
        return 0, (COMM_SUCCESS if ok else COMM_FAIL), 0

    def write2ByteTxRx(self, sid, addr, val):
        line = self._bus._transact(
            f"W2 {int(sid)} {int(addr)} {int(val)}", timeout=0.5)
        ok = bool(line and line.startswith("OK"))
        return 0, (COMM_SUCCESS if ok else COMM_FAIL), 0

    def read1ByteTxRx(self, sid, addr):
        line = self._bus._transact(
            f"R1 {int(sid)} {int(addr)}", timeout=0.5)
        if not line or not line.startswith("OK"):
            return 0, COMM_FAIL, 0
        parts = line.split()
        try:
            return int(parts[1]), COMM_SUCCESS, 0
        except (IndexError, ValueError):
            return 0, COMM_FAIL, 0

    def read2ByteTxRx(self, sid, addr):
        line = self._bus._transact(
            f"R2 {int(sid)} {int(addr)}", timeout=0.5)
        if not line or not line.startswith("OK"):
            return 0, COMM_FAIL, 0
        parts = line.split()
        try:
            return int(parts[1]), COMM_SUCCESS, 0
        except (IndexError, ValueError):
            return 0, COMM_FAIL, 0

    def ReadPos(self, sid):
        pos = self._bus._read_pos_counts(int(sid))
        if pos is None:
            return 0, COMM_FAIL, 0
        return pos, COMM_SUCCESS, 0

    def ping(self, sid):
        ok = self._bus.ping(int(sid))
        return 0, (COMM_SUCCESS if ok else COMM_FAIL), 0

    def unLockEprom(self, sid):
        self._bus._transact(f"UL {int(sid)}", timeout=0.5)
        return COMM_SUCCESS

    def LockEprom(self, sid):
        self._bus._transact(f"LK {int(sid)}", timeout=0.5)
        return COMM_SUCCESS


class McuFeetechBus:
    """Drop-in stand-in for ``FeetechBus`` over the MCU bridge."""

    def __init__(self, port: str = MCU_PORT_DEFAULT, baud: int = MCU_BAUD,
                 *, claim: bool = True):
        del baud
        if claim:
            claim_mcu_port(port)
        try:
            import serial
        except ImportError as e:
            raise SystemExit(
                "pyserial required for MCU bridge") from e
        self.port_name = port
        self.port = _FakePort(self)
        self.scs = type("scs", (), {"COMM_SUCCESS": COMM_SUCCESS})()
        self.pkt = _PktProxy(self)
        self.trims = load_trims()
        self._pending: list[tuple[int, int, int, int]] = []
        self._lock = threading.Lock()
        self._live_cache: list[int] | None = None
        self._live_cache_t = 0.0
        self._fb_cache: dict[int, dict] = {}
        self._fb_cache_mono = 0.0
        self._pos_cache: dict[int, float] = {}
        self._pos_cache_mono = 0.0
        self._imu_calib: dict | None = None
        self.reload_imu_calib()
        self._imu_mount: str = "normal"
        self.reload_imu_mount()
        self._ser = None
        self.baud = MCU_BAUD
        last_hello = None
        connected = False
        # Retry rounds: right after boot the bridge can be unresponsive for
        # several seconds (MCU still booting / TFT bitbang splash stalling
        # setup()). Short 2 s probes keep a just-woken MCU from missing its
        # window by seconds; every boot used to fail its first HELLO, exit,
        # and eat a systemd restart cycle (~16 s) — retrying in-process is
        # much cheaper.
        rounds = 6
        for attempt in range(rounds):
            for baud in MCU_BAUD_CANDIDATES:
                try:
                    if self._ser is not None:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = serial.Serial(
                    port, baud, timeout=0.8, write_timeout=1.0)
                self.baud = baud
                time.sleep(0.12)
                self._ser.reset_input_buffer()
                # TFT bitbang init can briefly stall the MCU after reset.
                hello = self._transact("HELLO", timeout=2.0)
                last_hello = hello
                if hello is not None and "HELLO" in hello:
                    if baud != MCU_BAUD:
                        print(f"[bus] MCU bridge on {port} @ {baud} "
                              f"(fallback; prefer {MCU_BAUD})")
                    connected = True
                    break
            if connected:
                break
            print(f"[bus] no HELLO from {port} (attempt {attempt + 1}/"
                  f"{rounds}, got {last_hello!r}) — retrying")
            if attempt == 2:
                # Several silent attempts: the MCU is probably not running
                # at all (it does not auto-boot after an SoC reset). Hard-
                # boot it over SWD and let the sketch paint its splash.
                print("[bus] MCU silent — SWD reset via remoteocd")
                if mcu_reset():
                    print("[bus] MCU reset ok")
                time.sleep(3.0)
            time.sleep(0.5)
        if not connected:
            try:
                self._ser.close()
            except Exception:
                pass
            raise SystemExit(
                f"No feetech_bridge on {port} (got {last_hello!r}). "
                "Flash firmware/feetech_bridge and wire URT UART to D0/D1.")

        # STREAM mode: ask the firmware to free-run acquisition so reads
        # are cache-served (module docstring). ERR = old sketch → every
        # path falls back to the legacy synchronous transactions.
        self.streaming = False
        self.has_stream = False
        cmd = "STREAM" if os.environ.get("HEXAPOD_NO_STREAM") else "STREAM 1"
        line = self._transact(cmd, timeout=1.5)
        if line and line.startswith("OK STREAM"):
            self.has_stream = True
            self.streaming = line.strip().endswith("1")
            print(f"[bus] MCU stream mode "
                  f"{'ON' if self.streaming else 'off'}")

    def reload_imu_calib(self) -> dict | None:
        """Load ``logs/imu_calib.json`` (or clear if missing)."""
        try:
            from imu_calibrate import load_imu_calib
            self._imu_calib = load_imu_calib()
        except Exception:
            self._imu_calib = None
        return self._imu_calib

    def reload_imu_mount(self) -> str:
        """Load the IMU mount orientation from ``logs/imu_mount.json``.

        ``{"mount": "flip_y"}`` — how the GY-521 sits relative to the
        chassis frame (X fwd, Z up; see rl_move/attitude.py). Applied to
        RAW axes in read_imu, BEFORE bias calibration, so a rest calib
        captured after a remount stays consistent. Options:
        normal | flip_x (y,z negated) | flip_y (x,z) | flip_z (x,y).
        History: 2026-08-09 the module was briefly remounted chip-down
        (flip_y), then remounted right-side-up during the same-day
        reassembly → back to "normal". This chip reads |g| ≈ 1.27 at
        rest (scale quirk); the rest calib absorbs it as z bias.
        """
        mount = "normal"
        try:
            d = json.loads((Path(__file__).resolve().parent / "logs"
                            / "imu_mount.json").read_text())
            m = str(d.get("mount", "normal")).lower()
            if m in ("normal", "flip_x", "flip_y", "flip_z"):
                mount = m
        except Exception:
            pass
        self._imu_mount = mount
        return mount

    def _readline(self, timeout: float) -> str | None:
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._ser.read(1)
            if not chunk:
                continue
            if chunk == b"\n":
                return buf.decode("ascii", errors="replace").strip()
            if chunk != b"\r":
                buf.extend(chunk)
                if len(buf) > 512:
                    buf.clear()
        return None

    def _transact(self, cmd: str, *, timeout: float = 0.8) -> str | None:
        t0 = time.monotonic()
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write((cmd.strip() + "\n").encode("ascii"))
            self._ser.flush()
            reply = None
            for _ in range(8):
                line = self._readline(timeout)
                if line is None:
                    break
                if line.startswith("HELLO") and not cmd.startswith("HELLO"):
                    continue
                reply = line
                break
        try:
            from event_log import emit_mcu
            emit_mcu(cmd.strip(), reply, ms=(time.monotonic() - t0) * 1000.0)
        except Exception:
            pass
        return reply

    def _transact_try(self, cmd: str, *, timeout: float = 0.8) -> str | None:
        """Best-effort transaction: return immediately if the MCU link is busy."""
        if not self._lock.acquire(blocking=False):
            return None
        t0 = time.monotonic()
        try:
            self._ser.reset_input_buffer()
            self._ser.write((cmd.strip() + "\n").encode("ascii"))
            self._ser.flush()
            reply = None
            for _ in range(8):
                line = self._readline(timeout)
                if line is None:
                    break
                if line.startswith("HELLO") and not cmd.startswith("HELLO"):
                    continue
                reply = line
                break
        finally:
            self._lock.release()
        try:
            from event_log import emit_mcu
            emit_mcu(cmd.strip(), reply, ms=(time.monotonic() - t0) * 1000.0)
        except Exception:
            pass
        return reply

    def ping(self, sid: int) -> bool:
        line = self._transact(f"PING {int(sid)}", timeout=0.4)
        return bool(line and line.startswith("OK"))

    def scan(self, id_range=range(1, 31)) -> list[int]:
        now = time.monotonic()
        if self._live_cache is not None and now - self._live_cache_t < 2.0:
            return [s for s in self._live_cache if s in id_range]
        line = self._transact("SCAN", timeout=2.5)
        found: list[int] = []
        if line and line.startswith("OK"):
            rest = line[2:].strip()
            if rest:
                for part in rest.split(","):
                    part = part.strip()
                    if part.isdigit():
                        found.append(int(part))
        self._live_cache = found
        self._live_cache_t = now
        return [s for s in found if s in id_range]

    def torque(self, sid: int, on: bool) -> None:
        self._transact(f"T {int(sid)} {1 if on else 0}", timeout=0.4)

    def enable_all_torque(self, on: bool = True) -> None:
        self._transact(f"TA {1 if on else 0}", timeout=1.0)

    def set_id(self, old_id: int, new_id: int) -> None:
        self.pkt.unLockEprom(old_id)
        self.pkt.write1ByteTxRx(old_id, 5, new_id)
        self.pkt.LockEprom(new_id)
        self._live_cache = None

    def _read_exact(self, n: int, timeout: float) -> bytes | None:
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.0005)
        return bytes(buf) if len(buf) == n else None

    def _bin_txn(self, frame: bytes, want: int, rec_size: int,
                 head_len: int = 0, *, timeout: float = 1.2
                 ) -> tuple[int, bytes] | None:
        """Send a binary frame; return (n, payload) of the A5 5A reply.

        ``payload`` is ``head_len`` fixed bytes + n × ``rec_size`` records
        (checksum verified, framing stripped).
        """
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._ser.flush()
            # Skip any ASCII chatter (HELLO) until A5 arrives.
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                b = self._ser.read(1)
                if not b:
                    continue
                if b[0] == 0xA5:
                    break
                # drain a possible ASCII line
                if b[0] in (ord("H"), ord("O"), ord("E")):
                    self._readline(0.05)
            else:
                return None
            hdr = self._read_exact(3, timeout)  # 5A cmd n
            if not hdr or hdr[0] != 0x5A or hdr[1] != want:
                return None
            rn = hdr[2]
            if rn > 18:
                return None
            payload = self._read_exact(head_len + rn * rec_size, timeout)
            if payload is None:
                return None
            chk = self._read_exact(1, timeout)
            if chk is None:
                return None
            x = hdr[1] ^ rn
            for b in payload:
                x ^= b
            if chk[0] != x:
                return None
            return rn, payload

    def _bin_req(self, cmd: int, ids: list[int] | None, *,
                 timeout: float = 1.2) -> tuple[int, bytes] | None:
        """Send A5 5A cmd n [ids…] xor; return (n, payload_bytes) of reply."""
        if ids is None:
            id_bytes = b""
        else:
            id_bytes = bytes(int(i) & 0xFF for i in ids)[:18]
        n = len(id_bytes)
        body = bytes([cmd & 0xFF, n]) + id_bytes
        x = 0
        for b in body:
            x ^= b
        frame = bytes([0xA5, 0x5A]) + body + bytes([x])
        want = ord("f") if cmd == ord("F") else ord("p")
        rec_size = 13 if cmd == ord("F") else 4
        return self._bin_txn(frame, want, rec_size, timeout=timeout)

    def _fb_dict_from_rec(self, rec: bytes) -> dict | None:
        if len(rec) < 13 or rec[1] == 0:
            return None
        sid = rec[0]
        pos, spd, load = struct.unpack_from("<hhH", rec, 2)
        volt, temp, moving = rec[8], rec[9], rec[10]
        cur, = struct.unpack_from("<h", rec, 11)
        joint = int(sid) - 2
        if joint < 0 or joint >= N_JOINTS:
            return None
        # Speed: library already sign-decoded; unit is counts/s (steps/s),
        # so deg/s = counts × 360/4096. (Until 2026-08-07 this used the
        # SCS-series 0.732 rpm/unit convention — a clean 50× inflation:
        # the battery's "1537 °/s" readings were exactly the commanded
        # 350 counts/s profile speed.)
        speed_deg_s = float(spd) * 360.0 / 4096.0
        return {
            "joint": joint,
            "id": int(sid),
            "deg": count_to_deg(joint, int(pos)),
            "load_pct": (int(load) & 0x3FF) / 10.0,
            "volt": volt / 10.0,
            "temp_c": int(temp),
            "current_a": cur * 0.0065,
            "speed_deg_s": speed_deg_s,
            "moving": int(moving),
            "pos_counts": int(pos),
        }

    def read_all_feedback(self, ids: list[int] | None = None
                          ) -> dict[int, dict]:
        """One MCU round-trip: FeedBack block for every id.

        Returns ``{joint: feedback_dict}``. ``ids=None`` → MCU default 2..19.
        """
        got = self._bin_req(ord("F"), ids, timeout=1.5)
        out: dict[int, dict] = {}
        if not got:
            return out
        rn, payload = got
        for k in range(rn):
            rec = payload[k * 13:(k + 1) * 13]
            fb = self._fb_dict_from_rec(rec)
            if fb is None:
                continue
            out[fb["joint"]] = fb
            self._fb_cache[fb["id"]] = fb
            self._pos_cache[fb["joint"]] = float(fb["deg"])
        self._fb_cache_mono = time.monotonic()
        self._pos_cache_mono = self._fb_cache_mono
        return out

    def read_all_positions(self, ids: list[int] | None = None
                           ) -> dict[int, float]:
        """One MCU round-trip: present position for every id → joint deg."""
        got = self._bin_req(ord("P"), ids, timeout=1.0)
        out: dict[int, float] = {}
        if not got:
            return out
        rn, payload = got
        for k in range(rn):
            sid, ok, pos = struct.unpack_from("<BBh", payload, k * 4)
            if not ok:
                continue
            joint = int(sid) - 2
            if 0 <= joint < N_JOINTS:
                deg = count_to_deg(joint, int(pos))
                out[joint] = deg
                self._pos_cache[joint] = deg
        self._pos_cache_mono = time.monotonic()
        return out

    def _read_pos_counts(self, sid: int) -> int | None:
        # Prefer a fresh bulk position cache when possible.
        joint = int(sid) - 2
        if (0 <= joint < N_JOINTS
                and time.monotonic() - self._pos_cache_mono < 0.02
                and joint in self._pos_cache):
            # Reverse via deg_to_count is trim-dependent; fall through to RP
            # for exact counts. Cache is for deg reads.
            pass
        line = self._transact(f"RP {int(sid)}", timeout=0.5)
        if not line or not line.startswith("OK"):
            return None
        parts = line.split()
        if len(parts) < 2:
            return None
        try:
            return int(parts[1])
        except ValueError:
            return None

    def read_position_deg(self, joint: int) -> float | None:
        if (time.monotonic() - self._pos_cache_mono < 0.05
                and joint in self._pos_cache):
            return self._pos_cache[joint]
        # Bulk-refresh all live (or default 2..19) — still 1 RTT.
        bulk = self.read_all_positions(self._live_cache)
        if joint in bulk:
            return bulk[joint]
        pos = self._read_pos_counts(joint_to_servo_id(joint))
        if pos is None:
            return None
        return count_to_deg(joint, pos)

    def read_feedback(self, joint: int) -> dict | None:
        """Match ``FeetechBus.read_feedback`` (bulk path when possible)."""
        sid = joint_to_servo_id(joint)
        if (time.monotonic() - self._fb_cache_mono < 0.05
                and sid in self._fb_cache):
            return dict(self._fb_cache[sid])
        bulk = self.read_all_feedback(self._live_cache)
        if joint in bulk:
            return bulk[joint]
        # Fallback: single-id bulk.
        bulk = self.read_all_feedback([sid])
        return bulk.get(joint)

    def write_joint(self, joint: int, deg: float,
                    speed: int = 1500, acc: int = 30,
                    *, allow_max_speed: bool = False) -> None:
        speed = normalize_speed(speed, allow_max=allow_max_speed)
        acc = normalize_acc(acc)
        count = deg_to_count(joint, deg, self.trims[joint])
        self.pkt.WritePosEx(joint_to_servo_id(joint), count, speed, acc)

    def write_all(self, degrees, speed: int = 1500, acc: int = 30, *,
                  allow_max_speed: bool = False) -> None:
        speed = normalize_speed(speed, allow_max=allow_max_speed)
        acc = normalize_acc(acc)
        for joint, deg in enumerate(degrees):
            count = deg_to_count(joint, deg, self.trims[joint])
            self.pkt.SyncWritePosEx(
                joint_to_servo_id(joint), count, speed, acc)
        self.pkt.groupSyncWrite.txPacket()
        self.pkt.groupSyncWrite.clearParam()

    def step_all(self, degrees, speed: int = 1500, acc: int = 30, *,
                 allow_max_speed: bool = False, apply_calib: bool = True
                 ) -> dict | None:
        """SyncWrite all 18 goals AND get a state snapshot: ONE round trip.

        The whole bus cost of a control tick. Requires stream-capable
        firmware ('S' command); returns ``None`` when unsupported or on
        a framing error — callers fall back to ``write_all`` +
        ``read_all_positions`` / ``read_imu``.

        Returns ``{"seq", "pos_age_ms", "imu_age_ms",
        "pos_deg": {joint: deg}, "speed_deg_s": {joint: deg/s},
        "imu": read_imu-style dict | None}``. Ages are how stale the
        MCU's caches were at reply time (streaming: typically <= one
        background pass, a few ms).
        """
        if not getattr(self, "has_stream", False):
            return None
        speed = normalize_speed(speed, allow_max=allow_max_speed)
        acc = normalize_acc(acc)
        items = []
        for joint, deg in enumerate(degrees):
            items.append((joint_to_servo_id(joint),
                          deg_to_count(joint, deg, self.trims[joint]),
                          speed, acc))
        return self._snapshot_txn(items, apply_calib=apply_calib)

    def read_snapshot(self, *, apply_calib: bool = True) -> dict | None:
        """Positions + speed + IMU in ONE round trip ('S' n=0, no write).

        Same return shape as ``step_all``; ``None`` on legacy firmware.
        """
        if not getattr(self, "has_stream", False):
            return None
        return self._snapshot_txn([], apply_calib=apply_calib)

    def _snapshot_txn(self, items: list[tuple[int, int, int, int]], *,
                      apply_calib: bool) -> dict | None:
        frame = encode_sync_frame(ord("S"), items)
        got = self._bin_txn(frame, ord("s"), SNAP_REC_LEN,
                            head_len=SNAP_HEAD_LEN, timeout=0.5)
        if not got:
            return None
        rn, payload = got
        snap = parse_snapshot_payload(rn, payload)
        pos_deg: dict[int, float] = {}
        speed_deg_s: dict[int, float] = {}
        for rec in snap["servos"]:
            joint = int(rec["id"]) - 2
            if not rec["ok"] or not 0 <= joint < N_JOINTS:
                continue
            deg = count_to_deg(joint, rec["pos_counts"])
            pos_deg[joint] = deg
            # STS speed unit is counts/s → deg/s = counts × 360/4096.
            speed_deg_s[joint] = rec["spd_counts_s"] * 360.0 / 4096.0
            self._pos_cache[joint] = deg
        self._pos_cache_mono = time.monotonic()
        imu = None
        if (snap["imu_age_ms"] != SNAP_AGE_INVALID
                and any(snap["imu_raw"])):
            ax, ay, az, gx, gy, gz, temp_raw = snap["imu_raw"]
            imu = self._imu_sample(ax, ay, az, gx, gy, gz, temp_raw,
                                   apply_calib=apply_calib)
        return {
            "seq": snap["seq"],
            "pos_age_ms": snap["pos_age_ms"],
            "imu_age_ms": snap["imu_age_ms"],
            "pos_deg": pos_deg,
            "speed_deg_s": speed_deg_s,
            "imu": imu,
        }

    def _flush_sync(self) -> None:
        items = list(self._pending)
        self._pending.clear()
        if not items:
            return
        frame = encode_sync_frame(ord("W"), items)
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._ser.flush()
            line = self._readline(0.8)
        if not line or not line.startswith("OK"):
            n = min(len(items), 18)
            parts = ["SW", str(n)]
            for sid, pos, speed, acc in items[:n]:
                parts.extend([str(sid), str(int(pos)),
                              str(int(speed)), str(int(acc))])
            fallback = self._transact(" ".join(parts), timeout=1.0)
            if not fallback or not fallback.startswith("OK"):
                cause = (
                    f"binary={line!r} fallback={fallback!r}")
                self._flush_sync_slow_wp_fallback(items, cause=cause)

    def _flush_sync_slow_wp_fallback(
            self, items: list[tuple[int, int, int, int]], *,
            cause: str) -> None:
        """Last-resort slow-pose fallback for calibration/search glides."""
        max_speed = max(int(speed) for _, _, speed, _ in items)
        max_acc = max(int(acc) for _, _, _, acc in items)
        if max_speed > 250 or max_acc > 30:
            raise RuntimeError(f"SyncWrite failed: {cause}")
        failures: list[str] = []
        for sid, pos, speed, acc in items:
            reply = self._transact(
                f"WP {int(sid)} {int(pos)} {int(speed)} {int(acc)}",
                timeout=0.6)
            if not reply or not reply.startswith("OK"):
                failures.append(f"{sid}:{reply!r}")
        if failures:
            preview = ", ".join(failures[:4])
            if len(failures) > 4:
                preview += f", +{len(failures) - 4} more"
            raise RuntimeError(
                f"SyncWrite failed: {cause}; WP fallback failed {preview}")

    def power_summary(self, *, timeout: float = 2.5) -> dict:
        """One-shot bus power: live count, sum current, avg volt, max load.

        Uses MCU ``PWR`` → ``OK <n> <I_mA> <V10> <load10>``.
        """
        line = self._transact("PWR", timeout=timeout)
        if not line or not line.startswith("OK"):
            raise RuntimeError(f"PWR failed: {line!r}")
        parts = line.split()
        if len(parts) < 5:
            raise RuntimeError(f"PWR bad reply: {line!r}")
        n = int(parts[1])
        i_ma = int(parts[2])
        v10 = int(parts[3])
        load10 = int(parts[4])
        return {
            "live": n,
            "current_a": i_ma / 1000.0,
            "volt": v10 / 10.0,
            "max_load_pct": load10 / 10.0,
        }

    def read_imu(self, *, timeout: float = 0.6,
                 apply_calib: bool = True) -> dict | None:
        """Read MPU-6050 via MCU ``IMUR`` (Wire SDA/SCL).

        Returns engineering units, or ``None`` if the sensor/bridge
        does not answer. Scale assumes the sketch's ±2 g / ±250 dps
        config. When ``logs/imu_calib.json`` exists and ``apply_calib``,
        subtracts rest gyro/accel biases from Calibrate → IMU.
        """
        if getattr(self, "has_stream", False):
            try:
                snap = self.read_snapshot(apply_calib=apply_calib)
            except Exception:
                snap = None
            if isinstance(snap, dict) and isinstance(snap.get("imu"), dict):
                return snap["imu"]

        line = self._transact("IMUR", timeout=timeout)
        if not line or not line.startswith("OK"):
            return None
        parts = line.split()
        if len(parts) < 8:
            return None
        try:
            ax, ay, az = int(parts[1]), int(parts[2]), int(parts[3])
            gx, gy, gz = int(parts[4]), int(parts[5]), int(parts[6])
            temp_raw = int(parts[7])
        except ValueError:
            return None
        if not any((ax, ay, az, gx, gy, gz, temp_raw)):
            # All-zero frame = MPU asleep (power-glitch default state).
            # "IMU" wakes it (PWR_MGMT_1) + WHO_AM_I; retry once. Still
            # zeros -> report NOT ANSWERING rather than a fake flat
            # reading (atan2(0,0)=0 false-passed the tilt gate once,
            # 2026-08-09).
            self._transact("IMU", timeout=max(timeout, 1.0))
            time.sleep(0.05)
            line = self._transact("IMUR", timeout=timeout)
            parts = line.split() if line and line.startswith("OK") else []
            if len(parts) < 8:
                return None
            try:
                ax, ay, az = int(parts[1]), int(parts[2]), int(parts[3])
                gx, gy, gz = int(parts[4]), int(parts[5]), int(parts[6])
                temp_raw = int(parts[7])
            except ValueError:
                return None
            if not any((ax, ay, az, gx, gy, gz, temp_raw)):
                return None
        return self._imu_sample(ax, ay, az, gx, gy, gz, temp_raw,
                                apply_calib=apply_calib)

    def _imu_sample(self, ax: int, ay: int, az: int, gx: int, gy: int,
                    gz: int, temp_raw: int, *, apply_calib: bool = True
                    ) -> dict:
        """Raw int16 MPU sample → engineering units (mount + calib)."""
        # Mount orientation (chip frame -> chassis frame), before calib —
        # see reload_imu_mount. Accel and gyro rotate together.
        m = self._imu_mount
        if m == "flip_x":
            ay, az, gy, gz = -ay, -az, -gy, -gz
        elif m == "flip_y":
            ax, az, gx, gz = -ax, -az, -gx, -gz
        elif m == "flip_z":
            ax, ay, gx, gy = -ax, -ay, -gx, -gy
        sample = {
            "ax_g": ax / 16384.0,
            "ay_g": ay / 16384.0,
            "az_g": az / 16384.0,
            "gx_dps": gx / 131.0,
            "gy_dps": gy / 131.0,
            "gz_dps": gz / 131.0,
            "temp_c": temp_raw / 340.0 + 36.53,
            "ax_raw": ax,
            "ay_raw": ay,
            "az_raw": az,
            "gx_raw": gx,
            "gy_raw": gy,
            "gz_raw": gz,
            "temp_raw": temp_raw,
            "mount": m,
            "calibrated": False,
        }
        if apply_calib and self._imu_calib:
            try:
                from imu_calibrate import apply_imu_calib
                return apply_imu_calib(sample, self._imu_calib)
            except Exception:
                return sample
        return sample

    def display_init(self, *, timeout: float = 6.0) -> bool:
        """Hard-reinit the ST7789 (MCU ``DI`` → RST + chrome).

        Always full reinit on the MCU — needed after the ribbon is reseated
        while the sketch keeps ``ready==true`` and would otherwise paint into
        a blank panel. Budget ≥~1.6 s for bitbang clear.
        """
        line = self._transact("DI", timeout=timeout)
        return bool(line and line.startswith("OK"))

    def display_recover(self, *, attempts: int = 3,
                        timeout: float = 6.0) -> bool:
        """Startup / fault path: force DI until OK (or attempts exhausted)."""
        for i in range(max(1, int(attempts))):
            try:
                if self.display_init(timeout=timeout):
                    return True
            except Exception:
                pass
            time.sleep(0.15 * (i + 1))
        return False

    def display_push(self, lines: list[str], *, timeout: float = 8.0
                     ) -> dict | None:
        """Push status lines; MCU also lights motors from live current.

        ``DX a|b|c`` → ``OK <n> <I_mA> <V10> <load10>`` (same as ``PWR``).
        """
        clean = []
        for s in lines[:12]:
            t = "".join(ch if 32 <= ord(ch) <= 126 else " " for ch in str(s))
            clean.append(t[:20])
        payload = "|".join(clean) if clean else "idle"
        line = self._transact("DX " + payload, timeout=timeout)
        if not line or not line.startswith("OK"):
            return None
        parts = line.split()
        if len(parts) < 5:
            return {"live": 0, "current_a": 0.0, "volt": 0.0,
                    "max_load_pct": 0.0}
        return {
            "live": int(parts[1]),
            "current_a": int(parts[2]) / 1000.0,
            "volt": int(parts[3]) / 10.0,
            "max_load_pct": int(parts[4]) / 10.0,
        }

    def display_job(self, lines: list[str], *, pct: int = -1,
                    timeout: float = 8.0) -> bool:
        """Job-mode TFT (MCU ``DJ``): full-screen text + progress bar.

        Rows are positional — title, 4 body lines, footer — 26 chars each.
        ``pct`` −1 hides the bar. This does not read servos, but it still
        transacts over the shared MCU serial link; callers must skip it while
        motion or calibration timing owns the bus. The next ``display_push``
        returns the panel to the normal schematic.
        """
        clean = []
        for s in list(lines)[:6]:
            t = "".join(ch if 32 <= ord(ch) <= 126 and ch != "|" else " "
                        for ch in str(s))
            clean.append(t[:26])
        while len(clean) < 6:
            clean.append("")
        payload = f"{int(pct)}|" + "|".join(clean)
        line = self._transact("DJ " + payload, timeout=timeout)
        return bool(line and line.startswith("OK"))

    def display_job_try(self, lines: list[str], *, pct: int = -1,
                        timeout: float = 1.5) -> bool:
        """Best-effort ``display_job`` that never waits to acquire the lock."""
        clean = []
        for s in list(lines)[:6]:
            t = "".join(ch if 32 <= ord(ch) <= 126 and ch != "|" else " "
                        for ch in str(s))
            clean.append(t[:26])
        while len(clean) < 6:
            clean.append("")
        payload = f"{int(pct)}|" + "|".join(clean)
        line = self._transact_try("DJ " + payload, timeout=timeout)
        return bool(line and line.startswith("OK"))

    def close(self) -> None:
        try:
            if getattr(self, "streaming", False):
                self._transact("STREAM 0", timeout=0.5)
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass
