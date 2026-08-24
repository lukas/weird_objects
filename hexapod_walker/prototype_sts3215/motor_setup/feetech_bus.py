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
    logical joint j  ->  servo ID (j + 2)   [IDs 2..19, set once with `setid`]
    ID 1 is reserved / never assigned -- every STS3215 ships as ID 1, so
    leaving it free means a factory-fresh servo can always join a live
    daisy-chain without colliding.

Per-axis SAFE software limits (MUST match mujoco_prototype._leg_xml and
check_workspace_self_collision in _verify_prototype.py):

    yaw:        -35 .. +35 deg
    hip pitch:  -80 .. +30 deg
    knee pitch: -20 .. +150 deg

Trims: the old Arduino bridge used to persist per-joint trims in
EEPROM.  Trims now live in a JSON file next to this script
(``feetech_trims.json``) on the Uno Q and are applied in software
before the angle->step conversion.

Examples
--------

    # One-time per servo (factory ID is 1; robot IDs are 2..19):
    python feetech_bus.py --port /dev/ttyACM0 setid --from 1 --to 4

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
    2: (-20.0, 150.0),   # knee pitch (Aug 2026: operator raised from 80 —
                         # the 80° cap made the plant pose the bottom of
                         # the envelope, no crouch possible)
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

# Motion profiles (STS WritePosEx / SyncWritePosEx).
# IMPORTANT: Feetech treats speed==0 as MAX speed, not "stop".  Never pass
# 0 when you mean a gentle hold — use HOLD_SPEED / normalize_speed().
STS_SPEED_MAX = 0           # Feetech magic: unlimited / max
DEFAULT_SPEED = 1500        # walk / general move (steps/s)
DEFAULT_ACC = 30            # 0..255, *100 steps/s^2
HOLD_SPEED = 250            # re-command current pose / freeze (gentle)
HOLD_ACC = 40
WALK_SPEED = DEFAULT_SPEED
WALK_ACC = DEFAULT_ACC

BAUD_DEFAULT = 1_000_000   # STS3215 factory default


def normalize_speed(speed: int | None, *, allow_max: bool = False) -> int:
    """Coerce a Feetech goal-speed argument safely.

    ``speed is None`` → ``DEFAULT_SPEED``.
    ``speed == 0`` without ``allow_max`` → ``HOLD_SPEED`` (0 means MAX on
    STS firmware — a common footgun for "hold still" callers).
    """
    if speed is None:
        return DEFAULT_SPEED
    speed = int(speed)
    if speed == 0 and not allow_max:
        return HOLD_SPEED
    if speed < 0:
        return HOLD_SPEED
    return speed


def normalize_acc(acc: int | None) -> int:
    if acc is None:
        return DEFAULT_ACC
    return int(max(0, min(254, acc)))

TRIM_PATH = Path(__file__).resolve().parent / "feetech_trims.json"

# Learned plant (stand home) from contact calibrate.  Prefer linux_control/logs.
# Default stand when no plant_pose.json.
# Sit zero = femur straight out (0°).  A real stand puts the femur down and
# the tibia near the measured floor contact family: hip ≈ +19° and knee
# ≈ +28° under the absolute-tibia convention.  The old +80° fallback was
# from the retired serial ``hip+knee`` geometry and drives far too deep.
DEFAULT_STAND_HIP_DEG = 19.0
DEFAULT_STAND_KNEE_DEG = 28.0
PLANT_PATH_CANDIDATES = (
    Path(__file__).resolve().parent.parent / "logs" / "plant_pose.json",
    Path.home() / "hexapod_sts" / "linux_control" / "logs" / "plant_pose.json",
    Path.home() / "hexapod_sts" / "logs" / "plant_pose.json",
    Path(__file__).resolve().parent / "plant_pose.json",
)


def axis_of(joint: int) -> int:
    return joint % 3


def clampf(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def joint_limits(joint: int) -> tuple[float, float]:
    return AXIS_LIMITS_DEG[axis_of(joint)]


# Factory-default STS3215 ID.  Never assigned to a robot joint so a fresh
# servo can always be added onto a live chain without an ID collision.
FACTORY_SERVO_ID = 1
SERVO_ID_OFFSET = 2   # joint 0 -> ID 2, ..., joint 17 -> ID 19


def joint_to_servo_id(joint: int) -> int:
    """Logical joint 0..17 -> servo bus ID 2..19 (ID 1 left free)."""
    return joint + SERVO_ID_OFFSET


def plant_pose_path() -> Path:
    """Writable path for plant_pose.json (prefer linux_control/logs)."""
    for path in PLANT_PATH_CANDIDATES:
        parent = path.parent
        if parent.is_dir() or path == PLANT_PATH_CANDIDATES[0]:
            return path
    return PLANT_PATH_CANDIDATES[0]


def _parse_joints_deg(data: dict) -> list[float] | None:
    """Optional full 18-joint snapshot (authoritative plant for RL)."""
    raw = data.get("joints_deg")
    if not isinstance(raw, (list, tuple)) or len(raw) != 18:
        return None
    out: list[float] = []
    try:
        for j, v in enumerate(raw):
            lo, hi = joint_limits(j)
            out.append(clampf(float(v), lo, hi))
    except (TypeError, ValueError):
        return None
    return out


def load_plant_pose() -> dict:
    """Return ``{hip_deg, knee_deg, ...}`` or defaults if missing/invalid.

    If ``joints_deg`` (len 18) is present it is the authoritative plant;
    hip/knee fields remain as summary / legacy for calibrate UI.
    """
    for path in PLANT_PATH_CANDIDATES:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
            hip = float(data["hip_deg"])
            knee = float(data["knee_deg"])
        except (OSError, ValueError, KeyError, TypeError):
            continue
        hip_lo, hip_hi = AXIS_LIMITS_DEG[1]
        knee_lo, knee_hi = AXIS_LIMITS_DEG[2]
        joints = _parse_joints_deg(data)
        return {
            "hip_deg": clampf(hip, hip_lo, hip_hi),
            "knee_deg": clampf(knee, knee_lo, knee_hi),
            "joints_deg": joints,
            "path": str(path),
            "learned": True,
            "contact_found": bool(data.get("contact_found", True)),
            "timestamp": data.get("timestamp"),
            "source": data.get("source") or "plant_calibrate",
        }
    return {
        "hip_deg": DEFAULT_STAND_HIP_DEG,
        "knee_deg": DEFAULT_STAND_KNEE_DEG,
        "joints_deg": None,
        "path": None,
        "learned": False,
        "contact_found": False,
        "timestamp": None,
        "source": "default",
    }


def save_plant_pose(hip_deg: float, knee_deg: float, *,
                    extra: dict | None = None) -> Path:
    """Persist stand-plant hip/knee; next ``standing_pose_degrees()`` uses it."""
    hip_lo, hip_hi = AXIS_LIMITS_DEG[1]
    knee_lo, knee_hi = AXIS_LIMITS_DEG[2]
    path = plant_pose_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "hip_deg": round(clampf(float(hip_deg), hip_lo, hip_hi), 3),
        "knee_deg": round(clampf(float(knee_deg), knee_lo, knee_hi), 3),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": "plant_calibrate",
    }
    if extra:
        payload.update(extra)
    path.write_text(json.dumps(payload, indent=2))
    return path


def clear_plant_pose() -> bool:
    """Remove learned plant file(s); stand home falls back to +19°/+28°."""
    removed = False
    for path in PLANT_PATH_CANDIDATES:
        if path.is_file():
            try:
                path.unlink()
                removed = True
            except OSError:
                pass
    return removed


def standing_pose_degrees() -> list[float]:
    """Stand / plant home.

    Prefer a captured 18-joint ``joints_deg`` snapshot. Otherwise expand
    shared hip/knee (learned or default +19/+28). RL Phase 1 should not
    use the default — capture a real stance first.
    """
    plant = load_plant_pose()
    joints = plant.get("joints_deg")
    if joints is not None and len(joints) == 18:
        return [float(x) for x in joints]
    hip = float(plant["hip_deg"])
    knee = float(plant["knee_deg"])
    out: list[float] = []
    for _leg in range(6):
        out.extend([0.0, hip, knee])
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
            "ftservo-python-sdk is required.  Install on the Uno Q "
            "(or your laptop) with:\n"
            "  uv pip install ftservo-python-sdk\n"
            "(FEETECH's official SDK; provides the `scservo_sdk` module "
            "with the `sms_sts` STS-series class used here.  NOT the "
            "unofficial `feetech-servo-sdk` package, which lacks it)."
        ) from exc


# STS/SMS control-table addresses used for the generic feedback reads that
# the high-level `sts` helper doesn't wrap directly.
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMP = 63
ADDR_PRESENT_CURRENT = 69


def _signed_speed_counts(raw: int) -> int:
    """STS present-speed: bit 15 = direction, lower 15 = magnitude."""
    mag = raw & 0x7FFF
    return -mag if (raw & 0x8000) else mag


def speed_counts_to_deg_s(counts_per_s: float) -> float:
    """STS3215 speed registers are counts/s (steps/s), 4096 counts/rev.

    deg/s = counts x 360/4096. The SCS-series 0.732 rpm/unit convention
    does NOT apply to STS — using it inflates speeds by exactly 50x
    (the 2026-08-07 "1537 deg/s" battery readings were the commanded
    350 counts/s profile speed). mcu_feetech_bus was fixed then; this
    USB path carried the bug until 2026-08-19.
    """
    return float(counts_per_s) * 360.0 / 4096.0


class FeetechBus:
    def __init__(self, port: str, baud: int = BAUD_DEFAULT):
        scs = _import_sdk()
        self.scs = scs
        self.port = scs.PortHandler(port)
        if not self.port.openPort():
            raise SystemExit(f"Failed to open bus port {port!r}")
        if not self.port.setBaudRate(baud):
            raise SystemExit(f"Failed to set baud {baud} on {port!r}")
        # STS-series high-level handler.  The official ftservo-python-sdk
        # names it `sms_sts` (SMS + STS share the protocol).
        self.pkt = scs.sms_sts(self.port)
        self.trims = load_trims()

    # -- low level ---------------------------------------------------------
    def ping(self, sid: int) -> bool:
        try:
            _model, result, _err = self.pkt.ping(sid)
            return result == self.scs.COMM_SUCCESS
        except Exception:
            # Port glitches during baud sweeps / unplug must not kill the tool.
            return False

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
                    speed: int = DEFAULT_SPEED, acc: int = DEFAULT_ACC,
                    *, allow_max_speed: bool = False) -> None:
        speed = normalize_speed(speed, allow_max=allow_max_speed)
        acc = normalize_acc(acc)
        count = deg_to_count(joint, deg, self.trims[joint])
        self.pkt.WritePosEx(joint_to_servo_id(joint), count, speed, acc)

    def write_all(self, degrees, speed: int = DEFAULT_SPEED,
                  acc: int = DEFAULT_ACC, *,
                  allow_max_speed: bool = False) -> None:
        """Group sync-write all joints so the whole robot moves in one packet.

        ``speed=0`` is coerced to ``HOLD_SPEED`` unless ``allow_max_speed``
        (Feetech's 0 = max-speed footgun).
        """
        speed = normalize_speed(speed, allow_max=allow_max_speed)
        acc = normalize_acc(acc)
        for joint, deg in enumerate(degrees):
            count = deg_to_count(joint, deg, self.trims[joint])
            self.pkt.SyncWritePosEx(joint_to_servo_id(joint), count, speed, acc)
        self.pkt.groupSyncWrite.txPacket()
        self.pkt.groupSyncWrite.clearParam()

    def centre(self, **kw) -> None:
        self.write_all([0.0] * N_JOINTS, **kw)

    def enable_all_torque(self, on: bool = True) -> None:
        for j in range(N_JOINTS):
            self.torque(joint_to_servo_id(j), on)

    def hold_current_pose(self, *,
                          speed: int = HOLD_SPEED,
                          acc: int = HOLD_ACC) -> list[float]:
        """Re-command each joint's *present* angle (gentle). Torque ON.

        Use this to freeze motion without slamming (never speed=0).
        """
        pose: list[float] = []
        for j in range(N_JOINTS):
            deg = self.read_position_deg(j)
            if deg is None:
                deg = 0.0
            pose.append(deg)
        self.enable_all_torque(True)
        self.write_all(pose, speed=speed, acc=acc)
        return pose

    def safe_stop(self, *, limp: bool = False) -> list[float]:
        """Freeze at current pose; optionally limp after (demo / E-stop).

        Walking should usually keep ``limp=False`` so the robot still
        supports itself.  Air demos / bench use ``limp=True`` to avoid
        hold-hunt buzz in the gears.
        """
        pose = self.hold_current_pose()
        if limp:
            self.enable_all_torque(False)
        return pose

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
        spd_raw, r_sp, _e = self.pkt.read2ByteTxRx(sid, ADDR_PRESENT_SPEED)
        # STS load: bit 10 = direction, lower 10 bits = magnitude (0.1%).
        load_pct = (load & 0x3FF) / 10.0
        # Present speed unit is counts/s → deg/s = counts × 360/4096.
        if r_sp == self.scs.COMM_SUCCESS:
            speed_deg_s = speed_counts_to_deg_s(_signed_speed_counts(spd_raw))
        else:
            speed_deg_s = 0.0
        return {
            "joint": joint,
            "id": sid,
            "deg": count_to_deg(joint, pos),
            "load_pct": load_pct,
            "volt": volt / 10.0,        # 0.1 V units
            "temp_c": temp,             # deg C
            "current_a": cur * 0.0065,  # ~6.5 mA/LSB
            "speed_deg_s": speed_deg_s,
        }

    def read_imu(self) -> dict | None:
        """MPU-6050 is MCU-Wire only; USB URT path has no IMU."""
        return None

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
        expected = [joint_to_servo_id(j) for j in range(N_JOINTS)]
        if found != expected:
            print(f"  expected IDs {expected[0]}..{expected[-1]} for a fully-built "
                  f"robot (ID {FACTORY_SERVO_ID} = un-ID'd factory-fresh servo).")
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
        bus.enable_all_torque(True)
        bus.centre(speed=DEFAULT_SPEED, acc=DEFAULT_ACC)
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
        bus.enable_all_torque(True)
        bus.write_all(standing_pose_degrees(),
                      speed=DEFAULT_SPEED, acc=DEFAULT_ACC)
        print("OK stance (torque on — robot supports itself)")
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
    """Torque ON + re-command present pose gently (no slam / max-speed)."""
    bus = FeetechBus(args.port, args.baud)
    try:
        if args.joint is not None:
            deg = bus.read_position_deg(args.joint)
            if deg is None:
                raise SystemExit(f"joint {args.joint} no reply")
            bus.torque(joint_to_servo_id(args.joint), True)
            bus.write_joint(args.joint, deg, speed=HOLD_SPEED, acc=HOLD_ACC)
            print(f"OK hold joint {args.joint} at {deg:+.1f} deg "
                  f"(speed={HOLD_SPEED})")
        else:
            pose = bus.hold_current_pose()
            print(f"OK hold all at present pose "
                  f"(speed={HOLD_SPEED}, n={len(pose)})")
    finally:
        bus.close()


def cmd_stop(args):
    """Freeze walk/motion at current pose (keeps supporting torque)."""
    bus = FeetechBus(args.port, args.baud)
    try:
        bus.safe_stop(limp=args.limp)
        print("OK stop" + (" + limp" if args.limp else " (holding gently)"))
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

    p = sub.add_parser("hold",
                       help="torque ON + re-command present pose (gentle)")
    p.add_argument("--joint", type=int, default=None)
    p.set_defaults(func=cmd_hold)

    p = sub.add_parser("stop",
                       help="freeze at current pose (walk E-stop); "
                            "keeps torque unless --limp")
    p.add_argument("--limp", action="store_true",
                   help="also torque OFF after freeze (bench/demos)")
    p.set_defaults(func=cmd_stop)

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
