#!/usr/bin/env python3
"""Demos for the installed STS3215 hexapod.

Most motions are offsets around **logical 0°** (legs straight out after
motor_setup action ``z``).  The ``rise`` demo glides to a deep reach
(stand is high; walking stance is not enough), logs peak current, holds,
then descends to zero on a keypress.

Control style
-------------
STS ``WritePosEx`` restarts an internal trapezoid on every packet.
Streaming a fixed speed/acc at 25 Hz made idle joints (and backlashy
yaws) buzz.  Demos now use ``PoseStreamer``: deadband skip + goal-speed
matched to |Δθ|/dt, plus a soft SRAM torque limit while waving.

On the MCU stream bridge (2026-08-19) ``configure_stream_profile``
densifies waypoints to 100 Hz with a proportionally smaller deadband:
dense + speed-matched, the trapezoid restarts blend into near-continuous
velocity tracking instead of visible stair-steps.  Legacy buses keep the
sparse 12.5 Hz profile.

Examples
--------
    python inplace_demos.py              # interactive menu
    python inplace_demos.py --demo shimmy
    python inplace_demos.py --demo rise
    python inplace_demos.py --demo rise+   # higher + faster reach
    python inplace_demos.py --demo rise_turn   # fast rise + small twist
    python inplace_demos.py --demo rise_show   # tether-safe planted show
    python inplace_demos.py --demo shimmy --log          # CSV + summary
    python inplace_demos.py --log-hold 6                 # hold + log only
    python inplace_demos.py --zero       # slowly drive to 0°
    python inplace_demos.py --list

From motor_setup: ``f`` = demos, ``g`` = go to zero pose.
"""
from __future__ import annotations

import argparse
import bisect
import math
import os
import random
import statistics
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_LINUX_CONTROL = _HERE.parent
for _p in (_HERE, _LINUX_CONTROL):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from feetech_bus import (  # noqa: E402
    ADDR_TORQUE_ENABLE, BAUD_DEFAULT, COUNTS_PER_DEG, FeetechBus, JOINT_SIGN,
    N_JOINTS, WALK_ACC, WALK_SPEED, count_to_deg, deg_to_count,
    joint_to_servo_id, normalize_acc, normalize_speed, standing_pose_degrees,
)
from motion_telemetry import (  # noqa: E402
    MotionLog, default_log_path, joint_name, run_hold_log,
)
from tripod_gait import FEMUR_MM, TIBIA_MM  # noqa: E402
from urt2_bench import default_port, keystroke_abort_watch, limp_now  # noqa: E402

# SRAM torque limit (0..1000). Softened for demos so PID can't hammer
# through backlash as hard. Volatile — restored to 1000 after limp/finish.
ADDR_TORQUE_LIMIT = 48
DEMO_TORQUE_LIMIT = 450

# Streaming control: do NOT blast fixed speed/acc every tick — that
# restarts each servo's trapezoid and makes yaws buzz.  Instead match
# goal-speed to |Δθ|/dt and skip joints inside a deadband.
ZERO_SPEED = 120
# Stand must push body weight — soft demo limits leave feet short of plant.
STAND_TORQUE_LIMIT = 1000
STAND_OK_DEG = 8.0
STAND_VERIFY_HOLD_S = 0.4
ZERO_ACC = 8
# Waypoint density is picked per-bus by ``configure_stream_profile``:
#
# LEGACY (USB bus / pre-stream firmware): a SyncWrite + telemetry tick
# costs >20 ms, so waypoints stay sparse (12.5 Hz) and the deadband big —
# each WritePosEx restarts the servo trapezoid, and sparse restarts with
# mismatched speeds are what made joints buzz through backlash.
#
# DENSE (MCU stream bridge, 2026-08-19): a SyncWrite round trip is
# ~1.5-3 ms, so 100 Hz waypoints fit in a 10 ms tick. Dense + goal-speed
# matched to |Δθ|/dt means the servo never finishes a step before the
# next one arrives — the restarts blend into near-continuous velocity
# tracking instead of visible stair-steps. Deadband scales down with the
# tick so per-tick deltas still pass it near peak velocity while
# turnaround jitter is still skipped; below ~0.15° (≈2 encoder counts)
# steps stop being expressible, so that is the floor — at 100 Hz a
# typical air-demo peak step is only ~0.3° (~3 counts).
LEGACY_DT = 0.08
LEGACY_SHOW_DT = 0.055
LEGACY_DEADBAND_DEG = 0.8  # skip tiny re-commands (was 0.4 — buzzed)
DENSE_DT = 0.01            # 100 Hz waypoints (was 0.02 in first cut)
DENSE_SHOW_DT = 0.01
DENSE_DEADBAND_DEG = 0.15
# Goal lead for dense streaming (see PoseStreamer.write): the commanded
# goal runs this far ahead of the trajectory so the STS position PID
# always has a healthy error to chase (~ the error legacy 12.5 Hz steps
# gave it implicitly). ~10 ticks at 100 Hz.
DENSE_LEAD_S = 0.10
# Speed-cap headroom over trajectory velocity. The speed register is a
# CAP, not a setpoint: capped exactly at trajectory speed a servo that
# falls behind can never catch back up to the receding goal.
DENSE_SPEED_MARGIN = 1.3
DT = LEGACY_DT
DEADBAND_DEG = LEGACY_DEADBAND_DEG
STREAM_DENSE = False  # set by configure_stream_profile()
MAX_STREAM_SPEED = 450
MIN_STREAM_SPEED = 40
# Rise: chassis sits on an elevated stand, so the old walking crouch never
# reaches the floor.  Match the measured stand family: femur angled down
# around +19° and absolute tibia angle around +28° to +36°.
#
# Presets (foot drop ≈ -FEMUR*sin(hip) - TIBIA*sin(knee), mm):
#   default / stand  hip +19° / knee +28° / 12 s  -> ~100 mm
#   high+fast        hip +20° / knee +36° / 5 s   -> ~119 mm
RISE_SECONDS = 12.0
DESCEND_SECONDS = 12.0
RISE_HIP_DEG = 19.0
RISE_KNEE_DEG = 28.0
RISE_HIGH_HIP_DEG = 20.0
RISE_HIGH_KNEE_DEG = 36.0
RISE_FAST_SECONDS = 5.0
RISE_FAST_DESCEND_SECONDS = 6.0
RISE_TORQUE_LIMIT = 700
# Stop the reach early when a hip/knee current spike says a foot planted
# (or snagged).  Yaws are ignored for contact.
RISE_CONTACT_CURRENT_A = 0.55
# Web UI: stay at the planted/reach pose — the old 1.5s hold felt like a bail.
RISE_WEB_HOLD_S = 45.0
PLANTED_WEB_HOLD_S = 8.0
# Planted demos (cords plugged into the robot): keep peak |yaw| small and
# always return to yaw 0 so USB/power don't wind.  No walking / spinning.
RISE_TURN_YAW_DEG = 10.0
RISE_TURN_SECONDS = 0.45
RISE_TURN_PAUSE_S = 0.08
RISE_SHOW_LOOK_YAW_DEG = 12.0
RISE_SHOW_BOUNCE_KNEE_DELTA = 14.0
RISE_SHOW_NOD_HIP_DELTA = 12.0
RISE_SHOW_SNAP_S = 0.28          # fire-and-wait between big poses
RISE_SHOW_OVERHEAD_S = 0.85
RISE_SHOW_PLANT_S = 2.0
RISE_SHOW_DESCEND_S = 2.6
RISE_SHOW_DT = LEGACY_SHOW_DT    # planted multi-leg stream tick (see profile)
# Lifted leg angles while others stay planted.
RISE_SHOW_LIFT_HIP_DEG = 2.0
RISE_SHOW_LIFT_KNEE_DEG = 28.0
RISE_SHOW_WAVE_YAW_DEG = 14.0
SETTLE_DEG = 2.5  # looser = snappier show glides
# Sit: from legs-out 0°, fold “arms” way up (past soft breathe).
ARMS_UP_HIP_DEG = -62.0
ARMS_UP_KNEE_DEG = 22.0
ARMS_UP_SECONDS = 2.2
ARMS_UP_HOLD_S = 1.4
# Stand: three legs raised toward overhead; other three stay on stand plant.
STAND_HANDS_LIFT_HIP_DEG = 8.0
STAND_HANDS_LIFT_KNEE_DEG = 18.0
STAND_HANDS_SECONDS = 1.8
STAND_HANDS_HOLD_S = 1.6
STAND_HANDS_LEGS = (0, 2, 4)  # every other leg = stable tripod support
# Dance: full choreography (heartbeat → breathe → hands up → RISE → wild
# planted acts → sleep).  Durations below are at speed 1.0×.
# Opening tightened per operator note (was ~40 s of quiet before the
# rise; now ~25 s, with big air choreography instead of small wiggles).
DANCE_HEARTBEAT_S = 2.2          # 2 lub-dub pulses @ 1.1 s cycle
DANCE_BREATH_GROWTH = (0.8, 1.6)        # per-breath amplitude (× size)
DANCE_BREATH_HALF_S = 1.5        # inhale / exhale glide each
DANCE_AIR_WAVE_S = 3.0           # counter-rotating spiral wave
DANCE_AIR_CHAOS_S = 4.5          # all 18 joints independent (lissajous)
DANCE_AIR_CANON_S = 3.0          # follow-the-leader, air-sized
DANCE_AIR_CONVERGE_S = 7.0       # wild arms -> staggered locks -> meet
DANCE_OVERHEAD_SWAY_S = 1.8      # arms-overhead shimmer before the rise
DANCE_OVERHEAD_HOLD_S = 0.6      # beat of stillness before the drop
DANCE_DESCEND_S = 8.0            # planted → sit zero
DANCE_OUTRO_HEARTBEAT_S = 2.2    # two final soft thumps
DANCE_HANDS_HOLD_S = 0.9         # standing hands-up feature hold
# Act V show tempo (snap/stream seconds at 1.0×) — tightened per
# operator note: "faster especially after it stands".
DANCE_SNAP_S = 0.18
DANCE_STOMP_S = 0.14
# Act V holding cap. 700 (the rise's contact-detection cap) let the
# STS3215s sag 15-20 deg under body weight — the 08-18 video showed a
# low wide crouch vs the sim's crisp ~144 mm stance (commanded poses
# were identical; execution sagged). 950 stiffens the hold; current
# guards (stream 3 A / hard 4 A) still protect against snags.
DANCE_PLANT_TORQUE = 950
# Victory-lap PRANCE (open-loop tripod, horse mode): quick cadence +
# high knees — far more aggressive than the RL walk's trained band.
# ACC 80 is deliberate (not WALK_ACC=30): sim sweep 08-18 showed the
# 0.29 s half-swings are ACCELERATION-limited — at ACC 20 the servos
# never reach cruise and the prance smears into a crawl (0.012 m/s);
# ACC 80 realizes ~0.038 m/s upright at full height.
DANCE_PRANCE_PERIOD = 0.58   # s/cycle (gentle walk demo uses 0.85)
DANCE_PRANCE_LIFT_MM = 32.0  # high knees (gentle walk demo uses 18)
DANCE_PRANCE_VX = 0.09       # m/s out (RL band tops out at 0.06)
DANCE_PRANCE_ACC = 80        # Feetech ACC units (×100 counts/s²)
# Lap shape (operator 08-18): prance OUT further, ABOUT-FACE (true half
# turn), prance HOME — no pirouette-for-no-reason. Out and home share
# the same gait + duration, so the return distance matches the out leg
# by symmetry no matter how much the feet slip.
DANCE_PRANCE_FWD_S = 7.5     # sim: ~285 mm out (hardware slip-dependent)
DANCE_PRANCE_TURN_OMEGA = 0.85  # rad/s commanded (gait clamps at 1.0)
DANCE_PRANCE_HALFTURN_S = 9.3   # sim-measured 08-18: 19.3 deg/s realized
                                # at ACC 80 regime → 180° in 9.3 s

RISE_PRESETS = {
    "default": {
        "hip": RISE_HIP_DEG,
        "knee": RISE_KNEE_DEG,
        "rise_seconds": RISE_SECONDS,
        "descend_seconds": DESCEND_SECONDS,
        "drop_mm": 100,
        "label": "stand plant",
    },
    "high_fast": {
        "hip": RISE_HIGH_HIP_DEG,
        "knee": RISE_HIGH_KNEE_DEG,
        "rise_seconds": RISE_FAST_SECONDS,
        "descend_seconds": RISE_FAST_DESCEND_SECONDS,
        "drop_mm": 119,
        "label": "higher + faster reach",
    },
}


def _zero_pose() -> list[float]:
    """Sit / air home — legs straight out (logical 0°)."""
    return [0.0] * N_JOINTS


def _stand_zero_pose() -> list[float]:
    """Stand home — learned plant or measured fallback +19°/+28°."""
    return standing_pose_degrees()


def _arms_up_pose() -> list[float]:
    """Sit pose with all six legs folded way overhead."""
    pose = _zero_pose()
    for leg in range(6):
        _set_leg(pose, leg, yaw=0.0,
                 hip=ARMS_UP_HIP_DEG, knee=ARMS_UP_KNEE_DEG)
    return pose


def _stand_hands_pose(legs: tuple[int, ...] = STAND_HANDS_LEGS) -> list[float]:
    """Stand plant with selected legs raised overhead (others planted)."""
    pose = _stand_zero_pose()
    for leg in legs:
        _set_leg(pose, int(leg), yaw=0.0,
                 hip=STAND_HANDS_LIFT_HIP_DEG,
                 knee=STAND_HANDS_LIFT_KNEE_DEG)
    return pose


def _elevated_stand_pose(*, hip: float = RISE_HIP_DEG,
                         knee: float = RISE_KNEE_DEG,
                         yaw: float = 0.0) -> list[float]:
    """Deep reach so feet can find the floor from the elevated stand.

    Default stand plant is hip +19° / knee +28° (~100 mm). The high
    preset reaches about 119 mm.
    Contact current can stop early.
    ``yaw`` is applied the same on every leg (in-place body twist).
    """
    out: list[float] = []
    for _leg in range(6):
        out.extend([float(yaw), float(hip), float(knee)])
    return out


def _set_leg(pose: list[float], leg: int, *,
             yaw: float | None = None,
             hip: float | None = None,
             knee: float | None = None) -> None:
    base = leg * 3
    if yaw is not None:
        pose[base + 0] = float(yaw)
    if hip is not None:
        pose[base + 1] = float(hip)
    if knee is not None:
        pose[base + 2] = float(knee)


def _planted_leg_up(hip: float, knee: float, leg: int, *,
                    lift_hip: float = RISE_SHOW_LIFT_HIP_DEG,
                    lift_knee: float = RISE_SHOW_LIFT_KNEE_DEG,
                    wave_yaw: float = 0.0) -> list[float]:
    """Five legs planted; one leg raised (salute / wave). Cord-safe."""
    pose = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
    _set_leg(pose, leg, yaw=wave_yaw, hip=lift_hip, knee=lift_knee)
    return pose


def _planted_legs_up(hip: float, knee: float, legs: list[int], *,
                     lift_hip: float = RISE_SHOW_LIFT_HIP_DEG,
                     lift_knee: float = RISE_SHOW_LIFT_KNEE_DEG) -> list[float]:
    """Raise several legs at once (others stay planted)."""
    pose = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
    for leg in legs:
        _set_leg(pose, leg, yaw=0.0, hip=lift_hip, knee=lift_knee)
    return pose


def _live_robot_ids(bus: FeetechBus) -> set[int]:
    return {sid for sid in bus.scan(range(2, 20))}


class CurrentPeakTracker:
    """Track per-joint and global peak |current| during a motion phase."""

    def __init__(self):
        self.max_a: dict[int, float] = {}
        self.peak_a = 0.0
        self.peak_joint: int | None = None
        self.samples = 0
        self._t0 = time.monotonic()
        self.peak_t_s = 0.0
        # Full feedback dicts from the most recent sweep — callers can
        # log "what the servos are saying", not just the current peak.
        self.last_fb: list[dict] = []

    def sample(self, bus: FeetechBus, live: set[int]) -> None:
        self.samples += 1
        t = time.monotonic() - self._t0
        sweep: list[dict] = []
        for joint in range(N_JOINTS):
            sid = joint_to_servo_id(joint)
            if sid not in live:
                continue
            fb = bus.read_feedback(joint)
            if fb is None:
                continue
            sweep.append(fb)
            a = abs(float(fb["current_a"]))
            prev = self.max_a.get(joint, 0.0)
            if a > prev:
                self.max_a[joint] = a
            if a > self.peak_a:
                self.peak_a = a
                self.peak_joint = joint
                self.peak_t_s = t
        self.last_fb = sweep

    def print_report(self, *, phase: str) -> None:
        print(f"  Max current during {phase}:")
        if self.peak_joint is None:
            print("    (no samples)")
            return
        print(f"    peak {self.peak_a:.2f} A  @ {joint_name(self.peak_joint)}"
              f"  t={self.peak_t_s:.1f}s  ({self.samples} samples)")
        ranked = sorted(self.max_a.items(), key=lambda kv: -kv[1])
        for joint, a in ranked[:6]:
            print(f"      {joint_name(joint):<10}  {a:.2f} A")
        if len(ranked) > 6:
            print(f"      … +{len(ranked) - 6} more")

    def write_log(self, path: Path, *, phase: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as fh:
            fh.write(f"# rise current peaks  phase={phase}\n")
            fh.write("# joint,name,max_current_a\n")
            for joint, a in sorted(self.max_a.items(), key=lambda kv: -kv[1]):
                fh.write(f"{joint},{joint_name(joint)},{a:.4f}\n")
            if self.peak_joint is not None:
                fh.write(
                    f"# peak,{joint_name(self.peak_joint)},"
                    f"{self.peak_a:.4f},t_s={self.peak_t_s:.2f}\n")
        print(f"  Current peaks → {path}")


MOTION_CMD_LOG_HZ = 2.0


def _motion_cmd_period_s() -> float:
    """Low-rate command telemetry period; 0 disables via environment."""
    try:
        hz = float(os.environ.get("HEXAPOD_MOTION_CMD_HZ",
                                  MOTION_CMD_LOG_HZ))
    except (TypeError, ValueError):
        hz = MOTION_CMD_LOG_HZ
    if hz <= 0.0:
        return 0.0
    return 1.0 / max(0.1, min(10.0, hz))


def _compact_imu(imu: dict | None) -> dict | None:
    if not imu:
        return None
    out: dict = {}
    for key in ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps",
                "gz_dps", "temp_c", "body_pitch_deg",
                "body_roll_deg", "body_pitch_target_deg",
                "body_frame_calibrated"):
        if key not in imu:
            continue
        val = imu.get(key)
        if isinstance(val, (int, float)):
            out[key] = round(float(val), 4)
        else:
            out[key] = val
    return out or None


def _emit_motion_cmd(label: str, q: list[float],
                     wrote: dict[int, tuple[int, int]], *,
                     t_s: float, seconds: float, wall_s: float,
                     rate: float, tick_s: float, lookahead_s: float,
                     tracker: CurrentPeakTracker,
                     balance_trim: QuadPitchTrim | None = None,
                     imu: dict | None = None,
                     servo_goals: dict[int, float] | None = None,
                     reason: str = "periodic") -> None:
    """Always-on, nonblocking command telemetry.

    This records the targets the host gave the servos without performing
    any extra servo feedback reads. The intrusive cmd-vs-encoder CSV stays
    opt-in; this path only serializes data already in memory.
    """
    try:
        from event_log import emit
    except Exception:
        return
    try:
        writes = [
            {
                "j": int(j),
                "id": joint_to_servo_id(int(j)),
                "deg": round(float(q[int(j)]), 2),
                "servo_goal_deg": round(float(
                    (servo_goals or {}).get(int(j), q[int(j)])), 2),
                "speed": int(sa[0]),
                "acc": int(sa[1]),
            }
            for j, sa in sorted((wrote or {}).items())
        ]
        data = {
            "label": str(label),
            "reason": str(reason),
            "t_s": round(float(t_s), 3),
            "duration_s": round(float(seconds), 3),
            "wall_s": round(float(wall_s), 3),
            "rate": round(float(rate), 3),
            "tick_s": round(float(tick_s), 4),
            "lookahead_s": round(float(lookahead_s), 4),
            "cmd_deg": [round(float(x), 2) for x in q],
            "wrote_n": len(writes),
            "wrote": writes,
            "peak_a": round(float(tracker.peak_a), 3),
            "peak_joint": tracker.peak_joint,
        }
        if balance_trim is not None:
            data["balance"] = balance_trim.event_data()
        imu_compact = _compact_imu(imu)
        if imu_compact is not None:
            data["imu"] = imu_compact
        emit("motion_cmd", f"{label} command", src="motion", data=data)
    except Exception:
        pass


def _enable_torque(bus: FeetechBus, live: set[int]) -> None:
    for sid in sorted(live):
        bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)


def _set_torque_limit(bus: FeetechBus, live: set[int], limit: int) -> None:
    for sid in sorted(live):
        try:
            bus.pkt.write2ByteTxRx(sid, ADDR_TORQUE_LIMIT, int(limit))
        except Exception:
            pass


def configure_stream_profile(bus: FeetechBus) -> bool:
    """Pick waypoint density from what the bus can sustain.

    ``bus.streaming`` is True only on the MCU stream-bridge firmware
    (2026-08-19), where a SyncWrite round trip costs ~1.5-3 ms — dense
    50 Hz speed-matched waypoints then track like velocity control.
    Legacy buses keep the sparse 12.5 Hz profile. Returns True if dense.
    """
    global DT, RISE_SHOW_DT, DEADBAND_DEG, STREAM_DENSE
    STREAM_DENSE = bool(getattr(bus, "streaming", False))
    DT = DENSE_DT if STREAM_DENSE else LEGACY_DT
    RISE_SHOW_DT = DENSE_SHOW_DT if STREAM_DENSE else LEGACY_SHOW_DT
    DEADBAND_DEG = (DENSE_DEADBAND_DEG if STREAM_DENSE
                    else LEGACY_DEADBAND_DEG)
    return STREAM_DENSE


def _speed_for_delta(delta_deg: float, dt: float, *,
                     min_speed: int | None = None,
                     max_speed: int | None = None,
                     max_acc: int = 80) -> tuple[int, int]:
    """Feetech speed/acc so a Δθ roughly completes in one tick."""
    counts = abs(delta_deg) * COUNTS_PER_DEG
    lo = MIN_STREAM_SPEED if min_speed is None else int(min_speed)
    hi = MAX_STREAM_SPEED if max_speed is None else int(max_speed)
    # Keep host-commanded speed below what finishes the step early —
    # overshooting + re-command is what makes the joints buzz.
    speed = int(min(hi, max(lo, counts / max(dt, 1e-3) * 0.9)))
    if STREAM_DENSE and dt <= 0.021:
        # Dense ticks pair with the lead goal (PoseStreamer.write):
        # speed cap ABOVE trajectory velocity (catch-up headroom; the
        # lead goal bounds overshoot), a near-zero floor so reversals
        # taper instead of jerking, and acc=0 (no ramp; a ramped
        # restart every 10 ms never leaves the slowest accel phase).
        # NOTE: a lower floor (12) + speed EMA were tried 2026-08-19 to
        # soften turnarounds — both made tracking AND shake worse (EMA
        # lag fights the carrot's catch-up dynamics). Keep the plain cap.
        speed = int(min(hi, max(lo, counts / max(dt, 1e-3)
                                * DENSE_SPEED_MARGIN)))
        return normalize_speed(speed), 0
    # Soft accel: take most of the tick to ramp (acc reg = ×100 step/s²).
    acc = int(min(max_acc, max(4, speed / (100.0 * max(dt, 1e-3) * 0.85))))
    return normalize_speed(speed), normalize_acc(acc)


def _glide_speed_acc(start: list[float], goal: list[float], live: set[int],
                     seconds: float) -> tuple[int, int]:
    """One-shot speed/acc so the largest live joint Δ finishes in ``seconds``."""
    max_delta = 0.0
    for joint, (a, b) in enumerate(zip(start, goal)):
        if joint_to_servo_id(joint) not in live:
            continue
        max_delta = max(max_delta, abs(b - a))
    seconds = max(float(seconds), 0.35)
    # Show moves are short — allow aggressive profiles so glides feel snappy.
    if seconds < 1.2:
        speed_cap, acc_cap = 1000, 140
    elif seconds < 3.0:
        speed_cap, acc_cap = 750, 100
    elif seconds < 7.0:
        speed_cap, acc_cap = 550, 70
    else:
        speed_cap, acc_cap = 350, 40
    speed = int(min(speed_cap, max(40, max_delta * COUNTS_PER_DEG / seconds)))
    # Soft ramp (~20–35% of the move); short moves get a snappier ramp.
    acc = int(min(acc_cap, max(8, speed / (100.0 * seconds * 0.28))))
    return normalize_speed(speed), normalize_acc(acc)


def _write_pose(bus: FeetechBus, degrees: list[float], live: set[int],
                *, speed: int = 250, acc: int = 20,
                speeds: list[int] | None = None) -> None:
    """One-shot sync-write. ``speeds[j]`` overrides per-joint when given."""
    speed = normalize_speed(speed)
    acc = normalize_acc(acc)
    for joint, deg in enumerate(degrees):
        sid = joint_to_servo_id(joint)
        if sid not in live:
            continue
        count = deg_to_count(joint, deg, bus.trims[joint])
        sp = speed
        if speeds is not None and joint < len(speeds):
            sp = normalize_speed(int(speeds[joint]))
        bus.pkt.SyncWritePosEx(sid, count, sp, acc)
    if live:
        bus.pkt.groupSyncWrite.txPacket()
        bus.pkt.groupSyncWrite.clearParam()


class PoseStreamer:
    """Stream a changing pose without restarting idle joints every tick.

    Each WritePosEx/SyncWritePosEx restarts the servo's internal
    trapezoid.  Re-commanding a joint that is already at target (hips/
    knees during a yaw shimmy) is what makes STS3215s buzz through
    gear backlash.  We only write joints that moved past a deadband,
    with speed/acc sized to the step.
    """

    def __init__(self):
        self.last: list[float] | None = None
        self.prev: list[float] | None = None
        self.last_written_goal: dict[int, float] = {}

    def reset(self) -> None:
        self.last = None
        self.prev = None
        self.last_written_goal = {}

    def write(self, bus: FeetechBus, degrees: list[float], live: set[int],
              *, dt: float = DT,
              deadband: float | None = None,
              min_speed: int | None = None,
              max_speed: int | None = None,
              max_acc: int = 80) -> dict[int, tuple[int, int]]:
        """Sync-write changed joints.

        Returns ``{joint: (speed, acc)}`` for joints that were commanded
        this tick (empty if nothing moved past the deadband).
        """
        db = DEADBAND_DEG if deadband is None else float(deadband)
        if self.last is None:
            # First frame: ease toward the pose gently (no slam).
            self.last = list(degrees)
            self.prev = list(degrees)
            _write_pose(bus, degrees, live, speed=120, acc=10)
            self.last_written_goal = {
                j: float(degrees[j]) for j in range(N_JOINTS)
                if joint_to_servo_id(j) in live}
            return {j: (120, 10) for j in range(N_JOINTS)
                    if joint_to_servo_id(j) in live}

        # Dense mode: trajectory velocity for the lead goal (see below).
        prev = getattr(self, "prev", None) or self.last
        vel = [(d - p) / max(dt, 1e-3) for d, p in zip(degrees, prev)]
        self.prev = list(degrees)

        wrote: dict[int, tuple[int, int]] = {}
        written_goal: dict[int, float] = {}
        for joint, deg in enumerate(degrees):
            sid = joint_to_servo_id(joint)
            if sid not in live:
                continue
            delta = deg - self.last[joint]
            if abs(delta) < db:
                continue
            speed, acc = _speed_for_delta(
                delta, dt, min_speed=min_speed, max_speed=max_speed,
                max_acc=max_acc)
            goal = deg
            if STREAM_DENSE and dt <= 0.021:
                # Lead the goal along the trajectory ("carrot"). With
                # 100 Hz waypoints the raw per-tick goal sits only 2-3
                # encoder counts ahead; the STS position PID barely
                # drives on an error that small and the servo creeps
                # (measured 2026-08-19: yaw amplitude collapsed 5.0° →
                # 3.7° std regardless of acc). Sparse 12.5 Hz ticks
                # accidentally provided an ~80 ms lead; do it on purpose
                # here — goal recedes ahead of the servo at matched
                # speed, so it never decelerates into the tiny-error
                # zone, and reversals still update within one tick.
                goal = deg + vel[joint] * DENSE_LEAD_S
            count = deg_to_count(joint, goal, bus.trims[joint])
            bus.pkt.SyncWritePosEx(sid, count, speed, acc)
            self.last[joint] = deg
            wrote[joint] = (speed, acc)
            written_goal[joint] = float(goal)
        if wrote:
            bus.pkt.groupSyncWrite.txPacket()
            bus.pkt.groupSyncWrite.clearParam()
        self.last_written_goal = written_goal
        return wrote


def _read_pose(bus: FeetechBus, live: set[int]) -> list[float]:
    """Present joint angles (deg); missing IDs → 0."""
    # MCU stream bridge: one cached bulk transaction beats 18 round trips.
    read_all = getattr(bus, "read_all_positions", None)
    if callable(read_all):
        try:
            bulk = read_all()
        except Exception:
            bulk = None
        if bulk:
            pose = [0.0] * N_JOINTS
            got = 0
            for joint in range(N_JOINTS):
                deg = bulk.get(joint)
                if deg is not None and joint_to_servo_id(joint) in live:
                    pose[joint] = float(deg)
                    got += 1
            if got:
                return pose
    pose = [0.0] * N_JOINTS
    for joint in range(N_JOINTS):
        sid = joint_to_servo_id(joint)
        if sid not in live:
            continue
        pos, result, _err = bus.pkt.ReadPos(sid)
        if result == bus.scs.COMM_SUCCESS:
            pose[joint] = count_to_deg(joint, pos)
    return pose


def _hold_here(bus: FeetechBus, live: set[int]) -> list[float]:
    """Re-command current pose so motion stops (torque stays on).

    Note: Feetech speed=0 means MAX speed — never use 0 for a hold.
    """
    pose = _read_pose(bus, live)
    _write_pose(bus, pose, live, speed=180, acc=25)
    return pose


def _limp_all(bus: FeetechBus, live: set[int] | None = None) -> None:
    ids = sorted(live if live is not None else _live_robot_ids(bus))
    for sid in ids:
        limp_now(bus, sid)
    print(f"  Torque OFF on {len(ids)} motor(s) — limp "
          f"(stops hold-hunt / shaking at a target).")


def ease_to_pose(bus: FeetechBus, goal: list[float], *,
                 abort_check=None,
                 seconds: float = 3.0,
                 label: str = "pose",
                 current_tracker: CurrentPeakTracker | None = None,
                 contact_current_a: float | None = None,
                 log: MotionLog | None = None) -> bool:
    """Slow glide to ``goal`` with ONE SyncWrite (servo-side trapezoid).

    Host-streamed intermediate poses restart each servo's profile every
    tick and feel like shake/buzz.  Here we size speed/acc for ``seconds``,
    command once, then poll — optionally logging current and stopping
    early on a hip/knee current spike (foot contact / snag).

    Returns True if completed (or stopped on contact), False if aborted.
    """
    live = _live_robot_ids(bus)
    if not live:
        print("  No robot servos on the bus.")
        return False
    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    start = _read_pose(bus, live)
    speed, acc = _glide_speed_acc(start, goal, live, seconds)
    print(f"  Gliding to {label} over ~{seconds:.1f}s "
          f"(speed={speed}, acc={acc}; one command, no stream) ...")
    print("  Any key aborts.")
    if contact_current_a is not None:
        print(f"  Contact stop if hip/knee |I| ≥ {contact_current_a:.2f} A.")
    if check():
        _hold_here(bus, live)
        print(f"    {label} aborted — holding here.")
        return False

    _write_pose(bus, goal, live, speed=speed, acc=acc)
    timeout = seconds * 1.6 + 2.0
    # Don't declare "arrived" in the first half of the profile — loose
    # SETTLE_DEG + slow MCU feedback made long glides stop midway.
    min_before_settle = max(0.35, min(float(seconds) * 0.55, float(seconds) - 0.25))
    t0 = time.monotonic()
    settle_needed = 3  # consecutive polls within SETTLE_DEG
    settled = 0
    tick = 0
    while time.monotonic() - t0 < timeout:
        if check():
            _hold_here(bus, live)
            print(f"    {label} aborted — holding here.")
            return False
        time.sleep(0.1)
        tick += 1
        # Full per-joint feedback is very heavy on the MCU bridge; sample
        # sparsely so the SyncWrite trapezoid isn't starved of the bus.
        if current_tracker is not None and tick % 5 == 0:
            current_tracker.sample(bus, live)
        if log is not None and tick % 4 == 0:
            log.sample(bus, goal, wrote={})
        if contact_current_a is not None and tick % 3 == 0:
            for joint in range(N_JOINTS):
                if joint % 3 == 0:  # yaw — ignore for plant detect
                    continue
                if joint_to_servo_id(joint) not in live:
                    continue
                fb = bus.read_feedback(joint)
                if fb is None:
                    continue
                amps = abs(float(fb["current_a"]))
                if amps >= contact_current_a:
                    _hold_here(bus, live)
                    print(f"    Contact/snag — {joint_name(joint)} "
                          f"at {amps:.2f} A; holding here.")
                    return True

        present = _read_pose(bus, live)
        worst = 0.0
        for joint, (g, p) in enumerate(zip(goal, present)):
            if joint_to_servo_id(joint) not in live:
                continue
            worst = max(worst, abs(g - p))
        elapsed = time.monotonic() - t0
        if worst <= SETTLE_DEG and elapsed >= min_before_settle:
            settled += 1
            if settled >= settle_needed:
                print(f"  At {label} (settled, err ≤ {SETTLE_DEG}°).")
                return True
        else:
            settled = 0

    # Time-box: re-issue a gentle final hold at wherever we are / goal.
    _write_pose(bus, goal, live, speed=ZERO_SPEED, acc=ZERO_ACC)
    if current_tracker is not None:
        current_tracker.sample(bus, live)
    print(f"  At {label} (timeout — check for binding).")
    return True


def go_to_zero_pose(bus: FeetechBus, *,
                    abort_check=None,
                    seconds: float = 3.0,
                    current_tracker: CurrentPeakTracker | None = None) -> bool:
    """Sit home: slowly drive all live joints to logical 0° (legs out)."""
    return ease_to_pose(
        bus, _zero_pose(), abort_check=abort_check, seconds=seconds,
        label="sit zero (0° / legs out)",
        current_tracker=current_tracker)


def assess_stand_pose(bus: FeetechBus, goal: list[float] | None = None,
                      *, tol_deg: float = STAND_OK_DEG) -> dict:
    """Compare live hip/knee angles to stand goal — True only if tracking.

    Used after stand zero so a soft-torque / stalled reach can't report
    success with feet still in the air.
    """
    live = _live_robot_ids(bus)
    goal = list(goal) if goal is not None else _stand_zero_pose()
    worst = 0.0
    worst_j: int | None = None
    present_hip: list[float] = []
    present_knee: list[float] = []
    n = 0
    for j in range(N_JOINTS):
        if j % 3 == 0:
            continue
        if joint_to_servo_id(j) not in live:
            continue
        d = bus.read_position_deg(j)
        if d is None:
            continue
        err = abs(float(goal[j]) - float(d))
        n += 1
        if j % 3 == 1:
            present_hip.append(float(d))
        else:
            present_knee.append(float(d))
        if err >= worst:
            worst = err
            worst_j = j
    ok = n >= 6 and worst <= float(tol_deg)
    hip_med = (statistics.median(present_hip) if present_hip else None)
    knee_med = (statistics.median(present_knee) if present_knee else None)
    return {
        "ok": ok,
        "max_err_deg": round(worst, 2),
        "worst_joint": worst_j,
        "worst_name": (joint_name(worst_j) if worst_j is not None else None),
        "tol_deg": float(tol_deg),
        "live_joints": n,
        "goal_hip": float(goal[1]) if len(goal) > 1 else None,
        "goal_knee": float(goal[2]) if len(goal) > 2 else None,
        "present_hip": (round(hip_med, 2) if hip_med is not None else None),
        "present_knee": (round(knee_med, 2) if knee_med is not None else None),
        "error": (
            None if ok else
            f"not standing — max {worst:.1f}° off on "
            f"{joint_name(worst_j) if worst_j is not None else '?'} "
            f"(need ≤{tol_deg:.0f}°; goal hip "
            f"{goal[1]:.0f}°/knee {goal[2]:.0f}°"
            + (f", now {hip_med:.0f}°/{knee_med:.0f}°"
               if hip_med is not None and knee_med is not None else "")
            + ")"
        ),
    }


def go_to_stand_pose(bus: FeetechBus, *,
                     abort_check=None,
                     seconds: float = 3.0,
                     current_tracker: CurrentPeakTracker | None = None,
                     result: dict | None = None) -> bool:
    """Stand home with full torque; verify encoders match plant afterward.

    ``result`` (optional) is filled with ``assess_stand_pose`` fields.
    """
    live = _live_robot_ids(bus)
    pose = _stand_zero_pose()
    hip = pose[1] if len(pose) > 1 else 20.0
    knee = pose[2] if len(pose) > 2 else 80.0
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, STAND_TORQUE_LIMIT)
    ok_glide = ease_to_pose(
        bus, pose, abort_check=abort_check,
        seconds=max(float(seconds), 4.5),
        label=f"stand zero (hip {hip:.0f}° / knee {knee:.0f}° · τ{STAND_TORQUE_LIMIT})",
        current_tracker=current_tracker)
    if not ok_glide:
        if result is not None:
            result.update({"ok": False, "error": "stand aborted",
                           "aborted": True})
        return False
    time.sleep(STAND_VERIFY_HOLD_S)
    # Firm re-hold at goal before measuring (kills mid-profile droop).
    _write_pose(bus, pose, live, speed=200, acc=25)
    time.sleep(0.25)
    st = assess_stand_pose(bus, pose)
    if result is not None:
        result.update(st)
    if not st["ok"]:
        print(f"  STAND FAILED — {st['error']}")
        return False
    print(f"  Stand verified (max err {st['max_err_deg']:.1f}°).")
    return True


def _ask_return_to_zero(bus: FeetechBus, live: set[int]) -> None:
    """After an abort: ask whether to ease back to 0°."""
    # Must leave cbreak before input(); caller restores terminal via
    # keystroke_abort_watch finally — so only call this OUTSIDE the watch,
    # or flush and use a plain prompt after the with-block.
    try:
        ans = input("  Return slowly to zero pose? [Y/n]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        ans = "n"
    if ans in ("", "y", "yes"):
        with keystroke_abort_watch() as check:
            go_to_zero_pose(bus, abort_check=check)
        _limp_all(bus, live)
    else:
        # Holding under load (legs out) makes STS3215s buzz/hunt through
        # gear backlash — limp is the quiet default.
        _limp_all(bus, live)


def _run_frames(bus: FeetechBus, live: set[int], frames, abort_check,
                *, label: str, log: MotionLog | None = None,
                dt: float | None = None,
                deadband: float | None = None,
                min_speed: int | None = None,
                max_speed: int | None = None,
                max_acc: int = 80) -> bool:
    """Play frames with velocity-matched streaming. False if aborted."""
    tick = DT if dt is None else float(dt)
    # Telemetry reads are heavy; at dense ticks sample every Nth frame so
    # logging keeps its old ~12.5 Hz cadence instead of eating the tick.
    log_every = max(1, round(LEGACY_DT / tick)) if log is not None else 0
    print(f"  {label}  — any key aborts immediately")
    if log is not None:
        print("  (querying motors during playback → CSV; a bit slower)")
    stream = PoseStreamer()
    t0 = time.monotonic()
    for i, pose in enumerate(frames):
        if abort_check():
            _hold_here(bus, live)
            print("    STOP — keystroke.  Holding pose.")
            return False
        wrote = stream.write(
            bus, pose, live, dt=tick, deadband=deadband,
            min_speed=min_speed, max_speed=max_speed, max_acc=max_acc)
        if log is not None and i % log_every == 0:
            log.sample(bus, pose, wrote=wrote)
        # Absolute schedule (not per-frame remainder) so dense ticks don't
        # accumulate drift when a write/telemetry read runs long.
        sleep_for = (t0 + (i + 1) * tick) - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
    return True


def _yaw_hip_knee(leg: int, pose: list[float], *,
                  yaw: float = 0.0, hip: float = 0.0,
                  knee: float = 0.0) -> None:
    base = leg * 3
    pose[base + 0] += yaw
    pose[base + 1] += hip
    pose[base + 2] += knee


# ---------------------------------------------------------------------------
# Live-speed streamed demos — the stand-up lab technique (2026-08-17).
#
# The experiments page's stand/sit feel comes from HOST-OWNED timing:
# sample a time-parameterized pose q(t) at ~20 Hz and stream it through
# PoseStreamer with speed/acc sized to the ACTUAL elapsed tick, plus a
# carrot lookahead so the servos never run out of goal and park between
# writes (the ~18 Hz stutter the stand-up lab already fixed).  This is
# that pursuit loop generalized for cyclic demos, with a LIVE speed
# multiplier read every tick (web slider while the demo runs) and the
# same stall-fight current guard as the stand-up lab.
# ---------------------------------------------------------------------------
STREAM_TICK_S = 0.05          # ~20 Hz host tick
STREAM_LOOKAHEAD_S = 0.12     # carrot: command ~2 ticks ahead of schedule
# Quad gaits stream at 10 Hz with max accel: every WritePosEx restarts
# the servo's internal trapezoid from ZERO velocity, so at 20 Hz the
# ramp (acc register x100 counts/s^2) only ever reaches ~500 counts/s —
# the 08-18 trot pranced in place (front-swing knees demanded ~900).
# Halving the write rate doubles realized swing speed and foot apex
# (restart-accurate sim: 6.8 -> 14 mm/s, apex 18 -> 33 mm).
QUAD_STREAM_TICK_S = 0.10
QUAD_STREAM_ACC = 254
# Reared gaits command ~20 deg of lean and rock another ~13 in normal
# running; past 45 deg the support diagonal is lost and the robot IS
# falling — go limp (soft landing) instead of riding it down rigid.
QUAD_TILT_GUARD_DEG = 45.0
STREAM_GUARD_A = 3.0          # stall-fight: joint over this while not moving
STREAM_HARD_CAP_A = 4.0       # instantaneous hard cap, trips regardless
STAND_DANCE_TORQUE = 900      # weight-bearing motion (end-hold restores 1000)
LIVE_SPEED_MIN = 0.25
LIVE_SPEED_MAX = 3.0

# Live quad balance trim: this is intentionally a small reflex around the
# existing gait, not a new gait planner. It uses the same sparse IMU read
# already used by the tilt guard, so normal operation adds no bus traffic.
QUAD_TRIM_MAX_PITCH_DEG = 5.0
QUAD_TRIM_MAX_DX_M = 0.012
QUAD_TRIM_DEADBAND_DEG = 1.0
QUAD_TRIM_GUARD_ERROR_DEG = 15.0
QUAD_TRIM_MIN_BASE_DEG = 6.0
# Recovery band: between the small-trim reflex and the fall guard there
# is a "tipped too far but not yet falling" regime.  Instead of marching
# on with saturated 5-deg trim until the 15-deg guard limps, recovery
# BRACE-HOLDS: stream_pose_fn finishes the current step at no more than
# 1x, FREEZES the gait clock at the next all-stance window (the pose fn
# exposes its stance schedule via ``all_stance_at``), and leans the
# body against the tip with the full second-line trim authority
# (quad_walk._trim clamps at +/-7 deg / +/-18 mm — recovery uses
# exactly that).  The pacing rules were measured on the MuJoCo twin
# (08-21): freezing mid-swing strands a foot on tripod support and
# DEEPENS the tip; slowing a swing does the same (the nose hangs
# unsupported for longer); and stepping on at full beat keeps rocking
# the nose ~10 deg per swing on top of the tip.  Hence: swings run at
# normal pace, stance freezes.  Resume normal walking once level again;
# if the lean has not come back within the timeout, the tip is real —
# go limp, never fight it.  Thresholds sized against the twin's walk,
# which runs ~6 deg sagged from the reared-hold calibration target with
# beat spikes past 12: enter above the steady sag at the spikes' onset;
# exit must also sit just above the steady sag or recovery could never
# release and would always time out into a needless limp.
QUAD_TRIM_RECOVER_ERR_DEG = 9.0        # enter recovery (2 consecutive)
QUAD_TRIM_RECOVER_EXIT_DEG = 5.0       # leave recovery (2 consecutive)
QUAD_TRIM_RECOVER_MAX_PITCH_DEG = 7.0  # = quad_walk 2nd-line pitch clamp
QUAD_TRIM_RECOVER_MAX_DX_M = 0.018     # = quad_walk 2nd-line dx clamp
QUAD_TRIM_RECOVER_TIMEOUT_S = 8.0      # still tipped after this -> limp


def _imu_roll_pitch_deg(imu: dict) -> tuple[float, float] | None:
    try:
        ax = float(imu.get("ax_g", 0.0))
        ay = float(imu.get("ay_g", 0.0))
        az = float(imu.get("az_g", 0.0))
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 0.25:
        return None
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return roll, pitch


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _deadband(x: float, band: float) -> float:
    if x > band:
        return x - band
    if x < -band:
        return x + band
    return 0.0


def _slew(current: float, desired: float, max_step: float) -> float:
    if desired > current + max_step:
        return current + max_step
    if desired < current - max_step:
        return current - max_step
    return desired


def _quad_trim_axis_label(axis_roll: float, axis_pitch: float) -> str:
    if abs(axis_roll) < 0.25:
        return "pitch"
    if abs(axis_pitch) < 0.25:
        return "roll"
    return "mix"


def _quad_trim_sign_to_cmd(expected_pitch_deg: float | None,
                           measured_base_deg: float) -> float:
    if expected_pitch_deg is not None and abs(expected_pitch_deg) > 5.0:
        # quad_walk command convention is negative nose-up. If the measured
        # rear-lean baseline reports the opposite sign, flip it into command
        # convention.
        return -1.0 if expected_pitch_deg * measured_base_deg < 0 else 1.0
    return -1.0


def quad_trim_calibration_from_roll_pitch(
        roll_deg: float, pitch_deg: float, *,
        expected_pitch_deg: float | None = None,
        gait: str = "quad",
        samples: int = 1,
        source: str = "quad_rear_lean") -> dict:
    """Build the saved quad-trim calibration payload from a rear lean."""
    base_roll = float(roll_deg)
    base_pitch = float(pitch_deg)
    measured_base = math.hypot(base_roll, base_pitch)
    if measured_base < QUAD_TRIM_MIN_BASE_DEG:
        return {
            "ok": False,
            "source": source,
            "gait": gait,
            "error": (
                "imu rear-lean baseline too small "
                f"(roll {base_roll:+.1f}, pitch {base_pitch:+.1f} deg)"
            ),
            "roll_deg": round(base_roll, 3),
            "pitch_deg": round(base_pitch, 3),
            "measured_lean_deg": round(measured_base, 3),
            "samples": int(samples),
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
    axis_roll = base_roll / measured_base
    axis_pitch = base_pitch / measured_base
    sign_to_cmd = _quad_trim_sign_to_cmd(
        expected_pitch_deg, measured_base)
    return {
        "ok": True,
        "version": 1,
        "source": source,
        "gait": gait,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "samples": int(samples),
        "roll_deg": round(base_roll, 3),
        "pitch_deg": round(base_pitch, 3),
        "measured_lean_deg": round(measured_base, 3),
        "expected_pitch_deg": (
            None if expected_pitch_deg is None
            else round(float(expected_pitch_deg), 3)),
        "imu_axis": _quad_trim_axis_label(axis_roll, axis_pitch),
        "imu_axis_roll": round(axis_roll, 6),
        "imu_axis_pitch": round(axis_pitch, 6),
        "imu_sign_to_cmd": sign_to_cmd,
        "target_pitch_deg": round(sign_to_cmd * measured_base, 3),
    }


def quad_trim_calibration_from_imu_samples(
        samples: list[dict], *,
        expected_pitch_deg: float | None = None,
        gait: str = "quad",
        source: str = "quad_rear_lean") -> dict:
    vals: list[tuple[float, float]] = []
    for sample in samples:
        rp = _imu_roll_pitch_deg(sample)
        if rp is not None:
            vals.append(rp)
    if not vals:
        return {
            "ok": False,
            "source": source,
            "gait": gait,
            "error": "no valid IMU samples",
            "samples": 0,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
    roll = sum(r for r, _p in vals) / len(vals)
    pitch = sum(p for _r, p in vals) / len(vals)
    return quad_trim_calibration_from_roll_pitch(
        roll, pitch, expected_pitch_deg=expected_pitch_deg, gait=gait,
        samples=len(vals), source=source)


class QuadPitchTrim:
    """Tiny IMU pitch reflex for split quad hold/walk/trot phases.

    The trim baseline is measured at the start of the reared phase. We
    treat the baseline (roll, pitch) pair as a 2D lean vector, so an IMU
    mounted off-axis can split rear lean across both computed axes. New
    samples are projected onto that vector, then converted into
    quad_walk's command convention (negative = nose up). If pitch drifts
    forward, the command nudges more nose-up/aft; if it drifts backward,
    the command nudges less nose-up/forward.

    Three regimes by pitch error:
      trim     |err| small: gentle nudges, capped at +/-5 deg / 12 mm.
      recover  |err| past QUAD_TRIM_RECOVER_ERR_DEG sustained:
               brace-hold — the streamer finishes the current step at
               no more than 1x, freezes the gait clock at the next
               all-stance window, and pushes the full second-line trim
               authority against the tip until level again — or limp
               on QUAD_TRIM_RECOVER_TIMEOUT_S.
      guard    |err| past QUAD_TRIM_GUARD_ERROR_DEG (or fast + big) for
               two IMU samples: fall onset — the streamer goes limp
               rather than fighting it.
    """

    def __init__(self, *, expected_pitch_deg: float | None = None,
                 gait: str = "quad",
                 calibration: dict | None = None):
        self.expected_pitch_deg = expected_pitch_deg
        self.gait = gait
        self.enabled = True
        self.ready = False
        self.disabled_reason = ""
        self.loaded_calibration = False
        self.calibration_source = "warmup"
        self.body_frame_mode = False
        self.samples = 0
        self.imu_axis = "pitch"
        self.imu_axis_roll = 0.0
        self.imu_axis_pitch = 1.0
        self.sign_to_cmd = -1.0
        self.target_pitch_deg: float | None = None
        self.pitch_deg: float | None = None
        self.pitch_error_deg = 0.0
        self.pitch_rate_deg_s = 0.0
        self.pitch_trim_deg = 0.0
        self.body_dx_trim_m = 0.0
        self.speed_scale = 1.0
        self.abort_reason: str | None = None
        self.recovering = False
        self.recover_count = 0
        self.recover_t0: float | None = None
        self._recover_hi = 0
        self._recover_lo = 0
        self._warmup: list[tuple[float, float]] = []
        self._axis_lp_meas: float | None = None
        self._prev_cmd_pitch: float | None = None
        self._prev_update_t: float | None = None
        self._danger_samples = 0
        self._last_emit_t = 0.0
        self._last_emit_bucket: tuple | None = None
        if calibration:
            self._load_calibration(calibration)

    def _rear_up_cmd_sign(self) -> float:
        """Sign of a command trim that asks the quad pose for more rear-up."""
        try:
            if self.expected_pitch_deg is not None:
                return -1.0 if float(self.expected_pitch_deg) < 0.0 else 1.0
        except (TypeError, ValueError):
            pass
        return -1.0

    def _axis_label(self) -> str:
        return _quad_trim_axis_label(self.imu_axis_roll, self.imu_axis_pitch)

    def _load_calibration(self, calibration: dict) -> None:
        try:
            axis_roll = float(calibration.get("imu_axis_roll"))
            axis_pitch = float(calibration.get("imu_axis_pitch"))
        except (TypeError, ValueError):
            return
        norm = math.hypot(axis_roll, axis_pitch)
        if norm < 0.5:
            return
        self.imu_axis_roll = axis_roll / norm
        self.imu_axis_pitch = axis_pitch / norm
        self.imu_axis = str(calibration.get("imu_axis") or self._axis_label())
        try:
            self.sign_to_cmd = float(
                calibration.get("imu_sign_to_cmd", self.sign_to_cmd))
        except (TypeError, ValueError):
            self.sign_to_cmd = -1.0
        if abs(self.sign_to_cmd) < 0.5:
            self.sign_to_cmd = -1.0
        try:
            target = float(calibration["target_pitch_deg"])
        except (KeyError, TypeError, ValueError):
            try:
                target = self.sign_to_cmd * float(
                    calibration["measured_lean_deg"])
            except (KeyError, TypeError, ValueError):
                return
        measured = target / self.sign_to_cmd
        self.target_pitch_deg = target
        self.pitch_deg = target
        self._axis_lp_meas = measured
        self._prev_cmd_pitch = target
        self.ready = True
        self.loaded_calibration = True
        self.calibration_source = str(calibration.get("source") or "saved")
        try:
            self.samples = int(calibration.get("samples") or 0)
        except (TypeError, ValueError):
            self.samples = 0

    def _project_lean_deg(self, roll_deg: float, pitch_deg: float) -> float:
        return (roll_deg * self.imu_axis_roll
                + pitch_deg * self.imu_axis_pitch)

    def _update_body_pitch(self, cmd_pitch: float, now: float,
                           target_pitch: float | None = None) -> bool:
        self.samples += 1
        if not self.ready:
            target = target_pitch
            if target is None:
                target = self.expected_pitch_deg
            if target is None or abs(float(target)) <= 5.0:
                target = cmd_pitch
            self.imu_axis = "body"
            self.imu_axis_roll = 0.0
            self.imu_axis_pitch = 1.0
            self.sign_to_cmd = 1.0
            self.target_pitch_deg = float(target)
            self.pitch_deg = cmd_pitch
            self._axis_lp_meas = cmd_pitch
            self._prev_cmd_pitch = cmd_pitch
            self._prev_update_t = now
            self.ready = True
            self.loaded_calibration = True
            self.calibration_source = "imu_body_frame"
            self.body_frame_mode = True
            return True
        return self._update_cmd_pitch(cmd_pitch, now)

    def _update_cmd_pitch(self, cmd_pitch: float, now: float) -> bool:
        prev_t = self._prev_update_t if self._prev_update_t is not None else now
        dt = max(0.05, min(0.75, now - prev_t))
        prev_pitch = (self._prev_cmd_pitch if self._prev_cmd_pitch is not None
                      else cmd_pitch)
        rate = _clamp((cmd_pitch - prev_pitch) / dt, -120.0, 120.0)
        target = (self.target_pitch_deg
                  if self.target_pitch_deg is not None else cmd_pitch)
        err = cmd_pitch - target
        eff = _deadband(err, QUAD_TRIM_DEADBAND_DEG)

        # Recovery entry/exit, two consecutive samples each way so one
        # noisy IMU read cannot flip the mode.
        if not self.recovering:
            self._recover_hi = (self._recover_hi + 1
                                if abs(err) >= QUAD_TRIM_RECOVER_ERR_DEG
                                else 0)
            if self._recover_hi >= 2:
                self.recovering = True
                self.recover_t0 = now
                self.recover_count += 1
                self._recover_lo = 0
        else:
            self._recover_lo = (self._recover_lo + 1
                                if abs(err) <= QUAD_TRIM_RECOVER_EXIT_DEG
                                else 0)
            if self._recover_lo >= 2:
                self.recovering = False
                self.recover_t0 = None
                self._recover_hi = 0

        if self.recovering:
            # Full second-line authority against the tip; the walk is
            # brace-holding so the lean dominates the stride dynamics.
            cmd_sign = self._rear_up_cmd_sign()
            desired_pitch = cmd_sign * (0.9 * eff + 0.05 * rate)
            desired_dx = cmd_sign * (0.0022 * eff + 0.00006 * rate)
            max_pitch = QUAD_TRIM_RECOVER_MAX_PITCH_DEG
            max_dx = QUAD_TRIM_RECOVER_MAX_DX_M
        else:
            cmd_sign = self._rear_up_cmd_sign()
            desired_pitch = cmd_sign * (0.55 * eff + 0.04 * rate)
            desired_dx = cmd_sign * (0.0012 * eff + 0.00004 * rate)
            max_pitch = QUAD_TRIM_MAX_PITCH_DEG
            max_dx = QUAD_TRIM_MAX_DX_M
        desired_pitch = _clamp(desired_pitch, -max_pitch, max_pitch)
        desired_dx = _clamp(desired_dx, -max_dx, max_dx)

        self.pitch_trim_deg = _slew(
            self.pitch_trim_deg, desired_pitch, 8.0 * dt)
        self.body_dx_trim_m = _slew(
            self.body_dx_trim_m, desired_dx, 0.030 * dt)
        # speed_scale keeps the normal slowdown formula in recovery too:
        # the streamer overrides pacing entirely (brace-hold) when the
        # pose fn exposes its stance schedule, and this is the sane
        # fallback when it does not.
        self.speed_scale = _clamp(
            1.0 - min(0.35, abs(err) / 28.0)
            - min(0.15, abs(rate) / 160.0),
            0.55, 1.0)
        if (self.recovering and self.abort_reason is None
                and self.recover_t0 is not None
                and now - self.recover_t0 > QUAD_TRIM_RECOVER_TIMEOUT_S):
            self.abort_reason = (
                f"balance:{err:+.1f}:{rate:+.0f}:recovery_timeout")
        self.pitch_deg = cmd_pitch
        self.pitch_error_deg = err
        self.pitch_rate_deg_s = rate
        self._prev_cmd_pitch = cmd_pitch
        self._prev_update_t = now

        # Rate branch is direction-aware: err and rate the SAME sign
        # means tipping further (fall onset); opposite signs mean the
        # body is swinging back toward level — that's a rock rebound or
        # recovery working, not a fall (limping there was measured to
        # cut off successful recoveries on the MuJoCo twin, 08-21).
        danger = (abs(err) > QUAD_TRIM_GUARD_ERROR_DEG
                  or (abs(err) > 10.0 and abs(rate) > 45.0
                      and err * rate > 0.0))
        self._danger_samples = self._danger_samples + 1 if danger else 0
        if self._danger_samples >= 2 and self.abort_reason is None:
            self.abort_reason = (
                f"balance:{err:+.1f}:{rate:+.0f}")
        return True

    def pose_trim(self) -> dict:
        if not self.enabled or not self.ready:
            return {"body_dx_m": 0.0, "pitch_rad": 0.0}
        return {
            "body_dx_m": self.body_dx_trim_m,
            "pitch_rad": math.radians(self.pitch_trim_deg),
        }

    def update(self, imu: dict | None, now: float) -> bool:
        if not self.enabled or not imu:
            return False
        if imu.get("body_frame_calibrated") and imu.get("body_pitch_deg") is not None:
            try:
                target = None
                if imu.get("body_pitch_target_deg") is not None:
                    target = float(imu["body_pitch_target_deg"])
                return self._update_body_pitch(
                    float(imu["body_pitch_deg"]), now,
                    target_pitch=target)
            except (TypeError, ValueError):
                pass
        rp = _imu_roll_pitch_deg(imu)
        if rp is None:
            return False
        roll_deg, pitch_deg = rp
        self.samples += 1

        if not self.ready:
            self._warmup.append((roll_deg, pitch_deg))
            if len(self._warmup) < 3:
                return True
            base_roll = sum(r for r, _p in self._warmup) / len(self._warmup)
            base_pitch = sum(p for _r, p in self._warmup) / len(self._warmup)
            calib = quad_trim_calibration_from_roll_pitch(
                base_roll, base_pitch,
                expected_pitch_deg=self.expected_pitch_deg,
                gait=self.gait, samples=len(self._warmup),
                source="warmup")
            if not calib.get("ok"):
                self.enabled = False
                self.disabled_reason = str(
                    calib.get("error") or "imu rear-lean baseline invalid")
                return True
            self._load_calibration(calib)
            self.loaded_calibration = False
            self.calibration_source = "warmup"
            self._prev_update_t = now
            return True

        selected = self._project_lean_deg(roll_deg, pitch_deg)
        alpha = 0.35
        if self._axis_lp_meas is None:
            self._axis_lp_meas = selected
        else:
            self._axis_lp_meas = (
                (1.0 - alpha) * self._axis_lp_meas + alpha * selected)
        cmd_pitch = self.sign_to_cmd * self._axis_lp_meas
        return self._update_cmd_pitch(cmd_pitch, now)

    def should_emit(self, now: float) -> bool:
        if self.abort_reason:
            return True
        if self.disabled_reason:
            return now - self._last_emit_t > 2.0
        if not self.ready:
            return False
        bucket = (round(self.pitch_trim_deg),
                  round(self.body_dx_trim_m * 1000), self.recovering)
        if bucket != self._last_emit_bucket and now - self._last_emit_t > 0.35:
            return True
        return now - self._last_emit_t > 1.25

    def mark_emitted(self, now: float) -> None:
        self._last_emit_t = now
        self._last_emit_bucket = (
            round(self.pitch_trim_deg),
            round(self.body_dx_trim_m * 1000), self.recovering)

    def event_data(self) -> dict:
        return {
            "gait": self.gait,
            "ready": self.ready,
            "enabled": self.enabled,
            "disabled_reason": self.disabled_reason,
            "calibration_source": self.calibration_source,
            "loaded_calibration": self.loaded_calibration,
            "body_frame_mode": self.body_frame_mode,
            "imu_axis": self.imu_axis,
            "imu_axis_roll": round(self.imu_axis_roll, 3),
            "imu_axis_pitch": round(self.imu_axis_pitch, 3),
            "imu_sign_to_cmd": self.sign_to_cmd,
            "target_pitch_deg": (
                None if self.target_pitch_deg is None
                else round(self.target_pitch_deg, 2)),
            "pitch_deg": (
                None if self.pitch_deg is None else round(self.pitch_deg, 2)),
            "err_deg": round(self.pitch_error_deg, 2),
            "rate_deg_s": round(self.pitch_rate_deg_s, 1),
            "pitch_trim_deg": round(self.pitch_trim_deg, 2),
            "body_dx_trim_mm": round(self.body_dx_trim_m * 1000.0, 1),
            "speed_scale": round(self.speed_scale, 2),
            "recovering": self.recovering,
            "recover_count": self.recover_count,
            "recover_s": (
                0.0 if (self.recover_t0 is None
                        or self._prev_update_t is None)
                else round(self._prev_update_t - self.recover_t0, 2)),
            "samples": self.samples,
        }

    def csv_cols(self) -> dict:
        d = self.event_data()
        return {
            "balance_axis": d["imu_axis"],
            "balance_axis_roll": f"{d['imu_axis_roll']:.3f}",
            "balance_axis_pitch": f"{d['imu_axis_pitch']:.3f}",
            "balance_target_pitch_deg": "" if d["target_pitch_deg"] is None
            else f"{d['target_pitch_deg']:.2f}",
            "balance_pitch_deg": "" if d["pitch_deg"] is None
            else f"{d['pitch_deg']:.2f}",
            "balance_err_deg": f"{d['err_deg']:.2f}",
            "balance_rate_deg_s": f"{d['rate_deg_s']:.2f}",
            "balance_pitch_trim_deg": f"{d['pitch_trim_deg']:.2f}",
            "balance_dx_trim_mm": f"{d['body_dx_trim_mm']:.2f}",
            "balance_speed_scale": f"{d['speed_scale']:.2f}",
            "balance_recovering": "1" if d["recovering"] else "0",
        }

    def status_suffix(self) -> str:
        if self.disabled_reason:
            return "trim off"
        if not self.ready:
            return "trim warmup"
        mode = "RECOVERING " if self.recovering else ""
        return (f"{mode}trim {self.imu_axis} {self.pitch_trim_deg:+.1f}deg "
                f"{self.body_dx_trim_m * 1000:+.0f}mm "
                f"err {self.pitch_error_deg:+.1f}")


def _clamp_live_speed(x) -> float:
    try:
        v = float(x)
    except (TypeError, ValueError):
        v = 1.0
    return max(LIVE_SPEED_MIN, min(LIVE_SPEED_MAX, v))


def _quad_recover_to_stand(bus: FeetechBus, live: set[int], check,
                           tracker: CurrentPeakTracker | None = None,
                           *, status_cb=None) -> str:
    """Try to leave a front/back quad tip in the normal stand pose.

    This is deliberately narrower than the hard tilt guard: if the robot
    is mostly pitching forward/backward, a slow six-foot stand is usually
    safer than dropping torque into a belly scrape.  If it is already far
    sideways or the IMU says the total tilt is huge, stop adding commands.
    """
    imu = None
    read_imu = getattr(bus, "read_imu", None)
    if callable(read_imu):
        try:
            imu = read_imu()
        except Exception:
            imu = None
    rp = _imu_roll_pitch_deg(imu) if imu else None
    if rp is not None:
        roll, pitch = rp
        total = math.hypot(roll, pitch)
        if abs(roll) > 30.0 or total > 55.0:
            _limp_all(bus, live)
            return (f"limped: tilt too large for auto-stand "
                    f"(roll {roll:+.0f}, pitch {pitch:+.0f})")
    try:
        _enable_torque(bus, live)
    except Exception:
        pass
    _set_torque_limit(bus, live, 850)
    if status_cb is not None:
        try:
            status_cb("quad recovery: returning to stand")
        except Exception:
            pass
    ok = _soft_glide(bus, _stand_zero_pose(), live, 2.5, check,
                     max_speed=650, max_acc=60, softness=1.4)
    if not ok:
        _hold_here(bus, live)
        return "recovery interrupted; holding current pose"
    if tracker is not None:
        tracker.sample(bus, live)
    _set_torque_limit(bus, live, 1000)
    return "recovered to stand"


def stream_pose_fn(bus: FeetechBus, live: set[int], pose_fn, *,
                   seconds: float, abort_check=None,
                   speed_fn=None, status_cb=None, label: str = "stream",
                   tracker: CurrentPeakTracker | None = None,
                   guard_a: float = STREAM_GUARD_A,
                   log: MotionLog | None = None,
                   max_speed: int = 3000, max_acc: int = 200,
                   tick_s: float = STREAM_TICK_S,
                   tilt_guard_deg: float | None = None,
                   balance_trim: QuadPitchTrim | None = None) -> str:
    """Stream ``pose_fn(t)`` at ~20 Hz with a live tempo multiplier.

    Demo time ``t`` advances by wall-dt x ``speed_fn()`` every tick, so
    the web speed slider changes tempo MID-MOTION.  ``seconds`` is demo
    time (equals wall time at 1x).  Returns ``done`` / ``aborted`` /
    ``guard`` — on ``guard`` the robot is already holding in place;
    the caller decides how to surface it.

    ``tilt_guard_deg``: for gaits that can tip (the reared quad walks)
    — if body tilt from the IMU exceeds this on two consecutive samples
    the robot is going over; go LIMP immediately (per the incident
    rules: never fight a tip) and return ``tilt:<deg>``.
    """
    check = abort_check or (lambda: False)
    spd = speed_fn or (lambda: 1.0)
    if tracker is None:
        tracker = CurrentPeakTracker()
    seconds = float(seconds)
    # Carrot scales with the tick so slower write rates still command
    # ~2+ ticks ahead of schedule.
    lookahead_s = max(STREAM_LOOKAHEAD_S, 2.4 * tick_s)
    q0 = pose_fn(0.0)
    # Align onto the start pose first (quick, abortable) so the streamer
    # never has to cover a big gap in one tick.
    present = _read_pose(bus, live)
    worst0 = max((abs(a - b) for j, (a, b) in enumerate(zip(present, q0))
                  if joint_to_servo_id(j) in live), default=0.0)
    if worst0 > 5.0:
        if not ease_to_pose(bus, q0, abort_check=check,
                            seconds=max(0.6, worst0 / 90.0),
                            label=f"{label} align",
                            current_tracker=tracker):
            return "aborted"
    streamer = PoseStreamer()
    streamer.last = list(q0)   # primed: skip the gentle first-write ease
    # Balance recovery brace-hold: quad pose fns expose their stance
    # schedule so the clock can freeze ONLY with all four feet planted.
    all_stance_at = getattr(pose_fn, "all_stance_at", None)
    stall_prev: set = set()
    tilt_prev = False
    read_imu = getattr(bus, "read_imu", None)
    need_imu = ((tilt_guard_deg is not None or balance_trim is not None)
                and callable(read_imu))
    sweep_n = 0
    t = 0.0
    t0 = time.monotonic()
    wall_prev = t0
    t_prev = 0.0
    last_sample = -1.0
    motion_period_s = _motion_cmd_period_s()
    last_motion_cmd = t0 - motion_period_s if motion_period_s else t0
    last_imu: dict | None = None
    while t < seconds:
        if check():
            # Return immediately — no bus ops here (concurrent hold from
            # the web Stop handler was hanging the MCU link).
            _emit_motion_cmd(
                label, q0 if streamer.last is None else streamer.last, {},
                t_s=t, seconds=seconds, wall_s=time.monotonic() - t0,
                rate=0.0, tick_s=tick_s, lookahead_s=lookahead_s,
                tracker=tracker, balance_trim=balance_trim, imu=last_imu,
                servo_goals=getattr(streamer, "last_written_goal", {}),
                reason="aborted")
            return "aborted"
        wall = time.monotonic()
        rate = _clamp_live_speed(spd())
        if balance_trim is not None and balance_trim.ready:
            if balance_trim.recovering and all_stance_at is not None:
                # Recovery brace-hold: finish the current step at no
                # more than 1x, then FREEZE at the next all-stance
                # window and let the trim lean against the tip
                # (pose_fn re-reads the trim every tick).  Measured on
                # the MuJoCo twin (08-21): freezing mid-swing strands a
                # foot on tripod support and deepens the tip, and so
                # does SLOWING a swing (the nose hangs unsupported for
                # longer) — never drag the clock while a foot is in
                # the air.
                rate = min(rate, 1.0)
                try:
                    if all_stance_at(t):
                        rate = 0.0
                except Exception:
                    pass
            else:
                rate = _clamp(rate * balance_trim.speed_scale,
                              LIVE_SPEED_MIN, LIVE_SPEED_MAX)
        t += (wall - wall_prev) * rate
        wall_prev = wall
        q = pose_fn(min(t + lookahead_s * rate, seconds))
        # dt*0.75 cancels _speed_for_delta's 0.9 undershoot and commands
        # slightly above the carrot rate so accumulated lag drains —
        # same sizing as the stand-up pursuit.
        wrote = streamer.write(
            bus, q, live,
            dt=min(max(wall - t0 - t_prev, 0.03), 0.25) * 0.75,
            deadband=0.3, max_speed=max_speed, max_acc=max_acc)
        t_prev = wall - t0
        if wall - t0 - last_sample > 0.25:
            # Feedback sweeps cost real bus time — sample sparsely.
            tracker.sample(bus, live)
            last_sample = wall - t0
            sweep_n += 1
            if log is not None and sweep_n % 2 == 0:
                extra = (balance_trim.csv_cols()
                         if balance_trim is not None else None)
                log.sample(bus, q, wrote=wrote, extra=extra)
            if tracker.peak_a > STREAM_HARD_CAP_A:
                _emit_motion_cmd(
                    label, q, wrote, t_s=t, seconds=seconds,
                    wall_s=wall - t0, rate=rate, tick_s=tick_s,
                    lookahead_s=lookahead_s, tracker=tracker,
                    balance_trim=balance_trim, imu=last_imu,
                    servo_goals=getattr(streamer, "last_written_goal", {}),
                    reason="current_guard")
                _hold_here(bus, live)
                return "guard"
            # Stall-fight semantics (same as the stand-up lab): a joint
            # over the limit while NOT moving, two sweeps in a row.
            now = {fb["joint"] for fb in tracker.last_fb
                   if abs(fb["current_a"]) > guard_a
                   and abs(fb["speed_deg_s"]) < 8.0}
            if now & stall_prev:
                _emit_motion_cmd(
                    label, q, wrote, t_s=t, seconds=seconds,
                    wall_s=wall - t0, rate=rate, tick_s=tick_s,
                    lookahead_s=lookahead_s, tracker=tracker,
                    balance_trim=balance_trim, imu=last_imu,
                    servo_goals=getattr(streamer, "last_written_goal", {}),
                    reason="stall_guard")
                _hold_here(bus, live)
                return "guard"
            stall_prev = now
            if need_imu:
                try:
                    imu = read_imu()
                except Exception:
                    imu = None
                last_imu = imu
                if balance_trim is not None:
                    updated = balance_trim.update(imu, wall)
                    if updated and balance_trim.should_emit(wall):
                        try:
                            from event_log import emit
                            emit("quad_trim", balance_trim.status_suffix(),
                                 src="balance",
                                 data=balance_trim.event_data())
                        except Exception:
                            pass
                        balance_trim.mark_emitted(wall)
                    if balance_trim.abort_reason:
                        _emit_motion_cmd(
                            label, q, wrote, t_s=t, seconds=seconds,
                            wall_s=wall - t0, rate=rate, tick_s=tick_s,
                            lookahead_s=lookahead_s, tracker=tracker,
                            balance_trim=balance_trim, imu=last_imu,
                            servo_goals=getattr(
                                streamer, "last_written_goal", {}),
                            reason="balance_guard")
                        return balance_trim.abort_reason
                if tilt_guard_deg is not None and imu and "az_g" in imu:
                    norm = math.sqrt(imu.get("ax_g", 0.0) ** 2
                                     + imu.get("ay_g", 0.0) ** 2
                                     + imu["az_g"] ** 2) or 1.0
                    tilt = math.degrees(
                        math.acos(max(-1.0, min(1.0, imu["az_g"] / norm))))
                    if tilt > tilt_guard_deg:
                        if tilt_prev:
                            # Going over — do NOT hold a fighting pose.
                            _emit_motion_cmd(
                                label, q, wrote, t_s=t, seconds=seconds,
                                wall_s=wall - t0, rate=rate,
                                tick_s=tick_s, lookahead_s=lookahead_s,
                                tracker=tracker, balance_trim=balance_trim,
                                imu=last_imu,
                                servo_goals=getattr(
                                    streamer, "last_written_goal", {}),
                                reason="tilt_guard")
                            for sid in sorted(live):
                                try:
                                    bus.pkt.write1ByteTxRx(
                                        sid, ADDR_TORQUE_ENABLE, 0)
                                except Exception:
                                    pass
                            return f"tilt:{tilt:.0f}"
                        tilt_prev = True
                    else:
                        tilt_prev = False
            if status_cb is not None:
                try:
                    suffix = (f" {balance_trim.status_suffix()}"
                              if balance_trim is not None else "")
                    status_cb(f"{label}: {t:.0f}/{seconds:.0f}s "
                              f"x{rate:.2f} peak {tracker.peak_a:.2f}A"
                              f"{suffix}")
                except Exception:
                    pass
        if motion_period_s and wall - last_motion_cmd >= motion_period_s:
            _emit_motion_cmd(
                label, q, wrote, t_s=t, seconds=seconds,
                wall_s=wall - t0, rate=rate, tick_s=tick_s,
                lookahead_s=lookahead_s, tracker=tracker,
                balance_trim=balance_trim, imu=last_imu,
                servo_goals=getattr(streamer, "last_written_goal", {}))
            last_motion_cmd = wall
        time.sleep(tick_s)
    _emit_motion_cmd(
        label, pose_fn(seconds), {}, t_s=seconds, seconds=seconds,
        wall_s=time.monotonic() - t0, rate=0.0, tick_s=tick_s,
        lookahead_s=lookahead_s, tracker=tracker,
        balance_trim=balance_trim, imu=last_imu,
        servo_goals=getattr(streamer, "last_written_goal", {}),
        reason="done")
    return "done"


# ---------------------------------------------------------------------------
# Demos — offsets around zero (legs straight out / "arms in the air")
# ---------------------------------------------------------------------------


def pose_shimmy(t: float) -> list[float]:
    """Odd/even yaw wave — arms shimmy, no stand-up.

    Kept slow (~0.55 Hz, ±8°) so STS trapezoids can track without
    overshooting into gear backlash.
    """
    pose = _zero_pose()
    amp = 8.0 * math.sin(2 * math.pi * 0.55 * t)
    for leg in range(6):
        sign = 1.0 if leg % 2 == 0 else -1.0
        _yaw_hip_knee(leg, pose, yaw=sign * amp)
    return pose


def frames_shimmy(seconds: float = 7.0):
    for i in range(max(1, int(seconds / DT))):
        yield pose_shimmy(i * DT)


def pose_ripple(t: float) -> list[float]:
    """Yaw wave travels around the hex."""
    pose = _zero_pose()
    for leg in range(6):
        phase = leg * (math.pi / 3.0)
        yaw = 10.0 * math.sin(2 * math.pi * 0.45 * t - phase)
        _yaw_hip_knee(leg, pose, yaw=yaw)
    return pose


def frames_ripple(seconds: float = 8.0):
    for i in range(max(1, int(seconds / DT))):
        yield pose_ripple(i * DT)


# Breathe is glide-based (one SyncWrite per half-breath).  Host-streaming
# restarts the STS trapezoid every tick and feels like discrete steps —
# same lesson as ease_to_pose / rise.
#
# ``size`` scales amplitude (1.0 = nominal, 2.0 ≈ old breathe+).  Cycle rate
# slows a little as size grows so large flexes still coast smoothly.
# Slower + slightly larger than before: looks like a breath without barking
# the STS trapezoid through backlash (web softness slider stacks on top).
BREATHE_HZ = 0.20
BREATHE_HIP_DEG = -11.0
BREATHE_KNEE_DEG = 14.0


def _clamp_breathe_size(size: float) -> float:
    try:
        s = float(size)
    except (TypeError, ValueError):
        s = 1.0
    return max(0.5, min(3.0, s))


def _breathe_pose(amount: float, *, hip_deg: float, knee_deg: float) -> list[float]:
    """``amount`` 0 = arms out (zero), 1 = soft folded peak."""
    amount = max(0.0, min(1.0, float(amount)))
    pose = _zero_pose()
    hip = hip_deg * amount
    knee = knee_deg * amount
    for leg in range(6):
        _yaw_hip_knee(leg, pose, hip=hip, knee=knee)
    return pose


def _soft_glide(bus: FeetechBus, goal: list[float], live: set[int],
                seconds: float, abort_check,
                *, start: list[float] | None = None,
                max_speed: int = 240,
                max_acc: int = 12,
                softness: float = 1.0,
                log: MotionLog | None = None) -> bool:
    """One SyncWrite + timed coast (no per-tick re-command).

    On abort: return False immediately — do **not** bus-scan hold here.
    Concurrent hold from the web Stop handler was hanging the MCU link.

    ``softness`` > 1 lowers accel (gentler ramps, less bark through backlash).
    Optional ``log`` samples cmd(=goal) vs encoder while coasting.
    """
    seconds = max(0.5, float(seconds))
    soft = max(0.5, min(3.0, float(softness)))
    if abort_check():
        return False
    if start is None:
        start = _read_pose(bus, live)
    max_delta = 0.0
    for joint, (a, b) in enumerate(zip(start, goal)):
        if joint_to_servo_id(joint) in live:
            max_delta = max(max_delta, abs(b - a))
    if max_delta < 0.3:
        t0 = time.monotonic()
        while time.monotonic() - t0 < min(0.25, seconds):
            if abort_check():
                return False
            if log is not None:
                log.sample(bus, goal, wrote={})
            time.sleep(0.05)
        return True
    speed = int(min(int(max_speed), max(20, max_delta * COUNTS_PER_DEG / seconds)))
    # Higher softness → longer accel fraction → lower acc register.
    acc_cap = max(3, int(int(max_acc) / soft))
    acc = int(min(acc_cap, max(2, speed / (100.0 * seconds * (0.45 + 0.30 * soft)))))
    speed, acc = normalize_speed(speed), normalize_acc(acc)
    if abort_check():
        return False
    _write_pose(bus, goal, live, speed=speed, acc=acc)
    wrote = {j: (speed, acc) for j in range(N_JOINTS)
             if joint_to_servo_id(j) in live}
    t0 = time.monotonic()
    tick = 0
    while time.monotonic() - t0 < seconds:
        if abort_check():
            return False
        # Sparse telemetry — full feedback every tick starves the MCU bridge.
        if log is not None and tick % 2 == 0:
            log.sample(bus, goal, wrote=wrote if tick == 0 else {})
        tick += 1
        time.sleep(0.05)
    return True


def run_breathe_demo(bus: FeetechBus, *,
                     seconds: float = 10.0,
                     abort_check=None,
                     size: float = 1.0,
                     rate: float | None = None,
                     torque: int | None = None,
                     softness: float = 1.0,
                     big: bool = False,
                     speed_fn=None,
                     log_path: Path | None = None) -> str:
    """Inhale / exhale via servo-side trapezoids (no host frame stream).

    Tunables (web sliders):
      size     — amplitude scale (0.5–3)
      rate     — breath Hz (None → auto from size)
      torque   — SRAM torque limit 200–1000 (None → DEMO_TORQUE_LIMIT)
      softness — accel gentleness 0.5–3 (1 = nominal)
      speed_fn — LIVE tempo multiplier, read at each half-breath (the
                 glide itself stays servo-side — that's breathe's charm)
    """
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    try:
        tlim = int(torque) if torque is not None else DEMO_TORQUE_LIMIT
    except (TypeError, ValueError):
        tlim = DEMO_TORQUE_LIMIT
    tlim = max(150, min(1000, tlim))
    _set_torque_limit(bus, live, tlim)

    size = _clamp_breathe_size(2.0 if big else size)
    soft = max(0.5, min(3.0, float(softness)))
    if rate is None:
        hz = BREATHE_HZ / (0.65 + 0.35 * size)
    else:
        try:
            hz = float(rate)
        except (TypeError, ValueError):
            hz = BREATHE_HZ
        hz = max(0.08, min(0.60, hz))
    hip = BREATHE_HIP_DEG * size
    knee = BREATHE_KNEE_DEG * size
    half = 0.5 / hz
    # Floor half-period so large/soft breaths never snap.
    half = max(1.6, half)
    n_cycles = max(1, int(round(float(seconds) * hz)))
    print(f"  breathe — {n_cycles} cycle(s) @ {hz:.2f} Hz  size {size:.2f}× "
          f"torque {tlim} soft {soft:.2f}× "
          f"(hip {hip:.0f}° / knee {knee:.0f}°)")
    print("  Any key aborts.")

    open_pose = _breathe_pose(0.0, hip_deg=hip, knee_deg=knee)
    peak_pose = _breathe_pose(1.0, hip_deg=hip, knee_deg=knee)
    glide_kw = dict(
        softness=soft,
        max_acc=max(3, int(10 / soft)),
        max_speed=max(60, int(220 / (0.7 + 0.3 * soft))),
    )

    log_cm = None
    if log_path is not None:
        log_cm = MotionLog(log_path, live)
        log_cm.__enter__()
        glide_kw["log"] = log_cm

    def _half_s() -> float:
        # Live tempo: each half-breath reads the web slider. Floor drops
        # with speed (a fast breath may snap a little; that's the ask).
        s = _clamp_live_speed(speed_fn()) if speed_fn is not None else 1.0
        return max(0.8, half / s)

    status = "done"
    try:
        if not _soft_glide(bus, open_pose, live, min(1.8, _half_s()), check,
                           **glide_kw):
            status = "aborted"
            return status
        here = open_pose
        for _i in range(n_cycles):
            if not _soft_glide(bus, peak_pose, live, _half_s(), check,
                               start=here, **glide_kw):
                status = "aborted"
                return status
            here = peak_pose
            if not _soft_glide(bus, open_pose, live, _half_s(), check,
                               start=here, **glide_kw):
                status = "aborted"
                return status
            here = open_pose

        if check():
            status = "aborted"
            return status
        print("  Demo finished — easing back to zero ...")
        if not go_to_zero_pose(bus, abort_check=check, seconds=2.5):
            status = "aborted"
            return status
        _limp_all(bus, live)
        return status
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)
        _set_torque_limit(bus, live, 1000)


def frames_breathe(seconds: float = 10.0):
    """Legacy frame generator (unused by run_demo — kept for tooling)."""
    base = _zero_pose()
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        pose = list(base)
        breath = 0.5 * (1.0 - math.cos(2 * math.pi * BREATHE_HZ * t))
        for leg in range(6):
            _yaw_hip_knee(leg, pose,
                          hip=BREATHE_HIP_DEG * breath,
                          knee=BREATHE_KNEE_DEG * breath)
        yield pose


# STS operating mode EEPROM reg (0=position, 1=wheel/speed, 2=PWM, 3=step).
ADDR_MODE = 33
ADDR_ACC = 41
ADDR_GOAL_SPEED = 46


def _encode_sts_speed(speed: int) -> int:
    """Signed steps/s → Feetech bit-15 direction encoding."""
    speed = int(speed)
    if speed < 0:
        return ((-speed) & 0x7FFF) | 0x8000
    return speed & 0x7FFF


def _set_servo_mode(bus: FeetechBus, sid: int, mode: int) -> None:
    """EEPROM mode switch (torque off → unlock → write → lock → torque on)."""
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.02)
    bus.pkt.unLockEprom(sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_MODE, int(mode) & 0xFF)
    bus.pkt.LockEprom(sid)
    time.sleep(0.02)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)


def _write_wheel_speed(bus: FeetechBus, sid: int, speed: int, acc: int) -> None:
    """Wheel-mode command: accel + goal speed (no position target)."""
    acc = max(0, min(254, int(acc)))
    bus.pkt.write1ByteTxRx(sid, ADDR_ACC, acc)
    bus.pkt.write2ByteTxRx(sid, ADDR_GOAL_SPEED, _encode_sts_speed(speed))


def _wheel_stop(bus: FeetechBus, sids: list[int]) -> None:
    for sid in sids:
        try:
            _write_wheel_speed(bus, sid, 0, 50)
        except Exception:
            pass


def run_breathe_vel_demo(bus: FeetechBus, *,
                         seconds: float = 10.0,
                         abort_check=None,
                         size: float = 1.0,
                         rate: float | None = None,
                         torque: int | None = None,
                         softness: float = 1.0,
                         log_path: Path | None = None) -> str:
    """Breathe using STS **wheel/speed mode** — no position targeting.

    Each half-breath: command a constant hip/knee speed (with accel ramp)
    and hold it for the half-period, then reverse.  Avoids the position-PID
    hunt through gearbox backlash / stiction that makes slow WritePosEx
    moves shake.

    ALWAYS restores mode=0 (position) on exit.
    """
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    check = abort_check or (lambda: False)

    size = _clamp_breathe_size(size)
    soft = max(0.5, min(3.0, float(softness)))
    if rate is None:
        hz = BREATHE_HZ / (0.65 + 0.35 * size)
    else:
        try:
            hz = float(rate)
        except (TypeError, ValueError):
            hz = BREATHE_HZ
        hz = max(0.08, min(0.60, hz))
    try:
        tlim = int(torque) if torque is not None else DEMO_TORQUE_LIMIT
    except (TypeError, ValueError):
        tlim = DEMO_TORQUE_LIMIT
    tlim = max(150, min(1000, tlim))

    hip_peak = BREATHE_HIP_DEG * size          # negative
    knee_peak = BREATHE_KNEE_DEG * size        # positive
    half = 0.5 / hz
    n_cycles = max(1, int(round(float(seconds) * hz)))
    # Speed in encoder steps/s so Δθ completes in ``half`` seconds.
    hip_spd = int(round(abs(hip_peak) * COUNTS_PER_DEG / half))
    knee_spd = int(round(abs(knee_peak) * COUNTS_PER_DEG / half))
    hip_spd = max(8, min(400, hip_spd))
    knee_spd = max(8, min(400, knee_spd))
    acc = max(1, min(40, int(12 / soft)))

    # Hip/knee joints only (yaws stay in position mode at 0).
    move_joints = [j for j in range(N_JOINTS)
                   if j % 3 in (1, 2) and joint_to_servo_id(j) in live]
    move_sids = [joint_to_servo_id(j) for j in move_joints]
    if not move_sids:
        print("  No hip/knee servos live — skip.")
        return "skipped"

    print(f"  breathe_v — VELOCITY mode  {n_cycles} cycle(s) @ {hz:.2f} Hz")
    print(f"    size {size:.2f}×  hip {hip_peak:.0f}° @ ±{hip_spd}  "
          f"knee {knee_peak:.0f}° @ ±{knee_spd}  acc={acc}  τ={tlim}")
    print("    (no position target — hold speed, then reverse)")
    print("  Any key aborts.  Mode restored to position on exit.")

    status = "done"
    log_cm = None
    if log_path is not None:
        log_cm = MotionLog(log_path, live)
        log_cm.__enter__()
    try:
        # Start from logical zero in position mode, then flip movers to wheel.
        _enable_torque(bus, live)
        _set_torque_limit(bus, live, tlim)
        if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                            seconds=max(2.5, half + 0.5),
                            label="zero before vel-breathe"):
            status = "aborted"
            return status

        # Refuse wheel mode if a hip/knee is still far from zero — the
        # old watchdog false-tripped on unzeroed / wrong-sign joints (~170°).
        bad = []
        for j in move_joints:
            deg = bus.read_position_deg(j)
            if deg is not None and abs(deg) > 20.0:
                bad.append((j, deg))
        if bad:
            print("  breathe_v: hip/knee not near zero — skip wheel mode:")
            for j, deg in bad[:6]:
                print(f"    j{j} at {deg:.1f}°")
            print("  Tip: Set HERE as zero, or use position breathe instead.")
            status = "aborted"
            return status

        print(f"  Switching {len(move_sids)} hip/knee servo(s) → wheel mode …")
        for sid in move_sids:
            if check():
                status = "aborted"
                return status
            _set_servo_mode(bus, sid, 1)
        _set_torque_limit(bus, live, tlim)

        def _drive(sign: float) -> bool:
            """sign +1 = inhale (toward peak), -1 = exhale (toward open)."""
            for j in move_joints:
                sid = joint_to_servo_id(j)
                axis = j % 3
                if axis == 1:  # hip: peak is negative → inhale speed < 0
                    spd = int(round(-sign * hip_spd))
                else:          # knee: peak is positive → inhale speed > 0
                    spd = int(round(+sign * knee_spd))
                spd *= int(JOINT_SIGN[j])
                _write_wheel_speed(bus, sid, spd, acc)
            t0 = time.monotonic()
            # Position watchdog — wheel mode has no angle goal.
            lim_margin = 8.0
            tick = 0
            while time.monotonic() - t0 < half:
                if check():
                    _wheel_stop(bus, move_sids)
                    return False
                time.sleep(0.08)
                tick += 1
                frac = min(1.0, (time.monotonic() - t0) / half)
                # Ideal cmd along open↔peak for telemetry (approx).
                amount = frac if sign > 0 else (1.0 - frac)
                cmd = _breathe_pose(amount, hip_deg=hip_peak, knee_deg=knee_peak)
                if log_cm is not None and tick % 2 == 0:
                    log_cm.sample(bus, cmd, wrote={})
                # Occasional bound check (don't spam the bus).
                if tick % 3 != 0:
                    continue
                for j in move_joints:
                    if joint_to_servo_id(j) not in live:
                        continue
                    deg = bus.read_position_deg(j)
                    if deg is None:
                        continue
                    axis = j % 3
                    peak = hip_peak if axis == 1 else knee_peak
                    lo = min(0.0, peak) - lim_margin
                    hi = max(0.0, peak) + lim_margin
                    if deg < lo or deg > hi:
                        print(f"    watchdog: j{j} at {deg:.1f}° "
                              f"outside [{lo:.0f},{hi:.0f}] — stopping")
                        _wheel_stop(bus, move_sids)
                        return False
            _wheel_stop(bus, move_sids)
            time.sleep(0.05)
            return True

        for _i in range(n_cycles):
            if not _drive(+1.0):
                status = "aborted"
                break
            if not _drive(-1.0):
                status = "aborted"
                break
    finally:
        if log_cm is not None:
            try:
                log_cm.__exit__(None, None, None)
            except Exception:
                pass
        print("  Restoring position mode on hip/knee servos …")
        _wheel_stop(bus, move_sids)
        time.sleep(0.05)
        for sid in move_sids:
            try:
                _set_servo_mode(bus, sid, 0)
            except Exception as e:
                print(f"    mode restore failed id={sid}: {e}")
        _set_torque_limit(bus, live, 1000)
        # Re-home gently so we aren't left mid-flex in position mode.
        try:
            if status != "aborted" and not check():
                print("  Easing back to zero …")
                go_to_zero_pose(bus, abort_check=check, seconds=2.0)
                _limp_all(bus, live)
            else:
                # Freeze wherever we are, then limp — don't fight stiction.
                try:
                    _hold_here(bus, live)
                except Exception:
                    pass
                _limp_all(bus, live)
        except Exception as e:
            print(f"  cleanup: {e}")
            try:
                _limp_all(bus, live)
            except Exception:
                pass
    return status


SHIMMY_V_AMP_DEG = 8.0
SHIMMY_V_HZ = 0.55
# Host P-gain (1/s): velocity correction per unit position error.
# 4.0 rang at the swing peaks (feed-forward is zero there, so the
# P-term alone unwinds the overshoot and oscillates through the
# ~20-40 ms loop delay — user-visible wiggle, 2026-08-19). 2.5 leans
# harder on the feed-forward and lets the peaks settle.
SHIMMY_V_KP = 2.5
SHIMMY_V_MAX_CPS = 500       # wheel speed clamp (~44 deg/s)
SHIMMY_V_WATCHDOG_DEG = 20.0
# Per-servo GOAL_SPEED register writes cost ~1.5 ms each through the
# MCU; six of them fit a 20 ms tick, not a 10 ms one. (SyncWritePosEx
# would be one frame, but writing its position field in wheel mode
# corrupts the servo's position bookkeeping — measured 2026-08-19 as a
# ~2.6 count/write drift of the reported position in the direction of
# motion, ~46° over 2 s, on all six yaws simultaneously.)
SHIMMY_V_DT = 0.02
# Wheel accel, set once at mode switch. 30 (3000 counts/s²) added
# ~0.1 s of speed-slew lag that carried joints ~20% past the peaks;
# 60 halves it. Per-tick speed steps are ~22 counts/s, so a snappier
# slew stays smooth.
SHIMMY_V_ACC = 60
# Wheel-mode speed calibration: actual speed runs ~28% above the
# commanded counts/s on these STS3215s (measured 2026-08-19: amplitude
# 7.35° std vs 5.67° target with pure feed-forward at Kp 2.5; the
# earlier Kp 4.0 run was masking it by pulling back). Scale the
# feed-forward so the P-term only trims residuals.
SHIMMY_V_FF = 5.67 / 7.35


def run_shimmy_vel_demo(bus: FeetechBus, *,
                        seconds: float = 8.0,
                        abort_check=None,
                        log_path: Path | None = None) -> str:
    """Shimmy in **wheel/speed mode** — 100 Hz velocity streaming.

    Position-mode streaming (PoseStreamer) is limited by the STS
    position PID dithering through gear backlash and restarting its
    profile on every packet. Here the six yaws switch to wheel mode and
    get a signed speed each tick: trajectory feed-forward velocity plus
    a small host P-correction on measured position error (closed over
    the ~200 Hz MCU position cache). No position goal, no profile
    restarts, no PID hunt.

    Each tick writes ONLY the goal-speed register (bit-15 signed) per
    yaw servo — never the position field, which in wheel mode corrupts
    the servo's position bookkeeping. ALWAYS restores mode 0 on exit.
    """
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    yaw_joints = [j for j in range(N_JOINTS)
                  if j % 3 == 0 and joint_to_servo_id(j) in live]
    yaw_sids = [joint_to_servo_id(j) for j in yaw_joints]
    if not yaw_joints:
        print("  No yaw servos live — skip.")
        return "skipped"
    read_all = getattr(bus, "read_all_positions", None)
    if not callable(read_all):
        print("  shimmy_v needs the MCU bulk-position path — skip.")
        return "skipped"

    print(f"  shimmy_v — VELOCITY mode, {len(yaw_joints)} yaws @ "
          f"{1.0 / SHIMMY_V_DT:.0f} Hz, ±{SHIMMY_V_AMP_DEG:.0f}° "
          f"{SHIMMY_V_HZ:.2f} Hz wave")
    print("  Any key aborts.  Mode restored to position on exit.")

    status = "done"
    log_cm = None
    if log_path is not None:
        log_cm = MotionLog(log_path, live)
        log_cm.__enter__()
    try:
        _enable_torque(bus, live)
        _set_torque_limit(bus, live, DEMO_TORQUE_LIMIT)
        if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                            seconds=2.5, label="zero before shimmy_v"):
            status = "aborted"
            return status
        # Refuse wheel mode away from zero (mirrors breathe_v guard).
        bad = [(j, d) for j in yaw_joints
               if (d := bus.read_position_deg(j)) is not None
               and abs(d) > 15.0]
        if bad:
            print("  shimmy_v: yaw(s) not near zero — skip wheel mode:")
            for j, d in bad[:6]:
                print(f"    j{j} at {d:.1f}°")
            status = "aborted"
            return status

        print(f"  Switching {len(yaw_sids)} yaw servo(s) → wheel mode …")
        for sid in yaw_sids:
            if check():
                status = "aborted"
                return status
            _set_servo_mode(bus, sid, 1)
        # Verify — a servo stuck in position mode would slam on the
        # first big signed-speed write, so refuse to run instead.
        for sid in yaw_sids:
            mode, r, _e = bus.pkt.read1ByteTxRx(sid, ADDR_MODE)
            if r != bus.scs.COMM_SUCCESS or int(mode) != 1:
                print(f"  mode verify failed on id={sid} — aborting.")
                status = "aborted"
                return status
        _set_torque_limit(bus, live, DEMO_TORQUE_LIMIT)
        # Wheel accel set ONCE — per-tick writes touch only GOAL_SPEED.
        for sid in yaw_sids:
            _write_wheel_speed(bus, sid, 0, SHIMMY_V_ACC)

        # Wheel mode re-references the present-position register (mode
        # switch at zero read ~149° once, 2026-08-19 — same quirk behind
        # breathe_v's old ~170° watchdog false trips), so absolute
        # angles are meaningless here. Control on DISPLACEMENT from a
        # post-switch baseline instead; we just verified the true pose
        # is zero, so displacement == logical angle for this wave.
        base = read_all()
        missing_base = [j for j in yaw_joints if base.get(j) is None]
        if missing_base:
            print(f"  no baseline reading for joints {missing_base} — abort.")
            status = "aborted"
            return status

        def _disp(j: int, p: float) -> float:
            """Displacement from baseline, unwrapped to (−180, 180]."""
            return ((p - base[j] + 180.0) % 360.0) - 180.0

        # Max believable per-tick move: speed clamp + margin. Sync-read
        # glitches can fabricate ~45° single-tick jumps (seen 2026-08-19,
        # j0); reject those instead of tripping the watchdog on them,
        # but only for a few consecutive ticks — persistent readings win.
        max_step = (SHIMMY_V_MAX_CPS / COUNTS_PER_DEG) * SHIMMY_V_DT * 4.0
        last_d: dict[int, float] = {}
        sus: dict[int, int] = {}

        omega = 2 * math.pi * SHIMMY_V_HZ
        n = max(1, int(float(seconds) / SHIMMY_V_DT))
        t0 = time.monotonic()
        tick_i = 0
        for i in range(n):
            if check():
                status = "aborted"
                return status
            t = i * SHIMMY_V_DT
            present = read_all()
            for j in yaw_joints:
                sign = 1.0 if (j // 3) % 2 == 0 else -1.0
                ref = sign * SHIMMY_V_AMP_DEG * math.sin(omega * t)
                vel = sign * SHIMMY_V_AMP_DEG * omega * math.cos(omega * t)
                p = present.get(j)
                if p is None:
                    continue
                d = _disp(j, p)
                prev_d = last_d.get(j)
                if prev_d is not None and abs(d - prev_d) > max_step:
                    sus[j] = sus.get(j, 0) + 1
                    if sus[j] < 3:
                        continue  # skip the glitched sample, coast
                else:
                    sus[j] = 0
                last_d[j] = d
                # Watchdog: wheel mode has no angle goal.
                if abs(d) > SHIMMY_V_WATCHDOG_DEG:
                    print(f"    watchdog: j{j} moved {d:.1f}° — stopping")
                    _wheel_stop(bus, yaw_sids)
                    status = "aborted"
                    return status
                err = ref - d
                cps = (vel * SHIMMY_V_FF + SHIMMY_V_KP * err) * COUNTS_PER_DEG
                cps *= float(JOINT_SIGN[j])
                cps = max(-SHIMMY_V_MAX_CPS, min(SHIMMY_V_MAX_CPS, cps))
                sid = joint_to_servo_id(j)
                bus.pkt.write2ByteTxRx(sid, ADDR_GOAL_SPEED,
                                       _encode_sts_speed(int(cps)))
            if log_cm is not None and i % 4 == 0:
                cmd = _zero_pose()
                for j in yaw_joints:
                    sign = 1.0 if (j // 3) % 2 == 0 else -1.0
                    cmd[j] = sign * SHIMMY_V_AMP_DEG * math.sin(omega * t)
                log_cm.sample(bus, cmd, wrote={})
            tick_i += 1
            sleep_for = (t0 + tick_i * SHIMMY_V_DT) - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
    finally:
        if log_cm is not None:
            try:
                log_cm.__exit__(None, None, None)
            except Exception:
                pass
        print("  Restoring position mode on yaw servos …")
        _wheel_stop(bus, yaw_sids)
        time.sleep(0.05)
        for sid in yaw_sids:
            try:
                _set_servo_mode(bus, sid, 0)
            except Exception as e:
                print(f"    mode restore failed id={sid}: {e}")
        _set_torque_limit(bus, live, 1000)
        try:
            if status != "aborted" and not check():
                print("  Easing back to zero …")
                go_to_zero_pose(bus, abort_check=check, seconds=2.0)
                _limp_all(bus, live)
            else:
                try:
                    _hold_here(bus, live)
                except Exception:
                    pass
                _limp_all(bus, live)
        except Exception as e:
            print(f"  cleanup: {e}")
            try:
                _limp_all(bus, live)
            except Exception:
                pass
    return status


def pose_heartbeat(t: float) -> list[float]:
    """Double-thump pulse on all knees (from zero)."""
    pose = _zero_pose()
    cycle = (t % 1.1) / 1.1
    if cycle < 0.12:
        k = 12.0 * math.sin(math.pi * cycle / 0.12)
    elif 0.18 < cycle < 0.30:
        k = 8.0 * math.sin(math.pi * (cycle - 0.18) / 0.12)
    else:
        k = 0.0
    for leg in range(6):
        _yaw_hip_knee(leg, pose, knee=k, hip=-0.5 * k)
    return pose


def frames_heartbeat(seconds: float = 6.0):
    for i in range(max(1, int(seconds / DT))):
        yield pose_heartbeat(i * DT)


def make_pose_twinkle():
    """Small independent wiggles — looks alive in the air.

    Factory: each run gets its own random phases/frequencies.
    """
    phases = [random.random() * 2 * math.pi for _ in range(N_JOINTS)]
    freqs = [0.35 + 0.25 * random.random() for _ in range(N_JOINTS)]

    def pose_fn(t: float) -> list[float]:
        pose = _zero_pose()
        for leg in range(6):
            j0 = leg * 3
            yaw = 6.0 * math.sin(2 * math.pi * freqs[j0] * t + phases[j0])
            hip = 4.0 * math.sin(2 * math.pi * freqs[j0 + 1] * t
                                 + phases[j0 + 1])
            knee = 5.0 * math.sin(2 * math.pi * freqs[j0 + 2] * t
                                  + phases[j0 + 2])
            _yaw_hip_knee(leg, pose, yaw=yaw, hip=hip, knee=knee)
        return pose

    return pose_fn


def frames_twinkle(seconds: float = 6.0):
    fn = make_pose_twinkle()
    for i in range(max(1, int(seconds / DT))):
        yield fn(i * DT)


def pose_conductor(t: float) -> list[float]:
    """One leg waves; others hold near zero."""
    pose = _zero_pose()
    pointer = int((t * 0.4) % 6)
    for leg in range(6):
        if leg == pointer:
            yaw = 12.0 * math.sin(2 * math.pi * 0.7 * t)
            hip = -5.0 * abs(math.sin(2 * math.pi * 0.7 * t))
            _yaw_hip_knee(leg, pose, yaw=yaw, hip=hip)
        elif abs(leg - pointer) % 6 in (1, 5):
            _yaw_hip_knee(leg, pose, yaw=-3.0)
    return pose


def frames_conductor(seconds: float = 8.0):
    for i in range(max(1, int(seconds / DT))):
        yield pose_conductor(i * DT)


def run_arms_up_demo(bus: FeetechBus, *,
                     abort_check=None,
                     speed: float = 1.0,
                     seconds: float | None = None,
                     torque: int | None = None,
                     log_path: Path | None = None) -> str:
    """Sit: fold all six legs way overhead, hold, return to sit zero + limp."""
    sc = _speed_scale(speed)
    up_s = (float(seconds) if seconds is not None else ARMS_UP_SECONDS) * (
        1.0 if seconds is not None else sc)
    hold_s = ARMS_UP_HOLD_S * sc
    down_s = max(1.6, up_s * 0.85)

    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    try:
        tlim = int(torque) if torque is not None else DEMO_TORQUE_LIMIT
    except (TypeError, ValueError):
        tlim = DEMO_TORQUE_LIMIT
    tlim = max(150, min(1000, tlim))
    _set_torque_limit(bus, live, tlim)

    print(f"  arms_up — sit → overhead (hip {ARMS_UP_HIP_DEG:.0f}° / "
          f"knee {ARMS_UP_KNEE_DEG:.0f}°) over ~{up_s:.1f}s")
    log_cm = MotionLog(log_path, live) if log_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        if not ease_to_pose(bus, _arms_up_pose(), abort_check=check,
                            seconds=up_s, label="arms way up", log=log_cm):
            _set_torque_limit(bus, live, 1000)
            return "aborted"

        signal = _wait_for_descend(bus, live, check, peaks=None,
                                   auto_hold_s=hold_s)
        if signal == "aborted":
            _set_torque_limit(bus, live, 1000)
            return "aborted"

        print("  Returning to sit zero …")
        descend_check = check
        if _abort_is_latched(check) and check():
            descend_check = (lambda: False)
        if not go_to_zero_pose(bus, abort_check=descend_check, seconds=down_s):
            _set_torque_limit(bus, live, 1000)
            return "aborted"
        _limp_all(bus, live)
        _set_torque_limit(bus, live, 1000)
        return "done"
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)


def run_stand_hands_demo(bus: FeetechBus, *,
                         abort_check=None,
                         speed: float = 1.0,
                         seconds: float | None = None,
                         legs: tuple[int, ...] = STAND_HANDS_LEGS) -> str:
    """Stand: raise three legs overhead (tripod support), return to stand."""
    sc = _speed_scale(speed)
    up_s = (float(seconds) if seconds is not None else STAND_HANDS_SECONDS) * (
        1.0 if seconds is not None else sc)
    hold_s = STAND_HANDS_HOLD_S * sc
    down_s = max(1.4, up_s * 0.9)

    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    peaks = CurrentPeakTracker()
    peak_path = default_log_path("stand_hands_current_peaks")

    _enable_torque(bus, live)
    _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
    print(f"  stand_hands — stand → lift legs {list(legs)} overhead "
          f"(hip {STAND_HANDS_LIFT_HIP_DEG:.0f}° / "
          f"knee {STAND_HANDS_LIFT_KNEE_DEG:.0f}°)")

    # Make sure we're on stand plant before raising (supports the body).
    if not go_to_stand_pose(bus, abort_check=check, seconds=max(1.2, up_s * 0.7),
                            current_tracker=peaks):
        peaks.write_log(peak_path, phase="stand_hands (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    hands = _stand_hands_pose(legs)
    if not ease_to_pose(bus, hands, abort_check=check, seconds=up_s,
                        label="three arms up", current_tracker=peaks):
        peaks.write_log(peak_path, phase="stand_hands (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    return _finish_planted_demo(bus, live, check, peaks, peak_path,
                                down_s, "stand_hands")


# ---------------------------------------------------------------------------
# Standing dances — streamed offsets around the LIVE captured plant.
#
# Built for the redone demos page (2026-08-17): they home via the
# validated keyframe stand-up (the experiments-page 10x technique), then
# dance AROUND standing_pose_degrees() — the stance the stand-up actually
# ends on — instead of yanking to the old hip+20/knee+80 display stilts.
# All are zero-mean / cyclic, cord-safe (yaw averages 0), and run through
# stream_pose_fn so the web speed slider works live.
#
# FK note (foot drop ≈ FEMUR*sin(hip) + TIBIA*sin(knee) mm): near the
# plant, both hip and knee affect height; bounce/sway still mostly modulate
# hips because they move the body without making the boot scrape as much.
# ---------------------------------------------------------------------------
STAND_SWAY_HZ = 0.35
STAND_SWAY_HIP_DEG = 5.0
STAND_BOUNCE_HZ = 0.8
STAND_BOUNCE_HIP_DEG = 7.0
STAND_TWIST_HZ = 0.45
STAND_TWIST_YAW_DEG = 10.0
STAND_WAVE_LEG_S = 3.0        # seconds per leg (demo time)
STAND_WAVE_LIFT_HIP_DEG = -12.0
STAND_WAVE_LIFT_KNEE_DEG = -30.0
STAND_WAVE_YAW_DEG = 12.0
STAND_RIPPLE_HZ = 0.45
STAND_RIPPLE_LIFT_HIP_DEG = -10.0
STAND_RIPPLE_LIFT_KNEE_DEG = -24.0


def make_stand_pose_fn(name: str):
    """Pose function for one standing dance, anchored to the live plant."""
    base = _stand_zero_pose()
    two_pi = 2.0 * math.pi

    def _with_offsets(offs) -> list[float]:
        pose = list(base)
        for leg in range(6):
            yaw, hip, knee = offs(leg)
            _yaw_hip_knee(leg, pose, yaw=yaw, hip=hip, knee=knee)
        return pose

    if name == "stand_sway":
        # Traveling crouch wave → the body leans in a slow circle.
        def fn(t: float) -> list[float]:
            w = two_pi * STAND_SWAY_HZ * t
            return _with_offsets(lambda leg: (
                0.0,
                STAND_SWAY_HIP_DEG * math.sin(w + leg * math.pi / 3.0),
                -0.5 * STAND_SWAY_HIP_DEG
                * math.sin(w + leg * math.pi / 3.0)))
        return fn
    if name == "stand_bounce":
        # Raised-cosine squat bob: starts and ends at the plant.
        def fn(t: float) -> list[float]:
            crouch = 0.5 * (1.0 - math.cos(two_pi * STAND_BOUNCE_HZ * t))
            return _with_offsets(lambda leg: (
                0.0,
                -STAND_BOUNCE_HIP_DEG * crouch,
                0.5 * STAND_BOUNCE_HIP_DEG * crouch))
        return fn
    if name == "stand_twist":
        # In-place body twist; zero-mean so cords never wind.
        def fn(t: float) -> list[float]:
            yaw = STAND_TWIST_YAW_DEG * math.sin(two_pi * STAND_TWIST_HZ * t)
            return _with_offsets(lambda leg: (yaw, 0.0, 0.0))
        return fn
    if name == "stand_wave":
        # One leg at a time lifts off the plant and waves, then sets
        # down; the lift envelope is 0 at every leg handoff (smooth).
        def fn(t: float) -> list[float]:
            k = int(t / STAND_WAVE_LEG_S) % 6
            u = (t % STAND_WAVE_LEG_S) / STAND_WAVE_LEG_S
            env = math.sin(math.pi * u) ** 2
            wave = math.sin(two_pi * 1.5 * t)
            return _with_offsets(lambda leg: (
                STAND_WAVE_YAW_DEG * env * wave if leg == k else 0.0,
                STAND_WAVE_LIFT_HIP_DEG * env if leg == k else 0.0,
                STAND_WAVE_LIFT_KNEE_DEG * env if leg == k else 0.0))
        return fn
    if name == "stand_ripple":
        # Traveling lift bump around the hex — sharp enough that only
        # ~one leg is meaningfully off the ground at a time.
        def fn(t: float) -> list[float]:
            w = two_pi * STAND_RIPPLE_HZ * t

            def offs(leg: int):
                env = max(0.0, math.sin(w - leg * math.pi / 3.0)) ** 3
                return (0.0,
                        STAND_RIPPLE_LIFT_HIP_DEG * env,
                        STAND_RIPPLE_LIFT_KNEE_DEG * env)
            return _with_offsets(offs)
        return fn
    raise SystemExit(f"unknown standing dance {name!r}")


# Streamed demos (live speed): standing dances + the air wiggles.
STAND_STREAM_DEMOS = ("stand_sway", "stand_bounce", "stand_twist",
                      "stand_wave", "stand_ripple")
# Quad mode: weight-bearing like the stand dances (τ900 + stall guard,
# home = stand for rear-up, then "quad" for walk/down). These are split
# into operator-sized primitives: rear up and hold; walk/trot forward or
# backward while reared; come down on request.
QUAD_VARIANTS = {
    "_safe": ("rear_safe", "walk_safe", "trot_safe", "safe"),
    "": ("rear", "walk", "trot", "cool"),
    "_pitch": ("rear_pitch", "walk_pitch", "trot_pitch", "pitched"),
    "_aft": ("rear_aft", "walk_aft", "trot_aft", "aft-shift"),
    "_high": ("rear_high", "walk_high", "trot_high", "high-body"),
    "_step": ("rear_step", "walk_step", "trot_step", "high-step"),
    "_aggressive": (
        "rear_aggressive", "walk_aggressive", "trot_aggressive",
        "aggressive"),
}


def _quad_name(action: str, suffix: str) -> str:
    return f"quad_{action}{suffix}"


QUAD_REAR_DEMOS = tuple(
    _quad_name("rear", suffix) for suffix in QUAD_VARIANTS)
QUAD_DOWN_DEMOS = tuple(
    _quad_name("down", suffix) for suffix in QUAD_VARIANTS)
QUAD_REARED_END_DEMOS = tuple(
    _quad_name(action, suffix)
    for suffix in QUAD_VARIANTS
    for action in ("rear", "hold", "walk", "walk_back",
                   "trot", "trot_back"))
QUAD_REQUIRES_REAR = tuple(
    _quad_name(action, suffix)
    for suffix in QUAD_VARIANTS
    for action in ("hold", "walk", "walk_back", "trot",
                   "trot_back", "down"))
QUAD_STREAM_DEMOS = (*QUAD_REARED_END_DEMOS, *QUAD_DOWN_DEMOS)
QUAD_BALANCE_TRIM_DEMOS = tuple(
    _quad_name(action, suffix)
    for suffix in QUAD_VARIANTS
    for action in ("hold", "walk", "walk_back", "trot", "trot_back"))
QUAD_BLOCKED_HARDWARE_DEMOS = tuple(
    _quad_name(action, "_aggressive")
    for action in ("walk", "walk_back", "trot", "trot_back"))
QUAD_DEMO_GAITS = {}
for _quad_suffix, (_rear_gait, _walk_gait, _trot_gait, _label) in (
        QUAD_VARIANTS.items()):
    QUAD_DEMO_GAITS[_quad_name("rear", _quad_suffix)] = _rear_gait
    QUAD_DEMO_GAITS[_quad_name("hold", _quad_suffix)] = _rear_gait
    QUAD_DEMO_GAITS[_quad_name("down", _quad_suffix)] = _rear_gait
    QUAD_DEMO_GAITS[_quad_name("walk", _quad_suffix)] = _walk_gait
    QUAD_DEMO_GAITS[_quad_name("walk_back", _quad_suffix)] = _walk_gait
    QUAD_DEMO_GAITS[_quad_name("trot", _quad_suffix)] = _trot_gait
    QUAD_DEMO_GAITS[_quad_name("trot_back", _quad_suffix)] = _trot_gait


def _make_quad_fn(seconds: float, *, gait: str = "walk",
                  direction: float = 1.0, phase: str = "walk",
                  trim_fn=None):
    from quad_walk import make_quad_walk_pose_fn
    return make_quad_walk_pose_fn(
        _stand_zero_pose(), seconds, gait=gait, direction=direction,
        phase=phase, trim_fn=trim_fn)


def _make_quad_variant_fn(action: str, suffix: str):
    rear_gait, walk_gait, trot_gait, _label = QUAD_VARIANTS[suffix]
    if action in ("rear", "hold", "down"):
        gait = rear_gait
    elif action in ("walk", "walk_back"):
        gait = walk_gait
    else:
        gait = trot_gait
    phase = ("rear" if action == "rear"
             else "hold" if action == "hold"
             else "down" if action == "down"
             else "walk")
    direction = -1.0 if action in ("walk_back", "trot_back") else 1.0

    def _fn(seconds: float, trim_fn=None):
        return _make_quad_fn(
            seconds, gait=gait, direction=direction, phase=phase,
            trim_fn=trim_fn)

    _fn.duration_aware = True
    return _fn


QUAD_STREAM_FACTORIES = {
    _quad_name(action, suffix): _make_quad_variant_fn(action, suffix)
    for suffix in QUAD_VARIANTS
    for action in ("rear", "hold", "walk", "walk_back",
                   "trot", "trot_back", "down")
}

STREAM_POSE_FACTORIES = {
    "shimmy": lambda: pose_shimmy,
    "ripple": lambda: pose_ripple,
    "heartbeat": lambda: pose_heartbeat,
    "conductor": lambda: pose_conductor,
    "twinkle": make_pose_twinkle,
    **{n: (lambda n=n: make_stand_pose_fn(n)) for n in STAND_STREAM_DEMOS},
    **QUAD_STREAM_FACTORIES,
}
# Default demo-time duration for standing dances (CLI; web sends its own).
STAND_STREAM_SECONDS = 20.0
QUAD_STREAM_SECONDS = 40.0
STREAM_SECONDS_MAX = 300.0


def run_streamed_demo(bus: FeetechBus, name: str, *,
                      seconds: float | None = None,
                      speed: float = 1.0,
                      speed_fn=None,
                      torque: int | None = None,
                      abort_check=None,
                      status_cb=None,
                      log_path: Path | None = None) -> str:
    """One streamed demo (stand dance or air wiggle) with live tempo."""
    if name not in STREAM_POSE_FACTORIES:
        raise SystemExit(f"unknown streamed demo {name!r}")
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 robot IDs answering; animating "
              f"those only: {sorted(live)}")
    check = abort_check or (lambda: False)
    if speed_fn is None:
        spd0 = _clamp_live_speed(speed)
        speed_fn = lambda: spd0  # noqa: E731
    if name in QUAD_DEMO_GAITS:
        # Per-gait speed caps are hardware prudence: trot and the more
        # pitched variants use fewer support margins than the stable walk.
        from quad_walk import GAITS
        cap = GAITS.get(QUAD_DEMO_GAITS[name], {}).get("speed_cap")
        if cap is not None:
            base_fn = speed_fn
            speed_fn = lambda: min(cap, base_fn())  # noqa: E731
    quad = name in QUAD_STREAM_DEMOS
    standing = name in STAND_STREAM_DEMOS or quad
    if name in QUAD_DOWN_DEMOS:
        from quad_walk import EXIT_TOTAL_S
        dur = EXIT_TOTAL_S
    elif seconds is None:
        dur = (QUAD_STREAM_SECONDS if quad
               else STAND_STREAM_SECONDS if standing
               else AIR_DEMO_SECONDS.get(name, 7.0))
    else:
        dur = float(seconds)
    dur = max(2.0, min(STREAM_SECONDS_MAX, dur))
    if quad:
        from quad_walk import ENTRY_TOTAL_S, MIN_SECONDS
        if name in QUAD_REAR_DEMOS:
            # Entry choreography needs room before the hold phase.
            dur = max(dur, ENTRY_TOTAL_S + 0.5)
        elif name in QUAD_DOWN_DEMOS:
            pass
        elif name not in QUAD_REQUIRES_REAR:
            # Legacy/full timelines need entry + exit + >= one gait cycle.
            dur = max(dur, MIN_SECONDS)

    _enable_torque(bus, live)
    if standing:
        # Weight-bearing motion: near-full torque (knees buckle at the
        # soft demo limit); the stall-fight guard is the safety net.
        tlim = STAND_DANCE_TORQUE
    else:
        try:
            tlim = int(torque) if torque is not None else DEMO_TORQUE_LIMIT
        except (TypeError, ValueError):
            tlim = DEMO_TORQUE_LIMIT
        tlim = max(150, min(1000, tlim))
    _set_torque_limit(bus, live, tlim)

    balance_trim = None
    if quad and name in QUAD_BALANCE_TRIM_DEMOS:
        from quad_walk import GAITS
        gait_key = QUAD_DEMO_GAITS.get(name)
        gait_cfg = GAITS.get(gait_key or "", {})
        pitch = gait_cfg.get("pitch")
        expected_pitch = (
            math.degrees(float(pitch)) if pitch is not None else None)
        balance_trim = QuadPitchTrim(
            expected_pitch_deg=expected_pitch, gait=gait_key or name)

    factory = STREAM_POSE_FACTORIES[name]
    if getattr(factory, "duration_aware", False):
        if quad:
            pose_fn = factory(
                dur,
                trim_fn=balance_trim.pose_trim if balance_trim else None)
        else:
            pose_fn = factory(dur)
    else:
        pose_fn = factory()
    tracker = CurrentPeakTracker()
    tick_s = QUAD_STREAM_TICK_S if quad else STREAM_TICK_S
    title = DEMOS[name][0] if name in DEMOS else name
    print(f"  {name} — {title}")
    print(f"    streamed @ ~{1.0 / tick_s:.0f} Hz for ~{dur:.0f}s "
          f"(live speed x{_clamp_live_speed(speed_fn()):.2f}) τ{tlim}")
    print("  Any key aborts.")

    log_cm = MotionLog(log_path, live) if log_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        st = stream_pose_fn(
            bus, live, pose_fn, seconds=dur, abort_check=check,
            speed_fn=speed_fn, status_cb=status_cb, label=name,
            tracker=tracker, log=log_cm, tick_s=tick_s,
            max_acc=QUAD_STREAM_ACC if quad else 200,
            tilt_guard_deg=QUAD_TILT_GUARD_DEG if quad else None,
            balance_trim=balance_trim)
        if st == "aborted":
            return "aborted"
        if st.startswith("balance:"):
            bits = st.split(":")
            err = bits[1] if len(bits) > 1 else "?"
            rate = bits[2] if len(bits) > 2 else "?"
            recovery = (_quad_recover_to_stand(
                bus, live, check, tracker, status_cb=status_cb)
                if quad else "went limp")
            if len(bits) > 3 and bits[3] == "recovery_timeout":
                msg = (f"stopped: still tipped {err} deg after "
                       f"{QUAD_TRIM_RECOVER_TIMEOUT_S:.0f}s of paused "
                       "recovery — leaning against it did not level the "
                       f"body; {recovery}. Check the robot before the "
                       "next run.")
            else:
                msg = (f"stopped: pitch drift {err} deg, rate {rate} deg/s "
                       "— balance trim saw a fall starting; "
                       f"{recovery}. "
                       "Check the robot before the next run.")
            print(f"  {msg}")
            return msg
        if st.startswith("tilt:"):
            msg = (f"stopped: body tilt {st[5:]} deg — tipping past "
                   f"{QUAD_TILT_GUARD_DEG:.0f}, went limp for a soft "
                   f"landing. Check the robot before the next run.")
            print(f"  {msg}")
            return msg
        if st == "guard":
            if quad:
                recovery = _quad_recover_to_stand(
                    bus, live, check, tracker, status_cb=status_cb)
                msg = (f"stopped: {tracker.peak_a:.2f} A peak on joint "
                       f"{tracker.peak_joint} — stall-fight; {recovery}")
            else:
                msg = (f"stopped: {tracker.peak_a:.2f} A peak on joint "
                       f"{tracker.peak_joint} — stall-fight, holding here")
            print(f"  {msg}")
            return msg
        # Natural finish. Split quad-mode walk/rear commands intentionally
        # keep holding the reared stance; quad_down is the explicit exit.
        _set_torque_limit(bus, live, 1000)
        if name in QUAD_REARED_END_DEMOS:
            return "done"
        if standing:
            if not _soft_glide(bus, _stand_zero_pose(), live, 0.9, check,
                               max_speed=600, max_acc=60):
                return "aborted"
            return "done"   # bench keeps holding the plant (stand hold)
        print("  Demo finished — easing back to zero ...")
        if not go_to_zero_pose(bus, abort_check=check, seconds=1.5):
            return "aborted"
        _limp_all(bus, live)
        return "done"
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)
        _set_torque_limit(bus, live, 1000)


def frames_air_meet(seconds: float = 24.0):
    """Six solos that LOCK into a lineup, scatter, and lock again.

    Sitting show (no stand-up).  The act cycles: every leg churns on
    its own incommensurate frequency (the chaos vocabulary), then all
    six blend into one MEETING pose, breathe there in perfect unison
    for a beat, and dissolve back into chaos.  Each cycle meets in a
    different lineup — overhead, a wide star, an odd/even
    checkerboard, and overhead again for the finale (bigger pulse).
    The chaos clock runs on GLOBAL time so re-entry after each meet
    looks like the same storm resuming, not a restart.
    """
    # (yaw per-leg fn, hip, knee, pulse gain) per meeting pose.
    meets = (
        (lambda leg: 0.0, ARMS_UP_HIP_DEG, ARMS_UP_KNEE_DEG, 1.0),
        (lambda leg: 14.0 if leg % 2 == 0 else -14.0, -12.0, -6.0, 1.0),
        (lambda leg: 0.0, None, None, 1.0),      # checkerboard (per-leg)
        (lambda leg: 0.0, ARMS_UP_HIP_DEG, ARMS_UP_KNEE_DEG, 1.8),
    )
    n = max(1, int(seconds / DT))
    cyc = max(1, n // len(meets))
    for i in range(n):
        t = i * DT
        k = min(i // cyc, len(meets) - 1)
        u = (i - k * cyc) / max(cyc - 1, 1)      # 0..1 inside this cycle
        yaw_fn, hip_m, knee_m, gain = meets[k]
        # lock weight: chaos <0.32, blend 0.32-0.52, hold 0.52-0.8,
        # dissolve 0.8-1.0 (blend ~1.2 s at the default tempo — 0.9 s
        # peaked 121 deg/s into the overhead lineup, past the servos'
        # ~115 deg/s comfortable tracking at demo write speed).
        if u < 0.32:
            w = 0.0
        elif u < 0.52:
            w = (u - 0.32) / 0.20
        elif u < 0.80:
            w = 1.0
        else:
            w = 1.0 - (u - 0.80) / 0.20
        w = 0.5 - 0.5 * math.cos(math.pi * w)    # ease both edges
        # Unison breath while held (everyone identical = the payoff).
        pulse = gain * math.sin(2.0 * math.pi * 1.1 * t) * w
        pose = _zero_pose()
        for leg in range(6):
            j = leg * 3
            wy = 2 * math.pi * _CHAOS_FREQ[j % 6]
            wh = 2 * math.pi * _CHAOS_FREQ[(j + 1) % 6]
            wk = 2 * math.pi * _CHAOS_FREQ[(j + 2) % 6]
            yaw_c = 18.0 * math.sin(wy * t + j * _CHAOS_GOLD)
            hip_c = -22.0 - 16.0 * math.sin(wh * t + (j + 1) * _CHAOS_GOLD)
            knee_c = 6.0 + 20.0 * math.sin(wk * t + (j + 2) * _CHAOS_GOLD)
            if hip_m is None:                    # checkerboard lineup
                hip_t = -52.0 if leg % 2 else -10.0
                knee_t = 16.0 if leg % 2 else -8.0
            else:
                hip_t, knee_t = hip_m, knee_m
            _yaw_hip_knee(
                leg, pose,
                yaw=yaw_c * (1.0 - w) + yaw_fn(leg) * w,
                hip=hip_c * (1.0 - w) + (hip_t + 4.0 * pulse) * w,
                knee=knee_c * (1.0 - w) + (knee_t - 3.0 * pulse) * w)
        yield pose


def frames_air_pendulum(seconds: float = 20.0):
    """Pendulum wave — six swings drift apart and SNAP back into sync.

    Sitting show.  Leg k swings at f0 + k*df: the tiny frequency
    stagger makes the arms fan into travelling waves, dissolve into
    apparent chaos, pass through a perfect odd/even checkerboard at
    the halfway mark, and land back in full unison — the classic
    pendulum-wave illusion, nobody steers it.  Alignment period is
    seconds/2, so a run shows sync → storm → checkerboard → storm →
    sync twice.  Amplitude fades in over ~1.5 s and out over the last
    ~1.5 s so it starts and ends at the lifted base pose.
    """
    n = max(1, int(seconds / DT))
    f0 = 0.45
    df = 2.0 / max(seconds, 4.0)     # full re-sync twice per run
    for i in range(n):
        t = i * DT
        a = min(1.0, t / 1.5, max(0.0, (seconds - t) / 1.5))
        pose = _zero_pose()
        for leg in range(6):
            s = math.sin(2.0 * math.pi * (f0 + leg * df) * t)
            _yaw_hip_knee(leg, pose,
                          yaw=14.0 * a * s,
                          hip=-22.0 - 15.0 * a * s,
                          knee=6.0 + 17.0 * a * s)
        yield pose


def frames_air_orbits(seconds: float = 18.0):
    """Six independent orbits that MAGNETIZE into one, then let go.

    Sitting show.  Every foot draws the same air-circle but at its own
    golden-angle phase — six planets on private clocks.  Every ~6 s a
    "magnet" pulls all six phases onto the shared clock over ~1 s:
    suddenly one synchronized orbit, two revolutions in lockstep, then
    the phases relax back out to the golden spread.  Phase-space
    blending (not pose crossfade) keeps the circle radius constant, so
    it reads as gathering, never shrinking.
    """
    n = max(1, int(seconds / DT))
    w_orb = 2.0 * math.pi * 0.40     # 0.4 Hz orbit
    per = 6.0                        # magnet period, s
    for i in range(n):
        t = i * DT
        a = min(1.0, t / 1.2, max(0.0, (seconds - t) / 1.2))
        # magnet strength: cosine bump, up ~1s -> hold ~2s -> release ~1s
        ph = (t % per) / per
        if ph < 0.30:
            m = 0.0
        elif ph < 0.45:
            m = (ph - 0.30) / 0.15
        elif ph < 0.80:
            m = 1.0
        else:
            m = 1.0 - (ph - 0.80) / 0.20
        m = 0.5 - 0.5 * math.cos(math.pi * m)
        pose = _zero_pose()
        for leg in range(6):
            off = math.remainder(leg * _CHAOS_GOLD, 2.0 * math.pi)
            th = w_orb * t + off * (1.0 - m)
            _yaw_hip_knee(leg, pose,
                          yaw=15.0 * a * math.sin(th),
                          hip=-24.0 - 13.0 * a * math.cos(th),
                          knee=6.0 + 15.0 * a * math.cos(th - 0.7))
        yield pose


# Adjacent-pair merges ("thick arms"): legs sit 60° apart and yaw stops
# at ±35°, so two neighbours can only converge 5° each past parallel —
# their tips bottom out ~64 mm apart (MuJoCo FK, 2026-08-19; the legs
# live in vertical planes through their yaw axes, and within reach the
# planes never get closer).  With the foot boots that's ~an inch of
# daylight: a pair moving in perfect sync at max convergence reads as
# ONE thick arm from any audience distance.  ±33 keeps 2° of margin.
AIR_PAIR_YAW_DEG = 33.0


def _pair_sign(leg: int, grouping: int) -> float:
    """Yaw direction (+ = CCW) that merges ``leg`` with its partner.

    Grouping 0 pairs (0,1)(2,3)(4,5); grouping 1 pairs (1,2)(3,4)(5,0).
    Legs run CCW around the hex, so the lower leg of each pair yaws CCW
    and the higher yaws CW; grouping 1 is the exact sign flip.
    """
    s = 1.0 if leg % 2 == 0 else -1.0
    return s if grouping == 0 else -s


def _win(u: float, a: float, b: float) -> float:
    """Smoothstep 0→1 as u crosses [a, b]."""
    if u <= a:
        return 0.0
    if u >= b:
        return 1.0
    x = (u - a) / (b - a)
    return x * x * (3.0 - 2.0 * x)


# ---------------------------------------------------------------------------
# Dance primitives — composable oscillator layers (2026-08-19).
#
# One LAYER = a sinusoid on one joint channel (yaw / hip / knee) whose
# per-leg phase comes from a GROUPING (which legs move together) and a
# SPIN (does the phase travel around the hex, and which way).  Shows are
# sums of layers at different frequencies: harmonically related layers
# re-align periodically ("everything lines up" moments), incommensurate
# ones weave forever, and counter-spinning ripples interfere into
# standing-wave flashes.  Crossfade layer amplitudes to morph groupings
# mid-show.  All angles are offsets from the seated zero pose.
# ---------------------------------------------------------------------------
_DP_GROUPS = {
    "solo":     lambda leg: leg / 6.0,               # ripple around the hex
    "unison":   lambda leg: 0.0,                     # everyone together
    "oddeven":  lambda leg: 0.5 * (leg % 2),         # checkerboard
    "pairsA":   lambda leg: (leg // 2) / 3.0,        # (0,1)(2,3)(4,5)
    "pairsB":   lambda leg: (((leg + 5) % 6) // 2) / 3.0,  # (1,2)(3,4)(5,0)
    "opposite": lambda leg: (leg % 3) / 3.0,         # (0,3)(1,4)(2,5)
}


def _dp(t: float, leg: int, *, amp: float, freq: float,
        group: str = "solo", spin: float = 1.0, ph: float = 0.0) -> float:
    """One dance-primitive layer, evaluated for one leg.

    ``group`` picks which legs share a phase slot; ``spin`` scales how
    far the slots spread the phase (+1 = wave travels CCW around the
    hex, -1 = CW, 0 = whole group in perfect unison).  ``ph`` is a
    fixed phase offset in turns.
    """
    u = _DP_GROUPS[group](leg)
    return amp * math.sin(2.0 * math.pi * (freq * t - spin * u + ph))


def frames_air_weave(seconds: float = 26.0):
    """WEAVE — ripples, shimmies and bends braided at odd frequencies.

    Sitting show.  Three layers run at incommensurate frequencies so the
    pattern never quite repeats: a hip ripple traveling CCW, an odd/even
    yaw shimmy, and a knee bend that counter-ripples CW.  Halfway, the
    hip ripple's grouping crossfades from six solo slots to the three
    thick-arm pairs (the wave suddenly has three blades), then to
    opposite pairs for the finale — same layers, new symmetry.
    """
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        u = i / max(n - 1, 1)
        a = min(1.0, t / 1.5, max(0.0, (seconds - t) / 1.5))
        # grouping morph weights for the hip ripple
        w_solo = 1.0 - _win(u, 0.38, 0.48)
        w_pair = _win(u, 0.38, 0.48) - _win(u, 0.72, 0.82)
        w_opp = _win(u, 0.72, 0.82)
        pose = _zero_pose()
        for leg in range(6):
            hip = (-24.0
                   + w_solo * _dp(t, leg, amp=14.0, freq=0.45)
                   + w_pair * _dp(t, leg, amp=14.0, freq=0.45,
                                  group="pairsA")
                   + w_opp * _dp(t, leg, amp=14.0, freq=0.45,
                                 group="opposite")
                   + _dp(t, leg, amp=4.0, freq=0.13, group="unison"))
            yaw = _dp(t, leg, amp=12.0, freq=0.70, group="oddeven",
                      spin=1.0)
            knee = (8.0
                    + _dp(t, leg, amp=7.0, freq=0.31, group="unison")
                    + _dp(t, leg, amp=6.0, freq=0.57, spin=-1.0))
            _yaw_hip_knee(leg, pose, yaw=yaw * a, hip=hip * a,
                          knee=knee * a)
        yield pose


def frames_air_gearbox(seconds: float = 24.0):
    """GEARBOX — three joint layers meshed at a strict 1:2:3 gear ratio.

    Sitting show.  Hips pump in thick-arm pairs at the base rate, yaws
    shimmy in the OTHER pairing at exactly 2x, knees tick in opposite
    pairs at exactly 3x — like watching three gears mesh.  Because the
    ratios are harmonic, all three layers re-align every base period
    (~2.9 s): the whole machine visibly clunks into phase, drifts into
    complexity, and clunks back, over and over.
    """
    n = max(1, int(seconds / DT))
    f0 = 0.35
    for i in range(n):
        t = i * DT
        a = min(1.0, t / 1.5, max(0.0, (seconds - t) / 1.5))
        # the "clunk": a subtle unison dip right on each alignment beat
        # (_swarm_pulse ramps the attack — a raw exp jumps at the wrap)
        hit = _swarm_pulse((t * f0) % 1.0)
        pose = _zero_pose()
        for leg in range(6):
            hip = (-24.0
                   + _dp(t, leg, amp=15.0, freq=f0, group="pairsA")
                   - 3.0 * hit)
            yaw = _dp(t, leg, amp=11.0, freq=2.0 * f0, group="pairsB")
            knee = (8.0
                    + _dp(t, leg, amp=8.0, freq=3.0 * f0,
                          group="opposite")
                    + 4.0 * hit)
            _yaw_hip_knee(leg, pose, yaw=yaw * a, hip=hip * a,
                          knee=knee * a)
        yield pose


def frames_air_tides(seconds: float = 28.0):
    """TIDES — two waves circle the hex in opposite directions.

    Sitting show.  A hip ripple travels CCW while a yaw ripple travels
    CW at a different rate; where the crests meet, one side of the robot
    blooms while the other flattens, and the bloom itself slowly orbits
    (the interference pattern rotates at the frequency difference).
    Knees breathe odd/even underneath.  For the last quarter both waves
    collapse into opposite-pair unison — six arms snapping into a
    two-phase finale from what looked like open water.
    """
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        u = i / max(n - 1, 1)
        a = min(1.0, t / 1.5, max(0.0, (seconds - t) / 1.5))
        w = _win(u, 0.70, 0.78)          # wave → opposite-pair collapse
        pose = _zero_pose()
        for leg in range(6):
            hip = (-24.0
                   + (1.0 - w) * _dp(t, leg, amp=13.0, freq=0.40,
                                     spin=1.0)
                   + w * _dp(t, leg, amp=13.0, freq=0.40,
                             group="opposite", spin=0.0))
            yaw = ((1.0 - w) * _dp(t, leg, amp=13.0, freq=0.55,
                                   spin=-1.0)
                   + w * _dp(t, leg, amp=10.0, freq=0.55,
                             group="opposite", spin=0.0, ph=0.25))
            knee = 8.0 + _dp(t, leg, amp=9.0, freq=0.27,
                             group="oddeven")
            _yaw_hip_knee(leg, pose, yaw=yaw * a, hip=hip * a,
                          knee=knee * a)
        yield pose


def frames_air_trident(seconds: float = 34.0):
    """THREE THICK ARMS — pairs merge, swap partners, split 2 thick + 2 thin.

    Sitting show (no stand-up).  Adjacent arms yaw to max convergence
    and move in perfect sync — each pair reads as one thick arm (see
    AIR_PAIR_YAW_DEG).  Acts: pairs gather into a rolling three-arm
    trident → PARTNER SWAP (every arm sweeps 60° and re-merges with its
    other neighbour) → a bowing round, one thick arm dipping at a time
    → swap back → two pairs stay merged as slow heavy arms while legs 2
    and 5 break out as fast fluttering thin arms → all six gather
    overhead for one unison pulse and breathe back down.
    """
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        u = i / max(n - 1, 1)
        pose = _zero_pose()

        # Grouping morph: 0 = pairing A, 1 = pairing B, back to A.
        grp = _win(u, 0.25, 0.30) - _win(u, 0.46, 0.51)
        # Formation weight (pairs merged at all?) and the final gather.
        form = _win(u, 0.0, 0.09)
        gather = _win(u, 0.74, 0.82)
        fade = _win(u, 0.88, 1.0)          # overhead → flat, ends at zero
        split = _win(u, 0.51, 0.56) - _win(u, 0.71, 0.74)  # 2+2+2 act

        for leg in range(6):
            pair_a = leg // 2              # pair index in grouping A
            ph_a = 2.0 * math.pi * pair_a / 3.0
            # --- merged trident motion (rolls around the three arms) --
            sway = 2.5 * math.sin(2.0 * math.pi * 0.35 * t + ph_a)
            hip_m = -26.0 + 12.0 * math.sin(2.0 * math.pi * 0.5 * t + ph_a)
            knee_m = 10.0 - 7.0 * math.sin(2.0 * math.pi * 0.5 * t + ph_a)
            # --- bowing round (grouping B, one thick arm dips) --------
            wb = _win(u, 0.30, 0.335) - _win(u, 0.425, 0.46)
            if wb > 0.0:
                pair_b = ((leg + 5) % 6) // 2   # pair index in grouping B
                bow_t = (u - 0.30) / 0.16 * 3.0  # three dips in the act
                bump = max(0.0, math.sin(math.pi * min(1.0, max(
                    0.0, bow_t - pair_b))))
                hip_m = hip_m * (1.0 - wb) + (-34.0 + 26.0 * bump) * wb
                knee_m = knee_m * (1.0 - wb) + (8.0 + 10.0 * bump) * wb
                sway *= 1.0 - wb
            # --- partner-swap yaw: blend the SIGN, keep magnitude -----
            sgn = (_pair_sign(leg, 0) * (1.0 - grp)
                   + _pair_sign(leg, 1) * grp)
            yaw = sgn * (AIR_PAIR_YAW_DEG - 1.0 + sway)
            # During the sweep lift the arms clear (looks like a salute).
            mid = 4.0 * grp * (1.0 - grp)   # 1 at the half-swapped point
            hip = hip_m - 14.0 * mid
            knee = knee_m - 6.0 * mid

            # --- 2 thick + 2 thin: legs 2 & 5 break out ---------------
            if split > 0.0 and leg in (2, 5):
                wob = 2.0 * math.pi * 1.0 * t + leg
                yaw_s = 10.0 * math.sin(wob)
                hip_s = -44.0 + 8.0 * math.sin(wob * 1.13 + 0.8)
                knee_s = 12.0 + 9.0 * math.sin(wob + 1.1)
                yaw = yaw * (1.0 - split) + yaw_s * split
                hip = hip * (1.0 - split) + hip_s * split
                knee = knee * (1.0 - split) + knee_s * split
            elif split > 0.0:
                # Thick arms slow way down while the thin arms flutter.
                slow = math.sin(2.0 * math.pi * 0.4 * t
                                + math.pi * (leg // 3))
                hip = hip * (1.0 - split) + (-24.0 + 14.0 * slow) * split
                knee = knee * (1.0 - split) + (10.0 - 8.0 * slow) * split

            # --- final gather: everyone overhead, one unison pulse ----
            pulse = 4.0 * math.sin(2.0 * math.pi * 1.1 * t)
            yaw = yaw * (1.0 - gather)
            hip = (hip * (1.0 - gather)
                   + (ARMS_UP_HIP_DEG + pulse) * gather)
            knee = (knee * (1.0 - gather)
                    + (ARMS_UP_KNEE_DEG - 0.75 * pulse) * gather)

            w = form * (1.0 - fade)
            _yaw_hip_knee(leg, pose, yaw=yaw * w, hip=hip * w,
                          knee=knee * w)
        yield pose



# ---------------------------------------------------------------------------
# SIMULATION SWARM — a sitting air dance choreographed to the song
# (Big Thief, 4:13).  The beat grid and loudness envelope were measured
# from the actual recording (librosa beat-track on the Bandcamp master,
# 2026-08-18): ~108 BPM but LIVE — inter-beat wanders 0.53–0.58 s, so
# the 436 real beat times are baked below (delta-encoded ms) instead of
# assuming a fixed tempo.  AMP99 is the smoothed per-beat loudness
# (00–99, p10→0 / p95→99 of the song's RMS); MOVES is the per-beat
# choreography plan derived from 16-beat phrase loudness:
#   0 BREATH (quiet verses + the operator-requested open/close)
#   1 SWAY   (builds — travelling wave around the hex)
#   2 PUMP   (driving sections — checkerboard knee pumps on the beat)
#   3 BIG    (choruses + outro jam — arms overhead, downbeat punches)
# Louder song → bigger movement: every move scales its ranges with the
# live AMP envelope.  Beat 0 is a true downbeat (onset-strength test).
_SWARM_T0_MS = 186
_SWARM_BEAT_DELTAS_MS = (
    "580,604,580,580,604,580,580,580,580,580,580,580,604,580,580,557,580,"
    "580,580,580,580,580,580,580,580,557,580,580,580,580,557,580,580,580,"
    "580,580,580,580,557,580,580,580,557,580,557,580,580,580,604,557,580,"
    "557,580,557,580,580,580,580,580,580,557,580,580,557,557,580,580,580,"
    "557,557,580,557,580,557,580,557,580,557,580,580,534,557,604,580,557,"
    "580,557,604,557,534,627,580,534,557,557,557,580,557,557,580,534,580,"
    "580,557,580,580,557,580,557,557,557,557,557,580,557,580,580,557,557,"
    "580,557,557,580,557,557,557,580,557,557,557,580,580,557,580,557,557,"
    "580,557,580,557,557,580,557,557,557,580,557,557,557,557,534,557,557,"
    "580,557,557,557,557,557,557,557,580,534,557,557,557,557,534,557,580,"
    "557,580,557,557,557,557,534,557,557,557,557,557,557,604,557,580,557,"
    "557,557,557,464,650,627,557,557,580,557,557,557,557,580,557,557,557,"
    "580,557,580,557,580,557,511,580,627,580,557,557,604,557,534,557,557,"
    "580,557,580,604,580,511,557,580,557,557,580,580,557,557,557,557,557,"
    "534,580,580,557,557,580,557,580,580,557,557,580,580,580,557,557,580,"
    "580,557,580,580,557,557,557,604,534,557,557,580,557,557,580,557,604,"
    "580,580,580,557,580,557,580,557,580,557,580,557,580,557,580,557,557,"
    "557,580,557,534,557,557,580,557,557,557,557,534,580,557,580,580,557,"
    "557,557,557,557,557,557,557,557,557,557,557,557,557,580,557,534,557,"
    "580,557,580,557,580,557,580,534,580,557,534,557,557,557,557,534,580,"
    "534,557,557,580,557,557,534,557,534,557,557,580,580,557,557,557,557,"
    "534,557,580,557,580,534,557,534,557,557,580,557,557,557,557,557,604,"
    "557,557,557,604,557,580,557,557,580,580,557,580,557,580,580,557,580,"
    "557,534,604,580,534,604,580,580,557,580,580,557,580,580,580,557,580,"
    "534,580,580,557,604,557,557,580,557,604,580,557,580,557,557,580,557,"
    "557,534,580,580,580,580,557,557,557,557")
_SWARM_AMP99 = (
    "00020204041310111015060606050205041111100708010202020101000000000206"
    "06232427292918262125343531312113080817171719171009151815182215182322"
    "18202022212826211523242937453737393633343431253341485252403419161713"
    "14171926313031282930324139394544383833323133464540393936334049444347"
    "43373936424253576271747777787069697571737767616966717478717267737176"
    "72736967647370727276696958503323100501031726294751414443342927253532"
    "36424441404553515250474337455760596357483736433740434746495353515156"
    "47646161577058665968646864696054485654586469606459605665616859554544"
    "30414643454843474645504746424241352534292828332723222624333641393635"
    "39364545444546415253637179868989847969656569687883777982727370656267"
    "64687382788184858489858881818580778580828386797979776772665961584133"
    "20222129364948515261576368767177656457666779818884816669687070838081"
    "71736767616768736969707367736661453114100000000000000000")
_SWARM_MOVES = (
    "00000000000000000000000000000000111111111111111100000000000000000000"
    "00000000000011111111111111111111111111111111111111111111111111111111"
    "11111111333333333333333333333333333333331111111111111111111111111111"
    "11112222222222222222111111111111111122222222222222222222222222222222"
    "11111111111111111111111111111111222222222222222233333333333333333333"
    "33333333333333333333333333331111111111111111333333333333333333333333"
    "3333333311111111000000000000")
_SWARM_CACHE: dict = {}
_SWARM_SECTION_NOTES = {
    "0": "breathing on the beat",
    "1": "sway — the build",
    "2": "PUMP — on the beat",
    "3": "GO BIG — chorus",
}


def _swarm_tables():
    """Decode the baked song data once: times_s, amps01, moves, starts."""
    if not _SWARM_CACHE:
        t = _SWARM_T0_MS / 1000.0
        times = [t]
        for d in _SWARM_BEAT_DELTAS_MS.split(","):
            t += int(d) / 1000.0
            times.append(t)
        amps = [int(_SWARM_AMP99[i:i + 2]) / 99.0
                for i in range(0, len(_SWARM_AMP99), 2)]
        starts = [0] * len(times)
        for i in range(1, len(times)):
            starts[i] = (starts[i - 1]
                         if _SWARM_MOVES[i] == _SWARM_MOVES[i - 1] else i)
        _SWARM_CACHE.update(times=times, amps=amps,
                            moves=_SWARM_MOVES, starts=starts)
    c = _SWARM_CACHE
    return c["times"], c["amps"], c["moves"], c["starts"]


def _swarm_pulse(u: float) -> float:
    """Percussive on-beat hit with a ~0.1 s attack ramp.

    A raw exp(-k*u) decay JUMPS to 1.0 at u=0 — one 0.08 s tick of
    step, which rate-checked at 160 deg/s on the finale's hip punch.
    Easing the attack over the first 18% of the beat keeps the hit
    punchy but streamable.
    """
    if u < 0.18:
        return math.sin(0.5 * math.pi * u / 0.18)
    return math.exp(-4.0 * (u - 0.18))


def _swarm_move_pose(move: str, bi: float, idx: int, u: float,
                     a: float) -> list[float]:
    """One move vocabulary evaluated at beat-clock (bi, idx, u), amp a."""
    pose = _zero_pose()
    pulse = _swarm_pulse(u)             # percussive on-beat hit
    if move == "0":                     # BREATH — all legs as one lung
        br = 0.5 - 0.5 * math.cos(2.0 * math.pi * (bi % 4.0) / 4.0)
        hip = -(3.0 + (7.0 + 9.0 * a) * br)
        knee = 1.5 + (4.0 + 7.0 * a) * br
        knee += (1.5 + 2.5 * a) * pulse
        if idx % 4 == 0:
            hip -= 2.0 * a * pulse
        for leg in range(6):
            _yaw_hip_knee(leg, pose, yaw=0.0, hip=hip, knee=knee)
    elif move == "1":                   # SWAY — travelling wave, 8 beats
        for leg in range(6):
            ph = 2.0 * math.pi * bi / 8.0 - leg * (math.pi / 3.0)
            hip = -16.0 - (7.0 + 8.0 * a) * math.sin(ph + 0.8)
            if idx % 2 == 0:
                hip -= 2.5 * a * pulse
            _yaw_hip_knee(
                leg, pose,
                yaw=(9.0 + 8.0 * a) * math.sin(ph),
                hip=hip,
                knee=5.0 + (7.0 + 6.0 * a) * math.sin(ph + 1.6))
    elif move == "2":                   # PUMP — checkerboard on the beat
        for leg in range(6):
            sgn = math.cos(math.pi * u) * (1.0 if (idx + leg) % 2 == 0
                                           else -1.0)
            knee = 7.0 + (8.0 + 12.0 * a) * sgn
            if idx % 4 == 0:
                knee += 4.0 * a * pulse
            _yaw_hip_knee(
                leg, pose,
                yaw=(5.0 + 7.0 * a) * sgn * (1.0 if leg % 2 else -1.0),
                hip=-20.0 - 8.0 * a + 5.0 * a * sgn,
                knee=knee)
    else:                               # BIG — arms overhead, punches
        hip_base = -42.0 - 10.0 * a + 1.5 * math.sin(2.0 * math.pi * bi / 8.0)
        knee_base = 13.0 + 7.0 * a
        punch = (5.0 + 4.0 * a) * pulse if idx % 4 == 0 else 0.0
        for leg in range(6):
            bounce = ((5.0 + 5.0 * a) * pulse
                      if (idx + leg) % 2 == 0 else 0.0)
            _yaw_hip_knee(
                leg, pose,
                yaw=(7.0 + 6.0 * a) * math.sin(
                    2.0 * math.pi * bi / 4.0 + leg * _CHAOS_GOLD),
                hip=hip_base - punch,
                knee=knee_base - bounce)
    return pose


def _swarm_pose(t: float) -> list[float]:
    """Pose at song time t (s): beat-locked, loudness-scaled, blended."""
    times, amps, moves, starts = _swarm_tables()
    n = len(times)
    if t <= times[0]:
        return _swarm_move_pose("0", 0.0, 0, 1.0, amps[0])
    idx = bisect.bisect_right(times, t) - 1
    idx = min(idx, n - 1)
    nxt = times[idx + 1] if idx + 1 < n else times[idx] + 0.557
    u = min(1.0, (t - times[idx]) / max(nxt - times[idx], 1e-6))
    bi = idx + u
    a = amps[idx] + (amps[min(idx + 1, n - 1)] - amps[idx]) * u
    pose = _swarm_move_pose(moves[idx], bi, idx, u, a)
    s = starts[idx]
    if s > 0 and bi - s < 2.0:          # 2-beat crossfade between moves
        w = 0.5 - 0.5 * math.cos(math.pi * min(1.0, (bi - s) / 2.0))
        old = _swarm_move_pose(moves[s - 1], bi, idx, u, a)
        pose = [o * (1.0 - w) + p * w for o, p in zip(old, pose)]
    return pose


# Standing-version timeline (beat indices into the baked grid): sweep
# down + STEP stand-up through the build, planted show from the first
# chorus, descend as the outro cools, closing breaths seated.
_SWARM_STAND_BEAT = 112      # ~64 s — the build: stand-up window opens
_SWARM_CHORUS_BEAT = 144     # ~82 s — first chorus (planted show target)
_SWARM_DESCEND_BEAT = 416    # ~236 s — outro cools: descend to sit
_SWARM_SIT_BEAT = 424        # ~240 s — closing breaths, seated

# Seated move id → planted rise_show mode + (base, span) loudness scale.
# march for the driving sections is deliberate: every step RE-PLANTS
# three feet at stance angles, undoing the foot-skate that loaded sway
# accumulates on smooth floors (08-18 video analysis).
_SWARM_PLANT_MODE = {"0": "orbit", "1": "orbit", "2": "march",
                     "3": "tripod"}
_SWARM_PLANT_SCALE = {"0": (0.30, 0.25), "1": (0.45, 0.40),
                      "2": (0.60, 0.40), "3": (0.65, 0.35)}
_SWARM_PLANT_NOTES = {
    "0": "standing breath — weight circling softly",
    "1": "standing sway — the body orbits, on the beat",
    "2": "MARCH — re-planting feet on every beat",
    "3": "TRIPOD — three knees high, swapping on the beat",
}


def _swarm_plant_warp(mode: str, bi: float) -> float:
    """Map the beat clock onto each planted mode's internal clock so
    gesture cycles land ON the beat grid.

    tripod sin(3.0 t): full cycle (both tripods) → 4 beats
    march  sin(3.4 t): half cycle (one step)     → 1 beat
    orbit  sin(2π·0.40 t): one lean lap          → 8 beats
    """
    if mode == "tripod":
        return bi * (2.0 * math.pi / 3.0) / 4.0
    if mode == "march":
        return bi * (math.pi / 3.4)
    return bi / (8.0 * 0.40)


def _swarm_plant_pose(t: float, hip: float, knee: float) -> list[float]:
    """Planted (standing) swarm pose at song time ``t``.

    Reuses the hardware-proven rise_show vocabulary: beat-warped so the
    lifts/swaps land on beats, and blended toward the flat stance by the
    live loudness (louder song → bigger gestures).  2-beat crossfades at
    section changes, feet loaded on both sides of every blend.
    """
    times, amps, moves, starts = _swarm_tables()
    n = len(times)
    idx = min(max(bisect.bisect_right(times, t) - 1, 0), n - 1)
    nxt = times[idx + 1] if idx + 1 < n else times[idx] + 0.557
    u = min(1.0, max(0.0, (t - times[idx]) / max(nxt - times[idx], 1e-6)))
    bi = idx + u
    a = amps[idx] + (amps[min(idx + 1, n - 1)] - amps[idx]) * u
    base = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)

    def mode_pose(mid: str) -> list[float]:
        mode = _SWARM_PLANT_MODE[mid]
        lo, span = _SWARM_PLANT_SCALE[mid]
        s = min(1.0, lo + span * a)
        show = _show_stream_pose(mode, _swarm_plant_warp(mode, bi),
                                 hip, knee)
        return [b + (p - b) * s for b, p in zip(base, show)]

    pose = mode_pose(moves[idx])
    st = starts[idx]
    if st > 0 and bi - st < 2.0:
        w = 0.5 - 0.5 * math.cos(math.pi * min(1.0, (bi - st) / 2.0))
        old = mode_pose(moves[st - 1])
        pose = [o * (1.0 - w) + p * w for o, p in zip(old, pose)]
    return pose


def run_swarm_dance(bus: FeetechBus, *, abort_check=None, status_cb=None,
                    torque=None, standup_fn=None, stand: bool = False,
                    log_path: Path | None = None) -> str:
    """SIMULATION SWARM — show synced to the Big Thief song (4:07).

    Count-in: four metronome nods, one second apart — press play on the
    GO nod; the choreography clock starts there and runs on WALL time
    (stream_pose_fn accumulates true elapsed seconds), so sync never
    drifts across the four minutes.  Tempo/speed sliders are ignored —
    the recording owns the clock.  Opens and closes with the breath on
    the beat.

    ``stand=True`` (dance_swarm_stand, needs the bench ``standup_fn``):
    seated verses, then the STEP stand-up rides the build (~64 s), the
    planted rise_show vocabulary takes the choruses (tripod flips
    swapping on the beat, marches that re-plant the feet), and the
    robot sits back down as the outro fades.  Segment starts recompute
    their song offset from wall time, so a slow/fast stand-up can't
    knock the rest of the show off the beat.

    Streaming uses the stand-up lab pursuit (carrot lookahead), which
    also fixes the seated version's slow-move shudder: the old runner's
    per-tick writes fell under the 0.8° deadband during breaths, so
    joints parked and lurched ~1.4° at a time (operator report 08-18).
    """
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) — need more.")
        return "skipped"
    check = abort_check or (lambda: False)

    def note(msg: str) -> None:
        print(f"  {msg}")
        if status_cb is None:
            return
        try:
            status_cb(str(msg))
        except Exception:
            pass

    cur_note = [""]

    def _progress(msg: str) -> None:
        if status_cb is None:
            return
        try:
            status_cb(f"{cur_note[0]} · {msg}" if cur_note[0] else str(msg))
        except Exception:
            pass

    if stand and standup_fn is None:
        note("no bench stand-up available — playing the seated version")
        stand = False

    _enable_torque(bus, live)
    try:
        tlim = int(torque) if torque is not None else DEMO_TORQUE_LIMIT
    except (TypeError, ValueError):
        tlim = DEMO_TORQUE_LIMIT
    tlim = max(150, min(1000, tlim))
    _set_torque_limit(bus, live, tlim)

    times, amps, moves, starts = _swarm_tables()
    t_end = times[-1] + 0.6
    hip, knee = RISE_HIGH_HIP_DEG, RISE_HIGH_KNEE_DEG
    peaks = CurrentPeakTracker()

    def bail(label: str) -> str:
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return f"error: {label}"

    log_cm = MotionLog(log_path, live) if log_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        # --- count-in: 3, 2, 1, GO ------------------------------------
        cnt_labels = ("count-in: 3", "count-in: 2", "count-in: 1",
                      "GO — PRESS PLAY NOW ♪")
        cnt_last = [-1]

        def nod(t: float) -> list[float]:
            i = min(3, int(t))
            if i != cnt_last[0]:
                cnt_last[0] = i
                note(cnt_labels[i])
            pose = _zero_pose()
            k = 5.0 * _swarm_pulse(t % 1.0)
            for leg in range(6):
                _yaw_hip_knee(leg, pose, knee=k)
            return pose

        st = stream_pose_fn(bus, live, nod, seconds=4.0,
                            abort_check=check, speed_fn=lambda: 1.0,
                            status_cb=None, label="count-in",
                            tracker=peaks)
        if st == "aborted":
            _set_torque_limit(bus, live, 1000)
            return "aborted"
        if st == "guard":
            return bail("count-in guard")
        t0_wall = time.monotonic()

        def segment(pose_at, t_until: float, label: str, *,
                    max_speed: int = 3000, max_acc: int = 200) -> str:
            """Stream pose_at(song_t) until song time ``t_until``.

            The song offset is recomputed from WALL time at entry, so
            variable-length acts (stand-up, descend) between segments
            never accumulate schedule error.
            """
            t_off = time.monotonic() - t0_wall
            secs = t_until - t_off
            if secs <= 0.05:
                return "done"
            return stream_pose_fn(
                bus, live, lambda tl: pose_at(t_off + tl),
                seconds=secs, abort_check=check, speed_fn=lambda: 1.0,
                status_cb=_progress, label=label, tracker=peaks,
                log=log_cm, max_speed=max_speed, max_acc=max_acc)

        def seated_pose(t: float) -> list[float]:
            idx = min(bisect.bisect_right(times, t) - 1, len(times) - 1)
            if idx >= 0:
                m = _SWARM_SECTION_NOTES.get(moves[idx], "")
                if m != cur_note[0]:
                    cur_note[0] = m
                    note(m)
            return _swarm_pose(t)

        def planted_pose(t: float) -> list[float]:
            idx = min(max(bisect.bisect_right(times, t) - 1, 0),
                      len(times) - 1)
            m = _SWARM_PLANT_NOTES.get(moves[idx], "")
            if m != cur_note[0]:
                cur_note[0] = m
                note(m)
            return _swarm_plant_pose(t, hip, knee)

        if not stand:
            st = segment(seated_pose, t_end, "swarm")
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail("current guard tripped")
        else:
            st = segment(seated_pose, times[_SWARM_STAND_BEAT],
                         "swarm (seated)")
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail("current guard (seated)")

            # The build: sweep to zero, STEP stand-up, set the stance.
            # Same recipe as the dance's act IV — a refused stand-up
            # ENDS the show (no improvised blends from unknown poses).
            cur_note[0] = ""
            note("the build — sweep down, STEP stand-up")
            if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                                seconds=1.2, label="swarm sweep down",
                                current_tracker=peaks):
                return bail("sweep down")
            _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
            ok, err = standup_fn()
            if check():
                return bail("stand-up aborted")
            if not ok:
                note(f"stand-up stopped: {err}")
                return bail(f"stand-up: {err}")
            _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
            planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
            if not ease_to_pose(bus, planted, abort_check=check,
                                seconds=0.7, label="swarm set stance",
                                current_tracker=peaks):
                return bail("set stance")
            _set_torque_limit(bus, live, DANCE_PLANT_TORQUE)

            # Planted show — groove rides in on whatever's left of the
            # build, choruses land on the baked beat plan. Speed capped
            # at the rise_show stream profile (900 counts): the show
            # modes' power-law lift onsets are STEEP in pure pose math
            # (~350 deg/s for one tick) and rely on this clamp for the
            # servo-side smoothing the planted acts were proven with.
            st = segment(planted_pose, times[_SWARM_DESCEND_BEAT],
                         "swarm (standing)", max_speed=900, max_acc=80)
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail("current guard (standing)")

            # Outro: descend to sit inside the cool-down window.
            cur_note[0] = ""
            note("the fade — coming back down to sit")
            elapsed = time.monotonic() - t0_wall
            desc_s = max(3.5, min(6.0, times[_SWARM_SIT_BEAT] - elapsed))
            if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                                seconds=desc_s, label="swarm descend",
                                current_tracker=peaks):
                return bail("descend")
            _set_torque_limit(bus, live, tlim)

            st = segment(seated_pose, t_end, "swarm (goodnight)")
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail("current guard (goodnight)")
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)

    peaks.print_report(phase="swarm")
    note("song over — breathing out")
    if not go_to_zero_pose(bus, abort_check=check, seconds=2.5):
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    _limp_all(bus, live)
    _set_torque_limit(bus, live, 1000)
    return "done"


# Steeple show: which adjacent pair lifts, in playing order (front,
# back-left, back-right — merged azimuths ≈ −5°, +115°, −125°).
STEEPLE_PAIRS = ((5, 0), (1, 2), (3, 4))
STEEPLE_LIFT_HIP = -62.0
STEEPLE_LIFT_KNEE = 28.0
STEEPLE_ACT_S = 8.0
# Leg mount azimuths (deg, CCW from +x — MuJoCo FK 2026-08-19).
_LEG_AZ_DEG = (25.5, 85.5, 145.5, -154.5, -94.5, -34.5)
# Weight shift while a pair is up: the four planted legs yaw their feet
# TOWARD the lifted side, so with the feet pinned by friction the body
# slides away from the open edge.  Without it the CoM sits ON the
# tipping edge at full lift (MuJoCo statics: −2 mm margin — the edge
# between the two legs flanking the lifted pair passes through the body
# center, and the merged arm's mass leans over it); 18° of yaw plus the
# near-vertical lift pose buys ~+19 mm of margin.
STEEPLE_SHIFT_YAW_DEG = 18.0


def _steeple_act_pose(pair: tuple[int, int], t: float,
                      hip: float, knee: float) -> list[float]:
    """One steeple act: lift the pair, merge overhead, sway, replant.

    ``t`` runs 0..STEEPLE_ACT_S.  The four planted legs stay at the
    stance (hip/knee) with a micro-bounce while the merged arm is up.
    Lifting two adjacent legs leaves a wide 4-foot support polygon —
    strictly easier than the rise_show tripod lifts already proven on
    hardware.
    """
    u = t / STEEPLE_ACT_S
    lift = _win(u, 0.0, 0.20) - _win(u, 0.78, 1.0)
    # Weight shift LEADS the lift and releases after the replant, so the
    # margin is already bought while the pair is only half airborne.
    shift_env = _win(u, 0.0, 0.12) - _win(u, 0.86, 1.0)
    sway = math.sin(2.0 * math.pi * 0.5 * (t - 1.6)) * _win(u, 0.22, 0.30) \
        * (1.0 - _win(u, 0.70, 0.78))
    pose = _zero_pose()
    a, b = pair
    az_up = _LEG_AZ_DEG[a] + 30.0            # merged-arm azimuth
    for leg in range(6):
        if leg in (a, b):
            sgn = 1.0 if leg == a else -1.0   # a yaws CCW into b
            yaw_l = sgn * AIR_PAIR_YAW_DEG + 2.0 * sway
            hip_l = STEEPLE_LIFT_HIP + 5.0 * sway
            knee_l = STEEPLE_LIFT_KNEE - 4.0 * sway
            _yaw_hip_knee(leg, pose,
                          yaw=yaw_l * lift,
                          hip=hip * (1.0 - lift) + hip_l * lift,
                          knee=knee * (1.0 - lift) + knee_l * lift)
        else:
            # Weight shift: swing this foot toward the lifted side.
            diff = math.remainder(az_up - _LEG_AZ_DEG[leg], 360.0)
            shift = math.copysign(STEEPLE_SHIFT_YAW_DEG, diff) * shift_env
            bounce = 2.5 * math.sin(2.0 * math.pi * 0.9 * t) * lift
            _yaw_hip_knee(leg, pose, yaw=shift, hip=hip,
                          knee=knee + bounce)
    return pose


def run_steeple_dance(bus: FeetechBus, *, abort_check=None, status_cb=None,
                      torque=None, speed: float = 1.0, speed_fn=None,
                      standup_fn=None, log_path: Path | None = None) -> str:
    """THE STEEPLE — stand up, then adjacent arms merge overhead in turns.

    Seated teaser (pairs flash the three-thick-arms pose), STEP stand-up
    (bench ``standup_fn`` — same recipe as the dance's act IV), then
    each adjacent pair takes a turn: lift, converge to the ±33° merge
    (one thick arm pointing front / back-left / back-right), sway as a
    unit while the four planted legs micro-bounce, replant with a stomp
    accent.  A traveling yaw wave closes the show before the descend.

    Planted segments stream at the rise_show clamp (900 counts ≈
    79 deg/s) with the stall-fight current guard; a refused stand-up
    ends the show, per the incident rules.
    """
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    spd = speed_fn or (lambda: _clamp_demo_speed(speed))

    def note(msg: str) -> None:
        print(f"  {msg}")
        if status_cb is not None:
            try:
                status_cb(str(msg))
            except Exception:
                pass

    if standup_fn is None:
        # run_demo degrades to the seated trident before getting here.
        note("no bench stand-up available — skipping")
        return "skipped"

    hip, knee = RISE_HIGH_HIP_DEG, RISE_HIGH_KNEE_DEG
    peaks = CurrentPeakTracker()
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, DEMO_TORQUE_LIMIT)

    def bail(label: str) -> str:
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return f"error: {label}"

    log_cm = MotionLog(log_path, live) if log_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        # --- seated teaser: breath, then the pairs flash the merge ----
        note("teaser — the arms find their partners")

        def teaser(t: float) -> list[float]:
            w = _win(t, 1.2, 2.6) - _win(t, 4.4, 5.8)
            pose = _zero_pose()
            for leg in range(6):
                br = 6.0 * math.sin(2.0 * math.pi * 0.45 * t)
                _yaw_hip_knee(
                    leg, pose,
                    yaw=_pair_sign(leg, 0) * AIR_PAIR_YAW_DEG * w,
                    hip=-4.0 + br * 0.4 + (-22.0 - br * 0.4) * w,
                    knee=(-0.6 * br) * (1.0 - w) + (10.0 + br) * w)
            return pose

        st = stream_pose_fn(bus, live, teaser, seconds=6.0,
                            abort_check=check, speed_fn=spd,
                            status_cb=status_cb, label="steeple teaser",
                            tracker=peaks, log=log_cm)
        if st == "aborted":
            _set_torque_limit(bus, live, 1000)
            return "aborted"
        if st == "guard":
            return bail("teaser guard")

        # --- STEP stand-up (same recipe as the dance's act IV) --------
        note("STEP stand-up")
        if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                            seconds=1.2, label="steeple sweep down",
                            current_tracker=peaks):
            return bail("sweep down")
        _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
        ok, err = standup_fn()
        if check():
            return bail("stand-up aborted")
        if not ok:
            note(f"stand-up stopped: {err}")
            return bail(f"stand-up: {err}")
        _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
        planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
        if not ease_to_pose(bus, planted, abort_check=check,
                            seconds=0.7, label="steeple set stance",
                            current_tracker=peaks):
            return bail("set stance")
        _set_torque_limit(bus, live, DANCE_PLANT_TORQUE)

        # --- three steeples: front, back-left, back-right -------------
        labels = ("steeple FRONT — one thick arm",
                  "steeple BACK-LEFT",
                  "steeple BACK-RIGHT")
        for pair, lbl in zip(STEEPLE_PAIRS, labels):
            note(lbl)
            st = stream_pose_fn(
                bus, live,
                lambda t, p=pair: _steeple_act_pose(p, t, hip, knee),
                seconds=STEEPLE_ACT_S, abort_check=check, speed_fn=spd,
                status_cb=status_cb, label=lbl, tracker=peaks,
                log=log_cm, max_speed=900, max_acc=80)
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail(f"current guard ({lbl})")

            def stomp(t: float) -> list[float]:
                pose = _zero_pose()
                dip = 8.0 * math.sin(math.pi * min(1.0, t))
                for leg in range(6):
                    _yaw_hip_knee(leg, pose, hip=hip, knee=knee + dip)
                return pose

            st = stream_pose_fn(bus, live, stomp, seconds=1.0,
                                abort_check=check, speed_fn=spd,
                                status_cb=None, label="stomp",
                                tracker=peaks, log=log_cm,
                                max_speed=900, max_acc=80)
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail("current guard (stomp)")

        # --- closing wave: planted yaw ripple around the hex ----------
        note("victory wave")

        def wave(t: float) -> list[float]:
            a = min(1.0, t / 0.8, max(0.0, (4.0 - t) / 0.8))
            pose = _zero_pose()
            for leg in range(6):
                _yaw_hip_knee(
                    leg, pose,
                    yaw=8.0 * a * math.sin(
                        2.0 * math.pi * 0.7 * t - leg * math.pi / 3.0),
                    hip=hip, knee=knee)
            return pose

        st = stream_pose_fn(bus, live, wave, seconds=4.0,
                            abort_check=check, speed_fn=spd,
                            status_cb=status_cb, label="victory wave",
                            tracker=peaks, log=log_cm,
                            max_speed=900, max_acc=80)
        if st == "aborted":
            _set_torque_limit(bus, live, 1000)
            return "aborted"
        if st == "guard":
            return bail("current guard (wave)")

        # --- descend to sit -------------------------------------------
        note("coming back down")
        _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
        if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                            seconds=4.5, label="steeple descend",
                            current_tracker=peaks):
            return bail("descend")
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)

    peaks.print_report(phase="steeple")
    note("goodnight")
    _limp_all(bus, live)
    _set_torque_limit(bus, live, 1000)
    return "done"


# The stallion's sky-reach (both fronts merged overhead while reared).
STALLION_SKY_HIP = -75.0
STALLION_SKY_KNEE = 45.0
STALLION_SECONDS = 26.0


def _make_stallion_fn(seconds: float = STALLION_SECONDS):
    """Reared 'stallion': quad entry → paw strokes → unicorn merge → exit.

    Rides quad_walk's proven tip-back entry/exit with the zero-stride
    "rear" gait (all four support feet stay planted).  During the hold
    window the front paws take over: alternating air-strokes like a
    pawing horse, then both fronts converge to the ±33° pair merge
    overhead — one thick horn.  Gestures blend in/out of the tuck so
    the entry/exit choreography is untouched.
    """
    from quad_walk import (ENTRY_TOTAL_S, EXIT_TOTAL_S, TUCK_DEG,
                           make_quad_walk_pose_fn)
    quad = make_quad_walk_pose_fn(_stand_zero_pose(), seconds, gait="rear")
    t_exit = max(ENTRY_TOTAL_S, seconds - EXIT_TOTAL_S)
    hold = max(0.1, t_exit - ENTRY_TOTAL_S)
    uni0 = min(max(0.0, hold - 4.0), 4.4)     # 4 paw strokes, then merge

    def gesture(tg: float, leg: int) -> tuple[float, float, float]:
        y0, h0, k0 = TUCK_DEG
        if tg < uni0:                          # alternating paw strokes
            k = int(tg / 1.1)
            active = 5 if k % 2 == 0 else 0
            if leg == active:
                b = math.sin(math.pi * (tg / 1.1 - k))
                return (y0, h0 + 18.0 * b, k0 - 35.0 * b)
            return (float(y0), float(h0), float(k0))
        uw = _win(tg, uni0, uni0 + 1.0)        # the unicorn
        trem = 2.0 * math.sin(2.0 * math.pi * 1.8 * tg) * uw
        sgn = 1.0 if leg == 5 else -1.0
        return (sgn * AIR_PAIR_YAW_DEG * uw,
                h0 + (STALLION_SKY_HIP - h0) * uw + trem,
                k0 + (STALLION_SKY_KNEE - k0) * uw)

    def pose_at(t: float) -> list[float]:
        pose = quad(t)
        tg = t - ENTRY_TOTAL_S
        if 0.0 <= tg < hold:
            w = min(1.0, tg / 0.8, max(0.0, (hold - tg) / 1.2))
            w = 0.5 - 0.5 * math.cos(math.pi * w)
            for leg in (0, 5):
                j = leg * 3
                gy, gh, gk = gesture(tg, leg)
                pose[j] = pose[j] * (1.0 - w) + gy * w
                pose[j + 1] = pose[j + 1] * (1.0 - w) + gh * w
                pose[j + 2] = pose[j + 2] * (1.0 - w) + gk * w
        return pose

    return pose_at


def run_wild_dance(bus: FeetechBus, *, abort_check=None, status_cb=None,
                   torque=None, speed: float = 1.0, speed_fn=None,
                   standup_fn=None, log_path: Path | None = None) -> str:
    """THE FEVER DREAM — the everything show (~2 min).

    I   resurrection: flatline, one weak twitch, a heartbeat that
        strengthens and accelerates into a full-body pulse
    II  drum solo: the feet TAP THE FLOOR — audible percussion cycling
        the hex, odd/even doubles, all-six triplets, two big slams
    III the storm: primitive layers accelerating ~1.7x, hard freeze in
        the three-thick-arms merge, sweep down
    IV  fake-outs: two comedy failed stand-ups (quarter-rise, wobble,
        flop), then the STEP stand-up nails it
    V   planted rampage: march → front steeple → counterwave
    VI  THE STALLION: tips back onto four legs (quad machinery, tilt
        guard armed), paws the air, merges both fronts into one horn
        overhead, comes back down
    VII finale: victory wave, descend, overhead gather + pulse,
        collapse, one last heartbeat

    Without the bench ``standup_fn`` (CLI): plays the seated acts and
    the finale only.  Live speed slider works everywhere; the stallion
    caps it at 1.5x (quad prudence, same as the trot).
    """
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    base_spd = speed_fn or (lambda: _clamp_demo_speed(speed))

    def note(msg: str) -> None:
        print(f"  {msg}")
        if status_cb is not None:
            try:
                status_cb(str(msg))
            except Exception:
                pass

    _enable_torque(bus, live)
    _set_torque_limit(bus, live, DEMO_TORQUE_LIMIT)
    peaks = CurrentPeakTracker()

    def bail(label: str) -> str:
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return f"error: {label}"

    log_cm = MotionLog(log_path, live) if log_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        def seg(pose_fn, secs: float, label: str, *, max_speed=3000,
                max_acc=200, tick_s=STREAM_TICK_S, tilt=None,
                spd=None) -> str:
            # Fresh tracker per segment: stream_pose_fn's 4 A hard cap
            # checks the tracker's ALL-TIME peak, so a shared tracker
            # would let one hot moment insta-guard every later act.
            trk = CurrentPeakTracker()
            st = stream_pose_fn(
                bus, live, pose_fn, seconds=secs, abort_check=check,
                speed_fn=spd or base_spd, status_cb=status_cb,
                label=label, tracker=trk, log=log_cm,
                max_speed=max_speed, max_acc=max_acc, tick_s=tick_s,
                tilt_guard_deg=tilt)
            if trk.peak_a > peaks.peak_a:
                peaks.peak_a = trk.peak_a
                peaks.peak_joint = trk.peak_joint
            for j, a in trk.max_a.items():
                if a > peaks.max_a.get(j, 0.0):
                    peaks.max_a[j] = a
            peaks.samples += trk.samples
            return st

        def gate(st: str, label: str) -> str | None:
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail(f"current guard ({label})")
            if st.startswith("tilt:"):
                # stream already went limp for the soft landing
                _set_torque_limit(bus, live, 1000)
                return f"error: tilt {st[5:]} deg during {label} — went limp"
            return None

        # ---- Act I: resurrection -------------------------------------
        note("act I — flatline …")
        said = [False]

        def resurrect(t: float) -> list[float]:
            pose = _zero_pose()
            if t < 3.0:
                return pose
            if t < 6.0:
                for t0, amp in ((3.0, 4.0), (4.5, 6.0), (5.4, 6.0)):
                    if t0 <= t < t0 + 0.35:
                        pose[8] = amp * math.sin(math.pi * (t - t0) / 0.35)
                return pose
            if not said[0]:
                said[0] = True
                note("act I — it's ALIVE (heartbeat accelerating)")
            # quadratic phase: beat accelerates 0.7 -> 1.6 Hz, smoothly
            tt = t - 6.0
            ph = 0.7 * tt + 0.45 * tt * tt / 6.0
            prog = min(1.0, tt / 6.0)
            k = (5.0 + 5.5 * prog) * _swarm_pulse(ph % 1.0)
            for leg in range(6):
                _yaw_hip_knee(leg, pose, knee=k, hip=-0.45 * k * prog)
            return pose

        st = seg(resurrect, 12.0, "resurrection")
        r = gate(st, "resurrection")
        if r:
            return r

        # ---- Act II: drum solo (floor-tap percussion) ----------------
        note("act II — DRUM SOLO (feet on the floor)")

        def drums(t: float) -> list[float]:
            pose = _zero_pose()
            hover = -8.0
            w_in = _win(t, 0.2, 1.4)
            w_out = 1.0 - _win(t, 14.0, 15.0)
            tt = t - 1.4
            hit = {leg: 0.0 for leg in range(6)}
            if 0.0 <= tt < 6.0:            # singles around the hex, 2 Hz
                idx = int(tt * 2.0)
                u = tt * 2.0 - idx
                if u < 0.5:
                    hit[idx % 6] = math.sin(math.pi * u / 0.5)
            elif tt < 9.6:                 # odd/even doubles, 3 Hz
                idx = int((tt - 6.0) * 3.0)
                u = (tt - 6.0) * 3.0 - idx
                if u < 0.55:
                    b = math.sin(math.pi * u / 0.55)
                    for leg in range(idx % 2, 6, 2):
                        hit[leg] = b
            elif tt < 11.6:                # all six, triplet rhythm 4 Hz
                idx = int((tt - 9.6) * 4.0)
                u = (tt - 9.6) * 4.0 - idx
                if idx % 4 != 3 and u < 0.6:
                    b = math.sin(math.pi * u / 0.6)
                    for leg in range(6):
                        hit[leg] = b
            else:                          # two big slams
                for t0 in (12.0, 12.9):
                    if t0 <= tt < t0 + 0.5:
                        b = 1.3 * math.sin(math.pi * (tt - t0) / 0.5)
                        for leg in range(6):
                            hit[leg] = max(hit[leg], b)
            for leg in range(6):
                b = min(1.3, hit[leg])
                _yaw_hip_knee(leg, pose,
                              hip=(hover * (1.0 - b)) * w_in * w_out,
                              knee=5.0 * b * w_in * w_out)
            return pose

        st = seg(drums, 15.0, "drum solo")
        r = gate(st, "drum solo")
        if r:
            return r

        # ---- Act III: the storm --------------------------------------
        note("act III — the storm (hold on)")

        def storm(t: float) -> list[float]:
            a = min(1.0, t / 1.2)
            ph_t = t + 0.7 * t * t / 32.0     # freq ramps ~1.7x by 16 s
            g = 1.0 + 0.25 * (t / 16.0)
            fz = _win(t, 12.6, 13.8)          # freeze into the trident
            end = _win(t, 15.0, 16.0)         # sweep down
            pose = _zero_pose()
            for leg in range(6):
                hip_s = (-24.0 + _dp(ph_t, leg, amp=12.0 * g, freq=0.45)
                         + _dp(ph_t, leg, amp=5.0, freq=0.30,
                               group="unison"))
                yaw_s = _dp(ph_t, leg, amp=10.0 * g, freq=0.70,
                            group="oddeven")
                knee_s = 8.0 + _dp(ph_t, leg, amp=8.0 * g, freq=0.57,
                                   spin=-1.0)
                yaw = (yaw_s * (1.0 - fz)
                       + _pair_sign(leg, 0) * 32.0 * fz)
                hip = hip_s * (1.0 - fz) + (-26.0) * fz
                knee = knee_s * (1.0 - fz) + 10.0 * fz
                s = a * (1.0 - end)
                _yaw_hip_knee(leg, pose, yaw=yaw * s, hip=hip * s,
                              knee=knee * s)
            return pose

        st = seg(storm, 16.0, "the storm")
        r = gate(st, "the storm")
        if r:
            return r

        if standup_fn is not None:
            # ---- Act IV: fake-outs, then the real stand-up -----------
            _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
            for att, (h, k, lbl) in enumerate(
                    (( 8.0, 32.0, "act IV — attempt #1 …"),
                     (11.0, 42.0, "act IV — attempt #2 …")), 1):
                note(lbl)
                part = _elevated_stand_pose(hip=h, knee=k)
                if not ease_to_pose(bus, part, abort_check=check,
                                    seconds=1.1, label=f"fakeout {att}",
                                    current_tracker=peaks):
                    return bail(f"fakeout {att}")

                def wobble(t: float, base=part) -> list[float]:
                    q = list(base)
                    for leg in range(6):
                        q[leg * 3 + 2] += 2.5 * math.sin(
                            2.0 * math.pi * 3.0 * t + leg)
                    return q

                st = seg(wobble, 0.9, f"wobble {att}",
                         max_speed=900, max_acc=80)
                r = gate(st, f"wobble {att}")
                if r:
                    return r
                note("… nope." if att == 1 else "… NOPE.")
                if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                                    seconds=0.7, label=f"flop {att}",
                                    current_tracker=peaks):
                    return bail(f"flop {att}")
            note("act IV — third time's the charm: STEP stand-up")
            ok, err = standup_fn()
            if check():
                return bail("stand-up aborted")
            if not ok:
                note(f"stand-up stopped: {err}")
                return bail(f"stand-up: {err}")
            _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
            hip, knee = RISE_HIGH_HIP_DEG, RISE_HIGH_KNEE_DEG
            planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
            if not ease_to_pose(bus, planted, abort_check=check,
                                seconds=0.7, label="set stance",
                                current_tracker=peaks):
                return bail("set stance")
            _set_torque_limit(bus, live, DANCE_PLANT_TORQUE)

            # ---- Act V: planted rampage ------------------------------
            note("act V — rampage: the march")
            st = seg(lambda t: _show_stream_pose("march", t, hip, knee),
                     6.0, "march", max_speed=900, max_acc=80)
            r = gate(st, "march")
            if r:
                return r
            note("act V — rampage: front steeple")
            st = seg(lambda t: _steeple_act_pose((5, 0), t, hip, knee),
                     STEEPLE_ACT_S, "front steeple",
                     max_speed=900, max_acc=80)
            r = gate(st, "front steeple")
            if r:
                return r
            # No counterwave here: it yaws all six LOADED legs in
            # opposing directions — torsional lock the chassis can't
            # relieve. Measured on hardware 08-19: yaw stall-fight
            # >3 A within two sweeps of the first frame. Tripod flips
            # move knees/hips with real lifts instead.
            note("act V — rampage: tripod flips")
            st = seg(lambda t: _show_stream_pose("tripod", t, hip, knee),
                     6.0, "tripod flips", max_speed=900, max_acc=80)
            r = gate(st, "tripod flips")
            if r:
                return r

            # ---- Act VI: THE STALLION --------------------------------
            note("act VI — THE STALLION (tipping back …)")
            _set_torque_limit(bus, live, STAND_DANCE_TORQUE)
            if not ease_to_pose(bus, _stand_zero_pose(),
                                abort_check=check, seconds=0.8,
                                label="stallion base",
                                current_tracker=peaks):
                return bail("stallion base")
            stallion_spd = lambda: min(1.5, _clamp_live_speed(  # noqa: E731
                base_spd()))
            st = seg(_make_stallion_fn(STALLION_SECONDS),
                     STALLION_SECONDS, "the stallion",
                     max_acc=QUAD_STREAM_ACC, tick_s=QUAD_STREAM_TICK_S,
                     tilt=QUAD_TILT_GUARD_DEG, spd=stallion_spd)
            r = gate(st, "the stallion")
            if r:
                return r
            _set_torque_limit(bus, live, DANCE_PLANT_TORQUE)

            # ---- Act VII (standing part): wave + descend -------------
            # Knee ripple, not a yaw wave: loaded-yaw scrub stall-fights
            # (see act V note).  Ripple lifts legs — friction releases.
            note("act VII — victory ripple")
            st = seg(lambda t: _show_stream_pose("ripple", t, hip, knee),
                     4.0, "victory ripple", max_speed=900, max_acc=80)
            r = gate(st, "victory ripple")
            if r:
                return r
            note("coming back down …")
            _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
            if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                                seconds=4.5, label="descend",
                                current_tracker=peaks):
                return bail("descend")
            _set_torque_limit(bus, live, DEMO_TORQUE_LIMIT)
        else:
            note("(no bench stand-up — skipping the standing acts)")

        # ---- Finale (seated) ------------------------------------------
        note("finale — hands to the sky")

        def finale(t: float) -> list[float]:
            pose = _zero_pose()
            up = _win(t, 0.0, 2.5)
            fall = 1.0 - _win(t, 5.5, 7.0)
            pulse = 5.0 * math.sin(2.0 * math.pi * 1.1 * max(0.0, t - 2.5))
            if t < 5.5:
                pulse *= _win(t, 2.5, 3.0)
            else:
                pulse = 0.0
            for leg in range(6):
                _yaw_hip_knee(
                    leg, pose,
                    hip=(ARMS_UP_HIP_DEG + pulse) * up * fall,
                    knee=(ARMS_UP_KNEE_DEG - 0.75 * pulse) * up * fall)
            if 7.6 <= t < 8.1:              # one last heartbeat
                b = 8.0 * math.sin(math.pi * (t - 7.6) / 0.5)
                for leg in range(6):
                    pose[leg * 3 + 2] += b
            return pose

        st = seg(finale, 9.0, "finale")
        r = gate(st, "finale")
        if r:
            return r
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)

    peaks.print_report(phase="fever dream")
    note("goodnight.")
    if not go_to_zero_pose(bus, abort_check=check, seconds=1.5):
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    _limp_all(bus, live)
    _set_torque_limit(bus, live, 1000)
    return "done"


ENCORE_QUAD_SPEED_CAP = 1.2   # the horse actually WALKS — tighter than 1.5


def run_encore_dance(bus: FeetechBus, *, abort_check=None, status_cb=None,
                     torque=None, speed: float = 1.0, speed_fn=None,
                     standup_fn=None, log_path: Path | None = None) -> str:
    """THE ENCORE — greatest hits remixed, plus three brand-new acts.

    I    overture: heartbeat vs drums — the resurrection heartbeat and
         the floor-tap drum solo answer each other, tightening each
         round until they collide in one unison slam
    II   gearshift: the 1:2:3 gearbox meshes, a clutch FREEZE locks the
         lineup, then it re-meshes spinning the other way with the
         gear ratios swapped
    III  trident bow: pairs merge into three thick arms, roll, take a
         bowing round, gather overhead for one pulse
    IV   STEP stand-up (no fake-outs — this show plays it straight)
    V    the typewriter (NEW): one leg at a time lifts, flicks, and
         taps back down around the hex — two laps, the second twice as
         fast — then a double all-six carriage-return stomp
    VI   the compass (NEW): a world-pinned no-slip turn — the feet
         never scrub while the body sweeps ~30° one way, holds, and
         sweeps back
    VII  THE CIRCUS HORSE (NEW): rears onto four legs, paws the air,
         then actually WALKS while reared — shies backward, holds,
         and walks forward to its mark — and comes down, quad balance
         trim armed the whole time
    VIII victory ripple and descend
    IX   curtain call: the legs take their bows overhead one by one,
         tremble in crescendo, sweep down, and one last heartbeat

    Without the bench ``standup_fn`` (CLI): acts I–III and the curtain
    call only.  Live speed works everywhere; every reared act caps it
    at 1.2x (the horse is walking, not just holding).
    """
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    base_spd = speed_fn or (lambda: _clamp_demo_speed(speed))

    def note(msg: str) -> None:
        print(f"  {msg}")
        if status_cb is not None:
            try:
                status_cb(str(msg))
            except Exception:
                pass

    _enable_torque(bus, live)
    _set_torque_limit(bus, live, DEMO_TORQUE_LIMIT)
    peaks = CurrentPeakTracker()

    def bail(label: str) -> str:
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return f"error: {label}"

    log_cm = MotionLog(log_path, live) if log_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        def seg(pose_fn, secs: float, label: str, *, max_speed=3000,
                max_acc=200, tick_s=STREAM_TICK_S, tilt=None,
                spd=None, trim=None) -> str:
            # Fresh tracker per segment (see run_wild_dance).
            trk = CurrentPeakTracker()
            st = stream_pose_fn(
                bus, live, pose_fn, seconds=secs, abort_check=check,
                speed_fn=spd or base_spd, status_cb=status_cb,
                label=label, tracker=trk, log=log_cm,
                max_speed=max_speed, max_acc=max_acc, tick_s=tick_s,
                tilt_guard_deg=tilt, balance_trim=trim)
            if trk.peak_a > peaks.peak_a:
                peaks.peak_a = trk.peak_a
                peaks.peak_joint = trk.peak_joint
            for j, a in trk.max_a.items():
                if a > peaks.max_a.get(j, 0.0):
                    peaks.max_a[j] = a
            peaks.samples += trk.samples
            return st

        def gate(st: str, label: str) -> str | None:
            if st == "aborted":
                _set_torque_limit(bus, live, 1000)
                return "aborted"
            if st == "guard":
                return bail(f"current guard ({label})")
            if st.startswith("tilt:"):
                # stream already went limp for the soft landing
                _set_torque_limit(bus, live, 1000)
                return f"error: tilt {st[5:]} deg during {label} — went limp"
            if st.startswith("balance:"):
                # quad balance trim saw a fall / recovery timeout and
                # already went limp — surface it, don't keep dancing.
                _set_torque_limit(bus, live, 1000)
                return f"error: balance ({st[8:]}) during {label} — went limp"
            return None

        # ---- Act I: overture — heartbeat vs drums ---------------------
        note("act I — overture: heartbeat vs drums")
        # Rounds tighten: (heartbeat_s, drum_s) per round, then a slam.
        rounds = ((2.4, 2.0), (2.0, 1.6), (1.6, 1.3))
        r_t0 = []
        acc_t = 0.6
        for hb_s, dr_s in rounds:
            r_t0.append((acc_t, hb_s, dr_s))
            acc_t += hb_s + dr_s
        slam_t0 = acc_t + 0.2
        overture_s = slam_t0 + 1.6

        def overture(t: float) -> list[float]:
            pose = _zero_pose()
            for t0, hb_s, dr_s in r_t0:
                if t0 <= t < t0 + hb_s:
                    # heartbeat: double knee thump, all six in unison
                    u = (t - t0) / hb_s
                    for p0 in (0.05, 0.42):
                        if p0 <= u < p0 + 0.28:
                            b = math.sin(math.pi * (u - p0) / 0.28)
                            for leg in range(6):
                                _yaw_hip_knee(leg, pose, knee=9.5 * b,
                                              hip=-3.5 * b)
                    return pose
                d0 = t0 + hb_s
                if d0 <= t < d0 + dr_s:
                    # drums answer: odd/even floor-tap doubles
                    u = (t - d0) / dr_s
                    hover = -8.0
                    idx = int(u * 4.0)
                    v = u * 4.0 - idx
                    hit = math.sin(math.pi * v / 0.7) if v < 0.7 else 0.0
                    for leg in range(6):
                        b = hit if leg % 2 == idx % 2 else 0.0
                        _yaw_hip_knee(leg, pose, hip=hover * (1.0 - b),
                                      knee=5.0 * b)
                    return pose
            if slam_t0 <= t < slam_t0 + 0.55:
                # both voices at once: one big unison slam
                b = 1.25 * math.sin(math.pi * (t - slam_t0) / 0.55)
                for leg in range(6):
                    _yaw_hip_knee(leg, pose, knee=8.0 * b, hip=-4.0 * b)
            return pose

        st = seg(overture, overture_s, "overture")
        r = gate(st, "overture")
        if r:
            return r

        # ---- Act II: gearshift ----------------------------------------
        note("act II — GEARSHIFT (clutch in …)")

        def gearshift(t: float) -> list[float]:
            a = min(1.0, t / 1.2, max(0.0, (14.0 - t) / 1.2))
            fz = _win(t, 6.2, 6.8) - _win(t, 7.4, 8.0)   # clutch freeze
            rev = _win(t, 7.4, 8.0)                       # reversed remesh
            f0 = 0.4
            hit = _swarm_pulse((t * f0) % 1.0)
            pose = _zero_pose()
            for leg in range(6):
                # forward mesh: hips 1x pairsA, yaws 2x pairsB, knees 3x
                hip_f = (-24.0 + _dp(t, leg, amp=15.0, freq=f0,
                                     group="pairsA") - 3.0 * hit)
                yaw_f = _dp(t, leg, amp=11.0, freq=2.0 * f0, group="pairsB")
                knee_f = (8.0 + _dp(t, leg, amp=8.0, freq=3.0 * f0,
                                    group="opposite") + 4.0 * hit)
                # reversed mesh: spin flipped, yaw/knee ratios swapped
                hip_r = (-24.0 + _dp(t, leg, amp=15.0, freq=f0,
                                     group="pairsA", spin=-1.0) - 3.0 * hit)
                yaw_r = _dp(t, leg, amp=10.0, freq=3.0 * f0,
                            group="opposite", spin=-1.0)
                knee_r = (8.0 + _dp(t, leg, amp=7.0, freq=2.0 * f0,
                                    group="pairsB", spin=-1.0) + 4.0 * hit)
                m = 1.0 - rev
                hip = hip_f * m + hip_r * rev
                yaw = yaw_f * m + yaw_r * rev
                knee = knee_f * m + knee_r * rev
                # the clutch: everything locks into a stiff lineup
                hip = hip * (1.0 - fz) + (-30.0) * fz
                yaw = yaw * (1.0 - fz)
                knee = knee * (1.0 - fz) + 14.0 * fz
                _yaw_hip_knee(leg, pose, yaw=yaw * a, hip=hip * a,
                              knee=knee * a)
            return pose

        st = seg(gearshift, 14.0, "gearshift")
        r = gate(st, "gearshift")
        if r:
            return r

        # ---- Act III: trident bow -------------------------------------
        note("act III — the trident bow")

        def trident_bow(t: float) -> list[float]:
            u = t / 12.0
            form = _win(u, 0.0, 0.10)
            gather = _win(u, 0.72, 0.82)
            fade = _win(u, 0.88, 1.0)
            pose = _zero_pose()
            for leg in range(6):
                pair_a = leg // 2
                ph_a = 2.0 * math.pi * pair_a / 3.0
                # rolling trident
                hip_m = -26.0 + 11.0 * math.sin(2.0 * math.pi * 0.5 * t
                                                + ph_a)
                knee_m = 10.0 - 6.0 * math.sin(2.0 * math.pi * 0.5 * t
                                               + ph_a)
                # bowing round: one thick arm dips at a time (hip stays
                # NEGATIVE — dips toward the floor, never into it)
                bow = _win(u, 0.38, 0.44) - _win(u, 0.66, 0.72)
                dip_ph = (t - 4.6) / 1.6
                dip = 0.0
                if 0.0 <= dip_ph < 3.0 and int(dip_ph) == pair_a:
                    dip = math.sin(math.pi * (dip_ph - int(dip_ph)))
                hip = (hip_m * (1.0 - bow)
                       + (-34.0 + 26.0 * dip) * bow)
                knee = (knee_m * (1.0 - bow)
                        + (8.0 + 10.0 * dip) * bow)
                yaw = _pair_sign(leg, 0) * AIR_PAIR_YAW_DEG
                # final gather overhead + one pulse
                pulse = 5.0 * math.sin(2.0 * math.pi * 1.2 * t) * gather
                hip = hip * (1.0 - gather) + (ARMS_UP_HIP_DEG
                                              + pulse) * gather
                knee = knee * (1.0 - gather) + ARMS_UP_KNEE_DEG * gather
                yaw = yaw * (1.0 - gather)
                s = form * (1.0 - fade)
                _yaw_hip_knee(leg, pose, yaw=yaw * s, hip=hip * s,
                              knee=knee * s)
            return pose

        st = seg(trident_bow, 12.0, "trident bow")
        r = gate(st, "trident bow")
        if r:
            return r

        if standup_fn is not None:
            # ---- Act IV: the real stand-up, played straight -----------
            note("act IV — STEP stand-up")
            ok, err = standup_fn()
            if check():
                return bail("stand-up aborted")
            if not ok:
                note(f"stand-up stopped: {err}")
                return bail(f"stand-up: {err}")
            _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
            hip, knee = RISE_HIGH_HIP_DEG, RISE_HIGH_KNEE_DEG
            planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
            if not ease_to_pose(bus, planted, abort_check=check,
                                seconds=0.7, label="set stance",
                                current_tracker=peaks):
                return bail("set stance")
            _set_torque_limit(bus, live, DANCE_PLANT_TORQUE)

            # ---- Act V: the typewriter --------------------------------
            # Sequential single-leg taps: the active leg LIFTS first
            # (friction released) before any yaw flick — never yaw a
            # loaded leg (hardware lesson 08-19, see run_wild_dance).
            note("act V — the typewriter")
            lap1, lap2 = 0.85, 0.55
            t_lap2 = 6 * lap1
            t_cr = t_lap2 + 6 * lap2
            type_s = t_cr + 1.6

            def typewriter(t: float) -> list[float]:
                pose = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
                if t < t_cr:
                    step = lap1 if t < t_lap2 else lap2
                    tt = t if t < t_lap2 else t - t_lap2
                    idx = min(5, int(tt / step))
                    u = tt / step - idx
                    b = math.sin(math.pi * min(1.0, u / 0.9))
                    flick = 10.0 * math.sin(
                        2.0 * math.pi * 1.5 * u) * b
                    y, h, k = _show_blend_leg(hip, knee, idx, 0.85 * b,
                                              wave_yaw=flick)
                    _set_leg(pose, idx, yaw=y, hip=h, knee=k)
                    return pose
                # carriage return: two crisp all-six stomps
                tt = t - t_cr
                for t0 in (0.15, 0.8):
                    if t0 <= tt < t0 + 0.4:
                        b = math.sin(math.pi * (tt - t0) / 0.4)
                        for leg in range(6):
                            y, h, k = _show_blend_leg(hip, knee, leg,
                                                      0.45 * b)
                            _set_leg(pose, leg, yaw=y, hip=h, knee=k)
                return pose

            st = seg(typewriter, type_s, "typewriter",
                     max_speed=900, max_acc=80)
            r = gate(st, "typewriter")
            if r:
                return r

            # ---- Act VI: the compass ----------------------------------
            # World-pinned no-slip turn (the drive controller's GAIT 1
            # machinery as choreography): clamp-fit timing, omega-only.
            # Feet never scrub; the body sweeps ~30° and comes back.
            note("act VI — the compass (no-slip pirouette)")
            try:
                from noslip_gait import NoSlipGait
            except ImportError:
                sys.path.insert(
                    0, str(_HERE.parent / "linux_control"))
                from noslip_gait import NoSlipGait
            gait = NoSlipGait.clamp_fit()
            gait.sync_plant_stance(hip, knee)

            def compass(t: float) -> list[float]:
                if t < 8.5:
                    gait.set_velocity(omega=0.30)
                elif t < 10.0:
                    gait.set_velocity(omega=0.0)
                elif t < 18.0:
                    gait.set_velocity(omega=-0.30)
                else:
                    gait.set_velocity(omega=0.0)
                return gait.desired_deg(t)

            st = seg(compass, 19.5, "the compass",
                     max_speed=900, max_acc=80)
            r = gate(st, "the compass")
            if r:
                return r
            if not ease_to_pose(bus, planted, abort_check=check,
                                seconds=0.8, label="compass home",
                                current_tracker=peaks):
                return bail("compass home")

            # ---- Act VII: THE CIRCUS HORSE ----------------------------
            note("act VII — THE CIRCUS HORSE (tipping back …)")
            from quad_walk import (ENTRY_TOTAL_S, EXIT_TOTAL_S, GAITS,
                                   TUCK_DEG)
            _set_torque_limit(bus, live, STAND_DANCE_TORQUE)
            if not ease_to_pose(bus, _stand_zero_pose(),
                                abort_check=check, seconds=0.8,
                                label="horse base",
                                current_tracker=peaks):
                return bail("horse base")
            horse_spd = lambda: min(  # noqa: E731
                ENCORE_QUAD_SPEED_CAP, _clamp_live_speed(base_spd()))

            def quad_trim(gait_key: str) -> QuadPitchTrim:
                pitch = GAITS[gait_key].get("pitch")
                return QuadPitchTrim(
                    expected_pitch_deg=(math.degrees(float(pitch))
                                        if pitch is not None else None),
                    gait=gait_key)

            def horse_seg(label: str, secs: float, *, gait_key: str,
                          phase: str, direction: float = 1.0,
                          gesture_fn=None,
                          trimmed: bool = True) -> str | None:
                trim = quad_trim(gait_key) if trimmed else None
                fn = _make_quad_fn(
                    secs, gait=gait_key, direction=direction,
                    phase=phase,
                    trim_fn=trim.pose_trim if trim else None)
                if gesture_fn is not None:
                    base_fn = fn

                    def fn(t: float, _b=base_fn, _g=gesture_fn):  # noqa: E731
                        return _g(t, _b(t))
                    # balance recovery needs the stance schedule
                    fn.all_stance_at = getattr(
                        base_fn, "all_stance_at", None)
                st = seg(fn, secs, label,
                         max_acc=QUAD_STREAM_ACC,
                         tick_s=QUAD_STREAM_TICK_S,
                         tilt=QUAD_TILT_GUARD_DEG,
                         spd=horse_spd, trim=trim)
                return gate(st, label)

            # rear up (entry choreography, untrimmed like quad_rear)
            r = horse_seg("rear up", ENTRY_TOTAL_S + 2.5,
                          gait_key="rear", phase="rear", trimmed=False)
            if r:
                return r

            # pawing flourish on the reared hold
            note("act VII — pawing the air")
            paws_s = 7.0

            def paws(t: float, pose: list[float]) -> list[float]:
                w = min(1.0, t / 0.8, max(0.0, (paws_s - t) / 1.2))
                w = 0.5 - 0.5 * math.cos(math.pi * w)
                y0, h0, k0 = TUCK_DEG
                for leg in (0, 5):
                    if t < 4.4:               # alternating paw strokes
                        k = int(t / 1.1)
                        active = 5 if k % 2 == 0 else 0
                        b = (math.sin(math.pi * (t / 1.1 - k))
                             if leg == active else 0.0)
                        gy, gh, gk = y0, h0 + 18.0 * b, k0 - 35.0 * b
                    else:                     # both paws salute together
                        b = math.sin(math.pi * min(1.0, (t - 4.4) / 1.4))
                        gy, gh, gk = y0, h0 + 22.0 * b, k0 - 30.0 * b
                    j = leg * 3
                    pose[j] = pose[j] * (1.0 - w) + gy * w
                    pose[j + 1] = pose[j + 1] * (1.0 - w) + gh * w
                    pose[j + 2] = pose[j + 2] * (1.0 - w) + gk * w
                return pose

            r = horse_seg("pawing the air", paws_s,
                          gait_key="rear", phase="hold", gesture_fn=paws)
            if r:
                return r

            # Backward first, forward home: in the loaded-fit sim the
            # reared walk travels reliably BACKWARD (~5 mm/s) but barely
            # forward (uphill against the lean), so retreat-then-return
            # keeps the net drift small and pointed AWAY from the
            # audience (probe 08-21, /tmp/encore_probe2.py).
            note("act VII — the horse SHIES BACK …")
            r = horse_seg("horse walk back", 9.0,
                          gait_key="walk", phase="walk", direction=-1.0)
            if r:
                return r

            r = horse_seg("reared hold", 2.5,
                          gait_key="rear", phase="hold")
            if r:
                return r

            note("act VII — … and WALKS back to its mark")
            r = horse_seg("horse walk fwd", 9.0,
                          gait_key="walk", phase="walk")
            if r:
                return r

            note("act VII — coming down")
            r = horse_seg("horse down", EXIT_TOTAL_S + 1.0,
                          gait_key="rear", phase="down", trimmed=False)
            if r:
                return r
            _set_torque_limit(bus, live, DANCE_PLANT_TORQUE)

            # ---- Act VIII: victory ripple + descend -------------------
            if not ease_to_pose(bus, planted, abort_check=check,
                                seconds=0.8, label="ripple stance",
                                current_tracker=peaks):
                return bail("ripple stance")
            note("act VIII — victory ripple")
            st = seg(lambda t: _show_stream_pose("ripple", t, hip, knee),
                     4.0, "victory ripple", max_speed=900, max_acc=80)
            r = gate(st, "victory ripple")
            if r:
                return r
            note("coming back down …")
            _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
            if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                                seconds=4.5, label="descend",
                                current_tracker=peaks):
                return bail("descend")
            _set_torque_limit(bus, live, DEMO_TORQUE_LIMIT)
        else:
            note("(no bench stand-up — skipping the standing acts)")

        # ---- Act IX: curtain call (seated) ------------------------------
        note("act IX — curtain call")
        order = (0, 5, 1, 4, 2, 3)          # criss-cross roll call

        def curtain(t: float) -> list[float]:
            pose = _zero_pose()
            fall = 1.0 - _win(t, 9.6, 10.8)
            for slot, leg in enumerate(order):
                t0 = 0.4 + 0.9 * slot
                up = _win(t, t0, t0 + 1.1)
                over = 6.0 * math.sin(math.pi * _win(t, t0, t0 + 1.6))
                trem = 0.0
                if t > 6.4:                  # unison tremolo crescendo
                    amp = 2.0 + 4.0 * _win(t, 6.4, 9.4)
                    trem = amp * math.sin(2.0 * math.pi * 2.2 * t)
                _yaw_hip_knee(
                    leg, pose,
                    hip=(ARMS_UP_HIP_DEG - over + trem) * up * fall,
                    knee=(ARMS_UP_KNEE_DEG - 0.6 * trem) * up * fall)
            if 12.0 <= t < 12.5:             # one last heartbeat
                b = 8.0 * math.sin(math.pi * (t - 12.0) / 0.5)
                for leg in range(6):
                    pose[leg * 3 + 2] += b
            return pose

        st = seg(curtain, 13.5, "curtain call")
        r = gate(st, "curtain call")
        if r:
            return r
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)

    peaks.print_report(phase="the encore")
    note("that's the show.")
    if not go_to_zero_pose(bus, abort_check=check, seconds=1.5):
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    _limp_all(bus, live)
    _set_torque_limit(bus, live, 1000)
    return "done"


# Ordered gentlest → spiciest (web UI + CLI menu follow this order).
DEMOS = {
    # --- gentle air (offsets around logical 0° / legs out) ---------------
    "breathe": ("[1 gentle] soft hip/knee flex in the air (size slider)",
                frames_breathe),
    "breathe_v": ("[1 gentle] VELOCITY breathe — speed hold, no position hunt",
                  None),
    "heartbeat": ("[1 gentle] double-thump knee pulse", frames_heartbeat),
    "twinkle": ("[1 gentle] small random alive wiggles", frames_twinkle),
    "shimmy": ("[2 easy] odd/even yaw shimmy (from zero)", frames_shimmy),
    "shimmy_v": ("[2 easy] shimmy in VELOCITY mode — no position hunt",
                 None),
    "ripple": ("[2 easy] yaw wave around the hex (air)", frames_ripple),
    "conductor": ("[2 easy] one leg waves; others hold", frames_conductor),
    "arms_up": ("[2 easy] sit: all six arms way over head", None),
    "air_meet": ("[3 air show] six solos LOCK into lineups, then scatter",
                 frames_air_meet),
    "air_pendulum": ("[3 air show] pendulum wave — drifts apart, snaps "
                     "into sync", frames_air_pendulum),
    "air_orbits": ("[3 air show] six orbits magnetize into one, release",
                   frames_air_orbits),
    "air_trident": ("[3 air show] THREE THICK ARMS — pairs merge, swap "
                    "partners, split 2 thick + 2 thin", frames_air_trident),
    "air_weave": ("[3 air show] WEAVE — ripple + shimmy + bend braided "
                  "at odd frequencies; grouping morphs mid-show",
                  frames_air_weave),
    "air_gearbox": ("[3 air show] GEARBOX — hips/yaws/knees meshed 1:2:3, "
                    "clunks into alignment every ~3 s", frames_air_gearbox),
    "air_tides": ("[3 air show] TIDES — counter-rotating waves interfere, "
                  "collapse to an opposite-pair finale", frames_air_tides),
    "dance_swarm": ("[3 air show] SIMULATION SWARM — beat-synced to the "
                    "Big Thief song (4:07) · counts you in, press play "
                    "on GO", None),
    # --- standing dances (streamed · live speed · around the live plant) --
    "stand_sway": ("[3 stand] slow body sway — weight orbits the hex", None),
    "stand_bounce": ("[3 stand] squat bob — smooth streamed bounce", None),
    "stand_twist": ("[3 stand] in-place body twist (cord-safe)", None),
    "stand_wave": ("[3 stand] one leg lifts + waves, cycles legs", None),
    "stand_ripple": ("[4 stand] traveling lift wave around the hex", None),
    # --- mild planted ----------------------------------------------------
    "rise": ("[3 plant] deep reach; ends at stand zero", None),
    "rise+": ("[3 plant] higher + faster reach; ends at stand zero", None),
    "rise_turn": ("[3 plant] fast plant + twist; ends at stand zero", None),
    "stand_hands": ("[3 plant] stand: three arms over head (tripod)", None),
    "plant_look": ("[3 plant] look L/R + nod (standing)", None),
    "plant_bounce": ("[4 lively] boom/pop chassis bounce (standing)", None),
    # --- spicy planted (rise_show acts, runnable alone) ------------------
    "plant_ripple": ("[4 lively] standing ripple — legs flying around", None),
    "plant_gallop": ("[5 spicy] standing gallop — opposite pairs", None),
    "plant_tripod": ("[5 spicy] tripod flip — three up / three down", None),
    "plant_fan": ("[5 spicy] fan — all six dancing", None),
    "plant_star": ("[5 spicy] odds/evens + star snaps", None),
    "plant_stomp": ("[5 spicy] stomp barrage + ta-da", None),
    "rise_show": ("[6 show] FULL planted show (all of the above)", None),
    "dance": ("[6 show] DANCE — heartbeat → breathe → hands up → RISE "
              "→ wild → sleep", None),
    "dance_walk": ("[6 show] DANCE + VICTORY LAP — the dance, then "
                   "prance out, about-face, prance home, sleep",
                   None),
    "dance_swarm_stand": ("[6 show] SIMULATION SWARM ON ITS FEET — "
                          "seated verses, stands up in the build, "
                          "planted tripod/march choruses, sits for the "
                          "fade (press play on GO)", None),
    "dance_steeple": ("[6 show] THE STEEPLE — stands up, then adjacent "
                      "arms merge into ONE thick arm overhead, each "
                      "pair takes a turn", None),
    "dance_wild": ("[6 show] THE FEVER DREAM — resurrection heartbeat, "
                   "floor-drum solo, storm, fake-out stand-ups, planted "
                   "rampage, REARS UP like a stallion, big finale "
                   "(~2 min)", None),
    "dance_encore": ("[6 show] THE ENCORE — heartbeat-vs-drums duel, "
                     "gearshift, trident bow, typewriter taps, no-slip "
                     "compass turn, and the CIRCUS HORSE: rears up and "
                     "WALKS on four legs, there and back (~2.5 min)",
                     None),
    # --- real walk (open-loop tripod gait) --------------------------------
    "walk": ("[7 walk] tripod forward a few strides, then stand", None),
    "walk_spin": ("[7 walk] in-place turn (tripod), then stand", None),
    "walk_oval": ("[7 walk] forward → spin → reverse → stand", None),
    # --- quad mode (own web tab: tip back, walk on four legs) -------------
    "quad_rear": ("[8 quad] REAR UP — tip back on 4 legs and hold", None),
    "quad_hold": ("[8 quad] HOLD — settle to stable reared hold", None),
    "quad_walk": ("[8 quad] WALK FORWARD — animal walk while reared", None),
    "quad_walk_back": ("[8 quad] WALK BACKWARD — reverse animal walk while reared", None),
    "quad_trot": ("[8 quad] TROT FORWARD — diagonal pairs while reared", None),
    "quad_trot_back": ("[8 quad] TROT BACKWARD — reverse diagonal pairs while reared", None),
    "quad_down": ("[8 quad] COME DOWN — untuck fronts and return to stand", None),
    "quad_rear_pitch": ("[8 quad] REAR UP PITCHED — -28 deg nose-up hold", None),
    "quad_hold_pitch": ("[8 quad] HOLD PITCHED — settle to -28 deg hold", None),
    "quad_walk_pitch": ("[8 quad] WALK FORWARD PITCHED — -28 deg stance", None),
    "quad_walk_back_pitch": ("[8 quad] WALK BACKWARD PITCHED — -28 deg stance", None),
    "quad_trot_pitch": ("[8 quad] TROT FORWARD PITCHED — capped diagonal pairs", None),
    "quad_trot_back_pitch": ("[8 quad] TROT BACKWARD PITCHED — capped diagonal pairs", None),
    "quad_down_pitch": ("[8 quad] COME DOWN PITCHED — exit from -28 deg hold", None),
    "quad_rear_aggressive": ("[8 quad] REAR UP AGGRESSIVE — legacy aft-heavy hold", None),
    "quad_hold_aggressive": ("[8 quad] HOLD AGGRESSIVE — settle to legacy aft-heavy hold", None),
    "quad_walk_aggressive": ("[8 quad] WALK FORWARD AGGRESSIVE — legacy aft-heavy stance", None),
    "quad_walk_back_aggressive": ("[8 quad] WALK BACKWARD AGGRESSIVE — legacy aft-heavy stance", None),
    "quad_trot_aggressive": ("[8 quad] TROT FORWARD AGGRESSIVE — heavily capped diagonal pairs", None),
    "quad_trot_back_aggressive": ("[8 quad] TROT BACKWARD AGGRESSIVE — heavily capped diagonal pairs", None),
    "quad_down_aggressive": ("[8 quad] COME DOWN AGGRESSIVE — exit from legacy aft-heavy hold", None),
}

_QUAD_ACTION_TITLES = {
    "rear": "REAR UP",
    "hold": "HOLD",
    "walk": "WALK FORWARD",
    "walk_back": "WALK BACKWARD",
    "trot": "TROT FORWARD",
    "trot_back": "TROT BACKWARD",
    "down": "COME DOWN",
}
for _suffix, (_rear_gait, _walk_gait, _trot_gait, _label) in (
        QUAD_VARIANTS.items()):
    _tag = "" if _suffix == "" else f" {_label.upper()}"
    for _action, _title in _QUAD_ACTION_TITLES.items():
        DEMOS.setdefault(
            _quad_name(_action, _suffix),
            (f"[8 quad] {_title}{_tag} — {_label} quad test", None))

# Standalone planted acts (not the full rise_show script).
PLANTED_ACTS = frozenset({
    "plant_look", "plant_bounce", "plant_ripple", "plant_gallop",
    "plant_tripod", "plant_fan", "plant_star", "plant_stomp",
})

# Default air-demo durations (seconds at speed=1.0).
AIR_DEMO_SECONDS = {
    "breathe": 10.0,
    "breathe_v": 10.0,
    "heartbeat": 6.0,
    "twinkle": 6.0,
    "shimmy": 7.0,
    "shimmy_v": 8.0,
    "ripple": 8.0,
    "conductor": 8.0,
    "arms_up": 6.0,
    "air_meet": 24.0,
    "air_pendulum": 20.0,
    "air_orbits": 18.0,
    "air_trident": 34.0,
    "air_weave": 26.0,
    "air_gearbox": 24.0,
    "air_tides": 28.0,
    "dance_swarm": 252.0,        # the song's length — the song is the clock
    "dance_swarm_stand": 252.0,  # same song, standing choruses
    "dance_steeple": 56.0,
    "dance_wild": 125.0,
    "dance_encore": 155.0,
}


def _clamp_demo_speed(speed: float) -> float:
    """1.0 = nominal; 2.0 = twice as fast; 0.5 = half speed."""
    try:
        s = float(speed)
    except (TypeError, ValueError):
        s = 1.0
    return max(0.25, min(3.0, s))


def _speed_scale(speed: float) -> float:
    """Multiply durations by this (speed↑ → scale↓)."""
    return 1.0 / _clamp_demo_speed(speed)


def _rise_preset(name: str = "default") -> dict:
    key = "high_fast" if name in ("rise+", "high_fast", "high+fast") else "default"
    return dict(RISE_PRESETS[key])


def run_rise_demo(bus: FeetechBus, *,
                  abort_check=None,
                  rise_seconds: float | None = None,
                  descend_seconds: float | None = None,
                  hip_deg: float | None = None,
                  knee_deg: float | None = None,
                  preset: str = "default",
                  log_path: Path | None = None) -> str:
    """Deep reach from the elevated stand, then key → descend to zero.

    Default: hip +19° / knee +28° over ~12 s (~100 mm drop).
    ``rise+`` / preset ``high_fast``: hip +20° / knee +36° over ~5 s
    (~119 mm).  One SyncWrite (no host streaming).  Logs peak current;
    stops early on hip/knee current spike (contact).  Key during motion
    aborts; key while holding starts the descent.
    """
    cfg = _rise_preset(preset)
    hip = float(hip_deg if hip_deg is not None else cfg["hip"])
    knee = float(knee_deg if knee_deg is not None else cfg["knee"])
    rise_s = float(rise_seconds if rise_seconds is not None
                   else cfg["rise_seconds"])
    descend_s = float(descend_seconds if descend_seconds is not None
                      else cfg["descend_seconds"])
    # Rough foot drop (mm) matching mujoco femur/tibia FK.
    _p = math.radians(hip)
    _k = math.radians(knee)
    drop_mm = int(round(FEMUR_MM * math.sin(_p) + TIBIA_MM * math.sin(_k)))
    if abs(hip - RISE_HIGH_HIP_DEG) < 0.5 and rise_s <= RISE_FAST_SECONDS + 0.1:
        label = "higher + faster reach"
    elif abs(hip - RISE_HIGH_HIP_DEG) < 0.5:
        label = "higher reach"
    elif rise_s <= RISE_FAST_SECONDS + 0.1:
        label = "faster reach"
    else:
        label = "deep reach"

    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 robot IDs answering; moving "
              f"those only: {sorted(live)}")

    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)

    if check():
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    peaks = CurrentPeakTracker()
    if log_path is not None:
        peak_path = (log_path.with_name(log_path.stem + "_peaks.csv")
                     if log_path.suffix else Path(str(log_path) + "_peaks.csv"))
        motion_path = log_path
    else:
        tag = "rise_plus_current_peaks" if preset != "default" else "rise_current_peaks"
        peak_path = default_log_path(tag)
        motion_path = None

    goal = _elevated_stand_pose(hip=hip, knee=knee)
    print(f"  Rise demo — {label} (stand is high — reach toward the floor):")
    print(f"    target yaw 0°, hip {hip:.0f}°, knee {knee:.0f}°  "
          f"(~{drop_mm} mm foot drop)")
    print(f"    glide in ~{rise_s:.0f} s + max-current log; "
          f"contact stop ≥ {RISE_CONTACT_CURRENT_A:.2f} A")
    log_cm = MotionLog(motion_path, live) if motion_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        ok = ease_to_pose(
            bus, goal, abort_check=check, seconds=rise_s,
            label=label, current_tracker=peaks,
            contact_current_a=RISE_CONTACT_CURRENT_A, log=log_cm)
        peaks.print_report(phase="rise")
        # Write even on abort so a snag leaves a breadcrumb.
        peaks.write_log(peak_path, phase="rise" if ok else "rise (aborted)")
        if not ok:
            _set_torque_limit(bus, live, 1000)
            return "aborted"

        signal = _wait_for_descend(bus, live, check, peaks,
                                   auto_hold_s=RISE_WEB_HOLD_S)
        if signal == "aborted":
            peaks.print_report(phase="rise+hold (aborted)")
            peaks.write_log(peak_path, phase="rise+hold (aborted)")
            _set_torque_limit(bus, live, 1000)
            return "aborted"
        print("  Returning to stand zero (not sit) …")
        descend_check = check
        if _abort_is_latched(check) and check():
            descend_check = (lambda: False)
        if not go_to_stand_pose(bus, abort_check=descend_check, seconds=descend_s,
                                current_tracker=peaks):
            peaks.print_report(phase="rise+hold+stand (aborted)")
            peaks.write_log(peak_path, phase="rise+hold+stand (aborted)")
            _set_torque_limit(bus, live, 1000)
            return "aborted"

        peaks.print_report(phase="rise+hold+stand")
        peaks.write_log(peak_path, phase="rise+hold+stand")
        # Stay torqued at stand home so the next planted demo can start immediately.
        _set_torque_limit(bus, live, 1000)
        _hold_here(bus, live)
        print("  At stand zero — ready for another planted demo.")
        return "done"
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)


def _planted_pause(bus: FeetechBus, live: set[int], check, peaks,
                   seconds: float) -> bool:
    """True if aborted."""
    if seconds <= 0:
        return False
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        if check():
            _hold_here(bus, live)
            return True
        if peaks is not None:
            peaks.sample(bus, live)
        time.sleep(0.05)
    return False


def _planted_glide(bus: FeetechBus, goal: list[float], *, check, peaks,
                   seconds: float, label: str,
                   contact: bool = False) -> bool:
    """Ease to pose; True on success, False on abort."""
    return ease_to_pose(
        bus, goal, abort_check=check, seconds=seconds, label=label,
        current_tracker=peaks,
        contact_current_a=(RISE_CONTACT_CURRENT_A if contact else None))


def _abort_is_latched(check) -> bool:
    """True when ``check`` is ``threading.Event.is_set`` (web Stop).

    Keystroke abort is edge-triggered (True only on the poll that saw a
    key).  A latched Event stays True forever after Stop — so the old
    ``while not check(): hold`` loop never released on the web UI.
    """
    import threading
    owner = getattr(check, "__self__", None)
    return isinstance(owner, threading.Event)


def _wait_for_descend(bus: FeetechBus, live: set[int], check, peaks,
                      *, auto_hold_s: float = 1.5) -> str:
    """Hold planted pose, then signal descend.

    Returns ``descend`` or ``aborted``.

    * CLI / keystroke watch: wait for any key → descend.
    * Web / latched Event (or no TTY): hold for ``auto_hold_s`` then
      return to stand; Stop / next-demo during the hold → aborted.
    """
    if _abort_is_latched(check) or not sys.stdin.isatty():
        print(f"  Holding planted pose ~{auto_hold_s:.0f}s "
              f"(Stop or next demo ends hold; then stand return)…")
        t_end = time.monotonic() + float(auto_hold_s)
        while time.monotonic() < t_end:
            if check():
                print("  Stop/switch during hold — staying here.")
                _hold_here(bus, live)
                return "aborted"
            if peaks is not None:
                peaks.sample(bus, live)
            time.sleep(0.1)
        return "descend"

    print("  Holding — press any key to descend to zero.")
    while not check():
        if peaks is not None:
            peaks.sample(bus, live)
        time.sleep(0.1)
    return "descend"


def _finish_planted_demo(bus: FeetechBus, live: set[int], check, peaks,
                         peak_path: Path, descend_s: float,
                         phase: str) -> str:
    signal = _wait_for_descend(bus, live, check, peaks,
                               auto_hold_s=PLANTED_WEB_HOLD_S)
    if signal == "aborted":
        peaks.print_report(phase=f"{phase} (aborted)")
        peaks.write_log(peak_path, phase=f"{phase} (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    print("  Returning to stand zero (not sit) …")
    # Latched Stop must not poison the glide — edge keystroke check is fine.
    descend_check = check
    if _abort_is_latched(check) and check():
        descend_check = (lambda: False)
    if not go_to_stand_pose(bus, abort_check=descend_check, seconds=descend_s,
                            current_tracker=peaks):
        peaks.print_report(phase=f"{phase} (aborted)")
        peaks.write_log(peak_path, phase=f"{phase} (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    peaks.print_report(phase=phase)
    peaks.write_log(peak_path, phase=phase)
    # Keep holding stand home (no limp) so chained planted demos stay up.
    _set_torque_limit(bus, live, 1000)
    _hold_here(bus, live)
    print("  At stand zero — ready for another planted demo.")
    return "done"


def run_rise_turn_demo(bus: FeetechBus, *,
                       abort_check=None,
                       rise_seconds: float | None = None,
                       descend_seconds: float | None = None,
                       yaw_deg: float = RISE_TURN_YAW_DEG,
                       turn_seconds: float = RISE_TURN_SECONDS,
                       speed: float = 1.0,
                       log_path: Path | None = None) -> str:
    """FAST high plant, small in-place yaw, yaw back, then descend.

    Cord-safe: peak |yaw| is small and always returns to 0 (no net wind
    on USB/power cables plugged into the robot).
    """
    sc = _speed_scale(speed)
    hip = RISE_HIGH_HIP_DEG
    knee = RISE_HIGH_KNEE_DEG
    rise_s = float(rise_seconds if rise_seconds is not None
                   else RISE_SHOW_PLANT_S) * sc
    descend_s = float(descend_seconds if descend_seconds is not None
                      else RISE_SHOW_DESCEND_S) * sc
    yaw = float(yaw_deg)
    turn_s = float(turn_seconds) * sc

    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 robot IDs answering; moving "
              f"those only: {sorted(live)}")

    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)

    if check():
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    peaks = CurrentPeakTracker()
    if log_path is not None:
        peak_path = (log_path.with_name(log_path.stem + "_peaks.csv")
                     if log_path.suffix else Path(str(log_path) + "_peaks.csv"))
    else:
        peak_path = default_log_path("rise_turn_current_peaks")

    planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
    turned = _elevated_stand_pose(hip=hip, knee=knee, yaw=yaw)

    print("  Rise-turn (FAST, cord-safe — small twist, always back to 0):")
    print(f"    plant hip {hip:.0f}°, knee {knee:.0f}° in ~{rise_s:.0f} s; "
          f"yaw {yaw:+.0f}° then 0 (~{turn_s:.1f} s each)")
    print(f"    contact stop ≥ {RISE_CONTACT_CURRENT_A:.2f} A during rise")

    # Quick overhead snap before the dive (same opener vibe as rise_show).
    if not _planted_glide(bus, _zero_pose(), check=check, peaks=peaks,
                          seconds=RISE_SHOW_OVERHEAD_S * sc, label="LEGS UP!"):
        peaks.write_log(peak_path, phase="overhead (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    if not _planted_glide(bus, planted, check=check, peaks=peaks,
                          seconds=rise_s, label="plant", contact=True):
        peaks.write_log(peak_path, phase="rise (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    peaks.print_report(phase="rise")

    if _planted_pause(bus, live, check, peaks, RISE_TURN_PAUSE_S * sc):
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    print(f"  Twist to yaw {yaw:+.0f}° ...")
    if not _planted_glide(bus, turned, check=check, peaks=peaks,
                          seconds=turn_s, label=f"yaw {yaw:+.0f}°"):
        peaks.write_log(peak_path, phase="turn (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    if _planted_pause(bus, live, check, peaks, RISE_TURN_PAUSE_S * sc):
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    print("  Untwist to yaw 0° ...")
    if not _planted_glide(bus, planted, check=check, peaks=peaks,
                          seconds=turn_s, label="yaw 0°"):
        peaks.write_log(peak_path, phase="turn-back (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    return _finish_planted_demo(bus, live, check, peaks, peak_path,
                                descend_s, "rise_turn+descend")


def _show_snap(bus: FeetechBus, goal: list[float], live: set[int], *,
               check, peaks, seconds: float, label: str) -> bool:
    """Fire a pose and wait a short beat — no full settle (show tempo)."""
    print(f"  → {label}")
    if check():
        _hold_here(bus, live)
        return False
    start = _read_pose(bus, live)
    speed, acc = _glide_speed_acc(start, goal, live, max(seconds, 0.25))
    # Bias snappy for the show.
    speed = normalize_speed(min(1200, int(speed * 1.25)))
    acc = normalize_acc(min(180, int(acc * 1.3)))
    _write_pose(bus, goal, live, speed=speed, acc=acc)
    t_end = time.monotonic() + max(0.12, seconds * 0.9)
    while time.monotonic() < t_end:
        if check():
            _hold_here(bus, live)
            return False
        if peaks is not None:
            peaks.sample(bus, live)
        time.sleep(0.04)
    return True


def _show_blend_leg(hip: float, knee: float, leg: int, lift: float,
                    *, wave_yaw: float = 0.0) -> tuple[float, float, float]:
    """lerp planted → lifted for one leg. ``lift`` in 0..1."""
    lift = max(0.0, min(1.0, lift))
    y = wave_yaw * lift
    h = hip * (1.0 - lift) + RISE_SHOW_LIFT_HIP_DEG * lift
    k = knee * (1.0 - lift) + RISE_SHOW_LIFT_KNEE_DEG * lift
    return y, h, k


def _show_stream_pose(mode: str, t: float, hip: float,
                      knee: float) -> list[float]:
    """One frame of a streamed show mode at time ``t`` (pure math, no bus)."""
    if mode == "jazz":
        pose = _zero_pose()
        for leg in range(6):
            pose[leg * 3] = (
                RISE_SHOW_WAVE_YAW_DEG
                * math.sin(4.2 * t + leg * math.pi / 3))
        return pose
    if mode == "orbit":
        # Body-lean circle (stand_sway math on the show plant): a
        # traveling crouch wave — every leg at its own 60° phase, all
        # feet loaded.  The chassis visibly leans in a slow circle.
        pose = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
        w = 2.0 * math.pi * 0.40 * t
        for leg in range(6):
            s = math.sin(w + leg * math.pi / 3.0)
            _yaw_hip_knee(leg, pose, hip=8.0 * s, knee=-4.0 * s)
        return pose
    if mode == "counterwave":
        # Two waves traveling in OPPOSITE directions around the hex —
        # knees ride one, yaws ride the other — so at any instant no
        # two legs are doing the same thing.  Small amplitudes; feet
        # stay loaded (sway-class move, not a lift).
        pose = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
        for leg in range(6):
            wk = 2.2 * t - leg * (math.pi / 3.0)   # knee wave, CW
            wy = 2.2 * t + leg * (math.pi / 3.0)   # yaw wave, CCW
            _yaw_hip_knee(leg, pose,
                          yaw=10.0 * math.sin(wy),
                          hip=5.0 * math.sin(wk + 0.6),
                          knee=9.0 * math.sin(wk))
        return pose

    pose = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
    for leg in range(6):
        if mode == "ripple":
            # 2–3 legs up at once; traveling wave around the hex.
            phase = 2.8 * t - leg * (2 * math.pi / 6)
            lift = max(0.0, math.sin(phase)) ** 0.65
            yaw_amp = RISE_SHOW_WAVE_YAW_DEG * math.sin(
                phase + 0.4)
        elif mode == "canon":
            # Follow-the-leader: leg 0 leads a lift+flourish gesture;
            # each next leg repeats it a fixed offset later and a
            # little smaller (decaying echo around the hex).  Narrow
            # envelope → ~one leg mid-gesture at a time.
            decay = 1.0 - 0.13 * leg
            phase = 1.9 * t - leg * (2 * math.pi / 6)
            lift = decay * max(0.0, math.sin(phase)) ** 2.0
            yaw_amp = (RISE_SHOW_WAVE_YAW_DEG * decay
                       * math.sin(2.0 * phase))
        elif mode == "gallop":
            # Opposite pairs pulse together (0+3, 1+4, 2+5).
            pair = leg % 3
            phase = 3.4 * t - pair * (2 * math.pi / 3)
            lift = max(0.0, math.sin(phase)) ** 0.6
            yaw_amp = 0.55 * RISE_SHOW_WAVE_YAW_DEG * math.sin(
                phase)
        elif mode == "tripod":
            # Odds vs evens alternate — three legs flying.
            group = 1.0 if (leg % 2) else -1.0
            phase = 3.0 * t
            raw = group * math.sin(phase)
            lift = max(0.0, raw) ** 0.55
            yaw_amp = RISE_SHOW_WAVE_YAW_DEG * 0.4 * math.sin(
                phase)
        elif mode == "march":
            # March in place — brisk alternating tripods, crisp partial
            # lifts, zero yaw sway.  Every step RE-PLANTS three feet at
            # the exact stance angles, so this act actively undoes the
            # outward foot-skate the loaded sway moves accumulate on
            # smooth floors (08-18 video analysis) — it self-corrects
            # while reading as a strut.
            group = 1.0 if (leg % 2) else -1.0
            phase = 3.4 * t
            raw = group * math.sin(phase)
            lift = 0.7 * max(0.0, raw) ** 0.45
            yaw_amp = 0.0
        elif mode == "fan":
            phase = 2.6 * t - leg * (math.pi / 3)
            lift = 0.5 + 0.5 * math.sin(phase)
            lift = max(0.0, min(1.0, lift))
            yaw_amp = RISE_SHOW_WAVE_YAW_DEG * math.sin(
                2.6 * t + leg)
        else:
            lift, yaw_amp = 0.0, 0.0
        y, h, k = _show_blend_leg(
            hip, knee, leg, lift, wave_yaw=yaw_amp)
        _set_leg(pose, leg, yaw=y, hip=h, knee=k)
    return pose


def _stream_multi_leg(bus: FeetechBus, live: set[int], *,
                      check, peaks, hip: float, knee: float,
                      seconds: float, mode: str,
                      label: str) -> bool:
    """Continuous multi-leg planted animation (several feet in the air)."""
    print(f"  → {label} ({seconds:.1f}s)")
    streamer = PoseStreamer()
    streamer.reset()
    dt = RISE_SHOW_DT
    n = max(1, int(seconds / dt))
    # Hotter stream profile for the show (restore after).
    global MAX_STREAM_SPEED, MIN_STREAM_SPEED, DEADBAND_DEG
    old_max, old_min, old_db = MAX_STREAM_SPEED, MIN_STREAM_SPEED, DEADBAND_DEG
    MAX_STREAM_SPEED = 900
    MIN_STREAM_SPEED = 80
    # Show moves are big/fast; deadband scales with tick like the profile.
    DEADBAND_DEG = DENSE_DEADBAND_DEG if STREAM_DENSE else 0.45
    t0 = time.monotonic()
    try:
        for i in range(n):
            if check():
                _hold_here(bus, live)
                return False
            t = i * dt
            pose = _show_stream_pose(mode, t, hip, knee)
            streamer.write(bus, pose, live, dt=dt)
            if peaks is not None and (i % 2 == 0):
                peaks.sample(bus, live)
            sleep_for = (t0 + (i + 1) * dt) - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
    finally:
        MAX_STREAM_SPEED = old_max
        MIN_STREAM_SPEED = old_min
        DEADBAND_DEG = old_db
    return True


def run_rise_show_demo(bus: FeetechBus, *,
                       abort_check=None,
                       rise_seconds: float | None = None,
                       descend_seconds: float | None = None,
                       speed: float = 1.0,
                       log_path: Path | None = None) -> str:
    """Next-level friend show — multi-leg, fast, tether-safe.

    Many legs move at once (ripples / gallops / tripod flips).  Body yaw
    oscillates but averages to 0 — no walking, no cord wind-up.
    """
    sc = _speed_scale(speed)
    hip = RISE_HIGH_HIP_DEG
    knee = RISE_HIGH_KNEE_DEG
    plant_s = float(rise_seconds if rise_seconds is not None
                    else RISE_SHOW_PLANT_S) * sc
    descend_s = float(descend_seconds if descend_seconds is not None
                      else RISE_SHOW_DESCEND_S) * sc
    snap = RISE_SHOW_SNAP_S * sc
    look = RISE_SHOW_LOOK_YAW_DEG
    twist = RISE_TURN_YAW_DEG
    bounce_knee = max(56.0, knee - RISE_SHOW_BOUNCE_KNEE_DELTA)
    nod_hip = hip - RISE_SHOW_NOD_HIP_DELTA

    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 robot IDs answering; moving "
              f"those only: {sorted(live)}")

    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
    if check():
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    peaks = CurrentPeakTracker()
    if log_path is not None:
        peak_path = (log_path.with_name(log_path.stem + "_peaks.csv")
                     if log_path.suffix else Path(str(log_path) + "_peaks.csv"))
    else:
        peak_path = default_log_path("rise_show_current_peaks")

    overhead = _zero_pose()
    planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
    squatted = _elevated_stand_pose(hip=hip, knee=bounce_knee, yaw=0.0)
    tall = _elevated_stand_pose(hip=min(28.0, hip + 6.0),
                                knee=max(64.0, knee - 8.0), yaw=0.0)
    nodded = _elevated_stand_pose(hip=nod_hip, knee=knee, yaw=0.0)
    look_l = _elevated_stand_pose(hip=hip, knee=knee, yaw=+look)
    look_r = _elevated_stand_pose(hip=hip, knee=knee, yaw=-look)
    twist_p = _elevated_stand_pose(hip=hip, knee=knee, yaw=+twist)
    twist_n = _elevated_stand_pose(hip=hip, knee=knee, yaw=-twist)
    # Simultaneous multi-leg stills (3+ legs up).
    odds_up = _planted_legs_up(hip, knee, [1, 3, 5])
    evens_up = _planted_legs_up(hip, knee, [0, 2, 4])
    pair_03 = _planted_legs_up(hip, knee, [0, 3])
    pair_14 = _planted_legs_up(hip, knee, [1, 4])
    pair_25 = _planted_legs_up(hip, knee, [2, 5])
    # Star: every other + one neighbor flourish
    star_a = _planted_legs_up(hip, knee, [0, 2, 4])
    star_b = _planted_legs_up(hip, knee, [1, 3, 5])

    print("  ★ RISE SHOW — next level (cords plugged in, multi-leg)")
    print(f"    speed {_clamp_demo_speed(speed):.2f}× — jazz → dive → acts → finale")

    def snap_to(goal, label, seconds=snap) -> bool:
        ok = _show_snap(bus, goal, live, check=check, peaks=peaks,
                        seconds=seconds, label=label)
        if not ok:
            _set_torque_limit(bus, live, 1000)
        return ok

    def stream(mode, seconds, label) -> bool:
        ok = _stream_multi_leg(
            bus, live, check=check, peaks=peaks, hip=hip, knee=knee,
            seconds=seconds, mode=mode, label=label)
        if not ok:
            _set_torque_limit(bus, live, 1000)
        return ok

    # --- OPENER: all legs overhead + streaming jazz -----------------------
    if not snap_to(overhead, "LEGS UP!", seconds=RISE_SHOW_OVERHEAD_S * sc):
        peaks.write_log(peak_path, phase="overhead (aborted)")
        return "aborted"
    if not stream("jazz", 1.6 * sc, "overhead jazz (all legs)"):
        return "aborted"
    if not snap_to(overhead, "ready…", seconds=0.25 * sc):
        return "aborted"

    # --- DIVE to plant (contact-aware once) --------------------------------
    print("  → dive to plant")
    if not _planted_glide(bus, planted, check=check, peaks=peaks,
                          seconds=plant_s, label="plant", contact=True):
        peaks.write_log(peak_path, phase="plant (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    peaks.print_report(phase="plant")

    # Power bounce (all legs together).
    for _ in range(3):
        if not snap_to(squatted, "BOOM", seconds=0.22 * sc):
            return "aborted"
        if not snap_to(tall, "POP", seconds=0.22 * sc):
            return "aborted"
    if not snap_to(planted, "plant", seconds=0.22 * sc):
        return "aborted"

    # Look + nod (still all legs).
    if not snap_to(look_l, "look L", seconds=0.28 * sc):
        return "aborted"
    if not snap_to(look_r, "look R", seconds=0.32 * sc):
        return "aborted"
    if not snap_to(planted, "center", seconds=0.22 * sc):
        return "aborted"
    if not snap_to(nodded, "nod", seconds=0.25 * sc):
        return "aborted"
    if not snap_to(planted, "plant", seconds=0.22 * sc):
        return "aborted"

    # --- MULTI-LEG STREAMED ACTS (the crowd-pleasers) ---------------------
    if not stream("ripple", 3.2 * sc, "ripple — 2–3 legs flying around the hex"):
        return "aborted"
    if not snap_to(planted, "plant", seconds=0.2 * sc):
        return "aborted"

    if not stream("gallop", 2.8 * sc, "gallop — opposite pairs together"):
        return "aborted"
    if not snap_to(planted, "plant", seconds=0.2 * sc):
        return "aborted"

    if not stream("tripod", 2.6 * sc, "tripod flip — three up / three down"):
        return "aborted"
    if not snap_to(planted, "plant", seconds=0.2 * sc):
        return "aborted"

    # Hard cuts between simultaneous stills (very readable).
    for goal, label in (
        (odds_up, "ODDS UP"),
        (evens_up, "EVENS UP"),
        (odds_up, "ODDS UP"),
        (pair_03, "pair 0+3"),
        (pair_14, "pair 1+4"),
        (pair_25, "pair 2+5"),
        (star_a, "STAR A"),
        (star_b, "STAR B"),
        (star_a, "STAR A"),
        (planted, "plant"),
    ):
        if not snap_to(goal, label, seconds=0.26 * sc):
            return "aborted"

    if not stream("fan", 2.4 * sc, "fan — all six dancing"):
        return "aborted"
    if not snap_to(planted, "plant", seconds=0.2 * sc):
        return "aborted"

    # Body twist (small, always returns).
    if not snap_to(twist_p, "twist +", seconds=RISE_TURN_SECONDS * sc):
        return "aborted"
    if not snap_to(twist_n, "twist −", seconds=RISE_TURN_SECONDS * sc):
        return "aborted"
    if not snap_to(planted, "untwist", seconds=0.3 * sc):
        return "aborted"

    # Finale barrage.
    if not stream("gallop", 1.4 * sc, "finale gallop"):
        return "aborted"
    for _ in range(4):
        if not snap_to(squatted, "STOMP", seconds=0.16 * sc):
            return "aborted"
        if not snap_to(planted, "STOMP", seconds=0.16 * sc):
            return "aborted"
    if not snap_to(odds_up, "★ TA-DA ★", seconds=0.35 * sc):
        return "aborted"
    if _planted_pause(bus, live, check, peaks, 0.55 * sc):
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    if not snap_to(planted, "finish", seconds=0.3 * sc):
        return "aborted"

    return _finish_planted_demo(bus, live, check, peaks, peak_path,
                                descend_s, "rise_show+descend")


def frames_air_wave(seconds: float = 3.0):
    """Counter-rotating spiral waves — evens go one way, odds the other.

    Even legs ride the traveling wave clockwise, odd legs counter-
    clockwise, so neighbouring legs always move in opposite directions.
    Hips are biased UP (−36..+8°) with the knee phase-locked so the
    feet stay off the floor — this plays sitting on the ground, not on
    the stand.  Amplitude ramps 0.3 → 1.0 (waking up).
    """
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        a = 0.3 + 0.7 * min(1.0, t / max(0.4, seconds * 0.5))
        pose = _zero_pose()
        for leg in range(6):
            d = 1.0 if leg % 2 == 0 else -1.0
            ph = 2.6 * t * d - leg * (math.pi / 3.0)
            _yaw_hip_knee(leg, pose,
                          yaw=16.0 * a * d * math.sin(ph + 0.5),
                          hip=-14.0 - 22.0 * a * math.sin(ph),
                          knee=8.0 + 24.0 * a * math.sin(ph - 0.8))
        yield pose


# Per-joint frequencies (Hz) and golden-angle phases for air chaos —
# deterministic but incommensurate, so no two joints ever sync up.
_CHAOS_GOLD = 2.399963  # golden angle, rad
_CHAOS_FREQ = [0.38, 0.55, 0.47, 0.63, 0.41, 0.71]


def frames_air_chaos(seconds: float = 4.5):
    """Everything moving in a different direction — 18 independent
    joints.

    Each yaw, hip and knee gets its own frequency (from an
    incommensurate set) and a golden-angle phase, so no two joints move
    together and the whole body churns like static.  Hips stay lifted
    (−38..−6°) so the feet wave in the air instead of scraping the
    floor.  A slow swell (0.4 → 1.0 over ~2 s) keeps it musical, and
    the last ~0.7 s FREEZES mid-churn — a statue beat before the
    canon explodes.
    """
    n = max(1, int(seconds / DT))
    n_freeze = min(int(0.7 / DT), n // 4)
    last: list[float] | None = None
    for i in range(n):
        if last is not None and i >= n - n_freeze:
            yield list(last)
            continue
        t = i * DT
        a = 0.4 + 0.6 * min(1.0, t / 2.0)
        pose = _zero_pose()
        for leg in range(6):
            j = leg * 3
            wy = 2 * math.pi * _CHAOS_FREQ[j % 6]
            wh = 2 * math.pi * _CHAOS_FREQ[(j + 1) % 6]
            wk = 2 * math.pi * _CHAOS_FREQ[(j + 2) % 6]
            _yaw_hip_knee(
                leg, pose,
                yaw=18.0 * a * math.sin(wy * t + j * _CHAOS_GOLD),
                hip=-22.0 - 16.0 * a * math.sin(wh * t + (j + 1) * _CHAOS_GOLD),
                knee=6.0 + 20.0 * a * math.sin(wk * t + (j + 2) * _CHAOS_GOLD))
        last = pose
        yield pose


def frames_air_converge(seconds: float = 7.0):
    """Six arms doing six different things — then MEETING overhead.

    Phase 1 (~50%): the chaos vocabulary — every joint on its own
    incommensurate frequency, no two arms alike.  Phase 2 (~22%): all
    six arms LOCK onto the overhead pose TOGETHER in one simultaneous
    sweep (operator request 08-18: everything snaps into place at
    once, not one arm after another).  Phase 3 (last ~14%): one
    synchronized dip-and-lift at the top, landing as a single
    organism.  Ends exactly at ``_arms_up_pose()``.
    """
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        u = i / max(n - 1, 1)
        swell = 0.5 + 0.5 * min(1.0, t / 1.5)
        pose = _zero_pose()
        # Unified pulse at the top — starts only after every leg locked.
        pulse = (math.sin(math.pi * min(1.0, (u - 0.86) / 0.14))
                 if u >= 0.86 else 0.0)
        hip_t = ARMS_UP_HIP_DEG + 6.0 * pulse
        knee_t = ARMS_UP_KNEE_DEG - 4.0 * pulse
        for leg in range(6):
            j = leg * 3
            wy = 2 * math.pi * _CHAOS_FREQ[j % 6]
            wh = 2 * math.pi * _CHAOS_FREQ[(j + 1) % 6]
            wk = 2 * math.pi * _CHAOS_FREQ[(j + 2) % 6]
            yaw_c = 18.0 * swell * math.sin(wy * t + j * _CHAOS_GOLD)
            hip_c = (-22.0 - 16.0 * swell
                     * math.sin(wh * t + (j + 1) * _CHAOS_GOLD))
            knee_c = (6.0 + 20.0 * swell
                      * math.sin(wk * t + (j + 2) * _CHAOS_GOLD))
            # Simultaneous lock (operator 08-18: all at once, not one
            # after another): every leg shares the same window, chaos
            # until 50% then a ~1.5 s cosine-eased sweep onto the
            # overhead pose (keeps peaks well under the servos'
            # ~132 deg/s tracking; the old 0.6 s staggered clicks were
            # rate-checked at 1x too).
            b = min(1.0, max(0.0, (u - 0.50) / 0.22))
            b = 0.5 - 0.5 * math.cos(math.pi * b)
            _yaw_hip_knee(
                leg, pose,
                yaw=yaw_c * (1.0 - b),
                hip=hip_c * (1.0 - b) + hip_t * b,
                knee=knee_c * (1.0 - b) + knee_t * b)
        yield pose



def frames_air_canon(seconds: float = 3.0):
    """Follow-the-leader in the air — big gesture, decaying echoes.

    Leg 0 leads a hip-dip + knee-curl + yaw-sweep gesture; each next
    leg around the hex repeats it one offset later and ~12% smaller.
    Narrow envelope → the gesture visibly travels leg to leg.
    """
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        pose = _zero_pose()
        for leg in range(6):
            decay = 1.0 - 0.12 * leg
            ph = 2.1 * t - leg * (2.0 * math.pi / 6.0)
            env = decay * max(0.0, math.sin(ph)) ** 2.0
            _yaw_hip_knee(leg, pose,
                          yaw=22.0 * env * math.sin(2.0 * ph),
                          hip=-45.0 * env,
                          knee=55.0 * env)
        yield pose


def frames_wave_goodbye(seconds: float = 1.8):
    """One last wave — leg 0 lifts and waves; everyone else asleep.

    Runs at sit zero after the final heartbeats.  Lift/lower envelopes
    (0.4 s each) mean it starts and ends exactly at zero.
    """
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        env = min(1.0, t / 0.4, max(0.0, (seconds - t) / 0.4))
        pose = _zero_pose()
        _yaw_hip_knee(0, pose,
                      yaw=18.0 * env * math.sin(2 * math.pi * 1.4 * t),
                      hip=-32.0 * env,
                      knee=12.0 * env)
        yield pose


def frames_overhead_sway(seconds: float = 3.0):
    """Arms-overhead shimmer — traveling yaw wave + soft knee pulse.

    Runs around the arms_up pose (all six legs folded overhead); the
    pre-rise 'inhale' moment of the dance.
    """
    base = _arms_up_pose()
    n = max(1, int(seconds / DT))
    for i in range(n):
        t = i * DT
        pose = list(base)
        for leg in range(6):
            yaw = 8.0 * math.sin(2 * math.pi * 0.6 * t - leg * math.pi / 3)
            knee = 4.0 * math.sin(2 * math.pi * 0.6 * t)
            _yaw_hip_knee(leg, pose, yaw=yaw, knee=knee)
        yield pose


def run_dance_demo(bus: FeetechBus, *,
                   abort_check=None,
                   speed: float = 1.0,
                   size: float = 1.0,
                   softness: float = 1.0,
                   torque: int | None = None,
                   status_cb=None,
                   part: str = "full",
                   standup_fn=None,
                   log_path: Path | None = None) -> str:
    """Full dance routine — quiet heartbeat to wild show and back to sleep.

    Act I    heartbeat pulses, then breaths that grow (``size`` scales)
    Act II   air show: counter-rotating wave, air chaos (all 18 joints
             independent — everything moving in a different direction),
             then the big air canon
    Act III  the MEET: six arms wild and different, locking overhead
             one at a time, one synchronized pulse at the top — then
             overhead sway (the inhale)
    Act IV   THE STAND-UP — arms sweep down, then the standup lab's
             STEP routine (tripod tuck + push, via ``standup_fn`` from
             the bench); CLI without a bench falls back to the old
             slow contact-aware reach
    Act V    wild planted acts at DANCE_PLANT_TORQUE (950 — the 700
             rise cap sagged 15-20 deg under load, 08-18 video):
             bounce, look, orbit sway, MARCH in place, ripple, canon
             (leg 0 leads — others echo at staggered offsets), gallop,
             tripod, slow stretch → DROP, standing hands-up feature,
             counterwave, fan, twist, accelerating stomp drumroll →
             dead stop → TA-DA.  Each streamed act ends in a tripod
             double-stomp RE-PLANT (geometry reset as percussion —
             clears the foot-skate that widened the stance open-loop)
    Act VI   slow descend to sit, last heartbeats, wave goodbye,
             one exhale, limp

    Starts and ends at sit zero (air home).  All durations pre-scale
    with ``speed`` (choreographed times — no live tempo, like
    rise_show); tether-safe (yaw always returns to 0, no walking).

    ``status_cb`` (optional callable) receives a short annotation
    string at each phase — the web UI shows it as the live demo
    status.  The CLI already narrates via prints.

    ``part`` splits the routine for the bench's dance+walk show:
    ``"full"`` (default) runs everything; ``"show"`` runs acts I–V and
    returns ``"planted"`` still standing and holding (torque ON) so
    the caller can hand off to the RL victory lap; ``"outro"`` assumes
    a planted stance and runs act VI only (descend → heartbeats →
    exhale → limp).

    ``standup_fn`` (optional, bench-provided): zero-arg callable that
    plays the standup lab's STEP stand-up inline and returns
    ``(ok, err)``.  When given, act IV uses it (operator 08-18: much
    better than the friction rise, and no settle-timeout pause after
    the stand).  On failure the dance STOPS safely — no improvised
    blend retries (hardware-safety rule).
    """
    if part not in ("full", "show", "outro"):
        raise ValueError(f"bad dance part {part!r}")
    sc = _speed_scale(speed)
    spd = _clamp_demo_speed(speed)
    size = _clamp_breathe_size(size)
    soft = max(0.5, min(3.0, float(softness)))
    hip = RISE_HIP_DEG
    knee = RISE_KNEE_DEG

    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 robot IDs answering; moving "
              f"those only: {sorted(live)}")

    check = abort_check or (lambda: False)
    try:
        tlim = int(torque) if torque is not None else DEMO_TORQUE_LIMIT
    except (TypeError, ValueError):
        tlim = DEMO_TORQUE_LIMIT
    tlim = max(150, min(1000, tlim))

    peaks = CurrentPeakTracker()
    if log_path is not None:
        peak_path = (log_path.with_name(log_path.stem + "_peaks.csv")
                     if log_path.suffix else Path(str(log_path) + "_peaks.csv"))
    else:
        peak_path = default_log_path("dance_current_peaks")

    def note(msg: str) -> None:
        """Live phase annotation (web status line); must never break the dance."""
        if status_cb is None:
            return
        try:
            status_cb(str(msg))
        except Exception:
            pass

    def bail(phase: str) -> str:
        peaks.write_log(peak_path, phase=f"dance {phase} (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    _enable_torque(bus, live)
    _set_torque_limit(bus, live, tlim)
    if check():
        _hold_here(bus, live)
        return bail("start")

    print("  ♥ DANCE — heartbeat → breathe → hands up → RISE → wild → sleep")
    print(f"    speed {spd:.2f}×  breath size {size:.2f}×  (~75 s at 1×)")

    glide_kw = dict(
        softness=soft,
        max_acc=max(3, int(10 / soft)),
        max_speed=max(60, int(220 / (0.7 + 0.3 * soft))),
    )

    def act6() -> str:
        """Act VI: descend from the plant, last heartbeats, sleep."""
        print("  → act VI — descend, last heartbeats, sleep")
        note(f"act VI — slow descend to sit "
             f"(~{max(4.0, DANCE_DESCEND_S * sc):.0f}s)")
        # Show torque (DANCE_PLANT_TORQUE) stays on while lowering the
        # body; the soft air cap returns right after.
        if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                            seconds=max(4.0, DANCE_DESCEND_S * sc),
                            label="act VI — descend to sit",
                            current_tracker=peaks):
            return bail("act6 descend")
        _set_torque_limit(bus, live, tlim)
        note("act VI — last heartbeats")
        if not _run_frames(bus, live,
                           frames_heartbeat(
                               seconds=DANCE_OUTRO_HEARTBEAT_S * sc),
                           check, label="act VI — last heartbeats"):
            return bail("act6 heartbeat")
        # One last wave from leg 0 — then sleep for real.
        note("act VI — wave goodbye")
        if not _run_frames(bus, live,
                           frames_wave_goodbye(seconds=max(1.4, 1.8 * sc)),
                           check, label="act VI — wave goodbye"):
            return bail("act6 wave")
        # One long exhale, then still.
        note("act VI — one long exhale… asleep")
        exhale = _breathe_pose(1.0, hip_deg=BREATHE_HIP_DEG * 0.6,
                               knee_deg=BREATHE_KNEE_DEG * 0.6)
        if not _soft_glide(bus, exhale, live, max(1.6, 2.2 * sc), check,
                           **glide_kw):
            return bail("act6 exhale")
        if not _soft_glide(bus, _zero_pose(), live, max(1.6, 2.6 * sc),
                           check, start=exhale, **glide_kw):
            return bail("act6 exhale")

        peaks.print_report(phase="dance")
        peaks.write_log(peak_path, phase="dance")
        _limp_all(bus, live)
        _set_torque_limit(bus, live, 1000)
        print("  Dance finished — asleep at sit zero (limp).")
        return "done"

    if part == "outro":
        # Victory-lap handoff: resume from a planted stance, act VI only.
        _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
        return act6()

    # --- Act I: asleep — heartbeat, then breaths that grow ----------------
    note("act I — asleep: settling at sit zero")
    if not _soft_glide(bus, _zero_pose(), live, 1.0 * sc, check, **glide_kw):
        return bail("act1 settle")
    note("act I — heartbeat (soft double thumps)")
    if not _run_frames(bus, live,
                       frames_heartbeat(seconds=DANCE_HEARTBEAT_S * sc),
                       check, label="act I — heartbeat"):
        return bail("act1 heartbeat")

    open_pose = _zero_pose()
    here = open_pose
    for i, growth in enumerate(DANCE_BREATH_GROWTH, 1):
        amp = min(2.5, growth * size)
        peak = _breathe_pose(1.0, hip_deg=BREATHE_HIP_DEG * amp,
                             knee_deg=BREATHE_KNEE_DEG * amp)
        half = max(1.0, DANCE_BREATH_HALF_S * sc)
        note(f"act I — breath {i}/{len(DANCE_BREATH_GROWTH)} "
             f"(deeper, {amp:.1f}×)")
        print(f"  → act I — breath {i} ({amp:.2f}×)")
        if not _soft_glide(bus, peak, live, half, check,
                           start=here, **glide_kw):
            return bail("act1 breathe")
        if not _soft_glide(bus, open_pose, live, half, check,
                           start=peak, **glide_kw):
            return bail("act1 breathe")
        here = open_pose

    # --- Act II: air show — big free-range choreography --------------------
    for frame_fn, label, secs in (
        (frames_air_wave,
         "act II — air wave: evens spin one way, odds the other",
         DANCE_AIR_WAVE_S),
        (frames_air_chaos,
         "act II — AIR CHAOS: all 18 joints, no two alike",
         DANCE_AIR_CHAOS_S),
        (frames_air_canon,
         "act II — air canon: leg 0 leads, others echo big",
         DANCE_AIR_CANON_S),
    ):
        note(label)
        if not _run_frames(bus, live, frame_fn(seconds=secs * sc),
                           check, label=label):
            return bail("act2")

    # --- Act III: the MEET — six wild arms lock overhead one by one --------
    note("act III — six arms wild… then all MEET at the top")
    if not _run_frames(bus, live,
                       frames_air_converge(
                           seconds=DANCE_AIR_CONVERGE_S * sc),
                       check, label="act III — the meet (converge overhead)"):
        return bail("act3 converge")
    note("act III — overhead sway (the inhale)")
    if not _run_frames(bus, live,
                       frames_overhead_sway(
                           seconds=DANCE_OVERHEAD_SWAY_S * sc),
                       check, label="act III — overhead sway"):
        return bail("act3 sway")
    # Re-square overhead: one beat of stillness before the drop.
    note("act III — still… (before the drop)")
    if not _soft_glide(bus, _arms_up_pose(), live,
                       max(0.6, DANCE_OVERHEAD_HOLD_S * sc), check,
                       **glide_kw):
        return bail("act3 hold")

    # --- Act IV: THE STAND-UP ----------------------------------------------
    _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
    planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
    if standup_fn is not None:
        # STEP stand-up (standup lab / experiments tab): arms sweep
        # down from overhead to the step start (legs straight out) as
        # the drop beat, then the lab's pursuit streamer tucks the
        # feet in one tripod at a time and pushes straight up. No
        # settle-timeout pause afterwards — straight into act V.
        note("act IV — the DROP: arms sweep to the floor")
        if not ease_to_pose(bus, _zero_pose(), abort_check=check,
                            seconds=max(0.9, 1.3 * sc),
                            label="act IV — sweep down",
                            current_tracker=peaks):
            return bail("act4 sweep")
        note("act IV — STEP stand-up (tripod tuck, push up)")
        ok, err = standup_fn()
        if check():
            return bail("act4 standup")
        if not ok:
            # Safety rule: a refused/tripped stand-up ends the dance
            # here — no improvised blend retries from an unknown pose.
            note(f"act IV — stand-up stopped: {err}")
            print(f"  step stand-up failed ({err}) — stopping the dance")
            return bail("act4 standup")
        _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
        if not ease_to_pose(bus, planted, abort_check=check,
                            seconds=0.7,
                            label="act IV — set the stance",
                            current_tracker=peaks):
            return bail("act4 stance")
    else:
        # CLI path (no bench): the original slow contact-aware reach
        # straight from overhead.
        note(f"act IV — THE RISE (~{max(3.0, RISE_SECONDS * sc):.0f}s "
             "reach down to plant)")
        if not _planted_glide(bus, planted, check=check, peaks=peaks,
                              seconds=max(3.0, RISE_SECONDS * sc),
                              label="act IV — THE RISE", contact=True):
            return bail("act4 rise")
    peaks.print_report(phase="rise")
    # Stiffen for the show: the 700 rise cap sagged visibly (see
    # DANCE_PLANT_TORQUE note); the acts need holding force, not the
    # rise's contact sensitivity.
    _set_torque_limit(bus, live, DANCE_PLANT_TORQUE)

    # --- Act V: wild (planted, rise_show vocabulary) ------------------------
    snap = RISE_SHOW_SNAP_S * sc
    look = RISE_SHOW_LOOK_YAW_DEG
    twist = RISE_TURN_YAW_DEG
    bounce_knee = max(56.0, knee - RISE_SHOW_BOUNCE_KNEE_DELTA)
    squatted = _elevated_stand_pose(hip=hip, knee=bounce_knee, yaw=0.0)
    tall = _elevated_stand_pose(hip=min(28.0, hip + 6.0),
                                knee=max(64.0, knee - 8.0), yaw=0.0)
    nodded = _elevated_stand_pose(hip=hip - RISE_SHOW_NOD_HIP_DELTA,
                                  knee=knee, yaw=0.0)
    look_l = _elevated_stand_pose(hip=hip, knee=knee, yaw=+look)
    look_r = _elevated_stand_pose(hip=hip, knee=knee, yaw=-look)
    twist_p = _elevated_stand_pose(hip=hip, knee=knee, yaw=+twist)
    twist_n = _elevated_stand_pose(hip=hip, knee=knee, yaw=-twist)
    odds_up = _planted_legs_up(hip, knee, [1, 3, 5])
    evens_up = _planted_legs_up(hip, knee, [0, 2, 4])

    def snap_to(goal, label, seconds=snap) -> bool:
        ok = _show_snap(bus, goal, live, check=check, peaks=peaks,
                        seconds=seconds, label=label)
        if not ok:
            _set_torque_limit(bus, live, 1000)
        return ok

    def stream(mode, seconds, label) -> bool:
        ok = _stream_multi_leg(
            bus, live, check=check, peaks=peaks, hip=hip, knee=knee,
            seconds=seconds, mode=mode, label=label)
        if not ok:
            _set_torque_limit(bus, live, 1000)
        return ok

    stomp_flip = [False]

    def replant_stomp() -> bool:
        """Geometry reset AS a beat: tripod double-stomp onto the marks.

        The loaded sway moves skate the feet outward a few mm per beat
        and nothing re-plants them open-loop (08-18 video: the stance
        ratchets wider through act V).  Lifting each tripod and snapping
        it back down at the exact stance angles clears the slide — it
        reads as stomp-stomp-stick punctuation, not a correction.
        Alternates which tripod leads so the percussion isn't samey.
        """
        first, second = ((odds_up, evens_up) if stomp_flip[0]
                         else (evens_up, odds_up))
        stomp_flip[0] = not stomp_flip[0]
        for goal, label, s in ((first, "re-STOMP", 0.20),
                               (planted, "set", 0.15),
                               (second, "STOMP", 0.20),
                               (planted, "stick!", 0.15)):
            if not snap_to(goal, label, seconds=s * sc):
                return False
        return True

    # Power bounce.
    note("act V — wild: power bounce ×3")
    for _ in range(3):
        if not snap_to(squatted, "BOOM", seconds=DANCE_SNAP_S * sc):
            return bail("act5 bounce")
        if not snap_to(tall, "POP", seconds=DANCE_SNAP_S * sc):
            return bail("act5 bounce")
    if not snap_to(planted, "plant", seconds=DANCE_SNAP_S * sc):
        return bail("act5 bounce")

    # Look + nod (personality beat).
    note("act V — look left/right + nod")
    for goal, label, s in ((look_l, "look L", 0.22), (look_r, "look R", 0.26),
                           (planted, "center", 0.18), (nodded, "nod", 0.2),
                           (planted, "plant", 0.18)):
        if not snap_to(goal, label, seconds=s * sc):
            return bail("act5 look")

    # Streamed crowd-pleasers — heavy on all-legs-different material:
    # orbit (every leg its own 60° phase), canon (leg 0 leads, the rest
    # echo at staggered offsets), plus the classic waves.
    for mode, secs, label in (
        ("orbit", 2.8, "orbit sway — body leans in a slow circle"),
        ("march", 2.4, "march in place — every step re-plants"),
        ("ripple", 2.4, "ripple — legs flying around"),
        ("canon", 3.6, "canon — leg 0 leads, others echo in turn"),
        ("gallop", 2.2, "gallop — opposite pairs"),
        ("tripod", 2.0, "tripod flip — 3 up / 3 down"),
    ):
        note(f"act V — {label}")
        if not stream(mode, secs * sc, label):
            return bail("act5 stream")
        # Stomp re-plant instead of a plain snap: clears the foot slide
        # the act just skated in (marches/tripods barely need it; the
        # loaded sways really do).
        if not replant_stomp():
            return bail("act5 stream")

    # Contrast beat: one luxurious slow stretch to full height… BAM.
    note("act V — sloooow stretch up…")
    if not snap_to(tall, "slow stretch…", seconds=1.3 * sc):
        return bail("act5 stretch")
    if _planted_pause(bus, live, check, peaks, 0.35 * sc):
        return bail("act5 stretch")
    note("act V — …DROP!")
    if not snap_to(squatted, "DROP!", seconds=0.12 * sc):
        return bail("act5 stretch")
    if not snap_to(planted, "plant", seconds=DANCE_SNAP_S * sc):
        return bail("act5 stretch")

    # Standing hands-up feature — echo of act III on a stable tripod.
    note("act V — HANDS UP, standing on a tripod")
    if not snap_to(evens_up, "HANDS UP (tripod)", seconds=0.4 * sc):
        return bail("act5 hands")
    if _planted_pause(bus, live, check, peaks, DANCE_HANDS_HOLD_S * sc):
        return bail("act5 hands")
    for goal, label in ((odds_up, "SWITCH"), (evens_up, "SWITCH"),
                        (planted, "plant")):
        if not snap_to(goal, label, seconds=0.22 * sc):
            return bail("act5 hands")

    note("act V — counterwave: two waves, no two legs alike")
    if not stream("counterwave", 3.0 * sc,
                  "counterwave — knees ride CW, yaws ride CCW"):
        return bail("act5 counterwave")
    if not replant_stomp():
        return bail("act5 counterwave")

    note("act V — fan: all six dancing")
    if not stream("fan", 1.8 * sc, "fan — all six dancing"):
        return bail("act5 fan")
    if not replant_stomp():
        return bail("act5 fan")

    # Body twist (small, always returns to 0 — tether-safe).  Twists are
    # the worst foot-skaters (all six feet loaded and shearing), so a
    # stomp re-plant follows.
    note("act V — body twist (and back)")
    for goal, label in ((twist_p, "twist +"), (twist_n, "twist −"),
                        (planted, "untwist")):
        if not snap_to(goal, label, seconds=0.32 * sc):
            return bail("act5 twist")
    if not replant_stomp():
        return bail("act5 twist")

    # Finale: accelerating stomp drumroll → dead stop → TA-DA.
    note("act V — finale: stomp drumroll + TA-DA")
    for s in (0.20, 0.16, 0.13, 0.10):
        if not snap_to(squatted, "STOMP", seconds=s * sc):
            return bail("act5 stomp")
        if not snap_to(planted, "STOMP", seconds=s * sc):
            return bail("act5 stomp")
    # Dead stop — the silence that sells the TA-DA.
    if _planted_pause(bus, live, check, peaks, 0.5 * sc):
        return bail("act5 finale")
    if not snap_to(odds_up, "★ TA-DA ★", seconds=0.3 * sc):
        return bail("act5 finale")
    if _planted_pause(bus, live, check, peaks, 0.8 * sc):
        return bail("act5 finale")
    if not snap_to(planted, "plant", seconds=0.25 * sc):
        return bail("act5 finale")

    if part == "show":
        # Victory-lap handoff: stay planted and HOLDING (torque on) so
        # the bench can run the RL walk from this stance.
        note("show ends planted — holding for the victory lap")
        peaks.print_report(phase="dance show")
        peaks.write_log(peak_path, phase="dance show (planted)")
        print("  Dance show finished — holding planted for the lap.")
        return "planted"

    return act6()


# Open-loop walk demos (TripodGait → SyncWrite). Gentle defaults for
# first floor tests / short cord reach.
WALK_DEMO_DT = 0.05
WALK_DEMO_VX = 0.045          # m/s ≈ 45 mm/s
WALK_DEMO_OMEGA = 0.40        # rad/s in-place
WALK_DEMO_FORWARD_S = 5.0
WALK_DEMO_SPIN_S = 4.0
WALK_DEMO_LIFT_MM = 18.0


def run_walk_demo(bus: FeetechBus, name: str = "walk", *,
                  abort_check=None,
                  speed: float = 1.0,
                  seconds: float | None = None,
                  log_path: Path | None = None) -> str:
    """Tripod gait: forward / spin / short oval, then hold stand plant."""
    try:
        from tripod_gait import TripodGait
    except ImportError as e:
        print(f"  walk demo needs tripod_gait: {e}")
        return "skipped"

    sc = _speed_scale(speed)
    live = _live_robot_ids(bus)
    if len(live) < 12:
        print(f"  Only {len(live)} robot servo(s) — need most of a hex for walk.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 IDs answering; walking those only: "
              f"{sorted(live)}")

    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, STAND_TORQUE_LIMIT)

    stand = standing_pose_degrees()
    if not ease_to_pose(bus, stand, abort_check=check,
                        seconds=max(2.0, 3.0 * sc),
                        label="stand before walk"):
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    gait = TripodGait(period=0.85, lift=WALK_DEMO_LIFT_MM * 0.001, ramp=0.4)
    gait.sync_plant_stance()
    gait.set_lift_mm(WALK_DEMO_LIFT_MM)

    # Segments: (label, vx, vy, omega, duration_s)
    if name == "walk_spin":
        dur = float(seconds) if seconds is not None else WALK_DEMO_SPIN_S
        segments = [("spin L", 0.0, 0.0, WALK_DEMO_OMEGA, dur * sc)]
    elif name == "walk_oval":
        fwd = (float(seconds) if seconds is not None else 3.0) * sc
        spin = 2.2 * sc
        segments = [
            ("forward", WALK_DEMO_VX, 0.0, 0.0, fwd),
            ("spin", 0.0, 0.0, WALK_DEMO_OMEGA, spin),
            ("reverse", -WALK_DEMO_VX * 0.85, 0.0, 0.0, fwd * 0.9),
        ]
    else:
        dur = float(seconds) if seconds is not None else WALK_DEMO_FORWARD_S
        segments = [("forward", WALK_DEMO_VX, 0.0, 0.0, dur * sc)]

    print(f"  {name}: plant hip {gait.plant_hip_deg:.0f}° / "
          f"knee {gait.plant_knee_deg:.0f}° · "
          + " → ".join(f"{lab} {d:.1f}s" for lab, *_r, d in segments))

    log_cm = None
    if log_path is not None:
        log_cm = MotionLog(log_path, live)
        log_cm.__enter__()
    # The gait is time-based, so the tick only sets waypoint density —
    # dense on the stream bridge tracks the leg curves much closer.
    walk_dt = DENSE_DT if STREAM_DENSE else WALK_DEMO_DT
    log_every = max(1, round(WALK_DEMO_DT / walk_dt))
    t0 = time.monotonic()
    gait.reset_phase(t=t0)
    try:
        for label, vx, vy, om, dur in segments:
            if check():
                break
            print(f"    · {label}  vx={vx*1000:.0f} mm/s  ω={om:.2f}")
            gait.set_velocity(vx=vx, vy=vy, omega=om)
            seg_t0 = time.monotonic()
            tick_i = 0
            while time.monotonic() - seg_t0 < dur:
                if check():
                    break
                now = time.monotonic()
                pose = gait.desired_deg(now)
                _write_pose(bus, pose, live, speed=WALK_SPEED, acc=WALK_ACC)
                if log_cm is not None and tick_i % log_every == 0:
                    try:
                        log_cm.sample(bus, pose, wrote={})
                    except Exception:
                        pass
                tick_i += 1
                sleep_for = (seg_t0 + tick_i * walk_dt) - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
            if check():
                break
        gait.stop()
        # Settle onto plant (kill residual swing).
        for _ in range(8 * max(1, round(WALK_DEMO_DT / walk_dt))):
            if check():
                break
            now = time.monotonic()
            pose = gait.desired_deg(now)
            _write_pose(bus, pose, live, speed=WALK_SPEED, acc=WALK_ACC)
            time.sleep(walk_dt)
        if not ease_to_pose(bus, stand, abort_check=check,
                            seconds=max(1.2, 1.8 * sc),
                            label="stand after walk"):
            _set_torque_limit(bus, live, 1000)
            return "aborted"
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)
        _set_torque_limit(bus, live, 1000)

    if check():
        _hold_here(bus, live)
        return "aborted"
    print(f"  {name} done — holding stand.")
    _write_pose(bus, stand, live, speed=200, acc=20)
    return "done"


def run_dance_prance(bus: FeetechBus, phase: str = "out", *,
                     abort_check=None, status_cb=None) -> str:
    """Victory-lap gait phases — the aggressive open-loop tripod.

    ``phase="out"``: horse-prance forward (quick cadence, high knees).
    ``phase="halfturn"``: about-face — a sim-calibrated 180° spin in
    place, so the "home" leg walks FORWARD back toward the start.
    ``phase="home"``: prance back — identical gait and duration to
    "out", so the return distance matches by symmetry however much
    the feet slip on the day's floor.
    Each phase starts by easing onto the walk plant (stand zero) and
    ends HOLDING it, so phases chain cleanly.
    """
    assert phase in ("out", "halfturn", "home")
    try:
        from tripod_gait import TripodGait
    except ImportError as e:
        print(f"  prance needs tripod_gait: {e}")
        return "skipped"

    live = _live_robot_ids(bus)
    if len(live) < 12:
        print(f"  Only {len(live)} robot servo(s) — need most of a hex.")
        return "skipped"
    check = abort_check or (lambda: False)

    def note(msg: str) -> None:
        if status_cb is None:
            return
        try:
            status_cb(str(msg))
        except Exception:
            pass

    _enable_torque(bus, live)
    _set_torque_limit(bus, live, STAND_TORQUE_LIMIT)
    stand = standing_pose_degrees()
    if not ease_to_pose(bus, stand, abort_check=check, seconds=1.4,
                        label=f"stand before prance {phase}"):
        return "aborted"

    gait = TripodGait(period=DANCE_PRANCE_PERIOD,
                      lift=DANCE_PRANCE_LIFT_MM * 0.001, ramp=0.4)
    gait.sync_plant_stance()
    gait.set_lift_mm(DANCE_PRANCE_LIFT_MM)

    if phase == "out":
        segments = [("PRANCE — high knees, quick cadence",
                     DANCE_PRANCE_VX, 0.0, 0.0, DANCE_PRANCE_FWD_S)]
    elif phase == "halfturn":
        segments = [("ABOUT-FACE — half turn in place",
                     0.0, 0.0, DANCE_PRANCE_TURN_OMEGA,
                     DANCE_PRANCE_HALFTURN_S)]
    else:
        segments = [("PRANCE home — same strut back",
                     DANCE_PRANCE_VX, 0.0, 0.0, DANCE_PRANCE_FWD_S)]

    gait.reset_phase(t=time.monotonic())
    for label, vx, vy, om, dur in segments:
        if check():
            break
        note(f"victory lap — {label}")
        print(f"  → {label}  vx={vx * 1000:.0f} mm/s  ω={om:.2f}")
        gait.set_velocity(vx=vx, vy=vy, omega=om)
        seg_t0 = time.monotonic()
        while time.monotonic() - seg_t0 < dur:
            if check():
                break
            pose = gait.desired_deg(time.monotonic())
            _write_pose(bus, pose, live, speed=WALK_SPEED,
                        acc=DANCE_PRANCE_ACC)
            time.sleep(WALK_DEMO_DT)

    gait.stop()
    for _ in range(8):
        if check():
            break
        pose = gait.desired_deg(time.monotonic())
        _write_pose(bus, pose, live, speed=WALK_SPEED,
                    acc=DANCE_PRANCE_ACC)
        time.sleep(WALK_DEMO_DT)
    if check():
        _hold_here(bus, live)
        return "aborted"
    if not ease_to_pose(bus, stand, abort_check=check, seconds=1.2,
                        label=f"stand after prance {phase}"):
        return "aborted"
    return "done"


def run_planted_act_demo(bus: FeetechBus, act: str, *,
                         abort_check=None,
                         rise_seconds: float | None = None,
                         descend_seconds: float | None = None,
                         speed: float = 1.0,
                         log_path: Path | None = None) -> str:
    """Planted stand, then one exciting act from the rise_show toolkit."""
    if act not in PLANTED_ACTS:
        raise SystemExit(f"unknown planted act {act!r}")

    sc = _speed_scale(speed)
    hip = RISE_HIGH_HIP_DEG
    knee = RISE_HIGH_KNEE_DEG
    plant_s = float(rise_seconds if rise_seconds is not None
                    else RISE_SHOW_PLANT_S) * sc
    descend_s = float(descend_seconds if descend_seconds is not None
                      else RISE_SHOW_DESCEND_S) * sc
    snap = RISE_SHOW_SNAP_S * sc
    look = RISE_SHOW_LOOK_YAW_DEG
    bounce_knee = max(56.0, knee - RISE_SHOW_BOUNCE_KNEE_DELTA)
    nod_hip = hip - RISE_SHOW_NOD_HIP_DELTA

    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 robot IDs answering; moving "
              f"those only: {sorted(live)}")

    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    _set_torque_limit(bus, live, RISE_TORQUE_LIMIT)
    if check():
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    peaks = CurrentPeakTracker()
    if log_path is not None:
        peak_path = (log_path.with_name(log_path.stem + "_peaks.csv")
                     if log_path.suffix else Path(str(log_path) + "_peaks.csv"))
    else:
        peak_path = default_log_path(f"{act}_current_peaks")

    planted = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
    squatted = _elevated_stand_pose(hip=hip, knee=bounce_knee, yaw=0.0)
    tall = _elevated_stand_pose(hip=min(28.0, hip + 6.0),
                                knee=max(64.0, knee - 8.0), yaw=0.0)
    nodded = _elevated_stand_pose(hip=nod_hip, knee=knee, yaw=0.0)
    look_l = _elevated_stand_pose(hip=hip, knee=knee, yaw=+look)
    look_r = _elevated_stand_pose(hip=hip, knee=knee, yaw=-look)
    odds_up = _planted_legs_up(hip, knee, [1, 3, 5])
    evens_up = _planted_legs_up(hip, knee, [0, 2, 4])
    star_a = _planted_legs_up(hip, knee, [0, 2, 4])
    star_b = _planted_legs_up(hip, knee, [1, 3, 5])

    def snap_to(goal, label, seconds=snap) -> bool:
        ok = _show_snap(bus, goal, live, check=check, peaks=peaks,
                        seconds=seconds, label=label)
        if not ok:
            _set_torque_limit(bus, live, 1000)
        return ok

    def stream(mode, seconds, label) -> bool:
        ok = _stream_multi_leg(
            bus, live, check=check, peaks=peaks, hip=hip, knee=knee,
            seconds=seconds, mode=mode, label=label)
        if not ok:
            _set_torque_limit(bus, live, 1000)
        return ok

    title = DEMOS[act][0]
    print(f"  {act} — {title}")
    print(f"    quick plant, then the act (speed {_clamp_demo_speed(speed):.2f}×)")

    if not _planted_glide(bus, planted, check=check, peaks=peaks,
                          seconds=plant_s, label="plant", contact=True):
        peaks.write_log(peak_path, phase="plant (aborted)")
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    peaks.print_report(phase="plant")

    ok = True
    if act == "plant_look":
        ok = (snap_to(look_l, "look L", 0.35 * sc)
              and snap_to(look_r, "look R", 0.40 * sc)
              and snap_to(planted, "center", 0.28 * sc)
              and snap_to(nodded, "nod", 0.30 * sc)
              and snap_to(planted, "plant", 0.28 * sc))
    elif act == "plant_bounce":
        for _ in range(4):
            if not snap_to(squatted, "BOOM", 0.22 * sc):
                ok = False
                break
            if not snap_to(tall, "POP", 0.22 * sc):
                ok = False
                break
        if ok:
            ok = snap_to(planted, "plant", 0.25 * sc)
    elif act == "plant_ripple":
        ok = stream("ripple", 4.0 * sc, "standing ripple") and snap_to(
            planted, "plant", 0.25 * sc)
    elif act == "plant_gallop":
        ok = stream("gallop", 3.5 * sc, "standing gallop") and snap_to(
            planted, "plant", 0.25 * sc)
    elif act == "plant_tripod":
        ok = stream("tripod", 3.2 * sc, "tripod flip") and snap_to(
            planted, "plant", 0.25 * sc)
    elif act == "plant_fan":
        ok = stream("fan", 3.0 * sc, "fan dance") and snap_to(
            planted, "plant", 0.25 * sc)
    elif act == "plant_star":
        for goal, label in (
            (odds_up, "ODDS UP"),
            (evens_up, "EVENS UP"),
            (odds_up, "ODDS UP"),
            (star_a, "STAR A"),
            (star_b, "STAR B"),
            (star_a, "STAR A"),
            (planted, "plant"),
        ):
            if not snap_to(goal, label, 0.28 * sc):
                ok = False
                break
    elif act == "plant_stomp":
        ok = stream("gallop", 1.2 * sc, "stomp lead-in")
        if ok:
            for _ in range(5):
                if not snap_to(squatted, "STOMP", 0.16 * sc):
                    ok = False
                    break
                if not snap_to(planted, "STOMP", 0.16 * sc):
                    ok = False
                    break
        if ok:
            ok = (snap_to(odds_up, "★ TA-DA ★", 0.40 * sc)
                  and not _planted_pause(bus, live, check, peaks, 0.45 * sc)
                  and snap_to(planted, "finish", 0.30 * sc))

    if not ok:
        peaks.write_log(peak_path, phase=f"{act} (aborted)")
        return "aborted"
    return _finish_planted_demo(bus, live, check, peaks, peak_path,
                                descend_s, f"{act}+descend")


def run_demo(bus: FeetechBus, name: str, *,
             seconds: float | None = None,
             speed: float = 1.0,
             size: float = 1.0,
             rate: float | None = None,
             torque: int | None = None,
             softness: float = 1.0,
             abort_check=None,
             speed_fn=None,
             status_cb=None,
             log_path: Path | None = None,
             rise_high: bool = False,
             rise_fast: bool = False,
             standup_fn=None,
             quad_reared: bool = False) -> str:
    """Run one demo.  Returns ``done`` / ``aborted`` / ``skipped``.

    ``speed``: 1.0 = nominal, 2.0 = twice as fast, 0.5 = half speed.
    ``speed_fn``: LIVE tempo callable (web slider) — streamed demos read
    it every tick, breathe at each half-breath.  When given, ``speed``
    is just the initial value (durations are NOT pre-scaled).
    ``status_cb``: optional live status-string callback (web UI).
    ``size`` / ``rate`` / ``softness``: breathe tunables.
    ``torque``: SRAM torque limit for air demos (None → default).
    ``seconds`` (optional) still overrides absolute durations when set;
    otherwise durations come from defaults scaled by ``speed``.
    """
    if name == "breathe+":
        name = "breathe"
        size = max(_clamp_breathe_size(size), 2.0)
    if name not in DEMOS:
        raise SystemExit(f"unknown demo {name!r}; try: {', '.join(DEMOS)}")
    if name in QUAD_BLOCKED_HARDWARE_DEMOS:
        print("  aggressive quad walk/trot is blocked on hardware after "
              "the forward fall; use Safe/Cool on hardware or simulate it only")
        return "skipped"
    if configure_stream_profile(bus):
        print(f"  stream: DENSE waypoints ({1.0 / DT:.0f} Hz, "
              f"deadband {DEADBAND_DEG:.2f}°) — MCU stream bridge")
    else:
        print(f"  stream: legacy waypoints ({1.0 / DT:.1f} Hz)")
    sc = _speed_scale(speed)
    spd = _clamp_demo_speed(speed)
    if name in QUAD_REQUIRES_REAR and not quad_reared:
        print("  quad: rear up first, then walk/trot/down from the web Quad tab")
        return "skipped"
    if name in STREAM_POSE_FACTORIES:
        # Streamed engine (standing dances + air wiggles): host-owned
        # timing, live tempo — the stand-up lab technique.
        return run_streamed_demo(
            bus, name, seconds=seconds, speed=spd, speed_fn=speed_fn,
            torque=torque, abort_check=abort_check, status_cb=status_cb,
            log_path=log_path)
    if name in ("rise", "rise+"):
        use_high = name == "rise+" or rise_high
        use_fast = name == "rise+" or rise_fast
        hip = RISE_HIGH_HIP_DEG if use_high else RISE_HIP_DEG
        knee = RISE_HIGH_KNEE_DEG if use_high else RISE_KNEE_DEG
        rise_s = RISE_FAST_SECONDS if use_fast else RISE_SECONDS
        descend_s = (RISE_FAST_DESCEND_SECONDS if use_fast else DESCEND_SECONDS)
        if seconds is not None:
            rise_s = descend_s = float(seconds)
        else:
            rise_s *= sc
            descend_s *= sc
        print(f"  demo speed {spd:.2f}×")
        return run_rise_demo(
            bus, abort_check=abort_check,
            rise_seconds=rise_s, descend_seconds=descend_s,
            hip_deg=hip, knee_deg=knee,
            preset="high_fast" if use_high else "default",
            log_path=log_path)
    if name == "rise_turn":
        return run_rise_turn_demo(
            bus, abort_check=abort_check,
            rise_seconds=seconds, descend_seconds=seconds,
            speed=spd, log_path=log_path)
    if name == "rise_show":
        return run_rise_show_demo(
            bus, abort_check=abort_check,
            rise_seconds=seconds, descend_seconds=seconds,
            speed=spd, log_path=log_path)
    if name in ("dance", "dance_walk"):
        # Choreographed times (like rise_show) — live tempo not wired in.
        # dance_walk's victory lap is composed by the web bench (it owns
        # the safety plumbing); from the CLI it degrades to the dance.
        if name == "dance_walk":
            print("  (victory lap runs from the web bench only — "
                  "playing the full dance)")
        return run_dance_demo(
            bus, abort_check=abort_check, speed=spd, size=size,
            softness=softness, torque=torque, status_cb=status_cb,
            standup_fn=standup_fn, log_path=log_path)
    if name == "stand_hands":
        print(f"  demo speed {spd:.2f}×")
        return run_stand_hands_demo(
            bus, abort_check=abort_check, speed=spd, seconds=seconds)
    if name in ("walk", "walk_spin", "walk_oval"):
        print(f"  demo speed {spd:.2f}×")
        return run_walk_demo(
            bus, name, abort_check=abort_check, speed=spd,
            seconds=seconds, log_path=log_path)
    if name in PLANTED_ACTS:
        return run_planted_act_demo(
            bus, name, abort_check=abort_check,
            rise_seconds=seconds, descend_seconds=seconds,
            speed=spd, log_path=log_path)

    if seconds is not None:
        air_s = float(seconds)
    elif speed_fn is not None:
        # Live tempo owns the pace — don't pre-scale the duration too.
        air_s = AIR_DEMO_SECONDS.get(name, 7.0)
    else:
        air_s = AIR_DEMO_SECONDS.get(name, 7.0) * sc
    print(f"  demo speed {spd:.2f}×  ({air_s:.1f}s)")

    # Breathe: servo-side glides (streaming looks stepped on STS3215s).
    if name == "breathe":
        return run_breathe_demo(
            bus, seconds=air_s, abort_check=abort_check, size=size,
            rate=rate, torque=torque, softness=softness,
            speed_fn=speed_fn, log_path=log_path)
    if name == "breathe_v":
        return run_breathe_vel_demo(
            bus, seconds=air_s, abort_check=abort_check, size=size,
            rate=rate, torque=torque, softness=softness, log_path=log_path)
    if name == "shimmy_v":
        return run_shimmy_vel_demo(
            bus, seconds=air_s, abort_check=abort_check, log_path=log_path)
    if name == "arms_up":
        return run_arms_up_demo(
            bus, abort_check=abort_check, speed=spd, seconds=seconds,
            torque=torque, log_path=log_path)
    if name in ("dance_swarm", "dance_swarm_stand"):
        # Song-locked: ignores speed/seconds — the recording is the clock.
        # The standing version needs the bench standup_fn; without it,
        # it degrades to the seated show.
        return run_swarm_dance(
            bus, abort_check=abort_check, status_cb=status_cb,
            torque=torque, standup_fn=standup_fn,
            stand=(name == "dance_swarm_stand"), log_path=log_path)
    if name == "dance_wild":
        # Without the bench standup_fn it plays the seated acts only.
        return run_wild_dance(
            bus, abort_check=abort_check, status_cb=status_cb,
            torque=torque, speed=spd, speed_fn=speed_fn,
            standup_fn=standup_fn, log_path=log_path)
    if name == "dance_encore":
        # Without the bench standup_fn it plays the seated acts only.
        return run_encore_dance(
            bus, abort_check=abort_check, status_cb=status_cb,
            torque=torque, speed=spd, speed_fn=speed_fn,
            standup_fn=standup_fn, log_path=log_path)
    if name == "dance_steeple":
        if standup_fn is None:
            # CLI degrade: the seated trident (same pair vocabulary).
            print("  (stand-up runs from the web bench only — "
                  "playing the seated trident)")
            name = "air_trident"
            if seconds is None:
                air_s = AIR_DEMO_SECONDS[name] * (1.0 if speed_fn else sc)
        else:
            return run_steeple_dance(
                bus, abort_check=abort_check, status_cb=status_cb,
                torque=torque, speed=spd, speed_fn=speed_fn,
                standup_fn=standup_fn, log_path=log_path)

    title, frame_fn = DEMOS[name]
    live = _live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) on the bus — need more.")
        return "skipped"
    if len(live) < 18:
        print(f"  Note: {len(live)}/18 robot IDs answering; animating "
              f"those only: {sorted(live)}")

    check = abort_check or (lambda: False)
    _enable_torque(bus, live)
    # Soft torque for air demos — less violent hunt through backlash.
    try:
        tlim = int(torque) if torque is not None else DEMO_TORQUE_LIMIT
    except (TypeError, ValueError):
        tlim = DEMO_TORQUE_LIMIT
    tlim = max(150, min(1000, tlim))
    _set_torque_limit(bus, live, tlim)
    # Start from zero offsets (arms out) — do NOT stand up.
    if check():
        _hold_here(bus, live)
        _set_torque_limit(bus, live, 1000)
        return "aborted"

    log_cm = None
    if log_path is not None:
        log_cm = MotionLog(log_path, live)
        log_cm.__enter__()
    try:
        ok = _run_frames(bus, live, frame_fn(seconds=air_s), check,
                         label=title, log=log_cm)
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)

    if not ok:
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    # Natural finish: ease back to zero, then LIMP.  Leaving torque on at
    # 0° with legs in the air makes some yaws buzz/hunt (backlash + PID).
    print("  Demo finished — easing back to zero ...")
    if not go_to_zero_pose(bus, abort_check=check, seconds=max(1.0, 2.0 * sc)):
        _set_torque_limit(bus, live, 1000)
        return "aborted"
    _limp_all(bus, live)
    _set_torque_limit(bus, live, 1000)
    return "done"


def run_demo_menu(bus: FeetechBus, *, log: bool = True) -> None:
    """Interactive picker (also used from motor_setup action ``f``)."""
    print()
    print("=" * 60)
    print("  DEMOS")
    print("=" * 60)
    print("  Sorted gentle → spicy.  [1–2] air @ 0°, [3+] planted standing.")
    print("  plant_* demos = rise_show acts alone (look/bounce/ripple/…).")
    print("  rise_show = full multi-act show.  ANY KEY aborts.")
    print()
    for i, (name, (title, _)) in enumerate(DEMOS.items(), 1):
        print(f"    {i:>2})  {name:<14}  {title}")
    print("    a)  all demos in sequence")
    print("    h)  hold + log (no motion — diagnose hunt)")
    print("    0)  go to zero pose now (slow)")
    print("    l)  limp all motors (torque OFF)")
    print("    q)  back")
    print()

    live = _live_robot_ids(bus)
    print(f"  live robot IDs: {sorted(live) or '(none)'}")
    if not live:
        return

    ans = input("  demo [name/#/a/h/0/l/q]: ").strip().lower()
    if not ans or ans in ("q", "quit", "back"):
        return

    if ans in ("l", "limp", "relax", "free"):
        _limp_all(bus, live)
        return

    if ans in ("h", "hold", "log-hold", "diag"):
        print("  Holding current pose and logging motor feedback ...")
        with keystroke_abort_watch() as abort_check:
            run_hold_log(bus, live, seconds=6.0, recommand=False,
                         abort_check=abort_check)
        _limp_all(bus, live)
        return

    if ans in ("0", "zero", "home", "rest"):
        if input("  Clear to ease to zero pose? [y/N]: ").strip().lower() not in (
                "y", "yes"):
            print("  cancelled.")
            return
        with keystroke_abort_watch() as abort_check:
            go_to_zero_pose(bus, abort_check=abort_check)
        return

    # Aliases for rise presets.
    if ans in ("rise+", "riseplus", "rise_plus", "rh", "rf", "high"):
        ans = "rise+"
    elif ans in ("rise_turn", "riseturn", "rt", "turn", "pivot"):
        ans = "rise_turn"
    elif ans in ("rise_show", "riseshow", "rs", "show"):
        ans = "rise_show"
    elif ans.startswith("plant_") and ans in DEMOS:
        pass
    elif ans in ("look", "nod"):
        ans = "plant_look"
    elif ans in ("bounce", "boom"):
        ans = "plant_bounce"
    elif ans in ("gallop",):
        ans = "plant_gallop"
    elif ans in ("tripod",):
        ans = "plant_tripod"
    elif ans in ("fan",):
        ans = "plant_fan"
    elif ans in ("star",):
        ans = "plant_star"
    elif ans in ("stomp", "tada"):
        ans = "plant_stomp"

    names: list[str]
    if ans in ("a", "all"):
        names = list(DEMOS)
    elif ans.isdigit() and 1 <= int(ans) <= len(DEMOS):
        names = [list(DEMOS)[int(ans) - 1]]
    elif ans in DEMOS:
        names = [ans]
    else:
        print("  unknown — use a name, number, a, h, 0, l, or q")
        return

    planted = [n for n in names
               if n in PLANTED_ACTS or n.startswith("rise")
               or n == "stand_hands"]
    if planted == ["rise+"]:
        prompt = (f"  Path clear — HIGHER+FASTER reach from the stand "
                  f"(hip {RISE_HIGH_HIP_DEG:.0f}°, knee {RISE_HIGH_KNEE_DEG:.0f}°, "
                  f"~{RISE_FAST_SECONDS:.0f} s)? [y/N]: ")
    elif planted == ["rise"]:
        prompt = (f"  Path clear — SLOW deep reach from the stand "
                  f"(hip {RISE_HIP_DEG:.0f}°, knee {RISE_KNEE_DEG:.0f}°)? "
                  f"[y/N]: ")
    elif planted == ["rise_turn"]:
        prompt = (f"  Path clear — FAST rise + turn {RISE_TURN_YAW_DEG:.0f}° "
                  f"(cord-safe, hip {RISE_HIGH_HIP_DEG:.0f}° / "
                  f"knee {RISE_HIGH_KNEE_DEG:.0f}°)? [y/N]: ")
    elif planted == ["rise_show"]:
        prompt = ("  Path clear — FULL multi-leg SHOW "
                  "(fast; tether-safe)? [y/N]: ")
    elif planted and all(n in PLANTED_ACTS for n in planted):
        prompt = (f"  Path clear — planted standing act(s) "
                  f"{', '.join(planted)}? [y/N]: ")
    elif planted:
        prompt = ("  Path clear — includes planted/rise demos? [y/N]: ")
    else:
        prompt = "  Path clear (gentle air demos from zero)? [y/N]: "
    if input(prompt).strip().lower() not in ("y", "yes"):
        print("  cancelled.")
        return

    aborted = False
    with keystroke_abort_watch() as abort_check:
        try:
            for name in names:
                print()
                log_tag = "rise_plus" if name == "rise+" else name
                log_path = default_log_path(log_tag) if log else None
                status = run_demo(bus, name, abort_check=abort_check,
                                  log_path=log_path)
                if status == "aborted" or abort_check():
                    aborted = True
                    break
        except KeyboardInterrupt:
            print("\n  Ctrl-C — stopping.")
            try:
                _hold_here(bus, live)
            except Exception:
                pass
            aborted = True

    if aborted:
        _set_torque_limit(bus, live, 1000)
        _ask_return_to_zero(bus, live)
    # Successful demos already limp at the end of run_demo.


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--demo", choices=list(DEMOS), default=None,
                    help="run one demo then exit (use rise+ for higher/faster)")
    ap.add_argument("--high", action="store_true",
                    help="with --demo rise: taller reach (hip +20° / ~119 mm)")
    ap.add_argument("--fast", action="store_true",
                    help="with --demo rise: quicker glide (~5 s up, ~6 s down)")
    ap.add_argument("--zero", action="store_true",
                    help="slowly drive all joints to 0° then exit")
    ap.add_argument("--seconds", type=float, default=None,
                    help="absolute duration override (before --speed scale)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="demo tempo multiplier (0.25–3.0; default 1.0)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--relax", action="store_true",
                    help="limp all after demo")
    ap.add_argument("--log", nargs="?", const="auto", default="auto",
                    help="CSV telemetry path (default: auto under "
                         "motor_setup/logs/). Pass empty via --no-log to disable.")
    ap.add_argument("--no-log", action="store_true",
                    help="disable motion CSV telemetry")
    ap.add_argument("--log-hold", type=float, nargs="?", const=6.0,
                    default=None, metavar="SEC",
                    help="hold present pose and log for SEC seconds "
                         "(no demo motion) — best for hunt diagnosis")
    ap.add_argument("--log-hold-recommand", action="store_true",
                    help="with --log-hold, re-send hold every tick "
                         "(compare against default command-once)")
    args = ap.parse_args(argv)

    if args.list:
        for name, (title, _) in DEMOS.items():
            print(f"  {name:<12}  {title}")
        return

    port = args.port or default_port()
    if not port:
        raise SystemExit("No USB serial port found")
    print(f"Opening {port} @ {args.baud} ...")
    bus = FeetechBus(port, args.baud)
    try:
        live = _live_robot_ids(bus)

        if args.log_hold is not None:
            with keystroke_abort_watch() as abort_check:
                run_hold_log(
                    bus, live, seconds=float(args.log_hold),
                    recommand=args.log_hold_recommand,
                    path=(None if args.log in (None, "auto")
                          else Path(args.log)),
                    abort_check=abort_check,
                )
            _limp_all(bus, live)
            return

        if args.zero:
            if input("  Clear to ease to zero pose? [y/N]: ").strip().lower() not in (
                    "y", "yes"):
                return
            with keystroke_abort_watch() as abort_check:
                go_to_zero_pose(bus, abort_check=abort_check,
                                seconds=args.seconds or 3.0)
            if args.relax:
                for sid in live:
                    limp_now(bus, sid)
                print("  limp.")
            return

        want_log = not args.no_log

        def _log_path_for(tag: str) -> Path | None:
            if not want_log:
                return None
            if args.log in (None, "auto"):
                return default_log_path(tag)
            return Path(args.log)

        if args.demo:
            if args.demo == "rise_turn":
                prompt = (f"  Path clear — FAST rise + turn "
                          f"{RISE_TURN_YAW_DEG:.0f}° (cord-safe)? [y/N]: ")
            elif args.demo == "rise_show":
                prompt = ("  Path clear — NEXT-LEVEL multi-leg SHOW "
                          "(fast; tether-safe)? [y/N]: ")
            elif args.demo in ("rise", "rise+") or args.high or args.fast:
                use_high = args.demo == "rise+" or args.high
                use_fast = args.demo == "rise+" or args.fast
                hip = RISE_HIGH_HIP_DEG if use_high else RISE_HIP_DEG
                knee = RISE_HIGH_KNEE_DEG if use_high else RISE_KNEE_DEG
                secs = RISE_FAST_SECONDS if use_fast else RISE_SECONDS
                kind = ("HIGHER+FASTER" if use_high and use_fast
                        else "HIGHER" if use_high
                        else "FASTER" if use_fast
                        else "SLOW")
                prompt = (f"  Path clear — {kind} reach from the stand "
                          f"(hip {hip:.0f}°, knee {knee:.0f}°, "
                          f"~{secs:.0f} s)? [y/N]: ")
            else:
                prompt = "  Path clear (arms will wave from zero)? [y/N]: "
            if input(prompt).strip().lower() not in ("y", "yes"):
                return
            aborted = False
            log_tag = ("rise_plus" if args.demo == "rise+" or (args.high and args.fast)
                       else args.demo)
            with keystroke_abort_watch() as abort_check:
                status = run_demo(bus, args.demo, seconds=args.seconds,
                                  speed=args.speed,
                                  abort_check=abort_check,
                                  log_path=_log_path_for(log_tag),
                                  rise_high=args.high, rise_fast=args.fast)
                aborted = status == "aborted"
            if aborted:
                _ask_return_to_zero(bus, _live_robot_ids(bus))
            if args.relax:
                for sid in _live_robot_ids(bus):
                    limp_now(bus, sid)
                print("  limp.")
        else:
            run_demo_menu(bus, log=want_log)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
