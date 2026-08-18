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
import math
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
DT = 0.08  # slower host tick → fewer trapezoid restarts
DEADBAND_DEG = 0.8  # skip tiny re-commands (was 0.4 — buzzed)
MAX_STREAM_SPEED = 450
MIN_STREAM_SPEED = 40
# Rise: chassis sits on an elevated stand, so walking stance (−25°/+60°,
# only ~37 mm of foot drop) never reaches the floor.  Match stand home:
# femur angled down (~+20°) and tibia steep (+80° knee ≈ right-angle plant).
#
# Presets (foot drop ≈ −FEMUR·sin(hip) − TIBIA·sin(hip+knee), mm):
#   default / stand  hip +20° / knee +80° / 12 s  → ~159 mm
#   high+fast        same angles / 5 s            → ~159 mm (faster)
RISE_SECONDS = 12.0
DESCEND_SECONDS = 12.0
RISE_HIP_DEG = 20.0
RISE_KNEE_DEG = 80.0
RISE_HIGH_HIP_DEG = 20.0
RISE_HIGH_KNEE_DEG = 80.0
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
RISE_SHOW_DT = 0.055             # planted multi-leg stream tick
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

RISE_PRESETS = {
    "default": {
        "hip": RISE_HIP_DEG,
        "knee": RISE_KNEE_DEG,
        "rise_seconds": RISE_SECONDS,
        "descend_seconds": DESCEND_SECONDS,
        "drop_mm": 159,
        "label": "stand plant",
    },
    "high_fast": {
        "hip": RISE_HIGH_HIP_DEG,
        "knee": RISE_HIGH_KNEE_DEG,
        "rise_seconds": RISE_FAST_SECONDS,
        "descend_seconds": RISE_FAST_DESCEND_SECONDS,
        "drop_mm": 159,
        "label": "higher + faster reach",
    },
}


def _zero_pose() -> list[float]:
    """Sit / air home — legs straight out (logical 0°)."""
    return [0.0] * N_JOINTS


def _stand_zero_pose() -> list[float]:
    """Stand home — learned plant or default +20°/+80° (femur down, tibia steep)."""
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

    Walking plant (hip −25° / knee +60°) only drops the foot ~37 mm below
    the hip.  Default / stand plant is hip +20° / knee +80° (~159 mm).
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


def _enable_torque(bus: FeetechBus, live: set[int]) -> None:
    for sid in sorted(live):
        bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)


def _set_torque_limit(bus: FeetechBus, live: set[int], limit: int) -> None:
    for sid in sorted(live):
        try:
            bus.pkt.write2ByteTxRx(sid, ADDR_TORQUE_LIMIT, int(limit))
        except Exception:
            pass


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

    def reset(self) -> None:
        self.last = None

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
            _write_pose(bus, degrees, live, speed=120, acc=10)
            return {j: (120, 10) for j in range(N_JOINTS)
                    if joint_to_servo_id(j) in live}

        wrote: dict[int, tuple[int, int]] = {}
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
            count = deg_to_count(joint, deg, bus.trims[joint])
            bus.pkt.SyncWritePosEx(sid, count, speed, acc)
            self.last[joint] = deg
            wrote[joint] = (speed, acc)
        if wrote:
            bus.pkt.groupSyncWrite.txPacket()
            bus.pkt.groupSyncWrite.clearParam()
        return wrote


def _read_pose(bus: FeetechBus, live: set[int]) -> list[float]:
    """Present joint angles (deg); missing IDs → 0."""
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
    print(f"  {label}  — any key aborts immediately")
    if log is not None:
        print("  (querying motors each tick → CSV; loop will be a bit slower)")
    stream = PoseStreamer()
    for pose in frames:
        if abort_check():
            _hold_here(bus, live)
            print("    STOP — keystroke.  Holding pose.")
            return False
        t0 = time.monotonic()
        wrote = stream.write(
            bus, pose, live, dt=tick, deadband=deadband,
            min_speed=min_speed, max_speed=max_speed, max_acc=max_acc)
        if log is not None:
            log.sample(bus, pose, wrote=wrote)
        # Sleep the remainder of DT so motion timing stays roughly honest
        # even when telemetry reads eat into the tick.
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
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
STREAM_GUARD_A = 3.0          # stall-fight: joint over this while not moving
STREAM_HARD_CAP_A = 4.0       # instantaneous hard cap, trips regardless
STAND_DANCE_TORQUE = 900      # weight-bearing motion (end-hold restores 1000)
LIVE_SPEED_MIN = 0.25
LIVE_SPEED_MAX = 3.0


def _clamp_live_speed(x) -> float:
    try:
        v = float(x)
    except (TypeError, ValueError):
        v = 1.0
    return max(LIVE_SPEED_MIN, min(LIVE_SPEED_MAX, v))


def stream_pose_fn(bus: FeetechBus, live: set[int], pose_fn, *,
                   seconds: float, abort_check=None,
                   speed_fn=None, status_cb=None, label: str = "stream",
                   tracker: CurrentPeakTracker | None = None,
                   guard_a: float = STREAM_GUARD_A,
                   log: MotionLog | None = None,
                   max_speed: int = 3000, max_acc: int = 200) -> str:
    """Stream ``pose_fn(t)`` at ~20 Hz with a live tempo multiplier.

    Demo time ``t`` advances by wall-dt x ``speed_fn()`` every tick, so
    the web speed slider changes tempo MID-MOTION.  ``seconds`` is demo
    time (equals wall time at 1x).  Returns ``done`` / ``aborted`` /
    ``guard`` — on ``guard`` the robot is already holding in place;
    the caller decides how to surface it.
    """
    check = abort_check or (lambda: False)
    spd = speed_fn or (lambda: 1.0)
    if tracker is None:
        tracker = CurrentPeakTracker()
    seconds = float(seconds)
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
    stall_prev: set = set()
    sweep_n = 0
    t = 0.0
    t0 = time.monotonic()
    wall_prev = t0
    t_prev = 0.0
    last_sample = -1.0
    while t < seconds:
        if check():
            # Return immediately — no bus ops here (concurrent hold from
            # the web Stop handler was hanging the MCU link).
            return "aborted"
        wall = time.monotonic()
        rate = _clamp_live_speed(spd())
        t += (wall - wall_prev) * rate
        wall_prev = wall
        q = pose_fn(min(t + STREAM_LOOKAHEAD_S * rate, seconds))
        # dt*0.75 cancels _speed_for_delta's 0.9 undershoot and commands
        # slightly above the carrot rate so accumulated lag drains —
        # same sizing as the stand-up pursuit.
        streamer.write(
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
                log.sample(bus, q, wrote={})
            if tracker.peak_a > STREAM_HARD_CAP_A:
                _hold_here(bus, live)
                return "guard"
            # Stall-fight semantics (same as the stand-up lab): a joint
            # over the limit while NOT moving, two sweeps in a row.
            now = {fb["joint"] for fb in tracker.last_fb
                   if abs(fb["current_a"]) > guard_a
                   and abs(fb["speed_deg_s"]) < 8.0}
            if now & stall_prev:
                _hold_here(bus, live)
                return "guard"
            stall_prev = now
            if status_cb is not None:
                try:
                    status_cb(f"{label}: {t:.0f}/{seconds:.0f}s "
                              f"x{rate:.2f} peak {tracker.peak_a:.2f}A")
                except Exception:
                    pass
        time.sleep(STREAM_TICK_S)
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
# Standing dances — streamed, around the LIVE captured plant (v2, 08-17).
#
# v1 modulated raw joint offsets and looked chaotic on hardware: without
# foot compensation every hip/knee offset scrubs the planted feet and
# lurches the body ("the robot collapses" — operator). The previous
# robot's dances looked good because of two tricks its firmware used
# (prototype_v1 stepDance):
#
#   1. PLANTED-FOOT BODY IK — the feet stay glued to the stance
#      footprint; the BODY is what sways / bobs / twists (hula, pogo,
#      twist & dip). Ported here as PlantedBodyIK via tripod_gait's
#      2-link IK.
#   2. Raised-cosine lift pulses (legPulse) with half-width < pi/2, so
#      alternating tripods NEVER overlap in the air — always >= 3 feet
#      down, statically stable.
#
# Every dance ramps its amplitude in over ~1.5 s (nothing snaps) and is
# cyclic / zero-mean (cord-safe). All run through stream_pose_fn, so
# the web speed slider works live.
# ---------------------------------------------------------------------------
DANCE_RAMP_S = 1.5

# Body-motion amplitudes (kept inside the IK reach at the plant stance).
HULA_R_MM = 16.0
HULA_HZ = 0.5
SWAY_Y_MM = 18.0
SWAY_HZ = 0.3
POGO_DROP_MM = 16.0
POGO_HZ = 0.9
TWIST_DEG = 8.0
TWIST_DIP_MM = 8.0
TWIST_HZ = 0.45
# Stadium wave: narrow crest (cosine cubed ~ 1-2 legs) circles the hex.
# v2 08-17: the joint-blend lift (49 deg hip+knee swings) measured 3.15 A
# support-hip spikes and 1.18 g lurches on hardware — lift the foot
# STRAIGHT UP through the planted IK instead (march's recipe: 0.43 A).
WAVE_PERIOD_S = 3.2
WAVE_LIFT_MM = 30.0
# Tripod march: legPulse half-width < pi/2 => always one tripod planted.
MARCH_PERIOD_S = 1.6
MARCH_LIFT_MM = 22.0
MARCH_SWAY_MM = 8.0
MARCH_PULSE_HW = 0.42 * math.pi
# Say hi: weight eases back + dips, then ONE front paw lifts and waves.
# v3 08-17: two-legs-up fell backward on hardware (az 0.42 g measured;
# CoM lands ~1 mm from the L1-L4 support edge). One paw = 5 feet down.
HI_SHIFT_BACK_MM = 22.0
HI_DIP_MM = 10.0
HI_WAVE_LEG = 0                # azimuth 30 deg (+x = forward): front-left
HI_LIFT_HIP_DEG = -25.0
HI_LIFT_KNEE_DEG = 45.0
HI_WAVE_YAW_DEG = 12.0
HI_WAVE_HZ = 0.8


class PlantedBodyIK:
    """Joint angles that keep all six feet PLANTED while the body moves.

    ``pose(ox, oy, dz, twist)``: body displaced (ox, oy) metres in the
    chassis frame, raised ``dz`` metres, yawed ``twist`` radians — feet
    stay at the footprint the live plant pose puts them. Per-leg lift
    overrides raise a foot straight up (still through the real IK).
    Ported from prototype_v1's plantedLegAnglesZ.
    """

    def __init__(self, base: list[float] | None = None):
        import tripod_gait as TG
        self._TG = TG
        self.base = list(base) if base is not None else _stand_zero_pose()
        self.feet: list[tuple[float, float, float]] = []
        self.origins: list[tuple[float, float]] = []
        self.azim: list[float] = []
        for leg in range(6):
            a = (leg + 0.5) * math.pi / 3.0
            yaw = math.radians(self.base[3 * leg + 0])
            hip = math.radians(self.base[3 * leg + 1])
            knee = math.radians(self.base[3 * leg + 2])
            reach = (TG.COXA + TG.FEMUR * math.cos(hip)
                     + TG.TIBIA * math.cos(hip + knee))
            fz = -TG.FEMUR * math.sin(hip) - TG.TIBIA * math.sin(hip + knee)
            ox0 = TG.LEG_RADIAL * math.cos(a)
            oy0 = TG.LEG_RADIAL * math.sin(a)
            self.feet.append((ox0 + reach * math.cos(a + yaw),
                              oy0 + reach * math.sin(a + yaw), fz))
            self.origins.append((ox0, oy0))
            self.azim.append(a)

    def leg_angles(self, leg: int, ox: float, oy: float, dz: float,
                   twist: float, *, foot_raise: float = 0.0
                   ) -> tuple[float, float, float]:
        """(yaw, hip, knee) degrees for one leg; falls back to the base
        pose when the target leaves the IK envelope."""
        fx, fy, fz = self.feet[leg]
        # Foot in the displaced/twisted body frame.
        px, py = fx - ox, fy - oy
        ct, st = math.cos(-twist), math.sin(-twist)
        bx = ct * px - st * py
        by = st * px + ct * py
        o0x, o0y = self.origins[leg]
        rx, ry = bx - o0x, by - o0y
        a = self.azim[leg]
        ca, sa = math.cos(a), math.sin(a)
        x_yaw = ca * rx + sa * ry
        y_yaw = -sa * rx + ca * ry
        z = fz - dz + foot_raise
        ik = self._TG._leg_ik((math.hypot(x_yaw, y_yaw), 0.0, z))
        if ik is None:
            return (self.base[3 * leg + 0], self.base[3 * leg + 1],
                    self.base[3 * leg + 2])
        hip, knee = ik
        return (math.degrees(math.atan2(y_yaw, x_yaw)),
                math.degrees(hip), math.degrees(knee))

    def pose(self, ox: float = 0.0, oy: float = 0.0, dz: float = 0.0,
             twist: float = 0.0,
             foot_raise: dict[int, float] | None = None) -> list[float]:
        out: list[float] = []
        for leg in range(6):
            fr = (foot_raise or {}).get(leg, 0.0)
            out.extend(self.leg_angles(leg, ox, oy, dz, twist,
                                       foot_raise=fr))
        return out


def _leg_pulse(phase: float, center: float, hw: float) -> float:
    """Raised-cosine lift pulse (prototype_v1 legPulse): 1 at ``center``,
    0 beyond ``hw`` radians either side. Two pulses pi apart with
    hw < pi/2 never overlap — always a double-support window."""
    d = (phase - center) % (2.0 * math.pi)
    if d > math.pi:
        d = 2.0 * math.pi - d
    if d >= hw:
        return 0.0
    return 0.5 * (1.0 + math.cos(d / hw * math.pi))


def make_stand_pose_fn(name: str):
    """Pose function for one standing dance, anchored to the live plant."""
    base = _stand_zero_pose()
    ik = PlantedBodyIK(base)
    two_pi = 2.0 * math.pi
    mm = 0.001

    def ramp(t: float) -> float:
        return min(t / DANCE_RAMP_S, 1.0)

    def _blend_leg(pose: list[float], leg: int, amt: float,
                   yaw: float, hip: float, knee: float) -> None:
        """Blend one leg from its current pose toward a raised target."""
        b = 3 * leg
        pose[b + 0] += amt * (yaw - pose[b + 0])
        pose[b + 1] += amt * (hip - pose[b + 1])
        pose[b + 2] += amt * (knee - pose[b + 2])

    if name == "stand_sway":
        # Old robot M18: slow gentle side-to-side body lean, feet planted.
        def fn(t: float) -> list[float]:
            oy = SWAY_Y_MM * mm * ramp(t) * math.sin(two_pi * SWAY_HZ * t)
            return ik.pose(oy=oy)
        return fn
    if name == "stand_hula":
        # Old robot B: feet planted, body circles — "the wiggle".
        def fn(t: float) -> list[float]:
            w = two_pi * HULA_HZ * t
            r = HULA_R_MM * mm * ramp(t)
            return ik.pose(ox=r * math.cos(w), oy=r * math.sin(w))
        return fn
    if name == "stand_bounce":
        # Old robot M9 pogo: feet planted, body bobs straight down/up.
        def fn(t: float) -> list[float]:
            dz = (-POGO_DROP_MM * mm * ramp(t)
                  * 0.5 * (1.0 - math.cos(two_pi * POGO_HZ * t)))
            return ik.pose(dz=dz)
        return fn
    if name == "stand_twist":
        # Old robot T: body twists on its yaws while dipping — feet
        # planted (the IK counter-rotates them), zero-mean, cord-safe.
        def fn(t: float) -> list[float]:
            w = two_pi * TWIST_HZ * t
            tw = math.radians(TWIST_DEG) * ramp(t) * math.sin(w)
            dz = (-TWIST_DIP_MM * mm * ramp(t)
                  * 0.5 * (1.0 - math.cos(2.0 * w)))
            return ik.pose(dz=dz, twist=tw)
        return fn
    if name == "stand_wave":
        # Old robot V: stadium wave — a narrow raised-foot crest circles
        # the body; cosine CUBED keeps it to ~1-2 legs, everyone else
        # stays planted at the exact footprint. Feet rise STRAIGHT UP
        # through the IK (no joint-space flailing — see WAVE_* note).
        def fn(t: float) -> list[float]:
            ph = two_pi * (t / WAVE_PERIOD_S)
            raise_m: dict[int, float] = {}
            for leg in range(6):
                c = 0.5 * (1.0 + math.cos(ph - ik.azim[leg]))
                bump = (c ** 3) * ramp(t)
                if bump > 1e-3:
                    raise_m[leg] = WAVE_LIFT_MM * mm * bump
            return ik.pose(foot_raise=raise_m)
        return fn
    if name == "stand_march":
        # Old robot M5/M6: alternating tripod march with a weight-shift
        # sway. legPulse half-width < pi/2 => the tripods never lift at
        # the same time; lifted feet rise STRAIGHT UP through the IK.
        def fn(t: float) -> list[float]:
            ph = two_pi * (t / MARCH_PERIOD_S)
            r = ramp(t)
            raise_m: dict[int, float] = {}
            for leg in range(6):
                center = 0.0 if leg % 2 == 0 else math.pi
                p = _leg_pulse(ph, center, MARCH_PULSE_HW)
                if p > 1e-3:
                    raise_m[leg] = MARCH_LIFT_MM * mm * p * r
            oy = MARCH_SWAY_MM * mm * r * math.sin(ph)
            return ik.pose(oy=oy, foot_raise=raise_m)
        return fn
    if name == "stand_hi":
        # Old robot O, v3: wave ONE paw. Both-front-legs-up measured a
        # backward fall on 08-17 — the front support edge runs exactly
        # through the two side feet and the raised legs pull the CoM
        # onto it (~1 mm margin). One leg up keeps 5 feet down and the
        # polygon reaches well forward. Weight still eases back + dips.
        def fn(t: float) -> list[float]:
            shift = min(t / 0.8, 1.0)
            arm = max(0.0, min((t - 0.7) / 1.0, 1.0))
            pose = ik.pose(ox=-HI_SHIFT_BACK_MM * mm * shift,
                           dz=-HI_DIP_MM * mm * shift)
            wph = two_pi * HI_WAVE_HZ * t
            _blend_leg(pose, HI_WAVE_LEG, arm,
                       HI_WAVE_YAW_DEG * math.sin(wph),
                       HI_LIFT_HIP_DEG,
                       HI_LIFT_KNEE_DEG + 12.0 * math.sin(wph))
            return pose
        return fn
    raise SystemExit(f"unknown standing dance {name!r}")


# Streamed demos (live speed): standing dances + the air wiggles.
STAND_STREAM_DEMOS = ("stand_sway", "stand_hula", "stand_bounce",
                      "stand_twist", "stand_wave", "stand_march",
                      "stand_hi")
# Quad demos: weight-bearing like the stand dances (τ900 + stall guard,
# home = stand) but grouped on their own web tab. The pose function is
# DURATION-AWARE: the exit choreography (pitch level, untuck fronts)
# is scripted into the last ~5.4 s, so the run ends back at the plant.
QUAD_STREAM_DEMOS = ("quad_walk", "quad_trot")


def _make_quad_walk_fn(seconds: float, gait: str = "walk"):
    from quad_walk import make_quad_walk_pose_fn
    return make_quad_walk_pose_fn(_stand_zero_pose(), seconds, gait=gait)


def _make_quad_trot_fn(seconds: float):
    return _make_quad_walk_fn(seconds, gait="trot")


_make_quad_walk_fn.duration_aware = True
_make_quad_trot_fn.duration_aware = True

STREAM_POSE_FACTORIES = {
    "shimmy": lambda: pose_shimmy,
    "ripple": lambda: pose_ripple,
    "heartbeat": lambda: pose_heartbeat,
    "conductor": lambda: pose_conductor,
    "twinkle": make_pose_twinkle,
    **{n: (lambda n=n: make_stand_pose_fn(n)) for n in STAND_STREAM_DEMOS},
    "quad_walk": _make_quad_walk_fn,
    "quad_trot": _make_quad_trot_fn,
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
    if name == "quad_trot":
        # sim-measured: above 1x the deep-lean trot loses its margin
        # (1.25x fell, 2x walks BACKWARD) — hard-cap the live slider.
        from quad_walk import GAITS
        cap = GAITS["trot"].get("speed_cap", 2.0)
        base_fn = speed_fn
        speed_fn = lambda: min(cap, base_fn())  # noqa: E731
    quad = name in QUAD_STREAM_DEMOS
    standing = name in STAND_STREAM_DEMOS or quad
    if seconds is None:
        dur = (QUAD_STREAM_SECONDS if quad
               else STAND_STREAM_SECONDS if standing
               else AIR_DEMO_SECONDS.get(name, 7.0))
    else:
        dur = float(seconds)
    dur = max(2.0, min(STREAM_SECONDS_MAX, dur))
    if quad:
        # entry + exit choreography needs room (plus >= one gait cycle).
        from quad_walk import MIN_SECONDS
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

    factory = STREAM_POSE_FACTORIES[name]
    pose_fn = (factory(dur) if getattr(factory, "duration_aware", False)
               else factory())
    tracker = CurrentPeakTracker()
    title = DEMOS[name][0] if name in DEMOS else name
    print(f"  {name} — {title}")
    print(f"    streamed @ ~{1.0 / STREAM_TICK_S:.0f} Hz for ~{dur:.0f}s "
          f"(live speed x{_clamp_live_speed(speed_fn()):.2f}) τ{tlim}")
    print("  Any key aborts.")

    log_cm = MotionLog(log_path, live) if log_path is not None else None
    if log_cm is not None:
        log_cm.__enter__()
    try:
        st = stream_pose_fn(
            bus, live, pose_fn, seconds=dur, abort_check=check,
            speed_fn=speed_fn, status_cb=status_cb, label=name,
            tracker=tracker, log=log_cm)
        if st == "aborted":
            return "aborted"
        if st == "guard":
            msg = (f"stopped: {tracker.peak_a:.2f} A peak on joint "
                   f"{tracker.peak_joint} — stall-fight, holding here")
            print(f"  {msg}")
            return msg
        # Natural finish: settle back onto the base pose.
        _set_torque_limit(bus, live, 1000)
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
    "ripple": ("[2 easy] yaw wave around the hex (air)", frames_ripple),
    "conductor": ("[2 easy] one leg waves; others hold", frames_conductor),
    "arms_up": ("[2 easy] sit: all six arms way over head", None),
    # --- standing dances (streamed · live speed · planted-foot body IK) ---
    "stand_sway": ("[3 stand] body leans side to side, feet planted", None),
    "stand_hula": ("[3 stand] body circles — the wiggle, feet planted", None),
    "stand_bounce": ("[3 stand] pogo bob — body dips straight down", None),
    "stand_twist": ("[3 stand] twist & dip, feet planted (cord-safe)", None),
    "stand_wave": ("[4 stand] stadium wave — lift crest circles the hex", None),
    "stand_march": ("[4 stand] tripod march in place + weight sway", None),
    "stand_hi": ("[4 stand] weight back, one front paw waves HI", None),
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
    # --- real walk (open-loop tripod gait) --------------------------------
    "walk": ("[7 walk] tripod forward a few strides, then stand", None),
    "walk_spin": ("[7 walk] in-place turn (tripod), then stand", None),
    "walk_oval": ("[7 walk] forward → spin → reverse → stand", None),
    # --- quad mode (own web tab: tip back, walk on four legs) -------------
    "quad_walk": ("[8 quad] TIP BACK — rear up on 4 legs, front paws in "
                  "the air, animal walk forward, sit back down", None),
    "quad_trot": ("[8 quad] TIP BACK + TROT — diagonal leg pairs like a "
                  "horse, ~2x the walk's pace, sit back down", None),
}

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
    "ripple": 8.0,
    "conductor": 8.0,
    "arms_up": 6.0,
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

    Default: hip +20° / knee +80° over ~12 s (~159 mm drop).
    ``rise+`` / preset ``high_fast``: hip +20° / knee +80° over ~5 s
    (~159 mm).  One SyncWrite (no host streaming).  Logs peak current;
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
    _pt = math.radians(hip + knee)
    drop_mm = int(round(90.0 * math.sin(_p) + 128.0 * math.sin(_pt)))
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
          f"(~{drop_mm} mm foot drop; walk stance −25°/+60° is only ~37 mm)")
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
    DEADBAND_DEG = 0.45
    t0 = time.monotonic()
    try:
        for i in range(n):
            if check():
                _hold_here(bus, live)
                return False
            t = i * dt
            if mode == "jazz":
                pose = _zero_pose()
                for leg in range(6):
                    pose[leg * 3] = (
                        RISE_SHOW_WAVE_YAW_DEG
                        * math.sin(4.2 * t + leg * math.pi / 3))
            else:
                pose = _elevated_stand_pose(hip=hip, knee=knee, yaw=0.0)
                for leg in range(6):
                    if mode == "ripple":
                        # 2–3 legs up at once; traveling wave around the hex.
                        phase = 2.8 * t - leg * (2 * math.pi / 6)
                        lift = max(0.0, math.sin(phase)) ** 0.65
                        yaw_amp = RISE_SHOW_WAVE_YAW_DEG * math.sin(
                            phase + 0.4)
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
    t0 = time.monotonic()
    gait.reset_phase(t=t0)
    try:
        for label, vx, vy, om, dur in segments:
            if check():
                break
            print(f"    · {label}  vx={vx*1000:.0f} mm/s  ω={om:.2f}")
            gait.set_velocity(vx=vx, vy=vy, omega=om)
            seg_t0 = time.monotonic()
            while time.monotonic() - seg_t0 < dur:
                if check():
                    break
                now = time.monotonic()
                pose = gait.desired_deg(now)
                _write_pose(bus, pose, live, speed=WALK_SPEED, acc=WALK_ACC)
                if log_cm is not None:
                    try:
                        log_cm.sample(bus, pose, wrote={})
                    except Exception:
                        pass
                time.sleep(WALK_DEMO_DT)
            if check():
                break
        gait.stop()
        # Settle onto plant (kill residual swing).
        for _ in range(8):
            if check():
                break
            now = time.monotonic()
            pose = gait.desired_deg(now)
            _write_pose(bus, pose, live, speed=WALK_SPEED, acc=WALK_ACC)
            time.sleep(WALK_DEMO_DT)
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
             rise_fast: bool = False) -> str:
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
    sc = _speed_scale(speed)
    spd = _clamp_demo_speed(speed)
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
    if name == "arms_up":
        return run_arms_up_demo(
            bus, abort_check=abort_check, speed=spd, seconds=seconds,
            torque=torque, log_path=log_path)

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
                    help="with --demo rise: taller reach (hip +20° / ~159 mm)")
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
