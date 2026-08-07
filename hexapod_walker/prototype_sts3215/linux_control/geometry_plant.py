"""Geometry + contact plant finder for Phase-1 balance.

Target stance (operator intent):
  * hip ≈ 0° (femur straight / out)
  * knee ≈ 90° (axis max is +80° — use that)
  * chassis ~1–2 in above the ground after contact

Method:
  1. Soft-torque, yaw=0, hip=0; start with shallow knees (feet high).
  2. Slowly deepen knees toward +80° while watching knee current/load.
  3. On multi-knee contact, hold and sample.
  4. Compute a taller pose: same hip=0, reduce knee so FK foot-z rises by
     ``clearance_mm`` (~40 mm ≈ 1.6 in).
  5. Ease to that pose and save full ``joints_deg[18]`` plant.

Callable from hexapod-web (``/api/rl/find_plant``) — no SSH required.
"""
from __future__ import annotations

import math
import statistics
import time
from typing import Callable

from feetech_bus import (
    N_JOINTS,
    save_plant_pose,
    standing_pose_degrees,
)

# Link lengths (mm) — match tripod_gait / hexapod_prototype.
FEMUR_MM = 90.0
TIBIA_MM = 128.0

# Hip 0 + knee 80 = tall stilts (~126 mm foot drop). For ~1–2 in use crouch.
TARGET_HIP_DEG = -20.0
TARGET_KNEE_DEG = 55.0
START_HIP_DEG = 0.0
START_KNEE_DEG = 25.0           # start mid; deepen hip/knee toward crouch
CLEARANCE_MM = 25.0             # ~1 in above contact plane
REACH_SECONDS = 14.0
CLEARANCE_SECONDS = 6.0
REACH_TORQUE = 500
SAMPLE_DT = 0.08

CONTACT_CURRENT_FLOOR_A = 0.16
CONTACT_CURRENT_DELTA_A = 0.08
CONTACT_LOAD_FLOOR_PCT = 16.0
CONTACT_LOAD_DELTA_PCT = 8.0
CONTACT_ARM_KNEE_DEG = 35.0     # arm earlier than old plant (hip stays 0)
CONTACT_MIN_KNEE_JOINTS = 3
CONTACT_WINDOW_S = 0.45
CONTACT_BASELINE_POSE_DEG = 12.0
CONTACT_MAX_KNEE_SPEED = 90.0
MAX_TILT_DEG = 8.0              # abort before tip (operator asked: don't tip)


def _imu_tilt_deg(bus) -> tuple[float | None, float | None]:
    """Best-effort roll/pitch degrees from MPU accel (IMUR)."""
    try:
        if not hasattr(bus, "read_imu"):
            return None, None
        imu = bus.read_imu()
        if not imu:
            return None, None
        ax = float(imu.get("ax_g", 0.0))
        ay = float(imu.get("ay_g", 0.0))
        az = float(imu.get("az_g", 0.0))
        # Same convention as rl_move complementary filter (accel only).
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return roll, pitch
    except Exception:
        pass
    return None, None


def foot_z_mm(hip_deg: float, knee_deg: float) -> float:
    """Foot Z in yaw frame (mm); more negative = lower / farther down."""
    p = math.radians(hip_deg)
    pt = math.radians(hip_deg + knee_deg)
    return -FEMUR_MM * math.sin(p) - TIBIA_MM * math.sin(pt)


def knee_for_foot_z(hip_deg: float, z_mm: float) -> float | None:
    """Solve knee (deg) for desired foot z at fixed hip. None if unreachable."""
    p = math.radians(hip_deg)
    # z = -F sin(p) - T sin(p+k)  →  sin(p+k) = -(z + F sin p)/T
    num = -(z_mm + FEMUR_MM * math.sin(p)) / TIBIA_MM
    if abs(num) > 1.0:
        return None
    pt = math.asin(max(-1.0, min(1.0, num)))
    # Prefer the "knee bent down" solution in our sign convention (pt > p).
    knee = math.degrees(pt - p)
    if knee < -20.0 or knee > 80.0:
        # Try π - asin branch
        pt2 = math.pi - pt
        knee2 = math.degrees(pt2 - p)
        if -20.0 <= knee2 <= 80.0:
            return float(knee2)
        return None
    return float(knee)


def _median(vals: list[float], default: float) -> float:
    return float(statistics.median(vals)) if vals else default


def run_geometry_plant(
    bus,
    *,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
    clearance_mm: float = CLEARANCE_MM,
    reach_seconds: float = REACH_SECONDS,
) -> dict:
    """Contact reach at hip=0 → raise by clearance → save joints_deg plant."""
    abort_check = abort_check or (lambda: False)

    try:
        from inplace_demos import (
            _enable_torque, _hold_here, _limp_all, _live_robot_ids,
            _set_torque_limit, _write_pose, ease_to_pose,
        )
    except ImportError as e:
        return {"ok": False, "error": f"inplace_demos missing: {e}",
                "mode": "geometry_plant"}

    live = _live_robot_ids(bus)
    if len(live) < 12:
        return {"ok": False, "error": f"need more servos (live={len(live)})",
                "mode": "geometry_plant", "live": sorted(live)}

    def _progress(msg: str, **extra):
        if on_progress:
            try:
                on_progress({"msg": msg, "mode": "geometry_plant", **extra})
            except Exception:
                pass

    def _pose(hip: float, knee: float) -> list[float]:
        out: list[float] = []
        for _ in range(6):
            out.extend([0.0, float(hip), float(knee)])
        return out

    def _fb(j: int):
        try:
            return bus.read_feedback(j)
        except Exception:
            return None

    clearance_mm = float(clearance_mm)
    _progress(f"soft torque → crouch approach "
              f"(end hip {TARGET_HIP_DEG:.0f}° / knee {TARGET_KNEE_DEG:.0f}°)")
    _set_torque_limit(bus, live, REACH_TORQUE)
    _enable_torque(bus, live)
    if abort_check():
        return {"ok": False, "aborted": True, "mode": "geometry_plant"}

    # Ease to start pose (not the tall hip0/knee80 stilt).
    start = _pose(START_HIP_DEG, START_KNEE_DEG)
    if not ease_to_pose(bus, start, abort_check=abort_check, seconds=3.0,
                        label="geo_start"):
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_plant"}

    # --- blend toward crouch; contact usually while still above target ---
    _progress("reaching crouch / ground (watch knee current/load)…")
    n = max(2, int(reach_seconds / SAMPLE_DT))
    t0 = time.monotonic()
    base_I: list[float] = []
    base_L: list[float] = []
    hot_since: dict[int, float] = {}
    contact_found = False
    contact_knee = None
    contact_joints: list[int] = []

    for i in range(n):
        if abort_check():
            _set_torque_limit(bus, live, 1000)
            return {"ok": False, "aborted": True, "mode": "geometry_plant"}
        a = (i + 1) / n
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        hip_cmd = START_HIP_DEG + s * (TARGET_HIP_DEG - START_HIP_DEG)
        knee_cmd = START_KNEE_DEG + s * (TARGET_KNEE_DEG - START_KNEE_DEG)
        goal = _pose(hip_cmd, knee_cmd)
        _write_pose(bus, goal, live, speed=160, acc=12)
        time.sleep(SAMPLE_DT)

        roll_d, pitch_d = _imu_tilt_deg(bus)
        if roll_d is not None and pitch_d is not None:
            if abs(roll_d) > MAX_TILT_DEG or abs(pitch_d) > MAX_TILT_DEG:
                _hold_here(bus, live)
                try:
                    _limp_all(bus, live)
                except Exception:
                    pass
                _set_torque_limit(bus, live, 1000)
                _progress(f"TILT abort roll={roll_d:+.1f}° pitch={pitch_d:+.1f}° — limp")
                return {
                    "ok": False,
                    "aborted": True,
                    "error": f"tilt abort ({roll_d:+.1f}°, {pitch_d:+.1f}°)",
                    "mode": "geometry_plant",
                    "contact_found": contact_found,
                }

        knee_presents: list[float] = []
        knee_speeds: list[float] = []
        for leg in range(6):
            jk = leg * 3 + 2
            fb = _fb(jk)
            if fb is None:
                continue
            knee_presents.append(float(fb["deg"]))
            knee_speeds.append(abs(float(fb.get("speed_deg_s") or 0)))

        knee_now = _median(knee_presents, 0.0)
        knee_spd = _median(knee_speeds, 999.0)
        i_thr = max(CONTACT_CURRENT_FLOOR_A,
                    (_median(base_I, 0.05) + CONTACT_CURRENT_DELTA_A))
        l_thr = max(CONTACT_LOAD_FLOOR_PCT,
                    (_median(base_L, 5.0) + CONTACT_LOAD_DELTA_PCT))
        armed = (knee_now >= CONTACT_ARM_KNEE_DEG
                 and knee_spd <= CONTACT_MAX_KNEE_SPEED)

        for leg in range(6):
            jk = leg * 3 + 2
            fb = _fb(jk)
            if fb is None:
                continue
            present = float(fb["deg"])
            cur = abs(float(fb.get("current_a") or 0.0))
            load = float(fb.get("load_pct") or 0.0)
            if (abs(present) < CONTACT_BASELINE_POSE_DEG
                    and (time.monotonic() - t0) < 3.0):
                base_I.append(cur)
                base_L.append(load)
            if armed and (cur >= i_thr or load >= l_thr):
                hot_since.setdefault(jk, time.monotonic())
            elif jk in hot_since:
                if time.monotonic() - hot_since[jk] > CONTACT_WINDOW_S:
                    del hot_since[jk]

        t_wall = time.monotonic()
        agreed = [j for j, th in hot_since.items()
                  if t_wall - th <= CONTACT_WINDOW_S]
        if armed and len(agreed) >= CONTACT_MIN_KNEE_JOINTS:
            contact_found = True
            contact_joints = sorted(agreed)
            contact_knee = knee_now
            _hold_here(bus, live)
            _progress(f"contact @ knee≈{knee_now:.0f}° "
                      f"({len(agreed)} knees, I≥{i_thr:.2f}A / "
                      f"L≥{l_thr:.0f}%)")
            break

        if (i + 1) % 12 == 0:
            _progress(f"reach {100 * a:.0f}%  knee≈{knee_now:.0f}°  "
                      f"hot={len(agreed)}")

    if abort_check():
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_plant"}

    if not contact_found or contact_knee is None:
        _set_torque_limit(bus, live, 1000)
        _progress("no contact — feet never loaded; abort (not saving)")
        return {
            "ok": False,
            "error": "no ground contact detected — raise platform or start lower",
            "mode": "geometry_plant",
            "contact_found": False,
        }

    # Sample contact pose.
    time.sleep(0.4)
    hips: list[float] = []
    knees: list[float] = []
    for _ in range(6):
        for leg in range(6):
            fh = _fb(leg * 3 + 1)
            fk = _fb(leg * 3 + 2)
            if fh:
                hips.append(float(fh["deg"]))
            if fk:
                knees.append(float(fk["deg"]))
        time.sleep(0.05)
    hip_c = _median(hips, TARGET_HIP_DEG)
    knee_c = _median(knees, contact_knee)
    z_c = foot_z_mm(hip_c, knee_c)
    z_target = z_c + clearance_mm  # less negative = higher chassis
    knee_up = knee_for_foot_z(TARGET_HIP_DEG, z_target)
    if knee_up is None:
        # Fallback: knock ~25° off contact knee.
        knee_up = max(START_KNEE_DEG, knee_c - 25.0)
    knee_up = max(START_KNEE_DEG, min(TARGET_KNEE_DEG, float(knee_up)))

    _progress(f"raise ~{clearance_mm:.0f} mm → hip {TARGET_HIP_DEG:.0f}° / "
              f"knee {knee_up:.0f}° (was {knee_c:.0f}°)")
    tall = _pose(TARGET_HIP_DEG, knee_up)
    if not ease_to_pose(bus, tall, abort_check=abort_check,
                        seconds=CLEARANCE_SECONDS, label="geo_raise"):
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_plant",
                "contact_found": True}

    # Capture full 18-joint snapshot.
    time.sleep(0.5)
    samples: list[list[float]] = []
    for _ in range(8):
        row = []
        ok_row = True
        for j in range(N_JOINTS):
            d = bus.read_position_deg(j)
            if d is None:
                ok_row = False
                break
            row.append(float(d))
        if ok_row:
            samples.append(row)
        time.sleep(0.05)
    if not samples:
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "error": "failed to read joints after raise",
                "mode": "geometry_plant"}

    # Median per joint.
    q_deg = []
    for j in range(N_JOINTS):
        q_deg.append(_median([s[j] for s in samples], samples[0][j]))
    hip_med = _median([q_deg[i] for i in range(1, 18, 3)], TARGET_HIP_DEG)
    knee_med = _median([q_deg[i] for i in range(2, 18, 3)], knee_up)

    path = save_plant_pose(
        hip_med, knee_med,
        extra={
            "joints_deg": [round(x, 3) for x in q_deg],
            "source": "geometry_plant",
            "contact_found": True,
            "contact_knee_deg": round(knee_c, 3),
            "clearance_mm": clearance_mm,
            "contact_joints": contact_joints,
            "foot_z_contact_mm": round(z_c, 2),
            "foot_z_plant_mm": round(foot_z_mm(hip_med, knee_med), 2),
            "target_hip_deg": TARGET_HIP_DEG,
            "target_knee_deg": TARGET_KNEE_DEG,
        },
    )
    _set_torque_limit(bus, live, 1000)
    _hold_here(bus, live)
    _progress(f"saved plant → {path}")
    return {
        "ok": True,
        "mode": "geometry_plant",
        "contact_found": True,
        "contact_knee_deg": round(knee_c, 2),
        "hip_deg": round(hip_med, 2),
        "knee_deg": round(knee_med, 2),
        "clearance_mm": clearance_mm,
        "joints_deg": [round(x, 3) for x in q_deg],
        "pose": standing_pose_degrees(),
        "path": str(path),
        "contact_joints": contact_joints,
        "msg": (f"plant hip {hip_med:+.1f}° / knee {knee_med:+.1f}° "
                f"(~{clearance_mm:.0f} mm above contact)"),
    }
