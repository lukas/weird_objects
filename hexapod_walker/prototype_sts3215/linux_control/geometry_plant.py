"""Geometry + contact plant finder for Phase-1 balance.

Target stance (operator intent):
  * search through hip 0° / knee 80° and continue deeper if unloaded
  * final saved stance raises roughly the requested clearance from contact
  * abort instead of tipping if the body tilt grows while searching

Method:
  1. Soft-torque, yaw=0, hip=0; start with shallow knees (feet high).
  2. Slowly deepen knees, then hip, while watching current/load/lag.
  3. On multi-knee contact, hold and sample.
  4. Compute a taller pose: reduce knee so FK foot-z rises by
     ``clearance_mm``.
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
    load_plant_pose,
    save_plant_pose,
    standing_pose_degrees,
)

# Link lengths (mm) — match tripod_gait / hexapod_prototype.
FEMUR_MM = 90.0
TIBIA_MM = 128.0

# Search down through the real standing family.  Hip 0 / knee 80 reaches
# much lower than the old -20 / 55 crouch; if that still does not load the
# feet, continue toward the captured hardware plant region.
TARGET_HIP_DEG = 20.0
TARGET_KNEE_DEG = 80.0
START_HIP_DEG = 0.0
START_KNEE_DEG = 25.0           # start mid; deepen hip/knee toward crouch
CLEARANCE_MM = 25.0             # ~1 in above contact plane
REACH_SECONDS = 22.0
CLEARANCE_SECONDS = 6.0
REACH_TORQUE = 500
SAMPLE_DT = 0.08
SEARCH_HIP_DEG = 20.0
SEARCH_KNEE_DEG = 100.0

CONTACT_CURRENT_FLOOR_A = 0.16
CONTACT_CURRENT_DELTA_A = 0.08
CONTACT_LOAD_FLOOR_PCT = 16.0
CONTACT_LOAD_DELTA_PCT = 8.0
CONTACT_ARM_KNEE_DEG = 35.0     # arm earlier than old plant (hip stays 0)
CONTACT_MIN_KNEE_JOINTS = 3
CONTACT_WINDOW_S = 0.45
CONTACT_BASELINE_POSE_DEG = 12.0
CONTACT_MAX_KNEE_SPEED = 90.0
CONTACT_POSITION_LAG_DEG = 6.0
# Lag-only contact is useful when current/load are quiet, but it is also the
# easiest way to false-trigger while servos are simply catching up in air.
# Only let lag vote once the search reached the normal plant depth.
CONTACT_LAG_MIN_KNEE_DEG = 80.0
CONTACT_LAG_MAX_KNEE_SPEED = 55.0
MAX_TILT_DELTA_DEG = 15.0       # abort before new lean turns into a tip
MAX_TILT_ABS_DEG = 25.0         # hard cap even if start/platform is tilted
TILT_TRIP_HOLD_S = 0.25         # ignore one-sample accel/IMU bumps
SOFT_SUPPORT_MIN_KNEE_DEG = 85.0
SOFT_SUPPORT_MAX_KNEE_SPEED = 12.0
SOFT_SUPPORT_MIN_LEGS = 2
SOFT_SUPPORT_CURRENT_A = 0.05
SOFT_SUPPORT_LOAD_PCT = 8.0


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


def _angle_delta_deg(now: float, base: float) -> float:
    """Small signed angle difference, robust to +/-180 wrap."""
    return (float(now) - float(base) + 180.0) % 360.0 - 180.0


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
    if knee < -20.0 or knee > 150.0:
        # Try π - asin branch
        pt2 = math.pi - pt
        knee2 = math.degrees(pt2 - p)
        if -20.0 <= knee2 <= 150.0:
            return float(knee2)
        return None
    return float(knee)


def search_pose_at(u: float) -> tuple[float, float]:
    """Piecewise downward search pose, shallow → normal plant → deeper."""
    u = max(0.0, min(1.0, float(u)))
    if u <= 0.55:
        a = u / 0.55
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        return (
            START_HIP_DEG + s * (0.0 - START_HIP_DEG),
            START_KNEE_DEG + s * (TARGET_KNEE_DEG - START_KNEE_DEG),
        )
    a = (u - 0.55) / 0.45
    s = 0.5 - 0.5 * math.cos(math.pi * a)
    return (
        0.0 + s * (SEARCH_HIP_DEG - 0.0),
        TARGET_KNEE_DEG + s * (SEARCH_KNEE_DEG - TARGET_KNEE_DEG),
    )


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
    _progress(f"soft torque → ground search "
              f"(to hip {SEARCH_HIP_DEG:+.0f}° / "
              f"knee {SEARCH_KNEE_DEG:.0f}° if needed)")
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
    base_tilts: list[tuple[float, float]] = []
    for _ in range(5):
        rp = _imu_tilt_deg(bus)
        if rp[0] is not None and rp[1] is not None:
            base_tilts.append((float(rp[0]), float(rp[1])))
        time.sleep(0.04)
    tilt_base = None
    if base_tilts:
        tilt_base = (
            _median([r for r, _p in base_tilts], 0.0),
            _median([p for _r, p in base_tilts], 0.0),
        )
        _progress(
            f"tilt baseline roll={tilt_base[0]:+.1f}° "
            f"pitch={tilt_base[1]:+.1f}°")

    # --- blend toward ground; contact usually appears before full depth ---
    _progress("reaching for ground (current/load/lag contact watch)…")
    n = max(2, int(reach_seconds / SAMPLE_DT))
    t0 = time.monotonic()
    base_I: list[float] = []
    base_L: list[float] = []
    hot_since: dict[int, float] = {}
    tilt_hot_since: float | None = None
    contact_found = False
    contact_knee = None
    contact_joints: list[int] = []
    last_hip_cmd = START_HIP_DEG
    last_knee_cmd = START_KNEE_DEG
    last_knee_now = 0.0
    last_knee_spd = 999.0
    last_support_legs: list[int] = []

    for i in range(n):
        if abort_check():
            _set_torque_limit(bus, live, 1000)
            return {"ok": False, "aborted": True, "mode": "geometry_plant"}
        a = (i + 1) / n
        s = 0.5 - 0.5 * math.cos(math.pi * a)
        hip_cmd, knee_cmd = search_pose_at(s)
        last_hip_cmd, last_knee_cmd = hip_cmd, knee_cmd
        goal = _pose(hip_cmd, knee_cmd)
        _write_pose(bus, goal, live, speed=160, acc=12)
        time.sleep(SAMPLE_DT)

        roll_d, pitch_d = _imu_tilt_deg(bus)
        if roll_d is not None and pitch_d is not None:
            if tilt_base is not None:
                roll_trip = _angle_delta_deg(roll_d, tilt_base[0])
                pitch_trip = _angle_delta_deg(pitch_d, tilt_base[1])
            else:
                roll_trip = float(roll_d)
                pitch_trip = float(pitch_d)
            trip_raw = (
                max(abs(roll_trip), abs(pitch_trip)) > MAX_TILT_DELTA_DEG
                or max(abs(float(roll_d)), abs(float(pitch_d))) > MAX_TILT_ABS_DEG)
            now = time.monotonic()
            if trip_raw:
                if tilt_hot_since is None:
                    tilt_hot_since = now
            else:
                tilt_hot_since = None
            if tilt_hot_since is not None and now - tilt_hot_since >= TILT_TRIP_HOLD_S:
                _hold_here(bus, live)
                try:
                    _limp_all(bus, live)
                except Exception:
                    pass
                _set_torque_limit(bus, live, 1000)
                _progress(
                    f"TILT abort roll={roll_d:+.1f}° pitch={pitch_d:+.1f}° "
                    f"(Δ{roll_trip:+.1f}°/{pitch_trip:+.1f}°) — limp")
                return {
                    "ok": False,
                    "aborted": True,
                    "error": (
                        f"tilt abort ({roll_d:+.1f}°, {pitch_d:+.1f}°; "
                        f"Δ{roll_trip:+.1f}°/{pitch_trip:+.1f}°)"),
                    "mode": "geometry_plant",
                    "contact_found": contact_found,
                    "tilt_baseline_deg": (
                        None if tilt_base is None else
                        [round(tilt_base[0], 2), round(tilt_base[1], 2)]),
                    "tilt_delta_deg": [
                        round(roll_trip, 2), round(pitch_trip, 2)],
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
        last_knee_now = knee_now
        last_knee_spd = knee_spd
        i_thr = max(CONTACT_CURRENT_FLOOR_A,
                    (_median(base_I, 0.05) + CONTACT_CURRENT_DELTA_A))
        l_thr = max(CONTACT_LOAD_FLOOR_PCT,
                    (_median(base_L, 5.0) + CONTACT_LOAD_DELTA_PCT))
        armed = (knee_now >= CONTACT_ARM_KNEE_DEG
                 and knee_spd <= CONTACT_MAX_KNEE_SPEED)

        support_legs: set[int] = set()
        for leg in range(6):
            jk = leg * 3 + 2
            fb = _fb(jk)
            if fb is None:
                continue
            present = float(fb["deg"])
            cur = abs(float(fb.get("current_a") or 0.0))
            load = float(fb.get("load_pct") or 0.0)
            speed = abs(float(fb.get("speed_deg_s") or 0.0))
            lag = float(knee_cmd) - present
            if (abs(present) < CONTACT_BASELINE_POSE_DEG
                    and (time.monotonic() - t0) < 3.0):
                base_I.append(cur)
                base_L.append(load)
            lag_contact = (
                knee_cmd >= CONTACT_LAG_MIN_KNEE_DEG
                and lag >= CONTACT_POSITION_LAG_DEG
                and speed <= CONTACT_LAG_MAX_KNEE_SPEED)
            if armed and (cur >= i_thr or load >= l_thr or lag_contact):
                hot_since.setdefault(jk, time.monotonic())
            elif jk in hot_since:
                if time.monotonic() - hot_since[jk] > CONTACT_WINDOW_S:
                    del hot_since[jk]
            if (cur >= SOFT_SUPPORT_CURRENT_A
                    or load >= SOFT_SUPPORT_LOAD_PCT
                    or lag_contact):
                support_legs.add(leg)
            fh = _fb(leg * 3 + 1)
            if fh is not None:
                hcur = abs(float(fh.get("current_a") or 0.0))
                hload = float(fh.get("load_pct") or 0.0)
                if hcur >= SOFT_SUPPORT_CURRENT_A or hload >= SOFT_SUPPORT_LOAD_PCT:
                    support_legs.add(leg)
        last_support_legs = sorted(support_legs)

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
            _progress(f"reach {100 * a:.0f}%  cmd hip {hip_cmd:+.0f}° / "
                      f"knee {knee_cmd:.0f}°  present knee≈{knee_now:.0f}°  "
                      f"hot={len(agreed)}")

    if abort_check():
        _set_torque_limit(bus, live, 1000)
        return {"ok": False, "aborted": True, "mode": "geometry_plant"}

    if not contact_found or contact_knee is None:
        stable_full_reach = (
            last_hip_cmd >= SEARCH_HIP_DEG - 1.0
            and last_knee_cmd >= SEARCH_KNEE_DEG - 2.0
            and last_knee_now >= SOFT_SUPPORT_MIN_KNEE_DEG
            and last_knee_spd <= SOFT_SUPPORT_MAX_KNEE_SPEED
            and len(last_support_legs) >= SOFT_SUPPORT_MIN_LEGS)
        try:
            plant = load_plant_pose()
        except Exception:
            plant = {}
        using_existing = (
            stable_full_reach
            and bool(plant.get("learned"))
            and isinstance(plant.get("joints_deg"), list)
            and len(plant.get("joints_deg") or []) == N_JOINTS)
        if using_existing:
            pose = standing_pose_degrees()
            hip = float(plant.get("hip_deg", TARGET_HIP_DEG))
            knee = float(plant.get("knee_deg", TARGET_KNEE_DEG))
            _progress(
                "no sharp load spike, but reached stable support; "
                "using existing learned plant")
            if not ease_to_pose(
                    bus, pose, abort_check=abort_check, seconds=2.0,
                    label="existing learned plant"):
                _set_torque_limit(bus, live, 1000)
                return {"ok": False, "aborted": True,
                        "mode": "geometry_plant",
                        "contact_found": False,
                        "used_existing_plant": True}
            _set_torque_limit(bus, live, 1000)
            _hold_here(bus, live)
            return {
                "ok": True,
                "mode": "geometry_plant",
                "contact_found": False,
                "used_existing_plant": True,
                "saved": False,
                "soft_support_legs": last_support_legs,
                "searched_hip_deg": round(last_hip_cmd, 2),
                "searched_knee_deg": round(last_knee_cmd, 2),
                "present_knee_deg": round(last_knee_now, 2),
                "hip_deg": round(hip, 2),
                "knee_deg": round(knee, 2),
                "pose": pose,
                "msg": (
                    "no force spike; stable support reached, using existing "
                    f"plant hip {hip:+.1f}° / knee {knee:+.1f}°"),
            }
        _set_torque_limit(bus, live, 1000)
        _progress("no contact after full depth — not saving")
        return {
            "ok": False,
            "error": (
                "no ground contact detected after search to "
                f"hip {SEARCH_HIP_DEG:+.0f}° / knee {SEARCH_KNEE_DEG:.0f}°"),
            "mode": "geometry_plant",
            "contact_found": False,
            "stable_full_reach": stable_full_reach,
            "soft_support_legs": last_support_legs,
            "present_knee_deg": round(last_knee_now, 2),
            "searched_hip_deg": SEARCH_HIP_DEG,
            "searched_knee_deg": SEARCH_KNEE_DEG,
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
    # Body-frame foot z is more negative when the chassis is higher above a
    # fixed floor.  After contact, raising the chassis by ``clearance_mm``
    # means the planted feet end up farther below the body.
    z_target = z_c - clearance_mm
    knee_up = knee_for_foot_z(TARGET_HIP_DEG, z_target)
    if knee_up is None:
        # Fallback: knock ~25° off contact knee.
        knee_up = max(START_KNEE_DEG, knee_c - 25.0)
    knee_up = max(START_KNEE_DEG, min(SEARCH_KNEE_DEG, float(knee_up)))

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

    z_plant = foot_z_mm(hip_med, knee_med)
    body_raise_mm = z_c - z_plant
    min_raise_mm = max(8.0, 0.45 * clearance_mm)
    if body_raise_mm < min_raise_mm:
        _set_torque_limit(bus, live, 1000)
        _hold_here(bus, live)
        err = (f"raise solve too shallow ({body_raise_mm:.1f} mm; "
               f"wanted ~{clearance_mm:.0f} mm)")
        _progress(err + " — not saving")
        return {
            "ok": False,
            "error": err,
            "mode": "geometry_plant",
            "contact_found": True,
            "contact_knee_deg": round(knee_c, 2),
            "hip_deg": round(hip_med, 2),
            "knee_deg": round(knee_med, 2),
            "clearance_mm": clearance_mm,
            "body_raise_mm": round(body_raise_mm, 2),
            "contact_joints": contact_joints,
        }

    path = save_plant_pose(
        hip_med, knee_med,
        extra={
            "joints_deg": [round(x, 3) for x in q_deg],
            "source": "geometry_plant",
            "contact_found": True,
            "contact_knee_deg": round(knee_c, 3),
            "clearance_mm": clearance_mm,
            "body_raise_mm": round(body_raise_mm, 2),
            "contact_joints": contact_joints,
            "foot_z_contact_mm": round(z_c, 2),
            "foot_z_plant_mm": round(z_plant, 2),
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
        "body_raise_mm": round(body_raise_mm, 2),
        "joints_deg": [round(x, 3) for x in q_deg],
        "pose": standing_pose_degrees(),
        "path": str(path),
        "contact_joints": contact_joints,
        "msg": (f"plant hip {hip_med:+.1f}° / knee {knee_med:+.1f}° "
                f"(~{body_raise_mm:.0f} mm above contact)"),
    }
