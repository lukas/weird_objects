"""Plant-height contact calibration: reach down until resistance, save stand home.

Slowly drives all legs from sit zero toward a deep reach pose. When enough
hip/knee joints show a current or load bump (feet hitting the surface),
holds, takes median hip/knee, and writes ``logs/plant_pose.json`` so
``standing_pose_degrees()`` becomes the real plant height.
"""
from __future__ import annotations

import csv
import statistics
import sys
import time
from pathlib import Path
from typing import Callable

_HERE = Path(__file__).resolve().parent
for _p in (_HERE / "urt2_setup", _HERE / "vendor",
           Path.home() / "hexapod_sts" / "urt2_setup", _HERE):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from feetech_bus import (  # noqa: E402
    N_JOINTS, clear_plant_pose, joint_to_servo_id, load_plant_pose,
    save_plant_pose, standing_pose_degrees,
)

LOG_DIR = _HERE / "logs"

# Deep reach target.  Knee is an absolute tibia angle, not the old
# ``hip+knee`` serial convention; +80° would drive well past normal contact.
REACH_HIP_DEG = 20.0
REACH_KNEE_DEG = 55.0
REACH_SECONDS = 10.0
REACH_TORQUE = 550

# Contact: soft torque barely spikes absolute current.  Hips travel ~20°
# while knees travel toward the measured plant range, so hips can still
# arrive early and stall/load.  Arm once knees are plausibly near the floor;
# require knee joints in the agreement set; ignore while knees are racing.
CONTACT_CURRENT_FLOOR_A = 0.16
CONTACT_CURRENT_DELTA_A = 0.08
CONTACT_CURRENT_REL = 3.0
CONTACT_LOAD_FLOOR_PCT = 16.0
CONTACT_LOAD_DELTA_PCT = 8.0
CONTACT_BASELINE_POSE_DEG = 8.0   # |θ| below this → still free-air baseline
CONTACT_ARM_KNEE_DEG = 28.0
CONTACT_MIN_JOINTS = 3
CONTACT_MIN_KNEE_JOINTS = 2
CONTACT_MAX_KNEE_ERR_DEG = 18.0   # |cmd − present| on median knee
CONTACT_MAX_KNEE_SPEED = 80.0     # deg/s — still swinging = not planted
CONTACT_WINDOW_S = 0.45
HOLD_AFTER_S = 0.7
SAMPLE_DT = 0.08
MIN_SAVE_HIP_DEG = 5.0
MIN_SAVE_KNEE_DEG = 18.0
# Legacy kwargs (API); real thresholds are baseline-relative.
CONTACT_CURRENT_A = CONTACT_CURRENT_FLOOR_A
CONTACT_LOAD_PCT = CONTACT_LOAD_FLOOR_PCT


def plant_state() -> dict:
    """Current learned / default stand plant for the UI."""
    p = load_plant_pose()
    joints = p.get("joints_deg")
    return {
        "ok": True,
        "hip_deg": p["hip_deg"],
        "knee_deg": p["knee_deg"],
        "joints_deg": joints,
        "has_joints": bool(joints and len(joints) == 18),
        "learned": p["learned"],
        "contact_found": p.get("contact_found", False),
        "path": p.get("path"),
        "timestamp": p.get("timestamp"),
        "source": p.get("source"),
        "pose": standing_pose_degrees(),
    }


def reset_plant_pose() -> dict:
    cleared = clear_plant_pose()
    st = plant_state()
    st["cleared"] = cleared
    st["msg"] = ("reset to default hip +19° / knee +28°"
                 if cleared or not st["learned"] else "already default")
    return st


def _median(vals: list[float], default: float) -> float:
    if not vals:
        return default
    return float(statistics.median(vals))


def run_plant_calibrate(
    bus,
    *,
    names: dict[int, str] | None = None,
    log_path: Path | None = None,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
    reach_seconds: float = REACH_SECONDS,
    contact_current_a: float = CONTACT_CURRENT_FLOOR_A,
    contact_load_pct: float = CONTACT_LOAD_FLOOR_PCT,
) -> dict:
    """Sit zero → slow reach → contact → save median plant. Returns summary."""
    abort_check = abort_check or (lambda: False)

    try:
        from inplace_demos import (
            _enable_torque, _hold_here, _live_robot_ids, _set_torque_limit,
            _write_pose, go_to_zero_pose,
        )
        from motion_telemetry import joint_name as _joint_name
    except ImportError as e:
        return {"ok": False, "error": f"inplace_demos missing: {e}",
                "mode": "plant"}

    live = _live_robot_ids(bus)
    if len(live) < 6:
        return {"ok": False, "error": f"need more servos (live={len(live)})",
                "mode": "plant", "live": sorted(live)}

    path = Path(log_path) if log_path else (
        LOG_DIR / f"plant_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    path.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()

    def _progress(msg: str, **extra):
        if on_progress:
            on_progress({"msg": msg, "t_s": round(time.monotonic() - t0, 2),
                         **extra})

    def _fb(j: int) -> dict | None:
        try:
            return bus.read_feedback(j)
        except Exception:
            return None

    goal = []
    for _leg in range(6):
        goal.extend([0.0, REACH_HIP_DEG, REACH_KNEE_DEG])

    _enable_torque(bus, live)
    _set_torque_limit(bus, live, REACH_TORQUE)
    status = "done"
    contact_found = False
    contact_joints: list[int] = []
    peak_current = 0.0
    peak_load = 0.0
    hip_med = knee_med = None
    saved_path = None

    try:
        _progress("homing to sit zero…")
        if not go_to_zero_pose(bus, abort_check=abort_check, seconds=3.5):
            return {"ok": False, "aborted": True, "mode": "plant",
                    "error": "aborted during sit home", "log": str(path)}

        if abort_check():
            return {"ok": False, "aborted": True, "mode": "plant",
                    "log": str(path)}

        _progress(
            f"reaching plant (hip {REACH_HIP_DEG:.0f}° / "
            f"knee {REACH_KNEE_DEG:.0f}°)…")

        # Per-joint speeds so hip and knee arrive together.
        # Same speed for all made hips stall early → false "contact".
        from feetech_bus import COUNTS_PER_DEG, normalize_acc, normalize_speed
        start = [0.0] * N_JOINTS
        for j in range(N_JOINTS):
            if joint_to_servo_id(j) not in live:
                continue
            d = bus.read_position_deg(j)
            if d is not None:
                start[j] = float(d)
        seconds = max(6.0, float(reach_seconds))
        speeds = [0] * N_JOINTS
        for j in range(N_JOINTS):
            if joint_to_servo_id(j) not in live:
                continue
            delta = abs(goal[j] - start[j])
            sp = int(min(220, max(20, delta * COUNTS_PER_DEG / seconds)))
            speeds[j] = normalize_speed(sp)
        acc = normalize_acc(int(max(2, min(12, 40 / seconds))))
        _write_pose(bus, goal, live, speed=max(speeds) or 80, acc=acc,
                    speeds=speeds)

        hot_since: dict[int, float] = {}
        base_I: list[float] = []
        base_L: list[float] = []
        i_thr = max(contact_current_a, CONTACT_CURRENT_FLOOR_A)
        l_thr = max(contact_load_pct, CONTACT_LOAD_FLOOR_PCT)
        samples = 0
        with path.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=[
                "t_s", "phase", "joint", "id", "name",
                "cmd_deg", "present_deg", "err_deg",
                "speed_deg_s", "load_pct", "current_a", "volt", "hot",
                "i_thr", "l_thr",
            ])
            w.writeheader()

            timeout = seconds * 1.5 + 2.0
            reach_t0 = time.monotonic()
            tick = 0
            while time.monotonic() - reach_t0 < timeout:
                if abort_check():
                    status = "aborted"
                    _hold_here(bus, live)
                    break
                time.sleep(SAMPLE_DT)
                tick += 1
                now = time.monotonic() - t0
                # Sparse full-leg sample (MCU bridge).
                if tick % 2 != 0:
                    continue

                # Refresh thresholds from free-air baseline once we have some.
                if len(base_I) >= 6:
                    b_i = statistics.median(base_I)
                    b_l = statistics.median(base_L) if base_L else 0.0
                    i_thr = max(
                        CONTACT_CURRENT_FLOOR_A,
                        b_i + CONTACT_CURRENT_DELTA_A,
                        b_i * CONTACT_CURRENT_REL,
                        float(contact_current_a),
                    )
                    l_thr = max(
                        CONTACT_LOAD_FLOOR_PCT,
                        b_l + CONTACT_LOAD_DELTA_PCT,
                        float(contact_load_pct),
                    )

                # Knee progress — hips arrive early and stall, so contact
                # is armed/accepted only from knee depth + tracking.
                knee_presents: list[float] = []
                knee_errs: list[float] = []
                knee_speeds: list[float] = []
                for leg in range(6):
                    jk = leg * 3 + 2
                    if joint_to_servo_id(jk) not in live:
                        continue
                    fb_k = _fb(jk)
                    if fb_k is None:
                        continue
                    pk = float(fb_k["deg"])
                    knee_presents.append(pk)
                    knee_errs.append(abs(REACH_KNEE_DEG - pk))
                    knee_speeds.append(abs(float(fb_k.get("speed_deg_s") or 0)))
                knee_now = _median(knee_presents, 0.0)
                knee_err = _median(knee_errs, 99.0)
                knee_spd = _median(knee_speeds, 999.0)
                contact_armed = (
                    knee_now >= CONTACT_ARM_KNEE_DEG
                    and knee_err <= CONTACT_MAX_KNEE_ERR_DEG
                    and knee_spd <= CONTACT_MAX_KNEE_SPEED
                )

                for j in range(N_JOINTS):
                    sid = joint_to_servo_id(j)
                    if sid not in live:
                        continue
                    axis = j % 3
                    fb = _fb(j)
                    if fb is None:
                        continue
                    present = float(fb["deg"])
                    load = float(fb.get("load_pct") or 0.0)
                    cur = abs(float(fb.get("current_a") or 0.0))
                    volt = float(fb.get("volt") or 0.0)
                    peak_current = max(peak_current, cur)
                    peak_load = max(peak_load, load)
                    cmd = float(goal[j])
                    hot = False
                    # Only KNEES vote for contact — hip stall is a false cue.
                    if axis == 2:
                        if (abs(present) < CONTACT_BASELINE_POSE_DEG
                                and (time.monotonic() - reach_t0) < 3.5):
                            base_I.append(cur)
                            base_L.append(load)
                        if contact_armed and (cur >= i_thr or load >= l_thr):
                            hot = True
                            hot_since.setdefault(j, time.monotonic())
                        elif j in hot_since:
                            if time.monotonic() - hot_since[j] > CONTACT_WINDOW_S:
                                del hot_since[j]
                    elif axis == 1:
                        # Still log hip baseline early; never mark hip hot.
                        if (abs(present) < CONTACT_BASELINE_POSE_DEG
                                and (time.monotonic() - reach_t0) < 3.5):
                            base_I.append(cur)
                            base_L.append(load)
                        if j in hot_since:
                            del hot_since[j]
                    name = (names[sid] if names and sid in names
                            else _joint_name(j, names))
                    w.writerow({
                        "t_s": f"{now:.3f}",
                        "phase": "reach",
                        "joint": j,
                        "id": sid,
                        "name": name,
                        "cmd_deg": f"{cmd:.2f}",
                        "present_deg": f"{present:.2f}",
                        "err_deg": f"{cmd - present:.2f}",
                        "speed_deg_s": f"{float(fb.get('speed_deg_s') or 0):.1f}",
                        "load_pct": f"{load:.1f}",
                        "current_a": f"{cur:.3f}",
                        "volt": f"{volt:.2f}",
                        "hot": int(hot),
                        "i_thr": f"{i_thr:.3f}",
                        "l_thr": f"{l_thr:.1f}",
                    })
                    samples += 1

                # Knees hot within the agreement window.
                t_wall = time.monotonic()
                agreed = [j for j, t_h in hot_since.items()
                          if t_wall - t_h <= CONTACT_WINDOW_S and j % 3 == 2]
                if (contact_armed
                        and len(agreed) >= max(CONTACT_MIN_JOINTS,
                                               CONTACT_MIN_KNEE_JOINTS)):
                    contact_found = True
                    contact_joints = sorted(agreed)
                    _progress(
                        f"contact — {len(agreed)} knees "
                        f"(knee≈{knee_now:.0f}° err≈{knee_err:.0f}° · "
                        f"I≥{i_thr:.2f}A or load≥{l_thr:.0f}%)")
                    _hold_here(bus, live)
                    break

                # Settled near deep knee goal — accept only if knees made it
                # and I/load still look planted (not hip-stall mid-reach).
                worst_knee = 0.0
                n_knee = 0
                for leg in range(6):
                    jk = leg * 3 + 2
                    if joint_to_servo_id(jk) not in live:
                        continue
                    d = bus.read_position_deg(jk)
                    if d is None:
                        continue
                    n_knee += 1
                    worst_knee = max(worst_knee, abs(REACH_KNEE_DEG - float(d)))
                elapsed = time.monotonic() - reach_t0
                if (n_knee >= 4 and worst_knee <= CONTACT_MAX_KNEE_ERR_DEG
                        and elapsed >= seconds * 0.75):
                    planted_end = (
                        peak_current >= i_thr and peak_load >= l_thr
                        and knee_now >= MIN_SAVE_KNEE_DEG
                    )
                    if planted_end:
                        contact_found = True
                        contact_joints = sorted(
                            j for j in hot_since if j % 3 == 2)
                        _progress(
                            f"deep knees + elevated I/load "
                            f"(peak {peak_current:.2f}A / {peak_load:.0f}%) "
                            f"— treating as contact")
                    else:
                        _progress(
                            f"knees near goal (err≤{worst_knee:.0f}°) "
                            f"— no contact bump")
                    _hold_here(bus, live)
                    break

        if status == "aborted" or abort_check():
            _set_torque_limit(bus, live, 1000)
            return {
                "ok": False, "aborted": True, "mode": "plant",
                "contact_found": contact_found, "log": str(path),
                "log_name": path.name, "samples": samples,
            }

        # Snapshot present hip/knee.
        _progress("sampling plant pose…")
        time.sleep(HOLD_AFTER_S)
        hips: list[float] = []
        knees: list[float] = []
        per_leg = []
        for leg in range(6):
            jh, jk = leg * 3 + 1, leg * 3 + 2
            if joint_to_servo_id(jh) not in live or joint_to_servo_id(jk) not in live:
                continue
            dh = bus.read_position_deg(jh)
            dk = bus.read_position_deg(jk)
            if dh is None or dk is None:
                continue
            hips.append(float(dh))
            knees.append(float(dk))
            per_leg.append({"leg": leg, "hip_deg": round(float(dh), 2),
                            "knee_deg": round(float(dk), 2)})

        hip_med = _median(hips, REACH_HIP_DEG)
        knee_med = _median(knees, REACH_KNEE_DEG)

        too_shallow = (
            hip_med < MIN_SAVE_HIP_DEG or knee_med < MIN_SAVE_KNEE_DEG
        )
        rejected_shallow = False
        if contact_found and too_shallow:
            rejected_shallow = True
            _progress(
                f"reject shallow contact hip {hip_med:.1f}° / "
                f"knee {knee_med:.1f}° (need hip≥{MIN_SAVE_HIP_DEG:.0f}° "
                f"knee≥{MIN_SAVE_KNEE_DEG:.0f}°) — not saved")
            contact_found = False

        if not contact_found:
            _set_torque_limit(bus, live, 1000)
            return {
                "ok": True,
                "aborted": False,
                "mode": "plant",
                "saved": False,
                "contact_found": False,
                "hip_deg": round(hip_med, 2),
                "knee_deg": round(knee_med, 2),
                "per_leg": per_leg,
                "peak_current_a": round(peak_current, 3),
                "peak_load_pct": round(peak_load, 1),
                "log": str(path),
                "log_name": path.name,
                "samples": samples,
                "plant": plant_state(),
                "hint": (
                    "False early contact rejected — NOT saved. "
                    f"Need knee≥{CONTACT_ARM_KNEE_DEG:.0f}° before "
                    f"detect and final hip≥{MIN_SAVE_HIP_DEG:.0f}° / "
                    f"knee≥{MIN_SAVE_KNEE_DEG:.0f}°. Retry plant height."
                    if rejected_shallow else
                    "No contact bump — plant NOT saved. "
                    "Check platform height / thresholds, then retry."
                ),
            }

        saved_path = save_plant_pose(
            hip_med, knee_med,
            extra={
                "contact_found": True,
                "contact_joints": contact_joints,
                "contact_current_a": contact_current_a,
                "contact_load_pct": contact_load_pct,
                "peak_current_a": round(peak_current, 3),
                "peak_load_pct": round(peak_load, 1),
                "per_leg": per_leg,
                "log": path.name,
            },
        )
        # Hold the learned plant.
        plant_pose = standing_pose_degrees()
        _write_pose(bus, plant_pose, live, speed=120, acc=10)
        time.sleep(0.8)
        _progress(
            f"saved plant hip {hip_med:.1f}° / knee {knee_med:.1f}°")

        return {
            "ok": True,
            "aborted": False,
            "mode": "plant",
            "saved": True,
            "contact_found": True,
            "contact_joints": contact_joints,
            "hip_deg": round(hip_med, 2),
            "knee_deg": round(knee_med, 2),
            "per_leg": per_leg,
            "peak_current_a": round(peak_current, 3),
            "peak_load_pct": round(peak_load, 1),
            "plant_path": str(saved_path),
            "log": str(path),
            "log_name": path.name,
            "samples": samples,
            "plant": plant_state(),
            "hint": (
                f"Stand home set to hip {hip_med:.1f}° / knee {knee_med:.1f}°. "
                "Stand zero + planted demos will use this."
            ),
        }
    finally:
        try:
            _set_torque_limit(bus, live, 1000)
        except Exception:
            pass
