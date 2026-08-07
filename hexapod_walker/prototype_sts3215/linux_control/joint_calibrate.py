"""Joint calibration: step tracking + shake/hold hunt diagnostics.

Used by the web Calibrate tab. Writes a CSV of every sample plus a
per-joint summary so stuck joints (step) and hunting/shaky holds (shake)
show up as yellow/red.
"""
from __future__ import annotations

import csv
import math
import sys
import time
from pathlib import Path
from typing import Callable

_HERE = Path(__file__).resolve().parent
for _p in (_HERE / "urt2_setup", _HERE / "vendor",
           Path.home() / "hexapod_sts" / "urt2_setup", _HERE):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from feetech_bus import N_JOINTS, joint_to_servo_id  # noqa: E402

AXIS = ("yaw", "hip", "knee")
LOG_DIR = Path(__file__).resolve().parent / "logs"

# Settle / move budget for a ~10° step at moderate speed.
DEFAULT_STEP_DEG = 10.0
DEFAULT_SPEED = 500
DEFAULT_ACC = 30
MOVE_TIMEOUT_S = 1.8
SAMPLE_DT = 0.05
RETURN_PAUSE_S = 0.55
# Pre-step home: sit zero so +step° isn't launched from a travel limit.
HOME_SPEED = 400
HOME_ACC = 20
HOME_TIMEOUT_S = 4.5
HOME_SETTLE_DEG = 2.5

# Shake / hold: tiny nudge then measure wobble while holding.
DEFAULT_NUDGE_DEG = 2.0
SHAKE_SPEED = 300
SHAKE_ACC = 15
SHAKE_ARRIVE_S = 1.2
SHAKE_HOLD_S = 1.6
SHAKE_HOLD_IGNORE_S = 0.35  # skip early hold samples (still settling)


def joint_name(joint: int, names: dict[int, str] | None = None) -> str:
    sid = joint_to_servo_id(joint)
    if names and sid in names:
        return names[sid]
    leg, axis = divmod(joint, 3)
    return f"L{leg} {AXIS[axis]}"


def joints_for_axis(axis: str | None) -> list[int]:
    """``None``/``all`` → 0..17; ``yaw``/``hip``/``knee`` → that column."""
    if not axis or axis in ("all", "*"):
        return list(range(N_JOINTS))
    key = axis.strip().lower()
    if key in AXIS:
        i = AXIS.index(key)
        return [j for j in range(N_JOINTS) if j % 3 == i]
    raise ValueError(f"bad axis {axis!r}; use all|yaw|hip|knee")


def _grade_step(tracking_pct: float, peak_load: float, moved: bool) -> str:
    if not moved or tracking_pct < 35.0:
        return "red"
    if tracking_pct < 75.0 or peak_load >= 55.0:
        return "yellow"
    return "green"


def _grade_shake(pos_pp: float, rms_err: float, speed_rms: float,
                 reached: bool) -> str:
    if not reached or pos_pp >= 1.5 or rms_err >= 0.85:
        return "red"
    if pos_pp >= 0.55 or rms_err >= 0.35 or speed_rms >= 25.0:
        return "yellow"
    return "green"


def default_log_path(prefix: str = "calibrate") -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return LOG_DIR / f"{prefix}_{stamp}.csv"


def _discover_live(bus) -> tuple[list[int], list[float]]:
    live: list[int] = []
    pose = [0.0] * N_JOINTS
    for j in range(N_JOINTS):
        try:
            d = bus.read_position_deg(j)
        except Exception:
            d = None
        if d is None:
            continue
        pose[j] = float(d)
        live.append(j)
    return live, pose


def _sample_row(now: float, phase: str, joint: int, name: str,
                cmd: float, fb: dict) -> dict:
    present = float(fb["deg"])
    return {
        "t_s": f"{now:.3f}",
        "phase": phase,
        "joint": joint,
        "id": joint_to_servo_id(joint),
        "name": name,
        "cmd_deg": f"{cmd:.2f}",
        "present_deg": f"{present:.2f}",
        "err_deg": f"{cmd - present:.2f}",
        "speed_deg_s": f"{float(fb.get('speed_deg_s') or 0):.1f}",
        "load_pct": f"{float(fb.get('load_pct') or 0):.1f}",
        "current_a": f"{float(fb.get('current_a') or 0):.3f}",
        "volt": f"{float(fb.get('volt') or 0):.2f}",
    }


def _home_sit_zero(
    bus, live: list[int], pose: list[float], *,
    names: dict[int, str] | None,
    abort_check: Callable[[], bool],
    on_progress: Callable[[dict], None] | None,
    t0: float,
    writer,
    samples: list,
    total: int,
) -> tuple[bool, float | None]:
    """Glide all live joints to 0°. Returns (ok, max_abs_err)."""
    def _progress(msg: str, **extra):
        if on_progress:
            on_progress({"msg": msg, "t_s": round(time.monotonic() - t0, 2),
                         **extra})

    def _fb(j: int) -> dict | None:
        try:
            return bus.read_feedback(j)
        except Exception:
            return None

    _progress("homing to sit zero…", index=0, total=total)
    for j in live:
        pose[j] = 0.0
    bus.write_all(pose, speed=HOME_SPEED, acc=HOME_ACC)
    home_ok = False
    home_max_err: float | None = None
    home_t0 = time.monotonic()
    while time.monotonic() - home_t0 < HOME_TIMEOUT_S:
        if abort_check():
            break
        worst = 0.0
        for j in live:
            fb = _fb(j)
            now = time.monotonic() - t0
            if not fb:
                continue
            present = float(fb["deg"])
            pose[j] = present
            worst = max(worst, abs(present))
            sample = _sample_row(now, "home", j, joint_name(j, names), 0.0, fb)
            writer.writerow(sample)
            samples.append(sample)
        if worst <= HOME_SETTLE_DEG and (time.monotonic() - home_t0) > 0.4:
            home_ok = True
            home_max_err = round(worst, 2)
            break
        time.sleep(SAMPLE_DT)
    for j in live:
        d = bus.read_position_deg(j)
        if d is not None:
            pose[j] = float(d)
    if home_max_err is None:
        home_max_err = round(max((abs(pose[j]) for j in live), default=0.0), 2)
        home_ok = home_max_err <= HOME_SETTLE_DEG
    _progress(
        f"at sit zero (max |θ|={home_max_err}°)"
        if home_ok else
        f"home incomplete (max |θ|={home_max_err}°) — continuing",
        home_ok=home_ok, home_max_err_deg=home_max_err,
    )
    return home_ok, home_max_err


def run_calibrate(
    bus,
    *,
    mode: str = "step",
    step_deg: float = DEFAULT_STEP_DEG,
    nudge_deg: float = DEFAULT_NUDGE_DEG,
    axis: str | None = "all",
    names: dict[int, str] | None = None,
    log_path: Path | None = None,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
) -> dict:
    """Dispatch step or shake calibrate."""
    key = (mode or "step").strip().lower()
    if key in ("shake", "hold", "hunt"):
        return run_shake_calibrate(
            bus, nudge_deg=nudge_deg, axis=axis, names=names,
            log_path=log_path, abort_check=abort_check,
            on_progress=on_progress,
        )
    return run_step_calibrate(
        bus, step_deg=step_deg, axis=axis, names=names,
        log_path=log_path, abort_check=abort_check,
        on_progress=on_progress,
    )


def run_step_calibrate(
    bus,
    *,
    step_deg: float = DEFAULT_STEP_DEG,
    axis: str | None = "all",
    speed: int = DEFAULT_SPEED,
    acc: int = DEFAULT_ACC,
    names: dict[int, str] | None = None,
    log_path: Path | None = None,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
) -> dict:
    """Run +step from sit zero on each selected joint; return summary + log."""
    abort_check = abort_check or (lambda: False)
    step_deg = max(3.0, min(20.0, float(step_deg)))
    speed = int(max(100, min(2000, speed)))
    acc = int(max(5, min(100, acc)))
    try:
        joints = joints_for_axis(axis)
    except ValueError as e:
        return {"ok": False, "error": str(e)}

    path = Path(log_path) if log_path else default_log_path("calibrate")
    path.parent.mkdir(parents=True, exist_ok=True)

    live, pose = _discover_live(bus)
    targets = [j for j in joints if j in live]
    if not targets:
        return {"ok": False, "error": "no live joints in selection",
                "live": live, "log": str(path), "mode": "step"}

    rows: list[dict] = []
    samples: list[dict] = []
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

    def _write_pose(p: list[float], *, spd: int | None = None,
                    ac: int | None = None) -> None:
        bus.write_all(p, speed=spd if spd is not None else speed,
                      acc=ac if ac is not None else acc)

    def _refresh_pose() -> None:
        for j in live:
            d = bus.read_position_deg(j)
            if d is not None:
                pose[j] = float(d)

    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=[
            "t_s", "phase", "joint", "id", "name",
            "cmd_deg", "present_deg", "err_deg",
            "speed_deg_s", "load_pct", "current_a", "volt",
        ])
        w.writeheader()

        home_ok, home_max_err = False, None
        if not abort_check():
            home_ok, home_max_err = _home_sit_zero(
                bus, live, pose, names=names, abort_check=abort_check,
                on_progress=on_progress, t0=t0, writer=w, samples=samples,
                total=len(targets),
            )

        for ji, joint in enumerate(targets):
            if abort_check():
                _progress("aborted", done=False)
                break
            name = joint_name(joint, names)
            sid = joint_to_servo_id(joint)
            _progress(f"{name}: step +{step_deg:.0f}°",
                      joint=joint, index=ji, total=len(targets))

            _refresh_pose()
            start = pose[joint]
            target = step_deg
            pose[joint] = target
            _write_pose(pose)

            peak_load = 0.0
            peak_current = 0.0
            min_volt = 99.0
            last_present = start
            move_t0 = time.monotonic()
            while time.monotonic() - move_t0 < MOVE_TIMEOUT_S:
                if abort_check():
                    break
                fb = _fb(joint)
                now = time.monotonic() - t0
                if fb is None:
                    time.sleep(SAMPLE_DT)
                    continue
                present = float(fb["deg"])
                last_present = present
                load = float(fb.get("load_pct") or 0.0)
                cur = float(fb.get("current_a") or 0.0)
                volt = float(fb.get("volt") or 0.0)
                peak_load = max(peak_load, load)
                peak_current = max(peak_current, cur)
                if volt > 0:
                    min_volt = min(min_volt, volt)
                sample = _sample_row(now, "step+", joint, name, target, fb)
                w.writerow(sample)
                samples.append(sample)
                err = target - present
                if (abs(err) < 1.5
                        and abs(float(fb.get("speed_deg_s") or 0)) < 15.0
                        and (now - (move_t0 - t0)) > 0.25):
                    break
                time.sleep(SAMPLE_DT)

            delta = last_present - start
            tracking = (abs(delta) / step_deg) * 100.0 if step_deg else 0.0
            moved = abs(delta) >= 0.4
            grade = _grade_step(tracking, peak_load, moved)
            rows.append({
                "joint": joint,
                "id": sid,
                "name": name,
                "axis": AXIS[joint % 3],
                "leg": joint // 3,
                "start_deg": round(start, 2),
                "cmd_deg": round(target, 2),
                "end_deg": round(last_present, 2),
                "delta_cmd_deg": round(step_deg, 2),
                "delta_actual_deg": round(delta, 2),
                "tracking_pct": round(tracking, 1),
                "peak_load_pct": round(peak_load, 1),
                "peak_current_a": round(peak_current, 3),
                "min_volt": (None if min_volt >= 90 else round(min_volt, 2)),
                "moved": moved,
                "grade": grade,
            })

            if abort_check():
                break
            pose[joint] = 0.0
            _write_pose(pose)
            ret_t0 = time.monotonic()
            while time.monotonic() - ret_t0 < RETURN_PAUSE_S:
                if abort_check():
                    break
                fb = _fb(joint)
                now = time.monotonic() - t0
                if fb:
                    sample = _sample_row(now, "return", joint, name, 0.0, fb)
                    w.writerow(sample)
                    samples.append(sample)
                time.sleep(SAMPLE_DT)

            d = bus.read_position_deg(joint)
            if d is not None:
                pose[joint] = float(d)

        fh.write("\n# summary joint,name,tracking_pct,delta_actual,peak_load,grade\n")
        for r in rows:
            fh.write(
                f"# {r['joint']},{r['name']},{r['tracking_pct']},"
                f"{r['delta_actual_deg']},{r['peak_load_pct']},{r['grade']}\n"
            )

    n_red = sum(1 for r in rows if r["grade"] == "red")
    n_yellow = sum(1 for r in rows if r["grade"] == "yellow")
    n_green = sum(1 for r in rows if r["grade"] == "green")
    return {
        "ok": True,
        "mode": "step",
        "aborted": abort_check(),
        "step_deg": step_deg,
        "axis": axis or "all",
        "speed": speed,
        "acc": acc,
        "homed": home_ok,
        "home_max_err_deg": home_max_err,
        "log": str(path),
        "log_name": path.name,
        "joints_tested": len(rows),
        "live": live,
        "counts": {"green": n_green, "yellow": n_yellow, "red": n_red},
        "rows": rows,
        "samples": len(samples),
        "hint": (
            "Homes to sit zero first, then +step° from 0°. "
            "red = barely moved (stuck / weak / under-voltage); "
            "yellow = partial travel or high load; "
            "green = tracked the step"
        ),
    }


def run_shake_calibrate(
    bus,
    *,
    nudge_deg: float = DEFAULT_NUDGE_DEG,
    axis: str | None = "all",
    speed: int = SHAKE_SPEED,
    acc: int = SHAKE_ACC,
    names: dict[int, str] | None = None,
    log_path: Path | None = None,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
) -> dict:
    """Tiny nudge + hold: score overshoot and position/load wobble.

    Each joint: home once → +nudge → arrive → hold ~1.6s → grade hunt →
    return to 0°. Hold peak-to-peak and RMS error catch shakiness that a
    big step test misses.
    """
    abort_check = abort_check or (lambda: False)
    nudge_deg = max(0.8, min(5.0, float(nudge_deg)))
    speed = int(max(80, min(1000, speed)))
    acc = int(max(5, min(80, acc)))
    try:
        joints = joints_for_axis(axis)
    except ValueError as e:
        return {"ok": False, "error": str(e), "mode": "shake"}

    path = Path(log_path) if log_path else default_log_path("shake")
    path.parent.mkdir(parents=True, exist_ok=True)

    live, pose = _discover_live(bus)
    targets = [j for j in joints if j in live]
    if not targets:
        return {"ok": False, "error": "no live joints in selection",
                "live": live, "log": str(path), "mode": "shake"}

    rows: list[dict] = []
    samples: list[dict] = []
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

    def _write_pose(p: list[float]) -> None:
        bus.write_all(p, speed=speed, acc=acc)

    def _refresh_pose() -> None:
        for j in live:
            d = bus.read_position_deg(j)
            if d is not None:
                pose[j] = float(d)

    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=[
            "t_s", "phase", "joint", "id", "name",
            "cmd_deg", "present_deg", "err_deg",
            "speed_deg_s", "load_pct", "current_a", "volt",
        ])
        w.writeheader()

        home_ok, home_max_err = False, None
        if not abort_check():
            home_ok, home_max_err = _home_sit_zero(
                bus, live, pose, names=names, abort_check=abort_check,
                on_progress=on_progress, t0=t0, writer=w, samples=samples,
                total=len(targets),
            )

        for ji, joint in enumerate(targets):
            if abort_check():
                _progress("aborted", done=False)
                break
            name = joint_name(joint, names)
            sid = joint_to_servo_id(joint)
            target = nudge_deg
            _progress(f"{name}: nudge +{nudge_deg:.1f}° + hold",
                      joint=joint, index=ji, total=len(targets))

            _refresh_pose()
            start = pose[joint]
            pose[joint] = target
            _write_pose(pose)

            peak_load = 0.0
            peak_current = 0.0
            min_volt = 99.0
            overshoot = 0.0
            last_present = start
            arrive_t0 = time.monotonic()
            while time.monotonic() - arrive_t0 < SHAKE_ARRIVE_S:
                if abort_check():
                    break
                fb = _fb(joint)
                now = time.monotonic() - t0
                if fb is None:
                    time.sleep(SAMPLE_DT)
                    continue
                present = float(fb["deg"])
                last_present = present
                load = float(fb.get("load_pct") or 0.0)
                cur = float(fb.get("current_a") or 0.0)
                volt = float(fb.get("volt") or 0.0)
                peak_load = max(peak_load, load)
                peak_current = max(peak_current, cur)
                if volt > 0:
                    min_volt = min(min_volt, volt)
                # Overshoot past the +nudge target.
                overshoot = max(overshoot, present - target)
                sample = _sample_row(now, "nudge", joint, name, target, fb)
                w.writerow(sample)
                samples.append(sample)
                err = target - present
                if (abs(err) < 0.8
                        and abs(float(fb.get("speed_deg_s") or 0)) < 20.0
                        and (time.monotonic() - arrive_t0) > 0.2):
                    break
                time.sleep(SAMPLE_DT)

            reached = abs(last_present - target) < 1.2

            # Hold: keep commanding target once; measure wobble.
            pose[joint] = target
            _write_pose(pose)
            hold_pos: list[float] = []
            hold_err: list[float] = []
            hold_spd: list[float] = []
            hold_load: list[float] = []
            hold_t0 = time.monotonic()
            while time.monotonic() - hold_t0 < SHAKE_HOLD_S:
                if abort_check():
                    break
                fb = _fb(joint)
                now = time.monotonic() - t0
                if fb is None:
                    time.sleep(SAMPLE_DT)
                    continue
                present = float(fb["deg"])
                last_present = present
                load = float(fb.get("load_pct") or 0.0)
                cur = float(fb.get("current_a") or 0.0)
                volt = float(fb.get("volt") or 0.0)
                spd = float(fb.get("speed_deg_s") or 0.0)
                peak_load = max(peak_load, load)
                peak_current = max(peak_current, cur)
                if volt > 0:
                    min_volt = min(min_volt, volt)
                sample = _sample_row(now, "hold", joint, name, target, fb)
                w.writerow(sample)
                samples.append(sample)
                if (time.monotonic() - hold_t0) >= SHAKE_HOLD_IGNORE_S:
                    hold_pos.append(present)
                    hold_err.append(present - target)
                    hold_spd.append(spd)
                    hold_load.append(load)
                time.sleep(SAMPLE_DT)

            if hold_pos:
                pos_pp = max(hold_pos) - min(hold_pos)
                rms_err = math.sqrt(
                    sum(e * e for e in hold_err) / len(hold_err))
                speed_rms = math.sqrt(
                    sum(s * s for s in hold_spd) / len(hold_spd))
                load_pp = max(hold_load) - min(hold_load)
            else:
                pos_pp = rms_err = speed_rms = load_pp = 0.0

            grade = _grade_shake(pos_pp, rms_err, speed_rms, reached)
            rows.append({
                "joint": joint,
                "id": sid,
                "name": name,
                "axis": AXIS[joint % 3],
                "leg": joint // 3,
                "start_deg": round(start, 2),
                "cmd_deg": round(target, 2),
                "end_deg": round(last_present, 2),
                "nudge_deg": round(nudge_deg, 2),
                "delta_cmd_deg": round(nudge_deg, 2),
                "delta_actual_deg": round(last_present - start, 2),
                "overshoot_deg": round(max(0.0, overshoot), 2),
                "hold_pp_deg": round(pos_pp, 2),
                "hold_rms_err_deg": round(rms_err, 2),
                "hold_speed_rms": round(speed_rms, 1),
                "hold_load_pp": round(load_pp, 1),
                "peak_load_pct": round(peak_load, 1),
                "peak_current_a": round(peak_current, 3),
                "min_volt": (None if min_volt >= 90 else round(min_volt, 2)),
                "reached": reached,
                "grade": grade,
            })

            if abort_check():
                break
            pose[joint] = 0.0
            _write_pose(pose)
            ret_t0 = time.monotonic()
            while time.monotonic() - ret_t0 < RETURN_PAUSE_S:
                if abort_check():
                    break
                fb = _fb(joint)
                now = time.monotonic() - t0
                if fb:
                    sample = _sample_row(now, "return", joint, name, 0.0, fb)
                    w.writerow(sample)
                    samples.append(sample)
                time.sleep(SAMPLE_DT)

            d = bus.read_position_deg(joint)
            if d is not None:
                pose[joint] = float(d)

        fh.write(
            "\n# summary joint,name,hold_pp,rms_err,overshoot,load_pp,grade\n"
        )
        for r in rows:
            fh.write(
                f"# {r['joint']},{r['name']},{r['hold_pp_deg']},"
                f"{r['hold_rms_err_deg']},{r['overshoot_deg']},"
                f"{r['hold_load_pp']},{r['grade']}\n"
            )

    n_red = sum(1 for r in rows if r["grade"] == "red")
    n_yellow = sum(1 for r in rows if r["grade"] == "yellow")
    n_green = sum(1 for r in rows if r["grade"] == "green")
    return {
        "ok": True,
        "mode": "shake",
        "aborted": abort_check(),
        "nudge_deg": nudge_deg,
        "step_deg": nudge_deg,  # UI reuse
        "axis": axis or "all",
        "speed": speed,
        "acc": acc,
        "homed": home_ok,
        "home_max_err_deg": home_max_err,
        "log": str(path),
        "log_name": path.name,
        "joints_tested": len(rows),
        "live": live,
        "counts": {"green": n_green, "yellow": n_yellow, "red": n_red},
        "rows": rows,
        "samples": len(samples),
        "hint": (
            "Homes to sit zero, nudges +N°, then holds and measures wobble. "
            "red = big hunt / never settled; yellow = mild shake; "
            "green = quiet hold"
        ),
    }
