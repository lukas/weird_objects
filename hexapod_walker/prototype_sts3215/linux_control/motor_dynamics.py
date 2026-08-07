"""Air-only motor sys-ID battery (no stand / no plant).

Trimmed battery (2026-08-07): start from sit zero, legs straight out on
the bench. One joint moves at a time, everything else just holds.

- **Full battery** on one representative joint per axis (default L2 yaw /
  hip / knee): steps at ±small (3°) and ±large (amp_deg) × ``repeats``,
  then a slow ±6° triangle ramp for deadband / hysteresis.
- **Verify pass** on every other live joint: a single ±large step pair,
  checked against its axis model.

Everything measured maps to a MuJoCo knob: delay→action latency,
rise/settle→kp/kv fit, peak speed→velocity ceiling, ramp→frictionloss/
deadband, ±step asymmetry→gravity/mass sanity check.

Logs a CSV and writes ``logs/motor_model.json``. Soft torque, current
trip, temp trip, single-joint writes only.

HTTP: ``POST /api/rl/probe_dynamics``.
"""
from __future__ import annotations

import csv
import json
import statistics
import time
from pathlib import Path
from typing import Callable

from feetech_bus import HOLD_SPEED, N_JOINTS, joint_to_servo_id

LOG_DIR = Path(__file__).resolve().parent / "logs"
AXIS = ("yaw", "hip", "knee")

DEFAULT_AMP_DEG = 12.0       # "large" step; verify joints use only this
SMALL_AMP_DEG = 3.0          # deadband-regime step (full-battery joints)
MAX_AMP_DEG = 15.0
DEFAULT_REPEATS = 2          # step repeats on full-battery joints
FULL_LEG = 2                 # leg whose yaw/hip/knee get the full battery
RAMP_AMP_DEG = 6.0           # triangle ramp ±amp (full-battery joints)
RAMP_RATE_DEG_S = 2.0        # slow: separates deadband from tracking lag
SAMPLE_DT = 0.04
MOVE_TIMEOUT_S = 2.2
RETURN_PAUSE_S = 0.7
SOFT_TORQUE = 350
SPEED = 350
ACC = 15
MAX_CURRENT_A = 0.9          # abort joint if peak exceeds (air)
MAX_TEMP_C = 55.0
SETTLE_ERR_DEG = 1.5
SETTLE_SPEED = 20.0


def joint_name(joint: int, names: dict[int, str] | None = None) -> str:
    sid = joint_to_servo_id(joint)
    if names and sid in names:
        return names[sid]
    leg, axis = divmod(joint, 3)
    return f"L{leg} {AXIS[axis]}"


def _fit_step(samples: list[dict], *, amp: float, t_cmd: float) -> dict:
    """Fit simple step-response metrics from rows with t_s, present, cmd, ..."""
    if not samples:
        return {"ok": False, "error": "no samples"}
    amp = float(amp)
    sign = 1.0 if amp >= 0 else -1.0
    a = abs(amp)
    p0 = float(samples[0]["present_deg"])
    # Use cmd target from last sample
    target = float(samples[-1].get("cmd_deg", p0 + amp))

    delay_ms = None
    t10 = t90 = None
    peak = p0
    peak_i = 0.0
    peak_l = 0.0
    peak_spd = 0.0
    min_v = 99.0

    # Speed derived from position deltas rather than the speed register.
    # Until 2026-08-07 the register was decoded with the wrong unit
    # (0.732 rpm/count instead of counts/s → 50× inflated; poisoned
    # settle_ms / vel_max in earlier aggregates). The decode is fixed in
    # the bus layers now, but position-derived speed stays: it is
    # self-consistent for old CSVs and immune to any future decode drift.
    prev_t = prev_p = None
    for s in samples:
        t = float(s["t_s"]) - t_cmd
        p = float(s["present_deg"])
        if prev_t is not None and t > prev_t + 1e-6:
            spd = abs(p - prev_p) / (t - prev_t)
        else:
            spd = 0.0
        s["_spd_deg_s"] = spd
        prev_t, prev_p = t, p
        peak_i = max(peak_i, abs(float(s.get("current_a") or 0)))
        peak_l = max(peak_l, abs(float(s.get("load_pct") or 0)))
        peak_spd = max(peak_spd, spd)
        v = float(s.get("volt") or 0)
        if v > 0:
            min_v = min(min_v, v)
        moved = (p - p0) * sign
        if delay_ms is None and moved >= 0.1 * a:
            delay_ms = max(0.0, t * 1000.0)
        if t10 is None and moved >= 0.1 * a:
            t10 = t
        if t90 is None and moved >= 0.9 * a:
            t90 = t
        if sign > 0:
            peak = max(peak, p)
        else:
            peak = min(peak, p)

    rise_ms = None
    if t10 is not None and t90 is not None and t90 >= t10:
        rise_ms = (t90 - t10) * 1000.0

    # Settle: first time |err|<band and slow, after 0.15s (position-derived
    # speed — see above).
    settle_ms = None
    for s in samples:
        t = float(s["t_s"]) - t_cmd
        if t < 0.15:
            continue
        err = abs(float(s["present_deg"]) - target)
        spd = float(s.get("_spd_deg_s") or 0)
        if err <= SETTLE_ERR_DEG and spd <= SETTLE_SPEED:
            settle_ms = t * 1000.0
            break

    end = float(samples[-1]["present_deg"])
    actual = end - p0
    tracking = (abs(actual) / a) * 100.0 if a > 1e-6 else 0.0
    overshoot = max(0.0, (peak - p0) * sign - a)

    return {
        "ok": tracking >= 35.0,
        "amp_deg": round(amp, 2),
        "start_deg": round(p0, 3),
        "end_deg": round(end, 3),
        "actual_delta_deg": round(actual, 3),
        "tracking_pct": round(tracking, 1),
        "delay_ms": None if delay_ms is None else round(delay_ms, 1),
        "rise_ms": None if rise_ms is None else round(rise_ms, 1),
        "settle_ms": None if settle_ms is None else round(settle_ms, 1),
        "overshoot_deg": round(overshoot, 2),
        "ss_err_deg": round(end - target, 3),
        "peak_current_a": round(peak_i, 3),
        "peak_load_pct": round(peak_l, 1),
        "peak_speed_deg_s": round(peak_spd, 1),
        "min_volt": None if min_v >= 90 else round(min_v, 2),
    }


def _fit_ramp(samples: list[dict]) -> dict:
    """Deadband / hysteresis from a slow triangle ramp.

    Rows need t_s, cmd_deg, present_deg. At ~2 °/s tracking lag is small,
    so the mean (cmd − present) error split by command direction is
    dominated by deadband + friction; half the fwd/rev gap ≈ deadband.
    """
    if len(samples) < 8:
        return {"ok": False, "error": "too few ramp samples"}
    fwd_err: list[float] = []
    rev_err: list[float] = []
    rmse_acc = 0.0
    n = 0
    prev_cmd = float(samples[0]["cmd_deg"])
    for s in samples[1:]:
        cmd = float(s["cmd_deg"])
        present = float(s["present_deg"])
        dcmd = cmd - prev_cmd
        prev_cmd = cmd
        err = cmd - present
        rmse_acc += err * err
        n += 1
        if dcmd > 1e-6:
            fwd_err.append(err)
        elif dcmd < -1e-6:
            rev_err.append(err)
    if not fwd_err or not rev_err:
        return {"ok": False, "error": "ramp missing a direction"}
    mean_fwd = statistics.mean(fwd_err)
    mean_rev = statistics.mean(rev_err)
    hysteresis = mean_fwd - mean_rev
    return {
        "ok": True,
        "mean_err_fwd_deg": round(mean_fwd, 3),
        "mean_err_rev_deg": round(mean_rev, 3),
        "hysteresis_deg": round(hysteresis, 3),
        "deadband_deg": round(abs(hysteresis) / 2.0, 3),
        "track_rmse_deg": round((rmse_acc / max(1, n)) ** 0.5, 3),
    }


def run_motor_dynamics(
    bus,
    *,
    amp_deg: float = DEFAULT_AMP_DEG,
    axis: str | None = "all",
    joints: list[int] | None = None,
    full_joints: list[int] | None = None,
    repeats: int = DEFAULT_REPEATS,
    soft_torque: int = SOFT_TORQUE,
    names: dict[int, str] | None = None,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
    log_dir: Path | None = None,
) -> dict:
    """Sys-ID battery in air from the current pose. No stand.

    ``amp_deg`` is the LARGE step; full-battery joints also get ±3° steps
    and a slow ramp. ``full_joints`` overrides which joints get the full
    battery (default: FULL_LEG's yaw/hip/knee, if live).
    """
    abort_check = abort_check or (lambda: False)
    amp_deg = max(6.0, min(MAX_AMP_DEG, float(amp_deg)))
    repeats = int(max(1, min(4, repeats)))
    soft_torque = int(max(200, min(800, soft_torque)))
    log_dir = Path(log_dir) if log_dir else LOG_DIR
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = log_dir / f"motor_dyn_{stamp}.csv"
    model_path = log_dir / "motor_model.json"

    try:
        from inplace_demos import (
            _enable_torque, _live_robot_ids, _limp_all,
            _set_torque_limit, _write_pose,
        )
    except ImportError as e:
        return {"ok": False, "error": f"inplace_demos missing: {e}",
                "mode": "dynamics"}

    live_ids = _live_robot_ids(bus)
    live_joints = sorted(
        j for j in range(N_JOINTS) if joint_to_servo_id(j) in live_ids
    )
    if joints is not None:
        want = set(int(j) for j in joints)
        targets = [j for j in live_joints if j in want]
    elif axis and axis not in ("all", "*"):
        key = axis.strip().lower()
        if key not in AXIS:
            return {"ok": False, "error": f"bad axis {axis!r}"}
        ai = AXIS.index(key)
        targets = [j for j in live_joints if j % 3 == ai]
    else:
        targets = list(live_joints)

    if not targets:
        return {"ok": False, "error": "no live joints to probe",
                "mode": "dynamics", "live_ids": sorted(live_ids)}

    # Which joints get the full battery (per-axis representatives).
    if full_joints is not None:
        full_set = {int(j) for j in full_joints if int(j) in targets}
    else:
        full_set = set()
        for ai in range(3):
            axis_targets = [j for j in targets if j % 3 == ai]
            if not axis_targets:
                continue
            preferred = FULL_LEG * 3 + ai
            full_set.add(preferred if preferred in axis_targets
                         else axis_targets[0])

    def _progress(msg: str, **extra):
        if on_progress:
            try:
                on_progress({"msg": msg, "mode": "dynamics", **extra})
            except Exception:
                pass

    def _fb(j: int) -> dict | None:
        try:
            return bus.read_feedback(j)
        except Exception:
            return None

    def _read_pose() -> list[float]:
        pose = [0.0] * N_JOINTS
        for j in live_joints:
            d = bus.read_position_deg(j)
            if d is not None:
                pose[j] = float(d)
        return pose

    def _write_one(joint: int, deg: float, *, speed: int = SPEED,
                   acc: int = ACC) -> None:
        """Command ONLY this joint — never SyncWrite the whole body."""
        if hasattr(bus, "write_joint"):
            bus.write_joint(joint, float(deg), speed=speed, acc=acc)
            return
        sid = joint_to_servo_id(joint)
        pose = _read_pose()
        pose[joint] = float(deg)
        _write_pose(bus, pose, {sid}, speed=speed, acc=acc)

    t0 = time.monotonic()
    _progress(f"soft torque {soft_torque}, HOLD current pose "
              f"(single-joint writes only), "
              f"{len(targets)} joints ±{amp_deg:.0f}°")
    _set_torque_limit(bus, live_ids, soft_torque)
    _enable_torque(bus, live_ids)
    if abort_check():
        _limp_all(bus, live_ids)
        return {"ok": False, "aborted": True, "mode": "dynamics"}

    # One SyncWrite hold from successful reads only — never invent 0° for
    # a failed read (that yanked stilts / browned the board before).
    base_pose = [0.0] * N_JOINTS
    hold_ids: set[int] = set()
    for j in live_joints:
        d = bus.read_position_deg(j)
        if d is None:
            continue
        base_pose[j] = float(d)
        hold_ids.add(joint_to_servo_id(j))
    if hold_ids:
        _write_pose(bus, base_pose, hold_ids, speed=HOLD_SPEED, acc=25)
    time.sleep(0.3)

    summaries: list[dict] = []
    aborted = False
    overcurrent = False

    with csv_path.open("w", newline="") as fh:
        fields = [
            "t_s", "phase", "joint", "id", "name",
            "cmd_deg", "present_deg", "err_deg",
            "speed_deg_s", "load_pct", "current_a", "volt", "temp_c",
        ]
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()

        def _log(phase: str, joint: int, cmd: float, fb: dict | None):
            now = time.monotonic() - t0
            name = joint_name(joint, names)
            sid = joint_to_servo_id(joint)
            if fb is None:
                row = {
                    "t_s": f"{now:.3f}", "phase": phase, "joint": joint,
                    "id": sid, "name": name, "cmd_deg": f"{cmd:.3f}",
                    "present_deg": "", "err_deg": "", "speed_deg_s": "",
                    "load_pct": "", "current_a": "", "volt": "", "temp_c": "",
                }
            else:
                present = float(fb["deg"])
                row = {
                    "t_s": f"{now:.3f}", "phase": phase, "joint": joint,
                    "id": sid, "name": name,
                    "cmd_deg": f"{cmd:.3f}",
                    "present_deg": f"{present:.3f}",
                    "err_deg": f"{cmd - present:.3f}",
                    "speed_deg_s": f"{float(fb.get('speed_deg_s') or 0):.2f}",
                    "load_pct": f"{float(fb.get('load_pct') or 0):.1f}",
                    "current_a": f"{abs(float(fb.get('current_a') or 0)):.3f}",
                    "volt": f"{float(fb.get('volt') or 0):.2f}",
                    "temp_c": f"{float(fb.get('temp_c') or 0):.1f}",
                }
            w.writerow(row)
            return row

        def _run_step(joint: int, amp: float, phase: str) -> tuple[dict, bool]:
            nonlocal overcurrent
            d0 = bus.read_position_deg(joint)
            start = float(d0) if d0 is not None else 0.0
            target = start + amp
            t_cmd = time.monotonic() - t0
            _write_one(joint, target)
            buf: list[dict] = []
            trip = False
            deadline = time.monotonic() + MOVE_TIMEOUT_S
            while time.monotonic() < deadline:
                if abort_check():
                    return {"ok": False, "aborted": True}, True
                fb = _fb(joint)
                row = _log(phase, joint, target, fb)
                if fb:
                    buf.append({
                        "t_s": float(row["t_s"]),
                        "present_deg": float(row["present_deg"]),
                        "cmd_deg": target,
                        "speed_deg_s": float(row["speed_deg_s"] or 0),
                        "current_a": float(row["current_a"] or 0),
                        "load_pct": float(row["load_pct"] or 0),
                        "volt": float(row["volt"] or 0),
                    })
                    if float(row["current_a"] or 0) > MAX_CURRENT_A:
                        overcurrent = True
                        trip = True
                        _progress(f"{joint_name(joint, names)}: overcurrent "
                                  f"{row['current_a']}A — abort joint")
                        break
                    temp = float(fb.get("temp_c") or 0)
                    if temp >= MAX_TEMP_C:
                        trip = True
                        _progress(f"{joint_name(joint, names)}: hot "
                                  f"{temp:.0f}°C — abort joint")
                        break
                    err = abs(float(row["present_deg"]) - target)
                    spd = abs(float(row["speed_deg_s"] or 0))
                    if (err <= SETTLE_ERR_DEG and spd <= SETTLE_SPEED
                            and (time.monotonic() - t0 - t_cmd) > 0.2):
                        break
                time.sleep(SAMPLE_DT)
            fit = _fit_step(buf, amp=amp, t_cmd=t_cmd)
            fit["phase"] = phase
            fit["tripped"] = trip
            return fit, abort_check()

        def _return_home(joint: int, home: float):
            _write_one(joint, home)
            t_end = time.monotonic() + RETURN_PAUSE_S
            while time.monotonic() < t_end:
                if abort_check():
                    return True
                fb = _fb(joint)
                _log("return", joint, home, fb)
                time.sleep(SAMPLE_DT)
            return abort_check()

        def _run_ramp(joint: int, home: float) -> tuple[dict, bool]:
            """Slow triangle home → +amp → −amp → home. Streams targets."""
            nonlocal overcurrent
            buf: list[dict] = []
            legs = [(home, home + RAMP_AMP_DEG),
                    (home + RAMP_AMP_DEG, home - RAMP_AMP_DEG),
                    (home - RAMP_AMP_DEG, home)]
            for a, b in legs:
                dur = abs(b - a) / RAMP_RATE_DEG_S
                t_leg = time.monotonic()
                while True:
                    if abort_check():
                        return {"ok": False, "aborted": True}, True
                    frac = min(1.0, (time.monotonic() - t_leg) / dur)
                    cmd = a + (b - a) * frac
                    _write_one(joint, cmd)
                    fb = _fb(joint)
                    row = _log("ramp", joint, cmd, fb)
                    if fb:
                        buf.append({
                            "t_s": float(row["t_s"]),
                            "cmd_deg": cmd,
                            "present_deg": float(row["present_deg"]),
                        })
                        if float(row["current_a"] or 0) > MAX_CURRENT_A:
                            overcurrent = True
                            return {"ok": False, "error": "overcurrent"}, True
                        if float(fb.get("temp_c") or 0) >= MAX_TEMP_C:
                            return {"ok": False, "error": "hot"}, True
                    if frac >= 1.0:
                        break
                    time.sleep(SAMPLE_DT)
            return _fit_ramp(buf), abort_check()

        for ji, joint in enumerate(targets):
            if abort_check():
                aborted = True
                break
            name = joint_name(joint, names)
            full = joint in full_set
            home = _read_pose()[joint]

            # Step plan: verify joints get one large ± pair; full-battery
            # joints get (small ± , large ±) × repeats, then the ramp.
            if full:
                plan = [(a, r) for r in range(repeats)
                        for a in (+SMALL_AMP_DEG, -SMALL_AMP_DEG,
                                  +amp_deg, -amp_deg)]
            else:
                plan = [(+amp_deg, 0), (-amp_deg, 0)]
            _progress(
                f"{name}: {'FULL battery' if full else 'verify'} "
                f"({len(plan)} steps{' + ramp' if full else ''})",
                joint=joint, index=ji, total=len(targets))

            summary = {
                "joint": joint, "id": joint_to_servo_id(joint),
                "name": name, "axis": AXIS[joint % 3],
                "leg": joint // 3, "full": full,
                "steps": [], "ramp": None,
                "plus": None, "minus": None,
            }
            summaries.append(summary)

            tripped = False
            for amp, rep in plan:
                sign = "+" if amp > 0 else "-"
                fit, ab = _run_step(
                    joint, amp, f"step{sign}{abs(amp):g}r{rep}")
                fit["rep"] = rep
                summary["steps"].append(fit)
                # Back-compat plus/minus = first large-amp pair.
                if abs(abs(amp) - amp_deg) < 1e-6:
                    if amp > 0 and summary["plus"] is None:
                        summary["plus"] = fit
                    elif amp < 0 and summary["minus"] is None:
                        summary["minus"] = fit
                ret_ab = _return_home(joint, home)
                if fit.get("tripped"):
                    tripped = True
                if ab or ret_ab or overcurrent:
                    aborted = ab or ret_ab or aborted
                    break
                if tripped:
                    break
            if aborted or overcurrent:
                break

            if full and not tripped:
                ramp_fit, ab = _run_ramp(joint, home)
                summary["ramp"] = ramp_fit
                _return_home(joint, home)
                if ab or overcurrent:
                    aborted = ab or aborted
                    break

    # Limp immediately — do not re-hold the whole body after probing.
    try:
        _set_torque_limit(bus, live_ids, 1000)
    except Exception:
        pass
    try:
        _limp_all(bus, live_ids)
    except Exception:
        pass

    # Aggregate model
    def _mean(vals: list, nd: int = 1):
        vals = [v for v in vals if v is not None]
        return None if not vals else round(statistics.mean(vals), nd)

    model_joints = []
    for s in summaries:
        steps = [f for f in s.get("steps", []) if f]
        good = [f for f in steps if f.get("ok") and not f.get("tripped")]
        large = [f for f in good
                 if abs(abs(f.get("amp_deg") or 0) - amp_deg) < 0.5]
        plus_l = [f for f in large if (f.get("amp_deg") or 0) > 0]
        minus_l = [f for f in large if (f.get("amp_deg") or 0) < 0]
        tripped = any(f.get("tripped") for f in steps)
        ok = bool(good) and not tripped
        # Gravity / mass hint: current asymmetry between ± large steps
        # (leg weight helps one direction, fights the other).
        asym = None
        if plus_l and minus_l:
            ip = max(f.get("peak_current_a") or 0 for f in plus_l)
            im = max(f.get("peak_current_a") or 0 for f in minus_l)
            asym = round(ip - im, 3)
        ramp = s.get("ramp") or None
        model_joints.append({
            "joint": s["joint"],
            "id": s["id"],
            "name": s["name"],
            "axis": s["axis"],
            "leg": s["leg"],
            "full": s.get("full", False),
            "ok": ok,
            "delay_ms": _mean([f.get("delay_ms") for f in good]),
            "rise_ms": _mean([f.get("rise_ms") for f in good]),
            "settle_ms": _mean([f.get("settle_ms") for f in good]),
            "tracking_pct": _mean([f.get("tracking_pct") for f in good]),
            "overshoot_deg": _mean([f.get("overshoot_deg") for f in good], 2),
            "vel_max_deg_s": (None if not large else round(
                max(f.get("peak_speed_deg_s") or 0 for f in large), 1)),
            "peak_current_a": (None if not good else round(
                max(f.get("peak_current_a") or 0 for f in good), 3)),
            "current_asym_a": asym,
            "deadband_deg": (ramp or {}).get("deadband_deg"),
            "hysteresis_deg": (ramp or {}).get("hysteresis_deg"),
            "steps": steps,
            "ramp": ramp,
            # Back-compat fields
            "plus": s.get("plus"),
            "minus": s.get("minus"),
        })

    # Per-axis model from the full-battery joints; verify joints are
    # checked against it (delay/rise within 50%).
    axes: dict[str, dict] = {}
    for ax in AXIS:
        fulls = [j for j in model_joints
                 if j["axis"] == ax and j["full"] and j["ok"]]
        if not fulls:
            continue
        axes[ax] = {
            "delay_ms": _mean([j["delay_ms"] for j in fulls]),
            "rise_ms": _mean([j["rise_ms"] for j in fulls]),
            "settle_ms": _mean([j["settle_ms"] for j in fulls]),
            "vel_max_deg_s": _mean([j["vel_max_deg_s"] for j in fulls]),
            "deadband_deg": _mean([j["deadband_deg"] for j in fulls], 3),
            "hysteresis_deg": _mean([j["hysteresis_deg"] for j in fulls], 3),
            "from_joints": [j["joint"] for j in fulls],
        }
    for j in model_joints:
        ref = axes.get(j["axis"])
        if not ref or j["full"] or not j["ok"]:
            continue
        flags = []
        for key in ("delay_ms", "rise_ms"):
            a, b = j.get(key), ref.get(key)
            if a is not None and b and abs(a - b) > 0.5 * b:
                flags.append(f"{key} {a} vs axis {b}")
        j["matches_axis_model"] = not flags
        if flags:
            j["deviation"] = "; ".join(flags)

    ok_n = sum(1 for j in model_joints if j["ok"])
    deviants = [j["name"] for j in model_joints
                if j.get("matches_axis_model") is False]
    model = {
        "ok": (not aborted) and ok_n >= max(1, len(targets) // 2),
        "mode": "dynamics",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "amp_deg": amp_deg,
        "small_amp_deg": SMALL_AMP_DEG,
        "repeats": repeats,
        "ramp_amp_deg": RAMP_AMP_DEG,
        "ramp_rate_deg_s": RAMP_RATE_DEG_S,
        "full_joints": sorted(full_set),
        "soft_torque": soft_torque,
        "speed": SPEED,
        "acc": ACC,
        "sample_dt_s": SAMPLE_DT,
        "csv": str(csv_path),
        "aborted": aborted,
        "overcurrent": overcurrent,
        "joints_tested": len(model_joints),
        "joints_ok": ok_n,
        "axis_model": axes,
        "deviants": deviants,
        "joints": model_joints,
        "msg": (f"dynamics {ok_n}/{len(model_joints)} ok · "
                f"full={sorted(full_set)} ±{amp_deg:.0f}°/"
                f"±{SMALL_AMP_DEG:.0f}° air"
                + (f" · deviants: {', '.join(deviants)}" if deviants
                   else "") + " · limp"),
    }
    model_path.write_text(json.dumps(model, indent=2))
    model["path"] = str(model_path)
    _progress(model["msg"])
    return model
