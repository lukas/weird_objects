"""On-robot sysid protocol runner: deterministic command streams + logging.

Executes a sysid protocol (see ``sysid_protocol.py``) at the protocol's
tick rate, streaming position targets exactly like the deployed RL
runner (a new absolute target every tick, fixed write speed/acc), while
recording synchronized telemetry with per-tick send/receive timestamps
so end-to-end latency DISTRIBUTIONS can be estimated offline (sysid
plan, Phases 1-2).

Safety (same posture as motor_dynamics.py — air battery rules):

- Soft torque limit for the whole run; limp immediately on any trip.
- The runner POSITIONS THE LEGS ITSELF: before the first segment it
  glides slowly (``GLIDE_RATE_DEG_S``) to the protocol's start pose
  (``home_deg`` / a leading traj's first row) — no hand-posing. The
  glide streams eased targets with every trip active, and the run
  aborts if the pose does not verify within ``GLIDE_TOL_DEG`` after.
- ``step``/``sine`` segments move ONE joint; all others just hold the
  pose captured at segment start (successful reads only — never
  invent 0°).
- ``traj`` segments (whole-body champion replay, Phase 8) require
  ``force=True`` from the API layer; a traj after other segments must
  be continuous with the present pose (``start_tol_deg``) or the run
  trips — wrong-zero-frame / bad-protocol protection.
- Trips: per-joint current, temperature, tracking error blow-up
  (unexpected force / wrong logical zero -> limp + descriptive error),
  missing servo reads.

Output: ``logs/sysid_<name>_<stamp>.csv`` + ``..._summary.json`` with
the full protocol embedded (raw data is never overwritten).

HTTP: ``POST /api/sysid/run`` (bench_api.sysid_run).
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Callable

from feetech_bus import HOLD_SPEED, N_JOINTS, joint_to_servo_id

try:
    from feetech_bus import AXIS_LIMITS_DEG as _BUS_LIMITS
    AXIS_LIMITS = {"yaw": tuple(_BUS_LIMITS["yaw"]),
                   "hip": tuple(_BUS_LIMITS["hip"]),
                   "knee": tuple(_BUS_LIMITS["knee"])}
except Exception:  # pragma: no cover - old bus module
    from sysid_protocol import AXIS_LIMITS_DEG as AXIS_LIMITS

from sysid_protocol import (
    DEFAULT_MAX_CURRENT_A, DEFAULT_SOFT_TORQUE, DEFAULT_WRITE_ACC,
    DEFAULT_WRITE_SPEED, axis_of, materialize, protocol_hash, start_pose,
    validate,
)

LOG_DIR = Path(__file__).resolve().parent / "logs"

MAX_TEMP_C = 55.0
# The MCU bridge occasionally corrupts a temp byte (observed 34->42/50/66,
# single-bit flips of 0x22). Real overheating persists for minutes, so a
# trip needs this many CONSECUTIVE fresh feedback polls at/over the limit
# (3 polls @ 10 Hz = 0.3 s) — a lone glitched read must never limp a run.
TEMP_TRIP_POLLS = 3
# Tracking trip compares present against a reference that slews toward
# the command at the commanded profile speed — NOT the raw target: a step
# larger than the limit (e.g. the ±40 deg ladder rungs) would trip on its
# very first tick before the servo has any chance to move.
MAX_TRACK_ERR_DEG = 30.0       # active-joint |slew_ref - present| trip
MAX_MISSED_READS = 5           # consecutive bulk-read misses of a joint
FEEDBACK_HZ = 10.0             # full-feedback (current/temp) throttle
DEFAULT_START_TOL_DEG = 12.0   # traj continuity gate (mid-protocol)
GLIDE_RATE_DEG_S = 12.0        # slow start-pose glide (air)
GLIDE_TIMEOUT_S = 45.0
GLIDE_SETTLE_S = 1.0
GLIDE_TOL_DEG = 3.0            # post-glide worst-joint verification


def _fmt(v, nd=3):
    return "" if v is None else f"{float(v):.{nd}f}"


def run_sysid_protocol(
    bus,
    protocol: dict,
    *,
    force: bool = False,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
    log_dir: Path | None = None,
) -> dict:
    """Execute a sysid protocol. Returns a result/summary dict."""
    abort_check = abort_check or (lambda: False)
    errs = validate(protocol)
    if errs:
        return {"ok": False, "mode": "sysid",
                "error": "invalid protocol: " + "; ".join(errs)}
    has_traj = any(s.get("kind") == "traj"
                   for s in protocol.get("segments", []))
    if has_traj and not force:
        return {"ok": False, "mode": "sysid",
                "error": "traj (whole-body) segments require force=true "
                         "and an operator watching"}

    try:
        from inplace_demos import (
            _enable_torque, _live_robot_ids, _limp_all,
            _set_torque_limit, _write_pose,
        )
    except ImportError as e:
        return {"ok": False, "mode": "sysid",
                "error": f"inplace_demos missing: {e}"}

    mat = materialize(protocol)
    hz = mat["hz"]
    dt = 1.0 / hz
    ticks = mat["ticks"]
    name = str(protocol.get("name", "unnamed"))
    phash = protocol_hash(protocol)
    write_speed = int(protocol.get("write_speed", DEFAULT_WRITE_SPEED))
    write_acc = int(protocol.get("write_acc", DEFAULT_WRITE_ACC))
    soft_torque = int(max(200, min(800, protocol.get(
        "soft_torque", DEFAULT_SOFT_TORQUE))))
    max_cur = float(protocol.get("max_current_a", DEFAULT_MAX_CURRENT_A))

    def _progress(msg: str, **extra):
        if on_progress:
            try:
                on_progress({"msg": msg, "mode": "sysid", **extra})
            except Exception:
                pass

    live_ids = _live_robot_ids(bus)
    live_joints = sorted(j for j in range(N_JOINTS)
                         if joint_to_servo_id(j) in live_ids)
    needed = sorted({j for t in ticks for j in t["active"]})
    missing = [j for j in needed if j not in live_joints]
    if missing:
        return {"ok": False, "mode": "sysid",
                "error": f"protocol needs dead joints {missing} "
                         f"(live: {live_joints})"}

    log_dir = Path(log_dir) if log_dir else LOG_DIR
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = log_dir / f"sysid_{name}_{stamp}.csv"
    sum_path = log_dir / f"sysid_{name}_{stamp}_summary.json"

    def _read_pose() -> tuple[dict[int, float], list[int]]:
        pos = bus.read_all_positions()
        if not isinstance(pos, dict):
            pos = {}
        return pos, [j for j in live_joints if j not in pos]

    pose0, miss0 = _read_pose()
    if miss0:
        return {"ok": False, "mode": "sysid",
                "error": f"servo IDs not answering: joints {miss0}"}
    # Start pose the runner will glide to by itself (no hand-posing):
    # protocol home_deg, or a leading traj's first row.
    glide_target = start_pose(protocol)

    _progress(f"sysid '{name}' ({phash}): {len(ticks)} ticks @ {hz:g} Hz, "
              f"soft torque {soft_torque}")
    _set_torque_limit(bus, live_ids, soft_torque)
    _enable_torque(bus, live_ids)

    # Hold pose from successful reads only (motor_dynamics pattern).
    base_pose = [0.0] * N_JOINTS
    hold_ids: set[int] = set()
    for j, d in pose0.items():
        base_pose[j] = float(d)
        hold_ids.add(joint_to_servo_id(j))
    if hold_ids:
        _write_pose(bus, base_pose, hold_ids, speed=HOLD_SPEED, acc=25)
    time.sleep(0.3)

    def _bail(error: str, extra: dict | None = None) -> dict:
        try:
            _set_torque_limit(bus, live_ids, 1000)
        except Exception:
            pass
        try:
            _limp_all(bus, live_ids)
        except Exception:
            pass
        res = {"ok": False, "mode": "sysid", "error": error,
               "csv": str(csv_path), **(extra or {})}
        _progress(f"TRIP: {error}")
        return res

    def _write_active(tick: dict, cmd_abs: list[float]) -> None:
        if len(tick["active"]) == 1 and hasattr(bus, "write_joint"):
            j = tick["active"][0]
            bus.write_joint(j, cmd_abs[j], speed=write_speed, acc=write_acc)
        else:
            bus.write_all(cmd_abs, speed=write_speed, acc=write_acc)

    seg_home: dict[int, list[float]] = {}
    seg_stats: list[dict] = []
    cur_seg = -1
    t_run0: float | None = None
    overruns = 0
    clamped = 0
    missed: dict[int, int] = {j: 0 for j in live_joints}
    tripped_error: str | None = None
    aborted = False
    last_fb: dict[int, dict] = {}
    last_fb_t = -1e9
    hot_polls: dict[int, int] = {}   # consecutive fresh polls >= MAX_TEMP_C
    last_pose = dict(pose0)
    # Slew-limited tracking-trip reference (deg per tick at the commanded
    # Feetech profile speed; 400 counts/s ~= 35 deg/s).
    trip_ref: dict[int, float] = {}
    trip_slew_deg = write_speed * (360.0 / 4096.0) * dt

    fields = (["t_s", "tick", "seg", "phase", "joint",
               "t_send_s", "t_recv_s", "overrun"]
              + [f"q{j}_deg" for j in range(N_JOINTS)]
              + [f"cmd{j}_deg" for j in range(N_JOINTS)]
              + ["cur_a", "load_pct", "volt", "temp_c"]
              # Full per-joint currents from the throttled 10 Hz bus poll
              # (values repeat between polls): the loaded/contact phases
              # need every servo's draw, not one summary sample.
              + [f"cur{j}_a" for j in range(N_JOINTS)])

    t0 = time.monotonic()
    started_iso = time.strftime("%Y-%m-%dT%H:%M:%S")
    with csv_path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(fields)

        def _log_row(k: int, seg: int, phase: str, act_j: int,
                     t_send: float, t_recv: float, overrun: int,
                     cmd_abs: list[float], fb_row: tuple) -> None:
            w.writerow(
                [f"{t_send:.4f}", k, seg, phase, act_j,
                 f"{t_send:.4f}", f"{t_recv:.4f}", overrun]
                + [_fmt(last_pose.get(j), 3) for j in range(N_JOINTS)]
                + [f"{c:.3f}" for c in cmd_abs]
                + [_fmt(fb_row[0]), _fmt(fb_row[1], 1),
                   _fmt(fb_row[2], 2), _fmt(fb_row[3], 1)]
                + [_fmt(abs(float(last_fb[j].get("current_a") or 0.0)))
                   if j in last_fb else ""
                   for j in range(N_JOINTS)])

        def _trip_feedback(active: list[int]) -> tuple:
            """Throttled current/temp trips; returns the fb CSV cells."""
            nonlocal last_fb, last_fb_t, tripped_error
            t_now = time.monotonic()
            fresh = False
            if t_now - last_fb_t >= 1.0 / FEEDBACK_HZ:
                try:
                    fb = bus.read_all_feedback()
                    if isinstance(fb, dict) and fb:
                        last_fb = fb
                        last_fb_t = t_now
                        fresh = True
                except Exception:
                    pass
            # CSV feedback cells follow the HIGHEST-current joint of the
            # active set (multi-joint traj mode used to log whichever
            # joint came last — useless for diagnosing an overcurrent).
            fb_row = (None, None, None, None)
            fb_row_cur = -1.0
            for j in active:
                rec = last_fb.get(j)
                if not rec:
                    continue
                cur_a = abs(float(rec.get("current_a") or 0.0))
                temp = float(rec.get("temp_c") or 0.0)
                if seg_stats:
                    seg_stats[-1]["peak_current_a"] = max(
                        seg_stats[-1]["peak_current_a"], cur_a)
                if cur_a > max_cur:
                    tripped_error = (f"joint {j} overcurrent {cur_a:.2f} A "
                                     f"(limit {max_cur:.2f}) — possible "
                                     f"jam or wrong logical zero; limped")
                elif temp >= MAX_TEMP_C:
                    # Debounced: only fresh polls advance the counter, so a
                    # single corrupted byte (cached for ~3 ticks) can't trip.
                    if fresh:
                        hot_polls[j] = hot_polls.get(j, 0) + 1
                    if hot_polls.get(j, 0) >= TEMP_TRIP_POLLS:
                        tripped_error = (f"joint {j} hot {temp:.0f} C "
                                         f"({hot_polls[j]} consecutive "
                                         f"polls); limped")
                elif fresh:
                    hot_polls[j] = 0
                if cur_a > fb_row_cur:
                    fb_row_cur = cur_a
                    fb_row = (cur_a, float(rec.get("load_pct") or 0.0),
                              float(rec.get("volt") or 0.0), temp)
            return fb_row

        # -- automatic start-pose glide (no operator hand-posing) --------
        if glide_target is not None and tripped_error is None:
            start = [pose0.get(j, 0.0) for j in range(N_JOINTS)]
            target = list(glide_target)
            for j in range(N_JOINTS):
                lo, hi = AXIS_LIMITS[axis_of(j)]
                target[j] = min(hi, max(lo, target[j]))
            travel = max(abs(target[j] - start[j])
                         for j in range(N_JOINTS))
            if travel > GLIDE_TOL_DEG:
                dur = min(GLIDE_TIMEOUT_S,
                          max(0.5, travel / GLIDE_RATE_DEG_S))
                n_glide = int(round((dur + GLIDE_SETTLE_S) * hz))
                _progress(f"glide to start pose: worst joint "
                          f"{travel:.0f} deg over {dur:.1f} s")
                tg0 = time.monotonic()
                for k in range(n_glide):
                    if abort_check():
                        aborted = True
                        break
                    t_sched = tg0 + k * dt
                    now = time.monotonic()
                    if now < t_sched:
                        time.sleep(t_sched - now)
                    frac = min(1.0, (k / hz) / dur)
                    ease = frac * frac * (3.0 - 2.0 * frac)
                    cmd_abs = [start[j] + (target[j] - start[j]) * ease
                               for j in range(N_JOINTS)]
                    t_send = time.monotonic() - t0
                    try:
                        bus.write_all(cmd_abs, speed=write_speed,
                                      acc=write_acc)
                    except Exception as e:
                        tripped_error = f"bus write failed (glide): {e}"
                        break
                    pose, _miss = _read_pose()
                    t_recv = time.monotonic() - t0
                    last_pose.update(pose)
                    fb_row = _trip_feedback(list(range(N_JOINTS)))
                    if tripped_error:
                        break
                    for j in range(N_JOINTS):
                        if (j in pose and abs(cmd_abs[j] - pose[j])
                                > MAX_TRACK_ERR_DEG):
                            tripped_error = (
                                f"joint {j} fell {abs(cmd_abs[j] - pose[j]):.0f}"
                                f" deg behind during the start-pose glide "
                                f"— mechanical jam or wrong logical zero; "
                                f"limped. Re-check set_zero.")
                            break
                    if tripped_error:
                        break
                    _log_row(-n_glide + k, -1, "glide", -1, t_send,
                             t_recv, 0, cmd_abs, fb_row)
                # Verify the pose actually arrived before any segment runs.
                if tripped_error is None and not aborted:
                    pose, miss = _read_pose()
                    if miss:
                        tripped_error = (f"joints {miss} not answering "
                                         f"after glide")
                    else:
                        worst_j = max(range(N_JOINTS), key=lambda j: abs(
                            pose.get(j, 0.0) - target[j]))
                        worst = abs(pose.get(worst_j, 0.0) - target[worst_j])
                        if worst > GLIDE_TOL_DEG:
                            tripped_error = (
                                f"start pose did not verify: joint "
                                f"{worst_j} off by {worst:.1f} deg after "
                                f"glide (tol {GLIDE_TOL_DEG:g}) — check "
                                f"for obstruction / wrong logical zero.")

        for k, tick in enumerate(ticks):
            if tripped_error or aborted:
                break
            if abort_check():
                aborted = True
                break
            # -- fixed-rate schedule (rl_policy pattern), anchored AFTER
            # the glide so its duration doesn't count as overrun ----------
            if t_run0 is None:
                t_run0 = time.monotonic()
            t_sched = t_run0 + k * dt
            now = time.monotonic()
            if now < t_sched:
                time.sleep(t_sched - now)
            elif now - t_sched > 0.5 * dt:
                overruns += 1

            # -- segment bookkeeping / home capture -----------------------
            if tick["seg"] != cur_seg:
                cur_seg = tick["seg"]
                pose, miss = _read_pose()
                if miss:
                    tripped_error = (f"seg {cur_seg}: joints {miss} not "
                                     f"answering at segment start")
                    break
                spec = protocol["segments"][cur_seg]
                if spec.get("kind") == "traj" and cur_seg > 0:
                    # Mid-protocol traj must be continuous with the pose
                    # the previous segments left behind.
                    tol = float(spec.get("start_tol_deg",
                                         DEFAULT_START_TOL_DEG))
                    row0 = spec["q_deg"][0]
                    worst_j = max(range(N_JOINTS), key=lambda j: abs(
                        pose.get(j, 0.0) - row0[j]))
                    worst = abs(pose.get(worst_j, 0.0) - row0[worst_j])
                    if worst > tol:
                        tripped_error = (
                            f"traj seg {cur_seg}: present pose off by "
                            f"{worst:.1f} deg at joint {worst_j} (tol "
                            f"{tol:.0f}) — protocol discontinuity; limped")
                        break
                seg_home[cur_seg] = [pose.get(j, 0.0)
                                     for j in range(N_JOINTS)]
                seg_stats.append({"seg": cur_seg,
                                  "label": mat["seg_labels"][cur_seg],
                                  "ticks": 0, "peak_current_a": 0.0})
                _progress(f"seg {cur_seg + 1}/{len(mat['seg_labels'])}: "
                          f"{mat['seg_labels'][cur_seg]}",
                          seg=cur_seg, tick=k, total=len(ticks))
                last_pose.update(pose)
                trip_ref = dict(pose)   # re-anchor trip ref at the segment

            # -- build absolute command -----------------------------------
            home = seg_home[cur_seg]
            cmd_abs = list(home)
            for j in tick["active"]:
                v = (tick["cmd"][j] if tick["mode"] == "abs"
                     else home[j] + tick["cmd"][j])
                lo, hi = AXIS_LIMITS[axis_of(j)]
                cv = min(hi, max(lo, v))
                if cv != v:
                    clamped += 1
                cmd_abs[j] = cv

            # -- send, then read -------------------------------------------
            t_send = time.monotonic() - t0
            try:
                _write_active(tick, cmd_abs)
            except Exception as e:
                tripped_error = f"bus write failed: {e}"
                break
            pose, miss = _read_pose()
            t_recv = time.monotonic() - t0
            for j in live_joints:
                if j in miss:
                    missed[j] += 1
                    if missed[j] >= MAX_MISSED_READS:
                        tripped_error = (f"joint {j} (ID "
                                         f"{joint_to_servo_id(j)}) missed "
                                         f"{missed[j]} consecutive reads")
                        break
                else:
                    missed[j] = 0
            if tripped_error:
                break
            last_pose.update(pose)

            # -- throttled full feedback (current/temp) --------------------
            act_j = tick["active"][0] if len(tick["active"]) == 1 else -1
            fb_row = _trip_feedback(tick["active"])
            if tripped_error:
                break

            # -- tracking blow-up trip (unexpected force / wrong zero) -----
            # Reference slews toward cmd at the profile speed so large
            # steps don't self-trip; a stalled servo still trips within
            # ~MAX_TRACK_ERR_DEG / profile-speed seconds.
            for j in tick["active"]:
                ref = trip_ref.get(j, pose.get(j, cmd_abs[j]))
                d = cmd_abs[j] - ref
                ref += max(-trip_slew_deg, min(trip_slew_deg, d))
                trip_ref[j] = ref
                if j in pose and abs(ref - pose[j]) > MAX_TRACK_ERR_DEG:
                    tripped_error = (
                        f"joint {j} tracking error "
                        f"{abs(ref - pose[j]):.0f} deg > "
                        f"{MAX_TRACK_ERR_DEG:.0f} (cmd {cmd_abs[j]:.1f}, "
                        f"ref {ref:.1f}, present {pose[j]:.1f}) — "
                        f"mechanical jam or wrong logical zero; limped. "
                        f"Re-check set_zero.")
                    break
            if tripped_error:
                break

            seg_stats[-1]["ticks"] += 1
            _log_row(k, cur_seg, tick["phase"], act_j, t_send, t_recv,
                     1 if now - t_sched > 0.5 * dt else 0, cmd_abs, fb_row)
            if (k + 1) % int(hz) == 0:
                fh.flush()

    # Always limp at the end — never leave torque on a sysid pose.
    try:
        _set_torque_limit(bus, live_ids, 1000)
    except Exception:
        pass
    try:
        _limp_all(bus, live_ids)
    except Exception:
        pass

    done = sum(s["ticks"] for s in seg_stats)
    result = {
        "ok": tripped_error is None and not aborted,
        "mode": "sysid",
        "name": name,
        "protocol_hash": phash,
        "hz": hz,
        "ticks_planned": len(ticks),
        "ticks_done": done,
        "overruns": overruns,
        "clamped_cmds": clamped,
        "aborted": aborted,
        "error": tripped_error,
        "csv": str(csv_path),
        "segments": seg_stats,
        "started": started_iso,
        "ended": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    sum_path.write_text(json.dumps(
        {**result, "protocol": protocol}, indent=1))
    result["summary_json"] = str(sum_path)
    _progress("aborted" if aborted else
              (f"TRIP: {tripped_error}" if tripped_error else
               f"done · {done}/{len(ticks)} ticks · limp"))
    return result
