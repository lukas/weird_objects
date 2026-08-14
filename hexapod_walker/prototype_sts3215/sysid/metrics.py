"""Scalar sysid metrics: step response, sine tracking, latency stats.

All functions operate on plain arrays so the SAME code scores hardware
traces and sim replays — the reality gap is just the difference between
the two metric sets.
"""
from __future__ import annotations

import math

import numpy as np

MOTION_THRESH_DEG = 0.3      # first-motion threshold (encoder LSB 0.088)
SETTLE_BAND_DEG = 1.0
SETTLE_SPEED_DEG_S = 15.0

PCTS = (10, 25, 50, 75, 90, 95)


def step_metrics(t: np.ndarray, cmd: np.ndarray, q: np.ndarray) -> dict:
    """Metrics for one step segment (single joint, absolute degrees).

    ``cmd``/``q`` are the segment's command and measured position; the
    step edge is found from the command stream itself, so hardware and
    sim are scored identically.
    """
    dcmd = np.abs(np.diff(cmd))
    if not (dcmd > 1e-6).any():
        return {"ok": False, "error": "no command edge"}
    i_edge = int(np.argmax(dcmd > 1e-6)) + 1
    t_edge = float(t[i_edge])
    q0 = float(np.median(q[max(0, i_edge - 5):i_edge])) if i_edge else q[0]
    target = float(cmd[-1])
    amp = target - float(cmd[i_edge - 1])
    if abs(amp) < 1e-6:
        return {"ok": False, "error": "zero amplitude"}
    s = math.copysign(1.0, amp)
    dq = (q - q0) * s
    rel_t = t - t_edge

    moved = (dq > MOTION_THRESH_DEG) & (rel_t >= 0)
    i_move = int(np.argmax(moved)) if moved.any() else None
    r10 = (dq >= 0.1 * abs(amp)) & (rel_t >= 0)
    r90 = (dq >= 0.9 * abs(amp)) & (rel_t >= 0)
    i10 = int(np.argmax(r10)) if r10.any() else None
    i90 = int(np.argmax(r90)) if r90.any() else None

    vel = np.diff(q) / np.maximum(np.diff(t), 1e-4)
    qf = float(np.mean(q[-max(3, len(q) // 10):]))

    settle_ms = None
    spd = np.abs(np.concatenate([[0.0], vel]))
    for k in range(len(t)):
        if rel_t[k] < 0.15:
            continue
        if (abs(q[k] - target) <= SETTLE_BAND_DEG
                and spd[k] <= SETTLE_SPEED_DEG_S):
            settle_ms = rel_t[k] * 1000.0
            break

    return {
        "ok": True,
        "amp_deg": round(amp, 2),
        "latency_ms": None if i_move is None else round(rel_t[i_move] * 1e3, 1),
        "t90_ms": None if i90 is None else round(rel_t[i90] * 1e3, 1),
        "rise_ms": (None if i10 is None or i90 is None or i90 < i10
                    else round((rel_t[i90] - rel_t[i10]) * 1e3, 1)),
        "settle_ms": None if settle_ms is None else round(settle_ms, 1),
        "overshoot_deg": round(float(max(0.0, np.max(dq) - abs(amp))), 3),
        "ss_err_deg": round(qf - target, 3),
        "tracking_pct": round(100.0 * (qf - q0) / amp, 1),
        "peak_vel_deg_s": round(float(np.max(np.abs(vel))), 1) if len(vel)
        else None,
    }


def sine_metrics(t: np.ndarray, cmd: np.ndarray, q: np.ndarray,
                 freq_hz: float) -> dict:
    """Gain / phase / RMSE of a sine-tracking segment at a known freq.

    Fits ``a·sin + b·cos + c`` (linear lstsq) to command and response
    over the samples after the first full cycle (transient dropped).
    """
    w = 2.0 * math.pi * freq_hz
    keep = t >= (t[0] + 1.0 / freq_hz)
    if keep.sum() < 8:
        keep = np.ones_like(t, dtype=bool)
    ts, cs, qs = t[keep], cmd[keep], q[keep]
    A = np.column_stack([np.sin(w * ts), np.cos(w * ts), np.ones_like(ts)])

    def fit(y):
        (a, b, _c), *_ = np.linalg.lstsq(A, y, rcond=None)
        return math.hypot(a, b), math.atan2(b, a)

    amp_c, ph_c = fit(cs)
    amp_q, ph_q = fit(qs)
    if amp_c < 1e-6:
        return {"ok": False, "error": "no commanded sine"}
    lag = ph_c - ph_q
    while lag > math.pi:
        lag -= 2.0 * math.pi
    while lag < -math.pi:
        lag += 2.0 * math.pi
    return {
        "ok": True,
        "freq_hz": freq_hz,
        "gain": round(amp_q / amp_c, 4),
        "phase_lag_deg": round(math.degrees(lag), 2),
        "phase_lag_ms": round(1000.0 * lag / w, 1),
        "rmse_deg": round(float(np.sqrt(np.mean((qs - cs) ** 2))), 3),
    }


def latency_stats(values_ms: list[float]) -> dict:
    """Distribution summary (plan Phase 2: distributions, not constants)."""
    v = np.asarray([x for x in values_ms if x is not None], dtype=float)
    if v.size == 0:
        return {"n": 0}
    out = {"n": int(v.size), "mean_ms": round(float(v.mean()), 1),
           "max_ms": round(float(v.max()), 1)}
    for p in PCTS:
        out[f"p{p}_ms"] = round(float(np.percentile(v, p)), 1)
    return out


def tick_jitter_stats(t_send: np.ndarray, hz: float) -> dict:
    """Control-loop jitter from consecutive send timestamps."""
    dts = np.diff(t_send) * 1000.0
    nominal = 1000.0 / hz
    if dts.size == 0:
        return {"n": 0}
    return {
        "n": int(dts.size),
        "nominal_ms": round(nominal, 1),
        "median_ms": round(float(np.median(dts)), 2),
        "p95_ms": round(float(np.percentile(dts, 95)), 2),
        "max_ms": round(float(dts.max()), 2),
        "late_pct": round(100.0 * float((dts > 1.5 * nominal).mean()), 2),
    }


def segment_metrics(tr: dict, seg: dict, *, q: np.ndarray | None = None) -> dict:
    """Score one segment of a trace (optionally against replacement q,
    e.g. the sim replay's joint stream)."""
    sl = seg["sl"]
    j = seg["joint"]
    qq = (q if q is not None else tr["q"])
    if seg["kind"] == "step":
        # Score the pre+move window only: the trailing "rest" phase
        # returns the command to home, which would zero the step edge.
        phases = tr["phase"][sl]
        end = next((k for k, p in enumerate(phases) if p == "rest"),
                   len(phases))
        ssl = slice(sl.start, sl.start + end)
        return step_metrics(tr["t"][ssl], tr["cmd"][ssl, j], qq[ssl, j])
    if seg["kind"] == "sine":
        return sine_metrics(tr["t"][sl], tr["cmd"][sl, j], qq[sl, j],
                            float(seg["spec"]["freq_hz"]))
    # traj: whole-body RMSE over moving joints.
    dq = qq[sl] - tr["cmd"][sl]
    moving = np.ptp(tr["cmd"][sl], axis=0) > 2.0
    return {"ok": True, "kind": "traj",
            "rmse_moving_deg": round(float(np.sqrt(np.mean(
                dq[:, moving] ** 2))), 3) if moving.any() else None}


def compare_joint_streams(t: np.ndarray, q_hw: np.ndarray,
                          q_sim: np.ndarray, *, moving_ptp_deg: float = 2.0
                          ) -> dict:
    """Position/velocity RMSE between hardware and sim joint streams."""
    moving = np.ptp(q_hw, axis=0) > moving_ptp_deg
    dq = q_sim - q_hw
    out = {"moving_joints": int(moving.sum())}
    if moving.any():
        out["q_rmse_deg"] = round(float(np.sqrt(np.mean(
            dq[:, moving] ** 2))), 3)
        dt = np.maximum(np.diff(t), 1e-4)[:, None]
        v_hw = np.diff(q_hw, axis=0) / dt
        v_sim = np.diff(q_sim, axis=0) / dt
        out["qd_rmse_deg_s"] = round(float(np.sqrt(np.mean(
            (v_sim - v_hw)[:, moving] ** 2))), 2)
    return out
