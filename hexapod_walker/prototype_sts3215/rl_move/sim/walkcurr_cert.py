"""walkcurr_cert.py — shared walk-curriculum certification machinery.

Moved out of rl_move/dynamics/train_ppo_transfer.py (operator order
2026-08-18, fb_20260818T065930_03b422: the walk curriculum must run on
the Warp/MJX GPU-physics trainer, so its gate/controller logic becomes
backend-neutral and importable from BOTH trainers). Behavior is
bit-identical to the walkcurr1/walkcurr2 originals; the transfer
trainer re-imports these names so its call sites and tests are
unchanged.

Also home to ``aggregate_walk_probe`` — the aggregation twin of the
transfer trainer's ``eval_task`` (same mean/nanmean rules, same keys)
over per-episode ``walk_probe`` payloads that walk_task.py's cfg-gated
in-env probe emits in the terminal info dict. The probe is how the MJX
cert loop measures the same physical quality metrics WITHOUT the deep
single-env introspection eval_task does on a C env (the MJX shims
mirror pad-body positions, not foot sites — the probe uses pad XY on
every backend, so C and MJX cert numbers are directly comparable).
"""
from __future__ import annotations

import numpy as np


def _nn(x, default=0.0) -> float:
    """nan-safe float."""
    x = float(x) if x is not None else float("nan")
    return default if x != x else x


# Walk-curriculum certification gate (operator order 2026-08-18,
# cw-dynrep-criticD-walkcurr1). A bucket passes its deterministic
# held-out assay (n >= 8 episodes) only if ALL hold:
#   no falls; six-leg gait cycling (aggregate switch rate AND the
#   weakest single foot both cycling); commanded progress >= 0.75;
#   correct direction (zero wrong-way episodes); bounded cross-track;
#   slip <= 2 per progress meter; roll <= 6 deg; bounded slew; and the
#   bucket's own stop threshold where stop segments exist.
WALKCURR_GATE = dict(
    cmd_prog_frac_min=0.75,
    slip_per_m_max=2.0,
    peak_roll_deg_max=6.0,
    slew_sat_max=0.5,
    cross_track_frac_max=0.30,
    contact_sw_per_s_min=3.0,
    foot_sw_min_per_s_min=0.5,
)


def walkcurr_bucket_pass(m: dict, spec: dict,
                         gate: dict | None = None
                         ) -> tuple[bool, dict]:
    """Apply the certification gate to one bucket's assay metrics.
    nan metrics FAIL their check (unmeasurable competence is not
    competence) except stop_speed, which is nan only when the assay
    drew no stop segments (nothing to gate that round).

    ``gate`` explicit arg > ``spec["gate"]`` (walkcurr2's per-bucket
    calibration, WALKCURR_GATE_V2_IGNITION/_QUALITY in walk_task.py)
    > the module WALKCURR_GATE default (walkcurr1, unchanged) — so the
    one trainer call site (``walkcurr_bucket_pass(m, spec)``) auto-
    selects the right tier for either curriculum version with no
    caller change."""
    if gate is None:
        gate = spec.get("gate", WALKCURR_GATE)
    def _ok_min(key, lo):
        v = _nn(m.get(key), float("-inf"))
        return v >= lo
    def _ok_max(key, hi):
        v = _nn(m.get(key), float("inf"))
        return v <= hi
    checks = {
        "no_falls": _nn(m.get("early_term_rate"), 1.0) == 0.0,
        "six_leg_gait": (_ok_min("contact_sw_per_s",
                                 gate["contact_sw_per_s_min"])
                         and _ok_min("foot_sw_min_per_s",
                                     gate["foot_sw_min_per_s_min"])),
        "progress": _ok_min("cmd_prog_frac", gate["cmd_prog_frac_min"]),
        "direction": _nn(m.get("wrong_way"), 1.0) == 0.0,
        "cross_track": _ok_max("cross_track_frac",
                               gate["cross_track_frac_max"]),
        "slip": _ok_max("slip_per_m", gate["slip_per_m_max"]),
        "roll": _ok_max("peak_roll_deg", gate["peak_roll_deg_max"]),
        "slew": _ok_max("slew_sat", gate["slew_sat_max"]),
    }
    stop_gate = spec.get("stop_gate")
    if stop_gate is not None:
        v = m.get("stop_speed_m_s")
        v = float(v) if v is not None else float("nan")
        # nan = the fixed held-out seeds drew no stop segment this
        # round — nothing to gate (possible but rare at n>=8).
        checks["stop"] = (v != v) or v <= float(stop_gate)
    return all(checks.values()), checks


class WalkCurrController:
    """Pure promotion/retention/rollback bookkeeping for the walk
    curriculum (unit-tested without SB3): consumes one
    walkcurr_update_admission status per cert round and answers
    'promote' / 'rollback' / None. Rollback fires after
    ``fail_streak_limit`` CONSECUTIVE rounds with a retained-bucket
    failure, and only once a promotion checkpoint exists to roll back
    to. A retention-clean round (retained all pass, whatever the
    frontier did) resets the streak."""

    def __init__(self, fail_streak_limit: int = 2):
        self.limit = int(fail_streak_limit)
        self.fail_streak = 0
        self.promotions = 0
        self.rollbacks = 0
        self.has_promo = False

    def record_round(self, status: dict) -> str | None:
        if status.get("promoted"):
            self.fail_streak = 0
            self.promotions += 1
            self.has_promo = True
            return "promote"
        if (status.get("frontier_bucket", 0) > 0
                and not status.get("retention_passed", True)):
            self.fail_streak += 1
            if self.fail_streak >= self.limit and self.has_promo:
                self.fail_streak = 0
                self.rollbacks += 1
                return "rollback"
        else:
            self.fail_streak = 0
        return None


# Per-episode metric keys the in-env walk probe emits (walk_task.py,
# cfg/attr-gated, default OFF). Mirrors train_ppo_transfer.QUALITY_KEYS
# plus the eval_task base metrics.
WALK_PROBE_KEYS = (
    "peak_roll_deg", "peak_pitch_deg", "peak_gyro_dps", "slip_m",
    "fwd_m", "contact_sw_per_s", "slew_sat", "slew_sat_all", "mean_h_m",
    "dh_m", "vx_rmse", "vy_rmse", "wz_rmse_dps", "cmd_prog_m",
    "cmd_prog_frac", "slip_per_m", "cross_track_frac", "wrong_way",
    "stop_speed_m_s", "foot_sw_min_per_s", "duty_factor")

# nan = "not measurable this episode" for the command-conditional keys
# (same set + reasoning as eval_task's _nan_ok).
_NAN_OK = {"cross_track_frac", "wrong_way", "stop_speed_m_s"}


def failed_probe_row() -> dict:
    """Synthetic row for an episode that terminated without a probe
    payload (safety early-rejection path): counts as a fall with
    unmeasurable metrics — it can only ever FAIL a gate."""
    row = {k: float("nan") for k in WALK_PROBE_KEYS}
    row.update({"return": float("nan"), "ep_len": 0.0,
                "early_term": 1.0})
    return row


def aggregate_walk_probe(rows: list[dict]) -> dict:
    """eval_task-compatible aggregation of per-episode walk_probe rows:
    plain mean for the historical keys (bit-exact aggregation
    semantics), nanmean for the command-conditional keys."""
    if not rows:
        raise ValueError("no probe rows to aggregate")
    out = {
        "return": float(np.mean([_nn(r.get("return")) for r in rows])),
        "ep_len": float(np.mean([_nn(r.get("ep_len")) for r in rows])),
        "early_term_rate": float(np.mean(
            [_nn(r.get("early_term"), 1.0) for r in rows])),
    }
    for k in WALK_PROBE_KEYS:
        arr = np.asarray([(float(r[k]) if r.get(k) is not None
                           else float("nan")) for r in rows], dtype=float)
        if k in _NAN_OK:
            out[k] = (float(np.nanmean(arr))
                      if np.any(~np.isnan(arr)) else float("nan"))
        else:
            out[k] = float(np.mean(arr))
    return out
