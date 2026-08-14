"""Bulk session evaluator (rl_move.sim.bulk_session_eval) — semantics.

Directive fb_20260814T205137_33f21c requirements test-locked here:
deterministic schedule generation, plan/manifest determinism (workers
recompute the plan, nothing is copied), resume/idempotence (a done
shard is never re-run), Wilson interval math, aggregation strata/
segment minima, and the legacy eval_modeseq path staying default-off
(--strip-ep=0, --torch-seed=None).

Run: python3 -m pytest rl_move/tests/test_bulk_session_eval.py -q
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from rl_move.sim import bulk_session_eval as bse


# ---- deterministic schedule generation (the shared-seed contract) ----

def test_drive_schedule_deterministic_and_shaped():
    from rl_move.sim.eval_modeseq import _random_drive_schedule
    a = _random_drive_schedule(np.random.default_rng(900007), 0.05,
                               0.06, 14.0)
    b = _random_drive_schedule(np.random.default_rng(900007), 0.05,
                               0.06, 14.0)
    assert a == b, "same seed must give the same joystick session"
    c = _random_drive_schedule(np.random.default_rng(900008), 0.05,
                               0.06, 14.0)
    assert a != c, "different seeds must differ"
    # engage dwell first, trailing zero-command settle window last
    assert a[0][1:] == (0.0, 0.0) and a[-1][1:] == (0.0, 0.0)
    # guaranteed mid-drive stop-go and a direction flip
    mids = a[1:-1]
    assert any(vx == 0.0 and vy == 0.0 for _d, vx, vy in mids)
    angs = [np.arctan2(vy, vx) for _d, vx, vy in mids
            if (vx, vy) != (0.0, 0.0)]
    assert any(abs(x + y) < 1e-9 and abs(x) > 1e-9
               for x, y in zip(angs, angs[1:])), "no direction flip"


# ---- plan determinism + matched seeds across candidates --------------

def test_plan_deterministic_and_matched():
    p1, p2 = bse.plan("c1"), bse.plan("c1")
    assert p1 == p2
    assert bse.manifest_hash(p1) == bse.manifest_hash(p2)
    by_cand = {}
    for sh in p1:
        by_cand.setdefault((sh["cand"], sh["mode"]), []).append(sh["seed"])
    seedsets = {k: tuple(v) for k, v in by_cand.items()}
    # every candidate sees the IDENTICAL seed list per mode
    det = {v for (c, m), v in seedsets.items() if m == "det"}
    sto = {v for (c, m), v in seedsets.items() if m == "sto"}
    assert len(det) == 1 and len(sto) == 1
    # det and sto banks are disjoint (fresh, held-out both)
    assert not set(next(iter(det))) & set(next(iter(sto)))
    # partition covers all shards exactly once
    parts = [s for i in range(11) for s in p1 if s["id"] % 11 == i]
    assert sorted(s["id"] for s in parts) == [s["id"] for s in p1]


def test_cohort_seed_bank_bump():
    # c1 (and any unlisted cohort) stays on the legacy bank —
    # bit-exact for every pre-existing caller.
    p_c1 = bse.plan("c1", ["spec"], ("det", "sto"))
    dets = {sh["seed"] for sh in p_c1 if sh["mode"] == "det"}
    stos = {sh["seed"] for sh in p_c1 if sh["mode"] == "sto"}
    assert min(dets) == bse.SEED_DET and min(stos) == bse.SEED_STO
    # c2 gets its own pre-registered fresh bank (SESSION_BULK_GATE.md
    # "Cohort c2"), never overlapping c1's (retired) bank.
    p_c2 = bse.plan("c2", ["spec"], ("det", "sto"))
    dets2 = {sh["seed"] for sh in p_c2 if sh["mode"] == "det"}
    stos2 = {sh["seed"] for sh in p_c2 if sh["mode"] == "sto"}
    assert min(dets2) == 920000 and min(stos2) == 930000
    assert not (dets2 & dets) and not (stos2 & stos)


def test_shard_cmd_candidates():
    sh = dict(cand="spec", mode="det", seed=900000, eps=6, out="/tmp/x.json")
    c = bse.shard_cmd(sh)
    assert "--stand" in c and "--walk" in c and "--single" not in c
    assert "--cfg-set" in c and "goal.walk_obs_body_vel=2" in c
    assert "--entry-slew" in c and "--drive-random" in c
    assert "--stochastic" not in c
    sh2 = dict(sh, cand="td2", mode="sto")
    c2 = bse.shard_cmd(sh2)
    assert "--single" in c2 and "--stand" not in c2
    assert "--stochastic" in c2
    # torch seed pinned to the shard seed => sto is reproducible
    assert c2[c2.index("--torch-seed") + 1] == str(sh["seed"])


# ---- resume / idempotence -------------------------------------------

def _fake_shard_json(sh, tmpdir):
    out = Path(tmpdir) / Path(sh["out"]).name
    eps = []
    for e in range(sh["eps"]):
        kind = ("flat", "bridge", "crouch")[e % 3]
        fall = (e % 3 == 1)  # bridge episodes fall in their walk segment
        segs = [
            {"i": 0, "mode": "rise", "start_kind": kind, "success": True,
             "switch_tilt_deg": 2.0},
            {"i": 1, "mode": "walk", "success": not fall,
             "gait_valid": not fall, "slip_per_m": 1.5,
             "prog_ratio": 1.0, "drive_z_mean_mm": 134.0,
             "stop_settle_ok": False, "stop_settle_speed_mps": 0.04,
             **({"fall": "tilt_roll"} if fall else {})},
            {"i": 2, "mode": "lower", "success": True} if not fall
            else {"i": 2, "mode": "lower", "skipped": True},
            {"i": 3, "mode": "rise", "start_kind": "reanchor_post_lower",
             "success": True} if not fall
            else {"i": 3, "mode": "rise", "skipped": True},
        ]
        eps.append({"ep": e, "zero_fall": not fall, "segments": segs})
    out.write_text(json.dumps({"episodes": eps, "summary": {}}))
    return out


def test_worker_resume_idempotent(tmp_path, monkeypatch):
    monkeypatch.setattr(bse, "OUT_ROOT", tmp_path)
    ran = []

    def fake_runner(sh):
        ran.append(sh["out"])
        Path(sh["out"]).parent.mkdir(parents=True, exist_ok=True)
        Path(sh["out"]).write_text(json.dumps(
            {"episodes": [], "summary": {}}))
        return True

    class A:  # minimal args namespace
        cohort, cands, modes = "t", ["spec"], "det"
        part, procs = "0/1", 1
        _runner = staticmethod(fake_runner)

    a = A()
    assert bse.cmd_worker(a) == 0
    n_first = len(ran)
    assert n_first == bse.N_SHARDS
    assert bse.cmd_worker(a) == 0
    assert len(ran) == n_first, "second pass must re-run NOTHING"
    # a corrupt output is not 'done' and gets re-run
    victim = bse.plan("t", ["spec"], ("det",))[0]
    Path(victim["out"]).write_text("{not json")
    assert bse.cmd_worker(a) == 0
    assert len(ran) == n_first + 1


# ---- wilson + aggregation -------------------------------------------

def test_wilson_known_values():
    lo, hi = bse.wilson(0, 12)
    assert lo == 0.0 and 0.22 < hi < 0.26      # 0/12 -> upper ~0.243
    lo, hi = bse.wilson(12, 12)
    assert hi > 0.999 and 0.73 < lo < 0.76
    lo, hi = bse.wilson(50, 100)
    assert 0.40 < lo < 0.41 and 0.59 < hi < 0.60
    assert bse.wilson(0, 0) == (0.0, 1.0)


def test_aggregate_strata_and_minima(tmp_path, monkeypatch):
    monkeypatch.setattr(bse, "OUT_ROOT", tmp_path)
    shards = bse.plan("t", ["spec"], ("det",), n_shards=2)
    for sh in shards:
        Path(sh["out"]).parent.mkdir(parents=True, exist_ok=True)
        _fake_shard_json(sh, Path(sh["out"]).parent)
    rows, missing = bse._flatten("t", ["spec"], ("det",))
    # plan default is N_SHARDS; only 2 exist -> the rest are missing
    assert len(missing) == bse.N_SHARDS - 2
    assert len(rows) == 2 * bse.SHARD_EPS
    agg = bse.summarize(rows)["spec"]["det"]
    assert agg["episodes"] == 12
    assert agg["zero_fall"]["k"] == 8            # bridge eps (2/shard) fall
    assert agg["zero_fall_by_stratum"]["bridge"]["k"] == 0
    assert agg["min_stratum"]["name"] == "bridge"
    assert agg["min_segment"]["name"] == "walk"
    assert agg["segments"]["walk"]["falls"] == 4
    assert agg["rise_by_ordinal"]["first"]["k"] == 12
    assert agg["rise_by_ordinal"]["post_lower"]["n"] == 8
    assert agg["fall_reasons"] == {"walk:tilt_roll": 4}
    assert agg["stop_settle_ok"]["k"] == 0
    assert agg["medians"]["slip_per_m"] == 1.5
    # skipped segments never count toward rates
    assert agg["segments"]["lower"]["n"] == 8


# ---- legacy eval_modeseq path stays default-off ----------------------

def test_eval_modeseq_legacy_defaults():
    src = Path("rl_move/sim/eval_modeseq.py").read_text()
    assert '"--strip-ep", type=int, default=0' in src
    assert '"--torch-seed", type=int, default=None' in src
    # strips gate on the (default 0) strip-ep, not a hardcoded ep 0
    assert "ep == args.strip_ep" in src
