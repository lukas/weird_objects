"""Bulk held-out joystick-session evaluator (hw product gate, 08-14).

Operator directive fb_20260814T205137_33f21c: replace 12-seed reads
with a BULK, held-out, stratified session cohort — the hierarchical
frozen-skill controller (stance specialist + tall-walk specialist
behind the explicit session grammar) is the PRODUCT BASELINE; single
shared-policy models are research. This tool measures both honestly
on matched fresh seeds/schedules and reports per-segment rates with
95% intervals so no pooled number can hide a failed skill.

Pure orchestration around `rl_move.sim.eval_modeseq --drive-random
--entry-slew` (the ~60 s guarded session REST->RISE->SETTLE->
WALK_ENTRY->DRIVE(stop-go+direction flip)->STOP_SETTLE->LOWER->RISE->
DRIVE): no env/reward/training code is touched; every shard is a
plain eval_modeseq invocation, so the legacy path stays bit-exact.

Design:

- The PLAN is a pure function of the CLI cohort spec — every worker
  recomputes it identically (test-locked), so nothing needs to be
  copied to pods except code. Seeds are the shard axis: shard i of
  the det pass uses seed SEED_DET+i, sto uses SEED_STO+i, IDENTICAL
  across candidates (drive schedules derive from the seed, so all
  candidates face the same joystick sessions). Shard size 6 keeps the
  cold-start rotation (flat,bridge,crouch x2) exactly balanced.
- WORKER is resumable/idempotent: a shard whose output JSON exists
  and parses (has "summary") is skipped; re-running a partial cohort
  only executes the holes. `--part i/N` statically partitions shards
  across pods; `--procs P` runs P eval subprocesses concurrently
  (each eval self-caps OMP threads at 2). No video in bulk.
- AGGREGATE emits episodes.jsonl (one line per session) + an
  aggregate JSON with Wilson 95% intervals on: complete-session
  zero-fall (overall + per cold-start stratum flat/bridge/crouch),
  per-segment-type success, first rise vs post-lower rise, walk
  gait_valid, stop-settle; plus fall-reason histograms and the
  evidence medians (slip/m, prog_ratio, drive height, switch tilt).
  It names the MIN stratum and MIN segment per candidate.
- RERENDER lists (or executes) matched re-runs of every failed
  episode plus a seeded random sample of clean ones, with frame
  strips (`--strip-ep`), reproducible because bulk shards pass
  `--torch-seed` (stochastic sampling is seeded).

    uv run python -m rl_move.sim.bulk_session_eval plan --print
    uv run python -m rl_move.sim.bulk_session_eval worker --part 0/11 --procs 8
    uv run python -m rl_move.sim.bulk_session_eval aggregate
    uv run python -m rl_move.sim.bulk_session_eval rerender --sample 12
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import subprocess
import sys
from pathlib import Path

# ---- cohort spec (the pre-registered defaults; SESSION_BULK_GATE.md) ----
GRAMMAR = "rise,walk,lower,rise,walk"
SPEED, SPEED_MAX, DRIVE_SECONDS = 0.05, 0.06, 14.0
ENTRY_SLEW = "1.5,0.25"           # TAKEOFF.md deploy design, all arms
SHARD_EPS = 6                     # flat,bridge,crouch x2 per shard
N_SHARDS = 50                     # x SHARD_EPS = 300 sessions/pass
SEED_DET = 900000                 # held-out seed banks (fresh; retire
SEED_STO = 910000                 # for tuning once the gate is read)
OUT_ROOT = Path("logs/bulk_session")

# Per-cohort seed-bank bases (SESSION_BULK_GATE.md clause 7: "future
# cohorts bump the bases" — each cohort's banks are retired for tuning
# the moment its aggregate.json is read, so the next cohort needs a
# fresh, never-before-used pair). Cohorts not listed here (including
# the legacy/test cohort "c1"/"t"...) fall back to SEED_DET/SEED_STO
# unchanged -> bit-exact for every pre-08-14-cycle-2 caller.
COHORT_SEED_BASE: dict[str, tuple[int, int]] = {
    "c2": (920000, 930000),   # cw-stand-postlower1 gate (retire on read)
    "c3": (940000, 950000),   # postlower3 lineage gate (retire on read)
    "c4": (960000, 970000),   # postlower4 gate (retire on read)
    "c5rr": (980000, 990000),  # rise-from-h probe (retire on read)
}

# Cohorts whose shards run eval_modeseq's --rise-from-h (WAITING-ON
# 08-17, hw): the post-lower rise segment's height schedule mirrors
# `goal.mode_seq_rise_from_h`'s trained "hold at current height, ramp
# to target" shape instead of the legacy "hold at belly, ramp to
# target" one. Prices the operator's postlower fork option (a) —
# aligning the runner/instrument to train==deploy semantics — by
# reading BOTH the parent (`spec`) and the c4 candidate (`spec-pl4`)
# under the SAME matched schedule (SESSION_BULK_GATE.md Cohort c4
# verdict named this exact mismatch as the open question).
COHORT_RISE_FROM_H: set[str] = {"c5rr"}

CANDIDATES: dict[str, dict] = {
    # A. product baseline: hierarchical frozen specialists
    "spec": {
        "stand": "rl_move/sim/policies/ppo_goal_cw_stand_footlow2_hard1.zip",
        "walk": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip",
        "cfg": ["goal.walk_obs_body_vel=2"],   # bcgait1 vel:=ref contract
    },
    # B. single shared model (winning distill artifact)
    "td2": {"single":
            "rl_move/sim/policies/ppo_goal_cw_gru_dual_bc_transdagger2.zip"},
    # C. single shared model (diagnostic)
    "td3": {"single":
            "rl_move/sim/policies/ppo_goal_cw_gru_dual_bc_transdagger3.zip"},
    # D. cohort c2 candidate: postlower1 stance (rise_start_bank) +
    # the SAME frozen tall walker as `spec` — SESSION_BULK_GATE.md
    # "Cohort c2".
    "spec-pl": {
        "stand": "rl_move/sim/policies/ppo_goal_cw_stand_postlower1.zip",
        "walk": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip",
        "cfg": ["goal.walk_obs_body_vel=2"],
    },
    # E. cohort c3 candidate: postlower3 stance (in-context lower->rise
    # sequence training, goal.mode_seq_stance) + the SAME frozen tall
    # walker as `spec` — SESSION_BULK_GATE.md "Cohort c3".
    "spec-pl3": {
        "stand": "rl_move/sim/policies/ppo_goal_cw_stand_postlower3.zip",
        "walk": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip",
        "cfg": ["goal.walk_obs_body_vel=2"],
    },
    # F. cohort c4 candidate: postlower4 stance (in-context sequence
    # training + goal.mode_seq_rise_from_h, the "stand up from where
    # you are" fix for the c3 belly-detour) + the SAME frozen tall
    # walker as `spec` — SESSION_BULK_GATE.md "Cohort c4".
    "spec-pl4": {
        "stand": "rl_move/sim/policies/ppo_goal_cw_stand_postlower4.zip",
        "walk": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip",
        "cfg": ["goal.walk_obs_body_vel=2"],
    },
}
MODES = ("det", "sto")


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score 95% interval for a binomial proportion."""
    if n <= 0:
        return (0.0, 1.0)
    p = k / n
    d = 1.0 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def shard_cmd(sh: dict) -> list[str]:
    cand = CANDIDATES[sh["cand"]]
    cmd = [sys.executable, "-m", "rl_move.sim.eval_modeseq",
           "--episodes", str(sh["eps"]), "--grammar", GRAMMAR,
           "--drive-random", "--speed", str(SPEED),
           "--drive-speed-max", str(SPEED_MAX),
           "--drive-seconds", str(DRIVE_SECONDS),
           "--entry-slew", ENTRY_SLEW,
           "--seed", str(sh["seed"]), "--torch-seed", str(sh["seed"]),
           "--out", sh["out"]]
    if sh["mode"] == "sto":
        cmd.append("--stochastic")
    if sh.get("rise_from_h"):
        # WAITING-ON 08-17 (hw): the eval-side "remaining rise"
        # semantics probe — mirrors goal.mode_seq_rise_from_h so the
        # post-lower rise segment is judged under the SAME schedule
        # shape postlower4 was trained on (SESSION_BULK_GATE.md
        # Cohort c4 verdict named this train/eval mismatch as the
        # reason c4 fell short of parity under the legacy schedule).
        cmd.append("--rise-from-h")
    if "single" in cand:
        cmd += ["--single", cand["single"]]
    else:
        cmd += ["--stand", cand["stand"], "--walk", cand["walk"]]
    for kv in cand.get("cfg", []):
        cmd += ["--cfg-set", kv]
    return cmd


def plan(cohort: str, cands=None, modes=MODES, n_shards: int = N_SHARDS,
         eps: int = SHARD_EPS) -> list[dict]:
    """Deterministic shard manifest — identical on every host."""
    cands = list(cands or CANDIDATES)
    det_base, sto_base = COHORT_SEED_BASE.get(cohort, (SEED_DET, SEED_STO))
    rise_from_h = cohort in COHORT_RISE_FROM_H
    shards, sid = [], 0
    for cand in cands:
        for mode in modes:
            base = det_base if mode == "det" else sto_base
            for i in range(n_shards):
                seed = base + i
                shards.append({
                    "id": sid, "cand": cand, "mode": mode,
                    "seed": seed, "eps": eps,
                    "rise_from_h": rise_from_h,
                    "out": str(OUT_ROOT / cohort /
                               f"{cand}_{mode}_s{seed}.json"),
                })
                sid += 1
    return shards


def manifest_hash(shards: list[dict]) -> str:
    """Seed-bank/manifest hash (recorded in the gate doc, retired
    once the gate is read)."""
    blob = json.dumps(shards, sort_keys=True).encode()
    return hashlib.sha256(blob).hexdigest()


def shard_done(sh: dict) -> bool:
    p = Path(sh["out"])
    if not p.is_file():
        return False
    try:
        return "summary" in json.loads(p.read_text())
    except (json.JSONDecodeError, OSError):
        return False


def _run_shard(sh: dict, timeout_s: float = 5400.0) -> bool:
    Path(sh["out"]).parent.mkdir(parents=True, exist_ok=True)
    log = Path(sh["out"]).with_suffix(".log")
    with log.open("w") as fh:
        try:
            # eval_modeseq's exit code is its PASS bar, not an error;
            # done-ness is "output json written and parseable".
            subprocess.run(shard_cmd(sh), stdout=fh,
                           stderr=subprocess.STDOUT, timeout=timeout_s)
        except subprocess.TimeoutExpired:
            fh.write("\nTIMEOUT\n")
    return shard_done(sh)


def cmd_worker(args) -> int:
    shards = plan(args.cohort, args.cands, _modes(args))
    part, nparts = (int(x) for x in args.part.split("/"))
    mine = [s for s in shards if s["id"] % nparts == part]
    todo = [s for s in mine if not shard_done(s)]
    print(f"[worker {args.part}] {len(mine)} shards, "
          f"{len(mine) - len(todo)} already done, {len(todo)} to run")
    runner = args._runner if getattr(args, "_runner", None) else _run_shard
    failed = []
    if args.procs <= 1:
        for sh in todo:
            if not runner(sh):
                failed.append(sh["out"])
    else:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=args.procs) as ex:
            for sh, ok in zip(todo, ex.map(runner, todo)):
                if not ok:
                    failed.append(sh["out"])
    print(f"[worker {args.part}] done; {len(failed)} shard(s) failed"
          + ("" if not failed else ": " + " ".join(failed)))
    return 1 if failed else 0


# ---------------------------- aggregation ----------------------------

def _flatten(cohort: str, cands, modes) -> tuple[list[dict], list[dict]]:
    """-> (episode rows, missing shards)"""
    rows, missing = [], []
    for sh in plan(cohort, cands, modes):
        if not shard_done(sh):
            missing.append(sh)
            continue
        data = json.loads(Path(sh["out"]).read_text())
        for ep in data["episodes"]:
            segs = [s for s in ep["segments"] if not s.get("skipped")]
            rises = [s for s in segs if s["mode"] == "rise"]
            row = {
                "cand": sh["cand"], "mode": sh["mode"],
                "seed": sh["seed"], "ep": ep["ep"],
                "stratum": (rises[0].get("start_kind", "?")
                            if rises else "?"),
                "zero_fall": bool(ep["zero_fall"]),
                "clean": all(s.get("success") for s in segs)
                          and len(segs) == len(ep["segments"]),
                "segments": ep["segments"],
                # carried through so cmd_rerender reconstructs the
                # EXACT shard invocation (a rise_from_h cohort's
                # rerender must also pass --rise-from-h, or the
                # eye-clause re-render silently checks the wrong
                # schedule -- bug found + fixed 08-17 while reading
                # Cohort c5rr's failures).
                "rise_from_h": bool(sh.get("rise_from_h", False)),
            }
            rows.append(row)
    return rows, missing


def _rate(k: int, n: int) -> dict:
    lo, hi = wilson(k, n)
    return {"k": k, "n": n, "rate": round(k / n, 4) if n else None,
            "ci95": [round(lo, 4), round(hi, 4)]}


def _med(vals: list) -> float | None:
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    vals = sorted(vals)
    m = len(vals) // 2
    return round((vals[m] if len(vals) % 2 else
                  (vals[m - 1] + vals[m]) / 2.0), 4)


def summarize(rows: list[dict]) -> dict:
    out: dict = {}
    keys = sorted({(r["cand"], r["mode"]) for r in rows})
    for cand, mode in keys:
        rs = [r for r in rows if r["cand"] == cand and r["mode"] == mode]
        agg: dict = {"episodes": len(rs)}
        agg["zero_fall"] = _rate(sum(r["zero_fall"] for r in rs), len(rs))
        agg["clean_session"] = _rate(sum(r["clean"] for r in rs), len(rs))
        strata = {}
        for st in ("flat", "bridge", "crouch"):
            g = [r for r in rs if r["stratum"] == st]
            if g:
                strata[st] = _rate(sum(r["zero_fall"] for r in g), len(g))
        agg["zero_fall_by_stratum"] = strata
        if strata:
            worst = min(strata, key=lambda s: strata[s]["rate"])
            agg["min_stratum"] = {"name": worst, **strata[worst]}
        # per-segment-type + rise ordinals + evidence fields
        segsum: dict = {}
        falls: dict = {}
        rise_ord: dict[int, list] = {}
        walk_ev = {"slip_per_m": [], "prog_ratio": [], "drive_z_mean_mm": [],
                   "stop_settle_speed_mps": []}
        tilts = []
        stop_ok = stop_n = gait_ok = walk_n = 0
        for r in rs:
            ordn = 0
            for s in r["segments"]:
                if s.get("skipped"):
                    continue
                m = s["mode"]
                d = segsum.setdefault(m, {"n": 0, "success": 0, "falls": 0})
                d["n"] += 1
                d["success"] += bool(s.get("success"))
                if s.get("fall"):
                    d["falls"] += 1
                    falls[f"{m}:{s['fall']}"] = \
                        falls.get(f"{m}:{s['fall']}", 0) + 1
                if "switch_tilt_deg" in s:
                    tilts.append(s["switch_tilt_deg"])
                if m == "rise":
                    rise_ord.setdefault(ordn, []).append(s)
                    ordn += 1
                if m == "walk":
                    walk_n += 1
                    gait_ok += bool(s.get("gait_valid"))
                    for k in walk_ev:
                        if s.get(k) is not None:
                            walk_ev[k].append(s[k])
                    if "stop_settle_ok" in s:
                        stop_n += 1
                        stop_ok += bool(s["stop_settle_ok"])
        agg["segments"] = {m: {**_rate(d["success"], d["n"]),
                               "falls": d["falls"]}
                           for m, d in segsum.items()}
        if segsum:
            worst = min(agg["segments"],
                        key=lambda m: agg["segments"][m]["rate"])
            agg["min_segment"] = {"name": worst, **agg["segments"][worst]}
        agg["rise_by_ordinal"] = {
            ("first" if i == 0 else "post_lower"):
                _rate(sum(1 for s in g if s.get("success")), len(g))
            for i, g in sorted(rise_ord.items())}
        # first-rise success by cold-start kind (finer than zero-fall)
        first = rise_ord.get(0, [])
        agg["first_rise_by_start"] = {
            st: _rate(sum(1 for s in first
                          if s.get("start_kind") == st
                          and s.get("success")),
                      sum(1 for s in first if s.get("start_kind") == st))
            for st in ("flat", "bridge", "crouch")
            if any(s.get("start_kind") == st for s in first)}
        agg["walk_gait_valid"] = _rate(gait_ok, walk_n)
        agg["stop_settle_ok"] = _rate(stop_ok, stop_n)
        agg["fall_reasons"] = dict(sorted(falls.items(),
                                          key=lambda kv: -kv[1]))
        agg["medians"] = {k: _med(v) for k, v in walk_ev.items()}
        agg["switch_tilt_deg"] = {"med": _med(tilts),
                                  "max": max(tilts) if tilts else None}
        out.setdefault(cand, {})[mode] = agg
    return out


def cmd_aggregate(args) -> int:
    rows, missing = _flatten(args.cohort, args.cands, _modes(args))
    outdir = OUT_ROOT / args.cohort
    outdir.mkdir(parents=True, exist_ok=True)
    with (outdir / "episodes.jsonl").open("w") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
    agg = {"cohort": args.cohort, "episodes": len(rows),
           "missing_shards": [m["out"] for m in missing],
           "manifest_sha256": manifest_hash(
               plan(args.cohort, args.cands, _modes(args))),
           "results": summarize(rows)}
    (outdir / "aggregate.json").write_text(json.dumps(agg, indent=1))
    print(json.dumps({k: v for k, v in agg.items() if k != "results"},
                     indent=1))
    for cand, per_mode in agg["results"].items():
        for mode, a in per_mode.items():
            print(f"[{cand}/{mode}] n={a['episodes']} "
                  f"zero_fall={a['zero_fall']['k']}/{a['zero_fall']['n']}"
                  f" {a['zero_fall']['ci95']} "
                  f"min_stratum={a.get('min_stratum', {}).get('name')}"
                  f" min_segment={a.get('min_segment', {}).get('name')}"
                  f"({a.get('min_segment', {}).get('rate')})")
    print(f"wrote {outdir/'aggregate.json'} and episodes.jsonl "
          f"({len(rows)} rows); {len(missing)} shard(s) missing")
    return 0 if not missing else 2


def cmd_rerender(args) -> int:
    rows, _ = _flatten(args.cohort, args.cands, _modes(args))
    fails = [r for r in rows if not r["clean"]]
    clean = [r for r in rows if r["clean"]]
    rng = random.Random(args.sample_seed)
    sample = rng.sample(clean, min(args.sample, len(clean)))
    if args.limit:
        fails = fails[:args.limit]
    outdir = OUT_ROOT / args.cohort / "rerender"
    cmds = []
    for tag, group in (("fail", fails), ("ok", sample)):
        for r in group:
            name = f"{r['cand']}_{r['mode']}_s{r['seed']}_ep{r['ep']}"
            sh = {"cand": r["cand"], "mode": r["mode"], "seed": r["seed"],
                  "eps": r["ep"] + 1,
                  "rise_from_h": bool(r.get("rise_from_h", False)),
                  "out": str(outdir / f"{tag}_{name}.json")}
            # per-episode strips subdir: eval_modeseq names the strip
            # modeseq_ep<k>.png, so a SHARED dir would overwrite
            # strips across re-runs with the same episode index
            cmd = shard_cmd(sh) + ["--strip-ep", str(r["ep"]),
                                   "--strips",
                                   str(outdir / "strips" / f"{tag}_{name}")]
            cmds.append((sh, cmd))
    print(f"[rerender] {len(fails)} failures + {len(sample)} clean "
          f"sample = {len(cmds)} re-runs")
    if not args.exec:
        outdir.mkdir(parents=True, exist_ok=True)
        listing = outdir / "commands.txt"
        listing.write_text(
            "\n".join(" ".join(c) for _sh, c in cmds) + "\n")
        print(f"wrote {listing} (use --exec to run here)")
        return 0
    from concurrent.futures import ThreadPoolExecutor

    def run_one(item):
        sh, cmd = item
        if shard_done(sh):
            return True
        Path(sh["out"]).parent.mkdir(parents=True, exist_ok=True)
        with Path(sh["out"]).with_suffix(".log").open("w") as fh:
            subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT,
                           timeout=5400.0)
        return shard_done(sh)

    with ThreadPoolExecutor(max_workers=args.procs) as ex:
        oks = list(ex.map(run_one, cmds))
    print(f"[rerender] {sum(oks)}/{len(cmds)} re-runs done; strips in "
          f"{outdir/'strips'}")
    return 0


def _modes(args):
    return tuple(args.modes.split(",")) if args.modes else MODES


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = dict(cohort="c1", cands=None, modes=None)

    def add_common(p):
        p.add_argument("--cohort", default="c1")
        p.add_argument("--cands", nargs="*", default=None,
                       help=f"subset of {list(CANDIDATES)}")
        p.add_argument("--modes", default=None, help="det,sto subset")

    p = sub.add_parser("plan"); add_common(p)
    p.add_argument("--print", action="store_true", dest="do_print")
    p = sub.add_parser("worker"); add_common(p)
    p.add_argument("--part", default="0/1", help="i/N static partition")
    p.add_argument("--procs", type=int, default=8)
    p = sub.add_parser("aggregate"); add_common(p)
    p = sub.add_parser("rerender"); add_common(p)
    p.add_argument("--sample", type=int, default=12,
                   help="random clean-episode strips to also render")
    p.add_argument("--sample-seed", type=int, default=0)
    p.add_argument("--limit", type=int, default=None,
                   help="cap failure re-runs (default all)")
    p.add_argument("--exec", action="store_true")
    p.add_argument("--procs", type=int, default=4)
    args = ap.parse_args(argv)
    _ = common

    if args.cmd == "plan":
        shards = plan(args.cohort, args.cands, _modes(args))
        h = manifest_hash(shards)
        if args.do_print:
            print(json.dumps(shards, indent=1))
        print(f"{len(shards)} shards x {SHARD_EPS} eps; "
              f"manifest_sha256={h}")
        return 0
    if args.cmd == "worker":
        return cmd_worker(args)
    if args.cmd == "aggregate":
        return cmd_aggregate(args)
    if args.cmd == "rerender":
        return cmd_rerender(args)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
