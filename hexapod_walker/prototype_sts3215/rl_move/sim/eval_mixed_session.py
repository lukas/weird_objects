"""Long mixed-control session gate: ONE policy, env-native sequences.

Operator priority change (2026-08-27, MCP lane 20260827T030823Z): the
campaign's product is ONE integrated policy covering rise-from-down/
unsafe, hold + height adjust, lower, walk (fwd/lateral/reverse), turns,
stops, and mixed joystick-style command changes. Short isolated mode
probes cannot certify that; this instrument evaluates LONG (60 s, and
where feasible 180 s) realistic sessions of mixed commands.

HOW IT WORKS (pure orchestration — no env/reward code, no shared
defaults touched): shells out to the exact-path harness
`eval_checkpoint.py` with the run's own cfg stack PLUS one canonical,
versioned session bundle that turns every episode into an env-native
composed sequence (`goal.mode_seq=1.0`, the same machinery stage-2
policies train on — NO external re-anchoring help; contrast
`eval_modeseq`, whose session driver re-derives goal frames at every
boundary and therefore measures the HELPED/wrapper protocol):

  - segments 8-12 s (clears the >=7 s rise-ramp floor the segfix
    dig-in measured), plan capped to fill the episode;
  - grammar = walk_task.SEQ_NEXT: rise -> {hold|walk} -> {walk|lower}
    -> lower -> rise -> ... so a 60/180 s episode chains stand-ups,
    holds, walks, and sit-downs back to back;
  - hold segments carry height up/down command scripts
    (goal.mode_seq_hold_height_cmd + hold_height_cmd_frac);
  - walk segments carry the full joystick stress_mix command family
    (direction changes, stops, reverses, sweeps — the SAME bundle as
    eval_joystick_gate, resampled every ~4 s);
  - first-segment mode is forced per --modes (default: rise = the
    whole from-down story incl. the run's own flat/bridge/rsi/tipped
    start diet; walk = standing start), so start-kind recovery is in
    the panel by construction;
  - held-out seeds (SEED_BASE 91000; eval_joystick_gate owns 90000,
    bulk_session_eval 900000/910000 — grep before reusing).

Passes: DR-0 det+sto and (if --own-dr-scale given) own-DR det+sto at
--episode-seconds, plus an optional long DR-0 det+sto pass at
--long-seconds. Scores come from eval_checkpoint's per-episode record
plus the additive seq_* fields (seq plan, segment reached counts,
terminating segment mode) it now writes for sequence episodes.

`aggregate_session()` is a pure function of parsed report.json dicts
(unit-tested without the simulator: test_eval_mixed_session.py).

HARD GATE (exit 1): zero terminations of any kind across every pass
(falls, tips, over_current, hold_min_load, ... — a session that needs
a termination is not a deployable session). Everything else is
REPORTED always and gated only with --strict:
  - session completion (every planned segment reached, no term)
    >= 0.9;
  - walk-segment gait validity fraction >= 0.9 (no sacrificed legs);
  - median walk slip/m <= 2.9 (the joystick teacher band);
  - median walk direction error <= 40 deg;
  - median |height err| at episode end <= 15 mm.

    cd prototype_sts3215 && uv run python -m rl_move.sim.eval_mixed_session \
        rl_move/sim/policies/<ckpt>.zip --own-dr-scale 0.5 \
        --extra-cfg-set env.model_source=mesh ... \
        [--episode-seconds 60] [--long-seconds 180] [--n 6] \
        [--modes rise walk] [--video] [--out-dir logs/ckpt_eval/X]

Exit 0 = hard gate passed, 1 = failed. ops.sh `sessioncmd <run>`
prints a ready-to-run invocation carrying the run's own cfg stack.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
import time
from pathlib import Path

_PROTO = Path(__file__).resolve().parents[2]

# Canonical session bundle. Versioned here so every candidate is
# measured against provably the SAME session distribution.
MIXED_SESSION_CFG = [
    "goal.mode_seq=1.0",                 # every episode a sequence
    "goal.mode_seq_segment_s_min=8.0",   # >= the 7 s rise-ramp floor
    "goal.mode_seq_segment_s_max=12.0",
    "goal.mode_seq_hold_height_cmd=1.0",  # height up/down in holds
    "goal.hold_height_cmd_frac=1.0",
    "goal.walk_cmd_mode=stress_mix",     # joystick command family
    "goal.walk_cmd_resample_s=4.0",
    "goal.walk_cmd_resample_jitter=0.5",
]

SEED_BASE_DEFAULT = 91000   # held out (90000=joystick gate,
                            # 900000/910000=bulk_session_eval)
SLIP_CAP_DEFAULT = 2.9      # joystick teacher band ceiling
DIR_ERR_CAP_DEFAULT = 40.0  # teacher tick-sway floor (35) + margin
HEIGHT_ERR_CAP_MM = 15.0    # rise/lower success bar in eval_checkpoint
COMPLETE_FRAC_MIN = 0.9
GAIT_VALID_FRAC_MIN = 0.9


def _seg_cap(episode_seconds: float) -> int:
    # Enough plan capacity to fill the episode at the min segment
    # length, +1 spare; the sampler stops early when the tail is <3 s.
    return int(math.ceil(episode_seconds / 8.0)) + 1


def _run_eval_checkpoint(ckpt: Path, *, task: str, dr_scale: float,
                         seed: int, n: int, episode_seconds: float,
                         modes: list[str], extra_cfg: list[str],
                         out_dir: Path, log_path: Path,
                         video: bool = False) -> Path:
    """One eval_checkpoint pass (det+sto via --stochastic)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, "-m", "rl_move.sim.eval_checkpoint",
           str(ckpt), "--task", task, "--modes", *modes,
           "--per-mode", str(n), "--dr-scale", str(dr_scale),
           "--seed", str(seed), "--stochastic",
           "--episode-seconds", str(episode_seconds),
           "--no-wandb", "--out", str(out_dir)]
    cmd += ["--video-every", "1"] if video else ["--no-video"]
    for kv in (list(extra_cfg) + MIXED_SESSION_CFG
               + [f"goal.mode_seq_max_segments={_seg_cap(episode_seconds)}"]):
        cmd += ["--cfg-set", kv]
    with open(log_path, "w") as fh:
        rc = subprocess.run(cmd, cwd=str(_PROTO), stdout=fh,
                            stderr=subprocess.STDOUT).returncode
    report = out_dir / "report.json"
    if not report.exists():
        tail = log_path.read_text()[-2000:] if log_path.exists() else ""
        raise RuntimeError(
            f"eval_checkpoint produced no report.json (rc={rc}) at "
            f"{out_dir}; log tail:\n{tail}")
    return report


def _episodes(report: dict):
    for label, eps in (report.get("episodes") or {}).items():
        for ep in eps:
            yield label, ep


def _med(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.median(vals), 3) if vals else None


def aggregate_session(reports: dict[str, dict], *,
                      slip_cap: float = SLIP_CAP_DEFAULT,
                      dir_err_cap: float = DIR_ERR_CAP_DEFAULT,
                      height_err_cap: float = HEIGHT_ERR_CAP_MM,
                      strict: bool = False) -> dict:
    """Pure function: {pass_name: parsed report.json} -> scorecard."""
    n = n_term = n_complete = n_seq = 0
    seg_planned = seg_reached = 0
    term_reasons: dict[str, int] = {}
    term_by_seg: dict[str, int] = {}
    term_by_start: dict[str, int] = {}
    dir_errs, vel_errs, slips, slips_total, prog = [], [], [], [], []
    gait_flags, sac_legs = [], set()
    h_errs, track_errs = [], []
    cur_max, cur_p95, cur_soft = [], [], []
    per_pass: dict[str, dict] = {}
    for pname, rep in reports.items():
        p_n = p_term = 0
        for label, ep in _episodes(rep):
            n += 1
            p_n += 1
            if ep.get("terminated"):
                n_term += 1
                p_term += 1
                r = str(ep.get("term_reason") or "?")
                term_reasons[r] = term_reasons.get(r, 0) + 1
                seg = str(ep.get("seq_end_seg_mode")
                          or label.split("/")[0])
                term_by_seg[seg] = term_by_seg.get(seg, 0) + 1
                sk = str(ep.get("start_kind") or "?")
                term_by_start[sk] = term_by_start.get(sk, 0) + 1
            if "seq_n_segments_planned" in ep:
                n_seq += 1
                seg_planned += int(ep["seq_n_segments_planned"])
                seg_reached += int(ep["seq_n_segments_reached"])
                if ep.get("seq_completed"):
                    n_complete += 1
            if ep.get("direction_err_mean_deg") is not None:
                dir_errs.append(float(ep["direction_err_mean_deg"]))
            if ep.get("vel_err_mean") is not None:
                vel_errs.append(float(ep["vel_err_mean"]))
            if ep.get("slip_per_m") is not None:
                slips.append(float(ep["slip_per_m"]))
            if ep.get("slip_m_total") is not None:
                slips_total.append(float(ep["slip_m_total"]))
            if ep.get("progress_ratio") is not None:
                prog.append(float(ep["progress_ratio"]))
            if "gait_valid" in ep:
                gait_flags.append(bool(ep["gait_valid"]))
                sac_legs.update(ep.get("sacrificed_legs") or [])
            if ep.get("height_err_end_mm") is not None:
                h_errs.append(abs(float(ep["height_err_end_mm"])))
            if ep.get("track_err_mean_deg") is not None:
                track_errs.append(float(ep["track_err_mean_deg"]))
            if ep.get("cur_max_a") is not None:
                cur_max.append(float(ep["cur_max_a"]))
            if ep.get("cur_p95_a") is not None:
                cur_p95.append(float(ep["cur_p95_a"]))
            if ep.get("cur_s_above_soft") is not None:
                cur_soft.append(float(ep["cur_s_above_soft"]))
        per_pass[pname] = {"n": p_n, "terms": p_term}
    complete_frac = (round(n_complete / n_seq, 3) if n_seq else None)
    seg_frac = (round(seg_reached / seg_planned, 3) if seg_planned
                else None)
    gait_frac = (round(sum(gait_flags) / len(gait_flags), 3)
                 if gait_flags else None)
    out = {
        "n_episodes": n,
        "n_sequence_episodes": n_seq,
        "n_terminations": n_term,
        "zero_falls": n_term == 0,
        "term_reasons": term_reasons,
        "terms_by_segment_mode": term_by_seg,
        "terms_by_start_kind": term_by_start,
        "session_complete_frac": complete_frac,
        "segments_reached_frac": seg_frac,
        "walk": {
            "n_with_walk_metrics": len(dir_errs),
            "direction_err_med_deg": _med(dir_errs),
            "vel_err_med": _med(vel_errs),
            "slip_per_m_med": _med(slips),
            "slip_m_total_med": _med(slips_total),
            "progress_ratio_med": _med(prog),
            "gait_valid_frac": gait_frac,
            "sacrificed_legs_seen": sorted(sac_legs),
        },
        "height_err_end_med_mm": _med(h_errs),
        "track_err_med_deg": _med(track_errs),
        "current": {
            "cur_max_a_max": (round(max(cur_max), 2) if cur_max
                              else None),
            "cur_p95_a_med": _med(cur_p95),
            "s_above_soft_med": _med(cur_soft),
        },
        "per_pass": per_pass,
    }
    hard = n_term == 0
    soft = {
        "complete_frac_ok": (complete_frac is not None
                             and complete_frac >= COMPLETE_FRAC_MIN),
        "gait_valid_ok": (gait_frac is not None
                          and gait_frac >= GAIT_VALID_FRAC_MIN),
        "slip_ok": (out["walk"]["slip_per_m_med"] is not None
                    and out["walk"]["slip_per_m_med"] <= slip_cap),
        "dir_err_ok": (out["walk"]["direction_err_med_deg"] is not None
                       and out["walk"]["direction_err_med_deg"]
                       <= dir_err_cap),
        "height_ok": (out["height_err_end_med_mm"] is not None
                      and out["height_err_end_med_mm"]
                      <= height_err_cap),
    }
    out["gate"] = {"zero_falls": hard, "soft": soft,
                   "strict": strict,
                   "pass": hard and (all(soft.values())
                                     if strict else True)}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Long mixed-control session gate (single policy, "
                    "env-native mode_seq sessions)")
    ap.add_argument("ckpt", type=Path)
    ap.add_argument("--task", default="joint_walk")
    ap.add_argument("--own-dr-scale", type=float, default=None,
                    help="the checkpoint's own training DR scale; "
                         "omit to run DR-0 only")
    ap.add_argument("--n", type=int, default=6,
                    help="episodes per first-segment mode per pass")
    ap.add_argument("--episode-seconds", type=float, default=60.0)
    ap.add_argument("--long-seconds", type=float, default=0.0,
                    help="if >0, run an extra DR-0 det+sto pass at "
                         "this duration (operator 08-27: 180 where "
                         "feasible)")
    ap.add_argument("--long-n", type=int, default=3)
    ap.add_argument("--modes", nargs="*", default=["rise", "walk"],
                    help="forced FIRST segment modes (rise = session "
                         "from down incl. unsafe starts; walk = "
                         "standing start)")
    ap.add_argument("--seed-base", type=int, default=SEED_BASE_DEFAULT)
    ap.add_argument("--extra-cfg-set", action="append", default=[],
                    help="the run's own cfg stack (ops.sh sessioncmd "
                         "prints it)")
    ap.add_argument("--slip-cap", type=float, default=SLIP_CAP_DEFAULT)
    ap.add_argument("--dir-err-cap", type=float,
                    default=DIR_ERR_CAP_DEFAULT)
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()

    stem = args.ckpt.stem.replace("ppo_goal_", "")
    out_root = args.out_dir or (_PROTO / "logs" / "ckpt_eval"
                                / f"{stem}_mixedsession")
    out_root.mkdir(parents=True, exist_ok=True)

    passes: list[tuple[str, float, float, int]] = [
        ("dr0", 0.0, args.episode_seconds, args.n)]
    if args.own_dr_scale is not None:
        passes.append(("owndr", args.own_dr_scale,
                       args.episode_seconds, args.n))
    if args.long_seconds > 0:
        passes.append(("dr0_long", 0.0, args.long_seconds,
                       args.long_n))

    reports: dict[str, dict] = {}
    t0 = time.time()
    for pname, dr, secs, n in passes:
        rp = _run_eval_checkpoint(
            args.ckpt, task=args.task, dr_scale=dr,
            seed=args.seed_base, n=n, episode_seconds=secs,
            modes=list(args.modes), extra_cfg=list(args.extra_cfg_set),
            out_dir=out_root / pname,
            log_path=out_root / f"{pname}.log", video=args.video)
        reports[pname] = json.loads(rp.read_text())
        print(f"[mixed-session] pass {pname} done "
              f"({time.time() - t0:.0f}s elapsed)", flush=True)

    verdict = aggregate_session(
        reports, slip_cap=args.slip_cap, dir_err_cap=args.dir_err_cap,
        strict=args.strict)
    verdict["checkpoint"] = str(args.ckpt)
    verdict["passes"] = [p[0] for p in passes]
    verdict["episode_seconds"] = args.episode_seconds
    verdict["long_seconds"] = args.long_seconds
    (out_root / "session_verdict.json").write_text(
        json.dumps(verdict, indent=1))
    print(json.dumps(verdict, indent=1))
    print(f"[mixed-session] verdict written to "
          f"{out_root / 'session_verdict.json'}")
    return 0 if verdict["gate"]["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
