"""Joystick-track DONE-gate harness (STATUS Next item 2, built 08-22).

Implements the operator's pre-registered `joystick` DONE gate
(CURRENT_TRUTHS.md / rl_docs/tracks/joystick/STATUS.md, 08-21 binding):

    "one policy (or the session-controller stack) follows a
    randomized 60-second joystick command script in MuJoCo --
    direction changes, stops, reverses, turns -- with:
      (a) ZERO falls across the full panel (n>=12 episodes, det+sto,
          DR-0 and the run's own DR, held-out command seeds);
      (b) directions actually followed (judged as a DELTA against the
          teacher clone's measured ~35 deg tick-level stride-sway
          floor -- compare deltas, not raw values);
      (c) slip/m no worse than the scripted teacher's measured band
          at the calibrated plant (<= ~2.9; teacher band 1.4-2.9)."

Pure orchestration -- no env/reward code touched. Shells out to the
existing `eval_checkpoint.py` harness (which already tracks
direction_err_mean_deg / slip_per_m / termination per episode) with
ONE canonical, versioned, held-out command bundle:

    goal.walk_cmd_mode=stress_mix           (full family: random_hold,
                                              flip_180 [reverse],
                                              sweep_circle/square
                                              [turns], stop_go, jitter
                                              -- this IS the default
                                              family set when
                                              goal.walk_cmd_stage is
                                              left at -1)
    goal.walk_cmd_resample_s=4.0            (a new command every ~4s)
    goal.walk_cmd_resample_jitter=0.5       (irregular intervals,
                                              2-6s, not a metronome)
    --episode-seconds 60                    (the gate's own 60s)
    --seed-base 90000                       (a seed range no training
                                              run or other harness
                                              uses -- genuinely
                                              held-out draws)

run TWICE: once at --dr-scale 0.0 (the DR-0 half of the gate) and
once at the checkpoint's OWN training dr-scale (the own-DR half),
each with BOTH a det and a sto pass (`eval_checkpoint --stochastic`).
The DONE-gate verdict is computed over ALL FOUR passes combined:
zero falls in every one of them, median slip/m across every episode
<= --slip-cap, and median direction error across every episode within
--dir-err-margin-deg of --teacher-dir-err-deg (the teacher clone's own
measured tick-sway floor, CURRENT_TRUTHS.md).

    cd prototype_sts3215 && uv run python -m rl_move.sim.eval_joystick_gate \
        rl_move/sim/policies/<ckpt>.zip --own-dr-scale 0.35 \
        --extra-cfg-set goal.walk_phase_obs=1 \
        --extra-cfg-set goal.walk_phase_hz=1.333333 \
        [--n 12] [--episode-seconds 60] [--out-dir logs/ckpt_eval/X]

Exit 0 = PASS, 1 = FAIL. Prints the per-axis numbers and which axis
missed. `aggregate_gate()` is a pure function of parsed report.json
episode lists -- unit-tested without running the simulator
(test_eval_joystick_gate.py).
"""
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

_PROTO = Path(__file__).resolve().parents[2]

# The canonical held-out joystick-session command bundle. Versioned
# here (not re-typed at every call site) so every DONE-gate reading
# is provably the SAME script across candidates.
JOYSTICK_SESSION_CFG = [
    "goal.walk_cmd_mode=stress_mix",
    "goal.walk_cmd_resample_s=4.0",
    "goal.walk_cmd_resample_jitter=0.5",
]

SEED_BASE_DEFAULT = 90000       # held out: no training/eval harness
                                 # elsewhere in the repo uses seeds
                                 # this high (grep before changing).
TEACHER_DIR_ERR_DEG_DEFAULT = 35.0   # measured tick-sway floor,
                                      # CURRENT_TRUTHS.md 08-22.
DIR_ERR_MARGIN_DEG_DEFAULT = 5.0     # same margin the phasedir gates
                                      # use for clone-relative dir_err.
SLIP_CAP_DEFAULT = 2.9               # teacher's own measured hardware
                                      # band ceiling (1.4-2.9 m/m).


def _run_eval_checkpoint(ckpt: Path, *, task: str, dr_scale: float,
                          seed: int, n: int, episode_seconds: float,
                          extra_cfg: list[str], out_dir: Path,
                          log_path: Path, video: bool = False) -> Path:
    """Shell out to eval_checkpoint.py for one (dr_scale) pass.

    Returns the report.json path. Raises on non-zero-but-not-a-known
    exit (eval_checkpoint always writes report.json before exiting,
    even when episodes fail the harness's own success criteria, so we
    only treat a MISSING report.json as a real error).

    `video` (08-22 follow-up, default False = prior behavior
    unchanged): pass through to eval_checkpoint instead of the
    hardcoded --no-video, for candidates close enough to the gate to
    need visual triage of the actual 60s session.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, "-m", "rl_move.sim.eval_checkpoint", str(ckpt),
           "--task", task, "--modes", "walk", "--per-mode", str(n),
           "--dr-scale", str(dr_scale), "--seed", str(seed),
           "--stochastic", "--episode-seconds", str(episode_seconds),
           "--no-wandb", "--out", str(out_dir)]
    cmd += ["--video-every", "1"] if video else ["--no-video"]
    for kv in JOYSTICK_SESSION_CFG + list(extra_cfg):
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


def _episodes(report: dict) -> list[dict]:
    eps = []
    for key, lst in report.get("episodes", {}).items():
        mode = key.split("/")[0]
        if mode == "walk":
            eps.extend(lst)
    return eps


def aggregate_gate(reports: dict[str, dict], *,
                    slip_cap: float = SLIP_CAP_DEFAULT,
                    teacher_dir_err_deg: float = TEACHER_DIR_ERR_DEG_DEFAULT,
                    dir_err_margin_deg: float = DIR_ERR_MARGIN_DEG_DEFAULT,
                    ) -> dict:
    """Pure aggregation: parsed report.json dicts -> DONE-gate verdict.

    `reports` keys are free-form labels (e.g. "dr0/det" is NOT a
    thing -- eval_checkpoint bundles det+sto into ONE report per
    dr_scale under episodes["walk/det"]/["walk/sto"]); this function
    just needs one dict per dr_scale pass (report["dr_scale"] read
    back out of each report for labeling).
    """
    all_eps: list[tuple[str, dict]] = []
    per_pass: dict[str, list[dict]] = {}
    for label, report in reports.items():
        eps = _episodes(report)
        per_pass[label] = eps
        all_eps.extend((label, e) for e in eps)
    if not all_eps:
        return {"pass": False, "reason": "no episodes parsed", "n": 0}

    n_total = len(all_eps)
    falls = [(lbl, e) for lbl, e in all_eps if e.get("terminated")]
    slips = [e["slip_per_m"] for _l, e in all_eps
             if e.get("slip_per_m") is not None]
    dirs = [e["direction_err_mean_deg"] for _l, e in all_eps
            if "direction_err_mean_deg" in e]
    gait_valid = [bool(e.get("gait_valid")) for _l, e in all_eps]

    slip_med = statistics.median(slips) if slips else float("nan")
    dir_med = statistics.median(dirs) if dirs else float("nan")
    dir_allow = teacher_dir_err_deg + dir_err_margin_deg

    checks = {
        "zero_falls": len(falls) == 0,
        "slip_ok": bool(slips) and slip_med <= slip_cap,
        "dir_ok": bool(dirs) and dir_med <= dir_allow,
        "gait_valid_all": all(gait_valid) if gait_valid else False,
    }
    passed = all(checks.values())
    per_pass_summary = {}
    for label, eps in per_pass.items():
        pf = [e for e in eps if e.get("terminated")]
        s = [e["slip_per_m"] for e in eps if e.get("slip_per_m") is not None]
        d = [e["direction_err_mean_deg"] for e in eps
             if "direction_err_mean_deg" in e]
        per_pass_summary[label] = {
            "n": len(eps),
            "falls": len(pf),
            "slip_med": round(statistics.median(s), 3) if s else None,
            "dir_err_med": round(statistics.median(d), 2) if d else None,
        }

    # Per-leg gait metrics (08-22 follow-up): eval_checkpoint already
    # reports duty_cycle/swing_count per episode as 6-element lists
    # and sacrificed_legs as an index list; the gate script itself
    # never summarized these across the panel. A walk without all six
    # feet cycling contact/swing is not walking (ORCHESTRATOR_PROMPT
    # judgment notes) -- this makes that check part of the gate's own
    # printed record instead of a manual per-run video read every
    # time. Pure aggregation, additive: does not affect `pass`.
    per_leg = None
    duty_lists = [e["duty_cycle"] for _l, e in all_eps
                  if isinstance(e.get("duty_cycle"), (list, tuple))
                  and len(e["duty_cycle"]) == 6]
    swing_lists = [e["swing_count"] for _l, e in all_eps
                   if isinstance(e.get("swing_count"), (list, tuple))
                   and len(e["swing_count"]) == 6]
    if duty_lists or swing_lists:
        sac_counts = [0] * 6
        for _l, e in all_eps:
            for leg in e.get("sacrificed_legs") or []:
                if 0 <= leg < 6:
                    sac_counts[leg] += 1
        per_leg = {
            "duty_median": [
                round(statistics.median(vals), 3)
                for vals in zip(*duty_lists)] if duty_lists else None,
            "swing_count_median": [
                round(statistics.median(vals), 1)
                for vals in zip(*swing_lists)] if swing_lists else None,
            "sacrificed_episode_count": sac_counts,
            "sacrificed_frac": [round(c / n_total, 3) for c in sac_counts],
        }

    return {
        "pass": passed,
        "n_total": n_total,
        "falls_total": len(falls),
        "fall_labels": [lbl for lbl, _e in falls],
        "slip_per_m_med": round(slip_med, 3) if slips else None,
        "slip_cap": slip_cap,
        "direction_err_med_deg": round(dir_med, 2) if dirs else None,
        "direction_err_allow_deg": dir_allow,
        "gait_valid_frac": (sum(gait_valid) / len(gait_valid)
                           if gait_valid else None),
        "checks": checks,
        "per_pass": per_pass_summary,
        "per_leg": per_leg,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--task", default="joint_walk")
    ap.add_argument("--own-dr-scale", type=float, default=0.0,
                    help="the checkpoint's own training dr-scale "
                         "(0.0 = DR-0-only checkpoint, own-DR pass "
                         "skipped as redundant)")
    ap.add_argument("--n", type=int, default=12,
                    help="episodes per (dr_scale x det/sto) pass; "
                         "gate spec wants n>=12")
    ap.add_argument("--episode-seconds", type=float, default=60.0)
    ap.add_argument("--seed-base", type=int, default=SEED_BASE_DEFAULT)
    ap.add_argument("--slip-cap", type=float, default=SLIP_CAP_DEFAULT)
    ap.add_argument("--teacher-dir-err-deg", type=float,
                    default=TEACHER_DIR_ERR_DEG_DEFAULT)
    ap.add_argument("--dir-err-margin-deg", type=float,
                    default=DIR_ERR_MARGIN_DEG_DEFAULT)
    ap.add_argument("--extra-cfg-set", action="append", default=[],
                    help="checkpoint's own obs-shaping cfg-sets "
                         "(e.g. goal.walk_phase_obs=1) -- repeatable; "
                         "REQUIRED for any checkpoint trained with a "
                         "non-default obs width")
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--video", action="store_true",
                    help="pass --video-every 1 through to "
                         "eval_checkpoint instead of --no-video "
                         "(default off, matches prior behavior; "
                         "turn on for visual triage of a near-gate "
                         "candidate)")
    args = ap.parse_args()

    out_dir = args.out_dir or (
        _PROTO / "logs" / "ckpt_eval" /
        f"{args.checkpoint.stem}_joystick_gate")
    out_dir.mkdir(parents=True, exist_ok=True)

    dr_scales = [0.0]
    if args.own_dr_scale > 0.0:
        dr_scales.append(args.own_dr_scale)

    reports: dict[str, dict] = {}
    t0 = time.time()
    for dr in dr_scales:
        label = f"dr{dr:g}".replace(".", "p")
        pass_dir = out_dir / label
        log_path = out_dir / f"{label}.log"
        print(f"[eval_joystick_gate] running {label} "
              f"(n={args.n} x2 det/sto, {args.episode_seconds}s "
              f"episodes, seed_base={args.seed_base}) ...")
        report_path = _run_eval_checkpoint(
            args.checkpoint, task=args.task, dr_scale=dr,
            seed=args.seed_base, n=args.n,
            episode_seconds=args.episode_seconds,
            extra_cfg=args.extra_cfg_set, out_dir=pass_dir,
            log_path=log_path, video=args.video)
        reports[label] = json.loads(report_path.read_text())
        print(f"[eval_joystick_gate] {label} done "
              f"({time.time() - t0:.0f}s elapsed)")

    verdict = aggregate_gate(reports, slip_cap=args.slip_cap,
                              teacher_dir_err_deg=args.teacher_dir_err_deg,
                              dir_err_margin_deg=args.dir_err_margin_deg)
    verdict["checkpoint"] = str(args.checkpoint)
    verdict["dr_scales"] = dr_scales
    verdict["episode_seconds"] = args.episode_seconds
    (out_dir / "gate_verdict.json").write_text(json.dumps(verdict, indent=2))

    print()
    print(f"== JOYSTICK DONE-GATE: {args.checkpoint.name} ==")
    for label, s in verdict["per_pass"].items():
        print(f"  {label}: n={s['n']} falls={s['falls']} "
              f"slip/m med={s['slip_med']} dir_err med={s['dir_err_med']}deg")
    print(f"  TOTAL n={verdict['n_total']} falls={verdict['falls_total']} "
          f"slip/m med={verdict['slip_per_m_med']} (cap {args.slip_cap}) "
          f"dir_err med={verdict['direction_err_med_deg']}deg "
          f"(allow <= {verdict['direction_err_allow_deg']}) "
          f"gait_valid_frac={verdict['gait_valid_frac']}")
    print(f"  checks: {verdict['checks']}")
    if verdict.get("per_leg"):
        pl = verdict["per_leg"]
        print(f"  per-leg duty_median: {pl['duty_median']}")
        print(f"  per-leg swing_count_median: {pl['swing_count_median']}")
        print(f"  per-leg sacrificed_frac: {pl['sacrificed_frac']}")
    print(f"  ==> {'PASS' if verdict['pass'] else 'FAIL'}")
    print(f"[eval_joystick_gate] verdict -> {out_dir / 'gate_verdict.json'}")
    sys.exit(0 if verdict["pass"] else 1)


if __name__ == "__main__":
    main()
