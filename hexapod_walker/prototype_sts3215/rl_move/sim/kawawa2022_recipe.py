"""Kawawa-Beaudan 2022 prior-free walking recipe — walkcurr track rung 1.

This is a local MJX/Warp adaptation of:

    Learning to Walk: Legged Hexapod Locomotion from Simulation to the
    Real World.  Maxime Kawawa-Beaudan, Avideh Zakhor, UC Berkeley
    EECS technical report UCB/EECS-2022-146, 2022.

The public project page advertises code, but the linked GitHub repo was
unavailable when this recipe was first written (2026-08-23), so this
file records the launch arguments explicitly instead of relying on
memory.

HISTORY / LESSON (binding for this track): the first adaptation,
``cw-kawawa2022-pf-flat1`` (2026-08-23, W&B m9gqkl5i, verdict FAIL in
the ledger), was launched from a desktop clone that never pushed its
code (temp commit b126ceb3 is unreachable; the RecurrentPPO/LSTM
trainer support it carried was overwritten on the pod and is LOST).
Its walk sub-goal fell over (tilt_pitch) at EVERY periodic eval from
step 8M to 40M while aggregate training reward kept rising — because
the run trained on the full multi-goal joint_walk diet and the
hold/raise/track/unload sub-goals carried the reward (hold 352 /
raise 980 vs walk 55).  This module is the canonical re-registration
of that recipe with the failure designed out:

  1. WALK-ONLY diet (``goal.walk_pure=1``): every episode is a walk
     episode; no other sub-goal exists to carry reward.
  2. Fixed forward command first (0.05-0.06 m/s, heading 0, no
     resampling).  Heading sets / irregular direction changes are
     LATER rungs, unlocked only by a rung-1 pass (see RUNG_LADDER).
  3. ``reward.term_penalty=1200``: the original stack left dying free;
     the walk goal tilt-terminated everywhere.  Sized by the 08-23
     WALKCURR_PF bank calibration in test_task_semantics.py (dying
     must sit strictly below even the wrong-way honest gaits).
  4. The reward ranking (clean commanded walking > park/stall >
     sideways/reverse/wrong-way > high-slip/skate/fall) is PROVEN by
     the WALKCURR_PF bank in rl_move/tests/test_task_semantics.py
     under this exact cfg before any launch.
  5. Binding triage rule: every triage compares training-reward trend
     against the walk-eval trend.  Reward rising while walk eval is
     flat/down or walk terminates = MISALIGNED; stop same-recipe
     seeds/continuations and audit reward/eval/simulator.

Architecture note: the paper uses LSTM(64) + ELU 128/64/32 heads with
24-step PPO rollouts.  The lost desktop trainer wired sb3-contrib
RecurrentPPO for that; rung 1 deliberately runs the memoryless MLP
128/64/32 + ELU + 24-step rollouts instead (fixed-command flat-ground
walking needs no memory, and the canonical eval/canary/video stack is
PPO-native).  If a later rung needs memory, the in-repo recurrent path
is ``--gru --gru-hidden-size 64`` (sb3-contrib RecurrentPPO with the
GRU policy), not a re-build of the lost LSTM code.  Because rung 1 is
memoryless it KEEPS the body-velocity observation (the paper's
LSTM-era proprioception-only obs would make a memoryless policy blind
to its own speed).
"""
from __future__ import annotations

import argparse
import shlex
from dataclasses import dataclass


PAPER_PAGE = (
    "https://www2.eecs.berkeley.edu/Pubs/TechRpts/2022/"
    "EECS-2022-146.html"
)
PAPER_PDF = (
    "https://www2.eecs.berkeley.edu/Pubs/TechRpts/2022/Archive/"
    "EECS-2022-146.pdf"
)
PROJECT_PAGE = "https://sites.google.com/view/hexapod-rl"
PUBLIC_CODE_LINK = "https://github.com/maximejkb/hexapod-isaac.git"


DEFAULT_RUN = "cw-walkcurr-pf-fwd1"
DEFAULT_STEPS = 2_000_000          # discovery rung (guardrails cap)
DEFAULT_PHASE = "discovery"
DEFAULT_TRACK = "walkcurr"


# The curriculum this track climbs (operator focus note
# 20260823T154657Z).  Each rung is its own pre-registered run gated on
# the previous rung's PASS; speed obedience is secondary throughout.
RUNG_LADDER = (
    "rung1 fixed forward 0.05-0.06 m/s, heading 0, DR0 (this file)",
    "rung2 small heading set (± up to ~15 deg), still one command/ep",
    "rung3 full fixed headings",
    "rung4 irregular direction changes (resampling mid-episode)",
    "rung5+ DR/pushes hardening (paper's friction 0.5-1.25 + pushes)",
)


HYPOTHESIS = (
    "Can a prior-free from-scratch PPO policy (no gait clock, no BC "
    "teacher, no motion prior) learn plain forward walking when the "
    "reward diet is walking and nothing else? Kawawa-Beaudan 2022 "
    "recipe re-run with the multi-goal reward-carry failure of "
    "cw-kawawa2022-pf-flat1 designed out: walk-pure episode diet, "
    "fixed forward command 0.05-0.06 m/s, ELU 128/64/32 MLP with "
    "24-step rollouts, loaded actuator calibration, term_penalty "
    "closing the suicide exploit, and the WALKCURR_PF semantics bank "
    "proving the reward ranking before launch. Prediction-if-true: "
    "real six-leg stepping with along-command progress emerges within "
    "2M steps (the walk goal of the failed run survived its 1M/5M "
    "evals even on a 20-30%% walk diet). Prediction-if-false: statue/"
    "park or tilt terminations with flat-or-falling walk eval — which "
    "per the binding walkcurr triage rule stops same-recipe seeds and "
    "forces a reward/eval/simulator audit, not a continuation."
)

GATE = (
    "Discovery gate at 2M: C-env deterministic fixed-forward panel "
    "(n>=6) — walk SURVIVES (zero tilt terminations), along-command "
    "progress cmd_prog_frac >= 0.35, direction_err_deg <= 30, slip/m "
    "<= 3.0, all six legs cycling contact/swing on >=4/6 episodes, "
    "and video showing real stepping (not tapping/skating/parking). "
    "BINDING TRIAGE RULE: read reward trend AND walk-eval trend "
    "together; reward rising with walk eval flat/down or terminating "
    "= misaligned -> STOP same-recipe seeds/continuations and audit "
    "reward/eval/sim. Reward AND eval both rising at 2M with the gate "
    "not yet met = continue per the 08-21 ruling."
)

EVIDENCE = (
    "cw-kawawa2022-pf-flat1 (W&B m9gqkl5i, FAIL): walk sub-goal "
    "survived its 1M/5M evals then was starved/broken by the "
    "multi-goal income mix — the recipe learns SOMETHING about "
    "walking early; the diet and the open suicide exploit were the "
    "named killers. WALKCURR_PF bank in test_task_semantics.py "
    "proves the reward ranking under this exact cfg."
)


CFG_SET = (
    "bus.servo_params=loaded",
    # WALK-ONLY diet: the named root cause of the flat1 FAIL was
    # hold/raise/track/unload carrying aggregate reward. walk_pure
    # zeroes every other goal probability at env construction.
    "goal.walk_pure=1",
    # Fixed forward command, rung 1: narrow band, no heading, no
    # resampling. The command is a direction, not a speed target
    # (freeprog caps useful progress; exceeding is never punished).
    "goal.walk_speed_min_m_s=0.05",
    "goal.walk_speed_max_m_s=0.06",
    "goal.walk_heading_max_rad=0.0",
    "goal.walk_cmd_resample_s=0.0",
    "goal.walk_cmd_metrics=1",
    # Direction-first income on stride-EMA velocity (the validated
    # anti-noise-cancellation form) at the BANK-CALIBRATED v2e doses
    # (test_task_semantics.py WALKCURR_PF_OVERRIDES, measured 08-23:
    # gait +346 > stall -31 > park -352 > sideways -609 > reverse
    # -741 > skate -1058 > topple -1164 — the operator's required
    # ranking exactly). The RAW kawawa launch doses were bank-REFUTED
    # (park out-earned walking +387 vs +325); the harsh SLIPWALK doses
    # are equally refuted for from-scratch discovery (8 statue arms on
    # the amp track). These sit in between: real travel is the only
    # positive-income behavior, refusal is separated from marching by
    # the park-duty charge, wrong-way honest gaits stay above the
    # slip/fall class, and dying is strictly the worst outcome.
    "reward.k_walk_freeprog=3.0",
    "reward.walk_freeprog_cap_m_s=0.08",
    "reward.walk_kernel_vel_ema=1.0",
    "reward.walk_kernel_vel_tau_s=0.75",
    "reward.k_walk_heading=0.5",
    "reward.k_step_event=1.0",
    "reward.k_park_duty=4.0",
    "reward.k_walk_idle_charge=2.0",
    "reward.walk_idle_speed_m_s=0.025",
    "reward.walk_idle_tau_s=1.0",
    "reward.walk_loadslip_gate=0.75",
    "reward.loadslip_ok=1.2",
    "reward.loadslip_max=3.0",
    "reward.loadslip_floor_m=0.03",
    "reward.k_loadslip_excess=4.5",
    "reward.term_penalty=1200",
)


TRAINER_ARGS = (
    "--task", "joint_walk",
    "--episode-seconds", "25",
    # Paper-shaped optimizer/policy: 24-step rollouts, 128/64/32 ELU
    # heads. batch 24576 = (4096 envs x 24 steps) / 4 exactly.
    "--n-steps", "24",
    "--batch-size", "24576",
    "--net-arch", "128,64,32",
    "--activation-fn", "elu",
    "--log-std-init", "-1.0",
    # Rung 1 is flat nominal physics: DR/pushes are rung 5+ hardening.
    "--dr-scale", "0.0",
    "--require-gpu-physics",
    "--eval-every", "1000000",
    "--video-every", "2000000",
    "--best-ckpt",
)


@dataclass(frozen=True)
class LaunchSpec:
    run: str = DEFAULT_RUN
    steps: int = DEFAULT_STEPS
    phase: str = DEFAULT_PHASE
    track: str = DEFAULT_TRACK
    hypothesis: str = HYPOTHESIS
    gate: str = GATE
    evidence: str = EVIDENCE


def trainer_args() -> list[str]:
    args = list(TRAINER_ARGS)
    for item in CFG_SET:
        args.extend(["--cfg-set", item])
    return args


def backlog_command(spec: LaunchSpec = LaunchSpec()) -> list[str]:
    return [
        "python", "launch_run.py", "backlog", "add",
        "--run", spec.run,
        "--steps", str(spec.steps),
        "--phase", spec.phase,
        "--track", spec.track,
        "--evidence", spec.evidence,
        "--hypothesis", spec.hypothesis,
        "--gate", spec.gate,
        "--",
        *trainer_args(),
    ]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", default=DEFAULT_RUN)
    ap.add_argument("--steps", type=int, default=DEFAULT_STEPS)
    ap.add_argument("--phase", default=DEFAULT_PHASE)
    ap.add_argument("--track", default=DEFAULT_TRACK)
    ap.add_argument("--trainer-args", action="store_true",
                    help="print only the train_ppo_mjx passthrough args")
    args = ap.parse_args(argv)
    if args.trainer_args:
        print(shlex.join(trainer_args()))
    else:
        spec = LaunchSpec(run=args.run, steps=args.steps,
                          phase=args.phase, track=args.track)
        print(shlex.join(backlog_command(spec)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
