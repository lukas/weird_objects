# cpg - Berkeley-style parameter gait search

Last updated: 2026-08-23 ~04:1x UTC (gate harness built + first
held-out gate run, see "Gate results 08-23". Track created same day:
**TRACK CREATED by operator direction:
make the Berkeley/Levine CPG result a third first-class path.** This
track is not PPO and should not be converted into seed sweeps. It
owns deterministic/black-box search over low-dimensional gait
parameters, direct behavioral scoring, and any controlled adoption of
the resulting gait as a controller or teacher source.)

## Why this is working better

The CPG search removes the two failure modes that keep hurting the RL
lines: the controller is a small interpretable parameter vector, and
the score is the eval. There is no hidden learned reward for the
policy to exploit, no stochastic policy noise floor, and no long run
where training reward rises while behavior gets worse. Each trial is a
real MuJoCo walk/turn attempt scored on commanded progress,
cross-track error, loaded-foot slip, falls, tilt/height, and effort.

## Current best

- Straight-50 winner verified real: tetrapod, period 2.0,
  swing_frac 0.18494, lift 0.035 m, cmd_tau 0.65225,
  workspace_margin 0.92865; progress_frac 0.9028, slip/m 0.5369,
  zero falls.
- Contextual-250 winner: tetrapod, period 2.0, swing_frac 0.2961,
  lift 0.0299 m, cmd_tau 0.1, workspace_margin 0.7759.
  Score 0.166 vs the straight winner's 0.033 under the fixed
  contextual scorer. Across 5 headings + 2 turns: heading progress
  0.84-0.89, cross <=0.043, slip roughly 0.56-0.75, yaw tracking
  0.993/1.012 of target, zero falls/terminations.

## Known fixes already made

- `sim_gait_compat.SE2FootGait` keeps the knee-frame boundary correct
  for MuJoCo.
- `paper_cpg_search` now supports replay and warm-started GP search.
- The contextual scorer integrates unwrapped yaw, so turns beyond pi
  no longer look sign-inverted.
- Pure-turn slip is normalized by progress plus a yaw foot-arc term
  instead of dividing by near-zero translation.

## DONE gate

The track is green when a CPG controller passes a held-out contextual
session gate with zero falls, no sacrificed legs, correct headings,
turns in both directions, stops/restarts, and slip better than or at
least competitive with the current teacher band. Required panels:
DR-0 plus a modest own-DR/friction/servo-profile sweep, 60 s command
scripts, video review, and a saved parameter artifact that the web UI
and teacher-library generator can load.

## Gate results 08-23 (`rl_move/sim/eval_cpg_gate.py`, built this cycle)

Held-out 60 s scripted session (seeded held-out headings +/-87/-49/+88/
+57/-20 deg, turns both ways, 3 stops/restarts), panels dr0 /
dr0-script2 / mu1.2 / mu0.8 / loaded-servo. Contextual-250 winner:
**DR-0 PASS decisively** (heading progress mean 0.899, turns yaw_along
1.03/1.05, stop drift <=3 mm, slip/m 0.70, zero falls, 6/6 legs
cycling duty ~0.70, roll/pitch peaks <0.6 deg; video reviewed, no
pathologies). Robustness 3/4: script2 0.899 PASS, mu1.2 PASS, loaded
PASS (0.923, best), **mu0.8 FAIL — open-loop turn overshoot 1.33/1.35
vs the pre-registered [0.70,1.30] band**, plus contact chatter (swing
counts 39-55 vs ~30). Everything else at mu0.8 passes (prog 0.827,
slip 1.35, zero falls). Artifact NOT exported (gate is strict; no
post-hoc threshold loosening). Verdict + videos:
`logs/cpg_gate/contextual250-winner/`. Root cause: the controller is
open-loop, so yaw-per-stride scales with ground friction; the search
never priced friction variation.

## Next

1. TRIAGE the robustness-refinement search
   `paper-cpg-robust120-20260823` (launched 08-23 ~04:2x on the
   controller, log `logs/paper_cpg_search/paper-cpg-robust120-*.log`):
   `--mu-list 0,1.2,0.8` panel scoring (new, default-off flag), GP
   warm-started from the contextual-250 winner, 120 iters. Then re-run
   the new winner through `eval_cpg_gate.py --robust`; on full PASS
   export `rl_move/sim/policies/cpg_controller_<name>.json`
   (`--export`) and expose it in the web UI.
2. If turn overshoot at mu0.8 survives parameter search, the honest
   fix is closed-loop yaw trim (IMU yaw-rate feedback scaling omega),
   a controller change — keep it a separate, A/B-tested arm.
3. Build a controlled `teacher_v2`/motion-library fork from the CPG
   winner and compare against the current teacher at equal AMP budget.
   Do not silently replace `teacher_v1` or recalibrate joystick slip
   bars without that A/B result.
