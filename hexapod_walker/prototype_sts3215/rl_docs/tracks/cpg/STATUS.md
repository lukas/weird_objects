# cpg - Berkeley-style parameter gait search

Last updated: 2026-08-23 (**TRACK CREATED by operator direction:
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

## Next

1. Build `eval_cpg_gate.py`: a reusable 60 s held-out command gate for
   `paper_cpg_search` winners, including headings, turns, stops, and
   restarts. It should emit one JSON verdict and short videos/contact
   sheets.
2. Run the contextual winner through that gate, first DR-0 then a
   small robustness panel. If it passes, save a named controller
   artifact and expose it in the web UI.
3. Build a controlled `teacher_v2`/motion-library fork from the CPG
   winner and compare against the current teacher at equal AMP budget.
   Do not silently replace `teacher_v1` or recalibrate joystick slip
   bars without that A/B result.
