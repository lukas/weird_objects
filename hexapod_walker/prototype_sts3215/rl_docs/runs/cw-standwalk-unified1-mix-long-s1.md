# cw-standwalk-unified1-mix-long-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T03:58:14+00:00

**pod**: hexapod-mjx-train-5

**steps**: 16000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1-acq8m

**wandb_id**: poe0uaa1

**hypothesis**: Plain English: seed-1 twin of cw-standwalk-unified1-mix-long-s0 - does 60 s full-length session training (5-7 chained segments, joystick command mix, hold-height commands) beat 30 s training on long-horizon zero-termination behavior, on the catastrophe-prone rescued seed? Continues the s1-acq8m PASS checkpoint; walk BC anchor coef=1.0 stays live. Operator order 20260828T033725Z. Completes the 2x2 grid (seed x episode length) so the episode-length read is not a single-seed fluke. Prediction-if-true: beats cw-standwalk-unified1-mix-s1 on session terminations/completion at matched steps with clean gait. Prediction-if-false: no session gain and/or seed1 walk relapse under the longer, rarer-reset diet. Strongest alternative: rise-start practice dilution hurts the rise panel - compare rise success vs the 30 s twin.

**gate**: At 16M, paired vs cw-standwalk-unified1-mix-s1 at matched steps: PASS if DR-0 det walk gait_valid >=5/6 zero-sac AND session terminations-per-episode / completion fraction beat the 30 s twin outside eval noise; PARTIAL if equal; FAIL if gait_valid regresses (seed1 relapse) or rise success drops materially vs the twin.

