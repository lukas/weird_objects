# cw-walkcurr-pf-fwd6-hgt2-pdw05-pdx15b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T01:05:53+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-hgt2

**wandb_id**: shjm1zyx

**hypothesis**: Plain English: dose sibling of hgt2-pdw05b -- same park_duty_window_s de-confound (2.0->0.5s so the duty-history buffer fills before the 2.0s height-drop termination fires) PLUS 1.5x k_park_duty (0.08->0.12) to test whether the de-confounded charge merely needs to fire vs. needs to fire HARDER to out-price the frozen/collapsing stance. A 3x dose (0.24) was tried in the bank first and REJECTED (breaks the required park_gated>belly_sit_gated ranking -- overpriced the honest 'refuse to move' park contrast behavior below the still-fast-terminating belly_sit); 1.5x is the largest dose that stays bank-legal (test_walkcurr_pf_hgt_gait_beats_belly_sit/ranking_still_holds green at 'tight_pdw05_pdx1p5'). Same sigma=11/drop=25/grace=2.0, same fresh 2M discovery init, not warm-started. Prediction-if-true: freeprog crosses toward/past 0 faster or further than the pdw05b sibling. Prediction-if-false (same or worse than pdw05b): the extra duty pricing doesn't help once the confound itself is fixed -- read both jointly, whichever (or neither) moves decides if height-gate can still land before escalating to a new foot-contact mechanism or BC-kickstart.

**gate**: Same rung-1 gate as hgt2-pdw05b; PASS = rung-1 lands. Read jointly with hgt2-pdw05b (window-fix-only) before any further height-gate calibration or new-mechanism escalation.

