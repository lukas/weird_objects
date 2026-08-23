# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn3-wzonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T14:32:50+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn1b

**wandb_id**: 7ysj6mrj

**hypothesis**: Plain English: tipspawn1b (start_frac=0.5 + spawn_wz=1.0) cut walk-slip sharply (3.1855 vs parent 3.67) but tipspawn2-startonly (start_frac=0.5 alone, spawn_wz=0) did NOT reproduce it (3.6015, unmoved) -- this arm tests the last untested cell of the 2x2 design: spawn_wz=1.0 ALONE (start_frac reverted to 0/default) on the same pushcal518 base. Mechanism: the turn-omega passthrough into mid-stride gait spawns may itself be what's buying the slip win (e.g. by changing foot-plant phase distribution at episode start), independent of the RSI/reset-distribution half. Prediction-if-true (spawn_wz is the driver alone): walk det_slip_med lands near tipspawn1b's 3.1855 (<=3.5 bar), completing the isolation -- promote goal.walk_gait_spawn_wz alone as the slip fix. Prediction-if-false (needs both / true interaction): slip reverts toward parent's 3.67 like tipspawn2-startonly did -- neither lever alone suffices, only the combination works, meaning any promotion must carry both levers together (even though spawn_wz's own named purpose, turn-in-place tip-tracking, already FAILED on tipspawn1b, refuted as a 5th independent turn-tracking mechanism if this reads badly on yaw too, though yaw is not this arm's target). Strongest alternative: tipspawn1b's slip result was noise (n=12) and neither half nor the combination is a real mechanism -- low prior given push/fault sections also read clean in both prior arms, but flagged for a re-check if this cell also fails and the eventual literature-clean answer is 'no reproducible effect'.

**gate**: eval_amp_m5 walk section on own cfg. walk det_slip_med <=3.5 with 0/12 falls, gait_valid 12/12 = spawn_wz ALONE confirmed as the slip mechanism (promote it standalone, start_frac was never needed). PARTIAL = det_slip_med improves >=0.15 vs parent's 3.67 but misses 3.5 (mechanism partially real, needs the start_frac half too -- promote the combined dose from tipspawn1b instead). FAIL = det_slip_med within 0.15 of parent's 3.67 (unmoved) -- spawn_wz alone insufficient, closes the isolation: the tipspawn1b slip win requires the true interaction of both levers together, or was itself noise; next step is a direct replicate of tipspawn1b (same combined cfg, different seed) to confirm the combined effect is real before any promotion.

**verdict**: spawn_wz alone does NOT reproduce the slip win — the 2x2 isolation is complete and only the COMBINED dose works (or tipspawn1b was noise). m5 walk det_slip_med 3.59 vs parent pushcal518's 3.67: an 0.08 improvement, inside the pre-registered +-0.15 'unmoved' band (tipspawn1b combined hit 3.1855; tipspawn2-startonly alone hit 3.6015, also unmoved). Safety clean everywhere: 0 falls all sections, walk gait_valid 12/12, m5 walk det strip shows an upright six-leg cycle with no flag leg; push section PASS (det slip 3.2405), fault PASS (gv 11/12). Side note: m5 yaw tip_left_err 0.1945 clears the 0.20 bar but tip_right 0.2223 regressed — the same asymmetric trade seen across the whole turn-tracking grid, no new yaw signal. Why: mid-stride omega passthrough alone changes plant-phase at spawn but evidently the slip win needs BOTH the mid-walk RSI (start_frac=0.5) and the live spawn omega together. What's next (pre-registered in this run's gate): direct seed replicates of tipspawn1b's combined cfg to confirm the interaction effect is real (not n=12 noise) before promoting both levers together.

