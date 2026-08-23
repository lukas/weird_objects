# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn3-wzonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T14:32:50+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn1b

**hypothesis**: Plain English: tipspawn1b (start_frac=0.5 + spawn_wz=1.0) cut walk-slip sharply (3.1855 vs parent 3.67) but tipspawn2-startonly (start_frac=0.5 alone, spawn_wz=0) did NOT reproduce it (3.6015, unmoved) -- this arm tests the last untested cell of the 2x2 design: spawn_wz=1.0 ALONE (start_frac reverted to 0/default) on the same pushcal518 base. Mechanism: the turn-omega passthrough into mid-stride gait spawns may itself be what's buying the slip win (e.g. by changing foot-plant phase distribution at episode start), independent of the RSI/reset-distribution half. Prediction-if-true (spawn_wz is the driver alone): walk det_slip_med lands near tipspawn1b's 3.1855 (<=3.5 bar), completing the isolation -- promote goal.walk_gait_spawn_wz alone as the slip fix. Prediction-if-false (needs both / true interaction): slip reverts toward parent's 3.67 like tipspawn2-startonly did -- neither lever alone suffices, only the combination works, meaning any promotion must carry both levers together (even though spawn_wz's own named purpose, turn-in-place tip-tracking, already FAILED on tipspawn1b, refuted as a 5th independent turn-tracking mechanism if this reads badly on yaw too, though yaw is not this arm's target). Strongest alternative: tipspawn1b's slip result was noise (n=12) and neither half nor the combination is a real mechanism -- low prior given push/fault sections also read clean in both prior arms, but flagged for a re-check if this cell also fails and the eventual literature-clean answer is 'no reproducible effect'.

**gate**: eval_amp_m5 walk section on own cfg. walk det_slip_med <=3.5 with 0/12 falls, gait_valid 12/12 = spawn_wz ALONE confirmed as the slip mechanism (promote it standalone, start_frac was never needed). PARTIAL = det_slip_med improves >=0.15 vs parent's 3.67 but misses 3.5 (mechanism partially real, needs the start_frac half too -- promote the combined dose from tipspawn1b instead). FAIL = det_slip_med within 0.15 of parent's 3.67 (unmoved) -- spawn_wz alone insufficient, closes the isolation: the tipspawn1b slip win requires the true interaction of both levers together, or was itself noise; next step is a direct replicate of tipspawn1b (same combined cfg, different seed) to confirm the combined effect is real before any promotion.

