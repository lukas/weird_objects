# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn2-startonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T14:04:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: t7eeb6jz

**hypothesis**: Plain English: tipspawn1b's turn-tracking half failed but its walk-slip improved sharply (3.1855 vs parent 3.67, first arm ever under the 3.5 bar) -- this arm isolates WHICH half of that dose earns the slip win by keeping the mid-walk RSI spawns (goal.walk_gait_start_frac=0.5, densifies mid-stride walk starts) but turning OFF the turn-omega passthrough (goal.walk_gait_spawn_wz=0.0, so turn-in-place spawns degrade back to benign walk RSI, same as before tipspawn1b existed). Mechanism: walk_gait_start_frac alone changes the episode-start state DISTRIBUTION (more mid-stride starts, less always-cold-start), independent of the tip-turn omega-passthrough mechanism. Prediction-if-true (RSI-half is the slip driver): walk det_slip_med stays <=3.5ish (near tipspawn1b's 3.1855), confirming state-visitation/reset-distribution as the real slip lever -- promote gait_start_frac alone as a slip fix, independent of the (refuted) turn-tracking mechanism. Prediction-if-false: slip reverts toward parent's 3.67, meaning the spawn_wz turn-half (even though it failed its own tip-tracking goal) was actually the thing buying the slip improvement via some other side effect -- reopens the slip mechanism question entirely. Strongest alternative: slip win in tipspawn1b was noise/lucky episode draw (n=12) unrelated to either lever -- this ablation's own n=12 read partially checks that too since it's an independent seed of episodes.

**gate**: eval_amp_m5 walk section on own cfg. walk det_slip_med <=3.5 with 0/12 falls, gait_valid 12/12 = RSI-half confirmed as the slip mechanism (promote walk_gait_start_frac alone, drop the refuted spawn_wz turn half). walk det_slip_med back near parent's 3.67 (>3.5, delta from tipspawn1b's 3.1855 >0.15) = spawn_wz turn-half (or an interaction) was actually driving the slip win -- do not promote either lever alone without re-testing.

**verdict**: FAIL on its pre-registered branch: the RSI-reset-distribution half ALONE does not reproduce tipspawn1b's slip win. eval_amp_m5 walk det_slip_med=3.6015 (bar <=3.5), close to parent pushcal518's 3.67 (delta 0.07) and far from tipspawn1b's 3.1855 (delta 0.42, >>0.15) -- this lands squarely in the pre-registered FAIL branch, not PARTIAL. Safety/gait unaffected: walk 0/12 raw falls, gait_valid 12/12, det_prog_med 0.94; push PASS (det_slip_med 3.5045, gait_valid 12/12); fault PASS (gait_valid 12/12); yaw FAIL as expected (tip 0.2099/0.2466, this arm never targeted yaw, spawn_wz=0 here). Video-clean six-leg gait throughout, no pathology. Root-cause conclusion: with walk_gait_start_frac=0.5 alone measured NOT sufficient, and the combined dose (tipspawn1b, start_frac=0.5+spawn_wz=1.0) measured sufficient (3.1855), the slip win requires the spawn_wz turn-omega passthrough -- either alone or via an interaction with start_frac -- completing 3 of 4 cells of the 2x2 (start_frac x spawn_wz) design. The remaining untested cell (spawn_wz=1.0 alone, start_frac=0/default) isolates which it is; launching that arm now.

