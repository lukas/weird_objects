# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-tipspawn2-startonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T14:04:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: t7eeb6jz

**hypothesis**: Plain English: tipspawn1b's turn-tracking half failed but its walk-slip improved sharply (3.1855 vs parent 3.67, first arm ever under the 3.5 bar) -- this arm isolates WHICH half of that dose earns the slip win by keeping the mid-walk RSI spawns (goal.walk_gait_start_frac=0.5, densifies mid-stride walk starts) but turning OFF the turn-omega passthrough (goal.walk_gait_spawn_wz=0.0, so turn-in-place spawns degrade back to benign walk RSI, same as before tipspawn1b existed). Mechanism: walk_gait_start_frac alone changes the episode-start state DISTRIBUTION (more mid-stride starts, less always-cold-start), independent of the tip-turn omega-passthrough mechanism. Prediction-if-true (RSI-half is the slip driver): walk det_slip_med stays <=3.5ish (near tipspawn1b's 3.1855), confirming state-visitation/reset-distribution as the real slip lever -- promote gait_start_frac alone as a slip fix, independent of the (refuted) turn-tracking mechanism. Prediction-if-false: slip reverts toward parent's 3.67, meaning the spawn_wz turn-half (even though it failed its own tip-tracking goal) was actually the thing buying the slip improvement via some other side effect -- reopens the slip mechanism question entirely. Strongest alternative: slip win in tipspawn1b was noise/lucky episode draw (n=12) unrelated to either lever -- this ablation's own n=12 read partially checks that too since it's an independent seed of episodes.

**gate**: eval_amp_m5 walk section on own cfg. walk det_slip_med <=3.5 with 0/12 falls, gait_valid 12/12 = RSI-half confirmed as the slip mechanism (promote walk_gait_start_frac alone, drop the refuted spawn_wz turn half). walk det_slip_med back near parent's 3.67 (>3.5, delta from tipspawn1b's 3.1855 >0.15) = spawn_wz turn-half (or an interaction) was actually driving the slip win -- do not promote either lever alone without re-testing.

