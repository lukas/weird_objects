# cw-walk-wander120-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: EVALUATED

**created**: 2026-08-10T15:52:29+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-wander120-dr05

**hardware_ready**: no

**hypothesis**: Seed-robustness check for the 120s endurance rung, one variable off wander120-dr05 (seed 0->1, identical wander_dr05 parent+recipe). Motivated by wander60-dr05-s1 (same seed-twin move at the 60s rung): det was clean but sto showed a genuine sacrificed leg (duty 0.08 vs siblings 0.37-0.89, slip/m 2.11) -- an 11/12 not 12/12, so seed-dependent sto fragility is a real pattern at this DR0.5 driving line, not noise. If-true: seed 1 at 120s reproduces r1's clean PASS (gv 12/12 det+sto, 0 term, along_dist_m>=4.87m every ep, prog_ratio med>=0.85) -- the 60s sto wobble does not compound over 2 minutes. If-false: sto shows the same or a worse sacrificed-leg/low-duty pattern at 120s -- seed-dependent sto fragility is real and compounds with horizon, and the wander line needs a duty-floor income term before being called seed-robust. Strongest alternative: a milder sto dip that still clears gv 12/12 (noise, not a defect) -- report per-leg duty either way.

**gate**: own-cfg DR0.5 120s det+sto: gv 12/12, 0 term, along_dist_m>=4.87m every episode (matching r1's PASS criterion, not the mis-specified net-displacement literal gate), prog_ratio med>=0.85; DR0 det retention gv 6/6; frames watched det full horizon + worst sto episode for per-leg duty

**verdict**: PASS -- seed-twin (seed 0->1) reproduces r1's 120s DR0.5 endurance-wander PASS cleanly. Own-cfg DR0.5 (own-DR eval, per-mode 6, det+sto): gait_valid 6/6 det + 6/6 sto, 0 term, 0 sacrificed legs, along_dist_m 5.00-5.76m det / 4.88-5.95m sto -- every one of 12 episodes clears the >=4.87m gate (matching r1's along-path criterion, not net forward_dist_m), prog_ratio med 0.94 det / 0.92 sto (>=0.85 gate with margin). DR0 retention: det 6/6 gv, sto 6/6 gv, 0 term, prog med 0.95/0.95, slip/m 1.24/1.24 -- matches r1's own band. Frames watched (det/0, det/2 [sole threshold-straddle 'ok False', prog 0.91/vel_err 0.032, along_dist still 5.73m], det/4, sto/4 [worst sto, along_dist 4.88m at the gate edge]): clean six-leg cycling every episode, no flag leg, no dragging, same character as the parent -- the recipe is seed-robust, not seed luck.

