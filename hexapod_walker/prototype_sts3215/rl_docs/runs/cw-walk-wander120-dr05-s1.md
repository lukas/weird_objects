# cw-walk-wander120-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T15:50:07+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-wander120-dr05

**hypothesis**: Seed-robustness check for the 120s endurance rung, one variable off wander120-dr05 (seed 0->1, identical wander_dr05 parent+recipe). Motivated by wander60-dr05-s1 (same seed-twin move at the 60s rung): det was clean but sto showed a genuine sacrificed leg (duty 0.08 vs siblings 0.37-0.89, slip/m 2.11) -- an 11/12 not 12/12, so seed-dependent sto fragility is a real pattern at this DR0.5 driving line, not noise. If-true: seed 1 at 120s reproduces r1's clean PASS (gv 12/12 det+sto, 0 term, along_dist_m>=4.87m every ep, prog_ratio med>=0.85) -- the 60s sto wobble does not compound over 2 minutes. If-false: sto shows the same or a worse sacrificed-leg/low-duty pattern at 120s -- seed-dependent sto fragility is real and compounds with horizon, and the wander line needs a duty-floor income term before being called seed-robust. Strongest alternative: a milder sto dip that still clears gv 12/12 (noise, not a defect) -- report per-leg duty either way.

**gate**: own-cfg DR0.5 120s det+sto: gv 12/12, 0 term, along_dist_m>=4.87m every episode (matching r1's PASS criterion, not the mis-specified net-displacement literal gate), prog_ratio med>=0.85; DR0 det retention gv 6/6; frames watched det full horizon + worst sto episode for per-leg duty

**refused_reason**: hexapod-mjx-train-0 already runs cw-dep-vref1-r1-payload125 — GPU pods host exactly one run; pick a free GPU pod.

