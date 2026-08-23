# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac07

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T06:50:50+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: xo64xo0s

**hypothesis**: Plain English: the turn-exposure dose-response is monotonic through 0.2/0.3/0.5 (tips 0.207/0.234 -> 0.201/0.214 -> 0.162/0.184) -- does pushing dedicated turn-episode exposure to 0.7 keep improving turn accuracy, or does starving translation exposure (only 30% of episodes left for walking) start eroding the walk section? Single lever vs tipfrac05: goal.walk_turn_in_place_frac 0.5->0.7, everything else matched (same pre-cheat init, 2M, pricing keys, permanent fault+push). Prediction-if-true(better): tips < 0.162/0.184 with walk section still clean -> curve still rising. Prediction-if-false(turnover): tips flat/worse or walk erodes -> 0.5 is the sweet spot, dose curve closed. Strongest alternative: tips improve but walk-section translating-episode slip/terms degrade (trade, not free win).

**gate**: BETTER = m5 yaw tips improve on tipfrac05's 0.162/0.184 AND walk section still passes post-fix eval_amp_m5 (translating-episode det slip <=3.5, terms 0, gait_valid 12/12) -> 0.7 becomes the recipe dose. TURNOVER = tips flat/worse OR walk section erodes -> dose curve closed at 0.5. Judge tips from the m5 yaw section (hazards zeroed), walk from translating episodes only (contamination gotcha).

**verdict**: Result: dose-curve TURNOVER, the pre-registered branch -- pushing dedicated turn-episode exposure from 0.5 (tipfrac05, champion) to 0.7 does NOT further improve turn tracking, it REGRESSES it, and also erodes general walk safety. Evidence (eval_amp_m5 formal verdict, m5_pass=false; walk=false yaw=false push=true fault=true): yaw section tip-left/right err 0.2359/0.2252 (up from tipfrac05's bar-clean 0.162/0.184 -- back to the tipfrac02/03 in-band-not-clean band, i.e. MORE turn-exposure made tip-tracking WORSE, not better). Walk section fails on BOTH counts: n_translating collapsed to 2/12 (0/6 det, 2/6 sto) at this dose -- too few translating episodes even to compute a det median (harness sampling gap at high turn-frac, a real methodology note for future high-dose arms) -- and the 2 translating episodes that DID land both exceed the slip bar (4.03/4.30 > 3.5, sto_slip_med 4.168). Own-cfg DR-0 gate also regresses vs tipfrac05's clean 12/12: gait_valid 10/12 with TWO video-confirmed tilt_roll falls (roll peak 35.0deg det, 34.6deg sto) plus 2 sacrificed-leg sto episodes -- a real safety-margin cost from the dose alone (same seed=7 as the champion, so this is NOT the seed-lottery finding from -s3, it's an independent dose-driven regression). Fault section is the one axis that holds/slightly improves (10/12 gv, meets bar, vs tipfrac05's 9/12) -- not enough to offset the yaw+walk+safety regression. CONCLUSION: the turn-exposure dose curve peaks at 0.5 and turns over by 0.7; tipfrac05 (seed7) remains the sole M5-candidate champion. Grid CLOSED (0.2/0.3/0.5/0.7 all read): 0.5 is confirmed the sweet spot, not merely the largest dose tried. Next: no further dose arms on this lever; the two already-flagged prerequisites (hold/forward income repricing for budget-stability, seed-safety-variance root cause) remain the funded path to an M5-candidate promotion, both already DIG-IN-flagged and in progress on a concurrent cycle (uncommitted walk_kernel_yaw_ema work in tree at review time) -- left untouched, no duplicate launch.

