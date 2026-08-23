# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-acq-kernelema

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T08:39:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-acq1

**hypothesis**: Does the hold/forward income-repricing kernel-noise-tax fix (reward.walk_kernel_yaw_ema + walk_kernel_vel_ema, bank-tested this window) rescue BUDGET-CONTINUATION turn-tracking, the exact failure the acq1 arm measured (tips eroded 0.162/0.184 -> 0.204/0.269 over +6M steps warm from the tipfrac05 checkpoint while reward rose then plateaued)? Single lever vs acq1: both EMA flags added on top of the identical +6M continuation from the SAME tipfrac05 checkpoint (fixed basin, matched seed=7) -- this controls for the basin-selection noise that just closed the fresh-2M 3-arm decomposition grid (kernelema1/-yawonly/-velonly2 all landed in tipfrac05's own seed-noise band with no attributable per-axis effect at a FRESH retrain). Testing the fix as a CONTINUATION instead answers the question it was actually built for (q_20260823T0240Z item b) rather than repeating the fresh-retrain confound. Prediction-if-true(rescued): tips stay <=0.20-0.25 through +6M, m5 walk/yaw both improve or hold vs acq1's erosion. Prediction-if-false: tips still erode similarly -- the repricing mechanism is real (confirmed on scripted refs) but too small next to the dominant actuation-cost asymmetry (current/gyro/roll 4-10x pricier on real motion), pointing at that harder untried piece next.

**gate**: PASS/RESCUED if eval_amp_m5 tips stay <=0.25 both signs (in-band) and do not erode further than the 2M tipfrac05 parent's own seed-variance band (~0.16-0.23) -- funds treating kernel-EMA as a standing default for any future >2M AMP turn continuation. FLAT/NO-RESCUE if tips erode similarly to acq1 (>0.25 either sign, or clearly worse than the 2M parent) despite reward rising -- confirms repricing alone is insufficient against the actuation-cost asymmetry and escalates to that harder lever. WORSE (safety floor drop: gait_valid <10/12, any new fall) reverts/re-scopes regardless of tip number.

