# cw-recover-any9-lessfocus-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-16T08:29:23+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-recover-any8-spacedreplay-scratch1

**wandb_id**: agf2k5k4

**hardware_ready**: False

**hypothesis**: Teach the fallen robot to get up from lying completely flat on its belly (bucket 11, 'zero'), which any8's spaced-replay curriculum got permanently stuck on despite dedicating half of all training time to it. This run tests whether the stall was caused by OVER-CONCENTRATING replay mass on a stuck frontier (crowding out the varied earlier-bucket practice -- crouch/partial curls -- that apparently scaffolds the flat-belly recovery) rather than by insufficient exposure. ONE hypothesis (frontier-mass share), expressed as a coupled 4-number redistribution because the masses must renormalize together: focus 0.50->0.20, recent (previous-3 buckets) 0.25->0.35, weak 0.15 unchanged, uniform 0.10->0.30. Warm-started from any8's exact stuck checkpoint (not from scratch) so this is a clean test of curriculum-schedule-only effect on an already-stuck policy. Prediction-if-true: bucket 11 success climbs off its ~0-3/16 floor within the 20M budget and the frontier resumes promoting toward B12+ (tangle). Prediction-if-false: bucket 11 stays flat near 0 even with more balanced replay mass -- pointing at a reward/mechanism gap specific to the flat-belly recovery (the same family as the campaign's other-named 'flat-rise stall'), not a curriculum-weighting problem; two misses on this wall (any8 + this run) would then call for a mechanism-level fix (e.g. targeted BC anchor exposure or reward reshaping for the zero family) rather than a third curriculum resweep.

**gate**: Read at 20M (or earlier plateau): bucket 11 (zero) CERT success_fraction must show a clear rising trend and reach >=0.5 at some point in the last 5 certs (vs any8's flat ~0.016 mean) for a PASS: frontier legitimately promotes past B11. Also required for PASS: buckets 0-10 retained >=0.8 gate_fraction at the final cert (no regression from rebalancing mass away from frontier). FAIL (bucket 11 stays <0.2 the whole run, or retention breaks) = curriculum-mass hypothesis refuted -- escalate to a mechanism-level design (BC anchor / reward term for the zero family), no third mass-tuning resample.

**verdict**: FAIL, matching the pre-registered if-false branch, PLUS a new retention regression not predicted by the gate. Bucket 11 (zero/flat-belly) CERT success_fraction: 0.25 (x2, inherited from any8) then flat 0.0 for the last 7/9 certs -- never reached the required >=0.5, frontier never promoted past B11 across the full 20M budget. Rebalancing curriculum mass off the stuck frontier (any8's 50% focus -> this run's 20%, recent 0.25->0.35, uniform 0.10->0.30) did NOT unstick it: two misses now on the curriculum-mass hypothesis (any8 over-concentrated, any9 diffuse), both FAIL on the identical wall -- CLOSES the curriculum-mass avenue per the two-miss rule. NEW finding: the required buckets 0-10 >=0.8 retention floor also broke -- bucket 6 (crouch_mid) dropped to 0.25 and bucket 7 (crouch_deep) to 0.0 at the final CERT (both below 0.8). Video (recover_det_11, zero) confirms a genuine capability gap, not an exploit -- robot stays flat/splayed on its belly, terminates on over_current, the same flat-rise-stall pathology named elsewhere in the campaign. Own-DR-0.1 single-sample harness agrees exactly (zero 0/1 det). Next lever per the pre-registered gate: mechanism-level (BC anchor / reward term targeted at the zero family), not a third curriculum-mass resample -- DIG-IN flagged for the design+code (reward/env code change).

