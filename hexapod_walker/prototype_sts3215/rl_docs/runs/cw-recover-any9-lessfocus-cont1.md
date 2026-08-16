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

**verdict**: FAIL, matching the pre-registered if-false branch: bucket 11 (zero/flat-belly) CERT success 0.25(x2 inherited)->0.0 for the last 7/9 certs over the full 20M; de-concentrating replay mass did not unstick the frontier. Two misses (any8 concentrated, any9 diffuse) CLOSE the curriculum-mass avenue. DIG-IN (deep cycle, 08-16 ~12:xx): root cause is a start-distribution COVERAGE GAP, not anchor pressure or pricing — (1) the recover BC anchor already fires (recover_bc_eligible~1.0, fill 131k/update) and its loss is near-minimized (~0.05), so more anchor weight has no headroom; (2) the ladder's partial_high/mid/low rungs are LINEAR curls (f*q_crouch), NOT states on the executable belly->plant rise trajectory, so a policy entrenched in the splay-to-low-crouch local optimum (video: over_current mid-push) never PRACTICES from mid-rise states — the exact exploration gap goal.rise_rsi_frac closed for the rise task (score1->rsi1 forensic, 08-10); (3) the 'retention regression' is oscillation, not forgetting (crouch_deep 0.0->1.0->0.0, B10 0.06->0.94 across adjacent certs) — PPO churn from grinding a 0%-success frontier, secondary to the same gap. MECHANISM BUILT this cycle: goal.recover_rsi_frac/_kinds (RECOVER RSI, default-off bit-exact, forced CERT kinds never RSI so certs stay pure, RSI episodes excluded from rollout/self-cert stats; tests test_recover_rsi_* + full RECOVER bank 24/24 green; snapshot a1994dee). Follow-up: cw-recover-any10-zerorsi-cont1 (matched A/B vs this run: same any8 checkpoint, same masses/seed, single delta recover_rsi_frac=0.5).

