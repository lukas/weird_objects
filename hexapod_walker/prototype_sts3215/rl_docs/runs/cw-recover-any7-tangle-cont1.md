# cw-recover-any7-tangle-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-16T02:54:20+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any6-microbuckets-scratch1

**wandb_id**: b49vialn

**hypothesis**: Teach the fallen robot to reliably get past 'legs badly tangled together' recovery, continuing straight from the any6 checkpoint instead of starting over. any6 already proved the curriculum itself has no artificial cliff (it reached B15, the second-to-last rung, by half its budget) but ran out of time oscillating in the tangle/bank band (B13-15) with a small 8-episode certification sample causing noisy promote/retreat swings. This arm warm-starts from any6's exact checkpoint and doubles the certification sample size (8->16 episodes/kind) so promotion decisions are less noisy, then gives it a fresh full 40M-step budget to either push the frontier past B15 to B16 (flip) or show the tangle wall holds even with more time and better statistics. Prediction-if-true: frontier climbs past 15 to 16 (flip) with sustained >=0.8 cert fractions on tangle/bank. Prediction-if-false: frontier still oscillates in the B13-15 band at 40M even with 16-episode certs -- which would upgrade tangle/bank recovery from 'still contested' to a genuine, statistically-solid capability wall needing a design lever (not just more steps).

**gate**: HARD integration gate at ~1M steps: CERT/recover_bucket_13_success_fraction (or whatever bucket is active) PRESENT with a 16-episode denominator, frontier before/after logged, matching any6's config otherwise. Full-arm PASS bar: frontier legitimately (CERT >=0.8, 16-ep denominator) reaches B16 (flip) at some point in the 40M budget, OR if it does not, a clear verdict naming whether tangle/bank recovery is now a statistically solid wall (fractions consistently <0.8 with the larger sample) vs still noisy/improving. Video-verified genuine six-foot recover-to-stand on the earned frontier, no flag/stilt/park.

