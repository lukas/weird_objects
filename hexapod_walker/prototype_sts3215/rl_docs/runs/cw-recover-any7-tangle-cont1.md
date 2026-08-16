# cw-recover-any7-tangle-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-16T02:54:20+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any6-microbuckets-scratch1

**wandb_id**: b49vialn

**hardware_ready**: False

**hypothesis**: Teach the fallen robot to reliably get past 'legs badly tangled together' recovery, continuing straight from the any6 checkpoint instead of starting over. any6 already proved the curriculum itself has no artificial cliff (it reached B15, the second-to-last rung, by half its budget) but ran out of time oscillating in the tangle/bank band (B13-15) with a small 8-episode certification sample causing noisy promote/retreat swings. This arm warm-starts from any6's exact checkpoint and doubles the certification sample size (8->16 episodes/kind) so promotion decisions are less noisy, then gives it a fresh full 40M-step budget to either push the frontier past B15 to B16 (flip) or show the tangle wall holds even with more time and better statistics. Prediction-if-true: frontier climbs past 15 to 16 (flip) with sustained >=0.8 cert fractions on tangle/bank. Prediction-if-false: frontier still oscillates in the B13-15 band at 40M even with 16-episode certs -- which would upgrade tangle/bank recovery from 'still contested' to a genuine, statistically-solid capability wall needing a design lever (not just more steps).

**gate**: HARD integration gate at ~1M steps: CERT/recover_bucket_13_success_fraction (or whatever bucket is active) PRESENT with a 16-episode denominator, frontier before/after logged, matching any6's config otherwise. Full-arm PASS bar: frontier legitimately (CERT >=0.8, 16-ep denominator) reaches B16 (flip) at some point in the 40M budget, OR if it does not, a clear verdict naming whether tangle/bank recovery is now a statistically solid wall (fractions consistently <0.8 with the larger sample) vs still noisy/improving. Video-verified genuine six-foot recover-to-stand on the earned frontier, no flag/stilt/park.

**verdict**: Pre-registered PASS bar (frontier legitimately reaches B16/flip) NOT met -- frontier climbed 0->15 by 34M (curriculum state resets on warm-start, unlike network weights) then held flat at B15 for the entire last 6M steps/6 consecutive 16-episode certs, never promoting. But this is the pre-registered if-false branch, and it upgrades the finding: splitting bucket 15 into its two kinds shows BANK is essentially solved (cert fraction 0.56-1.0, trending to 1.0 the last 3 reads) while TANGLE specifically is flat at 0.25-0.44 across all 6 late certs (n=16 each) -- a statistically solid capability wall, not noise (any8, running in parallel with spaced replay, is the other pre-registered lever on the same wall). Video (all 4 tangle severities, det pass) shows the skill is REAL when it works: legs start visibly crossed, the policy genuinely untangles them and settles into a clean six-foot stance within ~2s, holds clean to 16s, no flag-leg/park/stilt -- so this is a success-RATE wall under held-out variation, not a qualitative gap. flip (B16, never trained) fails as expected in the DR0 gate pass (genuine on-back flailing) but SUCCEEDED in the DR0.1 own-cfg pass (n=1, likely luck not a real B16 capability -- frontier never certified past 15). Stochastic pass 0/18 across the board matches the ALREADY-DOCUMENTED any4 action-noise-breaks-the-strict-hold-check artifact (policy_std 0.589) -- not new evidence. NOTE: plant_catch (bucket 0, the EASIEST bucket, solved 1.0 at its 1M promotion cert) FAILED in BOTH single-sample det passes (gate AND own-cfg, both over-current-flavored) -- bucket 0 is never re-certified once the frontier moves on (any7 has no retention assay), so this could be a real un-tracked retention regression, not just n=1 noise; any8s spaced-replay design explicitly tests old-bucket retention and should surface whether this is real. Bank sample size doubling (8->16 eps) was the tested lever and it did NOT move tangle -- next lever needs to be different (targeted tangle exposure/curriculum weight, not more precise measurement of the same training mix).

