# cw-recover-any1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T18:48:31+00:00

**pod**: hexapod-mjx-train-8

**steps**: 40000000

**parent**: cw-stand-footlow2-hard1

**wandb_id**: e2q5q5a3

**hypothesis**: Teach the robot to GET UP FROM ANYWHERE -- knocked over, on its side, upside-down, mid-crouch, sitting after a lower, or standing with one foot misplaced -- and settle into its full-height six-feet-loaded stand and stop; this arm tests whether the new potential-difference recovery reward (REWARD.md 4c: PBRS on U/L/H/M/P with smooth-min per-foot load, one-shot held-success bonus that ENDS the episode, time tax, no-early-abort fail cost) + the adaptive start-family curriculum (buckets 1-2 first, >=80 admit / <20 retreat) + the eligibility-gated cw-getup3 BC anchor can learn a universal recovery specialist warm-started from the stance champion. OPERATOR-ORDERED COUPLED BUNDLE: one-run exception to one-variable-per-run granted in the 08-15 ~18:15 KICK (ruling recorded in STATUS.md + CURRENT_TRUTHS.md). Prediction-if-true: env/recover_success climbs family by family and the mixture hardens to >=95 det / >=85 sto held recovery by 40M, video shows genuine roll-over-and-rise with all six feet cycling to load, ordinary rise/hold/lower intact. Prediction-if-false: success stalls on a named family (pointing at the family-5/6 harvest rungs or anchor coverage as the next lever) or the anchor's upright-manifold gate proves too narrow to bridge flips. Strongest alternative: buckets 1-3 master but side/back/inverted recovery never crosses 20 percent -- recovery from inversion needs demonstration data, not reward shaping. Recorded v1 deviations from the directive spec: families 5-6 + exact-qvel bank restore + in-train handoff eval not built (pre-registered next rungs); curriculum stats per-env not fleet-global; COM/support = footprint+all-loaded+level proxy; critic carried via obs-pad transplant (optimizer fresh) since actor-only warm start is unsupported; k_current kept at lineage default per the directive's keep-safety-costs-strong clause.

**gate**: PASS if by 40M (or an earlier milestone) held recovery on the ACTIVE family mixture reaches >=95 pct det / >=85 pct sto (SCORE/recover + env/recover_success; VERDICT REQUIRES VIDEO: genuine recover-to-stand from every active family, all six feet loaded, zero flag/stilt/park), AND standard eval shows no regression on ordinary rise/hold/lower vs footlow2_hard1, AND eval_handoff to the frozen stance controller holds >=1 s post-success at triage. STOP EARLY only on a pre-registered banked cheat class appearing in video or a safety failure. Research specialist -- never auto-replaces the product baseline.

