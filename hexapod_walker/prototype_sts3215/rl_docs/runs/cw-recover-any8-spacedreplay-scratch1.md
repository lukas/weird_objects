# cw-recover-any8-spacedreplay-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-16T04:51:59+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-recover-any6-microbuckets-scratch1

**wandb_id**: qe35tc9m

**hardware_ready**: False

**hypothesis**: Teach the fallen robot to stand back up without forgetting the easy cases as it learns the hard ones; this arm tests whether spaced replay of earlier difficulty buckets (50% frontier / 25% previous-3 / 15% weakest certified old bucket / 10% remaining history, monotonic unlock) plus the new W&B recovery scorecard lets a from-scratch policy climb the 17-rung micro-bucket ladder while KEEPING every rung it has certified. OPERATOR-ORDERED (MCP operator lane, Codex relaying Lukas, 20260816T045031Z; commit c5d2e0e8 'Add spaced replay and recovery scorecard'): exact cw-recover-any6-microbuckets-scratch1 recipe/config/start-bank, genuinely FROM SCRATCH — no --init-from, no checkpoint transplant, no warm start; any6 is recipe provenance only. New vs any6: bucket-level spaced-replay sampling masses as above, 16 cert envs (was 8), retention-buckets=3, and the RECOVER_SCORE/* scorecard (overall_points, overall_weighted_success, max_unlocked_bucket, per-bucket success fractions, bucket sample probabilities) alongside CERT frontier/retention assays. Prediction-if-true: the frontier climbs while previously certified buckets hold >=0.8 on retention assays (no catastrophic forgetting), visible as monotone RECOVER_SCORE/overall_points. Prediction-if-false: replay mass on old buckets starves frontier learning and the ladder climbs slower than any6/any7 at matched steps — which prices the replay tax. Existing any7 continues as the old-sampler control; this run is not a substitute for it.

**gate**: HARD integration gate at the first ~1M certification: W&B must contain RECOVER_SCORE/overall_points, RECOVER_SCORE/overall_weighted_success, RECOVER_SCORE/max_unlocked_bucket, RECOVER_SCORE/bucket_00_success_fraction, and bucket sample probabilities, plus CERT frontier/retention assays with 16-episode denominators. Full-arm PASS bar: legitimately earned frontier (>=0.8 deterministic cert fraction per promotion) at or past any6's best rung by 40M WITH all previously certified buckets retained >=0.8 on cert assays, and video-verified genuine recover-to-stand (all six feet loaded, no flag/stilt/park) on the earned frontier.

**verdict**: Pre-registered PASS bar (frontier at/past any6/any7's best rung B15 by 40M, with retention) NOT met -- and misses WORSE than the pre-registered if-false branch predicted. Frontier climbed cleanly B0->B11 by 13.0M steps (~1M/bucket, matching any6/7 pace), then got stuck at B11 (zero: belly-flat + small joint jitter) for the ENTIRE remaining 27M/40M steps: 28 consecutive 16-episode certs average 1.6% success (mostly exact 0/16, best-ever 3/16) despite holding 50% of all training sample mass throughout. It never reached the tangle wall (B12-15) any6/any7 both climbed to -- three rungs short, not just slower. Video (recover_det_17, the zero-bucket episode) confirms a genuine capability gap, not a cheat: robot starts flat, splays into a low crouch by frame 2, never completes the rise in the full 16s episode -- the same flat-rise-stall pathology named elsewhere in the campaign, no flag-leg/park/stilt. Single-sample harness eval agrees exactly: zero fails 0/1 det AND 0/1 sto in both DR-0 gate and own-DR-0.1 passes. Retention of buckets 0-10 looks solid at the FINAL read (>=0.8125, mostly 1.0) but a non-frontier bucket dipped below 0.8 in 14/28 rounds while B11 was frontier -- the no-forgetting promise partially held, not cleanly. This DOES answer any7's flagged open question: plant_catch (bucket 0) is NOT a real retention regression -- training cert shows it solid 16/16 throughout and the harness confirms 1/1 det (sto 0/18 on every bucket incl. easy ones is the already-documented any4 action-noise artifact, not new evidence). CONCLUSION: spaced replay (50/25/15/10 default masses, untuned) measures WORSE than any6/7's plain cert-gated curriculum for climbing the ladder -- confirms the pre-registered replay-tax if-false branch, and additionally exposes a THIRD wall (zero, B11) neither any6 nor any7 stalled on: over-concentrating replay mass on a stuck frontier is a plausible new suspect (one miss on this exact hypothesis, not two -- no resample without a mechanism change).

