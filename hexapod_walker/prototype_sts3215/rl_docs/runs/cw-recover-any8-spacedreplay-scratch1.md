# cw-recover-any8-spacedreplay-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-16T04:51:59+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-recover-any6-microbuckets-scratch1

**wandb_id**: qe35tc9m

**hypothesis**: Teach the fallen robot to stand back up without forgetting the easy cases as it learns the hard ones; this arm tests whether spaced replay of earlier difficulty buckets (50% frontier / 25% previous-3 / 15% weakest certified old bucket / 10% remaining history, monotonic unlock) plus the new W&B recovery scorecard lets a from-scratch policy climb the 17-rung micro-bucket ladder while KEEPING every rung it has certified. OPERATOR-ORDERED (MCP operator lane, Codex relaying Lukas, 20260816T045031Z; commit c5d2e0e8 'Add spaced replay and recovery scorecard'): exact cw-recover-any6-microbuckets-scratch1 recipe/config/start-bank, genuinely FROM SCRATCH — no --init-from, no checkpoint transplant, no warm start; any6 is recipe provenance only. New vs any6: bucket-level spaced-replay sampling masses as above, 16 cert envs (was 8), retention-buckets=3, and the RECOVER_SCORE/* scorecard (overall_points, overall_weighted_success, max_unlocked_bucket, per-bucket success fractions, bucket sample probabilities) alongside CERT frontier/retention assays. Prediction-if-true: the frontier climbs while previously certified buckets hold >=0.8 on retention assays (no catastrophic forgetting), visible as monotone RECOVER_SCORE/overall_points. Prediction-if-false: replay mass on old buckets starves frontier learning and the ladder climbs slower than any6/any7 at matched steps — which prices the replay tax. Existing any7 continues as the old-sampler control; this run is not a substitute for it.

**gate**: HARD integration gate at the first ~1M certification: W&B must contain RECOVER_SCORE/overall_points, RECOVER_SCORE/overall_weighted_success, RECOVER_SCORE/max_unlocked_bucket, RECOVER_SCORE/bucket_00_success_fraction, and bucket sample probabilities, plus CERT frontier/retention assays with 16-episode denominators. Full-arm PASS bar: legitimately earned frontier (>=0.8 deterministic cert fraction per promotion) at or past any6's best rung by 40M WITH all previously certified buckets retained >=0.8 on cert assays, and video-verified genuine recover-to-stand (all six feet loaded, no flag/stilt/park) on the earned frontier.

