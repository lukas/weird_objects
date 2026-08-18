# cw-recover-any5-mjxcert-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-15T22:35:45+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any4-b0scratch1

**wandb_id**: 1tmepjxh

**hardware_ready**: False

**hypothesis**: The recovery robot already learned to catch itself but training never gave it credit: any4's deterministic policy passed B0-B4 on the reference simulator while the noisy exploration rollouts that governed the curriculum scored zero (action jitter ~0.38 std kept unloading a foot and resetting the strict 0.5s six-foot hold). This run repeats the exact from-scratch recovery recipe with promotion judged ONLY by a deterministic 8-env certification pool on the exact training backend (Warp/MJX), one assay per active family every 1M steps. OPERATOR-ORDERED (fb_20260815T222943_d019de, commit 3589f418 'Use deterministic MJX recovery certification'): genuinely FROM SCRATCH — no --init-from, no --obs-pad-transplant, no checkpoint load of any kind; parent field is recipe provenance only. Clones any4's complete 40M MDP/PPO recipe exactly plus --recover-cert-every 1000000 --recover-cert-envs 8. Prediction-if-true: CERT/recover_bucket_0_success appears at ~1M with 8 episodes and passes, CERT/recover_frontier_after=1, training env frontier promotes to 2 families/B1. Prediction-if-false: CERT B0 fails while background C-MuJoCo SCORE B0 passes — a Warp-vs-C backend mismatch; no promotion, report it.

**gate**: HARD integration gate at ~1M steps: CERT/recover_bucket_0_success must be PRESENT with an 8-episode denominator. PASS: CERT B0 succeeds AND CERT/recover_frontier_after=1 AND training env/recover_active_families/frontier become 2/B1. If CERT B0 fails while C-MuJoCo SCORE B0 passes: do NOT promote — report Warp-vs-C backend mismatch. Stochastic env/recover_rollout_ema_* and SCORE/recover_* (C-MuJoCo assay) are telemetry only, never admission signals. Full-arm PASS bar: video-verified genuine recover-to-stand (all six feet loaded, no flag/stilt/park) on the legitimately earned frontier by 40M with per-bucket CERT denominators reported.

**verdict**: STOPPED at ~7.7M/40M by operator directive fb_20260815T230538_a6f8d2: preserved as the coarse-bucket deterministic-cert diagnostic. Not continued across the 3d556232 curriculum change (bucket identities B0-B16 redefined; old CERT/*_success metric names retired). Superseded by cw-recover-any6-microbuckets-scratch1 (from scratch on the micro-bucket ladder).

