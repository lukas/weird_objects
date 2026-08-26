# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1-s1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-26T14:32:34+00:00

**pod**: hexapod-mjx-train-1

**steps**: 1000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1-s1

**wandb_id**: zuearqs7

**hypothesis**: Infra-retry (once) of anchor1 seed 1: the original hung in a post-eval deadlock at 1.05M/2M steps with training healthy (6.1k env-steps/s, ckpt written); resuming the remaining ~1M steps from that checkpoint completes the seed-1 arm of the stance-only/walk-off bc_anchor fallback cross-seed canary.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as anchor1 seed 0; joint call reads both seeds together (this run's endpoint stands in for seed 1).

**verdict**: CANARY FAIL - INFRASTRUCTURE (superseded, discarded, no skill data): killed at own-step 1.05M, ~2 min into resume, negligible compute lost. Root cause of the retry (a "deadlock" observation on the parent anchor1-s1 run) was itself a false-positive per fresh W&B evidence gathered this cycle — the ORIGINAL anchor1-s1 run (wandb g6hghec7) shows a clean state=finished at global_step 2031616 with a full 787-row history and a real matching-size exported checkpoint (byte-identical size to seed0 anchor1), the same false-positive pattern already documented in STATUS.md for modeseq1-s1r one lineage step earlier. The original anchor1-s1 checkpoint/evals are the real seed-1 endpoint for the anchor1 joint call; this retry never trained long enough to contribute anything and is discarded.

