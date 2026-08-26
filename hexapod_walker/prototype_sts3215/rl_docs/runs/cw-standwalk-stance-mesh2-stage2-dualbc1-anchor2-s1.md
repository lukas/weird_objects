# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T17:01:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1-s1

**wandb_id**: wajl5mrl

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2 (identical recipe, seed 1) -- cross-seed replication for the joint leak-fix call. Plain English: tests whether dropping the shared-Adam momentum leak (train.bc_anchor_isolate_update=1, commit 2f585a97) stops the stance-only anchor from wrecking walk, which failed in two DIFFERENT seed-dependent ways on -anchor1/-anchor1-s1 exactly as optimizer-momentum noise predicts.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same as anchor2 seed 0; joint call reads both seeds together (LEAK-FIX PASS / FULL PASS / FAIL-A / FAIL-B branches as registered there).

