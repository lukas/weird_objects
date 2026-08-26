# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor3-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T19:20:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**wandb_id**: 95em5s9b

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-anchor3 (identical recipe, seed 1) -- cross-seed replication for the joint dose-response call. Plain English: tests whether doubling the stance-only BC anchor coef (3.0->6.0), now that the shared-Adam momentum-leak fix (isolate_update=1) is confirmed on both seeds, finally protects hold/lower (stuck at a clean total hold_min_load term 6/6 both DR in anchor2/-s1) without re-wrecking walk.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same as anchor3; joint call reads both seeds together (LEAK-STAYS-FIXED PASS / DOSE-WORKS / FAIL branches as registered there).

