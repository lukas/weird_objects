# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T17:31:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor11-walkretain-s1

**wandb_id**: b9y6ry0s

**hypothesis**: Seed1 twin of anchor14-walkretaincoef1-rescue: same single-lever change (train.bc_anchor_walk_coef=1.0) on anchor6b-logstdsplit-fix-s1's checkpoint -- THIS is the seed that actually carries the anchor4-class catastrophe (gait_valid 0/6, sacrificed legs [0,1,2,4,5]) the whole rescue thread targets, so this run's own read is the most decision-relevant half of the joint call.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate as cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue (read that run's ledger text) -- this is the seed1 half (the catastrophe seed itself) of the joint 2-seed RESCUE-PASS/PARTIAL/FAIL call.

