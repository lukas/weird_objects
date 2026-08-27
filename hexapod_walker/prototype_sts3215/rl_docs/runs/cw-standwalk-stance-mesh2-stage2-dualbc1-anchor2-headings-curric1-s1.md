# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings-curric1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T06:26:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1-s1

**wandb_id**: 93z0cisc

**hypothesis**: Seed-1 twin of anchor2-headings-curric1 (see that entry for the full hypothesis): does ramping the heading cone 0->0.7854 over the first 1.2M of a 2M continuation (instead of opening it fully at step 0) let the walk core learn to steer, tested on the second seed off anchor2-s1's own leak-fixed checkpoint.

**gate**: Same panel and WALK-SURVIVES / DIRECTION-LEARNS-FULL clauses as anchor2-headings-curric1; JOINT call with that seed-0 twin per that entry's promote/close branches.

