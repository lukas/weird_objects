# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor5-stdmild1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T00:51:53+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal-s1

**wandb_id**: duwxunzn

**hypothesis**: Seed-1 twin of anchor5-stdmild1 (see that entry for the full dose-bracket hypothesis): --log-std-final -1.0 on top of anchor4-stdanneal-s1's own leak-fixed coef=3.0 dual-core checkpoint, testing whether this soft a dose protects walk while still helping hold, on the second seed.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same panel as anchor5-stdmild1. WALK-SURVIVES if det gait_valid >=5/6 (no anchor1/anchor4-class freeze, prog_ratio >=~0.2). HOLD-HELPS if hold/sto DR-0 term improves meaningfully from anchor2-s1/anchor3-s1's 6/6 baseline. JOINT call with anchor5-stdmild1 (seed0) and anchor5-stdmild2/-s1 (the -2.0 dose pair) decides the dose-response: both doses protect walk on both seeds -> pick the dose with more hold improvement and promote; either dose wrecks walk on either seed -> magnitude is closed as a lever, per-core log_std split (DIG-IN) is next.

