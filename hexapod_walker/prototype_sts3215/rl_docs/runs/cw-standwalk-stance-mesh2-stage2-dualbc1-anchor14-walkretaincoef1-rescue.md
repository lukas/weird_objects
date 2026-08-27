# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T17:27:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor11-walkretain

**wandb_id**: o4evhjg7

**hypothesis**: Plain English: anchor11-walkretain's own seed0 read this cycle (not the catastrophe seed, but a real cost: prog_ratio 0.23->0.08, slip/m 6.56->13.64) showed the SAME shared coef=3.0 dose taxing walk on the RESCUE recipe too, mirroring anchor12's control on the other recipe family. This arm is anchor11-walkretain's exact recipe (init-from anchor6b-logstdsplit-fix's checkpoint, the hold-fixing/walk-catastrophe-prone pair) with ONE change: train.bc_anchor_walk_coef=1.0 instead of the shared 3.0. The decisive question this recipe alone can answer (unlike anchor13's control-recipe twin): does the LIGHTER decoupled dose still rescue seed1's real 0/6 gait_valid collapse, or does the anchor need the heavier dose to overpower the catastrophe -- i.e. is there a dose window that rescues seed1 without taxing seed0 the way coef=3.0-shared did?

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M, joint call with -s1 twin; matched controls = anchor6b-logstdsplit-fix{,-s1} (the untouched parent: seed0 clean prog_ratio 0.23, seed1 catastrophe gait_valid 0/6 sacrificed [0,1,2,4,5]) AND anchor11-walkretain{,-s1}'s own coef=3.0-shared numbers. WIRING CHECK: train/bc_anchor_loss_walk + train/bc_anchor_fill_walk nonzero. RESCUE-PASS if seed1 (the catastrophe target) shows gait_valid>=5/6 with zero sacrificed legs AND seed0 stays close to its OWN parent's clean band (prog_ratio >=0.18, i.e. most of anchor11-walkretain's cost recovered) -- the dose window exists, fund a 2x-seed acquisition wave. RESCUE-PARTIAL if seed1 improves (gait_valid up, fewer sacrificed legs) but doesn't fully clear the bar, or seed0's cost isn't recovered -- fund a middle dose (e.g. 2.0) next. FAIL if seed1 still shows the anchor4-class collapse (gait_valid 0-1/6, sacrificed legs) at this lighter dose -- the walk anchor mechanism itself cannot rescue this catastrophe class at any dose tried so far; escalate to the per-mode-group objective normalization item (-2.35a) or the reward/task-level audit. Read reward trend per 08-21 first.

