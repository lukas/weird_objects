# cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-26T11:59:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**wandb_id**: v38ba434

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1 (identical recipe, seed 1 only) -- same run for the cross-seed pass-rate reading the joint-call convention requires before promoting a stage-2 recipe.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as -modeseq1 seed 0; joint call reads both seeds together.

**verdict**: CANARY FAIL - INFRASTRUCTURE: the trainer froze at the 1,003,520-step periodic eval/video boundary — W&B global_step, the log, process CPU time (stuck at 6:00), and GPU util (0%) all flat for 4+ minutes while the process stayed alive; watcher checkup flagged SUSPECT at 12:09. Seed-0 twin (same recipe) finished normally at 2.03M, so this is a sporadic eval/video-render deadlock, not a config or mechanism problem. Killed the process tree on hexapod-mjx-train-1 (checkup rule: retry once) and queued an identical seed-1 retry as cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1r so the joint two-seed call can proceed. Last logged eval before the hang (info only, canary scope): rise f1/2 b1/2 c1/2, lower 2/2, walk dir err ~52deg.

