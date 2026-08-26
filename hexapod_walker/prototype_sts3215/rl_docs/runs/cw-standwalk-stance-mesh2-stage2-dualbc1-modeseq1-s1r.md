# cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1r

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T12:15:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1

**wandb_id**: gmtsnhem

**hypothesis**: Retry of an infra-hung run: does the modeseq1 stage-2 recipe hold up on a second seed? This is a byte-identical relaunch (seed 1) of cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1, which froze in a sporadic eval/video deadlock at its 1M-step eval boundary (no science signal; seed 0 finished normally). Prediction-if-true: run completes 2M steps with healthy canary probes like seed 0. Prediction-if-false: a second hang at the same boundary = reproducible infra bug, escalate instead of retrying again. Strongest alternative: the hang was seed-correlated compute load, not a race.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as -modeseq1 seed 0; joint call reads both seeds together.

