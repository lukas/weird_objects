# cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-26T11:54:55+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1 (identical recipe, seed 1 only) -- same run for the cross-seed pass-rate reading the joint-call convention requires before promoting a stage-2 recipe.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as -modeseq1 seed 0; joint call reads both seeds together.

**refused_reason**: hexapod-mjx-train-1 code marker 945b12f9056462579be7e118aa27d9ecd46e592c-dirty != local HEAD 945b12f9056462579be7e118aa27d9ecd46e592c and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).

