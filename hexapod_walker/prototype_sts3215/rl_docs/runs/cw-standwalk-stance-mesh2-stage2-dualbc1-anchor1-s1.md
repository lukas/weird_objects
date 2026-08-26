# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-26T14:13:59+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1

**wandb_id**: g6hghec7

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1 (identical recipe, seed 1) -- same run for the cross-seed pass-rate reading the joint-call convention requires before promoting or refuting the stance-only/walk-off bc_anchor fallback.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same as anchor1 seed 0; joint call reads both seeds together.

**verdict**: CANARY FAIL - INFRASTRUCTURE: trainer deadlocked immediately after the 1,003,520-step periodic eval + video reel (log frozen at 14:25:09, GPU 0%, CPU ticks flat over 20s, 768 threads, zombie eval workers). Mechanism health NOT judged — checkpoint at 1,048,576 steps pulled to controller (md5 2ca62910, matches pod) and relaunched as -r1 for the remaining ~1M steps. Joint canary call with anchor1 seed 0 proceeds on -r1's endpoint.

