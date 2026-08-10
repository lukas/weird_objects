# cw-arch-hist16-r7-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T06:21:46+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-r7

**wandb_id**: jvztwe1w

**hypothesis**: Seed twin of cw-arch-hist16-r7 (the first hist16 from-scratch attempt to actually boot after the /dev/shm root-cause fix, 3072 envs). P0 directive: freed GPU capacity goes to the dep line and the temporal-arch rung; operator authorized 1-2 pods on this line. r7 is mid-training (>20M/40M, reward climbing 66.9->895 across quarters) but unverified against its gate. A second seed run in parallel de-risks the eventual verdict (seed-robust vs seed-lucky bootstrap) without waiting idle GPUs for r7 to finish. If-true: matches r7's trend/gate once both finish. If-false: r7's boot/trend was seed-fragile -- treat as inconclusive pending a 3rd tiebreak, not a clean history16 result.

**gate**: Same gate as cw-arch-hist16-r7 (det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; bootstrap-failure vs history16-FAIL distinction applies); frames watched det

