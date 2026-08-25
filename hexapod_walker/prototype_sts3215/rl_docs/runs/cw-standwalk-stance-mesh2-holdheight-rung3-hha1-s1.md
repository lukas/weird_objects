# cw-standwalk-stance-mesh2-holdheight-rung3-hha1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T23:01:15+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung3-hha1

**wandb_id**: nws2zs8y

**hypothesis**: Seed twin (seed 1) of cw-standwalk-stance-mesh2-holdheight-rung3-hha1: does the full kind mix (hold/ramp/SINE/PULSE) at 15mm/s stay clean of the leg-unload cheat cross-seed? Same warm-start (rung-2 seed-1 checkpoint, the cleaner of the rung-2 pair), same cfg, only the seed differs. Judged jointly with seed 0 as a 2-seed pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Joint 2-seed pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, ZERO hold_min_load terminations, cur_max within noise of the rung-2 pair's det 0.62-1.06A / sto <=1.38A envelope. TRIPWIRE (registered at the rung-2 joint call): any per-leg duty <0.85 OR any hold_min_load termination at DR-0 in EITHER seed fires the S-gate/min-load-pricing fallback immediately.

