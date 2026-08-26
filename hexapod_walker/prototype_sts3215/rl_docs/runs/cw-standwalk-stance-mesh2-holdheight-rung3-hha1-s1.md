# cw-standwalk-stance-mesh2-holdheight-rung3-hha1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-25T23:01:15+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung3-hha1

**wandb_id**: nws2zs8y

**hypothesis**: Seed twin (seed 1) of cw-standwalk-stance-mesh2-holdheight-rung3-hha1: does the full kind mix (hold/ramp/SINE/PULSE) at 15mm/s stay clean of the leg-unload cheat cross-seed? Same warm-start (rung-2 seed-1 checkpoint, the cleaner of the rung-2 pair), same cfg, only the seed differs. Judged jointly with seed 0 as a 2-seed pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Joint 2-seed pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, ZERO hold_min_load terminations, cur_max within noise of the rung-2 pair's det 0.62-1.06A / sto <=1.38A envelope. TRIPWIRE (registered at the rung-2 joint call): any per-leg duty <0.85 OR any hold_min_load termination at DR-0 in EITHER seed fires the S-gate/min-load-pricing fallback immediately.

**verdict**: CANARY PASS - MECHANISM (seed twin; joint rung-3 call made on the seed-0 entry: PASS, tripwire NOT fired). The full kind mix stays clean of the leg-unload cheat on this seed too. Evidence: DR-0 12/12 valid_plant, ZERO hold_min_load terms, h_err_end 0.2-2.8mm, det Imax 0.70-0.82A and sto <=1.23A (fully inside the rung-2 envelope, cleanest current of any rung); worst per-leg duty 0.95 with 12 micro-swings on leg idx5 in one sto ep - trace-level, sub-visual on the frame strip, zero terms, smaller than rung-2's 0.87-0.89 dip family. Own-DR 0.2: 11/12, one det hold_min_load term (Imax 1.64A, h_err 9.7mm) - same residual as seed 0 and rung-2. Video level/planted throughout. Reward rising at cutoff (20.8/56.6/92.1/157.8). Next: covered by the joint call - 8M acquisition continuation pair warm from the rung-3 seed-0 ckpt (higher duty floor of the pair).

