# cw-standwalk-stance-mesh2-holdheight-rung3-acq8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T23:35:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung3-hha1

**wandb_id**: ux23038c

**hypothesis**: Seed twin (seed 1) of cw-standwalk-stance-mesh2-holdheight-rung3-acq8m: does the 8M acquisition continuation of the rung-3 height-command recipe close the persistent 1/12 own-DR hold_min_load residual cross-seed, without re-buying the leg-unload cheat at DR-0? Same warm-start (rung-3 seed-0 ckpt, duty floor 0.98 - cleaner of the canary pair), same cfg, only the seed differs. Judged jointly with seed 0 as a 2-seed pair.

**gate**: Joint 2-seed pair: own-DR 0.2 12/12 valid_plant with ZERO hold_min_load terms (the target residual); DR-0 det>=5/6 + sto>=4/6 valid_plant, zero min-load terms, per-leg duty >=0.85 (rung-3 tripwire carried), det cur_max within the rung-2/3 0.62-1.06A band.

