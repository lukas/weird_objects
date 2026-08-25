# cw-standwalk-stance-mesh2-holdheight-rung3-hha1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T22:57:07+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung2-hha1-s1

**wandb_id**: kpgowofs

**hypothesis**: STAND_HEIGHT rung 3: can the height-commandable stance track the full kind mix (hold/ramp/SINE/PULSE) at the default 15mm/s rate — a genuine joystick up/down replay target — without re-buying the leg-unload cheat? Warm-start = rung-2 seed-1 checkpoint (the cleaner of the rung-2 pair: 12/12 DR-0, det duty all 1.0, det current fully inside the rung-1 band). Everything else identical to rung-2 incl. height-aware BC anchor (hha=1) and [-40,20]mm range; the only levers are kinds+rate. Judged jointly with -s1 as a 2-seed pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Joint 2-seed pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, ZERO hold_min_load terminations, cur_max within noise of the rung-2 pair's det 0.62-1.06A / sto <=1.38A envelope. TRIPWIRE (registered at the rung-2 joint call): any per-leg duty <0.85 OR any hold_min_load termination at DR-0 in EITHER seed fires the S-gate/min-load-pricing fallback immediately.

