# cw-standwalk-stance-mesh2-holdheight-rung2-hha1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T22:24:24+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung2-hha1

**wandb_id**: o8aq5c1a

**hypothesis**: Seed twin (seed 1) of cw-standwalk-stance-mesh2-holdheight-rung2-hha1: does the full [-40,20]mm commanded-height range (STAND_HEIGHT rung 2) stay clean of the leg-unload cheat cross-seed, from the same rung-1 seed-0 clean checkpoint with the height-aware BC anchor on? Rung-1 showed a seed-dependent residual (seed 1 kept 6/12 min-load terms); this twin measures whether rung-2 pass is seed-robust.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as rung 1, judged jointly as a 2-seed pass-rate pair with seed 0: DR-0 det>=5/6 + sto>=4/6 valid_plant, ZERO hold_min_load terminations, no per-leg duty sacrifice (all legs >=0.9), cur_max within noise of rung-1 seed 0's 0.66-1.03A band.

**verdict**: CANARY PASS - MECHANISM (seed 1) + REGISTERED RUNG-2 JOINT CALL: PASS, PROCEED (seed-0's verdict delegated the joint call here). Plain English: at the full [-40,20]mm commanded-height range the robot rides the height elevator on a clean level six-foot stand in BOTH seeds — the flag-leg cheat did not return. Seed-1 evidence: DR-0 gate 12/12 valid_plant (needed 5/6 det + 4/6 sto), ZERO hold_min_load terminations, height_err_end 0.2-3.2mm, det duty 1.0 on all legs all episodes, det cur_max 0.62-0.76A (fully inside rung-1 seed-0's 0.66-1.03A band; sto <=1.31A vs rung-1's own sto 1.38A), contact sheet level/planted/quiet. Trace residual: one sto episode dips one leg to duty 0.89 (gate letter >=0.9) — smaller than seed-0's 2/6 sto at 0.87/0.89; cross-seed the min-load cheat magnitude SHRANK vs rung-1 (terminations 6/12 -> 0, duty floor 0.57 -> 0.87). FALLBACK RULING (assume-and-go): the pre-registered S-gate/min-load-pricing fallback ('EITHER seed reproduces min-load dips') does NOT fire — dips are shrinking trace artifacts with zero terminations, not reproductions of the rung-1 cheat; instead rung-3 carries an explicit tripwire (any duty <0.85 OR any hold_min_load term at DR-0 fires the fallback immediately). Own-DR(0.2): 11/12, one det min-load term (legs 4/5 duty 0.45/0.51, 2.1A) — better than rung-1 seed-0's 2/6 det trips; own-DR hardening stays registered later work. Reward rose all run (9.0/56.9/91.0/183.6), no misalignment signature. NEXT: rung-3 canary pair (full kind mix hold/ramp/sine/pulse at 15mm/s per STAND_HEIGHT.md) warm-started from THIS seed-1 checkpoint (cleaner of the pair: det duty all 1.0, det current fully in-band).

