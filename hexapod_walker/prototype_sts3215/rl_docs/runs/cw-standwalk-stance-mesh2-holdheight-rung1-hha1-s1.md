# cw-standwalk-stance-mesh2-holdheight-rung1-hha1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T21:45:40+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung1-s1

**wandb_id**: gfksq1nx

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-holdheight-rung1-hha1 (same recipe, seed 1): does the height-aware BC anchor fix (bc_anchor_coef restored 0.0->3.0 + train.bc_anchor_hold_height_aware=1, FixedFootBodyIK target at the next commanded height instead of the constant q_nom) replicate cross-seed, not just for one init? Same fix, same holdheight-rung1-s1 FAIL evidence.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate as the seed-0 twin, judged jointly as a pass-rate pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, zero hold_min_load terminations, cur_max within noise of the champion's 0.67-0.71A band.

**verdict**: CANARY PASS - MECHANISM, with a residual caveat on this seed; JOINT PAIR CALL: PROCEED UP THE LADDER. The height-aware BC anchor rescued this seed from the gross flag-leg cheat too, but not completely: the robot rides the +/-15mm/8mm/s commanded-height elevator on a level six-foot stand (DR-0 12/12 valid_plant, needed 5/6+4/6; height_err_end 0.1-3.0mm; video level/planted/quiet), yet 3/6 det + 3/6 sto episodes still trip hold_min_load (registered strict criterion was zero). The residual is a SUBTLE leg-5 load-lightening — duty dips only to 0.57-0.9 with 4-30 micro-swings, and only on terminated episodes; 6/12 episodes are fully honest all-duty-1.0 holds — versus the pre-fix gross flag (leg 2 duty 0.23-0.70, up to 55 swings, 10/12 terms). det Imax 0.62-0.94A vs pre-fix 2.0-2.6A. Reward rose all run (quarters 15.7/48.3/71.0/123.3) BUT env/hold_load_factor drifted DOWN late (0.99->0.89 at locked std 0.018): the optimizer is slowly re-buying partial unloading, a residual incentive leak — so extending THIS seed is the wrong move (08-22 ruling: misaligned residual, not undertrained). REGISTERED JOINT CALL (lands with this run): seed 0 passed clean (0/12 terms, duty 0.91-1.0); pair verdict = mechanism PROVEN cross-seed (cheat magnitude collapsed on both seeds), strict zero-term criterion 1/2. Ruling: PROCEED — rung 2 (full [-40,20] range, same hold+ramp kinds) launches now as a 2-seed canary pair warm-started from seed 0's CLEAN checkpoint with bc_anchor_hold_height_aware=1 as the recipe default and the same zero-term gate; if rung 2 reproduces min-load dips on either seed, the registered S-gate/min-load-pricing fallback fires. hardware-ready: no (canary; own-DR hardening still open on both seeds).

