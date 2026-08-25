# cw-standwalk-stance-mesh2-holdheight-rung3-hha1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-25T22:57:07+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung2-hha1-s1

**wandb_id**: kpgowofs

**hypothesis**: STAND_HEIGHT rung 3: can the height-commandable stance track the full kind mix (hold/ramp/SINE/PULSE) at the default 15mm/s rate — a genuine joystick up/down replay target — without re-buying the leg-unload cheat? Warm-start = rung-2 seed-1 checkpoint (the cleaner of the rung-2 pair: 12/12 DR-0, det duty all 1.0, det current fully inside the rung-1 band). Everything else identical to rung-2 incl. height-aware BC anchor (hha=1) and [-40,20]mm range; the only levers are kinds+rate. Judged jointly with -s1 as a 2-seed pass-rate pair.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Joint 2-seed pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, ZERO hold_min_load terminations, cur_max within noise of the rung-2 pair's det 0.62-1.06A / sto <=1.38A envelope. TRIPWIRE (registered at the rung-2 joint call): any per-leg duty <0.85 OR any hold_min_load termination at DR-0 in EITHER seed fires the S-gate/min-load-pricing fallback immediately.

**verdict**: CANARY PASS - MECHANISM, and the RUNG-3 JOINT CALL with -s1 lands: PASS, tripwire NOT fired. The height-commandable stance rides the full hold/ramp/sine/pulse command mix at 15mm/s on a clean level six-foot stand. Evidence: DR-0 12/12 valid_plant, ZERO hold_min_load terms, h_err_end 0.1-2.8mm, det Imax 0.75-0.82A (inside the rung-2 det 0.62-1.06A band), worst per-leg duty 0.98 (tripwire threshold 0.85); det+sto frame strips level/planted, no flag leg. Caveat: one sto ep Imax 1.50A vs the <=1.38A rung-2 sto envelope - single episode, duty >=0.98, roll clean, current-noise not the unload cheat. Own-DR 0.2: 11/12 with one det hold_min_load term - the same-magnitude residual as rung-2, DR hardening still the open gap. Why PASS: both gate clauses met on both seeds and the rung-2 trace duty dips SHRANK (0.87-0.89 -> 0.95-0.98 floors). Next: reward still rising at cutoff (quarters 20.5/50.2/79.2/176.4) -> per the 08-21 ruling fund the 8M acquisition continuation pair from this rung-3 ladder point targeting the own-DR residual; rungs 4-5 unblock behind the rise/stancemix joint calls owned by concurrent cycles.

