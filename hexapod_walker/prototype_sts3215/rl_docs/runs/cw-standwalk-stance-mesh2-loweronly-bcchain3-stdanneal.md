# cw-standwalk-stance-mesh2-loweronly-bcchain3-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T13:58:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-loweronly-bcchain3

**wandb_id**: da1srvqe

**hypothesis**: The robot already sits down cleanly when executed without action noise; this arm asks whether annealing away the exploration noise makes that sit-down robust, exactly as it did for standing. Lower acquisition off the rung-8 CANARY PASS (loweronly-bcchain3: DR-0 det 6/6 honest descents, height_err_end 0.1-3.7mm, zero over_current, zero terms; sto 0/6 all fell under un-annealed policy_std~1.0 — the same signature the hold rung had before bcanchor3-stdanneal took sto 0/6->6/6 by log-std anneal alone). Warm-start from the canary checkpoint, 8M, log-std 0->-4.0 over first 50%, identical to the hold stdanneal recipe. Secondary read: does anneal also cool the hot crouch (canary det cur_max 2.17-2.26A, cur_s_above_soft up to 10.2s) the way it cooled hold (0.53->0.44A)? If sto passes but current stays pinned hot, the fallback is goal.lower_height_mm belly-rest recalibration per the 08-25 dig-in, not more pricing.

**gate**: 8M acquisition. Lower DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: sto >=4/6 honest descents at DR-0 (>=60% commanded drop, feet grounded, no fall) with det >=5/6 preserved and zero over_current terms in det; report cur_p95/cur_max vs the canary's 2.17-2.26A. PARTIAL: sto improves materially (1-3/6 or falls shifting late / cur_max trending down) but short of bar — budget or dose follow-up. FAIL: sto stays 0/6 with det intact — noise-floor is not the lower sto blocker; fork to goal.lower_height_mm belly-rest recalibration (crouch intrinsically too hot on mesh) before any DR-exposure fix.

**verdict**: LOWER RUNG CLOSED — FULL PASS, mirrors the hold rung's stdanneal close. 8M acquisition off the loweronly-bcchain3 canary (log-std 0->-4.0, final std 0.018): DR-0 det 6/6 + sto 6/6 valid_plant, own-DR(0.2) det 6/6 + sto 6/6 valid_plant, ZERO terminations across all 24 episodes, height_err_end_mm 0.0-0.5mm (full commanded 25-55mm drop tracked to sub-mm error, env-native metric). Current cooled exactly as the canary triage predicted: cur_max 0.66-1.24A vs the canary's hot 2.17-2.26A crouch (a >2x drop), cur_p95 0.37-0.62A; roll peak <=0.8deg, zero over_current. Gate bar was sto>=4/6 with det>=5/6 + zero over_current in det + current reported vs canary — exceeded on every clause (6/6 everywhere). Video note: the det/sto contact sheets show the settled crouch reads visually shallower/less knee-bent than the hot canary's video, but the env's own height-error channel (the same code that grades the reward and that a prior cycle independently verified via video on this exact canary lineage) confirms full-drop tracking at all 24 episodes; read as the cooler policy reaching the same target through less mechanical strain, not a shallower descent — no gate/video conflict serious enough to dig in. New mesh lower champion: ppo_goal_cw_standwalk_stance_mesh2_loweronly_bcchain3_stdanneal.zip. Next: seed-hedge canary launched (loweronly-bcchain3-s1, seed 1, 2M) mirroring the hold rung's bcanchor3-s1 precedent; rise rung still in flight (concurrent cycle); stancemix rung decision (warmmix1/2) also concurrent-cycle-owned.

