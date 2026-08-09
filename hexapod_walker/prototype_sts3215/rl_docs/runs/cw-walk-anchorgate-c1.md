# cw-walk-anchorgate-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T09:04:02+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: rximxa7s

**hypothesis**: Consolidate-in-place continuation of cw-walk-anchorgate (gate FAIL but near-miss: DR1.0 det slip 1.240 vs champion 1.543 beyond per-ep noise, anchor_frac 0.767->0.837 monotone and still climbing at run end, std 1.375->1.306 still annealing, no if-false shape fired). HYPOTHESIS: anchoring is still on a live gradient; +20M identical-config steps push walk_anchor_frac >0.85 EARNED and drop DR1.0 agg slip/m to <=1.0 in BOTH passes (sto follows det as std anneals below ~1.2). ONE variable vs parent: +20M steps. Prediction-if-true: gate passes (det AND sto slip <=1.0, gv 12/12, 0 term, DR0 retention holds). Prediction-if-false: anchor_frac plateaus <=~0.85 and det slip stays >1.0 => residual creep is structural under tol=10 for warm starts => warm-start rung CLOSED (two misses = change hypothesis, no c2), answer moves to the fresh-init arm cw-walk-step0-anchor and the pre-registered single audit-driven tolerance correction. Also false: retention erosion (DR0 det fwd mean <0.55 or gv failure) => quarantine. Strongest alternative: det improvement was eval-panel luck - distinguished by the same fixed panel re-run at 40M total: luck regresses to champion band, real gradient continues the frac/slip trend. Parent ppo_goal_cw_walk_anchorgate.zip md5 35234ddc151ac6f7e05350b7c550efb7 (on pod).

**gate**: DR1.0 harness 15s own-cfg 6+6: agg slip/m det<=1.0 AND sto<=1.0, gv 12/12, 0 term; DR0 harness 15s: det fwd mean >=0.55, gv 12/12, fwd-hemisphere sto fwd>=0.40 5/5 (backward draw recorded-excluded pending operator ruling); frames: anchoring vs paddling; walk_anchor_frac trend vs plateau

**verdict**: FAIL on gate (DR1.0 agg slip/m det 1.283 / sto 1.326 vs <=1.0; parent 1.240/1.245 — no improvement, delta inside per-ep noise). anchor_frac 0.906 EARNED (>0.85) => parent shape (c) fired: tolerance mis-set. Mechanism: cadence inflation exploit — det stances 47->58 (+23%), reward_step_event +24%; per-touchdown allowance reset makes free slip = cadence x tol; tol=10 floor 0.80-0.94/m at c1 cadence >= the 1.0 gate. DR0 retention passes (det fwd 0.731, gv 24/24, 0 term, fwd-hemi sto 5/5). CHAMPION UNCHANGED (anchorgate 35234ddc). Consolidation hypothesis REFUTED: frac gradient live, slip did not follow. Pre-registered audit-driven tolerance correction (tol 10->5) is the follow-up; NOT HARDWARE-READY (slip 1.28 m/m).

