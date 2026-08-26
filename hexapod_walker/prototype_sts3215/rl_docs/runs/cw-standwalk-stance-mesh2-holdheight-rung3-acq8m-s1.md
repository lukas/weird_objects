# cw-standwalk-stance-mesh2-holdheight-rung3-acq8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T23:35:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung3-hha1

**wandb_id**: ux23038c

**hypothesis**: Seed twin (seed 1) of cw-standwalk-stance-mesh2-holdheight-rung3-acq8m: does the 8M acquisition continuation of the rung-3 height-command recipe close the persistent 1/12 own-DR hold_min_load residual cross-seed, without re-buying the leg-unload cheat at DR-0? Same warm-start (rung-3 seed-0 ckpt, duty floor 0.98 - cleaner of the canary pair), same cfg, only the seed differs. Judged jointly with seed 0 as a 2-seed pair.

**gate**: Joint 2-seed pair: own-DR 0.2 12/12 valid_plant with ZERO hold_min_load terms (the target residual); DR-0 det>=5/6 + sto>=4/6 valid_plant, zero min-load terms, per-leg duty >=0.85 (rung-3 tripwire carried), det cur_max within the rung-2/3 0.62-1.06A band.

**verdict**: Pre-registered fail branch, mirrors seed-0 exactly — JOINT CALL: rung-3 tripwire FIRES on both seeds. Plain result: the 8M acquisition continuation made seed-1's rung-3 height-command stance WORSE, not more robust. Evidence: DR-0 gate det 6/6 + sto 6/6 valid_plant with ZERO hold_min_load terminations (letter-clean) BUT breaches the tripwire on both named clauses — 2/6 sto episodes duty_min 0.84 (<0.85 floor) and cur_max climbs to 1.08-1.72A, well above the rung-2/3 0.62-1.06A band; own-DR 0.2 = 4/12 hold_min_load terms (2 det + 2 sto, duty crashing to 0.17/0.37/0.62/0.76) vs the 2M canary's 1/12 — the residual got WORSE with 4x budget, not better. Video (hold_det_5, TERM hold_min_load) shows the same single-leg unload/lift signature named at rung 1/2. Reward rose all run (quarters 76.5/325.2/563.3/715.8) while eval/tripwire regressed => 08-21 MISALIGNMENT branch, identical shape to seed-0's own verdict (79.8/316.9/545.0/709.3 rising, tripwire fired). JOINT CALL: fires the registered S-gate/min-load-pricing fallback (either-seed tripwire condition, now met by BOTH seeds independently) — extended optimization on this recipe re-buys the leg-unload cheat rather than curing it. Rung-3 champion stays the 2M canary ckpt pair (hha1/-s1); neither 8M ckpt is promoted. Next lever is reward/env code (price the S-gate/min-load margin directly, not more budget) — DIG-IN already flagged by seed-0's cycle, this closes the joint bookkeeping only, no new dig-in needed. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_holdheight_rung3_acq8m_s1_{gate,owncfg}/, W&B ux23038c.

