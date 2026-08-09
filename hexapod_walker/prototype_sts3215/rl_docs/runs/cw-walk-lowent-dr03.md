# cw-walk-lowent-dr03

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T01:42:21+00:00

**pod**: hexapod-sweep-long5m

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_step0_lowent.zip

**wandb_id**: xw5pdtum

**hypothesis**: 0-b rung 2: the step0-lineage gait survives moderate DR introduced one rung at a time (0.3) on a warm start from the lowent champion (md5 923ee55c) - DR at this scale robustifies rather than destroys (hist8 destruction came from jumping straight to wide distribution). One variable vs parent: --no-dr -> --dr-scale 0.3. If-true: DR0.3 harness gait_valid 12/12, forward >=0.10m 12/12, zero falls; DR0 retention no worse than parent. If-false: park/flag/falls at DR0.3 (gait_valid <10/12 or terminations) -> rung too big, drop to 0.15; OR DR0.3 passes but DR0 retention erodes -> warm-start interference. Alt: survives via DR-specific slop - distinguished by video + slip/duty at both DR0.3 and DR0. Snapshot e5f0c3e.

**gate**: DR0.3 harness (own DR) 6eps/mode det AND sto: gait_valid 12/12 AND forward >=0.10m 12/12 AND >=2 swings/leg AND 0 terminations AND det slip mean <=0.93; plus DR0 retention eval (det slip/succ within noise of parent)

**verdict**: INCONCLUSIVE-INVALID (hypothesis untested; execution invalid). Eval-only: gate clauses mechanically PASS at DR0.3 (gv 12/12, fwd 0.34-0.48m, 0 terms, det slip 0.628<=0.93) and DR0 retention within noise/better (det slip 0.640, det succ 3/6 vs parent 0/12, sto 4/6 vs 10/12, 1/6 sto tripod park on camera). NOT HARDWARE-READY. No champion update; do not warm-start from this ckpt without citing RL_LOG cycle 20. Evals: logs/ckpt_eval/cw_walk_lowent_dr03_{gate,ret0}.

