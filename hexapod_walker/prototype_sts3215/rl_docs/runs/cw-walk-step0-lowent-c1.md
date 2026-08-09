# cw-walk-step0-lowent-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T01:22:51+00:00

**pod**: hexapod-sweep-s6

**steps**: 4000000

**parent**: cw-walk-step0-lowent

**wandb_id**: uwi188rd

**checkpoint**: rl_move/sim/policies/ppo_goal_cw_walk_step0_lowent_c1.zip (md5 104ba8cf300d979e1c97979df794a0d9, pod s6 + controller copy)

**hypothesis**: Operator directive 0-a continuation of the A/B WINNER. lowent (ent 0.001) finished its 4M segment at ep_rew_mean 688 vs sibling c2 (ent 0.01) at 580, and c2 declined vs its own parent (591->580) with std runaway to ~5.9 — preliminary support for the entropy-runaway hypothesis. Continue the winner unchanged: init-from lowent final ckpt, identical config, +4M. Predict: reward keeps rising with stable std. Trailing verdict cycle owns the formal A/B verdict and may kill this run on video pathology.

**gate**: Same as step0: DR 0, det AND sto, >=10cm forward, all six legs cycling lift/swing/touchdown, duty [0.2,0.9], >=2 swings/leg, no drag, no park; video pathology-first

**verdict**: PASS vs recorded step0 gate (cycle 19): 5s DR0 harness det gv 6/6 fw 0.23-0.47m swings>=3, sto 6/6 succ gv 6/6; one det ep duty 0.18 marginal (same treatment as parent's 0.18-0.83). vs parent lowent (named baselines): det slip mean 0.674 vs 0.746/0.717 (ranges overlap), det succ 2/6 vs 0/12 pooled (p~0.08, borderline), sto 6/6 vs 10/12 (within noise), rew 688->~730 (+42, identical cfg). 15s probe: det gv 6/6, sto 5/6 with ONE TRIPOD PARK ep (duty 0.99/0.0/1.0/0.01/0.99/0.01, fw 0.014m) - park attractor still reachable under sampling noise, same 1/6 invalid rate as parent. Q4-Q3 rew delta +3.3 (inside W&B scatter +-15) = trend FLAT; identical-config continuation lineage CLOSED, no c2. CHAMPION: NOT updated - c1 >= lowent only within noise (det 2/6 vs 0/12 p~0.08 borderline; slip ranges overlap), and the concurrent h15b verdict (same wall-clock hour) beats c1 outright (5s det slip 0.584 vs 0.674 non-overlap, det succ 4/6 vs 2/6, std 1.705 vs 1.94). Walk champion = h15b md5 d0a12a94. NOT HARDWARE-READY: DR0 only, det overspeed ~4x command (v0.078 vs ref 0.020 on overlay), skating 0.58-0.78 m/ep, irregular cadence, policy std 1.94. HYPOTHESIS (reward keeps rising w/ stable std): SUPPORTED - 697->728.5 quarters, std 2.08->1.94; rise decelerated to noise by Q4.

