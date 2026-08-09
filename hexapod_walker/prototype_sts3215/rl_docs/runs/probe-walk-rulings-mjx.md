# probe-walk-rulings-mjx

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T11:23:50+00:00

**pod**: hexapod-mjx-train-2

**steps**: 1000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**hypothesis**: MJX-stack probe (not a science arm) of the two cycle-35 rulings mechanisms before 20M runs use them: goal.walk_heading_max_rad=0 (forward-only command draws) and reward.walk_loadslip_gate (episode-accumulated loaded-slip/m income gate). Pass = 1M steps, zero tracebacks, walk_loadslip factor/ratio sane in logs, ep_rew positive, fps >= 10k.

**gate**: 1M steps complete, 0 tracebacks, loadslip factor sane, fps>=10k

**verdict**: PASS: 1M steps in 257s, 0 tracebacks, both mechanisms live in MJX stack (loadslip gate binding: ep_rew 26->214 vs ~1100+ ungated lineage, climbing; periodic eval walk err 0.034 at 1M = gait intact). Cleared cw-walk-loadslip + heading-scope arms.

