# probe-walk-kgate

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T02:54:44+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowent_h15b_c1.zip

**hypothesis**: Mechanical probe of reward.walk_kernel_prog_gate=1.0 on the c1 warm start: walk_prog_factor present in logs, env/reward_walk reduced vs c1-era ~1.6-1.7 scale at matching factor, no tracebacks, fps sane

**verdict**: PASS mechanical: 150k steps in 68s on lower (8 envs), zero tracebacks, ep_rew_mean ~1060 vs c1-era 1177 (kernel income cut visible), std 1.52, approx_kl 0.0197. Cleared cw-walk-kgate 4M launch (snapshot 19e97f8).

