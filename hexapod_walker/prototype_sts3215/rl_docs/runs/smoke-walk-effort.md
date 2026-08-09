# smoke-walk-effort

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T06:29:12+00:00

**pod**: hexapod-sweep-lower

**steps**: 60000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**hypothesis**: Mechanism probe: reward.k_walk_effort=1.2 trains without error, reward_effort logged negative, no NaN/crash

**gate**: process survives 60k steps; reward_effort key present and negative in logs

**verdict**: launch verification failed: init-from checkpoint absent on pod (copied now); retrying as smoke-walk-effort-r2

**failed_reason**: log not growing (2228 -> 2228 bytes)

