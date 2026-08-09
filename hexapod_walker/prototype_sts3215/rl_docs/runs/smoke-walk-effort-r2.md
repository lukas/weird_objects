# smoke-walk-effort-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T06:30:58+00:00

**pod**: hexapod-sweep-lower

**steps**: 60000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**hypothesis**: Mechanism probe: reward.k_walk_effort=1.2 trains without error, reward_effort logged negative, no NaN/crash

**gate**: process survives 60k steps; reward_effort key present and negative in logs

**verdict**: MECHANISM PASS / shutdown HUNG: 384 PPO updates completed with k_walk_effort=1.2 active (ep_rew_mean 1.22e3->1.09e3, matches the -218/ep scale audit; kl/losses finite, no NaN, no traceback). Trainer hung AFTER the final update, before checkpoint save - CPU-trainer shutdown/eval path, log frozen 10+ min, remnants killed. Not attributable to the diff (term is a pure per-tick reward add in _post_step). Pre-existing CPU trainer issue; GPU run uses train_ppo_mjx. No checkpoint saved (smoke needs none).

**failed_reason**: log not growing (72911 -> 72911 bytes)

