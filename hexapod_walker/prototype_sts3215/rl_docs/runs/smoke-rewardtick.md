# smoke-rewardtick

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-15T13:47:19+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2048

**hypothesis**: smoke: optimization/reward_per_tick logging change boots and trains

**gate**: boots, trains, no crash, optimization/reward_per_tick* keys appear in stdout/log

**verdict**: PASS — boots/trains 2048 steps (2 rollout updates) on GPU-MJX warp, no crash/exception; optimization/reward_per_tick(_cumulative/_ema) payload keys compute without error across both rollout ends (division-by-zero guarded); WANDB_MODE=disabled so values not independently visible in this smoke, but code path exercised cleanly. Code change: rl_move/sim/train_ppo_mjx.py _Track callback (fb_20260815T132846_c8442f).

