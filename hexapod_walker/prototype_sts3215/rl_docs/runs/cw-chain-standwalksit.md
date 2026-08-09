# cw-chain-standwalksit

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T12:31:18+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: 25svd26e

**hypothesis**: RETRY (two prior attempts REFUSED on code-marker churn, never trained). SKILL CHAIN stand->walk->sit: goal-mix walk=0.6,rise=0.2,lower=0.2 off champion 35234ddc, canaries ON so rise/lower regressions auto-stop. If-true: one policy holds walk gate while rise/lower stay >=5/6 (chain viable in one head); if-false: walk income collapses the stance skills (multi-skill needs per-skill heads or schedules).

**gate**: walk: DR0 det 6/6 gait_valid zero-term slip/m <= 1.24; rise/lower canaries green; heights >=5/6 det

