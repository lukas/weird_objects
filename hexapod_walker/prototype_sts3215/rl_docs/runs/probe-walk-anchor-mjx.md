# probe-walk-anchor-mjx

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T08:20:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 1000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**hypothesis**: Mechanism probe for the anchored-stance income gate on the GPU/warp stack: walk_anchor_frac logged per tick, velocity income visibly gated (reward_walk below parent band while anchor_frac ~0.5-0.7 at init), MJX_SNAPSHOT_EXTRA anchor state survives host-half snapshots, no NaN/tracebacks, fps in band.

**gate**: survives 1M steps; walk_anchor_frac present in W&B/log; no tracebacks; periodic eval runs

**verdict**: PASS: 1M steps in 202s, 0 tracebacks/NaN; anchor gate live on MJX host path (ep_rew_mean 438-555 vs ungated parent band ~1100-1400 = income gated ~0.5 as scale audit predicted; snapshot-extra anchor state survived host halves); periodic eval ran (walk err 0.032). walk_anchor_frac W&B key check deferred to main-run launch verification (smokes run WANDB_MODE=disabled). Gates cw-walk-anchorgate.

