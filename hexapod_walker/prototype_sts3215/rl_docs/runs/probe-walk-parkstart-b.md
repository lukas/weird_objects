# probe-walk-parkstart-b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T04:21:02+00:00

**pod**: hexapod-sweep-friction

**steps**: 150000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_kgate.zip

**hypothesis**: Mechanical probe of goal.walk_park_start_frac=0.25 before cw-walk-parkstart: park starts execute in the training path, no tracebacks, ep_rew in lineage band. RETRY of probe-walk-parkstart (attempt 1 killed by launcher SIGTERM, not a code failure)

**gate**: mechanical only: healthy to 150k, no tracebacks, park resets occurring

**verdict**: PROBE PASS: 150k in 188s, 0 tracebacks, video reels ok, ckpt saved; park-start mechanism healthy in the training path

