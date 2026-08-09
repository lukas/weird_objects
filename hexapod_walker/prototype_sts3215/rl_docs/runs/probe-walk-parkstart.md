# probe-walk-parkstart

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T04:13:58+00:00

**pod**: hexapod-sweep-friction

**steps**: 150000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_kgate.zip

**hypothesis**: Mechanical probe of goal.walk_park_start_frac=0.25 before cw-walk-parkstart: park starts execute in the training path, no tracebacks, ep_rew in lineage band

**gate**: mechanical only: healthy to 150k, no tracebacks, park resets occurring

**failed_reason**: launcher SIGTERMed by cycle tool timeout during verification; cleanup killed the probe process; retrying once

