# probe-stance-bellyrest

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T04:23:08+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: rl_move/sim/policies/ppo_goal_cw_stance_endpost_c1.zip

**hypothesis**: Mechanical probe of goal.lower_belly_start_frac=0.35 before cw-stance-bellyrest: belly-rest starts execute in the training path (start_at zero on lower draws), no tracebacks, canaries green at baseline

**gate**: mechanical only: healthy to 150k, no tracebacks, belly starts occurring

**verdict**: PROBE PASS: 150k in 152s, 0 tracebacks, rise 5/5 raise 1/1 survived 10/10 (canaries green), ckpt saved; belly-start mechanism healthy

