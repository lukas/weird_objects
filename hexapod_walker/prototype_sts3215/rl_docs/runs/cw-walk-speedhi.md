# cw-walk-speedhi

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T19:42:09+00:00

**pod**: hexapod-sweep-friction

**steps**: 1500000

**parent**: ppo_goal_cw_walk_dr04b (init_dr04b.zip md5 52220b24)

**wandb_id**: g30vnbvl

**checkpoint**: pod friction: rl_move/sim/policies/ppo_goal_cw_walk_speedhi.zip (md5 0c3ea0bd14bcb7af1f4b57ef8f88f6b4)

**hypothesis**: Speed-range diagnostic (review 2a/3): the drag-shuffle is near-optimal only at 2-6 cm/s; commanding 0.10-0.15 m/s makes dragging physically insufficient and forces real stepping. If-true: swing counts rise on all six legs, sacrificed-leg count drops vs parent (3), speed exceeds 0.06 m/s -> curriculum frontier is fast-to-slow. If-false: shuffle persists (speed plateaus <=0.06, legs still sacrificed) or pure falls with no stepping -> slow-to-fast; go to phase reward (plan item c)

**gate**: diagnostic, informative either way: harness @1.5M, DR 0.4, commands 0.10-0.15: TRUE-branch if >=3/6 sto episodes gait-valid (no sacrificed legs) AND mean speed >=0.08 m/s; else FALSE-branch. Blunt video verdict required

**verdict**: DIAGNOSTIC ANSWERED: FALSE branch (cycle 11). 1.39M steps, finished 20:06. Harness @ DR 0.4, commands 0.10-0.15: walk det 0/6 gait-valid @ vel_err 0.099, sto 0/6 @ 0.084, mean speed 0.033-0.034 m/s (gate needed >=0.08 + >=3/6 gait-valid). Leg 3 sacrificed 12/12; one over_current termination; video shows the same ~3 cm/s drag-shuffle now flagging TWO legs vertically. Speed pressure does not force stepping; curriculum frontier stays slow->fast; phase-based tripod reward launched (cw-walk-phase). Ckpt never promotable (diagnostic).

