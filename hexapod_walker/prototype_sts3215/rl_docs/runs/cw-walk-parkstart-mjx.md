# cw-walk-parkstart-mjx

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T04:59:02+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_kgate.zip

**wandb_id**: su9yi1wc

**checkpoint**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**hypothesis**: STACK SWITCH-OVER relaunch of cw-walk-parkstart (killed on CPU 2026-08-09, ~0.5M steps in, no verdict). Same arm, same hypothesis: park-basin is state-visitation-starved, so 25% of walk episodes start IN a jittered tripod park (goal.walk_park_start_frac=0.25); kernel prog-gate kept at 1.0. If-true: env/reward_end distribution shows park-exit learning, walk gate passes with gait_valid. If-false: belly-start distribution mismatch vs no-exit, park persists. One variable vs the kgate segment (plus the trainer stack, validated behaviorally equivalent: MJX_PORT.md A/B + mjx-dr-canary-check2).

**gate**: standard 15s DR0 12/12 clauses plus park-exit eval at frac=1.0 needing 10/12 fwd 0.30m with gait_valid, plus retention 5s det slip/m at most 1.8

**verdict**: FAIL vs recorded gate: standard 15s fwd 11/12 (needs 12/12; sto[5] partial park fwd 0.192), gv 12/12, det fwd mean 0.745; park-exit clause PASS 10/12 at the bar; retention det slip/m 1.58 PASS. det[2] churn CONVERTED; kgate sto[4] full park gone; residual park weakened 4x and displaced. NOT HARDWARE-READY (1/6 sto partial park, skating 1.2-1.5 m/m, DR0). HYPOTHESIS INCONCLUSIVE leaning SUPPORTED - segment under-dosed ~61 vs ~325 PPO updates (stack-switch step-parity artifact). CHAMPION PROMOTED to this ckpt (beats h15b seed-wise 6/6 det fwd; named regression: 5s slow start 0.210 vs 0.382). Continuation cw-walk-parkstart-mjx-c1 at update parity is the discriminator. Evals logs/ckpt_eval/cw_walk_parkstart_mjx_{15s,parkexit,5s}, 36 eps, 10 strips watched.

