# cw-walk-joyhead90-lat25

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T20:52:04+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_joyhead90_r1.zip

**wandb_id**: x4b2scsn

**hypothesis**: Driving-line convergence: joyhead90-r1 (c62 PASS) holds the widest hardened envelope (+-90deg abrupt flips at DR0.5); joylat25 (c60 PASS) proved bus-latency 0.5-2.5x composes onto the +-45 package. One variable off the joyhead90-r1 checkpoint: add dr.latency_scale=0.5,2.5. Plain: the closest sim rung yet to the operator joysticking the real robot - any front-half-circle command, abrupt flips, physics spread AND real-bus latency jitter. Prediction-if-true: JOYSTICK GATE at +-90/DR0.2 zero in-envelope falls and own-cfg DR0.5+latency panel gv 12/12, 0 term, prog med >=0.80 - full driving package composed, new best driving candidate. Prediction-if-false: latency jitter breaks the tight 0.1s flip blends at lateral headings (falls or prog crater) - latency and envelope hardening interact and need joint training from a lower rung. Strongest alternative: passes but slip inflates well past the 1.7-1.8 parent band - report, keep parent as envelope champion.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5+latency harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

