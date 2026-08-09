# cw-walk-lowgait-dr035

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T20:47:33+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait_dr05_r1.zip

**wandb_id**: a3c7pdzx

**hypothesis**: Near-miss follow-up (one, per standing rules) to cw-walk-lowgait-dr05-r1 FAIL: at DR0.5 the -50mm crouch broke on the hardest draws (1 sto flag-leg skate ep, gv 11/12) while 10/12 were clean and DR0 retention held. One variable off the r1 checkpoint: dr-scale 0.5 -> 0.35. Plain: find the physics-variation level the crouch actually survives, like the strafe/wander lines found their DR ceilings. Prediction-if-true: own-cfg DR0.35 panel gv 12/12, 0 term, height err <=10mm, slip/m med <=1.6 with DR0 retention intact - crouch DR ceiling is between 0.35 and 0.5, banked as envelope knowledge. Prediction-if-false: flag-leg draws persist even at 0.35 - deep-crouch margins are brittle to any structural DR and the crouch stays a DR0 demo skill. Strongest alternative: passes but slip creeps toward 1.6 (DR-creep pattern).

**gate**: own-cfg DR0.35 15s at -50mm 6+6: gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 det retention gv 6/6, height err <=8mm, slip/m <=1.15; frames watched det

