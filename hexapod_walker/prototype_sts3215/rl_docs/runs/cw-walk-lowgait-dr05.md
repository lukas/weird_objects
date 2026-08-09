# cw-walk-lowgait-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T17:03:06+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait50.zip

**hypothesis**: RETRY 1 (first launch died at reset: train-4 /dev/shm 98% full of leaked hexmjx segments from the c53 stopgo35 kill; cleaned). Robustness rung for the crouch line (the same one-variable move that produced wander-dr05 and strafe-dr05 PASSes): the -50mm crouch is envelope-verified at DR0 only. One variable off lowgait50: --no-dr -> model-DR 0.5. Plain: the duck-and-walk height knob must survive real-world physics spread to be worth deploying. Prediction-if-true: own-cfg DR0.5 keeps the crouch (gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6) with DR0 retention intact. Prediction-if-false: deep-crouch workspace margins are too thin for physics variation - crouch stays a DR0 demo skill. Strongest alternative: DR0.5 holds but slip inflates past 1.6 like strafe's DR creep. Parent: rl_move/sim/policies/ppo_goal_cw_walk_lowgait50.zip.

**gate**: own-cfg DR0.5 15s at -50mm 6+6: gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 det retention gv 6/6, height err <=8mm, slip/m <=1.15; frames watched det

**verdict**: INFRA FAILURE, no training happened (0 steps): workers died at first env reset — train-4 /dev/shm was 98% full of hexmjx shm segments leaked by the c53 stopgo35 kill. shm cleaned, retried as cw-walk-lowgait-dr05-r1 (VERIFIED RUNNING on train-4). Not a hypothesis result.

**refused_reason**: W&B already has a run named cw-walk-lowgait-dr05 (names are append-only; pick a new one)

