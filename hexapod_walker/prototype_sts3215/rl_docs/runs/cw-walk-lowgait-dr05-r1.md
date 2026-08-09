# cw-walk-lowgait-dr05-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:03:56+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowgait50.zip

**wandb_id**: gd4ur7ti

**hardware_ready**: no

**hypothesis**: RETRY 1 of cw-walk-lowgait-dr05 (died at env reset: train-4 /dev/shm 98% full of hexmjx segments leaked by the c53 stopgo35 kill; shm cleaned, relaunched fresh — 0 steps were trained). Robustness rung for the crouch line (same one-variable move that produced wander-dr05 and strafe-dr05 PASSes): the -50mm crouch is envelope-verified at DR0 only. One variable off lowgait50: --no-dr -> model-DR 0.5. Plain: the duck-and-walk height knob must survive real-world physics spread to be worth deploying. Prediction-if-true: own-cfg DR0.5 keeps the crouch (gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6) with DR0 retention intact. Prediction-if-false: deep-crouch workspace margins are too thin for physics variation - crouch stays a DR0 demo skill. Strongest alternative: DR0.5 holds but slip inflates past 1.6 like strafe's DR creep. Parent: rl_move/sim/policies/ppo_goal_cw_walk_lowgait50.zip.

**gate**: own-cfg DR0.5 15s at -50mm 6+6: gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 det retention gv 6/6, height err <=8mm, slip/m <=1.15; frames watched det

**verdict**: FAIL — DR0.5 crouch panel gv 11/12 (gate 12/12): 1 sto draw sacrifices leg 5 and skates near-in-place (prog 0.15, slip/m 10.96; frames show leg 5 held out while the body churns); worst det draw squat-shuffles (prog 0.59, slip 2.54). The clean 10/12 pass medians (det prog 0.95, slip 1.10, height err 2.9mm) and DR0 retention is intact (gv 6/6, slip 0.97, height 3.5mm) — the -50mm crouch skill is kept, but deep-crouch workspace margins break into flag-leg skate on the hardest DR draws, per prediction-if-false. Near-miss: DR0.35 rung queued (cw-walk-lowgait-dr035) to locate the crouch DR ceiling.

