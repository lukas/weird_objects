# cw-uni-blend1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED_BY_OPERATOR

**created**: 2026-08-09T21:48:44+00:00

**pod**: hexapod-mjx-train-11

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05-c1

**wandb_id**: ywcuroj7

**hypothesis**: UNIFIED JOYSTICK POLICY arm 1 (operator top deliverable, 08-09 evening): one checkpoint that stands up, walks/steers, stops, sits down. Warm-start the joystick-gate-passing driving champion (joyjit-dr05-c1) and reopen the goal mix to walk=0.7/hold=0.1/rise=0.1/lower=0.1 (env is goal-conditioned; obs unchanged). If-true: blend keeps the joystick gate AND does rise/lower >=5/6 + quiet hold - the model-zoo problem collapses into one checkpoint and drive_policy gets mode keys. If-false: walk erodes under the blend (known multi-skill warm-start risk, review sec 12) - ladder the mix to 0.9 walk before abandoning. Strongest alternative: rise/lower learn as twitchy hacks that game the height check - videos + current draw will show it.

**gate**: own-cfg: eval_drive joystick gate PASS retained (0 in-envelope falls, DR0.2); rise and lower det >=5/6 each; hold quiet (0 term, no drift); walk own-cfg gv 12/12, 0 term. Frames watched det for all modes

**verdict**: killed 25min in (operator session): sim defect — leg segments had no floor collision, rise/lower could sweep shins through the ground. Fixed in 273ebde; requeued as cw-uni-blend1-r2 on fixed sim.

