# cw-walk-multiaxis2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T21:32:59+00:00

**pod**: hexapod-mjx-train-2

**steps**: 18000000

**parent**: cw-walk-multiaxis1

**wandb_id**: 763iku0y

**hardware_ready**: no

**hypothesis**: Fifth axis onto the stack, off the c63 multiaxis1 PASS: add floor slope (dr.ground_tilt_deg=5, validated standalone by groundtilt5 c60) to the 4-axis compose at dr-scale 0. One variable: +tilt axis. Plain: real floors are not flat AND the robot will be imperfect at the same time - the stacked recipe should absorb the validated tilt envelope too. If-true: 5-axis panel gv 12/12, 0 term, det med fwd >=1.2m, DR0 nominal retention clean - the stack absorbs its 5th axis, keep laddering. If-false: tilt+stack interference craters median transport (not just the known steep-draw tail) - the stack has a capacity limit at 4 axes, useful boundary. Strongest alternative: passes but the heavy-tail fraction grows past 2/6 det (tails compound even when medians hold) - count tail draws vs multiaxis1. Parent: rl_move/sim/policies/ppo_goal_cw_walk_multiaxis1.zip.

**gate**: Own-cfg harness at dr-scale 0 + all five dr overrides (4-axis stack + dr.ground_tilt_deg=5), det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: FAIL (letter/near-miss). Own-cfg 5-axis panel (4-axis stack + dr.ground_tilt_deg=5, dr-scale 0) gv 12/12, 0 term, det med fwd 1.27m (>=1.2 gate), slip med 1.14; sto med fwd 1.29m, slip 1.14 -- exposure panel clears cleanly, matches multiaxis1s established 2/6 harshest-draw tail. But the pre-registered DR0 flat-ground retention cap (det slip/m <=1.24) is missed: det slip med 1.30 (5% over); sto side is worse -- 4/6 sto draws flagged rough (vs multiaxis1-s1s clean 1/6), one severe (fwd 0.91m, slip 4.08). Reading: the 5th axis (tilt) pushes the stack past the retention cap that the 4-axis version (multiaxis1) held clean -- matches the pre-registered if-false (tails compound even though the det median still holds). Axis-stacking ceiling for this step budget is 4 at DR0; a 5th axis needs either more steps or is a genuine ceiling, untested which. No requeue this line.

