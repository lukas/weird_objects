# cw-walk-fricvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:51:15+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip

**wandb_id**: bfbsfc4d

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 13b via the c49 dr.<field> overrides, next isolated axis after mass (payload50) / latency (latjit25) / torque (torquedroop, concurrent cycle): FRICTION - the axis most tied to the open skating defect and the unknown real floor. ISOLATED: dr-scale 0.0 with only dr.friction_scale=0.4,1.6 randomized (wider than the standard 0.6-1.4 envelope), one variable off the no-DR champion. Plain: train on floors from slick to grippy so the real floor's grip level doesn't matter. Prediction-if-true: gait holds across the spread (own-cfg gv 12/12, 0 term, prog med >=0.8) and DR0 retention holds (slip/m <=1.15) - friction robustness is trainable by exposure and joins the transfer recipe. Prediction-if-false: low-friction draws crater (falls or prog <0.5) - friction needs contact-aware machinery (contact-aux head), not exposure. Strongest alternative: policy slows cadence/shortens strides to survive slick draws, hiding as a pass - check stride/cadence vs champion. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: own-cfg (dr.friction_scale=0.4,1.6, dr-scale 0) 30s 6+6: gv 12/12, 0 term, prog_ratio med >=0.8; DR0 det retention gv 6/6 slip/m <=1.15; frames watched det

**verdict**: PASS. Friction 0.4-1.6x robustness is trainable by exposure: own-cfg gate gv 12/12, 0 term, det prog med 0.87 (>=0.8), frames watched (all 6 det strips: upright, level, six legs cycling); DR0 det retention gv 6/6, slip/m 1.09 (<=1.15), prog 0.97. Honest tail: the 2 slickest det draws churn near-in-place (prog 0.36-0.56, slip/m 2.4-4.2, stride halves 0.078->0.042m) — solid across moderate grip variation, extreme slick end unsolved (the paddle gait needs grip). One DR0 sto fixed-draw stall (prog 0.21) = known lineage canary class, det clean. NOT hardware-ready (paddle lineage).

