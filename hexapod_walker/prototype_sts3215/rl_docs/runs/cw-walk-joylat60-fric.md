# cw-walk-joylat60-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:17:43+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-joylat60

**wandb_id**: u5e2nakk

**hardware_ready**: False

**hypothesis**: Floor-grip variation composed onto the 60s abrupt-flip+DR0.5+latency driving endurance package (joylat60 PASS). One variable off joylat60: add dr.friction_scale=0.4,1.6 (same axis that composed cleanly onto the 15s joyfric/joyheadfric driving packages). If-true: own-cfg 60s panel gv 12/12, 0 term, prog med >=0.85, no first/second-half decay, JOYSTICK GATE @DR0.2 0 falls -- friction composes onto the endurance rung same as it did onto the 15s rung. If-false: grip variation interacts badly with the 60s abrupt-resample horizon (slip runaway or late-episode terminations) -- endurance and grip-robustness don't compose for free.

**gate**: Own-cfg (DR0.5+latency0.5-2.5x+friction0.4-1.6x) det+sto 6/6 @60s: gait_valid 12/12, 0 term, prog med >=0.85, no det first/second-half slip decay beyond noise; JOYSTICK GATE eval_drive @DR0.2 0 in-envelope falls; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS. Friction-grip variation (0.4-1.6x) composes cleanly onto the 60s abrupt-flip+DR0.5+latency driving endurance package (joylat60 PASS). Own-cfg (DR0.5+latency+friction) det+sto 6/6 gv, 0 term, prog med 0.93 det/0.94 sto (gate>=0.85), slip/m med 1.70 det/1.56 sto, fwd med 1.81/1.72m @60s -- same band as joylat60 parent (slip 1.43-1.65), no runaway, no decay (swing_count even 41-59 across all six legs, video shows same gait start to finish). DR0 retention clean: gv 6/6, 0 term, prog med 0.95/0.94, slip/m med 1.72 det/1.53 sto (parent band 1.43-1.65, close, no term). JOYSTICK GATE (eval_drive DR0.2, own cfg incl. friction): 0 in-envelope falls across fwd/back/left/right/diag/stop-go panel + 3 flip-stress episodes (trk_err 0.026-0.034). Frames (det, own-DR): level torso, all six legs cycling throughout, known paddle foot-slide, no new pathology, no flag leg. Friction-robustness now banked on the endurance driving rung.

