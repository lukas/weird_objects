# cw-walk-joyfric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T20:42:36+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-joylat25

**wandb_id**: gdifa0tk

**hypothesis**: Driving-line compose off joylat25 (c60 best driving candidate: abrupt-flip DR0.5 package + latency 0.5-2.5x). One variable: add dr.friction_scale=0.4,1.6 (the band fricvar proved trainable at DR0 and fricvar-dr05 proved DR0.5-composable this cycle). Plain: joystick driving must survive the room's actual floor - tile, rug, hardwood - not just nominal grip. If-true: grip variation composes onto the driving package - joystick gate zero falls, own-cfg gv 12/12, prog med >=0.85, DR0 retention clean - floor-grip joins the driving envelope. If-false: grip draws break flip recovery (in-envelope falls on slick flips, or prog craters) - driving stays nominal-grip and the contact-pricing calibration inherits it. Strongest alternative: no falls but slick draws slow direction changes (re-track lag grows) - check flip-stress episode dists vs parent joystick-gate baseline.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 retention det 6/6 gv; frames watched det

