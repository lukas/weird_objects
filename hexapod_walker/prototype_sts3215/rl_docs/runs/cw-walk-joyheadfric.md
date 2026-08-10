# cw-walk-joyheadfric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:58:14+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-joyhead90-r1

**wandb_id**: uftbyrob

**hardware_ready**: no

**hypothesis**: Driving-line compose off joyhead90-r1 (c62 PASS: +-90deg abrupt-flip DR0.5 widest envelope, JOYSTICK GATE 0 falls). One variable: add dr.friction_scale=0.4,1.6 (proven trainable at DR0/DR0.5 by fricvar/fricvar-dr05, and just composed cleanly onto the lat25 driving line as cw-walk-joyfric this cycle). Distinct from the concurrent cycle's joyhead90-lat25 compose (latency axis) - this is the floor-grip axis on the WIDEST heading envelope. Plain: the room's real floor (tile/rug/hardwood) must not break flip recovery at the widest steering angles the operator will actually use. If-true: JOYSTICK GATE @90 0 falls AND own-cfg (DR0.5+friction) gv 12/12, 0 term, prog med >=0.80 - floor-grip joins the widest-envelope driving package. If-false: grip draws break flip recovery at +-90 (in-envelope falls or prog craters) - the wide envelope is friction-fragile even though +-45 (joyfric) wasn't. Strongest alternative: no falls but slick draws slow direction changes at the wider angle - check flip-stress dist vs joyhead90-r1 baseline.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg (DR0.5 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS. Floor-grip variation (0.4-1.6x) composes cleanly onto the widest +-90deg envelope: JOYSTICK GATE @90 0 in-envelope falls (left 0.207m/right 0.256m, flip-stress trk_err 0.029-0.038; cw_walk_joyheadfric_drive90.json), own-cfg (DR0.5+friction) det+sto gv 12/12, 0 term, prog med 0.88/0.87 (>=0.80 gate); DR0 nominal retention gv 12/12, 0 term, prog med 0.87/0.90 (= joyhead90_r1 baseline band, no erosion). Frames: level six-leg paddle gait, no flag leg, no leg-through-floor. Slip/m higher than the 45deg package (1.7-1.9 vs joyfric 1.3-1.7) -- wider steering + grip variation costs more foot-slide, expected of the paddle gait, not a regression.

