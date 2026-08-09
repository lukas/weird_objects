# cw-walk-joyheadfric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T22:58:14+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-joyhead90-r1

**wandb_id**: uftbyrob

**hypothesis**: Driving-line compose off joyhead90-r1 (c62 PASS: +-90deg abrupt-flip DR0.5 widest envelope, JOYSTICK GATE 0 falls). One variable: add dr.friction_scale=0.4,1.6 (proven trainable at DR0/DR0.5 by fricvar/fricvar-dr05, and just composed cleanly onto the lat25 driving line as cw-walk-joyfric this cycle). Distinct from the concurrent cycle's joyhead90-lat25 compose (latency axis) - this is the floor-grip axis on the WIDEST heading envelope. Plain: the room's real floor (tile/rug/hardwood) must not break flip recovery at the widest steering angles the operator will actually use. If-true: JOYSTICK GATE @90 0 falls AND own-cfg (DR0.5+friction) gv 12/12, 0 term, prog med >=0.80 - floor-grip joins the widest-envelope driving package. If-false: grip draws break flip recovery at +-90 (in-envelope falls or prog craters) - the wide envelope is friction-fragile even though +-45 (joyfric) wasn't. Strongest alternative: no falls but slick draws slow direction changes at the wider angle - check flip-stress dist vs joyhead90-r1 baseline.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg (DR0.5 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

