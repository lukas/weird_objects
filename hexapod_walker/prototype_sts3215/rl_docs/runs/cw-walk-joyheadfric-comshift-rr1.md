# cw-walk-joyheadfric-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T03:20:03+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-joyheadfric

**wandb_id**: 8z6ifmwe

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: widest joystick driving (+-90deg heading, DR0.5+latency+floor-grip 0.4-1.6x, the joyheadfric PASS) x off-center CoM payload (0.03m). Payload/mass already composes onto this exact package for free (joyheadfric_payload_r1 PASS); comshift is a distinct off-axis-load mechanism (biases which side digs in during a turn) never tried at this heading width. If-true: own-cfg (DR0.5+lat+fric+comshift) det+sto 6/6 gv, 0 term, prog med matching joyheadfric's own band (~0.87-0.90); JOYSTICK GATE @90deg 0 falls; DR0 nominal retention prog/slip within noise of joyheadfric's own retention band (prog 0.87/0.90). If-false: the off-axis load interacts with the wider steering envelope worse than straight mass did, costing falls on flips or an eroded nominal floor.

**gate**: Own-cfg (DR0.5+lat+fric+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 6/6, 0 term, det prog med>=0.80; JOYSTICK GATE @90deg 0 in-envelope falls; DR0 nominal retention det 6/6 gv, prog med>=0.85, slip within noise of joyheadfric's own flat-retention band; frames watched det

**verdict**: Compose PASS: 90deg head-turn joystick driving x off-center CoM payload (0.03m). Own-cfg (dr-scale 0.5, correct rate) det+sto 12/12 gait_valid, 0 term, det prog med 0.89 (>=0.80 gate); DR0 retention det 6/6 gv prog med 0.91, slip 1.67 in joyheadfric's own flat-retention band. JOYSTICK GATE PASS @90deg, 0 in-envelope falls incl flip-stress. Frames: clean six-leg tripod gait, no flag leg on worst-slip draws.

