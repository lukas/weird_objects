# cw-walk-joyheadfric-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:20:03+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-joyheadfric

**wandb_id**: 8z6ifmwe

**hypothesis**: NEW compose, untried pairing: widest joystick driving (+-90deg heading, DR0.5+latency+floor-grip 0.4-1.6x, the joyheadfric PASS) x off-center CoM payload (0.03m). Payload/mass already composes onto this exact package for free (joyheadfric_payload_r1 PASS); comshift is a distinct off-axis-load mechanism (biases which side digs in during a turn) never tried at this heading width. If-true: own-cfg (DR0.5+lat+fric+comshift) det+sto 6/6 gv, 0 term, prog med matching joyheadfric's own band (~0.87-0.90); JOYSTICK GATE @90deg 0 falls; DR0 nominal retention prog/slip within noise of joyheadfric's own retention band (prog 0.87/0.90). If-false: the off-axis load interacts with the wider steering envelope worse than straight mass did, costing falls on flips or an eroded nominal floor.

**gate**: Own-cfg (DR0.5+lat+fric+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 6/6, 0 term, det prog med>=0.80; JOYSTICK GATE @90deg 0 in-envelope falls; DR0 nominal retention det 6/6 gv, prog med>=0.85, slip within noise of joyheadfric's own flat-retention band; frames watched det

