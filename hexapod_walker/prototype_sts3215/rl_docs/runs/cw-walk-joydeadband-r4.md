# cw-walk-joydeadband-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T00:25:12+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-joydeadband-r3

**hardware_ready**: False

**hypothesis**: 4th retry of cw-walk-joydeadband (base+r1+r2+r3 all lost launch-collision races, gotcha 13b, 0 steps each). Same spec unchanged: driving-deadband compose off joylat25.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 45 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS — servo-deadband axis (1-3x) composes cleanly onto the joylat25 driving package (abrupt +-45deg flips + DR0.5 + latency 0.5-2.5x), 4th launch retry (base/r1/r2/r3 all lost to fleet launch-collision storms, 0 steps each). JOYSTICK GATE @DR0.2 45deg: PASS, 0 in-envelope falls (fwd/diag/stop-go panel + 3 flip-stress eps, trk_err 0.025-0.031). Own-cfg (DR0.5+lat+fric+deadband) harness det+sto 6/6 @15s: gv 12/12, 0 term, prog med 0.95/0.95 (>=0.80 gate). DR0 nominal retention: gv 12/12, 0 term, prog med 0.95/0.98, slip/m med 1.18/1.23 -- champion band, no erosion from adding the deadband axis. Frames det: level body, six legs cycling through the panel, no flag leg, known paddle foot-slide (contact-pricing root, unchanged). Not hardware-ready.

