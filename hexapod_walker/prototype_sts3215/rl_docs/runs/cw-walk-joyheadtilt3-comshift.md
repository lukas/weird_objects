# cw-walk-joyheadtilt3-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T05:21:38+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-joyheadtilt3

**hypothesis**: NEW compose, untried pairing: off-center CoM payload shift (0.03m) x the 3deg floor-slope + widest +-90deg driving package (joyheadtilt3, PASS this cycle). CoM offset has composed onto plain champion, groundtilt5, joylat25/60, joyfric, joyheadfric -- but never onto a package combining BOTH the widest steering envelope AND a floor slope simultaneously. If-true: JOYSTICK GATE @90 0 falls; own-cfg (DR0.5+lat+tilt3+comshift) det+sto 6/6 gv, 0 term, prog med matching joyheadtilt3's own band (~0.85/0.93); DR0 TRUE FLAT no-offset retention clean. If-false: the off-axis load biases turning response on a slope at the widest envelope, costing falls or a retention miss that narrower-envelope or flat-ground comshift composes did not show.

**gate**: JOYSTICK GATE @DR0.2 heading90: 0 in-envelope falls; own-cfg (DR0.5+lat+tilt3+comshift) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med>=0.80; DR0 TRUE FLAT no-offset retention det 6/6 gv; frames watched det

