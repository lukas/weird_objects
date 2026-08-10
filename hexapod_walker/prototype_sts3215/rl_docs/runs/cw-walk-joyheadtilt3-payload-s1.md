# cw-walk-joyheadtilt3-payload-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T04:50:14+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-joyheadtilt3-payload-r1

**hypothesis**: Ruling-7 seed twin of cw-walk-joyheadtilt3-payload-r1 PASS (90deg steering on 3deg ground-tilt x chassis payload 1.0-1.4x, new compose type). Seed-1 confirms the compose is a recipe not seed luck. If-true: own-cfg (dr-scale 0.5) det+sto gv 12/12, 0 term, det prog med>=0.80 matching seed0's 0.86; DR0 true-flat-no-payload retention clean det gv 6/6, prog med>=0.85, slip within noise of seed0's 1.55 band. If-false: seed1 materially erodes gait_valid or DR0 retention outside seed0's band -- compose is seed-sensitive, don't bank as a recipe.

**gate**: Own-cfg (dr-scale 0.5, DR0.5+lat+tilt3+mass1.0-1.4) det+sto 6/6 @15s: gait_valid 6/6 each, 0 term, det prog med>=0.80; JOYSTICK GATE @90deg 0 in-envelope falls; DR0 TRUE FLAT no-payload retention det 6/6 gv, prog med>=0.85, slip within noise of seed0's flat-retention band (1.55); frames watched det

