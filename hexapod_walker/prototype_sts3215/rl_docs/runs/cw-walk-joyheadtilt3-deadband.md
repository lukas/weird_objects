# cw-walk-joyheadtilt3-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T04:11:19+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-joyheadtilt3

**hypothesis**: NEW compose, untried pairing: servo deadband hardening (1.0-3.0x, the same range used on joyheaddeadband/deadband30) onto the 3deg floor-slope + widest +-90deg driving package (joyheadtilt3, PASS this cycle). Deadband has composed for free onto the plain champion (joyheaddeadband) and friction-hardened package (implied by joyheadfric-comshift training); slope specifically has not been tried with deadband-degraded actuation. If-true: JOYSTICK GATE @90 0 falls; own-cfg (DR0.5+lat+tilt3+deadband) det+sto 6/6 gv, 0 term, prog med matching joyheadtilt3's own band (~0.85/0.93); DR0 TRUE FLAT no-deadband retention clean. If-false: coarse actuation resolution on a slope biases foot placement enough to crater progress or cost falls that friction/payload composes did not show -- slope tolerance is deadband-fragile.

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+dr.ground_tilt_deg=3.0+dr.deadband_scale=1.0,3.0) det+sto 6/6 @15s: gait_valid 6/6, 0 term, det prog med>=0.80 (joyheadtilt3's own band minus noise); DR0 TRUE FLAT no-deadband retention det 6/6 gv, prog med>=0.85, slip within noise of joyheadtilt3's own flat-retention band; frames watched det

**refused_reason**: hexapod-mjx-train-10 already runs cw-walk-placementnoise6-payload-rr1-rr1 — GPU pods host exactly one run; pick a free GPU pod.

