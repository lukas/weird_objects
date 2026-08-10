# cw-walk-joyjit-dr05-payload-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T03:08:04+00:00

**pod**: hexapod-mjx-train-3

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05-c1

**hypothesis**: Command-jitter driving (randomized abrupt resample/blend, PASSED at DR0.5, 3-seed panel closed) hasn't been tested with chassis payload. Payload composes for free onto other DR0.5 driving packages (joyheadfric-payload PASS) but jitter's rapid direction flips under added inertia could differ. One variable off cw-walk-joyjit-dr05-c1: add dr.mass_scale=1.0,1.4. If-true: own-cfg DR0.5+payload det+sto 6/6 gv, 0 term, JOYSTICK GATE 0 falls incl. instant-flip stress; DR0 no-payload retention clean. If-false: added mass causes a flip-stress fall the unloaded jitter package never showed.

**gate**: Own-cfg (--dr-scale 0.5 + dr.mass_scale=1.0,1.4) det+sto 6/6 @15s: gait_valid 12/12, 0 term, no falls; JOYSTICK GATE @DR0.2+payload 0 in-envelope falls incl. instant-flip stress; DR0 no-payload retention det 6/6 gv, slip/m<=1.24; frames watched det

