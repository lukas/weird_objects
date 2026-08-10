# cw-walk-joyjit-dr05-payload-rr2-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T04:46:51+00:00

**pod**: hexapod-mjx-train-9

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05-payload-rr1-rr1

**hypothesis**: Retry after 2nd 0-step launch-collision infra death (wandb 0s1lcg7n, runtime 1s, fleet storm pattern) -- jitter x payload hypothesis still completely untested. Same spec as cw-walk-joyjit-dr05-payload-rr1-rr1 unchanged: If-true: own-cfg DR0.5+payload det+sto 6/6 gv, 0 term, JOYSTICK GATE 0 falls incl. instant-flip stress; DR0 no-payload retention clean. If-false: added mass causes a flip-stress fall the unloaded jitter package never showed.

**gate**: Own-cfg (--dr-scale 0.5 + dr.mass_scale=1.0,1.4) det+sto 6/6 @15s: gait_valid 12/12, 0 term, no falls; JOYSTICK GATE @DR0.2+payload 0 in-envelope falls incl. instant-flip stress; DR0 no-payload retention det 6/6 gv, slip/m<=1.24; frames watched det

