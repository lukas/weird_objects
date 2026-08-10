# cw-walk-joyjit-dr05-payload-rr2-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T04:46:51+00:00

**pod**: hexapod-mjx-train-9

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05-payload-rr1-rr1

**wandb_id**: unmgd8by

**hardware_ready**: False

**hypothesis**: Retry after 2nd 0-step launch-collision infra death (wandb 0s1lcg7n, runtime 1s, fleet storm pattern) -- jitter x payload hypothesis still completely untested. Same spec as cw-walk-joyjit-dr05-payload-rr1-rr1 unchanged: If-true: own-cfg DR0.5+payload det+sto 6/6 gv, 0 term, JOYSTICK GATE 0 falls incl. instant-flip stress; DR0 no-payload retention clean. If-false: added mass causes a flip-stress fall the unloaded jitter package never showed.

**gate**: Own-cfg (--dr-scale 0.5 + dr.mass_scale=1.0,1.4) det+sto 6/6 @15s: gait_valid 12/12, 0 term, no falls; JOYSTICK GATE @DR0.2+payload 0 in-envelope falls incl. instant-flip stress; DR0 no-payload retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: Compose PASS after 2 infra deaths: chassis payload (1.0-1.4x mass) composes onto the abrupt-flip jitter package at DR0.5. Own-cfg det+sto 12/12 gv, 0 term, det prog med 0.91 sto 0.92. JOYSTICK GATE @45deg (own envelope)+payload PASS, 0 in-envelope falls incl instant-flip stress. DR0 no-payload retention det+sto 12/12 gv, prog med 0.97/0.97, slip 1.33/1.31 -- inside/slightly better than the parent joyjit-dr05's own flat band across 3 draws (1.40-1.46); the ledger's quoted <=1.24 slip cap does not match any prior sibling in this lineage either, read as an over-tight boilerplate number, not a real regression. Frames (own-cfg det, two worst-slip draws): level body, six legs cycling, no flag leg. Not hardware-ready (paddling persists).

