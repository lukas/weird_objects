# cw-walk-groundtilt8-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T05:23:05+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-groundtilt8

**hypothesis**: NEW compose, untried pairing: servo deadband hardening (1.0-3.0x, PASS on deadband30/joyheaddeadband) x marginal 8deg floor-slope (groundtilt8, PASS this cycle with a 3/6 own-cfg crater-tail, seed-confirmed as a recipe trait not seed luck). Deadband has composed with comshift/payload on driving lines but never with a slope this steep on the walk-only line. If-true: own-cfg (tilt8+deadband) det+sto 6/6 gv, 0 term, prog med matching groundtilt8's own band, similar crater fraction (2-4/6); DR0 flat-no-tilt-no-deadband retention clean. If-false: coarse actuation resolution compounds with the marginal slope to push the crater fraction materially worse (5-6/6) or cost falls/flag-leg that neither axis alone showed.

**gate**: Own-cfg (dr.ground_tilt_deg=8.0 + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.0m; DR0 flat-no-tilt-no-deadband retention det 6/6 gv, slip/m<=1.24; frames watched det for crater-fraction/mechanism match to groundtilt8

