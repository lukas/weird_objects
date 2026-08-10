# cw-walk-groundtilt8-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T05:23:05+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-groundtilt8

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: servo deadband hardening (1.0-3.0x, PASS on deadband30/joyheaddeadband) x marginal 8deg floor-slope (groundtilt8, PASS this cycle with a 3/6 own-cfg crater-tail, seed-confirmed as a recipe trait not seed luck). Deadband has composed with comshift/payload on driving lines but never with a slope this steep on the walk-only line. If-true: own-cfg (tilt8+deadband) det+sto 6/6 gv, 0 term, prog med matching groundtilt8's own band, similar crater fraction (2-4/6); DR0 flat-no-tilt-no-deadband retention clean. If-false: coarse actuation resolution compounds with the marginal slope to push the crater fraction materially worse (5-6/6) or cost falls/flag-leg that neither axis alone showed.

**gate**: Own-cfg (dr.ground_tilt_deg=8.0 + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.0m; DR0 flat-no-tilt-no-deadband retention det 6/6 gv, slip/m<=1.24; frames watched det for crater-fraction/mechanism match to groundtilt8

**verdict**: PASS: servo deadband hardening (1-3x) composes onto the marginal 8deg floor-slope. Own-cfg (tilt8+deadband) det+sto 6/6 gv, 0 term, det prog med 0.86/slip 1.29/fwd 1.04m with a 2/6 crater tail (idx4,5: prog 0.43-0.54, slip 2.4-3.2), sto prog med 0.90/slip 1.12/fwd 1.21m with only mild dips (no catastrophic sto crater) -- matches the groundtilt8-alone lineage's own established 2-4/6 crater-tail character (not worse). DR0 flat-no-tilt-no-deadband retention: det gv 6/6 clean (all 6 draws prog 0.90-1.04, no craters), sto gv 6/6 with one known lineage fixed-draw stall (idx4: prog 0.20/slip 7.44). Frame review of both crater episodes (own-cfg det idx5, retention sto idx4): march-in-place pattern -- all 6 feet still cycling (tripod # . # pattern visible), body level (tilt <2.5deg), no flag leg, no dragging, no falls. Coarse actuation resolution does not compound with the marginal slope beyond the known lineage tail. Not hardware-ready (paddle gait, high slip); compose holds.

