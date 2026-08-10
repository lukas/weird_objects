# cw-walk-joyheadtilt3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T23:20:27+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**hypothesis**: Compose the 3deg floor-slope axis (clean PASS this cycle on the narrower joylat25 package, cw-walk-joytilt3) onto the WIDEST existing driving package (joyhead90-lat25: +-90deg abrupt flips + DR0.5 + latency 0.5-2.5x). One variable: add dr.ground_tilt_deg=3.0. Plain: does the slope tolerance survive when heading commands swing across the full front half-circle, not just +-45deg? If-true: JOYSTICK GATE @90 0 falls, own-cfg gv 12/12, 0 term, prog med >=0.85, DR0 flat retention clean -- slope composes onto the widest package too. If-false: wide-heading turns against a slope fall or crater progress -- slope tolerance is heading-envelope-dependent, cap the compose at +-45deg (joytilt3) for now.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg (DR0.5 + latency + dr.ground_tilt_deg=3.0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 flat retention det 6/6 gv; frames watched det

