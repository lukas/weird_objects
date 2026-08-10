# cw-walk-joyheadtilt3-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T04:27:16+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-joyheadtilt3

**wandb_id**: 3cdlnl1w

**hypothesis**: NEW compose (2nd queue attempt -- 1st vanished from backlog with zero ledger trace under this window's heavy concurrent-drain load, matching the c73/c74/c75-flagged lost-update symptom; 0 compute lost). Untried pairing: servo deadband hardening (1.0-3.0x) onto the 3deg floor-slope + widest +-90deg driving package (joyheadtilt3, PASS this cycle). If-true: JOYSTICK GATE @90 0 falls; own-cfg (DR0.5+lat+tilt3+deadband) det+sto 6/6 gv, 0 term, prog med matching joyheadtilt3's own band (~0.85/0.93); DR0 TRUE FLAT no-deadband retention clean. If-false: coarse actuation resolution on a slope biases foot placement enough to crater progress or cost falls that friction/payload composes did not show.

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+dr.ground_tilt_deg=3.0+dr.deadband_scale=1.0,3.0) det+sto 6/6 @15s: gait_valid 6/6, 0 term, det prog med>=0.80; DR0 TRUE FLAT no-deadband retention det 6/6 gv, prog med>=0.85; frames watched det

