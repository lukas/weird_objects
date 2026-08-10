# cw-walk-groundtilt5-deadband-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:37:25+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**wandb_id**: 7kbsrqc1

**hypothesis**: NEW compose, untried pairing: floor-slope (5deg, DR-free per groundtilt5 PASS) x servo deadband (1.0-3.0x, the deadband30 PASS envelope). Deadband has composed cleanly onto DR0.5 (deadband-dr05 PASS-caveat) but never onto an uneven-floor axis; a sluggish/dead-zone servo response could interact badly with the fine balance corrections a slope demands. If-true: own-cfg (tilt+deadband) det+sto 6/6 gv, 0 term, det med fwd>=1.2m (matches groundtilt5's own band); DR0 flat no-deadband retention slip<=1.24, prog>=0.90. If-false: deadband on a slope compounds into a one-sided tip/flag-leg draw beyond groundtilt5's own honest 2/6 steep-tilt tail.

**gate**: Own-cfg (tilt u(0,5deg) + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat no-deadband retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

