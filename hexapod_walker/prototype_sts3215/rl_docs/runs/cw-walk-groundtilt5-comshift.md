# cw-walk-groundtilt5-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T02:01:50+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**wandb_id**: 096fyqeg

**hypothesis**: NEW compose, untried pairing: floor-slope (5deg, DR-free per cw-walk-groundtilt5 PASS) x off-center payload (CoM offset 0.03m, the comshift_dr05 PASS envelope). CoM offset already composes cleanly onto the plain champion and the driving package (joylat25-comshift, in flight) but never onto an UNEVEN-FLOOR axis specifically -- an off-center load could bias which side digs in on a slope, a distinct failure mode from CoM-offset on flat ground. One variable off groundtilt5: add dr.com_offset_m=0.03. If-true: own-cfg (tilt u(0,5deg)+com_offset) det+sto gv 12/12, 0 term, det med fwd >=1.2m; DR0 flat no-offset retention clean (slip<=1.24, prog>=0.90). If-false: off-center mass on a slope compounds into a one-sided tip/flag-leg draw beyond groundtilt5's own honest 2/6 steep-tilt tail.

**gate**: own-cfg (tilt u(0,5deg) + dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, det med fwd >=1.2m; DR0 flat no-tilt-no-offset retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

