# cw-walk-groundtilt8-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:46:51+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-groundtilt8-r3

**wandb_id**: aj7pt80u

**hypothesis**: NEW compose, untried pairing: marginal 8deg floor-tilt exposure rung (groundtilt8-r3 PASS-with-caveat: 3/6 own-cfg det draws crater to a shuffle, at/past the exposure-training ceiling) x off-center CoM payload (0.03m, the comshift PASS envelope used everywhere on the driving lines). An asymmetric CoM shift could tip the balance on the already-marginal steepest-azimuth draws that groundtilt8 struggles with. If-true: own-cfg (tilt8+comshift) det+sto 6/6 gv, 0 term, det med fwd>=1.1m, crater fraction stays <=3/6 (no worse than groundtilt8 alone); DR0 flat-no-offset retention det 6/6 gv, slip/m<=1.24. If-false: the off-center load pushes the steepest-azimuth craters into falls or a majority-crater tail (>=4/6) -- 8deg is confirmed as a ceiling that does not compose further.

**gate**: Own-cfg (tilt u(0,8deg) + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.1m, crater fraction<=3/6, 0 falls; DR0 flat-no-offset retention det 6/6 gv, slip/m<=1.24; frames watched det

