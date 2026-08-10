# cw-walk-lowgait-dr035-comshift-rr2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:23:53+00:00

**pod**: hexapod-mjx-train-3

**steps**: 18000000

**parent**: cw-walk-lowgait-dr035

**wandb_id**: cxfs6j12

**hypothesis**: NEW compose, untried pairing: crouch stance (-50mm, DR0.35, the lowgait_dr035 PASS) x off-center CoM payload (0.03m, the comshift PASS envelope used everywhere on the driving lines). Payload already composes onto lowgait_dr035 via mass_scale (PASS); comshift is a distinct off-center-load axis never tried on the crouched stance specifically -- a lower CoM height could interact differently with an off-axis CoM shift than the driving-package results suggest. If-true: own-cfg (DR0.35+comshift) det+sto 6/6 gv, 0 term, mean end-height err<=10mm, slip/m<=1.6 (matches lowgait_dr035_payload band); DR0 no-offset retention gv 6/6, height err<=8mm, slip/m<=1.15. If-false: the off-center load biases the crouched stance into a one-sided tip/flag-leg draw beyond lowgait_dr035's own tail.

**gate**: Own-cfg (DR0.35+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 no-offset retention det 6/6 gv, mean height err<=8mm, slip/m med<=1.15; frames watched det

