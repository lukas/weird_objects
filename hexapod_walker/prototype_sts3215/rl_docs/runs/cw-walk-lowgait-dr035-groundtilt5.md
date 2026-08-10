# cw-walk-lowgait-dr035-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:32:58+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035

**wandb_id**: cl0umg2c

**hypothesis**: NEW compose, untried pairing: crouch stance (-50mm, DR0.35, the lowgait_dr035 PASS) x floor-slope (5deg, the groundtilt5 PASS envelope). Both axes are individually validated exposure wins; slope has never been tried on the crouched stance -- a lower CoM/shorter effective leg travel could interact differently with a tilted floor than the flat-height driving-package composes suggest. If-true: own-cfg (DR0.35+tilt5) det+sto 6/6 gv, 0 term, mean end-height err<=10mm, slip/m<=1.6; DR0 flat retention gv 6/6, height err<=8mm, slip/m<=1.15. If-false: the slope biases the crouched stance into a one-sided tip/flag-leg draw beyond either axis's own tail.

**gate**: Own-cfg (DR0.35 + dr.ground_tilt_deg=5.0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 flat no-tilt retention det 6/6 gv, mean height err<=8mm, slip/m med<=1.15; frames watched det

