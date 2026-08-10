# cw-walk-lowgait-dr035-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T02:03:04+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035

**wandb_id**: q56c2uib

**hypothesis**: NEW compose, untried pairing: crouch (-50mm, DR0.35-hardened per cw-walk-lowgait-dr035 PASS) x servo deadband (1.0-3.0x, the deadband30 PASS envelope). Deadband already composes onto the plain champion (deadband30, deadband-dr05 both PASS) but never onto a CROUCHED stance -- shorter effective stride at -50mm may depend on finer small-amplitude corrections that a wide deadband could swallow, a distinct risk from deadband on the nominal-height gait. One variable off lowgait-dr035: add dr.deadband_scale=1.0,3.0. If-true: own-cfg (DR0.35 crouch + deadband) det+sto gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 no-deadband crouch retention clean (height err <=8mm, slip<=1.15). If-false: wide deadband at the shorter crouch stride craters progress or forces jerky overdrive strokes (frames must lead with lurching if scalars still pass).

**gate**: own-cfg (DR0.35 + dr.deadband_scale=1.0,3.0, height_off_mm=-50) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; plus DR0 no-deadband crouch retention det 6/6 gv, height err <=8mm, slip/m<=1.15; frames watched det

