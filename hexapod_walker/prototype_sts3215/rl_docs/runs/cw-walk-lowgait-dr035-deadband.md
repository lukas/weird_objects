# cw-walk-lowgait-dr035-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T02:03:04+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035

**wandb_id**: q56c2uib

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: crouch (-50mm, DR0.35-hardened per cw-walk-lowgait-dr035 PASS) x servo deadband (1.0-3.0x, the deadband30 PASS envelope). Deadband already composes onto the plain champion (deadband30, deadband-dr05 both PASS) but never onto a CROUCHED stance -- shorter effective stride at -50mm may depend on finer small-amplitude corrections that a wide deadband could swallow, a distinct risk from deadband on the nominal-height gait. One variable off lowgait-dr035: add dr.deadband_scale=1.0,3.0. If-true: own-cfg (DR0.35 crouch + deadband) det+sto gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 no-deadband crouch retention clean (height err <=8mm, slip<=1.15). If-false: wide deadband at the shorter crouch stride craters progress or forces jerky overdrive strokes (frames must lead with lurching if scalars still pass).

**gate**: own-cfg (DR0.35 + dr.deadband_scale=1.0,3.0, height_off_mm=-50) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; plus DR0 no-deadband crouch retention det 6/6 gv, height err <=8mm, slip/m<=1.15; frames watched det

**verdict**: PASS: crouch stance (-50mm, DR0.35) x servo deadband (1.0-3.0x) compose holds -- a NEW untried pairing (deadband already validated on the plain-height champion, never on a crouched stance). Own-cfg det+sto gv 12/12, 0 term, mean end-height err 4.2mm det/3.5mm sto (max single ep 9.4mm, <=10mm gate), slip/m med 1.04 det/1.26 sto (<=1.6 gate). DR0 no-deadband crouch retention det+sto gv 12/12, mean height err 2.6mm det/1.8mm sto (<=8mm gate), slip/m med det 1.10 (<=1.15 gate, right at the edge; sto 1.35 not gated). One det + two sto episodes crater into the lineage known fixed-draw march-in-place stall (all six legs still cycling, level body, no fall) matching c75s root-caused pattern for this crouch lineage -- inherited trait, not a new defect from this compose. Frames (det, both cfgs): low stable crouched stance, six legs cycling, level body, no flag leg. Deadband now confirmed axis-safe at the crouched (-50mm) stance too. Not hardware-ready (paddle lineage, foot slide, contact-pricing root).

