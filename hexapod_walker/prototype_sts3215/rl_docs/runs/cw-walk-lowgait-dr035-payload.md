# cw-walk-lowgait-dr035-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:03:15+00:00

**pod**: hexapod-mjx-train-2

**steps**: 18000000

**parent**: cw-walk-lowgait-dr035

**wandb_id**: 0prkiyru

**hardware_ready**: no

**hypothesis**: NEW compose, untried pairing: crouch (-50mm stance, DR0.35-hardened per cw-walk-lowgait-dr035 PASS) x payload (mass_scale 1.0-1.5x, the validated payload range from cw-walk-payload50). Every payload compose so far (payload-dr05, joyfric-payload, joyhead90-payload, jointtiltpayload) has been on the DEFAULT flat-height gait; crouch+payload is untested and directly relevant to hardware (carrying something while low). ONE variable off cw-walk-lowgait-dr035: add dr.mass_scale=1.0,1.5 cfg-set override (payload only, no other DR change). Prediction-if-true: own-cfg panel (DR0.35 + payload) gv 12/12, 0 term, height err <=10mm, slip/m med <=1.6; DR0 no-payload retention clean (height err <=8mm, slip <=1.15). Prediction-if-false: payload destabilizes the crouch specifically (height ref drifts further under load, or flag-leg draws increase) - crouch+payload doesn't compose, stays two separate skills. Strongest alternative: passes own-cfg but nominal retention erodes like payload-dr05 did (mass-axis DR compose charges nominal walking per c61) - would make this the 2nd example of that pattern outside the driving-package lines.

**gate**: Own-cfg harness at --dr-scale 0.35 + dr.mass_scale=1.0,1.5, height_off_mm=-50, det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; plus DR0 no-payload retention det 6/6 gv, height err <=8mm, slip/m <=1.15; frames watched det

**verdict**: PASS — crouch(-50mm)+payload(1.0-1.5x mass) compose is NEW, untried pairing, holds. Own-cfg DR0.35+payload det+sto 6/6 gv 12/12, 0 term, mean end-height err ~4mm (<=10mm gate), slip/m med 1.20/1.25 (<=1.6 gate); DR0 no-payload retention det 6/6 gv, mean height err 4.8mm (<=8mm), slip/m med 1.08 (<=1.15). One det + a few sto fixed-draw churn/slow-shuffle outliers (prog 0.62-0.84, slip 1.9-2.3) match the known heavy-tail canary pattern seen elsewhere, not a new pathology — frames show normal low six-leg gait throughout, no flag leg, no falls. Crouch and payload compose cleanly.

