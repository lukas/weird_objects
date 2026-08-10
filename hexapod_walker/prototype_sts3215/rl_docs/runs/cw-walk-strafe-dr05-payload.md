# cw-walk-strafe-dr05-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T02:34:24+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-strafe-dr05

**hypothesis**: Lateral/omnidirectional strafing (PASSED at DR0.5, +/-90deg base) has never been composed with chassis payload. Payload composes for free onto forward-biased driving packages (joyheadfric-payload PASS) but strafing loads legs asymmetrically (sideways shear vs forward drag) so free-composability isn't guaranteed. One variable off cw-walk-strafe-dr05: add dr.mass_scale=1.0,1.4. If-true: own-cfg DR0.5+payload det+sto 6/6 gv, 0 term, prog med>=0.8, slip/m<=2.4; DR0 no-payload retention clean. If-false: lateral loading under payload produces a flag-leg or fall the forward-driving payload composes never showed.

**gate**: Own-cfg (--dr-scale 0.5 + dr.mass_scale=1.0,1.4, lateral +/-90deg) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog median>=0.8, slip/m med<=2.4; DR0 no-payload retention det 6/6 gv, slip/m<=2.4; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

