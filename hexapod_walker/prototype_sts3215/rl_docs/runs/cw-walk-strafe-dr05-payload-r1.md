# cw-walk-strafe-dr05-payload-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T02:51:18+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-strafe-dr05-payload

**wandb_id**: hgdgavtx

**hardware_ready**: False

**hypothesis**: Retry (2nd attempt): base died 0-steps to a launch-collision EOFError (gotcha 13b) during the fleet-wide storm, no science result. Same spec unchanged: lateral/omnidirectional strafing (DR0.5, +/-90deg) x chassis payload compose, untested combo -- payload composes for free onto forward-biased driving packages, unknown whether that holds for lateral gait.

**gate**: own-cfg (strafe-DR0.5+payload) det+sto gv 6/6 @15s, 0 term, prog med within strafe-dr05's own band; DR0 no-payload retention gv 6/6; frames watched det

**verdict**: PASS: lateral/omnidirectional strafe (DR0.5, +-90deg) x chassis payload compose holds. Own-cfg det/sto prog med 0.95/0.95 (>=0.8 gate), slip med 2.04/1.98 (<=2.4 gate), gv 12/12, 0 term. DR0 no-payload retention det/sto prog med 1.04/1.08, slip med 2.15/1.87 (<=2.4), gv 12/12, 0 term -- matches parent strafe-dr05-payload's own band. Video: six legs cycling, level body, no flag leg on any draw; same paddling/slip character as the rest of the walk lineage, nothing new. Payload composes free onto the lateral driving package too.

