# cw-dep-vref1-r1-groundtilt

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T19:18:05+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: hklqnfv1

**hardware_ready**: False

**hypothesis**: Plain English: test the floor-slope axis ALONE on the hardware candidate, as a clean baseline before today's several floor-tilt PAIR composes (fric-groundtilt5, deadband-groundtilt5, imumount-groundtilt5) -- none of those isolated whether ground_tilt_deg by itself is benign, only whether it stays benign paired with another axis. If-true: own-cfg (DR0.35 + dr.ground_tilt_deg=5.0, 2.5x nominal) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- floor slope alone is a non-issue. If-false: floor slope alone already breaks the gate -- the paired composes' PASSes are suspect and need re-examination for whether the OTHER axis in each pair was masking a real floor-slope sensitivity.

**gate**: own-cfg (DR0.35 + dr.ground_tilt_deg=5.0) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4 or det/5) is pre-allowed as baseline

**verdict**: PASS -- floor slope (5deg) tested ALONE for the first time (previously only in PAIR composes fric+tilt, deadband+tilt, imumount+tilt), confirming those PASSes were not masking a real slope sensitivity. DR0-gate det slip/m med 1.09/sto med 0.99 (in vref1-r1's own band). Own-cfg (DR0.35+dr.ground_tilt_deg=5.0) det+sto gv 6/6, 0 term, slip med 1.06 det/1.13 sto (in band); det/5+sto/0+sto/1 crater cluster is the known dig-in-resolved fixed-seed DR0.35 fingerprint (sac=[], gv=True), not new. Video clean six-leg creep, no flag-leg/fall. Not independently hardware-ready.

