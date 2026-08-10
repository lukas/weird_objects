# cw-dep-vref1-r1-groundtilt

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T19:18:05+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: hklqnfv1

**hypothesis**: Plain English: test the floor-slope axis ALONE on the hardware candidate, as a clean baseline before today's several floor-tilt PAIR composes (fric-groundtilt5, deadband-groundtilt5, imumount-groundtilt5) -- none of those isolated whether ground_tilt_deg by itself is benign, only whether it stays benign paired with another axis. If-true: own-cfg (DR0.35 + dr.ground_tilt_deg=5.0, 2.5x nominal) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- floor slope alone is a non-issue. If-false: floor slope alone already breaks the gate -- the paired composes' PASSes are suspect and need re-examination for whether the OTHER axis in each pair was masking a real floor-slope sensitivity.

**gate**: own-cfg (DR0.35 + dr.ground_tilt_deg=5.0) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4 or det/5) is pre-allowed as baseline

