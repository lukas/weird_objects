# cw-dep-vref1-r1-badstartdeg

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T18:55:33+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly when a badly-placed joint is off by a LARGER angle, separate from testing it happening more OFTEN (companion run cw-dep-vref1-r1-badstart tests probability; this tests magnitude). If-true: own-cfg (DR0.35 + dr.bad_start_deg=8,50 widened from the nominal 8-35deg ceiling) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- larger single-joint placement errors alone are absorbed. If-false: the wider offset breaks gait_valid or inflates slip -- placement-error MAGNITUDE, not just frequency, is a real pre-attempt-#2 risk worth isolating from the zero-drift-frame mechanism blamed in today's startvar1 failure.

**gate**: own-cfg (DR0.35 + dr.bad_start_deg=8,50) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4) is pre-allowed as baseline

