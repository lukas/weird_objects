# cw-dep-vref1-r1-badstartdeg

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T18:55:33+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: ycbtuxty

**hardware_ready**: False

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly when a badly-placed joint is off by a LARGER angle, separate from testing it happening more OFTEN (companion run cw-dep-vref1-r1-badstart tests probability; this tests magnitude). If-true: own-cfg (DR0.35 + dr.bad_start_deg=8,50 widened from the nominal 8-35deg ceiling) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- larger single-joint placement errors alone are absorbed. If-false: the wider offset breaks gait_valid or inflates slip -- placement-error MAGNITUDE, not just frequency, is a real pre-attempt-#2 risk worth isolating from the zero-drift-frame mechanism blamed in today's startvar1 failure.

**gate**: own-cfg (DR0.35 + dr.bad_start_deg=8,50) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4) is pre-allowed as baseline

**verdict**: PASS -- widening the bad-start joint-placement-error ceiling from the nominal 8-35deg ambient to 8-50deg composes free onto the contract-exact hardware base, isolating placement MAGNITUDE from the zero-drift-frame mechanism blamed in today's startvar1 failure. Own-cfg (DR0.35+bad_start_deg=8,50) det 5/6 ok / sto 6/6 ok, gait_valid 12/12, 0 term, slip/m med det 1.08 (excl crater)/sto 0.97 -- both within vref1-r1's own 0.89-1.13/1.13-1.36 band. The one det fail (idx4, prog 0.16 slip 11.30) is the lineage's known fixed-draw march-in-place crater seen identically across every prior sibling compose -- video-checked six-leg gait, level body, no flag-leg/fall. Not independently hardware-ready (inherits vref1-r1's paddle-gait economics).

