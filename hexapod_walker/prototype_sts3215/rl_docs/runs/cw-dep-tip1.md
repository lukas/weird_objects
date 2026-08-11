# cw-dep-tip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-11T00:27:19+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Tipped-start DR (dr.tipped_start_*, landed 08-10: plant/park episodes begin at a settled 6-18deg body roll with a LEVEL tilt reference) teaches sustained-lean recovery, the capability whose absence rolled the deployed walk into the 25deg trip on hardware (HARDWARE.md runaway roll 08-10). Full dose 0.30 vs the dr-scale default 0.105; parent baseline SCORE/tipped_recovery_success = 0.25 (probe 08-10, 12deg dose, 4 eps).

**gate**: PASS if SCORE/tipped_recovery_success >= 0.75 (12deg dose, det) AND matched-parent eval_checkpoint walk retention under identical config/seed: progress_ratio in 0.75-1.25 and slip_per_m not >20% worse than the frozen parent. FAIL if recovery < 0.5 or walk retention broken. Behavioral check on video: recovery must be stepping/weight-shift, not belly-drop (z_drop_mm <= 30 is already in the metric).

**refused_reason**: discovery runs cap at 2000000 steps (asked 18000000): the question is 'did qualitatively correct behavior emerge?' — if it already did, relaunch as --phase hardening with --evidence.

