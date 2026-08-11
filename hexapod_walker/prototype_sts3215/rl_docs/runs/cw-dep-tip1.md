# cw-dep-tip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T01:49:32+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: fxxj3e59

**hypothesis**: DISCOVERY (2M, warm from the deployed walk champion): tipped-start DR (dr.tipped_start_*, landed 08-10 — plant/park episodes begin at a settled 6-18deg body roll with a LEVEL tilt reference) teaches sustained-lean recovery, the capability whose absence rolled the deployed walk into the 25deg trip on hardware (HARDWARE.md runaway roll 08-10). Full dose 0.30 vs the dr-scale default 0.105. Parent baseline SCORE/tipped_recovery_success = 0.25 (12deg dose, 4 eps, probe 08-10). Requeue of the 08-10 spec that was REFUSED for asking 18M in discovery; hardening follow-up only with this run as evidence.

**gate**: PASS if SCORE/tipped_recovery_success rises clearly above the 0.25 parent baseline (>= 0.6 at the 12deg dose, det) AND walk retention holds: matched-parent eval_checkpoint under identical config/seed, progress_ratio 0.75-1.25, slip_per_m not >20% worse than the frozen parent. Behavioral check on video: recovery must be stepping/weight-shift, not belly-drop (z_drop_mm <= 30 is in the metric). FAIL if recovery <= baseline or walk broken. PASS -> relaunch 18M as --phase hardening with this run as --evidence.

