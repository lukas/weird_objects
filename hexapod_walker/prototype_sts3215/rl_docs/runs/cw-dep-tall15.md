# cw-dep-tall15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T20:44:54+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tall30

**wandb_id**: hgz0ptdn

**hypothesis**: TALL DEPLOYABLE WALKER rung 2 of the height-ref ladder. Rung 1 verdict (tall30 vs tall30h isolation, both 2M): height ref -30 cut eval walk height_err_end from parent tip1s 60mm to 15mm; the k_drag_stance=8000 charge was NOT the speed suppressor (charge arm 0.0295 m/s / slip 1.74 vs no-charge 0.0287 / 1.80 - identical to slightly better), so the charge rides along. This rung: warm from cw-dep-tall30, height ref -30 -> -15 (+15mm, the lowgait rung size). Cost model so far: ~25% of parent speed bought ~45mm of posture; watching whether the cost is per-rung or one-time.

**gate**: PASS: eval walk height_err_end <=8mm at -15 ref, speed >=0.028 m/s (no further loss vs rung 1), survived_frac 1, slip <=1.8, no park. If PASS: final rung 0 (full plant height). If height stalls >8mm or speed collapses: envelope edge - ship the -30 rung as the tall candidate instead.

