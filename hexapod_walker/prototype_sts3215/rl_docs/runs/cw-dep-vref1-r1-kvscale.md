# cw-dep-vref1-r1-kvscale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T19:11:35+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: hi893n71

**hypothesis**: Plain English: complete today's decomposition of the gainvar compose (2x position-gain AND 2x velocity-gain spread together, PASSED but with the widest slip margin yet) by isolating the OTHER half -- velocity-gain (kv) spread alone, leaving position-gain (kp) at its normal DR0.35 baseline (companion run cw-dep-vref1-r1-kpscale isolates kp alone). If-true: own-cfg (DR0.35 + dr.kv_scale_pct=0.50, double nominal) det+sto 6/6 gv, 0 term, slip/m comfortably within vref1-r1's own band -- kv alone is not the margin driver either, pointing at the COMBINATION as the real risk. If-false: kv alone reproduces gainvar's wide margin -- kv-spread (velocity-gain/damping uncertainty) is the dominant actuator-gain risk.

**gate**: own-cfg (DR0.35 + dr.kv_scale_pct=0.50) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4 or det/5) is pre-allowed as baseline

