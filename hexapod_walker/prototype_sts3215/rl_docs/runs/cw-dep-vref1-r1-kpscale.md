# cw-dep-vref1-r1-kpscale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T18:59:21+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 52fiotsr

**hardware_ready**: False

**hypothesis**: Plain English: isolate WHICH half of today's gainvar compose (2x position-gain spread + 2x velocity-gain spread together, PASSED but with the widest slip margin yet -- ~12% over the tolerance band edge) is driving that margin, by widening only the position-gain (kp) spread and leaving velocity-gain (kv) at its normal DR0.35 baseline. If-true: own-cfg (DR0.35 + dr.kp_scale_pct=0.40, double nominal) det+sto 6/6 gv, 0 term, slip/m comfortably within vref1-r1's own band (not near the gainvar edge) -- kp alone is not the margin driver, implicating kv or the combination. If-false: kp alone reproduces gainvar's wide margin -- kp-spread is the dominant actuator-gain risk, useful before stacking it with other axes (gainvar-torquescale/legmass, already training).

**gate**: own-cfg (DR0.35 + dr.kp_scale_pct=0.40) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4) is pre-allowed as baseline

**verdict**: kp-spread alone (2x nominal, dr.kp_scale_pct=0.40) composes free with vref1-r1: DR0 gate gv 6/6 both modes with only the pre-allowed det/4 crater; own-cfg (DR0.35+kp) det slip med 1.08 / sto slip med 1.17 land INSIDE vref1-r1's own band (0.89-1.13 det / 1.13-1.36 sto), not near gainvar's wide-margin edge. Kp-spread alone is NOT the driver of gainvar's widest-yet slip margin -- implicates kv-spread or the kp x kv interaction. Clean six-leg video both passes; part of the now-CLOSED dep-vref1-r1 DR-compose sweep (20-for-20+ no-effect).

