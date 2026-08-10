# cw-dep-vref1-r1-kvscale-groundtilt

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T20:05:57+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-groundtilt

**hypothesis**: Plain English: does the hardware candidate still walk cleanly with uncertain velocity-gain/damping AND a sloped floor at the same time -- servo response uncertainty x terrain, two axes that both individually PASSED (kv alone via cw-dep-vref1-r1-kvscale, ground tilt alone via cw-dep-vref1-r1-groundtilt) but never paired; this was queued but lost to fleet saturation earlier tonight. Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + dr.kv_scale_pct=0.50 + dr.ground_tilt_deg=5.0) det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- composes free like every other pairing tonight. If-false: uncertain damping on a sloped floor compounds worse than either alone -- flag as a real risk (real floors are rarely perfectly flat AND the exact fitted gain).

**gate**: own-cfg (DR0.35 + dr.kv_scale_pct=0.50 + dr.ground_tilt_deg=5.0) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto) +-20%; known fixed-draw crater (det/4 or det/5) pre-allowed as baseline

