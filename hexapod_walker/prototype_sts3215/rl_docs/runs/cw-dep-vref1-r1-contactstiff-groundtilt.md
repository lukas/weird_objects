# cw-dep-vref1-r1-contactstiff-groundtilt

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T20:17:53+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1-groundtilt

**hypothesis**: Plain English: does the hardware candidate still walk cleanly on a floor that is BOTH sloped AND has unknown squishiness (soft carpet vs hard tile) at the same time -- two floor-realism axes that both individually PASSED (contact compliance via cw-dep-vref1-r1-contactstiff, slope via cw-dep-vref1-r1-groundtilt) but never paired; the operator's real floor is both unknown-tilt and unknown-compliance simultaneously. Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + dr.contact_stiff_scale=0.7,2.0 + dr.ground_tilt_deg=5.0) det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- composes free like every other floor-realism pairing tonight (fric+groundtilt5 already PASSed). If-false: soft/stiff compliance on a slope compounds worse than either alone (plausible: reduced effective normal load on the downhill side interacting with variable contact stiffness) -- flag as a real pre-attempt-#2 floor risk.

**gate**: own-cfg (DR0.35 + dr.contact_stiff_scale=0.7,2.0 + dr.ground_tilt_deg=5.0) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto) +-20%; known fixed-draw crater (det/4 or det/5) pre-allowed as baseline

