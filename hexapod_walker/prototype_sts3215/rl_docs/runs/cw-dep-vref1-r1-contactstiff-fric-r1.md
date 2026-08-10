# cw-dep-vref1-r1-contactstiff-fric-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T20:02:24+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1-contactstiff

**hypothesis**: Plain English: does the hardware checkpoint still walk cleanly on a floor that is BOTH squishy/stiff (already PASSed alone) AND has uncertain grip (friction 0.4-1.6x, already PASSed alone) at the same time -- the realistic case, since a real floor's compliance and grip vary together, not independently? (r1: first attempt cw-dep-vref1-r1-contactstiff-fric died before training to a stale pod-code REFUSED + orphaned git tag, no science; retrying under a fresh name.) Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- composes free like every other floor-realism pairing tonight (fric+groundtilt5 already PASSed). If-false: compliance and grip uncertainty compound (a soft AND slick floor is worse than either alone) -- flag as a real pre-attempt-#2 floor-realism risk.

**gate**: own-cfg (DR0.35 + dr.contact_stiff_scale=0.7,2.0 + dr.friction_scale=0.4,1.6) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; known fixed-draw crater (det/4) pre-allowed as baseline

