# cw-dep-vref1-r1-contactstiff-fric-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T20:02:24+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1-contactstiff

**wandb_id**: 80dutw6y

**hardware_ready**: False

**hypothesis**: Plain English: does the hardware checkpoint still walk cleanly on a floor that is BOTH squishy/stiff (already PASSed alone) AND has uncertain grip (friction 0.4-1.6x, already PASSed alone) at the same time -- the realistic case, since a real floor's compliance and grip vary together, not independently? (r1: first attempt cw-dep-vref1-r1-contactstiff-fric died before training to a stale pod-code REFUSED + orphaned git tag, no science; retrying under a fresh name.) Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- composes free like every other floor-realism pairing tonight (fric+groundtilt5 already PASSed). If-false: compliance and grip uncertainty compound (a soft AND slick floor is worse than either alone) -- flag as a real pre-attempt-#2 floor-realism risk.

**gate**: own-cfg (DR0.35 + dr.contact_stiff_scale=0.7,2.0 + dr.friction_scale=0.4,1.6) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; known fixed-draw crater (det/4) pre-allowed as baseline

**verdict**: PASS -- squishy-floor (contact stiffness 0.7-2.0x) AND uncertain-grip (friction 0.4-1.6x) compose together onto the hardware candidate, same fingerprint as every other floor-realism pairing tonight. DR0-gate: gv 6/6 det+sto, 0 term, slip med 1.18 det/1.17 sto (in band); det/4 crater (known baseline) plus a slightly-elevated det/5 (slip 1.42, still gait_valid, no fall) -- a touch noisier than the single-axis fingerprint but not a new failure mode, matching the friction-alone finding that this axis nudges slip up without breaking gait. Own-cfg DR0.35: gv 6/6 both modes, 0 term, slip med 1.24 det/1.37 sto (in +-20% tolerance); det/5+sto/0-1 crater cluster is the known fixed-seed DR0.35 fingerprint. Video clean six-leg gait throughout, no flag-leg/dragging/fall. Not independently hardware-ready; compliance+grip compound no worse than either alone.

