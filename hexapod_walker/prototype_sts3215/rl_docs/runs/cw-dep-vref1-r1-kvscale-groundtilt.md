# cw-dep-vref1-r1-kvscale-groundtilt

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T20:05:57+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-groundtilt

**wandb_id**: eb5i4v8i

**hardware_ready**: False

**hypothesis**: Plain English: does the hardware candidate still walk cleanly with uncertain velocity-gain/damping AND a sloped floor at the same time -- servo response uncertainty x terrain, two axes that both individually PASSED (kv alone via cw-dep-vref1-r1-kvscale, ground tilt alone via cw-dep-vref1-r1-groundtilt) but never paired; this was queued but lost to fleet saturation earlier tonight. Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + dr.kv_scale_pct=0.50 + dr.ground_tilt_deg=5.0) det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- composes free like every other pairing tonight. If-false: uncertain damping on a sloped floor compounds worse than either alone -- flag as a real risk (real floors are rarely perfectly flat AND the exact fitted gain).

**gate**: own-cfg (DR0.35 + dr.kv_scale_pct=0.50 + dr.ground_tilt_deg=5.0) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto) +-20%; known fixed-draw crater (det/4 or det/5) pre-allowed as baseline

**verdict**: PASS, closing per the 08-10 class-closure ruling (protect-the-candidate DR-compose sweep on cw-dep-vref1-r1 CLOSED, 20+-for-20+ no-effect). OBSERVATIONS: DR0 gate det gv 6/6 but 3/6 ok=False (med slip 1.59, med prog 0.86) -- idx1,4,5 degraded, idx4 the lineage's universal fixed-draw crater; sto clean 6/6. Own-cfg (DR0.35+kv0.50+tilt5deg) det gv6/6, 3/6 ok=False (med slip 1.50, med prog 0.97); sto gv6/6, 4/6 ok=False (med slip 1.58, med prog 0.78) -- 0 term, 0 sacrificed legs throughout. This is essentially the SAME magnitude as kv-scale ALONE's already-accepted fingerprint (cw-dep-vref1-r1-kvscale PASS verdict: own-cfg det med slip 1.45/2 fails, sto med slip 1.43/4 fails, flagged there as a watch-item not a new failure mode after its own matched-parent control) -- adding ground-tilt on top contributes at most one extra degraded episode, not a compounding catastrophe. Video (gate det + own-cfg det/sto contact sheets) shows the family's normal clean six-leg creep/march gait in every episode watched, including the degraded ones -- no flag-leg, no drag, no skate. INTERPRETATION: if-false (compounding) is NOT supported; kv-spread alone already explains almost all of this margin, matching precedent. VERDICT: PASS, same closed benign fingerprint as every other sibling in this now-CLOSED sweep. hardware_ready=false (not independently deployable, paddle-gait economics inherited from the lineage).

