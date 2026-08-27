# cw-standwalk-stance-mesh2-riseonly-bcchain3-tuckrise45-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T20:04:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-tuckrise45

**hypothesis**: Seed twin of the concurrent cycle's tuckrise45 (rise_ref_mesh_tuckrise45.npz, floor/lookahead/coef unchanged at meshref's own values): does the 45mm tuck-rise dose hold up across seeds, or was any read on seed 0 luck? Same joint-pair discipline as every mechanism canary this campaign (meshref-s1, flatmix70-s1, tuckfloor0-s1, tuckexempt0-s1). Per the STATUS ~19:5x by-construction CPU probe, this ref's achieved height only starts climbing right at the tuck/press boundary (first 8mm ~tick258 vs tuck-end 250) so the pre-registered prediction-if-false (ref-content height-shaping refuted, same never-tucks pin or freeze as its siblings) is the likely outcome -- this twin exists to cross-verify that read with a second seed before the axis is fully closed, not because the mechanism looks promising in isolation. Judged jointly with tuckrise45 per its own gate.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same flat-pinned probe (goal.rise_flat_frac=1.0 rise_partial_frac=0 rise_rsi_frac=0, det+sto n=6+6, DR-0) + standard DR-0 gate as tuckrise45. PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant (genuine duty>0 tuck-then-press motion, not a freeze) AND standard-gate non-flat kinds at-or-above the meshref parent (det 5/6 + sto 4/6, oc<=3/12) -> promote to an 8M acquisition grid + port into stancemix. PARTIAL if flat-probe shows ANY genuine duty>0 tuck motion (even short of valid_plant) while non-flat holds >= meshref parent -> extend budget. FAIL if flat stays 0-1/12 valid with either the original 2.64A press-up pin or a duty=0 freeze, or non-flat regresses below the meshref parent -> ref-content height-shaping fully refuted (cross-verified 2/2 seeds) alongside anchor-floor plumbing and mid-tuck curriculum -> the anchor-floor progress-metric redesign (script-index/commanded-height keyed) is the sole surviving lever, DIG-IN territory, not routine triage.

**refused_reason**: hexapod-mjx-train-1 already runs cw-standwalk-stance-mesh2-riseonly-bcchain3-tuckrise45-s1 — GPU pods host exactly one run; pick a free GPU pod.

