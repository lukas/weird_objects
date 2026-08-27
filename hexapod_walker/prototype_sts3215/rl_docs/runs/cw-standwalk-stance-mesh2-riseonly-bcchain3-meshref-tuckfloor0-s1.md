# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T18:18:33+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**hypothesis**: Seed twin of tuckfloor0 (train.bc_anchor_min_h_ahead_mm 8->0 on the exact meshref canary recipe, seed 1): does removing the height-floor lookahead that skips the ref's height-flat tuck phase teach the flat-start tuck robustly across seeds, or was any single-seed result init luck? Judged jointly with tuckfloor0 per the pair gate.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: same flat-pinned probe + standard DR-0 gate as tuckfloor0; joint pair read. PASS if BOTH seeds clear flat det>=4/6 AND sto>=4/6 valid_plant, zero over_current, h_err_end<=10mm on valid eps, non-flat kinds not regressed vs the meshref canary pair -> fund 3-seed 8M acquisition with floor=0. PARTIAL if seeds disagree or both land in the PARTIAL band (>=2/6 flat valid or over_current halved with genuine tuck motion on video) -> extend budget or combine with flatmix. FAIL if both stay 0-1/12 flat valid with the zero-swing 2.64A press-up and no tuck motion -> floor removal refuted; next is the code-level phase-capped floor or tuck-segment curriculum. Watch for new press-phase pursuit stalls (low-current mid-press freeze) - counts against PASS.

**refused_reason**: a process for cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0-s1 already exists on hexapod-mjx-train-1

