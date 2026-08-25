# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-fullpace2-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T17:24:35+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: 3ti52q6s

**hypothesis**: Seed twin of -fullpace2 (seed 1): does full chain pace (lookahead 0.5s / min_h 15mm) transfer to the mesh-native rise ref, judged as a 2-seed pass-rate against the meshref s0/s1 canary pair? Same recipe, only the seed differs. Prediction-if-true: replicates fullpace2's read (pair matches/beats meshref det 5/6 + sto 4/6 at same-or-cooler currents). Prediction-if-false: seeds diverge -- pace read is seed-noise-dominated at 2M, keep half-pace default pending the 8M grid.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary, DR-0 rise det+sto n=6+6, judged jointly with -fullpace2 as a pass-rate vs the meshref s0/s1 pair. PASS: det>=5/6 AND sto>=4/6 valid_plant, no valid episode above 2.25A, over_current terms <=3/12. PARTIAL: det 4/6 or mixed read. FAIL: det<=3/6 or oc terms >3/12 or any all-six-leg freeze (cross-check height_err_end_mm).

