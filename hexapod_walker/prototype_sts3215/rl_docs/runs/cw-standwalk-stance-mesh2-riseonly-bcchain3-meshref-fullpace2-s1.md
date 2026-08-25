# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-fullpace2-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-25T17:24:35+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: 3ti52q6s

**hypothesis**: Seed twin of -fullpace2 (seed 1): does full chain pace (lookahead 0.5s / min_h 15mm) transfer to the mesh-native rise ref, judged as a 2-seed pass-rate against the meshref s0/s1 canary pair? Same recipe, only the seed differs. Prediction-if-true: replicates fullpace2's read (pair matches/beats meshref det 5/6 + sto 4/6 at same-or-cooler currents). Prediction-if-false: seeds diverge -- pace read is seed-noise-dominated at 2M, keep half-pace default pending the 8M grid.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary, DR-0 rise det+sto n=6+6, judged jointly with -fullpace2 as a pass-rate vs the meshref s0/s1 pair. PASS: det>=5/6 AND sto>=4/6 valid_plant, no valid episode above 2.25A, over_current terms <=3/12. PARTIAL: det 4/6 or mixed read. FAIL: det<=3/6 or oc terms >3/12 or any all-six-leg freeze (cross-check height_err_end_mm).

**verdict**: CANARY FAIL - MECHANISM: full chain pace does not transfer to the mesh-native rise reference at this seed (det 2/6 valid_plant vs PASS>=5/6; sto 4/6; oc terms 3/12; no valid episode >2.25A; no freezes). The interesting part is the NEW failure mode: 3 of the 4 det misses (bridge/bridge/rsi starts) COMPLETE the rise cleanly (h_err_end 1.4-3.7mm, clean roll, posture ok, cool p95 1.4-2.3A) but end with feet tucked under the body — footprint plant-check fail at margin ~108-110mm vs ~122+ on valid episodes; video confirms a narrow-stance stand, not a freeze or a current wall. Sto keeps meshref's shape (4/6, two rsi over_current at herr 42-44mm). Valid episodes are COOLER than meshref (p95 0.50-1.03A vs 1.19-1.36A medians). Read: pace redose trades the over_current wall for an invalid narrow footprint — worse on the DR-0 plant gate than half-pace meshref (det 5/6). Prediction-if-false branch holds: keep half-pace as the recipe default; the in-flight meshref 8M grid (half-pace) is unaffected. Joint pass-rate read completes when -fullpace2 (seed 0, other cycle) is triaged, but this seed alone already blocks the 'full pace preferred' promotion, which required the PAIR to match/beat meshref. Watcher SUSPECT at 17:35 was a false alarm — run finished its full 2M budget clean (W&B 3ti52q6s, 2,031,616 steps); stall read hit the end-of-run sync window. Next: nothing on this axis; the rise recipe question is owned by the 8M half-pace grid.

