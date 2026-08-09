# cw-walk-latjit-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T20:44:57+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-latjit25

**wandb_id**: deicmkfh

**hardware_ready**: False

**hypothesis**: Ruling-7 seed twin of cw-walk-latjit-dr05 (this cycle's cleanest DR0.5 compose PASS: det fwd med 1.50m, det slip 1.04 = champion band under DR+latency, retention clean). Identical config, seed 0 -> 1. Plain: confirm the latency+DR robustness recipe is not a seed fluke before it feeds the multi-axis consolidation. If-true: same band (own-cfg gv 12/12, 0 term, det fwd med >=1.1m, DR0 retention slip <=1.24) - recipe is seed-robust, safe to stack. If-false: seed-specific (gate miss or retention erosion) - compose PASSes need panels before feeding multiaxis. Strongest alternative: passes but with fricvar-dr05-style sto slip creep - then the tail, not the median, is seed-varying.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.latency_scale=0.5,2.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1 m; plus DR0 no-jitter retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: FAIL on retention (seed twin of latjit-dr05, md5 6a28e5e0). Own-DR0.5+latency panel replicates the parent CLEANLY: gv 12/12, 0 term, det fwd med 1.52m (gate >=1.1m), det/sto slip med 1.08/1.31 (parent 1.04/-) — own-cfg performance is seed-robust. BUT the DR0 no-jitter retention gate (det slip/m <=1.24) is MISSED: det slip med 1.26, driven by a REAL seed-specific tail — episodes 3-5 (same fixed command draws the harness gives every checkpoint at eval seed 0) shuffle at half/third speed (prog 0.62/0.48, slip 2.45/3.16) where the PARENT's identical draws were fully clean (prog 0.90-1.07, slip 0.99-1.19, 6/6 ok). This is not eval noise — it is the exact seed-fluke risk the ruling-7 panel exists to catch: seed 0's clean nominal retention did NOT generalize to seed 1. Frames (det eps 4/5): normal paddle shuffle, all six legs still cycling, no flag leg/dragging — a speed/slip regression, not a new pathology. Conclusion: the latjit-dr05 recipe's OWN-DR competence is seed-robust, but its DR0 retention is NOT — treat seed 0 (parent cw-walk-latjit-dr05) as the compose input for multiaxis/consolidation, not this seed. No requeue; this is the informative outcome the panel was designed to produce.

