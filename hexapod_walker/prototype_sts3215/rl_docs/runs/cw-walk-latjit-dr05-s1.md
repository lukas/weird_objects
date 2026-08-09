# cw-walk-latjit-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T20:44:57+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-latjit25

**hypothesis**: Ruling-7 seed twin of cw-walk-latjit-dr05 (this cycle's cleanest DR0.5 compose PASS: det fwd med 1.50m, det slip 1.04 = champion band under DR+latency, retention clean). Identical config, seed 0 -> 1. Plain: confirm the latency+DR robustness recipe is not a seed fluke before it feeds the multi-axis consolidation. If-true: same band (own-cfg gv 12/12, 0 term, det fwd med >=1.1m, DR0 retention slip <=1.24) - recipe is seed-robust, safe to stack. If-false: seed-specific (gate miss or retention erosion) - compose PASSes need panels before feeding multiaxis. Strongest alternative: passes but with fricvar-dr05-style sto slip creep - then the tail, not the median, is seed-varying.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.latency_scale=0.5,2.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1 m; plus DR0 no-jitter retention det 6/6 gv, det slip/m <=1.24; frames watched det

