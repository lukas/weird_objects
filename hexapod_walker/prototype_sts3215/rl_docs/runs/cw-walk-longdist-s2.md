# cw-walk-longdist-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T15:03:05+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: 7b3w13vp

**hardware_ready**: no

**hypothesis**: OPERATOR (08-09): third seed of the champion recipe (ruling-7 panel). The robot walks by this recipe better than any before; a third seed tells us how much was recipe vs luck. Same config as longdist-r2/s1, seed 2.

**gate**: DR0 det 6/6: median fwd dist >=1.2m @30s, 0 term, gait_valid, det slip/m <=1.10 (r2 0.96 / s1 0.94 band)

**verdict**: PASS. DR0 det 6/6: med fwd 1.54m@30s (gate >=1.2), det slip/m med 1.07 (gate <=1.10; r2 0.96 / s1 0.94), prog med 0.96, gv 6/6, 0 term. Sto 5/6 useful; the single miss is THE SAME fixed draw that stalls r2 and s1 (ep4: prog 0.12, slip/m 11.1, in-place churn, no fall) - known lineage canary, not seed pathology. Frames det: level six-leg alternating gait the full 30s, no flag leg. The 30s narrow-band recipe is now 3/3 seed-robust; champion stays r2 (s2 slip slightly worse: 1.07 vs 0.94-0.96). Not hardware-ready (slip).

