# cw-standwalk-stance-mesh2-holdminload40-bcanchor3-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T12:43:34+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**hypothesis**: Is the BC pose-anchor's escape from the 40mm hover basin seed-robust, or was seed 0 a fluke? Seed twin (seed 1) of the rung-7 dose-3.0 canary that just scored the ladder's first-ever 6/6 det valid_plant (0.7mm height err, 0.53A cur_p95, W&B 80jrhio3). Identical recipe, only the seed changes. Prediction-if-true: >=4/6 det valid_plant at 2M with the same clean-plant signature. Prediction-if-false: the pinned 40mm/2.64A hover signature reappears, meaning dose 3.0's PASS was seed luck and the joint dose read must weight it down. Strongest alternative: partial plant (1-3/6) indicating the anchor helps but the basin escape is slow/seed-dependent.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: 2M, hold DR-0 det+sto n=6+6; >=4/6 det valid_plant + cur_p95<=1.5A = seed-robust PASS; 1-3/6 = PARTIAL; 0/6 pinned 40mm/2.64A signature = seed-fluke evidence. Feeds the joint rung-7 dose read (bcanchor0p5/1/3); does NOT by itself fund or pick the 8M acquisition dose.

**refused_reason**: hexapod-mjx-train-1 already runs cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal — GPU pods host exactly one run; pick a free GPU pod.

