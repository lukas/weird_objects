# cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-26T03:31:22+00:00

**pod**: hexapod-mjx-train-7

**steps**: 8000000

**hypothesis**: Seed-1 twin of cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m: can the mesh robot learn ALL THREE stance skills (quiet hold, rise-from-flat, lower-to-sit) in one policy if the proven flat-rise recipe is trained FROM SCRATCH inside the 3-way mix (instead of warm-started) with the real std 1.0->0.018 exploration arc, on an independent seed? Answers the joint 2-seed pass-rate question this pair is registered to decide in one batch, same recipe as seed 0, seed=1 throughout.

**gate**: ACQUISITION (8M, judged jointly with seed-0): PASS if flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto 6+6, DR-0) >=10/12 valid_plant per seed with no majority 2.64A OC-pin, AND hold det+sto >=5/6+5/6 zero-term, AND lower >=5/6 honest (<=10mm herr) per seed -> from-scratch is THE mesh stancemix recipe; promote the better seed's checkpoint and move to STAND_HEIGHT rungs / walk distill. PARTIAL if one seed passes all clauses and the other shows a trough-but-rising trajectory at 8M -> continue the lagging seed per the 08-21 ruling before any recipe verdict. FAIL if both seeds are budget-invariant on the flat clause vs their own 2M mark or hold/lower never converge -> mix-diet interference is structural even under full exploration; next lever is staged/frozen-rise curriculum or per-mode gradient isolation, NOT more seeds/budget, and the track weighs promoting stdreopen-acq8m-s1's checkpoint as stage-1 output.

**refused_reason**: hexapod-mjx-train-7 already runs cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m-s1 — GPU pods host exactly one run; pick a free GPU pod.

