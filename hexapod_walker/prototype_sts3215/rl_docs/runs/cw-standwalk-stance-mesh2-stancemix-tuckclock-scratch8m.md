# cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-26T03:29:02+00:00

**pod**: hexapod-mjx-train-6

**steps**: 8000000

**hypothesis**: Can the mesh robot learn ALL THREE stance skills (quiet hold, rise-from-flat, lower-to-sit) in one policy if the proven flat-rise recipe is trained FROM SCRATCH inside the 3-way mix, instead of warm-started? Discovery 08-26 (seqrise dig-in): --log-std-init is a silent no-op under --init-from, so every warm-started mix arm (stdreopen family, seqrise pair) actually trained at pinned std 0.0183 and either kept the OC-pin, was seed-fragile at 8M (acq8m 2/12 vs 11/12), or erased a solved rise (seqrise 0/12+0/12). The only 2/2-reliable solver of mesh flat rise is the from-scratch riseonly-tuckclock recipe with the real std 1.0->0.018 anneal arc; this pair runs that exact arc on the mix diet (hold=0.1,rise=0.45,lower=0.45) at the same 8M budget.

**gate**: ACQUISITION (8M, judged jointly with -s1): PASS if flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto 6+6, DR-0) >=10/12 valid_plant per seed with no majority 2.64A OC-pin, AND hold det+sto >=5/6+5/6 zero-term, AND lower >=5/6 honest (<=10mm herr) per seed -> from-scratch is THE mesh stancemix recipe; promote the better seed's checkpoint and move to STAND_HEIGHT rungs / walk distill. PARTIAL if one seed passes all clauses and the other shows a trough-but-rising trajectory at 8M -> continue the lagging seed per the 08-21 ruling before any recipe verdict. FAIL if both seeds are budget-invariant on the flat clause vs their own 2M mark or hold/lower never converge -> mix-diet interference is structural even under full exploration; next lever is staged/frozen-rise curriculum or per-mode gradient isolation, NOT more seeds/budget, and the track weighs promoting stdreopen-acq8m-s1's checkpoint as stage-1 output.

**refused_reason**: hexapod-mjx-train-6 already runs cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m — GPU pods host exactly one run; pick a free GPU pod.

