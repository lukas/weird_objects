# cw-amp-m4-turnfault-seq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T03:40:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: bl8slho8

**hypothesis**: Plain English: the clean turn substrate (yawcmd0-r2, tip err 0.15/0.16) loses its turn accuracy the moment PUSH is grafted onto it (turnpush1-style05-acq1-r2: tips erode to 0.38/0.43) -- nobody has tested whether FAULT alone, grafted the same way (sequentially, onto the same clean checkpoint, skipping push), does the same damage or preserves tip tracking better. This isolates whether push specifically is the erosion driver, or whether ANY added axis erodes turn regardless of type (the fresh 3-way turnfault1 stack parked totally at 0.30, but that was never taught turn at all, so it cannot answer this). Single lever vs the champion: dr.fault_prob=1.0 + obs.fault_health=1 added, push cfg left OFF entirely.

**gate**: Own-cfg DR-0 gate (fault_prob=1.0 baked into eval, matching faultobs2/turnfault1 practice): mechanism-safety floor gait_valid>=9/12 (turnfault1-style05-acq1-r5's own fresh-stack bar). Hand-run eval_yaw (matched fast-servo cfg, wz-max 0.3): PRESERVED branch = tips stay <=0.20-0.25 (much better than push's 0.38/0.43) => names PUSH specifically as the erosion driver, points M4/M5 composition order at fault-first. EQUAL-EROSION branch = tips land ~0.35-0.45 (matching push's own erosion) => any additional axis erodes turn regardless of type, rules out push-specific mechanism. TOTAL-PARK branch = tips ~0.30 (matching turnfault1 fresh-stack) => fault composition destroys the learned tip skill more completely than push did.

