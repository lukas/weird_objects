# cw-amp-m4-turnfault-seq1-pushcont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T04:10:49+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1

**wandb_id**: 2redye2g

**hypothesis**: Plain English: turnfault-seq1 just showed that grafting FAULT (not push) onto the clean turn champion keeps tip-tracking almost intact (0.18/0.17 vs parent 0.15/0.16) -- but grafting PUSH directly onto the same clean turn substrate (turnpush1-style05-acq1-r2) eroded tips to 0.38/0.43, and grafting fault onto THAT already-eroded turn+push substrate (turnpushfault1-style05-r2) never repaired it (0.42/0.49). Nobody has tested pushing the ORDER around: does grafting PUSH onto an already-fault-preserved turn substrate erode tips as badly as pushing onto the plain turn substrate did, or does the fault-first path leave the turn skill in a more robust state that push disturbs less? Single lever vs turnfault-seq1: add dr.ext_push_prob=1.0 (proven 10-25N single-shove base dose), fault_prob/obs.fault_health left ON exactly as trained.

**gate**: Own-cfg DR-0 gate (both hazards baked into eval): mechanism-safety floor gait_valid>=9/12 (same bar the fault-only run cleared at 11/12). Hand-run eval_yaw (m5 suite, matched fast-servo cfg, both hazards zeroed for the yaw check): PRESERVED branch = tips stay <=0.20-0.25 (matching turnfault-seq1's own 0.18/0.17) => fault-first composition ORDER is the fix, not just fault-vs-push axis identity -- names the M4/M5 recipe. EROSION branch = tips land ~0.35-0.45 (matching turnpush1/turnpushfault1's own erosion) => push erodes turn regardless of what it's grafted onto, ordering doesn't help, the mechanism is push-vs-turn income competition intrinsic to push itself.

