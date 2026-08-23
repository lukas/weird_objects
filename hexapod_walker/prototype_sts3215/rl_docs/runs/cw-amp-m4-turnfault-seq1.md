# cw-amp-m4-turnfault-seq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T03:40:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: bl8slho8

**hypothesis**: Plain English: the clean turn substrate (yawcmd0-r2, tip err 0.15/0.16) loses its turn accuracy the moment PUSH is grafted onto it (turnpush1-style05-acq1-r2: tips erode to 0.38/0.43) -- nobody has tested whether FAULT alone, grafted the same way (sequentially, onto the same clean checkpoint, skipping push), does the same damage or preserves tip tracking better. This isolates whether push specifically is the erosion driver, or whether ANY added axis erodes turn regardless of type (the fresh 3-way turnfault1 stack parked totally at 0.30, but that was never taught turn at all, so it cannot answer this). Single lever vs the champion: dr.fault_prob=1.0 + obs.fault_health=1 added, push cfg left OFF entirely.

**gate**: Own-cfg DR-0 gate (fault_prob=1.0 baked into eval, matching faultobs2/turnfault1 practice): mechanism-safety floor gait_valid>=9/12 (turnfault1-style05-acq1-r5's own fresh-stack bar). Hand-run eval_yaw (matched fast-servo cfg, wz-max 0.3): PRESERVED branch = tips stay <=0.20-0.25 (much better than push's 0.38/0.43) => names PUSH specifically as the erosion driver, points M4/M5 composition order at fault-first. EQUAL-EROSION branch = tips land ~0.35-0.45 (matching push's own erosion) => any additional axis erodes turn regardless of type, rules out push-specific mechanism. TOTAL-PARK branch = tips ~0.30 (matching turnfault1 fresh-stack) => fault composition destroys the learned tip skill more completely than push did.

**verdict**: Result: FAULT-only sequential composition PRESERVES turn tip-tracking almost fully -- names PUSH specifically as the M4 turn-erosion driver, not 'any added axis'. Evidence: own-cfg DR-0 walk gate gait_valid 11/12 (6/6 det + 5/6 sto), 0 terminations, one legit carried-leg (sac=[2] sto/0, matches the known M4 fault-carried-leg pattern, video confirms clean 6-leg cycling not a topple) -- comfortably clears the 9/12 safety floor; det prog med 0.965/slip 3.69 and sto prog 0.53/slip 7.29 track the parent (0.946/2.24) and the fresh 3-way stack (0.902/4.52) closely, no walk-side collapse. Hand-run eval_yaw via the m5 suite (matched fast-servo cfg, dr.fault_prob explicitly zeroed for the yaw check): tip-left/right err 0.1818/0.1708 -- lands the PRESERVED branch cleanly (bar <=0.20-0.25) and nearly matches the parent yawcmd0-r2's own 0.1525/0.1614, versus push's 0.38/0.43, turnpushfault1-style05's 0.42/0.49, and the fresh-stack total-park 0.30/0.30. Directly refutes both the EQUAL-EROSION and TOTAL-PARK branches. Why: fault (a per-episode single-leg degradation) apparently does not compete with the turn-tracking objective the way push (an external disturbance requiring whole-body recovery torque) does -- push, not axis-count, is the erosion mechanism. Caveat found while building the m5 read: eval_yaw's scripted stop-hold scenario recorded ONE fall (deterministic, reproducible) where the matched parent recorded zero (logs/ckpt_eval/cw_amp_m2_turnclone_yawcmd0_r2_yawgate.json) -- a real but narrow anomaly (single scripted trial, zero-command hold, doesn't move the tip-tracking conclusion) worth a follow-up check, not blocking this verdict. Evidence: logs/ckpt_eval/cw_amp_m4_turnfault_seq1_gate/ (report+video), logs/ckpt_eval/cw_amp_m4_turnfault_seq1_m5/ (m5_verdict.json, yaw.json). What's next: this checkpoint is now the best turn-preserving hazard substrate available -- graft PUSH onto it next (fault-then-push order) to test whether sequencing push AFTER fault (instead of push directly onto the clean turn substrate, as turnpush1-style05 did) avoids or reduces the erosion; this is the concrete M4/M5 composition-order lever the hypothesis was built to unlock.

