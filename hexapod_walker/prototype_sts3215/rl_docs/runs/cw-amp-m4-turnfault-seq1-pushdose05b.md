# cw-amp-m4-turnfault-seq1-pushdose05b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T04:44:07+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: 7g4kw0h2

**hypothesis**: Plain English: same push-dose-sensitivity test as pushdose025, at the middle of the range. pushcont1 (dose=1.0, fault-first order) softened but did not fix push-driven turn erosion (0.27/0.30 vs fault-only parent's 0.18/0.17). Single lever vs pushcont1: dr.ext_push_prob 1.0 -> 0.5. Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint, per the init-basin rule. (Renamed -05b: the -05 name's tag already exists from a snapshot-then-refuse race, see OPERATOR_QUESTIONS q_20260823T0215Z residual defect -- no trainer ever started under that name.)

**gate**: Same gate as pushdose025: own-cfg DR-0 floor gait_valid>=9/12; eval_yaw tip-left/right err PASS-clean<=0.20-0.25, PARTIAL between that and 0.27/0.30, FLAT ~unchanged. eval_amp_m5 push section at eval-time ext_push_prob=1.0 confirms push-recovery survives the lower training dose.

**verdict**: Part of the 3-arm push-probability dose sweep (see pushdose025's verdict for the joint finding). This arm (dose=0.5): eval_yaw tip-left/right err 0.2394/0.2571 -- the closest-to-passing of the three doses (tip-left grazes the 0.20-0.25 preserved band's edge) but tip-right still misses both the preserved band and the m5 0.20 bar; same 0.24-0.27 cluster as dose=0.25 (0.264/0.269) and dose=0.75 (0.261/0.255), no monotonic dose-response. Mechanism-safety floor clears (gait_valid 11/12, 2 terms, 1 legit carried-fault leg [2]), video clean six-leg cycling. Confirms FLAT: dose is not the lever.

