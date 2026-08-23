# cw-amp-m4-turnfault-seq1-faultdose075

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T05:09:19+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: pv9s8ts0

**hypothesis**: Plain English: same fault-probability dose test as faultdose025/05, at the high end short of pushcont1's full dose -- see faultdose025 for the full design-tension rationale (q_20260823T0130Z). Single lever vs pushcont1: dr.fault_prob 1.0 -> 0.75, dr.ext_push_prob left at 1.0. Re-inits from the same pre-cheat turnfault-seq1 checkpoint.

**gate**: Same gate as faultdose025/05: own-cfg DR-0 floor gait_valid>=9/12; eval_amp_m5 walk-section terms/gait_valid vs pushcont1's 4 terms/10 gait_valid; secondary eval_yaw tip-err read.

