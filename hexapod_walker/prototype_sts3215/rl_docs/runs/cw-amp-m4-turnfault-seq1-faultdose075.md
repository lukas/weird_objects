# cw-amp-m4-turnfault-seq1-faultdose075

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T05:09:19+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: pv9s8ts0

**hypothesis**: Plain English: same fault-probability dose test as faultdose025/05, at the high end short of pushcont1's full dose -- see faultdose025 for the full design-tension rationale (q_20260823T0130Z). Single lever vs pushcont1: dr.fault_prob 1.0 -> 0.75, dr.ext_push_prob left at 1.0. Re-inits from the same pre-cheat turnfault-seq1 checkpoint.

**gate**: Same gate as faultdose025/05: own-cfg DR-0 floor gait_valid>=9/12; eval_amp_m5 walk-section terms/gait_valid vs pushcont1's 4 terms/10 gait_valid; secondary eval_yaw tip-err read.

**verdict**: Same design-tension test at dose=0.75 (see faultdose025 for rationale). Walk-section: gait_valid 11/12, terms=2 -- modest improvement over pushcont1's 10/12+4, smaller than faultdose025's swing, consistent with a dose-sensitive but noisy (n=12) partial effect rather than a clean monotonic fix. Secondary: eval_yaw tip-left/right 0.2116/0.3098, same cluster, fault dose confirmed not a turn-tracking lever. CLOSES the 3-arm fault-dose sweep: mixed fault curricula give real but incomplete, non-monotonic relief on the walk/fault-section design tension (q_20260823T0130Z) -- a proper fix needs either a larger n (this is a 12-episode read) or the harness itself excluding fault draws from the walk section (a small eval_amp_m5 code change: force dr.fault_prob=0 for the walk section specifically, matching how the yaw section already zeros both hazards), which is now the concretely scoped next step rather than more dose arms.

