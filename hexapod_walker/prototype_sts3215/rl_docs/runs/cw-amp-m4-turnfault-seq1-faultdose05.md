# cw-amp-m4-turnfault-seq1-faultdose05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T05:07:31+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: 190ylmgf

**hypothesis**: Plain English: same fault-probability dose test as faultdose025, at the middle of the range -- see faultdose025 for the full design-tension rationale (q_20260823T0130Z). Single lever vs pushcont1: dr.fault_prob 1.0 -> 0.5, dr.ext_push_prob left at 1.0. Re-inits from the same pre-cheat turnfault-seq1 checkpoint.

**gate**: Same gate as faultdose025: own-cfg DR-0 floor gait_valid>=9/12; eval_amp_m5 walk-section terms/gait_valid vs pushcont1's 4 terms/10 gait_valid; secondary eval_yaw tip-err read.

**verdict**: Same design-tension test at dose=0.5 (see faultdose025 for rationale). Walk-section: terms=0 (clears the strict terms bar!) but gait_valid only 10/12 (2 episodes non-gait-valid via sacrificed legs [0,5], not falls) -- opposite trade from faultdose025 (which had gv=12/12 but terms=2), same overall ballpark as pushcont1's baseline, no dose corresponds to a monotonic single best point at n=12. Secondary: eval_yaw tip-left/right 0.2757/0.2871, same cluster, fault dose confirmed not a turn-tracking lever.

