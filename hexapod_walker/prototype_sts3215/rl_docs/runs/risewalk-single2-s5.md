# risewalk-single2-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T22:32:46+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4500000

**parent**: none

**wandb_id**: 6thb0nwb

**hardware_ready**: False

**hypothesis**: GRU-encoder rise-to-walk A/B/C benchmark, seed 5 (operator emergency order 08-15 16:06, relaunched one-seed-per-pod after the train-10 OOM): does the dyn_scale_M_h16_large GRU dynamics encoder (frozen B / anchored C) retain rise competence while PPO learns walking, vs scratch A? 6-phase chain rise AxBxC then walk AxBxC on one pod.

**gate**: Per pod_risewalk.sh pre-registration: A loses hard-start rise (rise/return + rise/dh_m collapse) during walk training while C retains it and matches walk; quality columns decide if the walk is cleaner.

**note**: Script-owned cohort (pod_risewalk.sh, COHORT_NAME=risewalk-single2, manifest on-pod); registered post-hoc per operator order 20260815T221231Z (register all live runs) -- was launched by the 18:1x repair cycle outside the launcher; wandb_id is the CURRENT phase run (rw_rise_C_s5 6thb0nwb), each of the 6 phases gets its own W&B run; verified live via check_cohort this cycle.

