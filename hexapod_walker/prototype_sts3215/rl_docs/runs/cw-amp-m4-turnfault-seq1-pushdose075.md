# cw-amp-m4-turnfault-seq1-pushdose075

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T04:39:33+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: c75q3l0u

**hypothesis**: Plain English: same push-dose-sensitivity test as pushdose025/05, at the high end short of pushcont1's full dose. Single lever vs pushcont1: dr.ext_push_prob 1.0 -> 0.75. Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint, per the init-basin rule.

**gate**: Same gate as pushdose025/05: own-cfg DR-0 floor gait_valid>=9/12; eval_yaw tip-left/right err PASS-clean<=0.20-0.25, PARTIAL between that and 0.27/0.30, FLAT ~unchanged. eval_amp_m5 push section at eval-time ext_push_prob=1.0 confirms push-recovery survives the lower training dose.

