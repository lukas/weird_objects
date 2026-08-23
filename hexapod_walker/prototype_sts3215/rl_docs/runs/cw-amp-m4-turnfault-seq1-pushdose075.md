# cw-amp-m4-turnfault-seq1-pushdose075

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T04:39:33+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: c75q3l0u

**hypothesis**: Plain English: same push-dose-sensitivity test as pushdose025/05, at the high end short of pushcont1's full dose. Single lever vs pushcont1: dr.ext_push_prob 1.0 -> 0.75. Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint, per the init-basin rule.

**gate**: Same gate as pushdose025/05: own-cfg DR-0 floor gait_valid>=9/12; eval_yaw tip-left/right err PASS-clean<=0.20-0.25, PARTIAL between that and 0.27/0.30, FLAT ~unchanged. eval_amp_m5 push section at eval-time ext_push_prob=1.0 confirms push-recovery survives the lower training dose.

**verdict**: Part of the 3-arm push-probability dose sweep (see pushdose025's verdict for the joint finding). This arm (dose=0.75): eval_yaw tip-left/right err 0.2610/0.2549 -- same 0.24-0.27 band as dose=0.25 (0.264/0.269) and dose=0.5 (0.239/0.257), all clustered regardless of exposure fraction and all still >0.20 (m5 bar) though modestly better than pushcont1's full-dose 0.27/0.30. Mechanism-safety floor clears (gait_valid 12/12, 2 terms, 0 sacrificed legs -- the cleanest safety read of the three doses), video clean six-leg cycling. Confirms FLAT: dose is not the lever, push's mere presence in training (not cumulative exposure) drives the erosion.

