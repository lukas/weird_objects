# dynrep-tfwalk-A-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: ABORTED

**created**: 2026-08-15T22:31:29+00:00

**pod**: hexapod-mjx-train-8

**steps**: 1000000

**parent**: none

**wandb_id**: 11zsrpl9

**hardware_ready**: False

**hypothesis**: Baseline arm of the operator-ordered (20260815T221231Z) Transformer-encoder walking/heading transfer test: PPO learns commanded-velocity walking from scratch on raw stacked observations, giving the matched no-encoder reference for conditions B/C. Matched to the GRU futurewalk benchmark: task walk, 1M steps, seed 5, dr 0.3, eval rise/hold/walk + held-out dynamics suites.

**gate**: Judged as a matched triple with dynrep-tfwalk-B-s5/C-s5 at 1M steps: walk return/steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) vs B and C; A is the null hypothesis (representation gives no sample-efficiency win).

**verdict**: ABORTED/NON-EVIDENCE, class stopped by the C-s5 finding (operator fb_20260815T222316_26b670): the same train_ppo_transfer build hard-coded device=cpu for all conditions, so this A attempt (W&B 11zsrpl9) is CPU-compromised like C. No behavioral verdict taken; superseded by dynrep-tfwalk-gpu1-A-s5 (CUDA-required trainer c4f5b211, corrected order 20260815T224355Z).

**note**: Script-owned cohort (pod_tfwalk.sh, manifest tfwalk_manifest.jsonl on-pod); registered at launch per operator order; W&B attempt name dynrep-tfwalk-A-s5.0815-2221Z.

