# dynrep-tfwalk-gpu1-A-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T22:53:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 1000000

**git_sha**: 745816a059ddfbf62d5b3bae2aec3b192089b426

**wandb_id**: h9yy9fll

**hypothesis**: Scratch-baseline arm of the operator-ordered Transformer walking transfer test: PPO learns commanded-velocity walking from raw stacked obs with no pretrained encoder — the null hypothesis that the dynrep representation gives no sample-efficiency or gait-quality win. GPU-only relaunch (CUDA-required trainer c4f5b211) of the aborted CPU cohort.

**gate**: Judged as a matched triple with dynrep-tfwalk-gpu1-B-s5/C-s5 at 1M steps: walk return/steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) vs B and C.

**note**: Script-owned cohort (pod_tfwalk.sh, manifest tfwalk-gpu1_manifest.jsonl on-pod). CORRECTED GPU-only relaunch per operator order 20260815T224355Z after the CPU compliance failure (fb_20260815T222316_26b670); log shows '[device] CUDA required and active' before W&B init; supersedes dynrep-tfwalk-A-s5 (11zsrpl9, non-evidence).

