# dynrep-tfwalk-C-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: ABORTED

**created**: 2026-08-15T22:32:00+00:00

**pod**: hexapod-mjx-train-11

**steps**: 1000000

**parent**: cw-dynrep-tf-state2-recovered1

**wandb_id**: 9e4eimd8

**hardware_ready**: False

**hypothesis**: Anchored arm of the operator-ordered (20260815T221231Z) Transformer walking/heading transfer test: the G1/G1.1+G3-passing Transformer encoder (cw-dynrep-tf-state2-recovered1) fine-tunes at 0.1x LR while its self-supervised future-state prediction objective continues as 4x256 anchor batches per rollout on the ORIGINAL recovered v5_mjx_fresh corpus (no recollection, aggregate sha 6762fe81...); tests whether continued dynamics prediction keeps the representation honest while PPO learns walking, vs frozen (B) and scratch (A).

**gate**: Judged as a matched triple with A/B at 1M steps: beats A on walk steps-to-threshold/final return outside eval noise AND anchor/loss stays near the 2.2-ish pretraining val total (a blowup = the anchor is fighting PPO); gait-quality columns no worse than A.

**verdict**: ABORTED/NON-EVIDENCE (operator fb_20260815T222316_26b670 + corrected order 20260815T224355Z): both C attempts (incl. W&B 9e4eimd8) ran train_ppo_transfer with device hard-coded to CPU (PPO common, checkpoint load, anchor tensors) — GPU compliance failure; Codex stopped PIDs 2841874/2841719. Not retried under this name; superseded by dynrep-tfwalk-gpu1-C-s5 on the CUDA-required trainer (c4f5b211).

**note**: Script-owned cohort (pod_tfwalk.sh); only arm needing the 8.3G v5_mjx_fresh corpus on-pod (train-11, where it already lives); memwatch live; anchor-batch converter fix test-covered (test_dynrep_ppo_anchor.py green this cycle); W&B attempt name dynrep-tfwalk-C-s5.0815-2221Z.

