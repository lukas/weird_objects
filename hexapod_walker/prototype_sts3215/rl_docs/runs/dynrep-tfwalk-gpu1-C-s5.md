# dynrep-tfwalk-gpu1-C-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T22:53:42+00:00

**pod**: hexapod-mjx-train-11

**steps**: 1000000

**parent**: cw-dynrep-tf-state2-recovered1

**git_sha**: 745816a059ddfbf62d5b3bae2aec3b192089b426

**wandb_id**: dx4yw04i

**hypothesis**: Anchored arm of the operator-ordered Transformer walking transfer test: the same cw-dynrep-tf-state2-recovered1 encoder fine-tunes at 0.1x LR while its self-supervised future-state objective continues as 4x256 anchor batches per rollout on the ORIGINAL recovered v5_mjx_fresh corpus; tests whether continued dynamics prediction keeps the representation honest while PPO learns walking, vs frozen (B) and scratch (A). GPU-only relaunch (CUDA-required trainer c4f5b211, anchor tensors on CUDA).

**gate**: Judged as a matched triple with dynrep-tfwalk-gpu1-A-s5/B-s5 at 1M steps: walk return/steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) vs A and B; anchor_loss must stay finite/stable.

**note**: Script-owned cohort (pod_tfwalk.sh); only arm needing the 8.3G recovered v5_mjx_fresh corpus on-pod (train-11, where it lives). CORRECTED GPU-only relaunch per operator order 20260815T224355Z: anchor tensors now built on CUDA (anchor_batch_to_torch device=cuda, c4f5b211); '[device] CUDA required and active' before W&B init. Supersedes both dynrep-tfwalk-C-s5 attempts (incl. 9e4eimd8, CPU, non-evidence).

