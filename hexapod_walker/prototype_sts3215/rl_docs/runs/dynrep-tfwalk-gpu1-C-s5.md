# dynrep-tfwalk-gpu1-C-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-15T22:53:42+00:00

**pod**: hexapod-mjx-train-11

**steps**: 1000000

**parent**: cw-dynrep-tf-state2-recovered1

**git_sha**: 745816a059ddfbf62d5b3bae2aec3b192089b426

**wandb_id**: dx4yw04i

**hypothesis**: Anchored arm of the operator-ordered Transformer walking transfer test: the same cw-dynrep-tf-state2-recovered1 encoder fine-tunes at 0.1x LR while its self-supervised future-state objective continues as 4x256 anchor batches per rollout on the ORIGINAL recovered v5_mjx_fresh corpus; tests whether continued dynamics prediction keeps the representation honest while PPO learns walking, vs frozen (B) and scratch (A). GPU-only relaunch (CUDA-required trainer c4f5b211, anchor tensors on CUDA).

**gate**: Judged as a matched triple with dynrep-tfwalk-gpu1-A-s5/B-s5 at 1M steps: walk return/steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) vs A and B; anchor_loss must stay finite/stable.

**note**: Trainer exited cleanly at its 1M-step budget (POD_TFWALK_DONE in pod log, checkpoint ppo_dynrep-tfwalk-gpu1-C-s5.zip on pod); ledger row was stale-RUNNING. Corrected to FINISHED per operator order 20260816T042655Z. Learning-rate question (is C still learning at 1M?) superseded by the 2M dynrep-tfwalk-metrics1 cohort.

