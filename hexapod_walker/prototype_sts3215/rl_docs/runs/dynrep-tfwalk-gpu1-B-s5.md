# dynrep-tfwalk-gpu1-B-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T22:53:23+00:00

**pod**: hexapod-mjx-train-7

**steps**: 1000000

**parent**: cw-dynrep-tf-state2-recovered1

**git_sha**: dfe6e78e9bd670e404fd6c51136d9a30f99d5c84

**wandb_id**: dg5oj5hs

**hypothesis**: Frozen-encoder arm of the operator-ordered Transformer walking transfer test: the first G1/G1.1+G3-passing Transformer dynrep encoder (cw-dynrep-tf-state2-recovered1) is frozen and PPO learns commanded-velocity walking through its latent z; tests whether the pretrained body-dynamics representation alone speeds/cleans learning vs scratch (A). GPU-only relaunch (CUDA-required trainer c4f5b211).

**gate**: Judged as a matched triple with dynrep-tfwalk-gpu1-A-s5/C-s5 at 1M steps: walk return/steps-to-threshold and gait-quality columns (slip_m, peak_roll_deg, slew_sat) vs A and C.

**note**: Script-owned cohort (pod_tfwalk.sh). CORRECTED GPU-only relaunch per operator order 20260815T224355Z; exact cw-dynrep-tf-state2-recovered1 frozen Transformer encoder, no substitution; G1/G1.1 PASS record verified readable on-pod (no gate rerun per order); '[device] CUDA required and active' before W&B init. Supersedes dynrep-tfwalk-B-s5 (f086dlfd, CPU, non-evidence — its stale trainer was still alive on train-7 and was killed this cycle before relaunch).

