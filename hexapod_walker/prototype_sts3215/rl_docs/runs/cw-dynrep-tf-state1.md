# cw-dynrep-tf-state1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T17:30:56+00:00

**pod**: hexapod-mjx-train-10

**steps**: 40000

**hypothesis**: A CUDA causal Transformer trained on hardware-available proprio history and future actions can learn a task-independent state representation that accurately infers current velocity, heading, foot contact, and motor current while predicting future servo state, contact, velocity, and heading.

**gate**: W&B must show arch=transformer, CUDA H200, full_priv_fraction=1.0, and advancing global_step; held-out physical metrics must be logged for current vxy/heading/contact F1/current A and h1/h2/h5 servo/contact/privileged futures; final G1 requires beating persistence and matched linear baselines before PPO wiring.

**refused_reason**: hexapod-mjx-train-10 code marker e0520df1d5f6b9878df0e68e7345827de3472283 != local HEAD 4c723840ae847a63c4b579da7cf7055c70aefab6. Sync first: snapshot.sh --sync hexapod-mjx-train-10 (and snapshot/commit before that if the tree is dirty).

