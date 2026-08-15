# cw-dynrep-tf-state1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T17:53:27+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000

**wandb_id**: telnzd5r

**hypothesis**: A CUDA causal Transformer trained on full-label proprioceptive trajectories can learn current and future servo state, body velocity, yaw/heading, and foot contact accurately enough to provide a reusable representation for later walking control while continued auxiliary sampling prevents degradation.

**gate**: W&B must show arch=transformer, device=cuda on an H200, full_priv_fraction=100%, advancing global_step, and interpretable validation metrics for velocity, heading, contacts, currents, and future servo state.

