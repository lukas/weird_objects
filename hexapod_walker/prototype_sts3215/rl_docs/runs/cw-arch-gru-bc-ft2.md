# cw-arch-gru-bc-ft2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T22:30:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-gru-bc-ft1

**wandb_id**: cgrwl539

**hypothesis**: Teach the walk-champion+stand-champion GRU (already BC-distilled to do both) to keep BOTH skills while RL polishes the gait; ft1 (default lr 3e-4, target-kl 0.02, 10M steps) proved the walk-heavy finetune overwrites the imitation-learned stance (rise 0/6, hold 0/6) while walk stays clean. This arm tests whether a 3x smaller learning rate + tighter KL trust region lets PPO nudge the walk without erasing rise/hold, on a short 2M discovery budget before committing more steps.

**gate**: PASS (discovery, binary): rise det >=3/6 (>=1 crouch) AND hold det >=4/6 (some stance retention) AND walk stays cheat-free (det gait_valid >=5/6, no parked-leg fingerprint) even if walk is rougher than ft1's. FAIL: rise/hold still 0-1/6 (lr/KL doesn't protect stance either) -> stance retention needs explicit protection (e.g. an auxiliary BC-anchor loss during the finetune, or freezing early layers), not further lr/KL sweeps.

