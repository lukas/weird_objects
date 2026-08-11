# cw-arch-gru-bc-ft2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T22:30:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-gru-bc-ft1

**wandb_id**: cgrwl539

**hardware_ready**: False

**hypothesis**: Teach the walk-champion+stand-champion GRU (already BC-distilled to do both) to keep BOTH skills while RL polishes the gait; ft1 (default lr 3e-4, target-kl 0.02, 10M steps) proved the walk-heavy finetune overwrites the imitation-learned stance (rise 0/6, hold 0/6) while walk stays clean. This arm tests whether a 3x smaller learning rate + tighter KL trust region lets PPO nudge the walk without erasing rise/hold, on a short 2M discovery budget before committing more steps.

**gate**: PASS (discovery, binary): rise det >=3/6 (>=1 crouch) AND hold det >=4/6 (some stance retention) AND walk stays cheat-free (det gait_valid >=5/6, no parked-leg fingerprint) even if walk is rougher than ft1's. FAIL: rise/hold still 0-1/6 (lr/KL doesn't protect stance either) -> stance retention needs explicit protection (e.g. an auxiliary BC-anchor loss during the finetune, or freezing early layers), not further lr/KL sweeps.

**verdict**: FAIL exactly as pre-registered: at the DR0 gate, det rise 0/6 (bridge/crouch/flat all 0) and det hold 0/6 (all 6 episodes fail valid_plant on height+feet_down+footprint, worst foot 33mm off ground) — the 3x-lower LR + tighter target-KL did NOT protect stance any better than ft1's default hyperparameters. Walk stayed honest and cheat-free (gait_valid 6/6 det+sto, no sacrificed leg, slip/m 1.3-1.7 in-band). No exploit in video: rise is an honest stalled climb (plant_margin 122mm short, balanced duty, not a flag-leg), hold is an honest precision loss (feet 19-33mm proud, balanced duty), matching ft1's diagnosis (rise=data poverty, hold=real erosion) — hyperparameters do not touch either mechanism. Own-DR0.5 pass incidentally clears hold>=4/6 det (4/6) but that is NOT the gate (DR0) and is noise/inconsistent with the DR0 pass, not a promotion case.

