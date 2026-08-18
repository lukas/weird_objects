# cw-dynrep-tf-state2-recovered1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-15T20:12:56+00:00

**pod**: hexapod-mjx-train-11

**steps**: 40000

**parent**: cw-dynrep-tf-state2-fresh2

**wandb_id**: vt2ovznc

**hardware_ready**: False

**hypothesis**: Trainer-only recovery: the exact completed flaf42k7 corpus (10,240,039 train windows, <=2.0x reuse, gate already met) can train the unchanged 13.62M-param causal Transformer without any recollection now that each compressed NPZ shard member is loaded once per shard instead of once per episode (commit 3cd6c57a fixed the bug that OOMKilled cw-dynrep-tf-state2-fresh2's pod loading this same corpus). Launched directly against rl_move.dynamics.train (skipping fresh_pipeline's redundant stage-1 recollection) once the corpus was confirmed to already exist and be complete.

**gate**: Trainer-only on the H200; recovered dataset aggregate SHA-256 6762fe81a0694b647086c5f454070ec208c059c7e8efbd1dcb597b76f1c70cf0; 10,240,039 train windows and <=2.0x reuse; 13.62M-parameter Transformer (hidden=512,z=256,4 layers,8 heads,ff=1024); CUDA GPU-resident batches; loader stays below the 85GiB watchdog; W&B advances and reports train/validation velocity, heading, contacts, currents, future servo state, generalization gap; no immediate broad train/val divergence like telnzd5r; one-time sealed test evaluation at the end.

**verdict**: PASS -- fresh 10.24M-window GPU corpus fixes the Transformer's overfitting AND the encoder clears every binding gate for the first time. Trainer-only retrain on the recovered flaf42k7 corpus (no recollection needed, loader-bug fixed) ran its full 40k-step budget with NO divergence: train/val/test total 2.08/2.24/2.26 (val/train ratio 1.03, generalization/overfit_alarm never fired) -- the opposite of predecessor telnzd5r, which the operator killed at 21k/40k when held-out total blew up 5.53->8.22 while train collapsed <0.75. Ran the two BINDING gates this cycle (were not part of the training run itself): GATE G1/G1.1 (rl_move.dynamics.eval_model --split test) PASS outright at every horizon -- model beats both persistence AND matched linear ridge at k=1/2/5 (e.g. k=1 MSE 0.079 vs ridge 0.139, persistence 0.231) and beats unchanged-latent at k=10/25, no G1.1 tolerance even needed. GATE G3 (probe_latents.py) PASS -- linear ridge probes from z recover roll/pitch/gyro (R2 0.90-0.99), body velocity (R2 0.90-0.95), chassis height (R2 0.98) and per-foot contact (balanced-acc 0.97-0.98) on held-out windows, all far above the shuffled-label chance floor (~0 or negative); one real gap noted honestly: cos_yaw_rel R2 is -0.04 (degenerate, sin_yaw_rel only 0.72) -- command-frame yaw phase is not cleanly encoded, everything else is. This is the first TRANSFORMER-architecture encoder in the dynrep line to clear G1/G1.1/G3 (prior passes were GRU scale_S/M). Evidence: logs/ckpt_eval/cw_dynrep_tf_state2_recovered1/{eval_g1_test,eval_g1_g3dump_test}.json + g3_probe.txt. Does NOT itself change the product baseline (representation-learning line, no policy shipped); the open next step is wiring this specific encoder into a condition A/B/C PPO transfer comparison (not yet launched -- not pre-registered, and 6 GPU pods are already mid-cohort on the GRU-encoder A/B/C comparison) -- named as a Next candidate in dynrep/STATUS.md, not launched this cycle.

**note**: registered post-hoc by a concurrent orchestrator cycle (this one) -- launched directly by a parallel session/operator action outside launch_run.py, not through the standard launch/respec path; ledger entry added for watcher tracking (pre-staged evals, checkups) since none existed.

