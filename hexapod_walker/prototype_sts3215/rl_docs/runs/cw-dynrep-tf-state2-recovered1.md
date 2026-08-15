# cw-dynrep-tf-state2-recovered1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T20:12:56+00:00

**pod**: hexapod-mjx-train-11

**steps**: 40000

**parent**: cw-dynrep-tf-state2-fresh2

**wandb_id**: vt2ovznc

**hypothesis**: Trainer-only recovery: the exact completed flaf42k7 corpus (10,240,039 train windows, <=2.0x reuse, gate already met) can train the unchanged 13.62M-param causal Transformer without any recollection now that each compressed NPZ shard member is loaded once per shard instead of once per episode (commit 3cd6c57a fixed the bug that OOMKilled cw-dynrep-tf-state2-fresh2's pod loading this same corpus). Launched directly against rl_move.dynamics.train (skipping fresh_pipeline's redundant stage-1 recollection) once the corpus was confirmed to already exist and be complete.

**gate**: Trainer-only on the H200; recovered dataset aggregate SHA-256 6762fe81a0694b647086c5f454070ec208c059c7e8efbd1dcb597b76f1c70cf0; 10,240,039 train windows and <=2.0x reuse; 13.62M-parameter Transformer (hidden=512,z=256,4 layers,8 heads,ff=1024); CUDA GPU-resident batches; loader stays below the 85GiB watchdog; W&B advances and reports train/validation velocity, heading, contacts, currents, future servo state, generalization gap; no immediate broad train/val divergence like telnzd5r; one-time sealed test evaluation at the end.

**note**: registered post-hoc by a concurrent orchestrator cycle (this one) -- launched directly by a parallel session/operator action outside launch_run.py, not through the standard launch/respec path; ledger entry added for watcher tracking (pre-staged evals, checkups) since none existed.

