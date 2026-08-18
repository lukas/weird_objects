# cw-dynrep-tf-state2-fresh2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-15T19:01:03+00:00

**pod**: hexapod-mjx-train-10

**steps**: 40000

**wandb_id**: flaf42k7

**hardware_ready**: False

**hypothesis**: Retry of cw-dynrep-tf-state2-fresh after its stage-1 collection OOM-killed pod train-1 (96Gi limit, 18:52 UTC, at 1.18M/10.24M windows, with a heavyweight eval co-resident). Plain English: give the full 13.62M-parameter causal Transformer enough genuinely fresh GPU-simulated data (>=10.24M distinct windows, reuse <=2x) that it stops overfitting the way the frozen-corpus run (telnzd5r) did; this retry runs the identical recipe on a CLEAN pod with a memory watchdog, so a second OOM would name the leaking process instead of killing the pod.

**gate**: Stage 1 must run on the H200, preserve the five-actor and four-level DR recipe, log data/train_windows >= 10240000 and data/planned_window_reuse <= 2.0 in W&B, and must pass the previous death point (1.18M windows) without the pod's memory.current approaching 96Gi. Stage 2 must retain hidden=512,z=256,4 layers,8 heads,ff=1024, run CUDA/GPU-resident batches, and log train/val physical plus contact Brier/ECE metrics; validation must not show the immediate broad divergence seen in telnzd5r.

**verdict**: INFRA FAIL, root cause found + fixed: stage-1 GPU data collection completed cleanly and MET its gate (data/complete=1, train_windows=10,240,039 >= target, planned_reuse=1.9999x <= 2.0x, wandb flaf42k7 'cw-dynrep-tf-state2-fresh2-data') but the pod (train-10) hard-OOMKilled at the 96Gi cgroup limit 65s later, before stage 2 (Transformer training) ever called wandb.init -- so the failure left zero durable trace on its own. Root cause (found by a parallel investigation + fixed directly by the operator, commit 3cd6c57a 'Load each dynrep shard member once'): data.load_dataset indexed z["frames"]/z["actions"]/z["priv"] freshly INSIDE the per-episode loop -- NpzFile.__getitem__ allocates the complete member array on every call, so a ~2048-episode shard re-allocated its full frames/actions/priv arrays up to 2048 times each, exhausting a 96Gi pod while loading an ~8-9GiB corpus. This ALSO falsifies the prior 'stacking, not a collector leak' theory recorded for fresh2's own predecessor death (train-1, this same hypothesis's first attempt) -- fresh2 died SOLO on a clean pod with no co-resident job, so the real bug was always in load_dataset, not GPU-pod contention. Same cycle added durable forensics for next time regardless of root cause: rl_move/dynamics/memutil.py (host RSS + cgroup memory.current/max readers), wired into collect_mjx.py's periodic/final logs and moved train.py's wandb.init BEFORE the load/stats/sampler/model steps so a future crash in that stretch still leaves a W&B trail; pod_memwatch.sh poll tightened 60s->10s (a fast stage-transition spike could out-run the old interval). This is the SECOND death of the fresh-data-collection hypothesis (train-1 OOM -> retry fresh2 -> OOM again) but the cause is now understood and patched, not a repeat of the same unknown -- cw-dynrep-tf-state2-fresh3 relaunched with the fix on a clean pod.

