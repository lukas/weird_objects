# cw-dynrep-tf-state2-fresh

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DEAD

**created**: 2026-08-15T18:36:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000

**wandb_id**: wttfxanc

**hypothesis**: The full 13.62M-parameter causal Transformer will generalize current and future servo state, velocity, heading, current, and foot contact when its optimizer budget is supplied by at least 10.24M distinct windows from fresh GPU MJX/Warp trajectories instead of replaying a tiny frozen corpus; an EMA target encoder will stabilize long-horizon latent supervision.

**gate**: Stage 1 must run on the H200, preserve the intended five-actor and four-level DR recipe, log data/train_windows >= 10240000 and data/planned_window_reuse <= 2.0 in W&B. Stage 2 must retain hidden=512,z=256,4 layers,8 heads,ff=1024, run CUDA/GPU-resident batches, and log train/val physical plus contact Brier/ECE metrics; validation must not show the immediate broad divergence seen in telnzd5r.

**verdict**: Stage-1 fresh-data collection OOM-killed pod hexapod-mjx-train-1 at 18:52 UTC (96Gi cgroup limit, exit 137) ~16 min in, at 1.18M of 10.24M windows; W&B wttfxanc crashed, overlay fs + shards lost. Same OOM class as train-10 08-14/15. Confound: cw-mt-b1-dualgru1 eval co-resident on the pod. Retry-once on clean train-10 with pod_memwatch.sh as cw-dynrep-tf-state2-fresh2.

