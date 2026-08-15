# cw-dynrep-tf-state2-fresh2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T19:01:03+00:00

**pod**: hexapod-mjx-train-10

**steps**: 40000

**wandb_id**: flaf42k7

**hypothesis**: Retry of cw-dynrep-tf-state2-fresh after its stage-1 collection OOM-killed pod train-1 (96Gi limit, 18:52 UTC, at 1.18M/10.24M windows, with a heavyweight eval co-resident). Plain English: give the full 13.62M-parameter causal Transformer enough genuinely fresh GPU-simulated data (>=10.24M distinct windows, reuse <=2x) that it stops overfitting the way the frozen-corpus run (telnzd5r) did; this retry runs the identical recipe on a CLEAN pod with a memory watchdog, so a second OOM would name the leaking process instead of killing the pod.

**gate**: Stage 1 must run on the H200, preserve the five-actor and four-level DR recipe, log data/train_windows >= 10240000 and data/planned_window_reuse <= 2.0 in W&B, and must pass the previous death point (1.18M windows) without the pod's memory.current approaching 96Gi. Stage 2 must retain hidden=512,z=256,4 layers,8 heads,ff=1024, run CUDA/GPU-resident batches, and log train/val physical plus contact Brier/ECE metrics; validation must not show the immediate broad divergence seen in telnzd5r.

