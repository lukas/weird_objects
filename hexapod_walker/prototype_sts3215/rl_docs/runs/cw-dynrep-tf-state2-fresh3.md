# cw-dynrep-tf-state2-fresh3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T20:03:43+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000

**parent**: cw-dynrep-tf-state2-fresh2

**hypothesis**: Give the 13.62M-param causal Transformer its full >=10.24M-window fresh-GPU-data budget (identical recipe to fresh/fresh2) now that the actual OOM cause is fixed: load_dataset() was re-allocating each shard's full frames/actions/priv arrays on every episode (NpzFile.__getitem__ has no caching), exhausting a 96Gi pod loading an ~8-9GiB corpus (commit 3cd6c57a caches each member once per shard). fresh2's stage 1 already proved the collection recipe hits the window/reuse gate cleanly (flaf42k7: 10,240,039 train windows, reuse 1.9999x) and died only in the now-patched stage-2 load; this is a clean-pod retry of the SAME hypothesis with the load bug fixed, plus new mem/* W&B telemetry (this cycle) at each stage-1/stage-2 checkpoint in case anything still spikes.

**gate**: Stage 1: data/train_windows >= 10240000 and data/planned_window_reuse <= 2.0 in W&B (same as fresh2, already proven). Stage 2 must actually START (its own W&B run must exist, unlike fresh2) and survive load_dataset/compute_stats/sampler-build without the pod's cgroup memory.current exceeding ~50GiB (visible via the new mem/* fields), then train to convergence with val/train-eval gap behaved (no immediate broad divergence like telnzd5r) and log contact Brier/ECE.

**refused_reason**: hexapod-mjx-train-3 code marker 3d9f0113a477b86f0d0cc36430e2fc012c3f279b != local HEAD 909f002de5562c9ee251973eca001a1961558c37. Sync first: snapshot.sh --sync hexapod-mjx-train-3 (and snapshot/commit before that if the tree is dirty).

