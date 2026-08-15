# cw-dynrep-tf-state2-fresh3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-15T20:06:24+00:00

**pod**: hexapod-mjx-train-11

**steps**: 40000

**parent**: cw-dynrep-tf-state2-fresh2

**hardware_ready**: False

**hypothesis**: Give the 13.62M-param causal Transformer its full >=10.24M-window fresh-GPU-data budget (identical recipe to fresh/fresh2) now that the actual OOM cause is fixed: load_dataset() was re-allocating each shard's full frames/actions/priv arrays on every episode (NpzFile.__getitem__ has no caching), exhausting a 96Gi pod loading an ~8-9GiB corpus (commit 3cd6c57a caches each member once per shard). fresh2's stage 1 already proved the collection recipe hits the window/reuse gate cleanly (flaf42k7: 10,240,039 train windows, reuse 1.9999x) and died only in the now-patched stage-2 load; this is a clean-pod retry of the SAME hypothesis with the load bug fixed, plus new mem/* W&B telemetry (this cycle) at each stage-1/stage-2 checkpoint in case anything still spikes.

**gate**: Stage 1: data/train_windows >= 10240000 and data/planned_window_reuse <= 2.0 in W&B (same as fresh2, already proven). Stage 2 must actually START (its own W&B run must exist, unlike fresh2) and survive load_dataset/compute_stats/sampler-build without the pod's cgroup memory.current exceeding ~50GiB (visible via the new mem/* fields), then train to convergence with val/train-eval gap behaved (no immediate broad divergence like telnzd5r) and log contact Brier/ECE.

**verdict**: Crashed in ~1 min from a LAUNCHER BUG I found+fixed same cycle, not the dynrep hypothesis: respec's --out-name auto-append (mirrors the ppo-only convention in launch, previously unguarded in respec) is not a valid rl_move.dynamics.train flag, so fresh_pipeline's stage-2 os.execv into train.py died on argparse ('unrecognized arguments: --out-name ...') right after stage 1 instantly found the dataset already complete (10,240,039 train windows, reused not recollected -- the v5_mjx_fresh corpus survives on shared storage across the pod that died collecting it). Fixed launch_run.py respec to skip --out-name/--init-from-source for dynrep/dynrep-fresh sources (they have no such flags). Superseded: a concurrent operator/session launched cw-dynrep-tf-state2-recovered1 directly against rl_move.dynamics.train (bypassing recollection entirely, same corpus, same NPZ-cache fix) on the same pod right after this crashed -- that run is alive and is the correct continuation; registered it in the ledger this cycle since it had none.

