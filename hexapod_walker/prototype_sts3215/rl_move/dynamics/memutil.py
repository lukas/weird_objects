"""Cheap, dependency-free host-memory readings for pipeline forensics.

Born from the 08-15 ~19:47 UTC dynrep incident: `cw-dynrep-tf-state2-
fresh2`'s pod hard-OOMKilled (cgroup 96Gi limit) ~65s after its stage-1
GPU data collection logged `data/complete=1` and cleanly exited -- and
because pods mount no persistent volume, the OOM took the stdout log
file with it, leaving NOTHING to diagnose from after the fact (the W&B
run for stage 2 never even got created, since `train.py` calls
`_init_wandb` only after `load_dataset`/`compute_stats`/sampler
construction/model build). These helpers let both stages log where
memory actually goes, straight to W&B (durable, off-pod) instead of a
log file the next OOM can destroy.
"""
from __future__ import annotations


def host_mem_gib() -> dict:
    """Best-effort host memory reading; never raises.

    Returns GiB floats: ``rss`` (this process' resident set from
    /proc/self/status), ``cgroup_current``/``cgroup_max`` (the
    container's cgroup v2 memory accounting, if present -- this is
    what actually triggers a pod-level OOM kill, not any one
    process' RSS), or ``{}`` on any platform where these aren't
    available (e.g. local dev, non-Linux).
    """
    out: dict = {}
    try:
        with open("/proc/self/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    kib = int(line.split()[1])
                    out["rss_gib"] = kib / (1024.0 * 1024.0)
                    break
    except OSError:
        pass
    try:
        with open("/sys/fs/cgroup/memory.current") as fh:
            out["cgroup_current_gib"] = int(fh.read().strip()) / (1024.0 ** 3)
    except (OSError, ValueError):
        pass
    try:
        with open("/sys/fs/cgroup/memory.max") as fh:
            raw = fh.read().strip()
            if raw != "max":
                out["cgroup_max_gib"] = int(raw) / (1024.0 ** 3)
    except (OSError, ValueError):
        pass
    return out


def mem_checkpoint(tag: str, *, print_it: bool = True) -> dict:
    """Read memory now and return a flat dict of ``mem/<tag>_*`` fields
    suitable for merging straight into a ``wandb.log`` / ``run.log``
    payload; also prints a one-line summary (visible in the pod's own
    stdout log while it's still alive)."""
    m = host_mem_gib()
    payload = {f"mem/{tag}_{k}": v for k, v in m.items()}
    if print_it:
        bits = " ".join(f"{k}={v:.2f}GiB" for k, v in m.items())
        print(f"[mem] {tag}: {bits or '(unavailable)'}", flush=True)
    return payload
