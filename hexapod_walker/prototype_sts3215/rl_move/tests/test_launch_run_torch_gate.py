"""launch_run.py's --device cuda capability gate (08-15).

Closes the [code] WAITING-ON entry (STATUS.md / arch/STATUS.md): the
launcher must refuse an explicit `--device cuda` launch on a GPU pod
with no recorded pod_torch_capability.py capability, so a transformer/
attention-trunk arm can never silently fall back to the ~18x-slower
stock CPU-torch PPO update on GPU-priced hardware. `--device auto` (the
trainer default) is untouched — this only gates an EXPLICIT cuda ask.

These tests exercise `_launch_locked` with every external call
(kubectl, W&B, the ledger file) monkeypatched to canned/no-op values,
so nothing here touches a real pod or experiments.json.
"""
import argparse
import importlib.util
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parents[1] / "orchestrator"
sys.path.insert(0, str(_HERE))

_spec = importlib.util.spec_from_file_location("launch_run", _HERE / "launch_run.py")
lr = importlib.util.module_from_spec(_spec)
sys.modules["launch_run"] = lr
_spec.loader.exec_module(lr)


def _ns(**kw):
    base = dict(run="cw-arch-tf-gatecheck", pod="hexapod-mjx-train-1",
                steps=2_000_000, smoke=False, hypothesis="h", gate="g",
                parent=None, allow_slow=False, dry_run=True,
                phase="discovery", evidence=None, trainer="ppo",
                track="arch", operator_override=None)
    base.update(kw)
    return argparse.Namespace(**base)


def _patch_common(monkeypatch):
    # No live kubectl/W&B/ledger writes from this test, ever.
    local_sha = lr.sh(["git", "-C", str(lr.HERE), "rev-parse", "HEAD"]).strip()

    def _kexec(pod, script, timeout=60):
        if "code_sha" in script:
            return local_sha
        if "nvidia-smi" in script:
            return "GPU 0: NVIDIA H200\n"
        if "wandb.env" in script:
            return "OK"
        return ""

    monkeypatch.setattr(lr, "pod_cpu_limit", lambda pod: 56)
    monkeypatch.setattr(lr, "pod_trainers", lambda pod: [])
    monkeypatch.setattr(lr, "kexec", _kexec)
    monkeypatch.setattr(lr, "wandb_name_exists", lambda run: False)
    monkeypatch.setattr(lr, "wandb_running_runs", lambda: {})
    monkeypatch.setattr(lr, "upsert_entry", lambda entry: None)

    class _NeverExists:
        def exists(self):
            return False

    monkeypatch.setattr(lr, "LAUNCH_HOLD", _NeverExists())


def _guardrails():
    return {"compute": {
        "pods": ["hexapod-sweep-friction"],
        "gpu_pods": ["hexapod-mjx-train-1"],
        "gpu": {"n_envs": 4096, "min_eval_every": 1_000_000,
                "min_video_every": 2_000_000,
                "max_steps_per_run": 40_000_000, "host_workers": 24,
                "impl": "warp"},
        "n_envs": 256, "min_eval_every": 100_000, "min_video_every": 500_000,
        "max_steps_per_run": 6_000_000,
        "max_concurrent_runs": 16,
        "phases": {"discovery_max_steps": 2_000_000},
    }}


def test_explicit_device_cuda_refused_on_uncapable_pod(monkeypatch):
    _patch_common(monkeypatch)
    monkeypatch.setattr(lr._torch_cap, "is_capable", lambda pod: False)
    a = _ns()
    rc = lr._launch_locked(_guardrails(), a, ["--device", "cuda"])
    assert rc == 1


def test_explicit_device_cuda_passes_on_recorded_pod(monkeypatch):
    _patch_common(monkeypatch)
    monkeypatch.setattr(lr._torch_cap, "is_capable", lambda pod: True)
    a = _ns()
    rc = lr._launch_locked(_guardrails(), a, ["--device", "cuda"])
    # dry_run=True -> a clean pass returns 0 (DRY-RUN print), not a refusal
    assert rc == 0


def test_device_auto_default_is_never_gated(monkeypatch):
    _patch_common(monkeypatch)
    # is_capable would refuse if consulted; it must NOT be consulted for
    # the trainer's own "auto" default (no explicit cuda ask at all).
    monkeypatch.setattr(lr._torch_cap, "is_capable", lambda pod: False)
    a = _ns()
    rc = lr._launch_locked(_guardrails(), a, [])
    assert rc == 0


def test_dynrep_launches_are_exempt_from_this_gate(monkeypatch):
    # dynrep already runs its own stricter live cuda_torch_runtime probe
    # (kexec'd against GPU_TORCH_PYTHON) a few lines below this gate;
    # this gate must not double-refuse it, and must not even consult
    # is_capable() for a dynrep launch.
    _patch_common(monkeypatch)
    calls = []
    monkeypatch.setattr(lr._torch_cap, "is_capable",
                        lambda pod: calls.append(pod) or False)
    a = _ns(run="cw-dynrep-gatecheck", trainer="dynrep")
    rc = lr._launch_locked(
        _guardrails(), a,
        ["--device", "cuda", "--data", "ds1", "--arch", "transformer"])
    assert calls == []
    assert rc == 0
