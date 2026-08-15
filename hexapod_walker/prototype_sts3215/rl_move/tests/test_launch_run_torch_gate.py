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
from types import SimpleNamespace

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
        if "torch.cuda.is_available" in script:
            return "2.11.0+cu128 NVIDIA H200\n"
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
    return _kexec


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


def test_canary_and_acquisition_are_distinct_phase_contracts(monkeypatch):
    _patch_common(monkeypatch)
    monkeypatch.setattr(lr._torch_cap, "is_capable", lambda pod: True)

    assert lr._launch_locked(
        _guardrails(), _ns(phase="canary"), ["--device", "cuda"]) == 0
    assert lr._launch_locked(
        _guardrails(), _ns(phase="canary", steps=2_000_001),
        ["--device", "cuda"]) == 1
    assert lr._launch_locked(
        _guardrails(), _ns(phase="acquisition", steps=38_000_000,
                           evidence=None), ["--device", "cuda"]) == 1
    assert lr._launch_locked(
        _guardrails(), _ns(phase="acquisition", steps=38_000_000,
                           evidence="healthy 2M canary; matched family 40M"),
        ["--device", "cuda"]) == 0


def test_canary_verdict_guard_blocks_behavioral_category_error():
    base = {"phase": "canary", "hardware_ready": False}
    assert lr.canary_update_error({
        **base, "verdict": "CANARY PASS - optimizer and telemetry healthy"}) == ""
    assert lr.canary_update_error({
        **base, "verdict": "CANARY FAIL - MECHANISM - NaN at step 10"}) == ""
    assert "invalid" in lr.canary_update_error({
        **base, "verdict": "FAIL - KNOWN EXPLOIT; reward recipe closed"})
    assert "hardware_ready" in lr.canary_update_error({
        **base, "hardware_ready": True, "verdict": "CANARY PASS"})


def test_dynrep_uses_recorded_capability_and_live_runtime_probe(monkeypatch):
    # A recorded capability lets dynrep consider system python3 after the
    # private venv; the live CUDA probe still has to succeed before launch.
    _patch_common(monkeypatch)
    calls = []
    monkeypatch.setattr(lr._torch_cap, "is_capable",
                        lambda pod: calls.append(pod) or True)
    a = _ns(run="cw-dynrep-gatecheck", trainer="dynrep")
    rc = lr._launch_locked(
        _guardrails(), a,
        ["--device", "cuda", "--data", "ds1", "--arch", "transformer"])
    assert calls == ["hexapod-mjx-train-1"]
    assert rc == 0


def test_dynrep_launch_timeout_recovers_anchored_trainer_pid(monkeypatch):
    base_kexec = _patch_common(monkeypatch)
    monkeypatch.setattr(lr._torch_cap, "is_capable", lambda pod: True)

    def timeout_on_launch(pod, script, timeout=60):
        if "nohup" in script and " -m rl_move.dynamics.train" in script:
            raise lr.subprocess.TimeoutExpired("kubectl exec", timeout)
        return base_kexec(pod, script, timeout)

    recovered = []
    monkeypatch.setattr(lr, "kexec", timeout_on_launch)
    monkeypatch.setattr(
        lr, "_pod_trainer_pid",
        lambda pod, run: recovered.append((pod, run)) or "4184935")
    a = _ns(run="cw-dynrep-timeout", trainer="dynrep", dry_run=False)
    ctx = lr._launch_locked(
        _guardrails(), a,
        ["--device", "cuda", "--data", "ds1", "--arch", "transformer"])
    assert recovered == [("hexapod-mjx-train-1", "cw-dynrep-timeout")]
    assert ctx["pid"] == "4184935"


def test_dynrep_fresh_uses_gpu_data_pipeline(monkeypatch, capsys):
    _patch_common(monkeypatch)
    monkeypatch.setattr(lr._torch_cap, "is_capable", lambda pod: True)
    a = _ns(run="cw-dynrep-fresh-gatecheck", trainer="dynrep-fresh")
    rc = lr._launch_locked(
        _guardrails(), a,
        ["--device", "cuda", "--data", "fresh_ds",
         "--arch", "transformer", "--collect-n-envs", "2048"])
    command = capsys.readouterr().out
    assert rc == 0
    assert "-m rl_move.dynamics.fresh_pipeline" in command
    assert "--collect-n-envs 2048" in command
    assert "--arch transformer" in command


def test_respec_now_preserves_dynrep_fresh_trainer(monkeypatch):
    source = {
        "run": "cw-dynrep-source", "trainer": "dynrep-fresh",
        "steps": 40_000, "track": "dynrep", "phase": "discovery",
        "extra_args": ["--data", "old_ds", "--device", "cuda"],
        "checks": {"pid": "123"},
    }
    monkeypatch.setattr(lr, "load_ledger", lambda: [source])
    monkeypatch.setattr(
        lr.subprocess, "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0, stdout="snapshot ok", stderr=""))
    monkeypatch.setattr(lr, "_self_repair_pod", lambda pod, args: None)
    captured = {}

    def fake_launch(g, ns, args):
        captured.update(ns=ns, args=args)
        return 0

    monkeypatch.setattr(lr, "cmd_launch", fake_launch)
    args = argparse.Namespace(
        source="cw-dynrep-source", run="cw-dynrep-copy", seed=None,
        steps=None, parent="", hypothesis="h", gate="g", phase="",
        evidence="", arg=["--data=new_ds"], cfg=None,
        init_from_source=False, now=True, pod="hexapod-mjx-train-1",
        operator_override="", track="")
    assert lr.cmd_respec({}, args) == 0
    assert captured["ns"].trainer == "dynrep-fresh"
    assert captured["args"][captured["args"].index("--data") + 1] == "new_ds"
