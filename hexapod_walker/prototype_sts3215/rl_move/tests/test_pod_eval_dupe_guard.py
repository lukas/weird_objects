"""Duplicate-eval guard for pod_eval.py.

2026-08-27 (standwalk anchor14-rescue-acq8m idle-kick): pod_eval's old
idempotency check only asked "does the CONTROLLER-side artifact dir
already exist" — a pass still mid-flight on the pod (the normal state
for anything with per-mode video, easily 1-2h) was invisible to it, so
re-invoking `ops.sh podeval`/pod_eval.py on the same run silently
launched a SECOND eval_checkpoint tree on the same pod, racing the
first for CPU and for the same output-dir episode files. This test
pins `remote_eval_running`'s pure grep-matching logic (subprocess I/O
mocked — no real pod) so the guard can't silently regress.
"""
import importlib.util
import pathlib
import subprocess
import sys
import types

_ORCH = pathlib.Path(__file__).resolve().parents[1] / "orchestrator"
sys.path.insert(0, str(_ORCH))  # pod_eval imports its sibling ``tracks``
_P = _ORCH / "pod_eval.py"
_spec = importlib.util.spec_from_file_location("pod_eval", _P)
pod_eval = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pod_eval)


def _fake_kexec(returncode, stdout=""):
    def _f(pod, cmd, timeout=60):
        return types.SimpleNamespace(returncode=returncode, stdout=stdout,
                                      stderr="")
    return _f


def test_match_found_reports_running(monkeypatch):
    monkeypatch.setattr(pod_eval, "kexec", _fake_kexec(
        0, "uv run python -m rl_move.sim.eval_checkpoint ... --out logs/ckpt_eval/foo_gate\n"))
    assert pod_eval.remote_eval_running("some-pod", "logs/ckpt_eval/foo_gate") is True


def test_no_match_reports_not_running(monkeypatch):
    monkeypatch.setattr(pod_eval, "kexec", _fake_kexec(1, ""))
    assert pod_eval.remote_eval_running("some-pod", "logs/ckpt_eval/foo_gate") is False


def test_pod_timeout_fails_open_not_running(monkeypatch):
    # A slow/unreachable pod must not permanently block a genuinely
    # ready re-eval — fail open (treat as "not running") like the old
    # behavior, rather than wedging the caller.
    def _timeout(pod, cmd, timeout=60):
        raise subprocess.TimeoutExpired(cmd, timeout)
    monkeypatch.setattr(pod_eval, "kexec", _timeout)
    assert pod_eval.remote_eval_running("some-pod", "logs/ckpt_eval/foo_gate") is False


def test_module_arg_is_used_in_grep(monkeypatch):
    seen = {}

    def _f(pod, cmd, timeout=60):
        seen["cmd"] = cmd
        return types.SimpleNamespace(returncode=1, stdout="", stderr="")

    monkeypatch.setattr(pod_eval, "kexec", _f)
    pod_eval.remote_eval_running("some-pod", "logs/ckpt_eval/foo_joygate",
                                  "rl_move.sim.eval_joystick_gate")
    assert "rl_move.sim.eval_joystick_gate" in seen["cmd"]
    assert "logs/ckpt_eval/foo_joygate" in seen["cmd"]
