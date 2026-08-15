import sys

import pytest

from rl_move.dynamics import fresh_pipeline


def test_pipeline_collects_budget_then_execs_transformer(monkeypatch):
    calls = []

    def fake_run(command, check):
        calls.append(("collect", command, check))

    def fake_exec(executable, command):
        calls.append(("train", executable, command))
        raise RuntimeError("exec captured")

    monkeypatch.setattr(fresh_pipeline.subprocess, "run", fake_run)
    monkeypatch.setattr(fresh_pipeline.os, "execv", fake_exec)
    monkeypatch.setattr(sys, "argv", [
        "fresh_pipeline", "--name", "cw-dynrep-fresh-test",
        "--steps", "40000", "--data", "fresh_ds", "--batch", "512",
        "--max-window-reuse", "2", "--collect-n-envs", "2048",
        "--arch", "transformer", "--tf-layers", "4", "--tf-heads", "8",
        "--tf-ff", "1024", "--hidden", "512", "--z-dim", "256",
        "--device", "cuda",
    ])
    with pytest.raises(RuntimeError, match="exec captured"):
        fresh_pipeline.main()

    collect = calls[0][1]
    assert calls[0][2] is True
    assert "rl_move.dynamics.collect_mjx" in collect
    assert collect[collect.index("--optimizer-steps") + 1] == "40000"
    assert collect[collect.index("--max-window-reuse") + 1] == "2.0"
    assert collect[collect.index("--n-envs") + 1] == "2048"

    train = calls[1][2]
    assert "rl_move.dynamics.train" in train
    assert "--arch" in train and train[train.index("--arch") + 1] == "transformer"
    assert "--hidden" in train and train[train.index("--hidden") + 1] == "512"
    assert "--collect-n-envs" not in train
