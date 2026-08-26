import copy

from rl_move.np_policy import validate_np_policy


def _policy(training_hz=25.0):
    return {
        "meta": {
            "obs_dim": 68,
            "act_dim": 18,
            "activation": "tanh",
            "training_hz": training_hz,
        },
        "W1": [[0.0] * 68],
        "b1": [0.0],
        "W2": [[0.0]],
        "b2": [0.0],
        "Wout": [[0.0] for _ in range(18)],
        "bout": [0.0] * 18,
    }


def test_validate_np_policy_requires_training_hz():
    obj = _policy()
    del obj["meta"]["training_hz"]
    errs, _ = validate_np_policy(obj)
    assert "meta.training_hz is required" in errs


def test_validate_np_policy_reports_training_hz():
    errs, info = validate_np_policy(_policy(training_hz=100.0))
    assert errs == []
    assert info["training_hz"] == 100.0


def test_validate_np_policy_rejects_bad_training_hz():
    obj = _policy(training_hz=25.0)
    for bad in ("fast", 0.0, 500.0):
        bad_obj = copy.deepcopy(obj)
        bad_obj["meta"]["training_hz"] = bad
        errs, _ = validate_np_policy(bad_obj)
        assert any("meta.training_hz" in e for e in errs)
