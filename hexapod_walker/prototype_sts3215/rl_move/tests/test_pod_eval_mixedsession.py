"""Auto-prestage trigger for the MIXED-SESSION gate (pod_eval.py).

2026-08-28 (standwalk unified1-mix gap): two independent triage
cycles found, by hand, that a "unified command-following" recipe
(env-native `goal.mode_seq>0`) needs `rl_move.sim.eval_mixed_session`
for its own pre-registered gate, but pod_eval's auto-prestage never
launched it (only `eval_checkpoint`/`eval_session` were wired in) —
both cycles worked around it manually via `ops.sh sessioncmd`. This
pins the pure trigger (`mode_seq_frac`) so the auto-launch this cycle
added can't silently regress to the same gap for the next arm in the
lineage.
"""
import importlib.util
import pathlib
import sys

_ORCH = pathlib.Path(__file__).resolve().parents[1] / "orchestrator"
sys.path.insert(0, str(_ORCH))  # pod_eval imports its sibling ``tracks``
_P = _ORCH / "pod_eval.py"
_spec = importlib.util.spec_from_file_location("pod_eval", _P)
pod_eval = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pod_eval)

mode_seq_frac = pod_eval.mode_seq_frac


def test_no_mode_seq_key_is_zero():
    assert mode_seq_frac(["reward.foo=1", "control.hz=100"]) == 0.0


def test_mode_seq_present_reads_value():
    assert mode_seq_frac(["goal.mode_seq=0.75", "control.hz=100"]) == 0.75


def test_mode_seq_zero_value_does_not_trigger():
    # an explicit but disabled mode_seq (e.g. a respec that turns it
    # back off) must not fire the mixed-session auto-launch either.
    assert mode_seq_frac(["goal.mode_seq=0.0"]) == 0.0


def test_malformed_value_fails_safe_to_zero():
    assert mode_seq_frac(["goal.mode_seq=notanumber"]) == 0.0


def test_real_unified1_mix_cfg_stack_triggers():
    # abbreviated slice of the actual cw-standwalk-unified1-mix-s0
    # extra_args cfg-set list (order-sensitive keys omitted for
    # brevity — the function only cares about the one key).
    cfgs = ["env.model_source=mesh", "control.hz=100",
            "obs.mode_onehot=1", "goal.mode_seq=0.75",
            "goal.walk_cmd_mode=stress_mix"]
    assert mode_seq_frac(cfgs) > 0
