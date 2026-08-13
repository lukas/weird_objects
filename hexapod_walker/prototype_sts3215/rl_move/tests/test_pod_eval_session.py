"""Seat selection for the pre-staged interactive SESSION gate.

WISHLIST 8e (landed 08-13): the watcher's post-run evals (pod_eval.py)
add ``rl_move.sim.eval_session`` for every stance/walk candidate —
stance candidates pair with the deployed walk, walk candidates with
the deployed stance. These tests pin the pure seat-selection rule so
the wiring can't silently start sessioning quad/track-only runs or
seat a walk-width checkpoint on the stance side (eval_session requires
the stance obs to be a strict prefix of the walk env obs).
"""
import importlib.util
import pathlib

_P = pathlib.Path(__file__).resolve().parents[1] / "orchestrator" / "pod_eval.py"
_spec = importlib.util.spec_from_file_location("pod_eval", _P)
pod_eval = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pod_eval)

session_side = pod_eval.session_side


def test_stance_only_mix_takes_stance_seat():
    assert session_side(["rise", "hold", "lower"], "goal") == "stance"
    assert session_side(["hold"], "goal") == "stance"


def test_walk_only_takes_walk_seat():
    assert session_side(["walk"], "goal") == "walk"
    # plain joint_walk task with no --goal-mix is the walk default
    assert session_side([], "joint_walk") == "walk"


def test_both_families_take_walk_seat():
    # a walk-training policy is walk-env-width and can only sit in the
    # walk seat (stance seat must be a strict obs prefix)
    assert session_side(["walk", "rise", "hold", "lower"], "goal") == "walk"


def test_non_session_modes_get_no_seat():
    assert session_side(["track"], "goal") is None
    assert session_side(["quad"], "goal") is None
    assert session_side(["quadwalk"], "goal") is None
    assert session_side(["getup"], "goal") is None
    assert session_side([], "goal") is None


def test_deployed_pair_constants_exist_on_controller():
    pol = pathlib.Path(__file__).resolve().parents[1] / "sim" / "policies"
    for name in (pod_eval.DEPLOYED_STANCE, pod_eval.DEPLOYED_WALK):
        p = pol / name
        assert p.is_file() and p.stat().st_size, (
            f"deployed session partner {name} missing from "
            f"rl_move/sim/policies — update pod_eval.py's constants on "
            f"promotion (source of truth: linux_control/rl_policy.py)")
