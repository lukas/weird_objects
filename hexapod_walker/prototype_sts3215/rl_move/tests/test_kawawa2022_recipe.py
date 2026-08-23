"""Pins for the walkcurr rung-1 recipe (kawawa2022_recipe.py).

The first adaptation launched from an unpushed desktop clone; these
tests keep the canonical recipe honest: paper-shaped architecture,
walk-pure diet, fixed forward command, the anti-suicide term, and a
launcher-ready backlog command on the registered walkcurr track.
"""
from __future__ import annotations

from torch import nn

from rl_move.sim.kawawa2022_recipe import (
    CFG_SET,
    DEFAULT_RUN,
    DEFAULT_STEPS,
    backlog_command,
    trainer_args,
)
from rl_move.sim.train_ppo_mjx import _activation_fn


def test_recipe_pins_paper_architecture():
    args = trainer_args()
    assert args[args.index("--n-steps") + 1] == "24"
    assert args[args.index("--batch-size") + 1] == "24576"
    assert args[args.index("--net-arch") + 1] == "128,64,32"
    assert args[args.index("--activation-fn") + 1] == "elu"
    # The LSTM trainer support died with the unpushed desktop clone;
    # rung 1 is deliberately memoryless (fixed command needs no
    # memory). If this ever changes it must go through --gru, not a
    # rebuilt --lstm path.
    assert "--lstm" not in args


def test_recipe_is_walk_only_fixed_forward():
    # The named root cause of the flat1 FAIL: multi-goal reward carry.
    assert "goal.walk_pure=1" in CFG_SET
    assert "goal.walk_speed_min_m_s=0.05" in CFG_SET
    assert "goal.walk_speed_max_m_s=0.06" in CFG_SET
    assert "goal.walk_heading_max_rad=0.0" in CFG_SET
    assert "goal.walk_cmd_resample_s=0.0" in CFG_SET


def test_recipe_closes_the_suicide_exploit():
    # flat1's walk episodes tilt-terminated everywhere and dying was
    # free; the rerun prices death per the 08-22 calibration.
    assert "reward.term_penalty=400" in CFG_SET


def test_recipe_keeps_loaded_calibration_and_slip_pricing():
    assert "bus.servo_params=loaded" in CFG_SET
    assert "reward.k_walk_freeprog=2.0" in CFG_SET
    assert "reward.walk_kernel_vel_ema=1.0" in CFG_SET
    assert "reward.walk_loadslip_gate=0.75" in CFG_SET
    assert "reward.k_loadslip_excess=0.8" in CFG_SET


def test_recipe_rung1_has_no_dr_pushes():
    # DR/pushes are rung 5+ hardening; rung 1 is flat nominal physics.
    args = trainer_args()
    assert args[args.index("--dr-scale") + 1] == "0.0"
    assert not any("ext_push" in a for a in args)
    assert not any("friction_scale" in a for a in args)


def test_recipe_bank_cfg_matches_launch_cfg():
    """The WALKCURR_PF semantics bank must price the EXACT reward cfg
    the run launches with (reward/eval alignment is the whole point).
    Every reward.* key in CFG_SET must appear with the same value in
    WALKCURR_PF_OVERRIDES."""
    from rl_move.tests.test_task_semantics import WALKCURR_PF_OVERRIDES
    for item in CFG_SET:
        key, val = item.split("=", 1)
        sec, leaf = key.split(".", 1)
        if sec != "reward":
            continue
        assert (sec, leaf) in WALKCURR_PF_OVERRIDES, (
            f"bank does not price {key}")
        assert float(WALKCURR_PF_OVERRIDES[(sec, leaf)]) == float(val), (
            f"bank prices {key}={WALKCURR_PF_OVERRIDES[(sec, leaf)]} "
            f"but the run launches with {val}")


def test_recipe_backlog_command_is_launcher_ready():
    cmd = backlog_command()
    assert cmd[:4] == ["python", "launch_run.py", "backlog", "add"]
    assert cmd[cmd.index("--run") + 1] == DEFAULT_RUN
    assert cmd[cmd.index("--phase") + 1] == "discovery"
    assert cmd[cmd.index("--track") + 1] == "walkcurr"
    assert DEFAULT_STEPS <= 2_000_000  # guardrails discovery cap
    assert "--" in cmd
    tail = cmd[cmd.index("--") + 1:]
    assert "--steps" not in tail
    assert "--run-name" not in tail


def test_activation_fn_selector():
    assert _activation_fn("tanh") is nn.Tanh
    assert _activation_fn("relu") is nn.ReLU
    assert _activation_fn("elu") is nn.ELU
