"""Post-lower rise start bank (goal.rise_start_bank, 08-14).

SESSION_BULK_GATE named the post-lower rise the single trainable
boundary of the hierarchical product baseline. The fix mechanism is
reset-side exposure: rise episodes sampling the policy's OWN harvested
lower-endpoint poses (start_at="rise_bank"). Contract under test:
  - default OFF and bit-exact: frac>0 with NO bank path changes nothing;
  - bank + frac=1: every rise episode starts from a bank pose, the
    rise-RSI hook never overrides it, eval labels it "post_lower";
  - malformed banks fail loudly.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

pytest.importorskip("mujoco")

from rl_move.config import load_config                      # noqa: E402
from rl_move.sim.goal_task import SimHexapodGoalEnv         # noqa: E402
from rl_move.sim.sim_env import N_JOINTS                    # noqa: E402

RISE_REF = ROOT / "rl_move/sim/refs/rise_ref_belly2plant.npz"


def _mk_bank(tmp_path: Path, k: int = 4) -> tuple[Path, np.ndarray]:
    # Distinct belly-ish poses: zero pose with per-row knee curl so the
    # settled reset stays close to the commanded row.
    q = np.zeros((k, N_JOINTS))
    for i in range(k):
        q[i, 2::3] = 0.30 + 0.08 * i
    p = tmp_path / "lower_bank.npz"
    np.savez(p, q_rad=q, meta="{}")
    return p, q


def _rise_env(seed: int, **goal_over) -> SimHexapodGoalEnv:
    cfg = load_config()
    for k, v in goal_over.items():
        cfg.setdefault("goal", {})[k] = v
    env = SimHexapodGoalEnv(cfg=cfg, seed=seed)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 1.0 if m == "rise" else 0.0)
    return env


def test_frac_without_bank_is_bit_exact():
    """rise_start_bank_frac alone (no bank path) must not perturb the
    legacy reset stream — same seed, identical poses and start kinds."""
    env_a = _rise_env(seed=7)
    env_b = _rise_env(seed=7, rise_start_bank_frac=0.7)
    for _ in range(6):
        env_a.reset()
        env_b.reset()
        assert env_b._goal_traj.start_at != "rise_bank"
        assert env_a._goal_traj.start_at == env_b._goal_traj.start_at
        np.testing.assert_array_equal(
            env_a.data.qpos[env_a._qadr], env_b.data.qpos[env_b._qadr])
    env_a.close()
    env_b.close()


def test_bank_frac_one_starts_from_bank_and_skips_rsi(tmp_path):
    bank_path, bank = _mk_bank(tmp_path)
    cfg = load_config()
    cfg.setdefault("goal", {})["rise_start_bank"] = str(bank_path)
    cfg["goal"]["rise_start_bank_frac"] = 1.0
    # Arm the RSI hook at frac 1.0: it must be skipped on bank episodes.
    cfg["goal"]["rise_rsi_frac"] = 1.0
    cfg.setdefault("reward", {})["rise_ref_path"] = str(RISE_REF)
    env = SimHexapodGoalEnv(cfg=cfg, seed=3)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 1.0 if m == "rise" else 0.0)
    from rl_move.sim.eval_checkpoint import _start_kind
    for _ in range(5):
        obs, info = env.reset()
        assert info["goal_mode"] == "rise"
        assert env._goal_traj.start_at == "rise_bank"
        assert _start_kind(env._goal_traj) == "post_lower"
        assert not env._rsi_pending, "RSI must not override bank starts"
        assert np.all(np.isfinite(obs))
        # Settled reset stays near ONE bank row (+-2 deg jitter + sag).
        q = env.data.qpos[env._qadr]
        d = np.abs(bank - q[None, :]).max(axis=1).min()
        assert d < 0.25, f"reset pose {d:.3f} rad from nearest bank row"
    env.close()


def test_bank_frac_mixes_kinds(tmp_path):
    bank_path, _ = _mk_bank(tmp_path)
    env = _rise_env(seed=11, rise_start_bank=str(bank_path),
                    rise_start_bank_frac=0.5)
    kinds = set()
    for _ in range(12):
        env.reset()
        kinds.add(env._goal_traj.start_at)
    assert "rise_bank" in kinds, "bank starts never sampled at frac=0.5"
    assert kinds - {"rise_bank"}, "frac=0.5 must keep synthetic starts"
    env.close()


def test_canary_force_overrides_bank(tmp_path):
    """force_rise_start (canary probes) must pin the synthetic kind even
    with a bank configured — canary streams stay comparable."""
    bank_path, _ = _mk_bank(tmp_path)
    env = _rise_env(seed=5, rise_start_bank=str(bank_path),
                    rise_start_bank_frac=1.0)
    env._goal_gen.force_rise_start = "flat"
    env.reset()
    assert env._goal_traj.start_at == "zero"
    assert env._goal_traj.start_curl == 0.0
    env.close()


def test_malformed_bank_raises(tmp_path):
    bad = tmp_path / "bad.npz"
    np.savez(bad, q_rad=np.zeros((3, 7)))
    env = _rise_env(seed=1, rise_start_bank=str(bad),
                    rise_start_bank_frac=1.0)
    with pytest.raises(ValueError, match="rise_start_bank"):
        for _ in range(3):
            env.reset()
    env.close()
