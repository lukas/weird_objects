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
    if "rise_height_mm" in goal_over:
        # The default action ceiling (5mm) clips the band to nothing,
        # and the default 5s pre-ramp hold fills the whole 5s test
        # episode with zeros — match the deployed stance recipe's
        # ceiling and leave room for the ramp whenever a band is pinned.
        cfg.setdefault("actions", {})["max_height_mm"] = 115
        if "rise_hold_s" not in goal_over:
            cfg["goal"]["rise_hold_s"] = 1.0
            cfg["goal"]["rise_hold_min_s"] = 1.0
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


def _mk_full_bank(tmp_path: Path, env_probe: SimHexapodGoalEnv,
                  k: int = 3) -> tuple[Path, np.ndarray, np.ndarray]:
    """Bank with the full-state twin arrays (08-14 exact-restore mode):
    plausible settled states built by probing a real env reset."""
    nq = env_probe.model.nq
    nv = env_probe.model.nv
    q = np.zeros((k, N_JOINTS))
    qpos = np.zeros((k, nq))
    qvel = np.zeros((k, nv))
    for i in range(k):
        q[i, 2::3] = 0.30 + 0.08 * i
        qpos[i, 2] = 0.041 + 0.002 * i        # base z: belly-ish heights
        qpos[i, 3] = 1.0                       # identity quat
        qpos[i, env_probe._qadr] = q[i]
    p = tmp_path / "lower_bank_full.npz"
    np.savez(p, q_rad=q, qpos_full=qpos, qvel_full=qvel, meta="{}")
    return p, q, qpos


def test_exact_restore_uses_full_state(tmp_path):
    """goal.rise_start_bank_exact=1 + full-state bank: the episode
    starts verbatim at a bank state — exact joints (no jitter), base z
    from the bank (recentered x/y), no re-plant/settle distortion."""
    probe = _rise_env(seed=11)
    bank_path, q, qpos = _mk_full_bank(tmp_path, probe)
    probe.close()
    env = _rise_env(seed=11, rise_start_bank=str(bank_path),
                    rise_start_bank_frac=1.0, rise_start_bank_exact=1.0)
    for _ in range(4):
        obs, info = env.reset()
        assert env._goal_traj.start_at == "rise_bank"
        assert np.all(np.isfinite(obs))
        qj = env.data.qpos[env._qadr]
        # nearest bank row: joints exact at placement, then only the
        # 0.3 s stiff hold-settle may move them (servos already at cmd)
        d = np.abs(q - qj[None, :]).max(axis=1).min()
        assert d < 0.05, f"exact restore drifted {d:.3f} rad"
        assert abs(env.data.qpos[0]) < 0.02 and abs(env.data.qpos[1]) < 0.02
    env.close()


def test_exact_flag_with_legacy_bank_falls_back(tmp_path):
    """exact=1 on a joints-only legacy bank: no error, reconstruction
    path (jitter present) — never a silent crash on old banks."""
    bank_path, q = _mk_bank(tmp_path)
    env = _rise_env(seed=5, rise_start_bank=str(bank_path),
                    rise_start_bank_frac=1.0, rise_start_bank_exact=1.0)
    obs, info = env.reset()
    assert env._goal_traj.start_at == "rise_bank"
    assert np.all(np.isfinite(obs))
    env.close()


def test_exact_off_with_full_bank_is_bit_exact_with_legacy_path(tmp_path):
    """A full-state bank with exact OFF must behave exactly like the
    joints-only reconstruction path (default semantics unchanged)."""
    probe = _rise_env(seed=13)
    full_path, q, _ = _mk_full_bank(tmp_path, probe)
    probe.close()
    legacy_path = tmp_path / "legacy_twin.npz"
    np.savez(legacy_path, q_rad=q, meta="{}")
    env_a = _rise_env(seed=9, rise_start_bank=str(legacy_path),
                      rise_start_bank_frac=1.0)
    env_b = _rise_env(seed=9, rise_start_bank=str(full_path),
                      rise_start_bank_frac=1.0)
    for _ in range(3):
        env_a.reset()
        env_b.reset()
        np.testing.assert_array_equal(
            env_a.data.qpos[env_a._qadr], env_b.data.qpos[env_b._qadr])
    env_a.close()
    env_b.close()


def _mk_anchored_bank(tmp_path: Path, env_probe: SimHexapodGoalEnv,
                      k: int = 3) -> tuple[Path, np.ndarray, np.ndarray]:
    """Full-state bank WITH the z_stand anchor array (08-14 root-cause
    fix: bank spawns settle above the belly, so the belly-calibrated
    z0-relative band must be rewritten to the remaining rise)."""
    nq, nv = env_probe.model.nq, env_probe.model.nv
    q = np.zeros((k, N_JOINTS))
    qpos = np.zeros((k, nq))
    qvel = np.zeros((k, nv))
    z_stand = np.zeros(k)
    for i in range(k):
        q[i, 2::3] = 0.30 + 0.08 * i
        qpos[i, 2] = 0.041 + 0.002 * i
        qpos[i, 3] = 1.0
        qpos[i, env_probe._qadr] = q[i]
        # Anchor ~50mm above the synthetic spawn's settle (~40mm): the
        # remaining rise must come out ~half the belly band, so a band
        # leak is unambiguous.
        z_stand[i] = 0.090 + 0.001 * i
    p = tmp_path / "lower_bank_anchored.npz"
    np.savez(p, q_rad=q, qpos_full=qpos, qvel_full=qvel,
             z_stand=z_stand, meta="{}")
    return p, q, z_stand


def test_anchor_stand_rewrites_schedule_to_remaining_rise(tmp_path):
    """anchor_stand=1: the height schedule ends at z_stand - z0 (the
    remaining rise back to standing), NOT the belly band (108-114mm)."""
    probe = _rise_env(seed=17)
    bank_path, q, z_stand = _mk_anchored_bank(tmp_path, probe)
    probe.close()
    env = _rise_env(seed=17, rise_start_bank=str(bank_path),
                    rise_start_bank_frac=1.0, rise_start_bank_exact=1.0,
                    rise_start_bank_anchor_stand=1.0,
                    rise_height_mm=[108, 114])
    for _ in range(4):
        env.reset()
        assert env._goal_traj.start_at == "rise_bank"
        h_end = float(np.asarray(env._goal_traj.height)[-1])
        want = max(min(z_stand) - env._z0, 0.002)
        want_hi = max(max(z_stand) - env._z0, 0.002)
        assert want - 1e-9 <= h_end <= want_hi + 1e-9, (
            f"schedule end {h_end*1000:.1f}mm not the remaining rise "
            f"[{want*1000:.1f}, {want_hi*1000:.1f}]mm")
        assert h_end < 0.080, "belly band leaked through the re-anchor"
    env.close()


def test_anchor_stand_off_keeps_legacy_schedule(tmp_path):
    """Same anchored bank, flag OFF: schedule stays the belly band."""
    probe = _rise_env(seed=19)
    bank_path, q, _ = _mk_anchored_bank(tmp_path, probe)
    probe.close()
    env = _rise_env(seed=19, rise_start_bank=str(bank_path),
                    rise_start_bank_frac=1.0, rise_start_bank_exact=1.0,
                    rise_height_mm=[108, 114])
    env.reset()
    h_end = float(np.asarray(env._goal_traj.height)[-1])
    assert 0.100 <= h_end <= 0.120, f"legacy band changed: {h_end}"
    env.close()


def test_anchor_stand_on_legacy_bank_raises(tmp_path):
    """anchor_stand=1 on a bank without z_stand must fail LOUDLY."""
    bank_path, _ = _mk_bank(tmp_path)
    env = _rise_env(seed=21, rise_start_bank=str(bank_path),
                    rise_start_bank_frac=1.0,
                    rise_start_bank_anchor_stand=1.0)
    with pytest.raises(ValueError, match="z_stand"):
        env.reset()
    env.close()
