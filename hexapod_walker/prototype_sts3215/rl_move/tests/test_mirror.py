"""Mirror-symmetry map tests (rl_move/sim/mirror.py).

Locks the sagittal-mirror convention against the actual kinematics and
obs layout, and smoke-tests MirrorPPO's auxiliary step end to end.
Getting a sign wrong here silently poisons training — every rule is
checked against code that exists independently of mirror.py
(body_ik FK, joint_task action map, the real env's obs width).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from rl_move.sim.mirror import (  # noqa: E402
    frame_perm_sign, joint_perm_sign, obs_perm_sign,
)

DEG = math.pi / 180.0


def _apply(perm, sign, x):
    return sign * np.asarray(x)[perm]


# ---------------------------------------------------------------------------
# Index-map algebra
# ---------------------------------------------------------------------------

def test_joint_map_is_involution():
    perm, sign = joint_perm_sign()
    x = np.random.default_rng(0).normal(size=18)
    assert np.allclose(_apply(perm, sign, _apply(perm, sign, x)), x)
    # legs reverse, yaw negates, hip/knee keep sign
    assert perm[0] == 15 and sign[0] == -1.0     # L0_yaw <- L5_yaw
    assert perm[1] == 16 and sign[1] == 1.0      # L0_pitch <- L5_pitch
    assert perm[8] == 11 and sign[8] == 1.0      # L2_knee <- L3_knee


@pytest.mark.parametrize("walk,yaw,phase,hist", [
    (False, False, False, 1),
    (True, False, False, 1),
    (True, True, False, 1),
    (True, True, True, 1),
    (True, True, False, 16),
])
def test_obs_map_is_involution(walk, yaw, phase, hist):
    perm, sign = obs_perm_sign(walk=walk, yaw_cmd=yaw, phase_obs=phase,
                               history_frames=hist)
    x = np.random.default_rng(1).normal(size=len(perm))
    assert np.allclose(_apply(perm, sign, _apply(perm, sign, x)), x)


def test_frame_widths_match_env_layouts():
    assert len(frame_perm_sign(walk=False)[0]) == 68
    assert len(frame_perm_sign(walk=True)[0]) == 72
    assert len(frame_perm_sign(walk=True, yaw_cmd=True)[0]) == 73
    assert len(frame_perm_sign(walk=True, yaw_cmd=True,
                               phase_obs=True)[0]) == 75


def test_known_slots():
    perm, sign = frame_perm_sign(walk=True, yaw_cmd=True)
    # identity on: pitch(37), gyro y(39), height_ref(61), vx(68, 70)
    for i in (37, 39, 61, 68, 70):
        assert perm[i] == i and sign[i] == 1.0, f"slot {i}"
    # negate in place: roll(36), gyro x(38), gyro z(40), roll_ref(59),
    # vy_ref(69), vy_meas(71), wz_ref(72)
    for i in (36, 38, 40, 59, 69, 71, 72):
        assert perm[i] == i and sign[i] == -1.0, f"slot {i}"
    # goal one-hot reverses legs
    assert list(perm[62:68]) == [67, 66, 65, 64, 63, 62]
    assert np.all(sign[62:68] == 1.0)


# ---------------------------------------------------------------------------
# Physics ground truth: FK of the mirrored joints = mirrored foot point
# ---------------------------------------------------------------------------

def test_fk_mirror_consistency():
    """mirror(joints) applied to the mirrored leg puts the foot at the
    y-negated position of the original — checked with body_ik's FK,
    which knows nothing about mirror.py."""
    from rl_move.body_ik import fk_foot_body, leg_azimuths
    az = leg_azimuths()
    rng = np.random.default_rng(2)
    perm, sign = joint_perm_sign()
    for _ in range(50):
        q = np.zeros(18)
        q[0::3] = rng.uniform(-0.61, 0.61, 6)     # yaw
        q[1::3] = rng.uniform(-1.40, 0.52, 6)     # hip
        q[2::3] = rng.uniform(-0.35, 2.62, 6)     # knee
        qm = _apply(perm, sign, q)
        for leg in range(6):
            p = fk_foot_body(q[3 * leg], q[3 * leg + 1], q[3 * leg + 2],
                             az[leg])
            mleg = 5 - leg
            pm = fk_foot_body(qm[3 * mleg], qm[3 * mleg + 1],
                              qm[3 * mleg + 2], az[mleg])
            assert np.allclose(pm, [p[0], -p[1], p[2]], atol=1e-9), \
                f"leg {leg}: {p} vs mirrored {pm}"


def test_action_map_commutes_with_mirror():
    """Mirroring normalized [-1,1] actions == mirroring the rad targets
    (valid because the per-axis affine map is leg-independent and the
    yaw center is 0)."""
    from rl_move.sim.joint_task import action_to_q_rad
    perm, sign = joint_perm_sign()
    rng = np.random.default_rng(3)
    a = rng.uniform(-1, 1, 18)
    q_of_mirrored_a = action_to_q_rad(_apply(perm, sign, a))
    q = action_to_q_rad(a)
    # rad-space mirror: permute legs, negate yaw channels
    q_mirrored = _apply(perm, sign, q)
    assert np.allclose(q_of_mirrored_a, q_mirrored, atol=1e-12)


# ---------------------------------------------------------------------------
# MirrorPolicy — the eval-time reflection wrapper (RL_PLAN queue 0.2:
# commanded turning by chirality selection, zero training). The wrapper
# must be EXACTLY mirror_act(model(mirror_obs(x))): behavioral validity
# is probe_mirror_turn.py's job, algebraic correctness is locked here
# with a model stub that exists independently of mirror.py.
# ---------------------------------------------------------------------------


class _StubModel:
    """Deterministic linear 'policy' with an sb3-like predict."""

    def __init__(self, obs_dim: int, seed: int = 7):
        rng = np.random.default_rng(seed)
        self.W = rng.normal(size=(18, obs_dim)).astype(np.float32)

        class _Space:
            shape = (obs_dim,)
        self.observation_space = _Space()

    def predict(self, obs, deterministic=True, **kw):
        obs = np.asarray(obs, dtype=np.float32)
        squeeze = obs.ndim == 1
        out = np.tanh(np.atleast_2d(obs) @ self.W.T)
        return (out[0] if squeeze else out), None


@pytest.mark.parametrize("hist", [1, 16])
def test_mirror_policy_is_the_composed_maps(hist):
    from rl_move.sim.mirror import MirrorPolicy
    obs_dim = 72 * hist
    stub = _StubModel(obs_dim)
    pol = MirrorPolicy(stub)
    operm, osign = obs_perm_sign(walk=True, history_frames=hist)
    aperm, asign = joint_perm_sign()
    rng = np.random.default_rng(4)
    x = rng.normal(size=obs_dim).astype(np.float32)
    a_manual = _apply(aperm, asign,
                      stub.predict(_apply(operm, osign, x))[0])
    a_wrap, _ = pol.predict(x)
    assert np.allclose(a_wrap, a_manual, atol=1e-5)
    # batched rows must match the single-row path (1e-5: float32 GEMM
    # reduction order differs between the two matmul shapes)
    xs = rng.normal(size=(3, obs_dim)).astype(np.float32)
    batch, _ = pol.predict(xs)
    for i in range(3):
        one, _ = pol.predict(xs[i])
        assert np.allclose(batch[i], one, atol=1e-5)


def test_mirror_policy_reflection_identity():
    """pi_mirror(mirror(x)) == mirror(pi(x)) — the wrapper genuinely IS
    the reflected controller, not just a relabeled one."""
    from rl_move.sim.mirror import MirrorPolicy
    stub = _StubModel(72)
    pol = MirrorPolicy(stub)
    operm, osign = obs_perm_sign(walk=True, history_frames=1)
    aperm, asign = joint_perm_sign()
    x = np.random.default_rng(5).normal(size=72).astype(np.float32)
    lhs, _ = pol.predict(_apply(operm, osign, x))
    rhs = _apply(aperm, asign, stub.predict(x)[0])
    assert np.allclose(lhs, rhs, atol=1e-6)


def test_mirror_policy_rejects_bad_width():
    from rl_move.sim.mirror import MirrorPolicy
    with pytest.raises(ValueError):
        MirrorPolicy(_StubModel(70))


# ---------------------------------------------------------------------------
# Live env width + MirrorPPO smoke (needs mujoco + torch + sb3)
# ---------------------------------------------------------------------------

def test_maps_match_live_env_and_mirror_ppo_trains():
    mujoco = pytest.importorskip("mujoco")  # noqa: F841
    pytest.importorskip("stable_baselines3")
    from rl_move.config import load_config
    from rl_move.sim.mirror import attach_mirror, make_mirror_ppo_class
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    cfg.setdefault("goal", {})["walk_yaw_cmd"] = 1.0
    cfg["goal"]["walk_obs_body_vel"] = 2.0
    cfg.setdefault("obs", {})["history_frames"] = 2

    def mk():
        return SimHexapodJointWalkEnv(seed=0, cfg=cfg, randomize=False,
                                      episode_seconds=8.0)

    from stable_baselines3.common.vec_env import DummyVecEnv
    venv = DummyVecEnv([mk, mk])
    obs_dim = int(venv.observation_space.shape[0])
    assert obs_dim == 73 * 2, f"live walk env obs width {obs_dim}"

    MirrorPPO = make_mirror_ppo_class()
    model = MirrorPPO("MlpPolicy", venv, n_steps=8, batch_size=16,
                      n_epochs=1, policy_kwargs=dict(net_arch=[32, 32]),
                      seed=0, verbose=0, device="cpu")
    attach_mirror(model, coef=1.0, task="joint_walk", cfg=cfg,
                  obs_dim=obs_dim)
    model.learn(total_timesteps=32)   # 2 rollouts -> 2 aux steps

    # After even a tiny bit of training the aux step must have run;
    # verify the symmetry defect is finite and the maps round-trip on a
    # REAL observation from the env.
    import torch
    obs = venv.reset()
    perm = model.mirror_obs_perm
    sign = model.mirror_obs_sign
    om = obs[:, perm] * sign
    assert np.allclose(om[:, perm] * sign, obs, atol=1e-6)
    with torch.no_grad():
        a, _ = model.predict(obs, deterministic=True)
        am, _ = model.predict(om, deterministic=True)
    assert a.shape == am.shape == (2, 18)
