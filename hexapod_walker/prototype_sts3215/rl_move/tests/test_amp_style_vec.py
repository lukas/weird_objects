"""test_amp_style_vec.py — AMP track M1 item 3: the live reward-loop
wiring (2026-08-22). Proves, on CPU only (no jax/mjx needed):

1. env contract: goal.amp_style_obs=1 makes the walk env emit a
   finite 60-dim info["amp_obs_style"] whose first 18 dims are RAW
   joint angles (neutral=0); the DEFAULT cfg emits NO key and takes
   the untouched code path (bit-exact off);
2. MotionLibrary.neutral_pose exists, is 18-dim, and equals the
   per-clip derivation for every clip (the single-neutral convention
   the live wiring assumes);
3. AMPStyleVecWrapper blend math: r = task_w * r_env + style_w *
   r_style with r_style in [0,1], zero on the first tick of every
   episode (no valid pair) and after every done (boundary masking);
4. transitions crossing a done are never pushed to the replay ring;
   the ring wraps correctly at capacity;
5. train_discriminator returns None until one batch fits, then runs
   finite updates that move D(real) above D(fake) on the synthetic
   setup; save()/load() round-trips the discriminator exactly.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.sim.amp_discriminator import MotionLibrary  # noqa: E402
from rl_move.sim.amp_style_vec import (  # noqa: E402
    AMPStyleVecWrapper, _TransitionRing)


# ---------------------------------------------------------------- env

def _cpu_walk_env(cfg=None):
    from rl_move.config import load_config
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = cfg if cfg is not None else load_config()
    return SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=3.0, seed=0, cfg=cfg)


def test_env_emits_obs_style_only_when_cfg_on():
    from rl_move.config import load_config
    # OFF (default): no key — the untouched legacy path.
    env = _cpu_walk_env()
    env.reset()
    _, _, _, _, info = env.step(np.zeros(env.action_space.shape,
                                         dtype=np.float32))
    assert "amp_obs_style" not in info
    env.close()

    # ON: 60-dim finite vector, dims 0..17 = RAW joint angles.
    cfg = load_config()
    cfg.setdefault("goal", {})["amp_style_obs"] = 1.0
    env = _cpu_walk_env(cfg)
    env.reset()
    _, _, _, _, info = env.step(np.zeros(env.action_space.shape,
                                         dtype=np.float32))
    v = info["amp_obs_style"]
    assert v.shape == (60,)
    assert np.all(np.isfinite(v))
    q_now = np.asarray(env.data.qpos, dtype=np.float64)[env._qadr]
    np.testing.assert_allclose(v[:18], q_now, atol=1e-6)
    env.close()


# ------------------------------------------------------------ library

def test_motion_library_neutral_pose_single_convention():
    lib = MotionLibrary()
    assert lib.neutral_pose is not None
    assert lib.neutral_pose.shape == (18,)
    z = np.load(lib_path_default(), allow_pickle=True)
    jp, rel = z["joint_position"], z["joint_position_rel_neutral"]
    for s in z["clip_starts"]:
        np.testing.assert_allclose(lib.neutral_pose, jp[s] - rel[s],
                                   atol=1e-6)


def lib_path_default():
    from rl_move.sim.amp_discriminator import DEFAULT_LIBRARY
    return DEFAULT_LIBRARY


# ------------------------------------------------------- stub vec env

class _StubVecEnv:
    """Minimal VecEnv double: constant task reward 2.0/tick, scripted
    dones, obs_style rows drawn near the library manifold so the maths
    are deterministic and library-normalization stays finite."""

    def __init__(self, n_envs=3, feat_dim=60, done_script=None):
        self.num_envs = n_envs
        self.feat_dim = feat_dim
        self.observation_space = None
        self.action_space = None
        self.render_mode = None
        self._t = 0
        self._done_script = done_script or {}
        lib = MotionLibrary()
        self._base = lib.obs_style[0].astype(np.float32).copy()
        self._neutral = lib.neutral_pose.copy()
        self.emitted = []          # (t, env, row) for cross-checks

    def reset(self):
        self._t = 0
        return np.zeros((self.num_envs, 4), dtype=np.float32)

    def step_async(self, actions):
        pass

    def step_wait(self):
        self._t += 1
        rews = np.full(self.num_envs, 2.0, dtype=np.float32)
        dones = np.zeros(self.num_envs, dtype=bool)
        for (t, i) in self._done_script:
            if t == self._t:
                dones[i] = True
        infos = []
        for i in range(self.num_envs):
            row = self._base.copy()
            row += 0.01 * np.float32(self._t) + 0.001 * np.float32(i)
            # env emits RAW joints: add the neutral back onto dims 0..17
            raw = row.copy()
            raw[:18] += self._neutral
            infos.append({"amp_obs_style": raw})
            self.emitted.append((self._t, i, row))
        obs = np.zeros((self.num_envs, 4), dtype=np.float32)
        return obs, rews, dones, infos

    def close(self):
        pass

    # VecEnvWrapper delegates it needs
    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    def get_attr(self, name, indices=None):
        raise AttributeError(name)


def _wrap(stub, **kw):
    kw.setdefault("style_weight", 0.5)
    kw.setdefault("task_weight", 0.7)
    kw.setdefault("replay_size", 64)
    kw.setdefault("seed", 0)
    return AMPStyleVecWrapper(stub, **kw)


def test_wrapper_rejects_style_weight_zero():
    with pytest.raises(ValueError):
        _wrap(_StubVecEnv(), style_weight=0.0)


def test_blend_and_first_tick_masking():
    stub = _StubVecEnv(n_envs=2)
    w = _wrap(stub)
    w.reset()
    w.step_async(None)
    obs, r1, d1, infos1 = w.step_wait()
    # tick 1: no previous obs_style -> style contribution exactly 0
    np.testing.assert_allclose(r1, 0.7 * 2.0, atol=1e-6)
    assert all(i["reward_amp_style"] == 0.0 for i in infos1)
    w.step_async(None)
    obs, r2, d2, infos2 = w.step_wait()
    # tick 2: valid pair -> r = 0.7*2.0 + 0.5*style, style in [0,1]
    style = (np.asarray(r2) - 0.7 * 2.0) / 0.5
    assert np.all(style >= -1e-6) and np.all(style <= 1.0 + 1e-6)
    # the emitted info key carries style_weight * r_style
    for i, info in enumerate(infos2):
        np.testing.assert_allclose(info["reward_amp_style"],
                                   0.5 * style[i], atol=1e-6)
    assert len(w.ring) == 2  # one pair per env


def test_done_masks_boundary_pair():
    # env 0 done at t=2: pair (1->2) IS consumed (ends in terminal
    # state), pair (2->3) must NOT exist; env 1 never done.
    stub = _StubVecEnv(n_envs=2, done_script=[(2, 0)])
    w = _wrap(stub)
    w.reset()
    counts = []
    for _ in range(3):
        w.step_async(None)
        _, r, d, infos = w.step_wait()
        counts.append(len(w.ring))
    # t1: 0 pairs; t2: 2 pairs (both envs, incl. terminal-ending);
    # t3: +1 (env 1 only — env 0's boundary pair dropped)
    assert counts == [0, 2, 3]
    # and env 0's style contribution at t3 is exactly 0
    assert infos[0]["reward_amp_style"] == 0.0
    assert infos[1]["reward_amp_style"] >= 0.0


def test_ring_wraps_at_capacity():
    ring = _TransitionRing(8, 3)
    a = np.arange(30, dtype=np.float32).reshape(10, 3)
    ring.push(a[:5], a[:5] + 100)
    assert len(ring) == 5
    ring.push(a[5:], a[5:] + 100)          # 10 total -> wraps to 8
    assert len(ring) == 8
    # newest rows must all be present exactly once
    got = {tuple(r) for r in ring.s_t}
    want = {tuple(r) for r in a[2:]}       # oldest 2 evicted
    assert got == want
    s, s1 = ring.sample(16, np.random.default_rng(0))
    np.testing.assert_allclose(s1 - s, 100.0)


def test_train_discriminator_and_save_load(tmp_path):
    stub = _StubVecEnv(n_envs=4)
    w = _wrap(stub, replay_size=256)
    w.reset()
    # too little data -> None (no crash)
    assert w.train_discriminator(2, 64) is None
    for _ in range(40):
        w.step_async(None)
        w.step_wait()
    stats = w.train_discriminator(8, 64)
    assert stats is not None
    for k, v in stats.items():
        assert np.isfinite(v), k
    assert stats["disc_updates"] == 8.0
    # after a few updates real should score above fake on this setup
    stats2 = w.train_discriminator(20, 64)
    assert stats2["d_real_mean"] > stats2["d_fake_mean"]

    p = tmp_path / "disc.amp_disc.pt"
    w.save(p)
    w2 = _wrap(_StubVecEnv(n_envs=4), disc_init=p)
    for a, b in zip(w.disc.parameters(), w2.disc.parameters()):
        assert torch.equal(a, b)
    assert w2.disc_updates == w.disc_updates

    roll = w.pop_rollout_stats()
    assert roll["pairs"] > 0
    roll2 = w.pop_rollout_stats()
    assert roll2["pairs"] == 0.0
