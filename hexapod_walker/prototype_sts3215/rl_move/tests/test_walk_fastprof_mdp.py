"""Fast-profile command-tracking MDP prep (08-20, operator note
fb_20260820T000059 item 3 — pre-launch code for any funded fast-gait
run; the steer5-fastprof1 canary showed every checkpoint overruns a
0.05-0.06 m/s command 2.5x and drifts 50-60 deg off heading under the
raised servo profile):

  - reward.k_walk_overspeed / walk_overspeed_tol (walk_task): opt-in
    charge on commanded-band EXCEEDANCE (not progress shortfall) —
    -k * min(over/s_ref, 3), over = max(0, |v| - (1+tol)*s_ref).
  - reward.k_walk_heading / walk_heading_min_speed_m_s: opt-in charge
    -k * (1 - cos(heading error)) on ticks actually moving.
  - goal.walk_obs_body_vel=3: DEPLOYABLE leg-odometry estimator obs
    (rl_move.estimator.LegOdometryVelocity on the DR-corrupted observed
    state). Before 08-20 a cfg value of 3 silently fell into the
    PRIVILEGED branch; mode 2 (the cw-dep contract) carries zero
    body-velocity information by construction.
  - eval_checkpoint.pinned_speed_cfg: the pinned-speed panel's command
    pinning must actually pin (speed exact, pure forward, no resample).

All default-off; the default path must stay bit-exact (no new info
keys, same rewards). Fast (~seconds): short episodes, scripted or
forced velocities, no PPO.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

CMD = 0.05
NEW_INFO_KEYS = ("reward_walk_overspeed", "walk_overspeed_m_s",
                 "reward_walk_heading", "walk_heading_cos")


def _walk_env(extra=None, seed=0, episode_seconds=6.0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in (extra or {}).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _pin_forward(env, speed=CMD):
    """Constant forward command from tick 0 (no hold/ramp)."""
    traj = env._goal_traj
    traj.vx[:] = speed
    traj.vy[:] = 0.0
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0


def _force_vel(env, vx, vy):
    env._body_vel_xy = lambda: np.array([vx, vy], dtype=float)


def _step_info(env):
    act = np.zeros(env.action_space.shape, dtype=np.float32)
    _obs, r, _term, _trunc, info = env.step(act)
    return float(r), info


# ------------------------------------------------------------------ #
# default-off bit-exactness
# ------------------------------------------------------------------ #

def test_default_off_emits_no_new_keys_and_matches_zero_k():
    env_a = _walk_env(seed=3)
    env_b = _walk_env(seed=3, extra={
        ("reward", "k_walk_overspeed"): 0.0,
        ("reward", "k_walk_heading"): 0.0,
    })
    env_a.reset()
    env_b.reset()
    _pin_forward(env_a)
    _pin_forward(env_b)
    for _ in range(10):
        ra, ia = _step_info(env_a)
        rb, ib = _step_info(env_b)
        assert ra == pytest.approx(rb, abs=0.0)
        for k in NEW_INFO_KEYS:
            assert k not in ia and k not in ib
    env_a.close()
    env_b.close()


# ------------------------------------------------------------------ #
# overspeed charge semantics (forced velocity — exact operating points)
# ------------------------------------------------------------------ #

def test_overspeed_charge_zero_in_band_and_linear_beyond():
    env = _walk_env(extra={("reward", "k_walk_overspeed"): 2.0})
    env.reset()
    _pin_forward(env)

    _force_vel(env, CMD, 0.0)             # perfect tracking
    _r, info = _step_info(env)
    assert info["walk_overspeed_m_s"] == pytest.approx(0.0)
    assert "reward_walk_overspeed" not in info

    _force_vel(env, 1.05 * CMD, 0.0)      # inside the 10% band
    _r, info = _step_info(env)
    assert info["walk_overspeed_m_s"] == pytest.approx(0.0)
    assert "reward_walk_overspeed" not in info

    _force_vel(env, 0.14, 0.0)            # the canary's overshoot
    _r, info = _step_info(env)
    over = 0.14 - 1.10 * CMD
    assert info["walk_overspeed_m_s"] == pytest.approx(over, abs=1e-9)
    assert info["reward_walk_overspeed"] == pytest.approx(
        -2.0 * min(over / CMD, 3.0), abs=1e-9)

    _force_vel(env, 0.50, 0.0)            # absurd — cap must bound it
    _r, info = _step_info(env)
    assert info["reward_walk_overspeed"] == pytest.approx(-6.0)
    env.close()


def test_overspeed_reward_actually_reduces_return():
    """Same forced velocity, k on vs off: the charge lands in reward."""
    env_on = _walk_env(extra={("reward", "k_walk_overspeed"): 2.0})
    env_off = _walk_env()
    for env in (env_on, env_off):
        env.reset()
        _pin_forward(env)
        _force_vel(env, 0.14, 0.0)
    r_on, i_on = _step_info(env_on)
    r_off, i_off = _step_info(env_off)
    assert r_on < r_off - 1.0
    assert r_on - r_off == pytest.approx(
        i_on["reward_walk_overspeed"], abs=1e-6)
    env_on.close()
    env_off.close()


# ------------------------------------------------------------------ #
# heading-error charge semantics
# ------------------------------------------------------------------ #

def test_heading_charge_zero_aligned_scales_with_angle():
    env = _walk_env(extra={("reward", "k_walk_heading"): 2.0})
    env.reset()
    _pin_forward(env)

    _force_vel(env, CMD, 0.0)             # aligned
    _r, info = _step_info(env)
    assert info["walk_heading_cos"] == pytest.approx(1.0)
    assert info["reward_walk_heading"] == pytest.approx(0.0, abs=1e-12)

    c45 = CMD * np.cos(np.radians(45)), CMD * np.sin(np.radians(45))
    _force_vel(env, *c45)                 # 45 deg off
    _r, info = _step_info(env)
    r45 = info["reward_walk_heading"]
    assert r45 == pytest.approx(-2.0 * (1 - np.cos(np.radians(45))),
                                abs=1e-9)

    _force_vel(env, 0.0, CMD)             # 90 deg off — worse
    _r, info = _step_info(env)
    assert info["reward_walk_heading"] < r45
    assert info["reward_walk_heading"] == pytest.approx(-2.0, abs=1e-9)
    env.close()


def test_heading_charge_skips_near_stationary_ticks():
    env = _walk_env(extra={("reward", "k_walk_heading"): 2.0})
    env.reset()
    _pin_forward(env)
    _force_vel(env, 0.0, 0.005)           # below 0.01 m/s floor
    _r, info = _step_info(env)
    assert "reward_walk_heading" not in info
    env.close()


def test_charges_skip_stop_segments():
    env = _walk_env(extra={("reward", "k_walk_overspeed"): 2.0,
                           ("reward", "k_walk_heading"): 2.0})
    env.reset()
    _pin_forward(env, speed=0.0)          # commanded stop
    _force_vel(env, 0.10, 0.0)            # creeping through it
    _r, info = _step_info(env)
    for k in NEW_INFO_KEYS:
        assert k not in info
    env.close()


# ------------------------------------------------------------------ #
# estimator-sourced velocity obs (goal.walk_obs_body_vel=3)
# ------------------------------------------------------------------ #

def test_vel_obs_mode3_feeds_estimator_output():
    from rl_move.sim.walk_task import VEL_SCALE
    env = _walk_env(extra={("goal", "walk_obs_body_vel"): 3.0,
                           ("obs", "history_frames"): 1.0})
    obs, _ = env.reset()
    assert env._vel_est is not None
    # Velocity slots sit right after the base frame; with every other
    # optional tail off they are the last two dims.
    assert obs[-2:] == pytest.approx([0.0, 0.0])   # fresh estimator
    _pin_forward(env)

    seen = []
    orig = env._vel_est.update

    def rec(*a, **kw):
        out = orig(*a, **kw)
        seen.append(np.array(out, dtype=float))
        return out

    env._vel_est.update = rec
    act = np.zeros(env.action_space.shape, dtype=np.float32)
    for _ in range(8):
        obs, _r, term, trunc, _info = env.step(act)
        assert not (term or trunc)
        assert obs[-2:] == pytest.approx(seen[-1] / VEL_SCALE)
    env.close()


def test_vel_obs_mode3_differs_from_privileged_and_keeps_width():
    ex = {("obs", "history_frames"): 1.0}
    env1 = _walk_env(extra={("goal", "walk_obs_body_vel"): 1.0, **ex},
                     seed=5)
    env3 = _walk_env(extra={("goal", "walk_obs_body_vel"): 3.0, **ex},
                     seed=5)
    o1, _ = env1.reset()
    o3, _ = env3.reset()
    assert o1.shape == o3.shape           # warm-start compatible width
    _pin_forward(env1)
    _pin_forward(env3)
    act = np.zeros(env1.action_space.shape, dtype=np.float32)
    d = 0.0
    for _ in range(12):
        o1 = env1.step(act)[0]
        o3 = env3.step(act)[0]
        # identical seeds/actions -> same physics; only the velocity
        # slots' SOURCE differs, everything else identical.
        assert o1[:-2] == pytest.approx(o3[:-2], abs=1e-6)
        d = max(d, float(np.max(np.abs(o1[-2:] - o3[-2:]))))
    assert d > 0.0                        # estimator is not privileged
    env1.close()
    env3.close()


def test_vel_obs_mode3_resets_between_episodes():
    env = _walk_env(extra={("goal", "walk_obs_body_vel"): 3.0,
                           ("obs", "history_frames"): 1.0},
                    episode_seconds=1.0)
    env.reset()
    est_first = env._vel_est
    act = np.zeros(env.action_space.shape, dtype=np.float32)
    for _ in range(30):
        _obs, _r, term, trunc, _info = env.step(act)
        if term or trunc:
            break
    obs, _ = env.reset()
    assert env._vel_est is not est_first  # fresh per episode
    assert obs[-2:] == pytest.approx([0.0, 0.0])
    env.close()


# ------------------------------------------------------------------ #
# pinned-speed panel command pinning
# ------------------------------------------------------------------ #

def test_pinned_speed_cfg_pins_the_sampled_command():
    from rl_move.sim.eval_checkpoint import (PINNED_SPEED_DEFAULTS,
                                             pinned_speed_cfg)
    assert PINNED_SPEED_DEFAULTS == (0.04, 0.06, 0.08, 0.10)
    for speed in (0.04, 0.10):
        extra = {("goal", k): v
                 for k, v in pinned_speed_cfg(speed).items()}
        env = _walk_env(extra=extra, seed=7)
        env.reset()
        traj = env._goal_traj
        late = slice(int(2.5 / env.dt), None)   # past hold+ramp
        assert np.allclose(traj.vx[late], speed, atol=1e-12)
        assert np.allclose(traj.vy[late], 0.0, atol=1e-12)
        if getattr(traj, "wz", None) is not None:
            assert np.allclose(traj.wz[late], 0.0, atol=1e-12)
        env.close()


def test_pinned_speed_cfg_is_applyable_to_a_live_env():
    """The panel mutates a LIVE env's cfg between episodes; the next
    sampled episode must obey the new pin (sample-time keys only)."""
    from rl_move.sim.eval_checkpoint import pinned_speed_cfg
    env = _walk_env(seed=9)
    env.reset()
    for k, val in pinned_speed_cfg(0.08).items():
        env.cfg.setdefault("goal", {})[k] = val
    env.reset()
    traj = env._goal_traj
    late = slice(int(2.5 / env.dt), None)
    assert np.allclose(traj.vx[late], 0.08, atol=1e-12)
    assert np.allclose(traj.vy[late], 0.0, atol=1e-12)
    env.close()
