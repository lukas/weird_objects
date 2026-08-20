"""Contract lock for the recover-mode DEPLOYMENT runner.

Guards the operator-ordered productionization of the recover champion
(cw-recover-predictive1b-pop3-s13, 2026-08-20 order): the runner's
observation assembly (rl_move/sim/recover_runner.py) must reproduce
the TRAINING env's recover observation bit-exactly — 16x90 stack,
newest-first, reset-history probe, LEVEL tilt reference, plant-
relative q, prev_action bookkeeping — because the policy was trained
on the env's obs and any drift is silent behavioral corruption.

The heavy artifacts (118 MB champion zip + 109 MB frozen encoder) are
exercised in the loader test only when present; obs parity runs on
pure env/builder code so it is always enforced.
"""
from __future__ import annotations

import numpy as np
import pytest

from rl_move.robot_state import DEG2RAD
from rl_move.sim.recover_runner import (
    CHAMPION_ZIP, CONTRACT_CFG, ENCODER_PT, FRAME_WIDTH, HISTORY,
    RecoverObsBuilder, RecoverRunner, contract_cfg, load_recover_policy)


def _make_env(seed=7, episode_seconds=4):
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = contract_cfg()
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed,
        render_mode=None, cfg=cfg)
    env._goal_gen.p_recover = 1.0
    for m in ("walk", "hold", "lean", "track", "unload", "raise",
              "rise", "lower", "quad"):
        if hasattr(env._goal_gen, f"p_{m}"):
            setattr(env._goal_gen, f"p_{m}", 0.0)
    return env


@pytest.mark.parametrize("kind", ["zero", "crouch_mid", "tangle_90"])
def test_obs_parity_reset_and_steps(kind):
    """Builder obs == env obs bit-exactly through reset probe + steps."""
    env = _make_env()
    env.force_recover_start = kind

    states = []
    orig_read = env._read_state

    def tee_read():
        s = orig_read()
        states.append(s)
        return s

    env._read_state = tee_read
    obs_env, _info = env.reset()
    assert obs_env.shape == (HISTORY * FRAME_WIDTH,)
    # reset reads: settle reads + the reset obs read + 15 probe reads.
    assert len(states) >= HISTORY

    builder = RecoverObsBuilder(cfg=env.cfg, plant_rad=env._plant_deg
                                * DEG2RAD)
    # The env's reset obs (frame 0) is built from the LAST pre-probe
    # state read; probe frames follow. Reproduce: begin() on the state
    # the reset obs used, then push the 15 probe states.
    reset_state = states[-(HISTORY - 1) - 1]
    # Deployment q_nom = the entry encoder read; the env captures its
    # nominal at the passive-equilibrium tick ~0.3 s (7-8 mrad) before
    # the first obs read. Parity passes the env's own q_nom; the
    # deployment gap is the settle drift, bounded here.
    assert float(np.max(np.abs(reset_state.joint_position
                               - env._q_nom))) < 0.03
    builder.begin(reset_state, tilt_bias=env._tilt_ref0,
                  q_nom=env._q_nom)
    for s in states[-(HISTORY - 1):]:
        builder.push(s)
    np.testing.assert_array_equal(builder.obs(), obs_env)

    # LEVEL tilt reference: recover spawns anchor tilt to gravity truth
    # (bias only). At DR-0 that reference must be ~zero even for a
    # tipped spawn.
    assert abs(env._tilt_ref0[0]) < 1e-9 and abs(env._tilt_ref0[1]) < 1e-9

    # Policy steps: identical obs when the builder sees the same states
    # and actions.
    rng = np.random.default_rng(0)
    for _ in range(5):
        a = rng.uniform(-1, 1, 18)
        obs_env, _r, term, trunc, _i = env.step(a)
        builder.note_action(a)
        builder.push(env._state)
        np.testing.assert_array_equal(builder.obs(), obs_env)
        if term or trunc:
            break
    env.close()


def test_runner_gating_and_entry_hold():
    """Manual command semantics: refuse while tumbling; 15-tick entry
    hold commands the captured pose with prev_action kept at zero."""
    env = _make_env(seed=11)
    env.force_recover_start = "zero"
    env.reset()
    state = env._state

    class _NoPolicy:
        def predict(self, obs, deterministic=True):
            return np.zeros(18), None

    runner = RecoverRunner(_NoPolicy(), cfg=env.cfg,
                           plant_rad=env._plant_deg * DEG2RAD)
    # tumbling state: gyro spike -> refused
    import copy
    tumbling = copy.copy(state)
    tumbling.imu_gyro = np.array([3.0, 0.0, 0.0])
    ok, why = runner.start(tumbling)
    assert not ok and "tumbling" in why

    for _ in range(runner.quiet_ticks):
        runner.observe_idle(state)
    ok, why = runner.start(state)
    assert ok, why
    assert runner.state == "entry"

    from rl_move.sim.joint_task import action_to_q_rad
    hold_q = np.clip(runner.builder.q_nom,
                     action_to_q_rad(-np.ones(18)),
                     action_to_q_rad(np.ones(18)))
    a = runner.entry_action()
    np.testing.assert_allclose(action_to_q_rad(a), hold_q, atol=1e-9)
    for k in range(HISTORY - 2):
        a = runner.tick(state)
        runner.note_action(a)
        # entry hold commands the captured pose (within clip)
        np.testing.assert_allclose(action_to_q_rad(a), hold_q, atol=1e-9)
        # prev_action must stay zero through the probe (training
        # contract: probe frames pay no action)
        assert not runner.builder.prev_action.any()
        assert runner.state == "entry"
    # the 15th pushed frame completes the training reset stack: the
    # SAME tick returns the first policy action (zero from _NoPolicy)
    a = runner.tick(state)
    runner.note_action(a)
    assert runner.state == "active"
    assert not a.any()
    # prev_action now tracks the policy action (zeros here, but the
    # bookkeeping path is the note_action one)
    runner.note_action(np.full(18, 0.5))
    assert runner.builder.prev_action[0] == 0.5
    env.close()


@pytest.mark.skipif(not (CHAMPION_ZIP.is_file() and ENCODER_PT.is_file()),
                    reason="champion artifacts not staged on this host")
def test_loader_contract():
    """The packaged pair loads relocatably and matches the contract."""
    model = load_recover_policy()
    assert int(np.prod(model.observation_space.shape)) \
        == HISTORY * FRAME_WIDTH
    assert int(np.prod(model.action_space.shape)) == 18
    p = model.policy
    assert type(p).__name__ == "PredictiveCriticPolicy"
    assert p.actor_residual_enabled
    a, _ = model.predict(
        np.zeros(HISTORY * FRAME_WIDTH, dtype=np.float32),
        deterministic=True)
    assert a.shape == (18,) and np.all(np.isfinite(a))
