"""dr.fault_* per-joint fault injection (AMP brief §8; M0 checklist
"fault injection works").

Contract under test:
- default OFF: fault_prob=0.0 draws no fault, keeps the legacy rng
  stream bit-exact, and apply_fault_to_model is a byte-level no-op;
- weakened joint: kp/torque rows of exactly the faulted position
  actuator scale by the drawn strength;
- frozen joint: actuator force zeroed + FROZEN_DOF_DAMPING lock on
  exactly that DOF;
- disabled leg: all 3 joints of one leg at strength 0;
- every touched MjModel field is in mjx_backend.MODEL_DR_FIELDS (the
  per-world upload guarantee that makes this work on the GPU stacks);
- health vector per brief §8.2;
- end-to-end: HexapodSimEnv with dr.fault_prob=1.0 resets and steps
  finite with the fault visible in its model.
"""
import dataclasses

import numpy as np
import pytest

from rl_move.sim.domain_rand import (
    FROZEN_DOF_DAMPING, DomainRandomizer, EpisodeRandomization, N_LEGS,
    RandRanges)
from rl_move.sim.servo_model import (
    N_JOINTS, SimServoParams, _act_id, apply_params_to_model, build_model,
    joint_names, joint_qvel_addrs)


def _fresh_model():
    model = build_model()
    apply_params_to_model(model, SimServoParams.load())
    return model


def _snapshot(model):
    return {f: getattr(model, f).copy()
            for f in ("actuator_gainprm", "actuator_biasprm",
                      "actuator_forcerange", "dof_damping")}


def _er(**kw) -> EpisodeRandomization:
    """A neutral EpisodeRandomization with only fault fields set."""
    base = DomainRandomizer(scale=0.0).sample(np.random.default_rng(0))
    return dataclasses.replace(base, **kw)


# ---------------------------------------------------------------- default off

def test_fault_prob_zero_draws_nothing_and_keeps_rng_stream():
    # Same seed, fault axis off vs off-with-different-dose-menu: every
    # sampled field must be identical (the dose menu must not consume
    # rng when the axis is off — the guarded-draw convention).
    r1 = RandRanges()
    r2 = RandRanges(fault_weak_scales=(0.5,), fault_mix=(1.0, 0.0, 0.0))
    s1 = DomainRandomizer(r1).sample(np.random.default_rng(7))
    s2 = DomainRandomizer(r2).sample(np.random.default_rng(7))
    assert s1.fault_mode == "" and s2.fault_mode == ""
    assert s1.fault_joints == () and s1.fault_scale == 1.0
    assert s1.summary() == s2.summary()
    np.testing.assert_array_equal(s1.kp_scale, s2.kp_scale)
    np.testing.assert_array_equal(s1.start_offset_rad, s2.start_offset_rad)


def test_apply_fault_noop_when_healthy():
    model = _fresh_model()
    before = _snapshot(model)
    _er().apply_fault_to_model(model)
    after = _snapshot(model)
    for k in before:
        np.testing.assert_array_equal(before[k], after[k])


# ------------------------------------------------------------------ fault ops

def test_weak_joint_scales_only_its_actuator_rows():
    model = _fresh_model()
    before = _snapshot(model)
    j, s = 4, 0.4
    _er(fault_mode="weak", fault_joints=(j,), fault_scale=s
        ).apply_fault_to_model(model)
    names = joint_names()
    pa = _act_id(model, names[j])
    va = _act_id(model, names[j] + "_d")
    assert model.actuator_gainprm[pa, 0] == pytest.approx(
        before["actuator_gainprm"][pa, 0] * s)
    assert model.actuator_biasprm[pa, 1] == pytest.approx(
        before["actuator_biasprm"][pa, 1] * s)
    np.testing.assert_allclose(model.actuator_forcerange[pa],
                               before["actuator_forcerange"][pa] * s)
    np.testing.assert_allclose(model.actuator_forcerange[va],
                               before["actuator_forcerange"][va] * s)
    # every OTHER row untouched
    mask = np.ones(model.nu, bool)
    mask[[pa, va]] = False
    np.testing.assert_array_equal(model.actuator_gainprm[mask],
                                  before["actuator_gainprm"][mask])
    np.testing.assert_array_equal(model.actuator_forcerange[mask],
                                  before["actuator_forcerange"][mask])
    np.testing.assert_array_equal(model.dof_damping, before["dof_damping"])


def test_frozen_joint_zeroes_force_and_locks_dof():
    model = _fresh_model()
    before = _snapshot(model)
    j = 10
    _er(fault_mode="frozen", fault_joints=(j,)).apply_fault_to_model(model)
    names = joint_names()
    pa = _act_id(model, names[j])
    va = _act_id(model, names[j] + "_d")
    np.testing.assert_array_equal(model.actuator_forcerange[pa], [0.0, 0.0])
    np.testing.assert_array_equal(model.actuator_forcerange[va], [0.0, 0.0])
    dadr = joint_qvel_addrs(model)
    assert model.dof_damping[dadr[j]] == FROZEN_DOF_DAMPING
    mask = np.ones(model.nv, bool)
    mask[dadr[j]] = False
    np.testing.assert_array_equal(model.dof_damping[mask],
                                  before["dof_damping"][mask])
    # gain rows deliberately untouched (force clamp is the kill switch)
    np.testing.assert_array_equal(model.actuator_gainprm,
                                  before["actuator_gainprm"])


def test_disabled_leg_kills_three_joints():
    model = _fresh_model()
    leg = 2
    joints = (3 * leg, 3 * leg + 1, 3 * leg + 2)
    _er(fault_mode="leg", fault_joints=joints, fault_scale=0.0
        ).apply_fault_to_model(model)
    names = joint_names()
    for j in joints:
        pa = _act_id(model, names[j])
        assert model.actuator_gainprm[pa, 0] == 0.0
        np.testing.assert_array_equal(model.actuator_forcerange[pa],
                                      [0.0, 0.0])
    # neighbors alive
    pa_ok = _act_id(model, names[3 * leg - 1])
    assert model.actuator_gainprm[pa_ok, 0] != 0.0


def test_touched_fields_are_all_per_world_dr_fields():
    from rl_move.sim.mjx_backend import MODEL_DR_FIELDS
    for f in ("actuator_gainprm", "actuator_biasprm",
              "actuator_forcerange", "dof_damping"):
        assert f in MODEL_DR_FIELDS


# ------------------------------------------------------------------- sampling

def test_sample_draws_all_modes_with_valid_joints():
    r = RandRanges(fault_prob=1.0)
    dr = DomainRandomizer(r)
    rng = np.random.default_rng(3)
    seen = set()
    for _ in range(300):
        er = dr.sample(rng)
        assert er.fault_mode in ("weak", "frozen", "leg")
        seen.add(er.fault_mode)
        assert all(0 <= j < N_JOINTS for j in er.fault_joints)
        if er.fault_mode == "leg":
            assert len(er.fault_joints) == 3
            assert er.fault_joints[0] % 3 == 0
            assert er.fault_scale == 0.0
        elif er.fault_mode == "weak":
            assert er.fault_scale in r.fault_weak_scales
        else:
            assert len(er.fault_joints) == 1
        assert "fault" in er.summary() and er.summary()["fault"] != "none"
    assert seen == {"weak", "frozen", "leg"}


def test_scaled_shrinks_probability_not_dose():
    r = RandRanges(fault_prob=0.8, fault_weak_scales=(0.2,))
    r2 = r.scaled(0.5)
    assert r2.fault_prob == pytest.approx(0.4)
    assert r2.fault_weak_scales == (0.2,)
    assert r2.fault_mix == r.fault_mix


def test_fault_health_vector():
    h = _er().fault_health()
    np.testing.assert_array_equal(h, np.ones(N_JOINTS, np.float32))
    h = _er(fault_mode="weak", fault_joints=(5,),
            fault_scale=0.4).fault_health()
    assert h[5] == pytest.approx(0.4) and h.sum() == pytest.approx(17.4)
    h = _er(fault_mode="frozen", fault_joints=(2,)).fault_health()
    assert h[2] == 0.0
    h = _er(fault_mode="leg", fault_joints=(6, 7, 8),
            fault_scale=0.0).fault_health()
    np.testing.assert_array_equal(h[6:9], [0.0, 0.0, 0.0])


# ----------------------------------------------------------------- end to end

def test_env_reset_applies_fault_and_steps_finite():
    from rl_move.sim.sim_env import SimHexapodBalanceEnv
    env = SimHexapodBalanceEnv(randomize=True, seed=11,
                               cfg={"dr": {"fault_prob": 1.0}})
    try:
        obs, _ = env.reset()
        er = env._ep_rand
        assert er is not None and er.fault_mode in ("weak", "frozen", "leg")
        names = joint_names()
        j = er.fault_joints[0]
        pa = _act_id(env.model, names[j])
        if er.fault_mode == "frozen":
            np.testing.assert_array_equal(
                env.model.actuator_forcerange[pa], [0.0, 0.0])
        else:
            healthy = _fresh_model()
            assert (abs(env.model.actuator_forcerange[pa][1])
                    < abs(healthy.actuator_forcerange[pa][1]))
        for _ in range(5):
            obs, rew, term, trunc, info = env.step(
                np.zeros(env.n_act, dtype=np.float32))
            assert np.all(np.isfinite(obs)) and np.isfinite(rew)
        # a healthy-config env must NOT carry fault state
        env2 = SimHexapodBalanceEnv(randomize=True, seed=11, cfg={})
        env2.reset()
        assert env2._ep_rand.fault_mode == ""
        env2.close()
    finally:
        env.close()
