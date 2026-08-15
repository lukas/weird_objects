"""struct_compliance unit semantics (RISE_WALK_NEXT_48H P3 hook).

The module must (a) refuse to run without measured stiffnesses — the
directive forbids invented values — and (b) implement the quasi-static
series-spring math exactly.

Run: python3 -m pytest rl_move/tests/test_struct_compliance.py -q
"""
from __future__ import annotations

import numpy as np
import pytest

from rl_move.sim.struct_compliance import N_JOINTS, StructCompliance


K = {"yaw": 400.0, "hip": 120.0, "knee": 80.0}   # test-only numbers


def test_disabled_returns_none():
    assert StructCompliance.from_cfg({}) is None
    assert StructCompliance.from_cfg(
        {"struct_comp": {"enabled": 0.0}}) is None


def test_enabled_without_measurements_refuses():
    with pytest.raises(ValueError, match="COMPLIANCE.md"):
        StructCompliance.from_cfg({"struct_comp": {"enabled": 1.0}})
    with pytest.raises(ValueError, match="k_knee"):
        StructCompliance.from_cfg({"struct_comp": {
            "enabled": 1.0, "k_yaw_nm_rad": 400.0,
            "k_hip_nm_rad": 120.0}})
    with pytest.raises(ValueError):
        StructCompliance({"yaw": 400.0, "hip": 0.0, "knee": 80.0})


def test_deflection_and_reported_pose():
    comp = StructCompliance(K)
    tau = np.zeros(N_JOINTS)
    tau[2] = 4.0                       # knee joint of leg 0, 4 N·m
    d = comp.deflection(tau)
    assert d[2] == pytest.approx(4.0 / 80.0)
    assert np.all(d[np.arange(N_JOINTS) != 2] == 0.0)
    q_phys = np.full(N_JOINTS, 0.5)
    rep = comp.reported_q(q_phys, tau)
    # actual = reported + deflection: the encoder under-reads the
    # loaded knee by tau/k.
    assert rep[2] == pytest.approx(0.5 - 4.0 / 80.0)
    assert rep[0] == pytest.approx(0.5)


def test_effective_kp_series_limit():
    comp = StructCompliance(K)
    kp = np.full(N_JOINTS, 120.0)
    eff = comp.effective_kp(kp)
    # knee: 120*80/200 = 48; series combo is below both members
    assert eff[2] == pytest.approx(48.0)
    assert np.all(eff < kp)
    assert np.all(eff < comp.k + 1e-9)
    # a very stiff structure changes nothing measurable
    stiff = StructCompliance({a: 1e9 for a in K})
    assert stiff.effective_kp(kp) == pytest.approx(kp, rel=1e-6)


def test_dr_sample_stays_in_band_and_is_axis_shared():
    comp = StructCompliance(K, dr_lo=0.7, dr_hi=1.5)
    rng = np.random.default_rng(0)
    for _ in range(10):
        comp.sample(rng)
        scale = comp.k / comp._k_base
        assert np.all(scale >= 0.7 - 1e-12)
        assert np.all(scale <= 1.5 + 1e-12)
        # one scale per axis class, shared across the six legs
        for ax_i in range(3):
            s = scale[ax_i::3]
            assert np.allclose(s, s[0])
