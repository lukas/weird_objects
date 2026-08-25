"""Model-source plumbing: the mesh-accurate family vs the legacy primitive.

The calibrated behavior suite pins itself to ``primitive`` via conftest.py
(those tests encode legacy-robot dynamics).  These tests exercise the
2026-08-24 mesh family explicitly: resolution precedence, both variants
building and standing at the plant keyframe, the as-built ~3.5 kg mass
correction, and a full env reset on the MJX twin.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

mujoco = pytest.importorskip("mujoco")

from rl_move.sim import servo_model as SM  # noqa: E402
from rl_move.sim import eval_checkpoint as EC  # noqa: E402


def _total_mass(model) -> float:
    return float(model.body_mass.sum())


class _TinyModel:
    def __init__(self, *, nmesh: int):
        self.nmesh = nmesh
        self.ngeom = 12
        self.body_mass = np.array([1.25, 2.25])


class _TinyEnv:
    def __init__(self, *, source: str, nmesh: int):
        self._model_source = source
        self.model = _TinyModel(nmesh=nmesh)


def test_eval_report_model_identity_distinguishes_mesh_variants():
    full = EC.model_identity(_TinyEnv(source="mesh", nmesh=4))
    assert full["model_source"] == "mesh"
    assert full["model_variant"] == "full_mesh"
    assert full["model_mass_kg"] == 3.5

    twin = EC.model_identity(_TinyEnv(source="mesh", nmesh=0))
    assert twin["model_variant"] == "mesh_mjx_twin"

    primitive = EC.model_identity(_TinyEnv(source="primitive", nmesh=0))
    assert primitive["model_variant"] == "legacy_primitive"


def test_resolver_env_var_beats_cfg(monkeypatch):
    monkeypatch.setenv("HEXAPOD_MODEL_SOURCE", "primitive")
    assert SM.resolve_model_source(
        {"env": {"model_source": "mesh"}}) == "primitive"
    monkeypatch.setenv("HEXAPOD_MODEL_SOURCE", "mesh_mjx")
    assert SM.resolve_model_source(
        {"env": {"model_source": "primitive"}}) == "mesh_mjx"
    monkeypatch.setenv("HEXAPOD_MODEL_SOURCE", "bogus")
    with pytest.raises(ValueError):
        SM.resolve_model_source({})


def test_resolver_cfg_and_default(monkeypatch):
    monkeypatch.delenv("HEXAPOD_MODEL_SOURCE", raising=False)
    assert SM.resolve_model_source({}) == "mesh"
    assert SM.resolve_model_source(
        {"env": {"model_source": "primitive"}}) == "primitive"
    with pytest.raises(ValueError):
        SM.resolve_model_source({"env": {"model_source": "hulls"}})


def test_mesh_mjx_twin_builds_and_plants(monkeypatch):
    model = SM.build_model(source="mesh_mjx")
    # the as-built mass correction rides with the mesh family
    assert 3.2 < _total_mass(model) < 3.8, _total_mass(model)
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "plant")
    assert key >= 0, "plant keyframe missing from mesh_mjx twin"
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    low = SM.lowest_collidable_z(model, data)
    assert -0.001 < low < 0.01, f"plant keyframe floats/penetrates: {low}"


def test_mesh_full_matches_twin_masses():
    if not (SM.MESH_XML.exists() and (SM.MESH_DIR / "assets").is_dir()):
        pytest.skip("full-mesh assets not generated on this machine")
    m_full = SM.build_model(source="mesh")
    m_twin = SM.build_model(source="mesh_mjx")
    assert abs(_total_mass(m_full) - _total_mass(m_twin)) < 1e-6
    assert m_full.nq == m_twin.nq
    assert m_full.nu == m_twin.nu


def test_legacy_primitive_family_unchanged():
    model = SM.build_model(source="primitive")
    # bit-continuity for pre-08-24 checkpoint lineages: legacy budgets stand
    assert abs(_total_mass(model) - 2.104) < 0.02, _total_mass(model)


def test_env_reset_and_step_on_mesh_twin(monkeypatch):
    monkeypatch.setenv("HEXAPOD_MODEL_SOURCE", "mesh_mjx")
    from rl_move.sim.sim_env import SimHexapodBalanceEnv
    env = SimHexapodBalanceEnv(seed=0)
    assert _total_mass(env.model) > 3.0  # mesh family actually loaded
    obs, _ = env.reset()
    assert np.isfinite(obs).all()
    obs2, _, _, _, _ = env.step(np.zeros(env.action_space.shape,
                                         dtype=env.action_space.dtype))
    assert np.isfinite(obs2).all()
