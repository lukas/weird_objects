"""Value-learning audit + PPO update-path protection tests (08-17).

Operator-approved directive fb_20260817T005114_775298 after
cw-arch-joystick-long-scratch3 finished 40M with critic explained
variance ~0 throughout and a catastrophic post-14.7M collapse driven
by approx_kl~0.24 updates that target_kl=0.02 could not undo.

Bank contract:

1. FROZEN-ROLLOUT VALUE REGRESSION: both the MLP and the
   causal-transformer critic must fit a frozen batch of (obs, return)
   pairs to clearly positive explained variance in a few hundred Adam
   steps. PASS here + EV~0 in training = reward-scale/credit
   difficulty (e.g. the unbounded -730 terminal cliff), NOT a critic
   implementation bug. FAIL here = implementation bug — fix before
   any launch.
2. Actor/critic param-group split covers every trainable parameter
   exactly once, and the split puts the value trunk/head in the
   critic group.
3. attach_actor_critic_lr: linear actor decay by progress, constant
   critic LR, and SB3's _update_learning_rate override is in place.
4. attach_kl_rollback: an overshooting update is rolled back
   bit-exact and the actor-LR scale is cut; a healthy update is
   untouched.
5. HealthTracker: best-checkpoint marks on composite improvement;
   the stop fires ONLY on joint (reward+survival+direction)
   regression sustained for regress_n assays. EVTracker: EV~0 past
   the deadline is a hard failure.
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

import numpy as np
import pytest
import torch as th

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from gymnasium import spaces  # noqa: E402

from rl_move.sim.update_health import (  # noqa: E402
    EVTracker, HealthTracker, attach_actor_critic_lr,
    attach_kl_rollback, load_optimizer_state_if_compatible,
    split_actor_critic_params,
)

ACT_DIM = 4


def _lr_sched(_progress: float) -> float:
    return 3e-4


def _mlp_policy(obs_dim: int = 16):
    from stable_baselines3.common.policies import ActorCriticPolicy
    obs_sp = spaces.Box(-np.inf, np.inf, (obs_dim,), np.float32)
    act_sp = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
    return ActorCriticPolicy(obs_sp, act_sp, _lr_sched,
                             net_arch=[32, 32])


def _tf_policy(n_frames: int = 4, frame_w: int = 8):
    from rl_move.sim.transformer_policy import TransformerActorCriticPolicy
    obs_sp = spaces.Box(-np.inf, np.inf, (n_frames * frame_w,),
                        np.float32)
    act_sp = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
    return TransformerActorCriticPolicy(
        obs_sp, act_sp, _lr_sched, n_frames=n_frames, d_model=32,
        n_layers=1, n_heads=2, ff_dim=64, net_arch=[32])


def _explained_variance(pred: np.ndarray, y: np.ndarray) -> float:
    return 1.0 - float(np.var(y - pred)) / max(float(np.var(y)), 1e-8)


def _fit_critic(policy, obs: th.Tensor, returns: th.Tensor,
                steps: int = 400, lr: float = 3e-3) -> float:
    _actor, critic = split_actor_critic_params(policy)
    opt = th.optim.Adam(critic, lr=lr)
    for _ in range(steps):
        opt.zero_grad()
        pred = policy.predict_values(obs).squeeze(-1)
        loss = th.mean((pred - returns) ** 2)
        loss.backward()
        opt.step()
    with th.no_grad():
        pred = policy.predict_values(obs).squeeze(-1).numpy()
    return _explained_variance(pred, returns.numpy())


def test_frozen_rollout_value_regression_mlp():
    th.manual_seed(0)
    policy = _mlp_policy()
    obs = th.randn(512, 16)
    w = th.randn(16)
    returns = obs @ w + 0.05 * th.randn(512)
    ev = _fit_critic(policy, obs, returns)
    assert ev > 0.5, (
        f"MLP critic cannot fit a frozen linear-return rollout "
        f"(EV={ev:.3f}) — value-learning implementation bug")


def test_frozen_rollout_value_regression_transformer():
    """The scratch3 architecture's critic path, isolated: a small
    causal-transformer critic must fit frozen returns that depend on
    BOTH the newest and an older frame (uses the history)."""
    th.manual_seed(0)
    policy = _tf_policy()
    obs = th.randn(512, 4 * 8)
    frames = obs.view(512, 4, 8)   # newest-first
    returns = (frames[:, 0].sum(dim=1) - 0.5 * frames[:, 2].sum(dim=1)
               + 0.05 * th.randn(512))
    ev = _fit_critic(policy, obs, returns, steps=600)
    assert ev > 0.5, (
        f"transformer critic cannot fit a frozen rollout "
        f"(EV={ev:.3f}) — value-learning implementation bug in the "
        "scratch3 architecture path")


def test_param_split_disjoint_and_complete():
    for policy in (_mlp_policy(), _tf_policy()):
        actor, critic = split_actor_critic_params(policy)
        ids_a = {id(p) for p in actor}
        ids_c = {id(p) for p in critic}
        assert not (ids_a & ids_c)
        all_ids = {id(p) for p in policy.parameters()
                   if p.requires_grad}
        assert ids_a | ids_c == all_ids
        # the value head must be critic-side, log_std actor-side
        assert id(policy.value_net.weight) in ids_c
        assert id(policy.log_std) in ids_a


def test_transformer_split_puts_vf_extractor_in_critic():
    policy = _tf_policy()
    _actor, critic = split_actor_critic_params(policy)
    ids_c = {id(p) for p in critic}
    vf_ids = {id(p) for p in policy.vf_features_extractor.parameters()}
    assert vf_ids <= ids_c, "unshared critic transformer trunk must " \
                            "ride the critic LR group"


class _StubLogger:
    def __init__(self):
        self.name_to_value = {}

    def record(self, key, value):
        self.name_to_value[key] = value


def _stub_model(policy):
    m = types.SimpleNamespace()
    m.policy = policy
    m.logger = _StubLogger()
    m._current_progress_remaining = 1.0
    return m


def test_attach_actor_critic_lr_decay_and_constant_critic():
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=1e-4, actor_lr_final=1e-5,
                           critic_lr=3e-4)
    opt = policy.optimizer
    assert len(opt.param_groups) == 2
    m._update_learning_rate(opt)          # progress 1.0 (run start)
    assert opt.param_groups[0]["lr"] == pytest.approx(1e-4)
    assert opt.param_groups[1]["lr"] == pytest.approx(3e-4)
    m._current_progress_remaining = 0.0   # run end
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(1e-5)
    assert opt.param_groups[1]["lr"] == pytest.approx(3e-4)
    assert m.logger.name_to_value["train/actor_lr"] == pytest.approx(1e-5)


def _perturbing_train(model, kl: float):
    def train():
        with th.no_grad():
            for p in model.policy.parameters():
                p.add_(1.0)
        model.logger.record("train/approx_kl", kl)
    return train


def test_kl_rollback_restores_params_and_cuts_lr():
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=1e-4)
    before = {k: v.clone() for k, v in policy.state_dict().items()}
    m.train = _perturbing_train(m, kl=0.10)
    attach_kl_rollback(m, threshold=0.03, lr_factor=0.5)
    m.train()
    after = policy.state_dict()
    for k in before:
        assert th.equal(before[k], after[k]), f"{k} not rolled back"
    assert m._ac_state["actor_scale"] == pytest.approx(0.5)
    assert m._kl_rollback_count == 1
    assert m.logger.name_to_value["train/kl_rollback"] == 1
    m.train()   # second overshoot halves again
    assert m._ac_state["actor_scale"] == pytest.approx(0.25)


def test_kl_rollback_healthy_update_untouched():
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=1e-4)
    before = {k: v.clone() for k, v in policy.state_dict().items()}
    m.train = _perturbing_train(m, kl=0.005)
    attach_kl_rollback(m, threshold=0.03)
    m.train()
    after = policy.state_dict()
    changed = any(not th.equal(before[k], after[k]) for k in before)
    assert changed, "healthy update must be kept"
    assert m._ac_state["actor_scale"] == pytest.approx(1.0)
    assert m._kl_rollback_count == 0


def test_kl_rollback_requires_param_groups():
    policy = _mlp_policy()
    m = _stub_model(policy)
    m.train = lambda: None
    with pytest.raises(ValueError):
        attach_kl_rollback(m, threshold=0.03)


def test_health_tracker_best_and_joint_regression_stop():
    t = HealthTracker(regress_n=2)
    r = t.feed(1.0, 10.0, 0.8, -2.0)
    assert r["is_best"] and not r["should_stop"]
    # single-metric wobble: survival dips alone -> no streak
    r = t.feed(0.5, 12.0, 0.7, -2.1)
    assert not r["regressed"]
    # joint regression twice -> stop
    r = t.feed(0.3, 90.0, 0.0, -8.0)
    assert r["regressed"] and not r["should_stop"]
    r = t.feed(0.2, 120.0, -0.5, -10.0)
    assert r["should_stop"]


def test_health_tracker_improvement_resets_streak():
    t = HealthTracker(regress_n=2)
    t.feed(1.0, 10.0, 0.8, -2.0)
    t.feed(0.3, 90.0, 0.0, -8.0)
    t.feed(0.9, 15.0, 0.7, -2.5)   # recovery
    r = t.feed(0.3, 90.0, 0.0, -8.0)
    assert not r["should_stop"]


def test_ev_tracker_hard_failure_semantics():
    ev = EVTracker(ev_min=0.05, deadline_steps=1_000_000)
    ev.feed(0.0)
    assert not ev.failed(500_000)          # before deadline: no stop
    assert ev.failed(1_200_000)            # pinned at 0 past deadline
    good = EVTracker(ev_min=0.05, deadline_steps=1_000_000)
    for _ in range(20):
        good.feed(0.4)
    assert not good.failed(2_000_000)


def test_attachments_survive_save_load_roundtrip(tmp_path):
    """Real SB3 PPO: both attachments + save() + PPO.load() must
    round-trip (the wrappers close over the model and would drag the
    env into the pickle without the save exclusions)."""
    import gymnasium as gym
    from stable_baselines3 import PPO

    env = gym.make("Pendulum-v1")
    model = PPO("MlpPolicy", env, n_steps=8, batch_size=8,
                n_epochs=1, seed=0, device="cpu")
    attach_actor_critic_lr(model, actor_lr=1e-4, actor_lr_final=1e-5,
                           critic_lr=3e-4)
    attach_kl_rollback(model, threshold=0.03)
    model.learn(total_timesteps=16)
    p = tmp_path / "roundtrip.zip"
    model.save(p)
    loaded = PPO.load(p, device="cpu")
    # stock class methods restored on load (re-attach is the trainer's
    # job on warm start)
    assert "train" not in loaded.__dict__
    assert "_update_learning_rate" not in loaded.__dict__
    assert "save" not in loaded.__dict__
    assert len(loaded.policy.optimizer.param_groups) == 1
    # the LIVE model keeps its 2-group optimizer after a save
    assert len(model.policy.optimizer.param_groups) == 2
    obs = np.zeros(env.observation_space.shape, dtype=np.float32)
    loaded.predict(obs, deterministic=True)
    env.close()


# --- actor-freeze window (operator order fb_20260818T102844_116d4c) --

def test_actor_freeze_window_zeroes_then_releases_actor_lr():
    from rl_move.sim.update_health import set_actor_freeze
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=5e-5, actor_lr_final=5e-5,
                           critic_lr=3e-4)
    set_actor_freeze(m, 100)
    opt = policy.optimizer
    m.num_timesteps = 0
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == 0.0          # actor frozen
    assert opt.param_groups[1]["lr"] == pytest.approx(3e-4)  # critic on
    assert m.logger.name_to_value["train/actor_frozen"] == 1.0
    m.num_timesteps = 99
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == 0.0
    m.num_timesteps = 100                            # boundary: release
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(5e-5)
    assert m.logger.name_to_value["train/actor_frozen"] == 0.0
    assert m.logger.name_to_value["train/actor_lr"] == pytest.approx(5e-5)


def test_actor_freeze_off_is_bit_exact():
    """Without set_actor_freeze the LR path and the logged keys are
    exactly the pre-change behavior (no freeze_until key consulted, no
    actor_frozen key logged)."""
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=1e-4, actor_lr_final=1e-5,
                           critic_lr=3e-4)
    m.num_timesteps = 0
    opt = policy.optimizer
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(1e-4)
    assert "train/actor_frozen" not in m.logger.name_to_value
    # explicit 0 disarms too
    from rl_move.sim.update_health import set_actor_freeze
    set_actor_freeze(m, 0)
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(1e-4)
    assert "train/actor_frozen" not in m.logger.name_to_value


def test_set_actor_freeze_requires_attached_groups():
    policy = _mlp_policy()
    m = _stub_model(policy)
    from rl_move.sim.update_health import set_actor_freeze
    with pytest.raises(ValueError):
        set_actor_freeze(m, 100)


def test_actor_freeze_composes_with_kl_rollback_scale():
    """A rollback-cut actor_scale must not resurrect a frozen actor,
    and must still apply after release."""
    from rl_move.sim.update_health import set_actor_freeze
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=1e-4)
    set_actor_freeze(m, 100)
    m._ac_state["actor_scale"] = 0.5
    opt = policy.optimizer
    m.num_timesteps = 50
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == 0.0
    m.num_timesteps = 200
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(5e-5)


# --- load_optimizer_state_if_compatible (bridge1 rollback crash fix) -

def test_load_optimizer_state_matching_groups_loads():
    """Same group count (the common case: no --actor-lr, or a
    checkpoint saved by the SAME live optimizer shape) loads exactly
    as a plain load_state_dict would."""
    policy = _mlp_policy()
    opt = policy.optimizer   # single stock group
    saved = opt.state_dict()
    # perturb live lr so we can tell the load actually happened
    opt.param_groups[0]["lr"] = 0.999
    ok = load_optimizer_state_if_compatible(opt, saved)
    assert ok is True
    assert opt.param_groups[0]["lr"] == saved["param_groups"][0]["lr"]


def test_load_optimizer_state_group_mismatch_skips_not_crashes():
    """The bridge1 crash: a save_stock_optimizer checkpoint (1 group)
    reloaded into the live 2-group actor/critic optimizer must SKIP,
    not raise torch's 'different number of parameter groups' error,
    and must leave the live optimizer's groups/lrs untouched."""
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=5e-5, critic_lr=3e-4)
    opt = policy.optimizer
    assert len(opt.param_groups) == 2
    before_lrs = [g["lr"] for g in opt.param_groups]
    # a single-group "stock" state dict, as save_stock_optimizer writes
    stock_opt = th.optim.Adam(policy.parameters(), lr=1e-2)
    saved = stock_opt.state_dict()
    assert len(saved["param_groups"]) == 1
    ok = load_optimizer_state_if_compatible(opt, saved,
                                           context="walkcurr rollback")
    assert ok is False
    assert len(opt.param_groups) == 2   # untouched, no crash
    assert [g["lr"] for g in opt.param_groups] == before_lrs


def test_rollback_reload_survives_frozen_and_unfrozen_phases():
    """Operator order fb 20260818T111051Z (bridge1 recovery): the
    walkcurr rollback reload must be safe in BOTH actor phases. A
    stock 1-group checkpoint reloaded into the live 2-group optimizer
    DURING the freeze window must skip without crashing and leave the
    actor group at lr=0 after the next _update_learning_rate; the same
    reload AFTER release must leave the scheduled actor lr intact."""
    from rl_move.sim.update_health import set_actor_freeze
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=5e-5, critic_lr=3e-4)
    set_actor_freeze(m, 100)
    opt = policy.optimizer
    stock = th.optim.Adam(policy.parameters(), lr=1e-2).state_dict()
    # frozen phase: reload skips, freeze semantics keep actor lr 0
    m.num_timesteps = 50
    ok = load_optimizer_state_if_compatible(opt, stock,
                                            context="frozen rollback")
    assert ok is False
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == 0.0
    assert opt.param_groups[1]["lr"] == pytest.approx(3e-4)
    # unfrozen phase: reload skips, scheduled actor lr restored
    m.num_timesteps = 200
    ok = load_optimizer_state_if_compatible(opt, stock,
                                            context="released rollback")
    assert ok is False
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(5e-5)
    assert opt.param_groups[1]["lr"] == pytest.approx(3e-4)


# --- bridge2 mechanisms (fb_20260818T112826_9ed832, default-off) ----

def test_restore_actor_only_leaves_critic_and_its_moments():
    """Actor-only curriculum rollback: actor tensors restore to the
    snapshot, critic tensors and the critic group's Adam moments are
    untouched, actor moments reset."""
    from rl_move.sim.update_health import restore_actor_only_from_state
    th.manual_seed(3)
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=5e-5, critic_lr=3e-4)
    opt = policy.optimizer
    # populate Adam moments in BOTH groups with one real step
    loss = sum((p ** 2).sum() for g in opt.param_groups
               for p in g["params"])
    loss.backward()
    opt.step()
    assert all(p in opt.state for g in opt.param_groups
               for p in g["params"])
    snap = {k: v.detach().clone()
            for k, v in policy.state_dict().items()}
    # drift every tensor
    with th.no_grad():
        for p in policy.parameters():
            p.add_(0.1)
    critic_drifted = {k: v.detach().clone()
                      for k, v in policy.state_dict().items()
                      if any(mk in k for mk in
                             ("value_net", "vf_features_extractor"))}
    critic_moments_before = [
        {kk: vv.detach().clone() if th.is_tensor(vv) else vv
         for kk, vv in opt.state[p].items()}
        for p in opt.param_groups[1]["params"]]
    n, ok = restore_actor_only_from_state(policy, snap, optimizer=opt)
    assert ok is True and n > 0
    after = policy.state_dict()
    for k in after:
        if k in critic_drifted:      # critic stays at drifted values
            assert th.equal(after[k], critic_drifted[k]), k
        elif k in snap:              # actor back at the snapshot
            assert th.allclose(after[k], snap[k]), k
    # actor moments cleared, critic moments preserved by value
    assert all(p not in opt.state
               for p in opt.param_groups[0]["params"])
    for p, before in zip(opt.param_groups[1]["params"],
                         critic_moments_before):
        assert p in opt.state
        for kk, vv in before.items():
            cur = opt.state[p][kk]
            if th.is_tensor(vv):
                assert th.equal(cur, vv)
            else:
                assert cur == vv


def test_ev_readiness_holds_freeze_releases_and_fails_closed():
    """The critic-EV readiness gate: actor stays frozen past the step
    floor until EV holds the threshold for N consecutive updates;
    releases after; a never-ready critic aborts at the cap."""
    from rl_move.sim.update_health import (
        attach_ev_readiness_release, set_actor_freeze)
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=5e-5, critic_lr=3e-4)
    set_actor_freeze(m, 100)
    m.num_timesteps = 0
    m.train = lambda: None
    attach_ev_readiness_release(m, ev_threshold=0.2, ev_windows=2,
                                max_steps=1000)
    opt = policy.optimizer

    def _update(ev, steps):
        m.num_timesteps = steps
        m.logger.name_to_value["train/explained_variance"] = ev
        m.train()
        m._update_learning_rate(opt)
        return opt.param_groups[0]["lr"]

    # past the 100-step floor but EV low -> still frozen
    assert _update(0.05, 200) == 0.0
    # one good window is not enough (needs 2 consecutive)
    assert _update(0.30, 300) == 0.0
    # a dip resets the streak
    assert _update(0.10, 400) == 0.0
    assert _update(0.25, 500) == 0.0
    # second consecutive good window -> released at scheduled lr
    assert _update(0.28, 600) == pytest.approx(5e-5)
    # stays released
    assert _update(-1.0, 700) == pytest.approx(5e-5)
    # fail-closed: a fresh armed model that never becomes ready
    p2 = _mlp_policy()
    m2 = _stub_model(p2)
    attach_actor_critic_lr(m2, actor_lr=5e-5, critic_lr=3e-4)
    set_actor_freeze(m2, 100)
    m2.num_timesteps = 0
    m2.train = lambda: None
    attach_ev_readiness_release(m2, ev_threshold=0.2, ev_windows=2,
                                max_steps=1000)
    m2.logger.name_to_value["train/explained_variance"] = 0.0
    m2.num_timesteps = 1200
    with pytest.raises(RuntimeError, match="FAIL-CLOSED"):
        m2.train()


def test_ev_readiness_off_is_bit_exact_fixed_step_freeze():
    """Without attach_ev_readiness_release the freeze window behaves
    exactly as before (freeze_ready defaults True)."""
    from rl_move.sim.update_health import set_actor_freeze
    policy = _mlp_policy()
    m = _stub_model(policy)
    attach_actor_critic_lr(m, actor_lr=5e-5, critic_lr=3e-4)
    set_actor_freeze(m, 100)
    opt = policy.optimizer
    m.num_timesteps = 50
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == 0.0
    m.num_timesteps = 150
    m._update_learning_rate(opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(5e-5)


def test_load_optimizer_state_real_state_dict_shape():
    """Regression-pin the exact shape save_stock_optimizer produces
    (param_groups list under that key) so the compatibility check
    keeps matching torch's real Optimizer.state_dict() format."""
    policy = _mlp_policy()
    opt = policy.optimizer
    sd = opt.state_dict()
    assert "param_groups" in sd
    assert isinstance(sd["param_groups"], list)
