"""update_health.py — PPO update-path protection + run-health tracking.

Operator-approved direction (fb_20260817T005114_775298) after
cw-arch-joystick-long-scratch3 (W&B a7rtr3lq) finished its 40M budget
in catastrophic late-run regression: reward/tick EMA peaked -2.16 @
14.7M and ended -12.45, mean episode length fell ~1204 -> ~105 ticks,
late updates ran approx_kl ~0.24 / clip_fraction ~0.40 DESPITE
target_kl=0.02 (SB3's early stop does not undo the damaging
minibatch), and critic explained variance stayed ~0 for the entire
run. Machinery here (all opt-in, trainer flags default OFF, training
is bit-exact when off):

- ``split_actor_critic_params`` / ``attach_actor_critic_lr``:
  separate actor and critic optimizer param groups with independent
  learning rates and a linear actor-LR decay, so lowering the actor
  LR (update-size control) does not further starve the critic.
- ``attach_kl_rollback``: transactional PPO updates — snapshot the
  policy before each ``model.train()``, roll the parameters back and
  multiply the actor LR down when the realized approx_kl overshoots
  a hard ceiling. SB3 target_kl early-stopping only stops FUTURE
  epochs; the overshooting minibatch has already been applied.
- ``HealthTracker``: best-checkpoint composite score + joint
  regression stop (reward, survival and direction must regress
  TOGETHER over repeated assays before a stop fires — any one metric
  alone is eval noise).
- ``EVTracker``: critic explained-variance EMA + hard-failure test
  (EV ~0 past a deadline = the value function is learning nothing —
  a canary hard failure, never a normal plateau).

Pure-torch/py logic, no env or trainer imports — unit-tested in
rl_move/tests/test_value_learning.py.
"""
from __future__ import annotations

import functools
import types


def _exclude_from_save(model, names: tuple[str, ...]) -> None:
    """Keep monkeypatched instance attributes out of model.save().

    SB3's save() cloudpickles __dict__ minus _excluded_save_params();
    a bound wrapper closing over the model would otherwise drag the
    whole model (env included) into the zip or crash pickling. The
    exclusion wrapper excludes ITSELF too; load() restores the stock
    class methods, and the trainer re-attaches on warm start.
    """
    if not hasattr(model, "_excluded_save_params"):
        return   # duck-typed test stubs
    orig = model._excluded_save_params

    def _excluded() -> list:
        return (list(orig()) + list(names)
                + ["_excluded_save_params"])

    model._excluded_save_params = _excluded

# Parameter-name markers that put a parameter into the CRITIC group.
# SB3 ActorCriticPolicy names: value_net (head), mlp_extractor.value_net
# (trunk), vf_features_extractor (unshared extractor, e.g. the
# transformer critic trunk). A SHARED features_extractor deliberately
# lands in the ACTOR group: its updates move the action distribution,
# so the (smaller, safer) actor LR must govern it.
CRITIC_MARKERS = ("value_net", "vf_features_extractor")


def split_actor_critic_params(policy, critic_markers=None):
    """(actor_params, critic_params) — disjoint, covers every
    trainable parameter of the policy exactly once.

    ``critic_markers`` overrides the module-level ``CRITIC_MARKERS``
    (additive callers only — e.g. ``PredictiveCriticPolicy`` adds
    ``value_gate``/``latent_adapter``, the critic-only residual path
    that sits alongside the frozen predictive encoder and shares no
    substring with the stock SB3 markers; using the stock set on that
    policy would silently throttle those two modules with the ACTOR
    learning rate instead of the critic's). Default preserves the
    exact prior behavior for every existing caller."""
    markers = critic_markers if critic_markers is not None else CRITIC_MARKERS
    actor, critic = [], []
    for name, p in policy.named_parameters():
        if not p.requires_grad:
            continue
        (critic if any(m in name for m in markers)
         else actor).append(p)
    return actor, critic


def attach_actor_critic_lr(model, actor_lr: float,
                           actor_lr_final: float | None = None,
                           critic_lr: float | None = None,
                           critic_markers=None) -> None:
    """Rebuild the policy optimizer with separate actor/critic groups.

    - actor group: linear decay actor_lr -> actor_lr_final over the
      run (progress from ``model._current_progress_remaining``), then
      multiplied by the mutable ``model._ac_state['actor_scale']``
      (the KL-rollback lever).
    - critic group: constant critic_lr (the critic must keep
      learning while the actor is throttled).

    Overrides ``model._update_learning_rate`` so SB3's own schedule
    can never clobber the per-group rates.
    """
    if actor_lr_final is None:
        actor_lr_final = actor_lr
    if critic_lr is None:
        critic_lr = actor_lr
    actor, critic = split_actor_critic_params(model.policy, critic_markers)
    if not actor or not critic:
        raise ValueError(
            "actor/critic param split failed (one group empty) — "
            f"policy {type(model.policy).__name__} has unexpected "
            "parameter names")
    opt_cls = getattr(model.policy, "optimizer_class", None)
    opt_kw = dict(getattr(model.policy, "optimizer_kwargs", None) or {})
    if opt_cls is None:
        import torch as th
        opt_cls = th.optim.Adam
        opt_kw.setdefault("eps", 1e-5)
    model.policy.optimizer = opt_cls(
        [{"params": actor, "lr": actor_lr},
         {"params": critic, "lr": critic_lr}], **opt_kw)
    model._ac_state = {
        "actor_lr": float(actor_lr),
        "actor_lr_final": float(actor_lr_final),
        "critic_lr": float(critic_lr),
        "actor_scale": 1.0,
        "n_actor": len(actor), "n_critic": len(critic),
    }

    def _update_learning_rate(self, optimizers) -> None:
        st = self._ac_state
        progress = float(
            getattr(self, "_current_progress_remaining", 1.0))
        lr_a = (st["actor_lr_final"]
                + (st["actor_lr"] - st["actor_lr_final"]) * progress)
        lr_a *= st["actor_scale"]
        opt = self.policy.optimizer
        opt.param_groups[0]["lr"] = lr_a
        opt.param_groups[1]["lr"] = st["critic_lr"]
        logger = getattr(self, "logger", None)
        if logger is not None:
            logger.record("train/actor_lr", lr_a)
            logger.record("train/critic_lr", st["critic_lr"])

    model._update_learning_rate = types.MethodType(
        _update_learning_rate, model)
    _exclude_from_save(model, ("_update_learning_rate",))

    # Checkpoint compatibility: PPO.load reconstructs a STOCK
    # single-param-group optimizer and torch refuses a 2-group state
    # dict, which would break every harness/eval/warm-start load of
    # this run's checkpoints. Save therefore swaps in a fresh stock
    # optimizer for the duration of the write (same "fresh Adam
    # moments on warm start" contract the obs-pad transplant already
    # has); the live 2-group optimizer is untouched.
    orig_save = getattr(model, "save", None)
    if orig_save is None:
        return   # duck-typed test stubs have no save path

    def save_stock_optimizer(*a, **kw):
        real_opt = model.policy.optimizer
        # BUG FIX 2026-08-18 (cw-dynrep-criticD-walkcurr3 eval crash):
        # this MUST match what a freshly-constructed policy's default
        # optimizer would contain, or PPO.load's set_parameters raises
        # "parameter group that doesn't match the size of optimizer's
        # group" on every eval/warm-start of this run's checkpoints.
        # A policy with a frozen submodule (e.g. PredictiveCriticPolicy's
        # frozen dynamics-transformer encoder) has model.policy.para-
        # meters() >> trainable params — the encoder tensors sit in
        # policy.parameters() with requires_grad=False but were never
        # part of either the actor or critic optimizer group. The
        # ORIGINAL "policy.parameters()" call silently included them,
        # producing a stock optimizer with N=all-params instead of
        # N=trainable-params (172 vs 18 on walkcurr3) — every
        # attach_actor_critic_lr checkpoint of a policy with frozen
        # parameters (all condition-D/criticD walkcurr runs using
        # --actor-lr) saved an uneval-able optimizer. Filter to
        # trainable params only, matching PredictiveCriticPolicy._build
        # (and every other policy's default optimizer construction).
        model.policy.optimizer = opt_cls(
            [p for p in model.policy.parameters() if p.requires_grad],
            lr=model._ac_state["critic_lr"], **opt_kw)
        try:
            return orig_save(*a, **kw)
        finally:
            model.policy.optimizer = real_opt

    model.save = save_stock_optimizer
    _exclude_from_save(model, ("save",))


def attach_kl_rollback(model, threshold: float,
                       lr_factor: float = 0.5,
                       scale_min: float = 0.01) -> None:
    """Transactional PPO update: snapshot -> train -> verify realized KL.

    On ``train/approx_kl`` > threshold: restore the pre-update policy
    parameters and multiply the actor-LR scale by ``lr_factor``
    (floored at ``scale_min``). Requires ``attach_actor_critic_lr``
    first (the scale lives in ``model._ac_state``). Counters are
    recorded to the SB3 logger: train/kl_rollback_count (cumulative),
    train/kl_rollback (1/0 this update).
    """
    if not hasattr(model, "_ac_state"):
        raise ValueError("attach_actor_critic_lr must be attached "
                         "before attach_kl_rollback (it owns the "
                         "actor-LR scale the rollback reduces)")
    model._kl_rollback_count = 0
    orig_train = model.train

    @functools.wraps(orig_train)
    def train_guarded() -> None:
        snap = {k: v.detach().clone()
                for k, v in model.policy.state_dict().items()}
        orig_train()
        logger = getattr(model, "logger", None)
        kl = None
        if logger is not None:
            kl = logger.name_to_value.get("train/approx_kl")
        rolled = 0
        if kl is not None and float(kl) > threshold:
            model.policy.load_state_dict(snap)
            st = model._ac_state
            st["actor_scale"] = max(
                st["actor_scale"] * lr_factor, scale_min)
            model._kl_rollback_count += 1
            rolled = 1
            print(f"[kl-rollback] approx_kl {float(kl):.4f} > "
                  f"{threshold:.4f}: policy restored, actor-LR scale "
                  f"-> {st['actor_scale']:.4f} "
                  f"(#{model._kl_rollback_count})")
        if logger is not None:
            logger.record("train/kl_rollback", rolled)
            logger.record("train/kl_rollback_count",
                          model._kl_rollback_count)

    model.train = train_guarded
    _exclude_from_save(model, ("train",))


class HealthTracker:
    """Best-checkpoint + joint-regression stop over periodic assays.

    feed() one assay at a time. ``is_best`` fires on a composite
    score improvement (survival first, then direction, then aligned
    velocity). ``should_stop`` fires only when reward, survival AND
    direction are ALL below/behind their bests by clear margins for
    ``regress_n`` consecutive assays — single-metric wobbles are eval
    noise and must never stop a run.
    """

    def __init__(self, regress_n: int = 3,
                 surv_margin: float = 0.15,
                 dir_margin_deg: float = 15.0,
                 rew_margin_frac: float = 0.10):
        self.regress_n = int(regress_n)
        self.surv_margin = float(surv_margin)
        self.dir_margin_deg = float(dir_margin_deg)
        self.rew_margin_frac = float(rew_margin_frac)
        self.best_score = None
        self.best_surv = None
        self.best_dir_err = None
        self.best_rew = None
        self.streak = 0
        self.n_assays = 0

    @staticmethod
    def composite(survived_frac: float, dir_err_deg: float,
                  v_along_ratio: float) -> float:
        return (float(survived_frac)
                - min(max(float(dir_err_deg), 0.0), 180.0) / 180.0
                + min(max(float(v_along_ratio), -1.0), 1.0))

    def feed(self, survived_frac: float, dir_err_deg: float,
             v_along_ratio: float, rew_per_tick: float) -> dict:
        self.n_assays += 1
        score = self.composite(survived_frac, dir_err_deg,
                               v_along_ratio)
        is_best = self.best_score is None or score > self.best_score
        if is_best:
            self.best_score = score
        if self.best_surv is None or survived_frac > self.best_surv:
            self.best_surv = float(survived_frac)
        if self.best_dir_err is None or dir_err_deg < self.best_dir_err:
            self.best_dir_err = float(dir_err_deg)
        if self.best_rew is None or rew_per_tick > self.best_rew:
            self.best_rew = float(rew_per_tick)
        rew_margin = self.rew_margin_frac * max(abs(self.best_rew), 0.1)
        regressed = (
            survived_frac < self.best_surv - self.surv_margin
            and dir_err_deg > self.best_dir_err + self.dir_margin_deg
            and rew_per_tick < self.best_rew - rew_margin)
        self.streak = self.streak + 1 if regressed else 0
        return {"score": score, "is_best": is_best,
                "regressed": regressed,
                "should_stop": self.streak >= self.regress_n}


class EVTracker:
    """Critic explained-variance EMA + hard-failure deadline.

    fb_20260817T005114 item 2: EV pinned at ~0 is a HARD failure of
    value learning, not a plateau. failed(step) is True when the EMA
    is still below ``ev_min`` after ``deadline_steps``.
    """

    def __init__(self, ev_min: float = 0.05,
                 deadline_steps: int = 1_500_000, beta: float = 0.8):
        self.ev_min = float(ev_min)
        self.deadline_steps = int(deadline_steps)
        self.beta = float(beta)
        self.ema = None

    def feed(self, ev: float) -> float:
        ev = float(ev)
        self.ema = ev if self.ema is None \
            else self.beta * self.ema + (1.0 - self.beta) * ev
        return self.ema

    def failed(self, step: int) -> bool:
        return (self.ema is not None
                and step >= self.deadline_steps
                and self.ema < self.ev_min)
