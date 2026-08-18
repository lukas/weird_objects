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
        # Actor-freeze window (set_actor_freeze; default absent/0 =
        # bit-exact off): while num_timesteps < freeze_until the actor
        # group runs lr=0 (Adam with lr 0 changes nothing), so a fresh
        # critic can adapt to a transplanted actor without erasing it
        # (operator order fb_20260818T102844_116d4c item 3). The critic
        # group is never frozen.
        freeze_until = float(st.get("freeze_until", 0.0))
        # EV-readiness hold (attach_ev_readiness_release; operator
        # bridge2 spec fb_20260818T112826_9ed832 item 2, default
        # absent = True = bit-exact prior behavior): while armed, the
        # freeze window releases only when BOTH the step floor AND the
        # critic-EV readiness flag are satisfied.
        ready = bool(st.get("freeze_ready", True))
        frozen = (freeze_until > 0.0
                  and (float(getattr(self, "num_timesteps", 0))
                       < freeze_until or not ready))
        if frozen:
            lr_a = 0.0
        opt = self.policy.optimizer
        opt.param_groups[0]["lr"] = lr_a
        opt.param_groups[1]["lr"] = st["critic_lr"]
        logger = getattr(self, "logger", None)
        if logger is not None:
            logger.record("train/actor_lr", lr_a)
            logger.record("train/critic_lr", st["critic_lr"])
            if freeze_until > 0.0:
                logger.record("train/actor_frozen", float(frozen))

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


def load_optimizer_state_if_compatible(optimizer, state_dict: dict,
                                       context: str = "") -> bool:
    """Load ``state_dict`` into ``optimizer`` iff the param-group COUNT
    matches; else skip (fresh Adam moments) and print why.

    BUG FIX 2026-08-18 (cw-dynrep-criticD-walkcurr4-bridge1 crash,
    first walkcurr promotion+rollback ever reached on a
    --actor-lr/condition-D checkpoint): ``save_stock_optimizer``
    (above) deliberately writes every checkpoint's ``policy.optimizer``
    as a FRESH, SINGLE-group stock optimizer state (so external
    ``PPO.load``/eval never crashes on the live 2-group actor/critic
    split) — but the walkcurr curriculum's own IN-TRAINING rollback
    reloads a promotion checkpoint straight into the LIVE 2-group
    optimizer with a blind ``load_state_dict``, which torch refuses
    ("loaded state dict has a different number of parameter groups")
    the moment a run actually reaches a promotion+rollback under
    ``--actor-lr``. Every checkpoint saved by ``attach_actor_critic_lr``
    now carries a group-count mismatch by DESIGN (that is the whole
    point of the eval-compatibility fix); the correct rollback
    behavior is the same one warm-starts already use: restore the
    POLICY weights exactly, let the optimizer restart with fresh
    moments. Returns True if the load happened, False if skipped."""
    live_n = len(optimizer.param_groups)
    saved_n = len(state_dict.get("param_groups", []))
    if saved_n == live_n:
        optimizer.load_state_dict(state_dict)
        return True
    print(f"  [optimizer-reload{': ' + context if context else ''}] "
          f"skipping optimizer-state reload (checkpoint saved "
          f"{saved_n} stock optimizer group(s), live optimizer has "
          f"{live_n} group(s) — save_stock_optimizer's eval-"
          "compatibility contract writes a single-group stock "
          "snapshot regardless of the live split); policy WEIGHTS are "
          "still restored exactly, optimizer momentum resets fresh, "
          "same as any warm start.")
    return False


def set_actor_freeze(model, until_steps: int) -> None:
    """Arm the default-off actor-freeze window (operator order
    fb_20260818T102844_116d4c item 3): the update_health ACTOR param
    group (which includes log_std and any shared trunk) runs lr=0
    until ``model.num_timesteps >= until_steps``, then returns to the
    attached actor-LR schedule. The critic group learns throughout —
    the point is letting a fresh critic adapt to a transplanted,
    already-competent actor without erasing the actor. Requires
    ``attach_actor_critic_lr`` first; ``until_steps <= 0`` disarms
    (bit-exact off). ``train/actor_frozen`` is logged every update
    while a window is armed."""
    st = getattr(model, "_ac_state", None)
    if st is None:
        raise ValueError("set_actor_freeze requires "
                         "attach_actor_critic_lr to be attached first")
    st["freeze_until"] = max(0.0, float(until_steps))


def restore_actor_only_from_state(policy, saved_policy_state: dict,
                                  optimizer=None,
                                  critic_markers=None,
                                  ) -> tuple[int, bool]:
    """Restore ONLY the actor-side tensors of ``policy`` from a saved
    ``policy.state_dict()`` snapshot; never touch critic/value tensors
    or frozen buffers (operator bridge2 spec fb_20260818T112826_9ed832
    item 1: whole-policy curriculum rollback repeatedly undid the
    fresh condition-D critic's own adaptation — critic EV +.304 ->
    -.183 across bridge1-retry1's three rollbacks — which is
    structurally wrong for a fresh critic + pretrained actor).

    ``critic_markers``: substrings marking NON-actor tensors (default
    ``CRITIC_MARKERS``; condition-D callers add ``value_gate``/
    ``latent_adapter``/``critic_predictor``/``obs_to_frames``/
    ``snapshot_version``). Every state-dict key NOT matching a marker
    is considered actor-side (matches split_actor_critic_params'
    complement, plus ``log_std``, which carries no marker).

    If ``optimizer`` is given and has the attach_actor_critic_lr
    2-group shape, the ACTOR group's Adam moments are cleared (fresh
    lazily-reinitialized state) while the critic group's moments are
    PRESERVED untouched. Returns ``(n_restored,
    critic_unchanged)`` where ``critic_unchanged`` verifies by value
    that no non-actor tensor moved (assertion material for the
    caller)."""
    markers = (critic_markers if critic_markers is not None
               else CRITIC_MARKERS)

    def _is_actor(name: str) -> bool:
        return not any(m in name for m in markers)

    current = policy.state_dict()
    critic_before = {k: v.detach().clone()
                     for k, v in current.items() if not _is_actor(k)}
    n = 0
    for k in list(current.keys()):
        if _is_actor(k) and k in saved_policy_state:
            current[k] = saved_policy_state[k]
            n += 1
    policy.load_state_dict(current)
    after = policy.state_dict()
    import torch as th
    critic_unchanged = all(
        th.equal(after[k], critic_before[k]) for k in critic_before)
    if optimizer is not None and len(optimizer.param_groups) == 2:
        cleared = 0
        for p in optimizer.param_groups[0]["params"]:
            if p in optimizer.state:
                del optimizer.state[p]
                cleared += 1
        print(f"  [actor-only rollback] {n} actor tensors restored, "
              f"{len(critic_before)} critic/frozen tensors untouched "
              f"(verified {'UNCHANGED' if critic_unchanged else 'CHANGED — BUG'}), "
              f"{cleared} actor Adam-moment entries reset, critic "
              "optimizer state preserved")
    return n, critic_unchanged


def attach_ev_readiness_release(model, ev_threshold: float,
                                ev_windows: int,
                                max_steps: int) -> None:
    """Arm the critic-EV readiness gate on an existing actor-freeze
    window (operator bridge2 spec fb_20260818T112826_9ed832 item 2,
    default-off — only wired when the trainer passes
    ``--actor-freeze-ev-threshold`` > 0): the frozen actor releases
    only after ``train/explained_variance`` >= ``ev_threshold`` for
    ``ev_windows`` CONSECUTIVE updates (and the existing
    ``freeze_until`` step floor has passed — enforced in
    ``_update_learning_rate``). If the critic is still not ready at
    ``max_steps``, the run ABORTS fail-closed rather than training an
    unfrozen actor against a critic that never converged.

    Requires attach_actor_critic_lr + set_actor_freeze first. Wraps
    ``model.train`` (same post-update logger read attach_kl_rollback
    uses); logs ``train/actor_freeze_ready`` and
    ``train/critic_ev_ready_windows`` every update while armed."""
    st = getattr(model, "_ac_state", None)
    if st is None or float(st.get("freeze_until", 0.0)) <= 0.0:
        raise ValueError("attach_ev_readiness_release requires "
                         "attach_actor_critic_lr + set_actor_freeze "
                         "(it gates an armed freeze window)")
    if ev_windows < 1 or max_steps <= 0:
        raise ValueError("attach_ev_readiness_release: ev_windows>=1 "
                         "and max_steps>0 required (fail-closed cap)")
    st["freeze_ready"] = False
    st["_ev_streak"] = 0
    orig_train = model.train

    @functools.wraps(orig_train)
    def train_ev_gated() -> None:
        orig_train()
        if st.get("freeze_ready"):
            return
        logger = getattr(model, "logger", None)
        ev = (logger.name_to_value.get("train/explained_variance")
              if logger is not None else None)
        if ev is not None:
            st["_ev_streak"] = (st["_ev_streak"] + 1
                                if float(ev) >= ev_threshold else 0)
        if st["_ev_streak"] >= ev_windows:
            st["freeze_ready"] = True
            print(f"[ev-readiness] critic EV >= {ev_threshold} for "
                  f"{ev_windows} consecutive updates @ "
                  f"{model.num_timesteps:,} steps — actor freeze "
                  "released (step floor still applies)")
        elif int(getattr(model, "num_timesteps", 0)) >= max_steps:
            raise RuntimeError(
                f"[ev-readiness] FAIL-CLOSED: critic EV never held >= "
                f"{ev_threshold} for {ev_windows} consecutive updates "
                f"by the {max_steps:,}-step cap (last ev="
                f"{'n/a' if ev is None else f'{float(ev):.3f}'}, "
                f"streak={st['_ev_streak']}) — refusing to unfreeze "
                "an actor against a critic that never converged "
                "(fb_20260818T112826_9ed832 item 2)")
        if logger is not None:
            logger.record("train/actor_freeze_ready",
                          float(bool(st.get("freeze_ready"))))
            logger.record("train/critic_ev_ready_windows",
                          float(st["_ev_streak"]))

    model.train = train_ev_gated
    _exclude_from_save(model, ("train",))


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
