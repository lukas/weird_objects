"""Reference BC anchor — auxiliary trainer loss for the rise task
(and, since 08-11, hold/track quiet-stand supervision — see below).

Why this exists (RISE.md, 08-11): six reward-different stand-up arms
(score1 / scoreref1 / plantgate1 / rsi1 / rsi2 / rsi3) collapsed on the
IDENTICAL feet-factor curve (0.87 -> ~0.17 by the 25% mark) regardless
of which income/penalty/RSI mechanism was live — behavior that does not
respond to reward changes is not reward-driven. Diagnosis: warm-start
out-of-distribution drift. The 108–114 mm command band is ~2.2x the
stance champion's trained range; the 6-degree tracking kernel only pays
a policy that is ALREADY nearly perfect there (widening it is
bank-blocked: at 10 degrees flag-leg farms 17% of replay), so early
update noise erodes the warm start into the tripod and no reward
stream produces a gradient that pulls it back.

The anchor attacks that mechanism directly (DeepMimic-family standard;
operator's preferred lever (a), RL_PLAN queue 2a): a supervised
auxiliary loss  coef * mse( pi_mean(obs), a_ref )  where a_ref is the
normalized action whose joint target is the reference pose one
reference tick ahead of the episode's live ref clock. It supervises
ACTIONS, not visited rewards, so it is immune to pose-farming and
needs no rollout luck: even at drifted states the target points back
onto the demonstrated path. The reward stack is UNTOUCHED — this is
not a reward term, the rise semantics bank is unaffected.

08-11 follow-up (RL_PLAN queue 2.3): hold/track stillness pricing
(reward.hold_still_gate + hold_flag_fade) moved the wrong-leg-parked
behavior twice (hard zero, then a linear fade) but never reached a
quiet plant — same "earning zero is not being pushed back" failure
mode as rise. Hold/track have no moving reference to chase, so the
target there is simply the pose the episode actually settled at
(sim_env._q_nom, constant for the whole episode — "stand still right
here"), reusing the identical collect/train-step machinery below.

08-11 second follow-up (RL_PLAN queue 2.1 / probe_walk_income): WALK
ticks emit too. The omni-translation arms (mirror2/dr02/trans1) each
collapsed into a different degenerate gait while the term-by-term
income probe shows the stack pays honest gait 2-4x above every
degenerate in all four directions at DR 0 and 0.5 — and the collapsed
checkpoints earn BELOW a freeze. Optimization failure, not pricing:
the same "nothing tells the leg which way to move" signature as rise
and hold. Walk target = the command-conditioned scripted TripodGait
(the open-loop gait that walks/crabs/turns the REAL robot) one tick
ahead, per-episode instance on SNAP_ATTRS; NO emission on
zero-command (stop) ticks — the gait marches in place at v=0 while
the commanded behavior there is standing still.

08-12 follow-up (cw-getup2-r1 informative FAIL — see RL_PLAN queue):
GETUP ticks emit too, cfg-gated by ``train.bc_anchor_getup`` (default
0 = off). Warm-starting the getup task from the rise+hold specialist
(cw-getup2-r1) showed the specialist's stand skill does NOT survive
contact with the getup reward: env/getup_S and the training return
both DECLINED over 2M steps from an initially-elevated warm-start
band back toward the from-scratch cw-getup1 collapse, and the final
video shows the same static splayed/collapsed hold. A soft prior
(warm-start weights) is not enough; the getup task needs the same
explicit pull-toward-a-demonstration the rise lever needed. Reuses
the rise reference demo (``reward.rise_ref_path``) — the only demo we
have of a stand-up path — but ALWAYS state-aligned (nearest reference
pose to the CURRENT joints, the mode anchorstate1/2 proved for
plant-adjacent rise starts): getup starts are arbitrary (belly,
tangled, crouch, park, ...) and there is no live clock to align a
fixed-time-index target to. Same one-lookahead-ahead pursuit target
as rise's state-aligned mode.

Data path: sim_env._step_finish emits ``info["bc_target"]`` (float32,
18) for every rise tick with a live reference clock, every getup tick
(when bc_anchor_getup > 0), every hold/track tick, OR every commanded
walk tick, when ``train.bc_anchor_coef`` > 0 (the trainer cfg key
rides into the env cfg dict, same pattern as train.mirror_loss_coef).
``BCAnchorCollectCallback`` pairs each target with the post-step obs
(``new_obs``) into a ring buffer on the model; ``BCAnchorPPO.train()``
runs the aux optimizer step AFTER the untouched PPO update, exactly
the MirrorPPO pattern (separate step, no SB3 internals copied, PPO
clip/target_kl byte-identical to every other run).

Episode boundaries: on done the vec env swaps ``new_obs`` for the NEXT
episode's reset obs while the info stays the finished step's — those
pairs are misaligned and are skipped (the lost reset-tick pair is
negligible).

Knobs (set via attach_bc_anchor / cfg):
  train.bc_anchor_coef         loss weight (0 = everything off, exact)
  train.bc_anchor_minibatches  aux minibatches per update (default 8)
  train.bc_anchor_batch_size   aux minibatch size (default 4096)
  train.bc_anchor_buffer       ring capacity in pairs (default 131072)
  train.bc_anchor_stratified   equal per-mode minibatch quotas
                               (default 0 = legacy uniform; see
                               _bc_sample_idx for the loweranchor1
                               dilution measurement that motivates it)
  train.bc_anchor_foot_z       coefficient on an ADDITIONAL foot-
                               height anchor term: mse of the
                               commanded FK foot heights (body frame,
                               scaled by bc_anchor_foot_z_mm) between
                               pi_mean and the target. 0 (default) =
                               off, legacy update bit-exact. Exists
                               because the one-parked-foot habit is
                               invisible to joint-space MSE (08-12
                               audit: parked leg costs 0.0032 vs the
                               clean parent's 0.0031 on the same leg —
                               a mm hover is fractions of a degree);
                               in foot-height space a 10 mm hover
                               costs ~1.0 at the default scale.
  train.bc_anchor_foot_z_mm    height scale for the term (default 10:
                               a 10 mm commanded deviation -> unit
                               squared error)
  train.bc_anchor_min_h_ahead_mm  (env-side, state-aligned rise only)
                               height-floor pursuit: the target tick
                               must command at least this many mm
                               above the chassis's CURRENT height
                               (default 0 = off, bit-exact). Exists
                               because the 08-12 footlow1 dig-in
                               (probe_anchor_align) measured the
                               flat-rise stall as a PLATEAU FIXED
                               POINT: the reference crawls 0->25 mm
                               over 5+ s, so the +0.5 s time lookahead
                               commands a pose 1-5 mm higher, loaded-
                               servo sag cancels it, the matched index
                               pins, and the anchor supervises the
                               stall with a low, converged loss.
  train.bc_anchor_tilt_comp    (env-side, HOLD episodes only) tip-aware
                               hold reference: instead of the constant
                               q_nom, anchor toward FixedFootBodyIK
                               (q_nom) solved at BodyOffset(roll/pitch
                               = -comp * measured attitude relative to
                               the episode tilt reference) — a
                               proportional posture-feedback teacher
                               that gives the anchor a LEVELING
                               gradient (default 0 = legacy constant
                               target, bit-exact). Exists because
                               cw-stand-footlow2-tip1 (08-12) showed
                               tipped-start DR under a tilt-blind
                               anchor teaches tilt TOLERANCE (holds
                               q_nom leaning 7deg), and the hardware
                               candidate stands with a persistent
                               ~8deg lean. Track episodes are excluded
                               (they command attitude goals the
                               compensation would fight).
  train.bc_anchor_tilt_from_settle  (default 0 = current-lean source,
                               bit-exact) source the tilt-comp from
                               the episode's POST-SETTLE lean
                               (_settle_lean, per-episode constant,
                               SNAP_ATTRS) instead of the live
                               measured lean. probe_tilt_teacher
                               (08-13) measured the live source is a
                               P-controller with a closed-loop fixed
                               point at (L0+deadband)/2 — a PERFECT
                               student settles 3.95deg from 6.5deg
                               tipped spawns (predicted 3.98), never
                               the <=3deg bar, because the commanded
                               correction shrinks as the student
                               levels. With the settle source the
                               ideal student levels to the deadband /
                               cap residual, where the k_track income
                               Gaussian regains gradient.
  train.bc_anchor_tilt_deadband_deg  soft deadband (deg, default 1.5)
                               subtracted from |rel attitude| before
                               compensation — settled-level ticks keep
                               the exact q_nom target, and the target
                               stays continuous at the threshold.
  train.bc_anchor_tilt_max_deg cap (deg, default 6) on the commanded
                               counter-rotation per axis: the measured
                               action-space expressibility boundary
                               (7deg+ saturates a joint bound from the
                               settled stance; IK itself reaches 10).
  train.bc_anchor_detach_trunk (recurrent only) stop the anchor
                               gradient at the GRU/feature-extractor
                               output — only the actor head trains on
                               it (default 0 = legacy, grad flows into
                               the whole shared trunk; see
                               cw-arch-gru-anchor2 in _bc_policy_mean)
  train.bc_anchor_isolate_update  (default 0 = legacy, bit-exact) drop
                               all-zero grads (set .grad=None) before
                               the aux optimizer step so the SHARED
                               Adam never touches parameters the aux
                               loss did not reach. Exists because the
                               08-26 stage-2 dig-in (cw-standwalk-
                               stance-mesh2-stage2-dualbc1-anchor1/-s1)
                               measured the dual-core "walk-off" anchor
                               still MOVING the walk core: on the
                               gated-out core autograd populates exact-
                               ZERO .grad tensors (gate=0 multiplies,
                               accumulation still happens), and
                               Adam.step() with a populated zero grad
                               applies a pure MOMENTUM update — 8 extra
                               stale-PPO-momentum steps per PPO update
                               into the walk actor path, every update.
                               Gradient isolation was never violated
                               (test_dual_gradient_isolation held);
                               UPDATE isolation was. Harmless when the
                               core is converged (cw-arch-gru-dual1,
                               small momentum), catastrophic mid-
                               collapse (anchor1: walk destroyed
                               2 seeds, 2 different pathologies).

  train.bc_anchor_debug_adv_stats (default 0, bit-exact when off)
                               READ-ONLY diagnostic (08-27,
                               anchor7-detachtrunk dig-in): after
                               super().train() has already consumed
                               and applied the rollout buffer, logs
                               train/adv_{loco,stance}_{mean,std,share}
                               — the RAW (pre sb3-contrib
                               per-minibatch normalization) advantage
                               moments split by the obs-tail mode
                               one-hot (requires obs.mode_onehot=1).
                               Tests whether the dual-core catastrophe
                               that survives both the log_std-split and
                               bc_anchor_detach_trunk fixes is actually
                               a THIRD, still-untested mechanism: sb3-
                               contrib's global per-minibatch advantage
                               normalization mixes loco/stance ticks
                               (this stack has no VecNormalize/reward
                               scaling, and goal.mode_seq composes
                               modes inside one episode, so minibatches
                               are cross-mode by construction) — if
                               stance's raw advantage variance dwarfs
                               walk's, the shared normalizer could be
                               shrinking walk's own PPO gradient toward
                               zero independent of any weight-sharing
                               path. Changes no gradient/weight/RNG
                               draw; see _maybe_log_adv_mode_stats.
                               MEASURED (08-27, anchor8-advstats seed0,
                               2M steps): REFUTED, and not just "no
                               support" but the opposite of the
                               predicted direction — adv_loco_std is
                               often SEVERAL-X adv_stance_std (up to
                               ~7x mid-training, during the exact
                               reward-trough window), never the
                               reverse. Escalates to the pre-registered
                               fallback: PPO's global
                               ``clip_grad_norm_`` (see
                               bc_anchor_debug_gradnorm below).

  train.bc_anchor_debug_gradnorm (default 0, bit-exact when off)
                               READ-ONLY diagnostic (08-27,
                               anchor8-advstats REFUTATION follow-up):
                               the dual-core policy already gives
                               loco/stance fully separate GRUs, actor
                               heads, and value heads (verified by
                               code read: the only shared per-tick
                               computation left is a parameterless
                               FlattenExtractor, i.e. no shared
                               trainable trunk remains) — but
                               sb3-contrib's ``RecurrentPPO.train()``
                               still calls ONE global
                               ``clip_grad_norm_(self.policy.
                               parameters(), max_grad_norm)`` over
                               EVERY parameter together each minibatch.
                               If one core's raw gradient norm
                               dominates, the shared clip factor
                               rescales BOTH cores' grads identically,
                               starving the smaller core even though
                               its own weights are otherwise fully
                               isolated. Wraps ``super().train()`` in a
                               context manager that transiently
                               monkeypatches
                               ``torch.nn.utils.clip_grad_norm_`` to
                               record the PRE-CLIP per-core (a/b) and
                               shared-param grad L2 norms
                               (``train/gradnorm_{a,b,shared}_mean``,
                               averaged over the train() call's
                               minibatches) before delegating to the
                               real clip (unchanged behavior — same
                               tensors, same max_norm, same return).
                               No-op / restores the original function
                               immediately when off or when the policy
                               isn't dual-core (no ``mlp_extractor_b``).
                               See _gradnorm_diag_ctx.

Logged: train/bc_anchor_loss (post-step mse of the last minibatch),
train/bc_anchor_fill (ring occupancy, pairs), and per mode present in
the ring train/bc_anchor_loss_{rise,hold,lower,walk} +
train/bc_anchor_fill_* (no_grad diagnostic AFTER the optimizer steps —
zero training effect; landed 08-12, the pre-registered observability
gate before any further stand arm).

Recurrent support (08-11, arch track unblock): pass RecurrentPPO as
``base_cls`` and the anchor works on GRU policies too. Each pair then
carries the rollout's actor hidden state (the collect-loop local
``lstm_states.pi[0]`` — the state the policy holds when it next sees
``new_obs``), and the aux step computes the pi mean by ONE fused GRU
cell step from that stored state (``_bc_policy_mean``). Anchoring at a
zero hidden state instead would supervise a policy the rollouts never
run. Gradients flow through the cell weights for that single step —
no BPTT in the aux step, so cost matches the MLP anchor.
"""
from __future__ import annotations

import numpy as np

N_ACT = 18

_FOOT_Z_CONST = None  # lazy torch constants for _bc_foot_z


def _bc_foot_z(actions):
    """Commanded foot HEIGHT (body frame, metres) per leg from a
    normalized action batch — differentiable torch twin of
    ``body_ik.fk_all_feet()[:, 2]``.

    Why (08-12 park audit, cw-stand-margin1/transdrag1 dig-in): the
    one-parked-foot hold habit is INVISIBLE to the joint-space anchor
    MSE — the parked policy's per-leg anchor loss (0.0032) is
    byte-comparable to the six-foot parent's same leg (0.0031),
    because a millimetre-scale contact break needs only fractions of
    a degree of hip/knee lift. Near the plant, foot height is the
    contact-relevant coordinate and joint MSE dilutes it 3-dims-in-18.
    Supervising the FK foot height directly makes a 10 mm commanded
    hover cost ~1.0 (at the default 10 mm scale) instead of ~1e-4.

    Foot z depends only on hip and knee (yaw rotates in-plane):
        z = -FEMUR*sin(hip) - TIBIA*sin(hip+knee)
    """
    import torch
    global _FOOT_Z_CONST
    if _FOOT_Z_CONST is None:
        from rl_move.body_ik import FEMUR, TIBIA
        from .joint_task import _CENTER_RAD, _HALF_RAD
        _FOOT_Z_CONST = (
            torch.as_tensor(_CENTER_RAD, dtype=torch.float32),
            torch.as_tensor(_HALF_RAD, dtype=torch.float32),
            float(FEMUR), float(TIBIA))
    center, half, femur, tibia = _FOOT_Z_CONST
    if center.device != actions.device:
        center = center.to(actions.device)
        half = half.to(actions.device)
        _FOOT_Z_CONST = (center, half, femur, tibia)
    q = center + actions * half
    hip = q[..., 1::3]
    knee = q[..., 2::3]
    return -femur * torch.sin(hip) - tibia * torch.sin(hip + knee)


def _lazy_sb3():
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    return PPO, BaseCallback


def make_bc_anchor_ppo_class(base_cls=None):
    """BCAnchorPPO: ``base_cls`` (default stock PPO; pass MirrorPPO to
    compose both aux losses) + one supervised anchor step per update."""
    PPO, _ = _lazy_sb3()
    base = base_cls or PPO

    class BCAnchorPPO(base):
        bc_coef: float = 0.0

        def _excluded_save_params(self) -> list:
            # The anchor ring is rollout data, not model state: pickling
            # it bloats every checkpoint (36MB obs/act, +134MB once the
            # recurrent _bc_h rides along) and stale buffers already bit
            # one warm start (the _bc_mode backfill above). Fresh runs
            # refill the ring within one rollout.
            return super()._excluded_save_params() + [
                "_bc_obs", "_bc_act", "_bc_mode", "_bc_h",
                "_bc_n", "_bc_i"]

        def _bc_init_buffer(self, obs_dim: int) -> None:
            cap = int(getattr(self, "bc_buffer_cap", 131072))
            self._bc_obs = np.zeros((cap, obs_dim), dtype=np.float32)
            self._bc_act = np.zeros((cap, N_ACT), dtype=np.float32)
            self._bc_mode = np.zeros(cap, dtype=np.int8)
            self._bc_n = 0          # valid rows
            self._bc_i = 0          # write cursor

        def _bc_push(self, obs: np.ndarray, act: np.ndarray,
                     mode: int = 0, h: np.ndarray | None = None) -> None:
            if not hasattr(self, "_bc_obs"):
                self._bc_init_buffer(int(np.asarray(obs).shape[-1]))
            if not hasattr(self, "_bc_mode"):
                # Warm-start backfill: checkpoints trained before the
                # mode tag PICKLE their _bc_obs/_bc_act buffers into
                # the zip, so hasattr(_bc_obs) is True on load and
                # _bc_init_buffer never runs (bit cw-stand-anchormix1
                # on its first launch: AttributeError on tick one).
                self._bc_mode = np.zeros(self._bc_obs.shape[0],
                                         dtype=np.int8)
            cap = self._bc_obs.shape[0]
            if h is not None and not hasattr(self, "_bc_h"):
                # Recurrent anchor: the pair's supervision point is the
                # policy AT THE VISITED HIDDEN STATE, so the rollout's
                # hidden state rides along (flattened n_layers*hidden).
                self._bc_h = np.zeros((cap, h.size), dtype=np.float32)
            self._bc_obs[self._bc_i] = obs
            self._bc_act[self._bc_i] = act
            self._bc_mode[self._bc_i] = mode
            if h is not None:
                self._bc_h[self._bc_i] = h
            self._bc_i = (self._bc_i + 1) % cap
            self._bc_n = min(self._bc_n + 1, cap)

        def _bc_sample_idx(self, rng, n: int, bs: int) -> np.ndarray:
            """Minibatch indices. Legacy: uniform over the ring — the
            per-mode gradient share is then proportional to emission
            share, which is the DILUTION mechanism cw-stand-loweranchor1
            measured (adding lower pairs regressed the hold park leg 4
            from 0.95 back to 0.02 and re-stalled flat rise: hold/rise
            supervision weakened exactly when a third mode joined the
            buffer). Stratified (bc_stratified): equal quotas per mode
            PRESENT in the buffer, so each skill's anchor keeps full
            strength regardless of the goal/emission mix."""
            if not getattr(self, "bc_stratified", False):
                return rng.integers(0, n, size=bs)
            modes = np.unique(self._bc_mode[:n])
            quota = bs // len(modes)
            parts = [rng.choice(np.flatnonzero(self._bc_mode[:n] == m),
                                size=quota, replace=True)
                     for m in modes]
            rem = bs - quota * len(modes)
            if rem:
                parts.append(rng.integers(0, n, size=rem))
            return np.concatenate(parts)

        def _maybe_log_adv_mode_stats(self) -> None:
            """DIAGNOSTIC ONLY, default off, zero effect on training
            (08-27, anchor7-detachtrunk joint-close dig-in): reads
            already-computed rollout-buffer advantages/observations
            AFTER super().train() has fully consumed and updated the
            policy from them, and logs the RAW (pre sb3-contrib
            normalization) per-mode-family advantage mean/std/share.

            Motivation: sb3-contrib's RecurrentPPO.train() normalizes
            advantages with ONE global (mean, std) per minibatch
            (`advantages[mask].mean()/.std()`, ppo_recurrent.py) —
            there is no VecNormalize/reward-scaling anywhere in this
            stack (grep-confirmed). Every dual-core standwalk recipe
            trains with `goal.mode_seq` composing hold/rise/lower/walk
            INSIDE one episode, so a single env's own rollout chunk
            already interleaves multiple mode families in time before
            any cross-env mixing — meaning the shared normalization
            constant is structurally cross-mode on essentially every
            minibatch, independent of any trunk/log_std sharing (both
            already tested and refuted as the sole cause: anchor6/6b
            log_std-split and anchor7 detach_trunk both left the
            anchor4-class walk degradation only partially changed).
            If the stance family's raw advantage variance dominates
            the walk family's (very plausible: rise/hold/lower carry
            large terminal penalties up to `term_cost_max`, walk's
            reward is a bounded per-tick kernel), the shared normalizer
            would shrink walk's own gradient signal toward zero for
            free, independent of any weight-sharing path — a
            previously untested root-cause candidate for the same
            "reward trough, partial recovery, degenerate frozen/
            leg-sacrifice gait" shape every log_std/trunk arm has hit.

            This method only ever READS `self.rollout_buffer.advantages`
            / `.observations` (already fully consumed/mutated in place
            by the base class's own `.get()` inside `super().train()`
            — see sb3_contrib's RecurrentRolloutBuffer.get(), which
            flattens tensors to (n_envs*n_steps, ...) on first call)
            and calls `self.logger.record(...)`; it changes no
            gradient, no weight, no RNG draw. Gated behind
            `train.bc_anchor_debug_adv_stats` (default 0 = skipped
            entirely, bit-exact)."""
            if not getattr(self, "bc_debug_adv_stats", False):
                return
            buf = getattr(self, "rollout_buffer", None)
            adv = getattr(buf, "advantages", None) if buf is not None \
                else None
            obs = getattr(buf, "observations", None) if buf is not None \
                else None
            if adv is None or obs is None:
                return
            try:
                from rl_move.sim.walk_task import (
                    MODE_ONEHOT_ORDER, N_MODE_OBS)
            except Exception:
                return
            adv = np.asarray(adv).reshape(-1)
            obs = np.asarray(obs).reshape(adv.shape[0], -1)
            if obs.shape[-1] < N_MODE_OBS:
                return  # obs.mode_onehot not enabled on this run
            onehot = obs[:, -N_MODE_OBS:]
            n_loco = 3  # trailing slots (walk, turn, quad); see
            # gru_policy._N_LOCO_SLOTS / walk_task.MODE_ONEHOT_ORDER
            loco_mask = onehot[:, -n_loco:].sum(axis=-1) > 0.5
            stance_mask = ~loco_mask
            for name, mask in (("loco", loco_mask), ("stance", stance_mask)):
                m = adv[mask]
                if m.size == 0:
                    continue
                self.logger.record(f"train/adv_{name}_mean", float(m.mean()))
                self.logger.record(f"train/adv_{name}_std", float(m.std()))
                self.logger.record(
                    f"train/adv_{name}_share", float(mask.mean()))
            del MODE_ONEHOT_ORDER  # imported for the module-shape check only

        def _gradnorm_diag_ctx(self):
            """DIAGNOSTIC ONLY, default off (08-27, anchor8-advstats
            REFUTATION follow-up). Returns a context manager that,
            when ``train.bc_anchor_debug_gradnorm`` is set AND the
            policy is dual-core (has ``mlp_extractor_b``), transiently
            monkeypatches ``torch.nn.utils.clip_grad_norm_`` for the
            duration of ``super().train()`` so every call captures the
            PRE-CLIP per-core grad L2 norm before delegating to the
            real clip (same tensors, same max_norm, same return value
            — zero behavioral change). On exit, logs the per-call
            values averaged over the train() call as
            train/gradnorm_{a,b,shared}_mean. Off (default) or on a
            non-dual policy: returns a no-op context, byte-for-byte
            the pre-08-27 code path (clip_grad_norm_ never touched)."""
            import contextlib

            @contextlib.contextmanager
            def _noop():
                yield

            if not getattr(self, "bc_debug_gradnorm", False):
                return _noop()
            policy = self.policy
            if not hasattr(policy, "mlp_extractor_b"):
                return _noop()

            def _group(name: str) -> str:
                if (name == "log_std_b" or "core_b" in name
                        or name.startswith(("mlp_extractor_b",
                                             "action_net_b",
                                             "value_net_b"))):
                    return "b"
                if (name == "log_std" or "core_a" in name
                        or name.startswith(("mlp_extractor.",
                                             "action_net.",
                                             "value_net."))
                        or name in ("mlp_extractor", "action_net",
                                    "value_net")):
                    return "a"
                return "shared"

            names = [n for n, _ in policy.named_parameters()]
            groups = [_group(n) for n in names]
            samples: list[tuple[float, float, float]] = []
            self_ = self

            @contextlib.contextmanager
            def _patched():
                import torch

                orig_clip = torch.nn.utils.clip_grad_norm_

                def _clip(parameters, max_norm, *a, **kw):
                    params = list(parameters)
                    sq = {"a": 0.0, "b": 0.0, "shared": 0.0}
                    for n, g, p in zip(names, groups, params):
                        if p.grad is None:
                            continue
                        sq[g] += float(p.grad.detach().pow(2).sum())
                    samples.append(
                        (sq["a"] ** 0.5, sq["b"] ** 0.5, sq["shared"] ** 0.5))
                    return orig_clip(params, max_norm, *a, **kw)

                torch.nn.utils.clip_grad_norm_ = _clip
                try:
                    yield
                finally:
                    torch.nn.utils.clip_grad_norm_ = orig_clip
                    if samples:
                        arr = np.asarray(samples)
                        self_.logger.record(
                            "train/gradnorm_a_mean", float(arr[:, 0].mean()))
                        self_.logger.record(
                            "train/gradnorm_b_mean", float(arr[:, 1].mean()))
                        if arr[:, 2].max() > 0.0:
                            self_.logger.record(
                                "train/gradnorm_shared_mean",
                                float(arr[:, 2].mean()))

            return _patched()

        def _bc_policy_mean(self, th_obs, th_h=None):
            """pi mean at the anchor obs. MLP: stateless. Recurrent
            (GRU): one fused cell step FROM the stored rollout hidden
            state — supervising the zero state instead would anchor a
            policy the rollouts never run.

            ``bc_detach_trunk`` (08-12, cw-arch-gru-anchor2 follow-up):
            when set, the feature extractor + GRU cell are run under
            no_grad and their output is .detach()-ed before the actor
            head — the anchor loss then only updates
            mlp_extractor.forward_actor + action_net, never the shared
            recurrent trunk. Motivation: cw-arch-gru-anchor2 turned OFF
            the walk-tick anchor (train.bc_anchor_walk=0, no walk pairs
            in the ring) and walk STILL froze solid identically to
            cw-arch-gru-anchor1 (gait_valid reads 6/6 but prog_ratio
            0.01, pixel-static video) while hold/lower stayed strong —
            proof the interference is not the walk-anchor loss term
            itself but the SHARED trunk gradient from rise/hold/lower
            anchor pairs bleeding into the one recurrent core every
            mode's forward pass shares. Default off, bit-exact: with
            the flag off this method is byte-for-byte the pre-08-12
            code path (same ops, same autograd graph)."""
            if th_h is None:
                return self.policy.get_distribution(
                    th_obs).distribution.mean
            import torch
            pol = self.policy
            if hasattr(pol, "bc_anchor_mean"):
                # Dual-core GRU (gru_policy.DualGruActorCriticPolicy):
                # the policy owns its mode-gated routing; it honors the
                # same detach-trunk contract (cores no_grad, heads
                # train).
                return pol.bc_anchor_mean(
                    th_obs, th_h,
                    detach_trunk=bool(
                        getattr(self, "bc_detach_trunk", False)))
            gru = pol.lstm_actor
            detach_trunk = bool(getattr(self, "bc_detach_trunk", False))
            starts = torch.zeros(len(th_obs), device=th_obs.device)
            h = th_h.reshape(len(th_obs), gru.num_layers,
                             gru.hidden_size).permute(1, 0, 2).contiguous()
            if detach_trunk:
                with torch.no_grad():
                    feats = pol.extract_features(th_obs)
                    if isinstance(feats, tuple):
                        feats = feats[0]
                    latent, _ = pol._process_sequence(
                        feats, (h, h), starts, gru)
                latent = latent.detach()
            else:
                feats = pol.extract_features(th_obs)
                if isinstance(feats, tuple):  # non-shared extractor form
                    feats = feats[0]
                latent, _ = pol._process_sequence(feats, (h, h), starts, gru)
            return pol.action_net(pol.mlp_extractor.forward_actor(latent))

        def train(self) -> None:
            with self._gradnorm_diag_ctx():
                super().train()
            self._maybe_log_adv_mode_stats()
            coef = float(getattr(self, "bc_coef", 0.0))
            n = int(getattr(self, "_bc_n", 0))
            if coef <= 0.0 or n == 0:
                return
            import torch
            import torch.nn.functional as F
            recurrent = hasattr(self, "_bc_h")
            n_mb = int(getattr(self, "bc_minibatches", 8))
            bs = min(int(getattr(self, "bc_batch_size", 4096)), n)
            dev = self.device
            rng = np.random.default_rng(self.num_timesteps)
            last = 0.0
            for _ in range(n_mb):
                idx = self._bc_sample_idx(rng, n, bs)
                th_obs = torch.as_tensor(self._bc_obs[idx], device=dev)
                th_act = torch.as_tensor(self._bc_act[idx], device=dev)
                th_h = (torch.as_tensor(self._bc_h[idx], device=dev)
                        if recurrent else None)
                mean = self._bc_policy_mean(th_obs, th_h)
                joint_mse = F.mse_loss(mean, th_act)
                loss = joint_mse
                # FOOT-HEIGHT anchor term (08-12 park audit): the
                # joint-space MSE above cannot see a mm-scale parked
                # foot (see _bc_foot_z); when enabled, additionally
                # supervise the commanded FK foot heights toward the
                # target's. fz_coef == 0 (default) skips the branch
                # entirely — legacy update sequence bit-exact.
                fz_coef = float(getattr(self, "bc_foot_z_coef", 0.0))
                if fz_coef > 0.0:
                    scale = float(getattr(
                        self, "bc_foot_z_mm", 10.0)) * 1e-3
                    dz = (_bc_foot_z(mean) - _bc_foot_z(th_act)) / scale
                    fz_loss = dz.pow(2).mean()
                    loss = joint_mse + fz_coef * fz_loss
                    last_fz = float(fz_loss.detach().cpu())
                self.policy.optimizer.zero_grad()
                (coef * loss).backward()
                if getattr(self, "bc_isolate_update", False):
                    # UPDATE isolation (08-26 stage-2 dig-in): a param
                    # the aux loss did not reach can still carry a
                    # POPULATED all-zero .grad (e.g. the dual-core
                    # policy's gated-out core: mean = 0*mu_a + 1*mu_b
                    # backprops exact zeros through the whole mu_a
                    # subgraph). Adam steps such params on pure stale
                    # momentum. Dropping the zero grads makes Adam
                    # skip them entirely (state untouched). Params
                    # with any nonzero grad are stepped identically,
                    # and clip_grad_norm_ is unaffected (zeros
                    # contribute nothing to the norm; None is
                    # filtered).
                    for p in self.policy.parameters():
                        if p.grad is not None and not p.grad.any():
                            p.grad = None
                torch.nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()
                last = float(joint_mse.detach().cpu())
            self.logger.record("train/bc_anchor_loss", last)
            if float(getattr(self, "bc_foot_z_coef", 0.0)) > 0.0:
                self.logger.record("train/bc_anchor_footz_loss", last_fz)
            self.logger.record("train/bc_anchor_fill", n)
            # Per-mode diagnostic loss (2026-08-12, pre-registered:
            # CURRENT_TRUTHS orders per-mode bc_anchor_loss logging
            # BEFORE any further stand arm — only the aggregate was
            # logged, so "the anchor itself teaches the parked foot"
            # vs "PPO ignores the hold supervision" was undecidable
            # across six identical-fingerprint runs). Runs no_grad
            # AFTER the optimizer steps: zero effect on training; the
            # rng draws here happen after every gradient minibatch was
            # sampled, so the update sequence is bit-identical.
            with torch.no_grad():
                names = {0: "rise", 1: "hold", 2: "lower", 3: "walk",
                          4: "getup", 6: "recover"}
                for m in np.unique(self._bc_mode[:n]):
                    rows = np.flatnonzero(self._bc_mode[:n] == m)
                    sel = (rows if len(rows) <= bs
                           else rng.choice(rows, size=bs, replace=False))
                    th_obs = torch.as_tensor(self._bc_obs[sel], device=dev)
                    th_act = torch.as_tensor(self._bc_act[sel], device=dev)
                    th_h = (torch.as_tensor(self._bc_h[sel], device=dev)
                            if recurrent else None)
                    mean_m = self._bc_policy_mean(th_obs, th_h)
                    l_m = float(F.mse_loss(mean_m, th_act).detach().cpu())
                    tag = names.get(int(m), str(int(m)))
                    self.logger.record(f"train/bc_anchor_loss_{tag}", l_m)
                    if float(getattr(self, "bc_foot_z_coef", 0.0)) > 0.0:
                        scale = float(getattr(
                            self, "bc_foot_z_mm", 10.0)) * 1e-3
                        dz_m = (_bc_foot_z(mean_m)
                                - _bc_foot_z(th_act)) / scale
                        self.logger.record(
                            f"train/bc_anchor_footz_loss_{tag}",
                            float(dz_m.pow(2).mean().detach().cpu()))
                    self.logger.record(f"train/bc_anchor_fill_{tag}",
                                       int(len(rows)))

    return BCAnchorPPO


def make_bc_collect_callback():
    """Callback pairing each step's ``info["bc_target"]`` with the
    post-step obs into the model's ring buffer (dones skipped — their
    new_obs already belongs to the next episode)."""
    _, BaseCallback = _lazy_sb3()

    class BCAnchorCollectCallback(BaseCallback):
        def _on_step(self) -> bool:
            infos = self.locals.get("infos", ())
            new_obs = self.locals.get("new_obs")
            dones = self.locals.get("dones")
            if new_obs is None:
                return True
            h_np = None
            if hasattr(self.model.policy, "lstm_actor"):
                # RecurrentPPO: the collect-loop local ``lstm_states``
                # is the post-forward actor state h_t — exactly the
                # state the policy holds when it next sees new_obs
                # (obs_{t+1}); the anchor must supervise the policy AT
                # that state, not at a zero state it never runs from.
                st = self.locals.get("lstm_states")
                if st is None:
                    return True
                h = st.pi[0]  # (n_layers, n_envs, hidden); GRU: c unused
                h_np = (h.permute(1, 0, 2).reshape(h.shape[1], -1)
                        .detach().cpu().numpy())
            push = self.model._bc_push
            for i, info in enumerate(infos):
                t = info.get("bc_target")
                if t is None or (dones is not None and dones[i]):
                    continue
                push(new_obs[i], t, int(info.get("bc_mode", 0)),
                     h=None if h_np is None else h_np[i])
            return True

    return BCAnchorCollectCallback()


def attach_bc_anchor(model, *, coef: float, cfg: dict | None,
                     task: str) -> None:
    """Wire the anchor onto a BCAnchorPPO model, validating the run is
    actually able to produce targets — a silently-empty buffer must
    never train (the pool-restore lesson: mechanisms that can fail
    quietly, do)."""
    from rl_move.config import cfg_get
    if task not in ("joint_goal", "joint_walk"):
        raise SystemExit(
            f"train.bc_anchor_coef set but task {task!r} is not a "
            "raw-18-joint task (only joint_goal/joint_walk emit "
            "bc_target)")
    if task == "joint_goal":
        # rise/hold supervision needs the recorded reference; the walk
        # task's reference is the scripted TripodGait (code, no file).
        ref = cfg_get(cfg, "reward", "rise_ref_path", default=None)
        if not ref:
            raise SystemExit(
                "train.bc_anchor_coef set but reward.rise_ref_path is "
                "missing — the env would never emit a bc_target and the "
                "anchor would silently no-op")
    if float(cfg_get(cfg, "train", "bc_anchor_getup", default=0.0)) > 0.0:
        # GETUP lever (08-12, cw-getup2-r1 follow-up): reuses the rise
        # reference demo, state-aligned only (no live clock makes sense
        # for an arbitrary getup start). Same silent-no-op trap as the
        # joint_goal check above, just gated by the getup flag instead
        # of the task name.
        ref = cfg_get(cfg, "reward", "rise_ref_path", default=None)
        if not ref:
            raise SystemExit(
                "train.bc_anchor_getup set but reward.rise_ref_path is "
                "missing — the env would never emit a getup bc_target "
                "and the anchor would silently no-op")
    if float(cfg_get(cfg, "train", "bc_anchor_recover", default=0.0)) > 0.0:
        ref = cfg_get(cfg, "reward", "rise_ref_path", default=None)
        if not ref:
            raise SystemExit(
                "train.bc_anchor_recover set but reward.rise_ref_path is "
                "missing — the env would never emit a recovery bc_target "
                "and the anchor would silently no-op")
    model.bc_coef = float(coef)
    model.bc_stratified = float(cfg_get(
        cfg, "train", "bc_anchor_stratified", default=0.0)) > 0.0
    model.bc_detach_trunk = float(cfg_get(
        cfg, "train", "bc_anchor_detach_trunk", default=0.0)) > 0.0
    model.bc_isolate_update = float(cfg_get(
        cfg, "train", "bc_anchor_isolate_update", default=0.0)) > 0.0
    model.bc_debug_adv_stats = float(cfg_get(
        cfg, "train", "bc_anchor_debug_adv_stats", default=0.0)) > 0.0
    model.bc_debug_gradnorm = float(cfg_get(
        cfg, "train", "bc_anchor_debug_gradnorm", default=0.0)) > 0.0
    model.bc_minibatches = int(float(cfg_get(
        cfg, "train", "bc_anchor_minibatches", default=8)))
    model.bc_batch_size = int(float(cfg_get(
        cfg, "train", "bc_anchor_batch_size", default=4096)))
    model.bc_buffer_cap = int(float(cfg_get(
        cfg, "train", "bc_anchor_buffer", default=131072)))
    model.bc_foot_z_coef = float(cfg_get(
        cfg, "train", "bc_anchor_foot_z", default=0.0))
    model.bc_foot_z_mm = float(cfg_get(
        cfg, "train", "bc_anchor_foot_z_mm", default=10.0))
