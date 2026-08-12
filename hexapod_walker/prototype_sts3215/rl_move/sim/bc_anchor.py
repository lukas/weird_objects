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

Data path: sim_env._step_finish emits ``info["bc_target"]`` (float32,
18) for every rise tick with a live reference clock, every hold/track
tick, OR every commanded walk tick, when ``train.bc_anchor_coef`` > 0
(the trainer cfg key rides into the env cfg dict, same pattern as
train.mirror_loss_coef).
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

        def _bc_policy_mean(self, th_obs, th_h=None):
            """pi mean at the anchor obs. MLP: stateless. Recurrent
            (GRU): one fused cell step FROM the stored rollout hidden
            state — supervising the zero state instead would anchor a
            policy the rollouts never run."""
            if th_h is None:
                return self.policy.get_distribution(
                    th_obs).distribution.mean
            import torch
            pol = self.policy
            gru = pol.lstm_actor
            feats = pol.extract_features(th_obs)
            if isinstance(feats, tuple):  # non-shared extractor form
                feats = feats[0]
            h = th_h.reshape(len(th_obs), gru.num_layers,
                             gru.hidden_size).permute(1, 0, 2).contiguous()
            starts = torch.zeros(len(th_obs), device=th_obs.device)
            latent, _ = pol._process_sequence(feats, (h, h), starts, gru)
            return pol.action_net(pol.mlp_extractor.forward_actor(latent))

        def train(self) -> None:
            super().train()
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
                loss = F.mse_loss(mean, th_act)
                self.policy.optimizer.zero_grad()
                (coef * loss).backward()
                torch.nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()
                last = float(loss.detach().cpu())
            self.logger.record("train/bc_anchor_loss", last)
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
                names = {0: "rise", 1: "hold", 2: "lower", 3: "walk"}
                for m in np.unique(self._bc_mode[:n]):
                    rows = np.flatnonzero(self._bc_mode[:n] == m)
                    sel = (rows if len(rows) <= bs
                           else rng.choice(rows, size=bs, replace=False))
                    th_obs = torch.as_tensor(self._bc_obs[sel], device=dev)
                    th_act = torch.as_tensor(self._bc_act[sel], device=dev)
                    th_h = (torch.as_tensor(self._bc_h[sel], device=dev)
                            if recurrent else None)
                    l_m = float(F.mse_loss(
                        self._bc_policy_mean(th_obs, th_h),
                        th_act).detach().cpu())
                    tag = names.get(int(m), str(int(m)))
                    self.logger.record(f"train/bc_anchor_loss_{tag}", l_m)
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
    model.bc_coef = float(coef)
    model.bc_stratified = float(cfg_get(
        cfg, "train", "bc_anchor_stratified", default=0.0)) > 0.0
    model.bc_minibatches = int(float(cfg_get(
        cfg, "train", "bc_anchor_minibatches", default=8)))
    model.bc_batch_size = int(float(cfg_get(
        cfg, "train", "bc_anchor_batch_size", default=4096)))
    model.bc_buffer_cap = int(float(cfg_get(
        cfg, "train", "bc_anchor_buffer", default=131072)))
