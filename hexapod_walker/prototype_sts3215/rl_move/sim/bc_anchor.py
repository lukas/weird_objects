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

Logged: train/bc_anchor_loss (post-step mse of the last minibatch),
train/bc_anchor_fill (ring occupancy, pairs).
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

        def _bc_init_buffer(self, obs_dim: int) -> None:
            cap = int(getattr(self, "bc_buffer_cap", 131072))
            self._bc_obs = np.zeros((cap, obs_dim), dtype=np.float32)
            self._bc_act = np.zeros((cap, N_ACT), dtype=np.float32)
            self._bc_n = 0          # valid rows
            self._bc_i = 0          # write cursor

        def _bc_push(self, obs: np.ndarray, act: np.ndarray) -> None:
            if not hasattr(self, "_bc_obs"):
                self._bc_init_buffer(int(np.asarray(obs).shape[-1]))
            cap = self._bc_obs.shape[0]
            self._bc_obs[self._bc_i] = obs
            self._bc_act[self._bc_i] = act
            self._bc_i = (self._bc_i + 1) % cap
            self._bc_n = min(self._bc_n + 1, cap)

        def train(self) -> None:
            super().train()
            coef = float(getattr(self, "bc_coef", 0.0))
            n = int(getattr(self, "_bc_n", 0))
            if coef <= 0.0 or n == 0:
                return
            import torch
            import torch.nn.functional as F
            n_mb = int(getattr(self, "bc_minibatches", 8))
            bs = min(int(getattr(self, "bc_batch_size", 4096)), n)
            dev = self.device
            rng = np.random.default_rng(self.num_timesteps)
            last = 0.0
            for _ in range(n_mb):
                idx = rng.integers(0, n, size=bs)
                th_obs = torch.as_tensor(self._bc_obs[idx], device=dev)
                th_act = torch.as_tensor(self._bc_act[idx], device=dev)
                mean = self.policy.get_distribution(
                    th_obs).distribution.mean
                loss = F.mse_loss(mean, th_act)
                self.policy.optimizer.zero_grad()
                (coef * loss).backward()
                torch.nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()
                last = float(loss.detach().cpu())
            self.logger.record("train/bc_anchor_loss", last)
            self.logger.record("train/bc_anchor_fill", n)

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
            push = self.model._bc_push
            for i, info in enumerate(infos):
                t = info.get("bc_target")
                if t is None or (dones is not None and dones[i]):
                    continue
                push(new_obs[i], t)
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
    model.bc_minibatches = int(float(cfg_get(
        cfg, "train", "bc_anchor_minibatches", default=8)))
    model.bc_batch_size = int(float(cfg_get(
        cfg, "train", "bc_anchor_batch_size", default=4096)))
    model.bc_buffer_cap = int(float(cfg_get(
        cfg, "train", "bc_anchor_buffer", default=131072)))
