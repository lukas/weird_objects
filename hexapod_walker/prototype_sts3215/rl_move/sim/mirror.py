"""Sagittal-plane mirror maps + symmetry-regularized PPO (MirrorPPO).

Why this exists (TURN.md, 08-10): every walk-lineage policy carries a
command-invariant ~+0.09 rad/s left-yaw drift, and EIGHT reward-side
arms across two mechanism families failed to move it. The drift is
baked into the learned gait's asymmetric limb phasing. This module
attacks the asymmetry itself: a soft auxiliary loss that pushes the
policy toward pi(mirror(obs)) == mirror(pi(obs)).

SOFT, not hard weight tying, deliberately: the body is only
APPROXIMATELY bilaterally symmetric — every leg's hip-pitch axis is
offset tangentially by the SAME signed COXA_HIP_ANCHOR_Y (-25.65 mm,
hexapod_prototype.py:5804), so the model (and the physical robot) is
a slight pinwheel. An exactly symmetric policy cannot be exactly
optimal; a regularizer lets physics win where it must. That chirality
is also a plausible physical seed for the left drift.

Mirror convention (x-z plane reflection; +x forward, +y left):

- legs reverse: L0<->L5, L1<->L4, L2<->L3 (azimuths (i+0.5)*60 deg
  reflect onto each other; all six legs are the SAME template rotated
  about z — mujoco_prototype._leg_xml)
- per leg: yaw NEGATES (axis 0 0 1 = body-up for every leg; a mirror
  flips handedness), hip/knee KEEP sign (axis 0 1 0 in the leg-local
  frame; the axis maps to MINUS the partner leg's axis, and the two
  handedness flips cancel — verified against body_ik.fk_foot_body in
  tests/test_mirror.py)
- body signals: roll, gyro x, gyro z, vy, wz negate; pitch, gyro y,
  vx, height keep sign; the unload/lift one-hot permutes with legs
- actions use the same per-joint rule (the [-1,1]->rad map is per-AXIS
  affine, identical across legs, and yaw's center is 0 — joint_task
  action_to_q_rad — so negating normalized yaw negates the target)

Obs layout mirrored here = rl_move/env.py build_obs (+ walk tail from
walk_task._augment_obs), stacked newest-first by sim_env._final_obs.
tests/test_mirror.py locks every index against those sources.
"""
from __future__ import annotations

import numpy as np

N_LEGS = 6
N_JOINTS = 18
# build_obs: q_rel 18 | qd 18 | tilt 2 | gyro 3 | prev_action 18 | goal 9
GOAL_BASE = 59          # goal block start (roll_ref)
FRAME_JOINT_GOAL = 68   # joint_goal frame width
FRAME_WALK = 72         # + [vx_ref, vy_ref] + [vx_meas, vy_meas]


def joint_perm_sign() -> tuple[np.ndarray, np.ndarray]:
    """(perm, sign) with mirror(x) = sign * x[perm] for any 18-wide
    per-joint vector (q_rel, qd, prev_action, actions)."""
    perm = np.zeros(N_JOINTS, dtype=np.int64)
    sign = np.ones(N_JOINTS, dtype=np.float32)
    for j in range(N_JOINTS):
        leg, axis = j // 3, j % 3
        perm[j] = 3 * (N_LEGS - 1 - leg) + axis
        if axis == 0:               # yaw
            sign[j] = -1.0
    return perm, sign


def frame_perm_sign(*, walk: bool, yaw_cmd: bool = False,
                    phase_obs: bool = False, mode_onehot: bool = False
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Mirror maps for ONE obs frame (no history stacking).

    walk=False -> the 68-obs joint_goal frame; walk=True -> the walk
    frame (72 + 2*phase_obs + 1*yaw_cmd + 6*mode_onehot, tail order per
    walk_task._augment_obs: vel_meas, phase, wz_ref, mode one-hot).
    The mode one-hot is mirror-INVARIANT (identity perm, +1 sign):
    every skill family — hold/rise/lower/walk/turn/quad — is
    bilaterally symmetric as a COMMAND; the leg one-hot and vy/wz refs
    already carry all the chirality.
    """
    width = FRAME_WALK if walk else FRAME_JOINT_GOAL
    if walk and phase_obs:
        width += 2
    if walk and yaw_cmd:
        width += 1
    if walk and mode_onehot:
        width += 6
    perm = np.arange(width, dtype=np.int64)
    sign = np.ones(width, dtype=np.float32)
    jp, js = joint_perm_sign()
    for base in (0, 18, 41):        # q_rel, qd, prev_action
        perm[base:base + 18] = base + jp
        sign[base:base + 18] = js
    sign[36] = -1.0                 # roll (tilt block 36:38)
    sign[38] = -1.0                 # gyro x
    sign[40] = -1.0                 # gyro z
    sign[GOAL_BASE] = -1.0          # roll_ref
    # goal one-hot legs 0..5 at 62:68 -> reversed
    perm[62:68] = 62 + (N_LEGS - 1 - np.arange(N_LEGS))
    if walk:
        sign[69] = -1.0             # vy_ref
        sign[71] = -1.0             # vy_meas
        pos = FRAME_WALK
        if phase_obs:
            # Tripod A (0,2,4) mirrors onto tripod B (5,3,1): the
            # mirrored clock is phase+pi, i.e. sin,cos both negate.
            sign[pos] = -1.0
            sign[pos + 1] = -1.0
            pos += 2
        if yaw_cmd:
            sign[pos] = -1.0        # wz_ref
        # mode one-hot tail (if any): identity perm, +1 sign — nothing
        # to write; perm/sign were initialized to identity above.
    return perm, sign


def obs_perm_sign(*, walk: bool, yaw_cmd: bool = False,
                  phase_obs: bool = False, mode_onehot: bool = False,
                  history_frames: int = 1
                  ) -> tuple[np.ndarray, np.ndarray]:
    """Full-observation maps: the frame maps tiled over the history
    stack (frames are whole obs copies, newest first)."""
    fp, fs = frame_perm_sign(walk=walk, yaw_cmd=yaw_cmd,
                             phase_obs=phase_obs, mode_onehot=mode_onehot)
    w = len(fp)
    k = max(int(history_frames), 1)
    perm = np.concatenate([i * w + fp for i in range(k)])
    sign = np.concatenate([fs] * k)
    return perm, sign


def _lazy_sb3():
    from stable_baselines3 import PPO
    return PPO


def make_mirror_ppo_class():
    """MirrorPPO: stock SB3 PPO + one auxiliary symmetry step per
    rollout.

    The aux step runs AFTER super().train() (the untouched PPO update)
    and minimizes  coef * mse( pi_mean(mirror(obs)),
                               mirror(pi_mean(obs)) )
    on minibatches drawn from the fresh rollout buffer. Implemented as
    a separate optimizer step rather than a term inside SB3's loss so
    we never copy sb3 internals (version-fragile) and the PPO
    clip/target_kl logic stays byte-identical to every other run.

    Configure by setting attributes after construction (see
    attach_mirror): model.mirror_coef, model.mirror_obs_perm/sign,
    model.mirror_act_perm/sign; mirror_minibatches (default 4) and
    mirror_batch_size (default 4096) tune the aux step.
    """
    PPO = _lazy_sb3()

    class MirrorPPO(PPO):
        mirror_coef: float = 0.0

        def train(self) -> None:
            super().train()
            coef = float(getattr(self, "mirror_coef", 0.0))
            if coef <= 0.0:
                return
            import torch
            import torch.nn.functional as F
            obs = self.rollout_buffer.observations
            flat = np.asarray(obs, dtype=np.float32).reshape(
                -1, obs.shape[-1])
            n_mb = int(getattr(self, "mirror_minibatches", 4))
            bs = min(int(getattr(self, "mirror_batch_size", 4096)),
                     flat.shape[0])
            dev = self.device
            operm = torch.as_tensor(self.mirror_obs_perm, device=dev)
            osign = torch.as_tensor(self.mirror_obs_sign, device=dev)
            aperm = torch.as_tensor(self.mirror_act_perm, device=dev)
            asign = torch.as_tensor(self.mirror_act_sign, device=dev)
            rng = np.random.default_rng(self.num_timesteps)
            last = 0.0
            for _ in range(n_mb):
                idx = rng.integers(0, flat.shape[0], size=bs)
                th_obs = torch.as_tensor(flat[idx], device=dev)
                th_mirror = th_obs[:, operm] * osign
                mean = self.policy.get_distribution(
                    th_obs).distribution.mean
                mean_m = self.policy.get_distribution(
                    th_mirror).distribution.mean
                sym = F.mse_loss(mean_m, mean[:, aperm] * asign)
                self.policy.optimizer.zero_grad()
                (coef * sym).backward()
                torch.nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()
                last = float(sym.detach().cpu())
            self.logger.record("train/mirror_sym_loss", last)

    return MirrorPPO


def attach_mirror(model, *, coef: float, task: str, cfg: dict | None,
                  obs_dim: int) -> None:
    """Wire the mirror maps onto a (Mirror)PPO model, validating the
    layout against the env's actual obs width. Raises on any mismatch
    or unsupported task so a layout drift can never train silently."""
    from rl_move.config import cfg_get
    if task not in ("joint_goal", "joint_walk"):
        raise SystemExit(
            f"train.mirror_loss_coef set but task {task!r} has no "
            "mirror maps (only joint_goal / joint_walk layouts)")
    walk = task == "joint_walk"
    yaw = walk and float(
        cfg_get(cfg, "goal", "walk_yaw_cmd", default=0.0)) == 1.0
    phase = walk and float(
        cfg_get(cfg, "goal", "walk_phase_obs", default=0.0)) == 1.0
    mode = walk and float(
        cfg_get(cfg, "obs", "mode_onehot", default=0.0)) == 1.0
    hist = int(float(
        cfg_get(cfg, "obs", "history_frames", default=1)))
    perm, sign = obs_perm_sign(walk=walk, yaw_cmd=yaw, phase_obs=phase,
                               mode_onehot=mode, history_frames=hist)
    if len(perm) != int(obs_dim):
        raise SystemExit(
            f"mirror maps expect obs dim {len(perm)} (walk={walk} "
            f"yaw={yaw} phase={phase} mode={mode} hist={hist}) but env "
            f"has {obs_dim} — obs layout drifted, fix mirror.py first")
    ap, asgn = joint_perm_sign()
    model.mirror_coef = float(coef)
    model.mirror_obs_perm = perm
    model.mirror_obs_sign = sign
    model.mirror_act_perm = ap
    model.mirror_act_sign = asgn
