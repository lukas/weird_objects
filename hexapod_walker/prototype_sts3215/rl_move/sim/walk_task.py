"""SimHexapodJointWalkEnv — velocity-command locomotion, raw joint actions.

The most ambitious task so far: a ``walk`` goal mode where the policy is
given a commanded planar body velocity (body-frame vx/vy, m/s) and must
discover a gait that tracks it. Built on the raw joint-space env because
the fixed-foot IK cannot express stepping at all — the same structural
ceiling that blocked flat rise blocked locomotion outright.

Goal obs grows by 2 (scaled vx/vy refs, zero for non-walk modes), so
obs = 59 proprio + 11 goal = 70. All other modes (hold/lean/track/
unload/raise/rise/lower) remain in the mix so the policy keeps its
balance/rise competence while learning to walk.

Reward for walk episodes: the shared kernel/regularizer stack (zero tilt
refs — stay level while moving) PLUS a Gaussian kernel on velocity
tracking error, same design language as the tilt/height kernels:
``k_walk * exp(-|v - v_ref|^2 / 2 sigma^2)``. Velocity is the chassis'
body-frame planar velocity from privileged sim state; hardware transfer
will need an estimator (or optical flow), which is acknowledged and
deferred — sim-first, like everything else in Phase 1.

v2 (post cw-walk, which plateaued at a ~0.04 m/s shuffle):

1. MEASURED body velocity appended to the obs (2 dims, obs 70 -> 72).
   v1 was open-loop on the exact quantity it was scored on — the policy
   could never correct speed error it cannot sense.
2. PROGRESS reward: k_prog * (v . u_ref)/s_ref, capped at 1.25. The
   kernel alone pays ~nothing until tracking is already close (sigma
   0.04 at 0.075 m/s error = 0.17), so "stand still and collect the
   level-body kernel" was a local optimum. The linear term pays every
   cm/s in the commanded direction from the very first step, and
   charges moving against it.
3. Walk mix 0.40 -> 0.70; kernel sigma widened to 0.05.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from rl_move.config import cfg_get
from rl_move.env import GOAL_DIM, TaskGoal
from .joint_task import SimHexapodJointGoalEnv
from .goal_task import GoalTrajectory
from .sim_env import N_OBS

try:
    import gymnasium as _gym
except Exception:  # pragma: no cover
    _gym = None

WALK_GOAL_DIM = GOAL_DIM + 2
N_VEL_OBS = 2             # measured body-frame vx, vy appended to obs
VEL_SCALE = 0.15          # m/s; matches the max commanded speed
K_WALK = 2.0              # kernel peak, ~2x the tilt-tracking kernel
SIGMA_V = 0.05            # m/s; kernel width
# Linear progress shaping (v2 fix): measured velocity projected onto the
# commanded direction, as a fraction of the commanded speed, capped at
# 1.25. The kernel alone had no gradient from "standing" to "walking" —
# run cw-walk (5M) plateaued at a 0.04 m/s shuffle because standing
# still collected the level-body kernel risk-free while the walk kernel
# paid ~nothing until tracking was already good. Every cm/s toward the
# goal now pays immediately; moving against the command costs.
K_PROG = 1.0


@dataclass
class WalkGoal(TaskGoal):
    """TaskGoal + commanded body-frame planar velocity."""
    vx_ref: float = 0.0
    vy_ref: float = 0.0

    def as_obs(self, cfg: dict) -> np.ndarray:
        base = super().as_obs(cfg)
        return np.concatenate(
            [base, [self.vx_ref / VEL_SCALE, self.vy_ref / VEL_SCALE]])


@dataclass
class WalkTrajectory(GoalTrajectory):
    """Constant-velocity command, eased in after a settle hold."""
    vx: np.ndarray = None  # (n_steps,) m/s
    vy: np.ndarray = None

    def at(self, step: int) -> WalkGoal:
        i = min(max(step, 0), len(self.roll) - 1)
        return WalkGoal(roll_ref=float(self.roll[i]),
                        pitch_ref=float(self.pitch[i]),
                        height_ref=float(self.height[i]),
                        unload_leg=self.unload_leg,
                        vx_ref=float(self.vx[i]),
                        vy_ref=float(self.vy[i]))


def _wrap_goal(goal: TaskGoal | None) -> WalkGoal | None:
    """Give non-walk goals the widened obs with zero velocity refs."""
    if goal is None or isinstance(goal, WalkGoal):
        return goal
    return WalkGoal(roll_ref=goal.roll_ref, pitch_ref=goal.pitch_ref,
                    height_ref=goal.height_ref, unload_leg=goal.unload_leg)


class SimHexapodJointWalkEnv(SimHexapodJointGoalEnv):
    """Joint-action goal env + walk mode (obs 59 + 11 + 2 vel feedback)."""

    # Modes the periodic eval isolates for this env (train_ppo_sim reads
    # this class attribute; "lean" dropped to keep eval time bounded —
    # lean is a subset of track).
    EVAL_MODES = ("hold", "track", "unload", "raise", "rise", "walk")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Walk probability lives on the generator so the eval callback's
        # p_<mode> isolation mechanism works unchanged. 0.70: run 1's
        # 0.40 diluted the hard skill with tasks the lineage had solved.
        self._goal_gen.p_walk = 0.70
        # Swing-bonus bookkeeping (see step()): per-foot contact state
        # and world XY at the moment of liftoff.
        self._pad_bids = [self.model.body(f"L{i}_pad").id
                          for i in range(6)]
        self._foot_on = [True] * 6
        self._liftoff_xy = [None] * 6
        self._liftoff_step = [0] * 6
        if _gym is not None:
            self.observation_space = _gym.spaces.Box(
                -np.inf, np.inf,
                shape=(N_OBS - 6 + self.n_act + WALK_GOAL_DIM
                       + N_VEL_OBS,),
                dtype=np.float32)

    def _append_vel(self, obs: np.ndarray) -> np.ndarray:
        v = self._body_vel_xy() / VEL_SCALE
        return np.concatenate([obs, v]).astype(np.float32)

    def reset(self, *args, **kwargs):
        obs, info = super().reset(*args, **kwargs)
        self._foot_on = [True] * 6
        self._liftoff_xy = [None] * 6
        return self._append_vel(obs), info

    # ------------------------------------------------------------------

    def _sample_walk(self) -> WalkTrajectory:
        n = self.episode_steps + 1
        rng = self.rng
        # Command range is configurable for speed curricula: cw-walk2-gait
        # doubled stride but plateaued at ~0.045 m/s while commands ran to
        # 0.12 — mostly-unreachable commands mean the tracking kernel never
        # engages. A narrowed range makes tracking learnable first.
        s_lo = float(cfg_get(self.cfg, "goal", "walk_speed_min_m_s",
                             default=0.03))
        s_hi = float(cfg_get(self.cfg, "goal", "walk_speed_max_m_s",
                             default=0.12))
        speed = float(rng.uniform(s_lo, s_hi))
        r = rng.random()
        if r < 0.60:
            ang = 0.0                                   # forward
        elif r < 0.80:
            ang = float(rng.uniform(-math.pi / 4, math.pi / 4))
        else:
            ang = float(rng.uniform(-math.pi, math.pi))  # anywhere
        vx_t, vy_t = speed * math.cos(ang), speed * math.sin(ang)
        hold_n = max(1, int(round(1.0 / self.dt)))
        ramp_n = max(1, int(round(1.0 / self.dt)))
        vx = np.full(n, vx_t)
        vy = np.full(n, vy_t)
        vx[:hold_n] = 0.0
        vy[:hold_n] = 0.0
        end = min(hold_n + ramp_n, n)
        vx[hold_n:end] = np.linspace(0.0, vx_t, end - hold_n)
        vy[hold_n:end] = np.linspace(0.0, vy_t, end - hold_n)
        zeros = np.zeros(n)
        return WalkTrajectory(mode="walk", roll=zeros, pitch=zeros,
                              height=zeros, unload_leg=None,
                              start_at="plant", vx=vx, vy=vy)

    def _sample_goal(self):
        gen = self._goal_gen
        p_walk = float(getattr(gen, "p_walk", 0.0))
        p_base = (gen.p_hold + gen.p_lean + gen.p_track + gen.p_unload
                  + gen.p_raise + gen.p_rise + gen.p_lower)
        tot = p_walk + p_base
        if tot <= 0 or self.rng.random() < p_walk / tot:
            return self._sample_walk()
        return super()._sample_goal()

    def _current_goal(self):
        return _wrap_goal(super()._current_goal())

    def _body_vel_xy(self) -> np.ndarray:
        """Chassis planar velocity in the body frame (privileged)."""
        v_world = self.data.qvel[:3]
        R = self.data.xmat[self._chassis_bid].reshape(3, 3)
        return (R.T @ v_world)[:2]

    def step(self, action):
        obs, reward, term, trunc, info = super().step(action)
        if (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "walk"):
            goal = self._current_goal()
            v = self._body_vel_xy()
            err = float(np.hypot(v[0] - goal.vx_ref, v[1] - goal.vy_ref))
            r_walk = K_WALK * math.exp(-(err ** 2) / (2.0 * SIGMA_V ** 2))
            # Linear progress: fraction of the commanded speed achieved
            # along the commanded direction. Negative when moving against
            # the command, capped so overspeeding isn't a strategy.
            r_prog = 0.0
            s_ref = float(np.hypot(goal.vx_ref, goal.vy_ref))
            if s_ref > 1e-3:
                along = (v[0] * goal.vx_ref + v[1] * goal.vy_ref) / s_ref
                k_prog = float(cfg_get(self.cfg, "reward", "k_walk_prog",
                                       default=K_PROG))
                r_prog = k_prog * min(along / s_ref, 1.25)
            reward = float(reward) + r_walk + r_prog
            info["reward_walk"] = r_walk
            info["reward_walk_prog"] = r_prog
            info["walk_vel_err"] = err
            info["walk_speed"] = float(np.hypot(*v))
            # Swing touchdown bonus (default OFF): both cw-walk and
            # cw-walk2 plateaued at a ~0.04 m/s skate — nothing in the
            # reward ever pays for LIFTING a foot, and the smoothness
            # regularizers charge for it. Pay a one-shot bonus when a
            # foot completes a real swing (airborne, then lands >=15 mm
            # from where it lifted off) while a velocity is commanded.
            # Enable with --cfg-set reward.k_walk_swing=<k>.
            k_swing = float(cfg_get(self.cfg, "reward", "k_walk_swing",
                                    default=0.0))
            if k_swing > 0.0 and s_ref > 1e-3:
                r_swing = 0.0
                for f in range(6):
                    adr = self._touch_adr[f]
                    on = (adr >= 0 and
                          float(self.data.sensordata[adr]) > 0.5)
                    xy = self.data.xpos[self._pad_bids[f], :2]
                    if self._foot_on[f] and not on:
                        self._liftoff_xy[f] = xy.copy()
                        self._liftoff_step[f] = self._step_i
                    elif on and not self._foot_on[f] \
                            and self._liftoff_xy[f] is not None:
                        stride = float(np.linalg.norm(
                            xy - self._liftoff_xy[f]))
                        # >=2 ticks airborne filters contact chatter /
                        # settle wobble (zero-action probe scored one
                        # phantom swing without this).
                        if stride >= 0.015 \
                                and self._step_i - self._liftoff_step[f] >= 2:
                            r_swing += k_swing
                    self._foot_on[f] = on
                if r_swing:
                    reward += r_swing
                info["reward_swing"] = r_swing
        return self._append_vel(obs), reward, term, trunc, info
