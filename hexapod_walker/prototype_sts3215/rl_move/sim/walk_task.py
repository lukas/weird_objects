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
# Learning-progress curriculum (goal.walk_lp_curriculum=1): commanded
# speed is drawn from one of these buckets instead of a single global
# uniform range. Bucket weights start uniform and are re-weighted during
# training by the LP callback in train_ppo_sim (sample where tracking is
# IMPROVING, not where it is solved or currently impossible — the manual
# global widenings to 0.07/0.08 both regressed). Default off = legacy.
LP_BUCKETS = ((0.02, 0.03), (0.03, 0.04), (0.04, 0.05), (0.05, 0.06),
              (0.06, 0.07), (0.07, 0.08), (0.08, 0.10), (0.10, 0.12))
# Phase-based alternating-tripod reward (Siekmann-style periodic reward
# composition, plan §Walk item c; enabled by goal.walk_phase_obs=1 +
# reward.k_phase_contact>0). An internal clock at goal.walk_phase_hz
# advances while a walk velocity is commanded; the policy SEES the clock
# (sin/cos appended to obs, +2 dims) and a modest reward pays contact
# states that agree with alternating tripods: PHASE_TRIPOD_A expected in
# stance while sin(phase) >= 0, the complement otherwise. NO joint
# targets, NO trajectories, NO hard constraint — a parked or dragged leg
# scores 50% agreement = zero net reward; only clock-synchronized
# stepping pays. Rationale: three penalty-style levers failed because
# PPO paid the fine rather than restructure the gait (flag, flagw,
# speedhi); this term makes stepping itself the paid behavior, densely,
# every tick.
N_PHASE_OBS = 2
PHASE_TRIPOD_A = (0, 2, 4)      # alternating tripod around the body
PHASE_HZ_DEFAULT = 1.0          # ~stride rate of the 0.02-0.06 lineage


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

    # Per-episode walk bookkeeping the batched MJX vec env must carry in
    # its pooled reset-state snapshots (see mjx_vec_env.py).
    MJX_SNAPSHOT_EXTRA = ("_foot_on", "_liftoff_xy", "_liftoff_step",
                          "_foot_prev_xy", "_duty_hist", "_phase",
                          "_walk_bucket")

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
        self._foot_prev_xy = [None] * 6
        self._duty_hist: list = []
        # Learning-progress curriculum state: sampling weights over
        # LP_BUCKETS (None = uniform) and the bucket of the current
        # walk episode (surfaced in step info for the LP callback).
        self._lp_weights = None
        self._walk_bucket = None
        # Tripod phase clock (default OFF = legacy obs width; see module
        # docstring on the phase reward). Obs order: [base, vel, phase].
        self._phase_obs = float(cfg_get(self.cfg, "goal", "walk_phase_obs",
                                        default=0.0)) == 1.0
        self._phase = 0.0
        if _gym is not None:
            self.observation_space = self._obs_space_box(
                N_OBS - 6 + self.n_act + WALK_GOAL_DIM + N_VEL_OBS
                + (N_PHASE_OBS if self._phase_obs else 0))

    def _augment_obs(self, obs: np.ndarray, *,
                     reset: bool = False) -> np.ndarray:
        # Per-tick walk extras, applied via the base-env hook so the
        # obs-history stack (obs.history_frames) includes them in every
        # frame. goal.walk_obs_body_vel=0 zeroes the privileged
        # measured-velocity entries (deployable-obs experiment: hardware
        # has no velocity sensor). Obs WIDTH is unchanged so checkpoints
        # stay warm-start compatible; the policy must infer body velocity
        # from joint velocities / gyro instead. Default 1.0 = original.
        if float(cfg_get(self.cfg, "goal", "walk_obs_body_vel",
                         default=1.0)) == 0.0:
            v = np.zeros(N_VEL_OBS)
        else:
            v = self._body_vel_xy() / VEL_SCALE
        obs = np.concatenate([obs, v])
        if self._phase_obs:
            # Phase clock advance lives here (post-physics, pre-obs) so
            # obs and the phase-agreement reward in step() see the same
            # value — behavior identical to the pre-hook code for the
            # legacy walk_phase_obs=1 runs (all refuted/retired).
            if not reset:
                goal = self._current_goal()
                s_ref = (float(np.hypot(goal.vx_ref, goal.vy_ref))
                         if goal is not None else 0.0)
                if s_ref > 1e-3:
                    hz = float(cfg_get(self.cfg, "goal", "walk_phase_hz",
                                       default=PHASE_HZ_DEFAULT))
                    self._phase = (self._phase
                                   + 2.0 * math.pi * hz * self.dt) \
                        % (2.0 * math.pi)
            obs = np.concatenate(
                [obs, [math.sin(self._phase), math.cos(self._phase)]])
        return obs.astype(np.float32)

    def _reset_begin(self, seed: int | None = None):
        # State the obs hook reads must be reset BEFORE _reset_finalize
        # builds the first observation. Done in the pre-physics reset
        # hook so both the C path (reset()) and the batched MJX vec env
        # (which drives _reset_begin directly) get it.
        self._foot_on = [True] * 6
        self._liftoff_xy = [None] * 6
        self._foot_prev_xy = [None] * 6
        self._duty_hist = []
        self._phase = 0.0
        return super()._reset_begin(seed)

    # ------------------------------------------------------------------

    def set_walk_bucket_weights(self, w) -> None:
        """LP-curriculum hook (called via VecEnv.env_method)."""
        w = np.clip(np.asarray(w, dtype=float), 0.0, None)
        s = float(w.sum())
        self._lp_weights = (w / s) if s > 0 else None

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
        self._walk_bucket = None
        if float(cfg_get(self.cfg, "goal", "walk_lp_curriculum",
                         default=0.0)) == 1.0:
            # Bucketed command sampling; weights come from the LP
            # callback via set_walk_bucket_weights (uniform until the
            # first update, and always uniform in the eval harness).
            w = self._lp_weights
            if w is None:
                w = np.full(len(LP_BUCKETS), 1.0 / len(LP_BUCKETS))
            b = int(rng.choice(len(LP_BUCKETS), p=w))
            self._walk_bucket = b
            speed = float(rng.uniform(*LP_BUCKETS[b]))
        else:
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
        # Park-basin reset diversity (goal.walk_park_start_frac, default
        # 0.0 = feature off): with probability f the episode STARTS in
        # a tripod-park posture (three hips lifted; pose built env-side
        # in sim_env reset). Rationale (cycle 24): the sto park persisted
        # at the SAME seed index through lowent->h15b->c1->kgate even
        # after kernel gating cut the park's return ~1250 -> 274/ep —
        # pricing is refuted; park-adjacent states are simply too RARE
        # (1/6 episodes, entered at t~1 s) for PPO's gradient to teach an
        # exit. Making them common at reset densifies exactly that
        # gradient. The draw is taken unconditionally so the rng stream
        # shifts identically whether or not the feature is enabled at
        # the same frac.
        park_frac = float(cfg_get(self.cfg, "goal", "walk_park_start_frac",
                                  default=0.0))
        start_at = "park" if rng.random() < park_frac else "plant"
        return WalkTrajectory(mode="walk", roll=zeros, pitch=zeros,
                              height=zeros, unload_leg=None,
                              start_at=start_at, vx=vx, vy=vy)

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

    def _post_step(self, result):
        # Walk-mode shaping — in the _post_step hook (not a step()
        # wrapper) so the batched MJX vec env, which drives the
        # begin/tick/finish halves directly, applies it too. Behavior
        # identical to the historical step() override, including on the
        # rejected-action early return.
        obs, reward, term, trunc, info = super()._post_step(result)
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
            along = 0.0
            if s_ref > 1e-3:
                along = (v[0] * goal.vx_ref + v[1] * goal.vy_ref) / s_ref
                k_prog = float(cfg_get(self.cfg, "reward", "k_walk_prog",
                                       default=K_PROG))
                r_prog = k_prog * min(along / s_ref, 1.25)
            # Progress-gated kernel income (cycle 20, cw-walk-kgate;
            # cfg reward.walk_kernel_prog_gate in [0,1], default 0=off):
            # multiply the velocity-error kernel by
            # clip(along/s_ref, 0, 1). Root cause: at commands
            # 0.02-0.06 m/s the ABSOLUTE-error kernel pays a parked
            # robot (v=0) 0.97-1.85/tick (up to 93% of peak income), so
            # the tripod park stays a paid basin (return +519 vs +1220
            # walking) that k_park_duty merely discounts. Gating income
            # on achieved progress makes the park earn ~0 kernel income
            # by construction while perfect tracking is unchanged
            # (factor 1); overspeed unaffected (clip at 1). Walk-mode
            # only by construction (this block).
            g_kernel = float(cfg_get(self.cfg, "reward",
                                     "walk_kernel_prog_gate",
                                     default=0.0))
            if g_kernel > 0.0 and s_ref > 1e-3:
                factor = min(max(along / s_ref, 0.0), 1.0)
                r_walk *= (1.0 - g_kernel) + g_kernel * factor
                info["walk_prog_factor"] = factor
            reward = float(reward) + r_walk + r_prog
            info["reward_walk"] = r_walk
            info["reward_walk_prog"] = r_prog
            info["walk_vel_err"] = err
            info["walk_speed"] = float(np.hypot(*v))
            if self._walk_bucket is not None:
                info["walk_bucket"] = self._walk_bucket
            # Tripod phase clock + contact-agreement reward (walk-routed
            # by construction; runs only while a velocity is commanded so
            # the settle hold is never charged). Parked/dragged legs
            # average 50% agreement = zero net reward; only stepping in
            # sync with the clock pays.
            if self._phase_obs and s_ref > 1e-3:
                # clock already advanced in _augment_obs (same tick)
                k_phase = float(cfg_get(self.cfg, "reward",
                                        "k_phase_contact", default=0.0))
                if k_phase > 0.0:
                    stance_a = math.sin(self._phase) >= 0.0
                    agree = 0
                    for f in range(6):
                        adr = self._touch_adr[f]
                        on = (adr >= 0 and
                              float(self.data.sensordata[adr]) > 0.5)
                        expect_on = ((f in PHASE_TRIPOD_A) == stance_a)
                        agree += int(on == expect_on)
                    r_phase = k_phase * (agree / 6.0 - 0.5) * 2.0
                    reward = float(reward) + r_phase
                    info["reward_phase_contact"] = r_phase
                    info["phase_agreement"] = agree / 6.0
            # Swing touchdown bonus (default OFF): both cw-walk and
            # cw-walk2 plateaued at a ~0.04 m/s skate — nothing in the
            # reward ever pays for LIFTING a foot, and the smoothness
            # regularizers charge for it. Pay a one-shot bonus when a
            # foot completes a real swing (airborne, then lands >=15 mm
            # from where it lifted off) while a velocity is commanded.
            # Enable with --cfg-set reward.k_walk_swing=<k>.
            k_swing = float(cfg_get(self.cfg, "reward", "k_walk_swing",
                                    default=0.0))
            # Step-event reward package (operator directive 08-08
            # ~23:00Z, queue item 0 — cw-walk-step0). Walk-mode ONLY by
            # construction (this block). Three cfg-gated terms, all
            # default OFF:
            #   k_step_event: one-shot per-leg credit for a COMPLETED
            #     lift->swing->touchdown whose displacement projects
            #     >=10 mm along the commanded direction; scaled by
            #     along/30 mm, capped at 1.5x. A parked leg never
            #     touches down -> never paid, by construction.
            #   k_drag_loaded: per-tick charge on foot XY translation
            #     while IN CONTACT (skating), 0.5 mm/tick deadband for
            #     compliance jitter.
            #   k_park_duty: per-tick charge on per-leg contact duty
            #     pinned outside [0.1, 0.9] over a trailing 2 s window
            #     of COMMANDED ticks — a tripod park (3 legs at 1.0,
            #     3 at 0.0) pays ~0.6*k every tick; a real gait
            #     (duty ~0.3-0.8) pays nothing.
            k_step = float(cfg_get(self.cfg, "reward", "k_step_event",
                                   default=0.0))
            k_drag = float(cfg_get(self.cfg, "reward", "k_drag_loaded",
                                   default=0.0))
            k_park = float(cfg_get(self.cfg, "reward", "k_park_duty",
                                   default=0.0))
            if (k_swing > 0.0 or k_step > 0.0 or k_drag > 0.0
                    or k_park > 0.0) and s_ref > 1e-3:
                r_swing = 0.0
                r_step = 0.0
                r_drag = 0.0
                contacts = [False] * 6
                for f in range(6):
                    adr = self._touch_adr[f]
                    on = (adr >= 0 and
                          float(self.data.sensordata[adr]) > 0.5)
                    contacts[f] = on
                    xy = self.data.xpos[self._pad_bids[f], :2]
                    if self._foot_on[f] and not on:
                        self._liftoff_xy[f] = xy.copy()
                        self._liftoff_step[f] = self._step_i
                    elif on and not self._foot_on[f] \
                            and self._liftoff_xy[f] is not None:
                        d = xy - self._liftoff_xy[f]
                        stride = float(np.linalg.norm(d))
                        air = self._step_i - self._liftoff_step[f]
                        # >=2 ticks airborne filters contact chatter /
                        # settle wobble (zero-action probe scored one
                        # phantom swing without this).
                        if k_swing > 0.0 and stride >= 0.015 and air >= 2:
                            r_swing += k_swing
                        if k_step > 0.0 and air >= 2:
                            along = float(
                                d[0] * goal.vx_ref + d[1] * goal.vy_ref
                            ) / s_ref
                            if along >= 0.010:
                                r_step += k_step * min(along / 0.030, 1.5)
                    elif on and self._foot_on[f] and k_drag > 0.0 \
                            and self._foot_prev_xy[f] is not None:
                        slip = float(np.linalg.norm(
                            xy - self._foot_prev_xy[f]))
                        if slip > 0.0005:
                            r_drag -= k_drag * slip
                    self._foot_prev_xy[f] = xy.copy()
                    self._foot_on[f] = on
                if r_swing:
                    reward += r_swing
                if k_swing > 0.0:
                    info["reward_swing"] = r_swing
                if k_step > 0.0:
                    reward += r_step
                    info["reward_step_event"] = r_step
                if k_drag > 0.0:
                    reward += r_drag
                    info["reward_drag"] = r_drag
                if k_park > 0.0:
                    self._duty_hist.append(
                        [1.0 if c else 0.0 for c in contacts])
                    n_win = int(round(float(cfg_get(
                        self.cfg, "goal", "park_duty_window_s",
                        default=2.0)) / self.dt))
                    if len(self._duty_hist) > n_win:
                        self._duty_hist = self._duty_hist[-n_win:]
                    r_park = 0.0
                    if len(self._duty_hist) >= n_win:
                        duty = np.mean(self._duty_hist, axis=0)
                        over = float(np.sum(
                            np.maximum(0.0, duty - 0.9)
                            + np.maximum(0.0, 0.1 - duty)))
                        r_park = -k_park * over
                        reward += r_park
                    info["reward_park_duty"] = r_park
        return obs, reward, term, trunc, info
