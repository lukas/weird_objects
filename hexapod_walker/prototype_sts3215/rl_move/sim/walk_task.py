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


WZ_SCALE = 0.5            # rad/s; obs scale for the commanded yaw rate

# Explicit mode/command one-hot (obs.mode_onehot=1; RL_PLAN queue 2.4,
# the flagship-unified-policy prerequisite). Today the policy must
# INFER the commanded skill from the reference trajectory's shape (a
# hold and a zero-command walk look identical; rise vs lower only
# differ in the height ramp's sign, ticks later). The flagship
# multitask MDP gives the policy the mode as a direct input instead:
# a 6-wide one-hot appended at the very TAIL of each obs frame (after
# vel/phase/wz extras — same tail-append convention as wz_ref, so
# --obs-pad-transplant can still warm-start from any narrower
# champion). Default OFF: obs layout of every existing checkpoint is
# bit-exact unchanged.
#
# Slot order is FROZEN (append-only, like checkpoints): goal-mix modes
# map onto skill FAMILIES — attitude/stillness goals (hold/lean/track/
# unload) all light "hold"; raise (small up-ramp from plant) rides
# with "rise"; "turn" is RESERVED (de-scoped 08-11, no camera = no
# front) and never lit today so a future re-scope needs no width
# change. The leg one-hot (unload/lift) and vx/vy/wz refs still carry
# the WITHIN-mode command exactly as before.
MODE_ONEHOT_ORDER = ("hold", "rise", "lower", "walk", "turn", "quad")
N_MODE_OBS = len(MODE_ONEHOT_ORDER)
_MODE_FAMILY = {
    "hold": "hold", "lean": "hold", "track": "hold", "unload": "hold",
    "raise": "rise", "rise": "rise",
    "lower": "lower",
    "walk": "walk",
    # getup carries a velocity command and its own state-based stand
    # score — command-wise it is the walk family (vx/vy refs already
    # in the goal obs carry the within-mode command).
    "getup": "walk",
    "quad": "quad",
}


def mode_onehot(mode: str) -> np.ndarray:
    """6-wide skill-family one-hot for a goal-trajectory mode string.

    Unknown/missing modes map to "hold" (the zero-reference balance
    family) so an unconditioned probe can never light a motion bit.
    """
    out = np.zeros(N_MODE_OBS, dtype=float)
    fam = _MODE_FAMILY.get(str(mode), "hold")
    out[MODE_ONEHOT_ORDER.index(fam)] = 1.0
    return out


@dataclass
class WalkGoal(TaskGoal):
    """TaskGoal + commanded body-frame planar velocity (+ yaw rate)."""
    vx_ref: float = 0.0
    vy_ref: float = 0.0
    wz_ref: float = 0.0   # rad/s, +CCW; only in obs when walk_yaw_cmd=1

    def as_obs(self, cfg: dict) -> np.ndarray:
        # NOTE: the commanded yaw rate (wz_ref, walk_yaw_cmd lineage) is
        # deliberately NOT here — it is appended at the obs TAIL by
        # _augment_obs so --obs-pad-transplant 1 can warm-start a yaw
        # run from any non-yaw champion (transplant pads tail columns).
        base = super().as_obs(cfg)
        return np.concatenate(
            [base, [self.vx_ref / VEL_SCALE, self.vy_ref / VEL_SCALE]])


@dataclass
class WalkTrajectory(GoalTrajectory):
    """Constant-velocity command, eased in after a settle hold."""
    vx: np.ndarray = None  # (n_steps,) m/s
    vy: np.ndarray = None
    wz: np.ndarray = None  # (n_steps,) rad/s; None = no yaw channel

    def at(self, step: int) -> WalkGoal:
        i = min(max(step, 0), len(self.roll) - 1)
        return WalkGoal(roll_ref=float(self.roll[i]),
                        pitch_ref=float(self.pitch[i]),
                        height_ref=float(self.height[i]),
                        unload_leg=self.unload_leg,
                        lift_legs=self.lift_legs,
                        vx_ref=float(self.vx[i]),
                        vy_ref=float(self.vy[i]),
                        wz_ref=float(self.wz[i])
                        if self.wz is not None else 0.0)


def _wrap_goal(goal: TaskGoal | None) -> WalkGoal | None:
    """Give non-walk goals the widened obs with zero velocity refs."""
    if goal is None or isinstance(goal, WalkGoal):
        return goal
    return WalkGoal(roll_ref=goal.roll_ref, pitch_ref=goal.pitch_ref,
                    height_ref=goal.height_ref, unload_leg=goal.unload_leg,
                    lift_legs=goal.lift_legs)


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
                          "_anchor_xy", "_anchor_prev_on",
                          "_walk_bucket", "_step_disp_bank",
                          "_ls_prev_xy", "_ls_prev_on",
                          "_ls_slip_m", "_ls_prog_m",
                          "_yaw_still_ema", "_stance_slip_acc")

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
        # Anchored-stance income gate bookkeeping (cycle 30): per-foot
        # world XY at touchdown ("anchor point") and its own prev-contact
        # state, kept SEPARATE from the step-event vars above so the two
        # mechanisms cannot perturb each other. prev_on starts False so
        # the first loaded tick registers as a touchdown and anchors the
        # initial stance feet where they stand.
        self._anchor_xy = [None] * 6
        self._anchor_prev_on = [False] * 6
        # Displacement budget for step-event credit (cycle 34, 0-c.2):
        # net body displacement (m) accrued along the commanded
        # direction and not yet spent on step credits.
        self._step_disp_bank = 0.0
        # Heading-hold drift EMA (reward.yaw_still_avg_s); per-episode,
        # reset in _reset_begin, snapshot via MJX_SNAPSHOT_EXTRA.
        self._yaw_still_ema = 0.0
        # Loaded-slip income gate bookkeeping (operator ruling 2026-08-09
        # §3/WALK-SLIP): episode-accumulated loaded foot-XY travel and
        # along-command body progress. NEVER reset by touchdown — only
        # at episode reset — so cadence cannot re-buy an allowance.
        self._ls_prev_xy = [None] * 6
        self._ls_prev_on = [False] * 6
        self._ls_slip_m = 0.0
        self._ls_prog_m = 0.0
        # Structural stance-slip charge (2026-08-11 charge-magnitude
        # audit, probe_drag_audit.py / GAIT.md P2): accumulated loaded
        # XY travel of the CURRENT stance period per foot. Reset at
        # touchdown (a new stance earns a fresh allowance), charged
        # continuously beyond reward.drag_stance_allow_mm. Audit truth:
        # per-TICK slip cannot separate skating from honest walking
        # (medians 0.40-0.47 vs 0.31 mm overlap; the 0.5 mm deadband
        # leaves 53-97% of skating free), but per-STANCE travel splits
        # them 3.3x (learned skaters median 9.8 mm vs scripted gait
        # 2.9 mm, p90 5.7) — charge the stroke, not the jitter.
        self._stance_slip_acc = [0.0] * 6
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
        # Yaw-rate command channel (goal.walk_yaw_cmd=1): +1 goal obs
        # (scaled wz_ref via WalkGoal.as_obs). New-lineage flag — the
        # width change means no warm start from a non-yaw checkpoint.
        self._yaw_cmd = float(cfg_get(self.cfg, "goal", "walk_yaw_cmd",
                                      default=0.0)) == 1.0
        # Explicit mode/command one-hot (obs.mode_onehot=1): +6 obs at
        # the frame TAIL (see module constants). New-lineage flag like
        # walk_yaw_cmd — the width change means no direct warm start
        # from a non-mode checkpoint (--obs-pad-transplant works).
        self._mode_obs = float(cfg_get(self.cfg, "obs", "mode_onehot",
                                       default=0.0)) == 1.0
        if _gym is not None:
            self.observation_space = self._obs_space_box(
                N_OBS - 6 + self.n_act + WALK_GOAL_DIM + N_VEL_OBS
                + (N_PHASE_OBS if self._phase_obs else 0)
                + (1 if self._yaw_cmd else 0)
                + (N_MODE_OBS if self._mode_obs else 0))

    def _augment_obs(self, obs: np.ndarray, *,
                     reset: bool = False) -> np.ndarray:
        # Per-tick walk extras, applied via the base-env hook so the
        # obs-history stack (obs.history_frames) includes them in every
        # frame. goal.walk_obs_body_vel selects the source of the
        # "measured" velocity entries (obs WIDTH unchanged in all modes,
        # so checkpoints stay warm-start compatible):
        #   1.0 (default) — privileged simulator body velocity;
        #   0.0 — zeroed (policy infers velocity from qdot/gyro);
        #   2.0 — meas := ref, EXACTLY what the hardware runner feeds
        #         (board has no velocity estimate; 08-09 walk deploy).
        #         Required for deployment-equivalence arms — zeroing is
        #         a DIFFERENT contract than the robot's ref-copy.
        vel_mode = float(cfg_get(self.cfg, "goal", "walk_obs_body_vel",
                                 default=1.0))
        if vel_mode == 0.0:
            v = np.zeros(N_VEL_OBS)
        elif vel_mode == 2.0:
            goal = self._current_goal()
            v = (np.array([float(getattr(goal, "vx_ref", 0.0)),
                           float(getattr(goal, "vy_ref", 0.0))])
                 / VEL_SCALE) if goal is not None else np.zeros(N_VEL_OBS)
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
        if self._yaw_cmd:
            # Commanded yaw rate at the obs TAIL (see WalkGoal.as_obs
            # note): measured yaw rate is already in the gyro obs, so
            # only the reference is appended. Zero for non-walk goals
            # and during the settle hold.
            goal = self._current_goal()
            wz_ref = float(getattr(goal, "wz_ref", 0.0)) \
                if goal is not None else 0.0
            obs = np.concatenate([obs, [wz_ref / WZ_SCALE]])
        if self._mode_obs:
            # Skill-family one-hot, constant per episode, re-derived
            # every tick from _goal_traj (already in mjx_host.SNAP_ATTRS
            # — pool-restore safe by construction, no new episode attr).
            obs = np.concatenate([obs, mode_onehot(
                getattr(self._goal_traj, "mode", "hold")
                if self._goal_traj is not None else "hold")])
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
        self._anchor_xy = [None] * 6
        self._anchor_prev_on = [False] * 6
        self._step_disp_bank = 0.0
        self._ls_prev_xy = [None] * 6
        self._ls_prev_on = [False] * 6
        self._ls_slip_m = 0.0
        self._ls_prog_m = 0.0
        self._stance_slip_acc = [0.0] * 6
        self._phase = 0.0
        self._yaw_still_ema = 0.0
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
        # Command-heading scope (operator rulings 2026-08-09 §2/§5:
        # rear hemisphere DEFERRED; current phase is forward /
        # forward-diagonal only). goal.walk_heading_max_rad >= 0 caps
        # |heading| at that angle (0 = pure forward; pi/4 = the ruled
        # fwd-diagonal promotion scope). Default -1 = legacy 60% fwd /
        # 20% +-45deg / 20% anywhere mix, draw-stream exact.
        h_max = float(cfg_get(self.cfg, "goal", "walk_heading_max_rad",
                              default=-1.0))
        if h_max >= 0.0:
            ang = 0.0 if h_max == 0.0 \
                else float(rng.uniform(-h_max, h_max))
        else:
            r = rng.random()
            if r < 0.60:
                ang = 0.0                               # forward
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
        # Yaw-rate command (goal.walk_yaw_cmd=1; all draws gated so
        # legacy rng streams are untouched). Per segment: zero with
        # p=walk_yaw_zero_frac (heading-hold — the drift fix pays it),
        # else uniform +-walk_yaw_max_rad_s. Drawn independently of the
        # linear command, so stop segments with wz != 0 are TURN IN
        # PLACE episodes for free.
        wz = None
        wz_max = float(cfg_get(self.cfg, "goal", "walk_yaw_max_rad_s",
                               default=0.3))
        wz_zero_frac = float(cfg_get(self.cfg, "goal", "walk_yaw_zero_frac",
                                     default=0.5))

        def draw_wz() -> float:
            if rng.random() < wz_zero_frac:
                return 0.0
            return float(rng.uniform(-wz_max, wz_max))

        if self._yaw_cmd:
            wz_t = draw_wz()
            wz = np.full(n, wz_t)
            wz[:hold_n] = 0.0
            wz[hold_n:end] = np.linspace(0.0, wz_t, end - hold_n)
        # Mid-episode command resampling (operator wishlist 2026-08-09:
        # "walking around and changing direction"; default 0 = off, rng
        # stream unchanged). Every walk_cmd_resample_s seconds draw a
        # new (speed, heading) within the same scope and blend to it
        # over 1 s; with walk_stop_frac probability a segment is a full
        # stop — the policy learns start/steer/stop transitions instead
        # of one frozen command per episode.
        rs_s = float(cfg_get(self.cfg, "goal", "walk_cmd_resample_s",
                             default=0.0))
        if rs_s > 0.0:
            stop_frac = float(cfg_get(self.cfg, "goal", "walk_stop_frac",
                                      default=0.15))

            def draw_heading() -> float:
                if h_max >= 0.0:
                    return 0.0 if h_max == 0.0 \
                        else float(rng.uniform(-h_max, h_max))
                r = rng.random()
                if r < 0.60:
                    return 0.0
                if r < 0.80:
                    return float(rng.uniform(-math.pi / 4, math.pi / 4))
                return float(rng.uniform(-math.pi, math.pi))

            # Joystick realism (operator, 08-09): a human on a stick flips
            # commands at IRREGULAR intervals with near-INSTANT transitions.
            # walk_cmd_resample_jitter j draws each segment length uniform
            # in [rs_s*(1-j), rs_s*(1+j)]; walk_cmd_blend_s_min/max draw
            # each transition's blend time (default 1.0/1.0 = legacy fixed
            # 1 s ramp; set min 0.1 for flick-like flips). Defaults leave
            # every existing lineage's rng stream unchanged.
            jit = float(cfg_get(self.cfg, "goal", "walk_cmd_resample_jitter",
                                default=0.0))
            bl_lo = float(cfg_get(self.cfg, "goal", "walk_cmd_blend_s_min",
                                  default=1.0))
            bl_hi = float(cfg_get(self.cfg, "goal", "walk_cmd_blend_s_max",
                                  default=1.0))

            def seg_len() -> int:
                s = rs_s if jit <= 0.0 \
                    else rs_s * float(rng.uniform(1.0 - jit, 1.0 + jit))
                return max(1, int(round(max(s, self.dt) / self.dt)))

            def blend_len() -> int:
                b = bl_lo if bl_hi <= bl_lo \
                    else float(rng.uniform(bl_lo, bl_hi))
                return max(1, int(round(max(b, self.dt) / self.dt)))

            cvx, cvy = vx_t, vy_t
            cwz = float(wz[min(end, n - 1)]) if wz is not None else 0.0
            i = hold_n + ramp_n + seg_len()
            while i < n:
                if rng.random() < stop_frac:
                    nvx = nvy = 0.0
                else:
                    s2 = float(rng.uniform(s_lo, s_hi))
                    a2 = draw_heading()
                    nvx, nvy = s2 * math.cos(a2), s2 * math.sin(a2)
                end_b = min(i + blend_len(), n)
                vx[i:end_b] = np.linspace(cvx, nvx, end_b - i)
                vy[i:end_b] = np.linspace(cvy, nvy, end_b - i)
                vx[end_b:] = nvx
                vy[end_b:] = nvy
                cvx, cvy = nvx, nvy
                if wz is not None:
                    nwz = draw_wz()
                    wz[i:end_b] = np.linspace(cwz, nwz, end_b - i)
                    wz[end_b:] = nwz
                    cwz = nwz
                i += seg_len()
        # Commanded gait height (operator wishlist 2026-08-09: walk in a
        # HIGHER or LOWER stance; default 0 = today's nominal walk).
        # goal.walk_height_off_mm ramps the height ref alongside the
        # velocity ramp; the shared height kernel already rewards
        # tracking it (same machinery as raise/rise/lower).
        h_off = float(cfg_get(self.cfg, "goal", "walk_height_off_mm",
                              default=0.0)) / 1000.0
        height = zeros
        if h_off != 0.0:
            height = np.full(n, h_off)
            height[:hold_n] = 0.0
            height[hold_n:end] = np.linspace(0.0, h_off, end - hold_n)
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
        # Turn-in-place curriculum (operator direction 08-10: the fix
        # for the structural left drift is COMMAND EXPOSURE, not more
        # price tuning). Under independent sampling, turn-in-place
        # states are ~7.5% of segments — too rare for PPO to learn
        # the skill it is being scored on. goal.walk_turn_in_place_frac
        # (default 0 = off, rng stream unchanged): with probability f
        # the WHOLE episode becomes a dedicated turn — zero linear
        # command, guaranteed non-trivial yaw command with a 50/50
        # sign draw (both directions get equal exposure by
        # construction; the drift direction can never dominate the
        # curriculum). Applied LAST so it overrides resample segments.
        tip_frac = float(cfg_get(self.cfg, "goal",
                                 "walk_turn_in_place_frac", default=0.0))
        if self._yaw_cmd and tip_frac > 0.0 and rng.random() < tip_frac:
            vx[:] = 0.0
            vy[:] = 0.0
            mag = float(rng.uniform(0.5 * wz_max, wz_max))
            wz_t = mag if rng.random() < 0.5 else -mag
            wz = np.full(n, wz_t)
            wz[:hold_n] = 0.0
            wz[hold_n:end] = np.linspace(0.0, wz_t, end - hold_n)
            start_at = "plant"
        # Mid-stride reset diversity (TALL LADDER T6: RSI-for-walk,
        # 08-11 eve). Five reward-side arms (ref ladder, income gate,
        # gate+budget, k_height 3x/10x, speed relief) all left the
        # mid-gait posture pinned at −72..−75 mm below spawn — while
        # every episode already SPAWNS tall at the plant. The policy
        # knows tall STANDING; it has never been inside a tall
        # mid-stride WALKING state, so no pricing can select for one
        # (same shape as the park persistence above: pricing refuted →
        # densify the missing states at reset). goal.walk_gait_start_frac
        # (default 0 = off, conditional draw keeps legacy rng streams
        # bit-exact): with prob f the episode spawns MID-STRIDE in the
        # scripted tripod gait's tall pose (built env-side in sim_env
        # reset from this trajectory's command) and the walk command is
        # active from the start (0.3 s ramp, no hold — the point is
        # CONTINUING a tall walk, not re-entering it from a stand).
        gait_frac = float(cfg_get(self.cfg, "goal",
                                  "walk_gait_start_frac", default=0.0))
        if (gait_frac > 0.0 and start_at == "plant"
                and rng.random() < gait_frac):
            start_at = "gait"
            # Replace only the hold+ramp HEAD (any resampled segments
            # after it are preserved).
            ramp_fast = max(1, int(round(0.3 / self.dt)))
            head = min(max(end, ramp_fast), n)
            vx[:head] = vx_t
            vy[:head] = vy_t
            vx[:ramp_fast] = np.linspace(0.0, vx_t, ramp_fast)
            vy[:ramp_fast] = np.linspace(0.0, vy_t, ramp_fast)
            if h_off != 0.0:
                height[:head] = h_off
                height[:ramp_fast] = np.linspace(0.0, h_off, ramp_fast)
        return WalkTrajectory(mode="walk", roll=zeros, pitch=zeros,
                              height=height, unload_leg=None,
                              start_at=start_at, vx=vx, vy=vy, wz=wz)

    # GETUP start-kind mix (see _sample_getup): random legal tangle,
    # belly-zero, partial curl, crouch, plant, tripod park. The pose
    # itself is built env-side in sim_env._reset_begin ("any" branch).
    GETUP_START_KINDS = (("tangle", 0.30), ("zero", 0.20),
                         ("partial", 0.20), ("crouch", 0.10),
                         ("plant", 0.10), ("park", 0.10))

    def _sample_getup(self) -> WalkTrajectory:
        """GETUP mode (operator 08-11: "from any position I want the
        robot to get to zero pose, stand up and walk around").

        One episode = one unified recover→stand→walk MDP: spawn
        ANYWHERE along the pipeline (GETUP_START_KINDS above), a
        quiet command head (recover + stand first), then a
        joystick-style velocity schedule with stops. There are NO
        tilt/height references and no ramp schedule — all pricing is
        the state-based staged stand score in _post_step (REWARD.md
        §4b). Falls are recoverable states: runs enabling this mode
        must widen safety.max_roll/pitch_deg (e.g. 60°).

        Hooks (bank/canary only, not cfg keys): force_getup_start
        pins the start kind; force_getup_cmd=(vx, vy) replaces the
        command schedule with a constant command after a 0.5 s head.
        All draws happen regardless, so rng streams are identical
        whether or not a hook is armed.
        """
        n = self.episode_steps + 1
        rng = self.rng
        dt = self.dt
        r = rng.random()
        kind = self.GETUP_START_KINDS[-1][0]
        acc = 0.0
        for k, p in self.GETUP_START_KINDS:
            acc += p
            if r < acc:
                kind = k
                break
        force = getattr(self, "force_getup_start", None)
        if force is not None:
            kind = str(force)
        vx = np.zeros(n)
        vy = np.zeros(n)
        forced_cmd = getattr(self, "force_getup_cmd", None)
        if forced_cmd is not None:
            head = max(1, int(round(0.5 / dt)))
            vx[head:] = float(forced_cmd[0])
            vy[head:] = float(forced_cmd[1])
        else:
            # Quiet head: long enough to recover + stand from the worst
            # starts (a belly rise alone takes ~5-8 s through the servo
            # profile). Commands arriving before the robot is up simply
            # earn nothing (the S gate), so an early head is not fatal.
            q_lo = float(cfg_get(self.cfg, "goal", "getup_quiet_s_min",
                                 default=4.0))
            q_hi = float(cfg_get(self.cfg, "goal", "getup_quiet_s_max",
                                 default=8.0))
            s_lo = float(cfg_get(self.cfg, "goal", "getup_speed_min_m_s",
                                 default=0.03))
            s_hi = float(cfg_get(self.cfg, "goal", "getup_speed_max_m_s",
                                 default=0.08))
            stop_frac = float(cfg_get(self.cfg, "goal", "getup_stop_frac",
                                      default=0.35))
            seg_lo = float(cfg_get(self.cfg, "goal", "getup_seg_s_min",
                                   default=3.0))
            seg_hi = float(cfg_get(self.cfg, "goal", "getup_seg_s_max",
                                   default=6.0))
            i = max(1, int(round(float(rng.uniform(q_lo, q_hi)) / dt)))
            cvx = cvy = 0.0
            blend_n = max(1, int(round(1.0 / dt)))
            while i < n:
                if rng.random() < stop_frac:
                    tvx = tvy = 0.0
                else:
                    sp = float(rng.uniform(s_lo, s_hi))
                    ang = (0.0 if rng.random() < 0.60
                           else float(rng.uniform(-math.pi / 4,
                                                  math.pi / 4)))
                    tvx, tvy = sp * math.cos(ang), sp * math.sin(ang)
                end_b = min(i + blend_n, n)
                vx[i:end_b] = np.linspace(cvx, tvx, end_b - i)
                vy[i:end_b] = np.linspace(cvy, tvy, end_b - i)
                vx[end_b:] = tvx
                vy[end_b:] = tvy
                cvx, cvy = tvx, tvy
                i += max(1, int(round(float(
                    rng.uniform(seg_lo, seg_hi)) / dt)))
        zeros = np.zeros(n)
        traj = WalkTrajectory(mode="getup", roll=zeros, pitch=zeros,
                              height=zeros, unload_leg=None,
                              start_at="any", vx=vx, vy=vy, wz=None)
        traj.start_kind = kind
        return traj

    def _sample_goal(self):
        gen = self._goal_gen
        p_walk = float(getattr(gen, "p_walk", 0.0))
        p_getup = float(getattr(gen, "p_getup", 0.0))
        p_base = (gen.p_hold + gen.p_lean + gen.p_track + gen.p_unload
                  + gen.p_raise + gen.p_rise + gen.p_lower
                  + getattr(gen, "p_quad", 0.0))
        tot = p_walk + p_getup + p_base
        if tot <= 0:
            return self._sample_walk()
        # Single draw, walk-first cdf: with p_getup == 0 (default) the
        # draw and its use are bit-identical to the legacy two-way
        # split, so every existing lineage's rng stream is unchanged.
        r = self.rng.random() * tot
        if r < p_walk:
            return self._sample_walk()
        if r < p_walk + p_getup:
            return self._sample_getup()
        return super()._sample_goal()

    def _current_goal(self):
        return _wrap_goal(super()._current_goal())

    def _body_vel_xy(self) -> np.ndarray:
        """Chassis planar velocity in the body frame (privileged)."""
        v_world = self.data.qvel[:3]
        R = self.data.xmat[self._chassis_bid].reshape(3, 3)
        return (R.T @ v_world)[:2]

    def _body_wz(self) -> float:
        """Chassis yaw rate about the body z axis (rad/s, +CCW)."""
        w_world = self.data.qvel[3:6]
        R = self.data.xmat[self._chassis_bid].reshape(3, 3)
        return float((R.T @ w_world)[2])

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
            # Yaw-rate tracking kernel (goal.walk_yaw_cmd lineage only;
            # reward.k_walk_yaw default 0 = off). Paid in EVERY walk
            # tick, including wz_ref = 0 — heading-hold earns income, so
            # the free ~+10 deg/20 s drift of the yaw-blind lineage is
            # finally priced. NOT gated on s_ref: a stop segment with
            # wz_ref != 0 is a commanded turn in place and must pay.
            k_yaw = float(cfg_get(self.cfg, "reward", "k_walk_yaw",
                                  default=0.0))
            if self._yaw_cmd and k_yaw > 0.0:
                wz = self._body_wz()
                sig_w = float(cfg_get(self.cfg, "reward",
                                      "yaw_sigma_rad_s", default=0.15))
                yaw_err = wz - goal.wz_ref
                r_yaw = k_yaw * math.exp(
                    -(yaw_err ** 2) / (2.0 * sig_w ** 2))
                # Yaw income gated on ACHIEVED rotation (cw-walk-yawcmd1
                # dig-in, 08-10: with sigma 0.15 the ungated kernel pays
                # a command-ignoring policy exp(-.5*(0.135/0.15)^2)=0.67
                # of max income every tick — both yawcmd1 seeds learned
                # exactly that: command-invariant ~+0.09 rad/s drift,
                # turn |wz_err| med 0.24 vs gate 0.10. This was the
                # pre-registered WISHLIST item-3 risk; fix is the
                # walk_kernel_prog_gate analog: on turn segments
                # multiply yaw income by clip(wz/wz_ref, 0, 1) —
                # parked/wrong-direction earns ~0 by construction,
                # perfect tracking unchanged, over-rotation clipped
                # (the Gaussian already prices overshoot). Hold
                # segments (wz_ref=0) have no fraction-of-zero, so this
                # gate skips them — but see walk_yaw_hold_prog_gate
                # below (08-11): ungated hold income turned out to be a
                # stillness subsidy, priced by linear progress instead.
                # cfg reward.walk_yaw_kernel_gate in [0,1], default
                # 0=off (byte-identical to pre-change behavior).
                g_yaw = float(cfg_get(self.cfg, "reward",
                                      "walk_yaw_kernel_gate",
                                      default=0.0))
                if g_yaw > 0.0 and abs(goal.wz_ref) > 1e-3:
                    factor = min(max(wz / goal.wz_ref, 0.0), 1.0)
                    r_yaw *= (1.0 - g_yaw) + g_yaw * factor
                    info["walk_yaw_gate_factor"] = factor
                # Heading-hold yaw income gated on achieved LINEAR
                # progress (08-11, probe_walk_income latent-defect fix).
                # Root cause (probe, mirror2 stack): on linear-command
                # ticks (s_ref > 0, wz_ref = 0) the "hold segments stay
                # ungated" branch above pays a MOTIONLESS body full
                # heading-hold income — wz = 0 matches the zero yaw ref
                # exactly — 373-375/ep to freeze, sacrifice and paddle
                # alike, the single largest channel in the turn stack,
                # while the honest gait's natural wz oscillation earns
                # slightly less AND pays k_yaw_still. Net: the yaw stack
                # taxed honest walking ~-100/ep RELATIVE to stillness.
                # Fix is the walk_kernel_yaw_gate mirror image: on those
                # ticks multiply yaw income by achieved-progress
                # fraction clip(along/s_ref, 0, 1). A freeze earns ~0
                # heading-hold income by construction; a tracking walker
                # keeps it all. Genuine stop segments (s_ref ~ 0 AND
                # wz_ref ~ 0) stay paid — standing still with no body
                # spin IS the commanded behavior there. cfg
                # reward.walk_yaw_hold_prog_gate in [0,1], default
                # 0 = off (byte-identical legacy).
                g_hold = float(cfg_get(self.cfg, "reward",
                                       "walk_yaw_hold_prog_gate",
                                       default=0.0))
                if (g_hold > 0.0 and abs(goal.wz_ref) <= 1e-3
                        and s_ref > 1e-3):
                    factor = min(max(along / s_ref, 0.0), 1.0)
                    r_yaw *= (1.0 - g_hold) + g_hold * factor
                    info["walk_yaw_hold_factor"] = factor
                reward = float(reward) + r_yaw
                info["reward_walk_yaw"] = r_yaw
                info["walk_yaw_err"] = abs(yaw_err)
                info["walk_wz"] = wz
            # Anti-drift yaw pricing (operator, 08-10). Price
            # ESCALATION on the symmetric kernel is CLOSED (yawcmd1 /
            # yawgate1 / yawgate2: the structural ~+0.09 rad/s left
            # drift survives any kernel weight — near wz_ref=0 the
            # Gaussian's gradient at the drift point is tiny, and on
            # turn segments the kernel never goes NEGATIVE for
            # wrong-direction rotation). Two different mechanisms,
            # both default 0 = byte-identical legacy:
            #  - reward.k_yaw_prog: SIGNED rotation income, the
            #    k_walk_prog analog for turn segments: pay
            #    k * clip(wz/wz_ref, -1.5, 1.25) — constant gradient
            #    toward the commanded direction and genuinely negative
            #    when rotating against it.
            #  - reward.k_yaw_still: quadratic drift charge on
            #    heading-hold segments (wz_ref == 0): -k * wz^2. At
            #    the measured drift (0.09 rad/s) k=50 costs ~0.4/tick
            #    (real money vs the ~2/tick kernel); gyro-noise-level
            #    wz stays ~free by the square law.
            if self._yaw_cmd:
                k_yp = float(cfg_get(self.cfg, "reward", "k_yaw_prog",
                                     default=0.0))
                k_ys = float(cfg_get(self.cfg, "reward", "k_yaw_still",
                                     default=0.0))
                if k_yp > 0.0 or k_ys > 0.0:
                    wz_now = self._body_wz()
                    if k_yp > 0.0 and abs(goal.wz_ref) > 1e-3:
                        r_yp = k_yp * min(max(
                            wz_now / goal.wz_ref, -1.5), 1.25)
                        reward = float(reward) + r_yp
                        info["reward_yaw_prog"] = r_yp
                    if k_ys > 0.0 and abs(goal.wz_ref) <= 1e-3:
                        # DC-drift charge, not oscillation tax (08-11,
                        # probe_walk_income latent-defect fix). The
                        # instantaneous -k*wz^2 charged the honest
                        # gait's zero-mean stride oscillation
                        # (wz_rms ~ 0.044) about -73/ep while a frozen
                        # body paid ~0 — the charge taxed exactly the
                        # wrong policy. The drift being priced is DC
                        # (a fixed ~+0.09 rad/s offset); the gait's
                        # oscillation is zero-mean AC. Charging the
                        # EMA of wz separates them: the oscillation
                        # averages toward 0, the drift keeps its full
                        # offset. cfg reward.yaw_still_avg_s = EMA time
                        # constant in seconds, default 0 = legacy
                        # instantaneous (byte-identical). Per-episode
                        # EMA state rides MJX_SNAPSHOT_EXTRA
                        # (pool-restore lesson, commit 65edba7).
                        tau = float(cfg_get(self.cfg, "reward",
                                            "yaw_still_avg_s",
                                            default=0.0))
                        if tau > 0.0:
                            a_ema = min(self.dt / tau, 1.0)
                            self._yaw_still_ema += a_ema * (
                                wz_now - self._yaw_still_ema)
                            wz_chg = self._yaw_still_ema
                            info["yaw_still_wz_avg"] = wz_chg
                        else:
                            wz_chg = wz_now
                        r_ys = -k_ys * wz_chg * wz_chg
                        reward = float(reward) + r_ys
                        info["reward_yaw_still"] = r_ys
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
            # Achieved-yaw kernel gate for turn-in-place ticks (08-11,
            # cw-omni-mirror1-r1 freeze exploit; cfg
            # reward.walk_kernel_yaw_gate in [0,1], default 0=off).
            # Root cause (probe-confirmed): on yaw-commanded ticks with
            # NO linear command (s_ref ~ 0, wz_ref != 0) the linear
            # kernel above pays a FROZEN robot full income — v_lin = 0
            # matches ref exactly and walk_kernel_prog_gate never
            # engages (it requires s_ref > 1e-3). With turn-in-place
            # episodes at 30% of training, a park banked ~1130/ep, more
            # than the mid-training policy earned by walking (500-860)
            # — so PPO parked and the 40M run collapsed. Fix: on those
            # ticks multiply the linear kernel by achieved-yaw fraction
            # clip(wz/wz_ref, 0, 1) — the exact prog-gate analog.
            # Freeze/wrong-direction earns ~0 by construction; perfect
            # turning unchanged (factor 1); genuine stop segments
            # (s_ref ~ 0 AND wz_ref ~ 0) stay paid — standing there IS
            # the commanded behavior. Walk-mode only by construction.
            g_ykernel = float(cfg_get(self.cfg, "reward",
                                      "walk_kernel_yaw_gate",
                                      default=0.0))
            if (g_ykernel > 0.0 and self._yaw_cmd and s_ref <= 1e-3
                    and abs(goal.wz_ref) > 1e-3):
                wz_k = self._body_wz()
                factor = min(max(wz_k / goal.wz_ref, 0.0), 1.0)
                r_walk *= (1.0 - g_ykernel) + g_ykernel * factor
                info["walk_yaw_kernel_factor"] = factor
            # Anchored-stance income gate (cycle 30; the dense-
            # decomposition rung's stance-no-slip component, implemented
            # as INCOME GATING per operator 0-c.2 / step0 "worth less by
            # construction" — additive charging of slip is refuted 2x
            # (kernel-gating c24, effort c29) and a timing reference is
            # refuted (phase prior c30: agreement locked 0.93, slip
            # unmoved). A foot is ANCHORED while loaded and within
            # reward.anchor_tol_mm (default 10 mm) of its own touchdown
            # point; velocity income (kernel + positive progress) is
            # multiplied by the anchored fraction of loaded feet.
            # Paddling (all six feet creeping ~24 mm per stance) collects
            # ~0.53-0.70 of income at tol=10 (measured, controller scale
            # audit 2026-08-09); an anchored gait collects ~1.0. Negative
            # progress (moving against command) is NOT gated - the gate
            # must never shrink a penalty. Zero loaded feet => factor
            # (1-g): ballistic ticks earn no anchored income. Walk-mode
            # only by construction (this block); default OFF = legacy
            # exact. cfg: reward.walk_anchor_gate in [0,1],
            # reward.anchor_tol_mm.
            g_anchor = float(cfg_get(self.cfg, "reward",
                                     "walk_anchor_gate", default=0.0))
            if g_anchor > 0.0 and s_ref > 1e-3:
                tol_m = float(cfg_get(self.cfg, "reward",
                                      "anchor_tol_mm",
                                      default=10.0)) / 1000.0
                loaded = 0
                anchored = 0
                for f in range(6):
                    adr = self._touch_adr[f]
                    on = (adr >= 0 and
                          float(self.data.sensordata[adr]) > 0.5)
                    xy = self.data.xpos[self._pad_bids[f], :2]
                    if on and not self._anchor_prev_on[f]:
                        self._anchor_xy[f] = xy.copy()
                    elif not on:
                        self._anchor_xy[f] = None
                    self._anchor_prev_on[f] = on
                    if on and self._anchor_xy[f] is not None:
                        loaded += 1
                        if float(np.linalg.norm(
                                xy - self._anchor_xy[f])) <= tol_m:
                            anchored += 1
                frac = (anchored / loaded) if loaded > 0 else 0.0
                a_factor = (1.0 - g_anchor) + g_anchor * frac
                r_walk *= a_factor
                if r_prog > 0.0:
                    r_prog *= a_factor
                info["walk_anchor_frac"] = frac
            # Loaded-slip income gate (operator ruling 2026-08-09
            # WALK-SLIP; the structural fix for the cadence-reset
            # exploit). The anchor gate's per-touchdown allowance is an
            # accounting identity the policy exploited (free slip =
            # cadence x tol; c1 +23% stances, tol5 paid the gate).
            # This gate multiplies velocity income (kernel + positive
            # progress) by a factor of the EPISODE-ACCUMULATED loaded
            # slip per meter of along-command body progress — the same
            # quantity the eval harness scores — which no touchdown can
            # reset: factor = clip((ls_max - ratio)/(ls_max - ls_ok),
            # 0, 1); ratio = slip_m / max(progress_m, ls_floor_m).
            # Slip accumulates while a foot was in contact on the prior
            # tick (harness definition, no deadband); progress banks
            # max(along,0)*dt; both only while a velocity is commanded.
            # Never shrinks a penalty; zero-slip gait factor 1; default
            # 0 = off, legacy exact. cfg: reward.walk_loadslip_gate in
            # [0,1], reward.loadslip_ok, reward.loadslip_max,
            # reward.loadslip_floor_m.
            g_ls = float(cfg_get(self.cfg, "reward",
                                 "walk_loadslip_gate", default=0.0))
            # Measurement always runs while a velocity is commanded
            # (operator 08-10: loadslip_ratio was invisible in W&B for
            # every run that didn't enable the gate — the METRIC must
            # not be coupled to the reward MODIFIER). The gate itself
            # still only scales income when walk_loadslip_gate > 0.
            if s_ref > 1e-3:
                for f in range(6):
                    adr = self._touch_adr[f]
                    on = (adr >= 0 and
                          float(self.data.sensordata[adr]) > 0.5)
                    xy = self.data.xpos[self._pad_bids[f], :2]
                    if self._ls_prev_on[f] \
                            and self._ls_prev_xy[f] is not None:
                        self._ls_slip_m += float(np.linalg.norm(
                            xy - self._ls_prev_xy[f]))
                    self._ls_prev_xy[f] = xy.copy()
                    self._ls_prev_on[f] = on
                self._ls_prog_m += max(along, 0.0) * self.dt
                floor_m = float(cfg_get(self.cfg, "reward",
                                        "loadslip_floor_m",
                                        default=0.05))
                ls_ok = float(cfg_get(self.cfg, "reward",
                                      "loadslip_ok", default=0.75))
                ls_max = float(cfg_get(self.cfg, "reward",
                                       "loadslip_max", default=1.50))
                ratio = self._ls_slip_m / max(self._ls_prog_m, floor_m)
                factor = min(max(
                    (ls_max - ratio) / max(ls_max - ls_ok, 1e-6),
                    0.0), 1.0)
                info["walk_loadslip_ratio"] = ratio
                info["walk_loadslip_factor"] = factor
                if g_ls > 0.0:
                    ls_factor = (1.0 - g_ls) + g_ls * factor
                    r_walk *= ls_factor
                    if r_prog > 0.0:
                        r_prog *= ls_factor
            # Height-keeping income gate (2026-08-10, hardware finding
            # rl_docs/HARDWARE.md "sag": deployed walk policies migrate
            # to a crouch 54-70 mm below the spawn stance — measured on
            # the bench, knees track commands so it is COMMANDED posture,
            # not slip. The base stack's quadratic height charge
            # (env.py k_height) is ~0.36/tick at 60 mm while walk income
            # is ~3/tick, so the crouch simply outbids it. Gate the
            # income instead: multiply kernel + positive progress by a
            # Gaussian on the body's height error vs the episode ref
            # (z0-anchored, same as env.py h_err). Factor 1 at ref
            # height, 0.61 at one sigma (30 mm default), 0.14 at two;
            # symmetric so stilting up is never a strategy either.
            # Never shrinks a penalty; metric exports whenever a
            # velocity is commanded, the modifier only when enabled;
            # default 0 = off, legacy exact. cfg:
            # reward.walk_height_gate in [0,1],
            # reward.walk_height_sigma_mm.
            g_hgt = float(cfg_get(self.cfg, "reward",
                                  "walk_height_gate", default=0.0))
            if s_ref > 1e-3:
                h_err_m = (float(self.data.xpos[self._chassis_bid, 2])
                           - self._z0) - goal.height_ref
                sig_m = float(cfg_get(self.cfg, "reward",
                                      "walk_height_sigma_mm",
                                      default=30.0)) / 1000.0
                h_gauss = math.exp(
                    -0.5 * (h_err_m / max(sig_m, 1e-6)) ** 2)
                info["walk_height_factor"] = h_gauss
                if g_hgt > 0.0:
                    hgt_factor = (1.0 - g_hgt) + g_hgt * h_gauss
                    r_walk *= hgt_factor
                    if r_prog > 0.0:
                        r_prog *= hgt_factor
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
            # Structural stance-slip charge (charge-magnitude audit,
            # 2026-08-11, probe_drag_audit.py): per-foot accumulated
            # loaded XY travel per STANCE PERIOD, charged continuously
            # beyond drag_stance_allow_mm. Unlike k_drag_loaded's
            # per-tick form (audit: cannot separate skating from
            # honest touchdown scuff at ANY k/deadband), the stance
            # accumulator prices the dragging STROKE: audit-derived
            # operating point k=7000/m @ allow 6mm makes a learned
            # skater's drag cost ~2.4x its income while the honest
            # scripted gait pays on <5% of stances (~20% of income).
            # Default 0 = off, legacy exact.
            k_ds = float(cfg_get(self.cfg, "reward", "k_drag_stance",
                                 default=0.0))
            allow_m = float(cfg_get(self.cfg, "reward",
                                    "drag_stance_allow_mm",
                                    default=6.0)) / 1000.0
            # Contact-solver micro-jitter (~0.2 mm/tick on a motionless
            # loaded foot) must not integrate into the accumulator, or
            # any long stance eventually pays regardless of behavior:
            # only ticks sliding faster than this floor accumulate.
            # Dragging strokes run 0.4-0.5 mm/tick (audit medians).
            ds_floor = float(cfg_get(self.cfg, "reward",
                                     "drag_stance_tick_floor_mm",
                                     default=0.25)) / 1000.0
            # Displacement-gated step-event credit (cycle 34; operator
            # 0-c.2 "so stride-in-place can't collect", pre-registered
            # escalation from the closed tolerance rung). Root cause: the
            # step-event credit pays PER TOUCHDOWN with no body-
            # displacement accounting, so cadence inflation is the one
            # income channel that still pays for creeping transport
            # (step income +24% at c1, +32% at tol5, 2x cadence from
            # scratch at c33). Mechanism: each tick banks the body's net
            # displacement along the commanded direction; each PAID step
            # credit CONSUMES reward.step_disp_budget_mm of bank; a
            # touchdown with an empty bank earns 0. Total step income is
            # therefore <= k_step*1.5*(ground covered)/budget BY
            # CONSTRUCTION — extra cadence at fixed distance is worth
            # nothing, stride-in-place (no net body motion) collects
            # nothing. Backward motion never banks (max(along,0)); the
            # gate removes income only, never shrinks a penalty. Default
            # 0 = off, legacy exact. Walk-mode only by construction.
            budget_m = float(cfg_get(self.cfg, "reward",
                                     "step_disp_budget_mm",
                                     default=0.0)) / 1000.0
            if (k_swing > 0.0 or k_step > 0.0 or k_drag > 0.0
                    or k_park > 0.0 or k_ds > 0.0) and s_ref > 1e-3:
                if budget_m > 0.0:
                    # `along` here is still the BODY along-command
                    # velocity (m/s) from the r_prog block above (the
                    # foot-displacement loop below shadows it).
                    self._step_disp_bank += max(along, 0.0) * self.dt
                r_step_denied = 0.0
                r_swing = 0.0
                r_step = 0.0
                r_drag = 0.0
                r_ds = 0.0
                contacts = [False] * 6
                for f in range(6):
                    adr = self._touch_adr[f]
                    on = (adr >= 0 and
                          float(self.data.sensordata[adr]) > 0.5)
                    contacts[f] = on
                    xy = self.data.xpos[self._pad_bids[f], :2]
                    if on and not self._foot_on[f]:
                        # Touchdown: a new stance period earns a fresh
                        # slip allowance (k_drag_stance bookkeeping).
                        self._stance_slip_acc[f] = 0.0
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
                            along_f = float(
                                d[0] * goal.vx_ref + d[1] * goal.vy_ref
                            ) / s_ref
                            if along_f >= 0.010:
                                credit = k_step * min(along_f / 0.030, 1.5)
                                if budget_m > 0.0:
                                    if self._step_disp_bank >= budget_m:
                                        self._step_disp_bank -= budget_m
                                    else:
                                        r_step_denied += credit
                                        credit = 0.0
                                r_step += credit
                    elif on and self._foot_on[f] \
                            and (k_drag > 0.0 or k_ds > 0.0) \
                            and self._foot_prev_xy[f] is not None:
                        slip = float(np.linalg.norm(
                            xy - self._foot_prev_xy[f]))
                        if k_drag > 0.0 and slip > 0.0005:
                            r_drag -= k_drag * slip
                        if k_ds > 0.0 and slip > ds_floor:
                            acc0 = self._stance_slip_acc[f]
                            acc1 = acc0 + slip
                            self._stance_slip_acc[f] = acc1
                            # Incremental charge: integrates to
                            # k * max(stance travel - allowance, 0)
                            # per stance, paid as it accrues (a foot
                            # that never lifts cannot defer payment).
                            r_ds -= k_ds * (max(acc1 - allow_m, 0.0)
                                            - max(acc0 - allow_m, 0.0))
                    self._foot_prev_xy[f] = xy.copy()
                    self._foot_on[f] = on
                if r_swing:
                    reward += r_swing
                if k_swing > 0.0:
                    info["reward_swing"] = r_swing
                if k_step > 0.0:
                    reward += r_step
                    info["reward_step_event"] = r_step
                    if budget_m > 0.0:
                        info["walk_step_denied"] = r_step_denied
                        info["walk_step_bank_m"] = self._step_disp_bank
                if k_drag > 0.0:
                    reward += r_drag
                    info["reward_drag"] = r_drag
                if k_ds > 0.0:
                    reward += r_ds
                    info["reward_drag_stance"] = r_ds
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
            # Walk-routed effort / cost-of-transport term (cycle 27;
            # external review "additional cheap lever", adopted after
            # BOTH sanctioned checks passed: speed diagnostic found
            # 43% forward overspeed via paddling — all-six-legs
            # mid-stance loaded creep, slip ~= body travel — and the
            # per-servo current check confirmed drag ticks draw 1.38x
            # planted ticks (2.76 vs 2.01 A/leg). Root cause: the
            # objective prices physical effort at ~4% of velocity
            # income (current -13, drag -4, action -29 vs +1250/ep at
            # champion parkstart-mjx), so friction-overpowering
            # paddling wins. This charges mean servo current per tick,
            # WALK MODE ONLY (routing per review 2b) — thermal load is
            # the hardware-fatal quantity (a motor already cooked).
            # cfg reward.k_walk_effort, default 0 = off, legacy exact.
            k_eff = float(cfg_get(self.cfg, "reward", "k_walk_effort",
                                  default=0.0))
            if k_eff > 0.0:
                cur = getattr(self._state, "servo_current", None)
                r_eff = 0.0
                if cur is not None:
                    r_eff = -k_eff * float(np.mean(cur))
                    reward += r_eff
                info["reward_effort"] = r_eff
        elif (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "quad"):
            # Quad-hold shaping (feasibility GO, c57; all cfg-gated,
            # default 0 = mode earns only the base kernels). Two income
            # terms, no new charges — the level kernel, current charge
            # and tilt trip already price the failure modes:
            #   k_quad_clear: pay each LIFT leg's height above its
            #     episode-start pad z, clipped at quad_clear_cap_mm,
            #     and only while that foot is OFF the ground (a loaded
            #     "lifted" leg earns nothing by construction).
            #   k_quad_plant: pay the loaded fraction of the four
            #     support legs — the four-planted half of the task.
            # A grace window (quad_grace_s) keeps the settle + lift
            # transient unpaid so income starts only once the hold
            # could actually be happening.
            goal = self._current_goal()
            lift = tuple(goal.lift_legs) if goal.lift_legs else ()
            grace_n = int(round(float(cfg_get(
                self.cfg, "goal", "quad_grace_s", default=1.5)) / self.dt))
            k_qc = float(cfg_get(self.cfg, "reward", "k_quad_clear",
                                 default=0.0))
            k_qp = float(cfg_get(self.cfg, "reward", "k_quad_plant",
                                 default=0.0))
            if lift and self._step_i > grace_n and (k_qc > 0.0
                                                    or k_qp > 0.0):
                cap_m = float(cfg_get(self.cfg, "reward",
                                      "quad_clear_cap_mm",
                                      default=30.0)) / 1000.0
                clear_sum = 0.0
                clear_mm = 0.0
                fronts_off = 0
                for f in lift:
                    adr = self._touch_adr[f]
                    on = (adr >= 0 and
                          float(self.data.sensordata[adr]) > 0.5)
                    z_ref = (self._pad_z_ref[f]
                             if self._pad_z_ref is not None else 0.0)
                    clear = float(
                        self.data.xpos[self._pad_bids[f], 2]) - z_ref
                    clear_mm += clear * 1000.0
                    if not on:
                        fronts_off += 1
                        clear_sum += min(max(clear / cap_m, 0.0), 1.0)
                support = [f for f in range(6) if f not in lift]
                n_on = sum(
                    1 for f in support
                    if self._touch_adr[f] >= 0
                    and float(self.data.sensordata[
                        self._touch_adr[f]]) > 0.5)
                r_qc = k_qc * clear_sum / max(len(lift), 1)
                r_qp = k_qp * n_on / max(len(support), 1)
                reward = float(reward) + r_qc + r_qp
                info["reward_quad_clear"] = r_qc
                info["reward_quad_plant"] = r_qp
                info["quad_clear_mm"] = clear_mm / max(len(lift), 1)
                info["quad_fronts_off"] = fronts_off / max(len(lift), 1)
                info["quad_planted_frac"] = n_on / max(len(support), 1)
        elif getattr(self, "_is_getup", False):
            reward, info = self._getup_reward(float(reward), info)
        return obs, reward, term, trunc, info

    # ------------------------------------------------------------------
    # GETUP mode reward (08-11, from-scratch redesign; REWARD.md §4b).
    #
    # Design constraints inherited from the whole stand campaign:
    #  - the flag-leg/tripod-at-height cheat beat every height-income
    #    mechanism on warm starts; the sanctioned unexplored lever is a
    #    STRUCTURAL coupling between height and MEASURED foot contact
    #    (RL_PLAN queue 2b) — here, height only counts through the
    #    supported-stand score S = f_height * f_feet(loaded)^2 *
    #    f_level * f_footprint (fades, not hard zeros: the holdstill1
    #    zero-gradient lesson);
    #  - freezing must earn ~0 (no kernel income in this mode at all —
    #    a level belly-rest satisfies the tilt kernel for free);
    #  - income is a ONE-SHOT staged ratchet (pays only new bests of
    #    the pipeline potential P) plus GATED steady pay, so neither
    #    regressing nor re-farming a partial rise is a living;
    #  - falling is not terminal (runs widen the tilt trip): after a
    #    fall the ratchet stays banked and the incentive to get back
    #    up is the restoration of the S-gated hold/walk income.
    # ------------------------------------------------------------------

    def _getup_geom(self) -> tuple[float, float]:
        """(z_plant, weight_n): chassis height at the plant stance
        (the f_height ceiling, same construction as _place_at_plant's
        base_z) and the robot's total weight in newtons (the
        load-stage denominator). Model constants — cached once, not
        episode state."""
        if not hasattr(self, "_getup_geom_cache"):
            import mujoco_prototype as MP
            from rl_move.body_ik import fk_all_feet
            deg2rad = math.pi / 180.0
            q_p = self._plant_deg * deg2rad
            feet_p = fk_all_feet(q_p)
            z_plant = (MP.YAW_OUTPUT_HEIGHT
                       - float(np.min(feet_p[:, 2])) + MP.FOOT_R)
            weight_n = float(np.sum(self.model.body_mass)) * 9.81
            self._getup_geom_cache = (z_plant, weight_n)
        return self._getup_geom_cache

    def _getup_reward(self, reward: float, info: dict
                      ) -> tuple[float, dict]:
        # 1) Strip the base kernel + tilt shaping. The tracking kernel
        #    pays a LEVEL body ~1/tick and a flat belly-rest is level —
        #    that is an alive bonus in disguise here. The quadratic
        #    tilt shaping would likewise charge every recovery tick at
        #    the widened envelope. Regularizers (gyro/action/current)
        #    stay: they price physics, not task shape.
        r_strip = (info.get("reward_task", 0.0)
                   + info.get("reward_roll", 0.0)
                   + info.get("reward_pitch", 0.0))
        if r_strip != 0.0:
            reward -= r_strip
            info["reward_task"] = 0.0
            info["reward_roll"] = 0.0
            info["reward_pitch"] = 0.0

        # 2) Supported-stand score S in [0, 1] — the structural
        #    height<->contact coupling. Every factor is a fade with a
        #    downhill slope; only genuinely carried height scores.
        load_n = float(cfg_get(self.cfg, "reward", "getup_load_n",
                               default=1.0))
        # Graded per-foot load saturation (bank-measured: an honest
        # plant carries its light tripod at only ~0.7-1.2 N, so a hard
        # threshold reads a REAL stand as 4/6 feet). An airborne flag
        # leg contributes exactly 0 either way; grading just keeps the
        # slope (holdstill1 lesson) and reads honest stands as ~0.95.
        load_sat = 0.0
        touch_sum_n = 0.0
        for f in range(6):
            adr = self._touch_adr[f]
            if adr >= 0:
                t_n = max(float(self.data.sensordata[adr]), 0.0)
                touch_sum_n += t_n
                load_sat += min(t_n / max(load_n, 1e-6), 1.0)
        f_feet = (load_sat / 6.0) ** 2
        z_plant, weight_n = self._getup_geom()
        z_belly = float(cfg_get(self.cfg, "reward", "getup_z_belly_mm",
                                default=38.0)) * 0.001
        z = float(self.data.xpos[self._chassis_bid, 2])
        # Full height credit at a FRACTION of the rigid-FK plant span:
        # servo/contact compliance sags the physical stance ~15-25 mm
        # below the FK height (bank-measured 148.5 vs 170.8 mm), and a
        # real stand must be able to score 1.0.
        z_frac = float(cfg_get(self.cfg, "reward", "getup_z_full_frac",
                               default=0.80))
        z_full = z_belly + z_frac * max(z_plant - z_belly, 1e-3)
        f_h = min(max((z - z_belly) / max(z_full - z_belly, 1e-3),
                      0.0), 1.0)
        # Symmetric ceiling: a stilt pop overshoots the plant height —
        # fade to 0 between +20 and +80 mm above it, so "higher" is
        # never a strategy.
        over = z - (z_plant + 0.02)
        if over > 0.0:
            f_h *= min(max(1.0 - over / 0.06, 0.0), 1.0)
        t_roll, t_pitch = self._true_roll_pitch()
        lev_deg = float(cfg_get(self.cfg, "reward", "getup_level_deg",
                                default=20.0))
        tilt_deg = max(abs(t_roll), abs(t_pitch)) * 180.0 / math.pi
        f_level = min(max(1.0 - tilt_deg / max(lev_deg, 1e-6), 0.0), 1.0)
        curl = self._curl_dist()
        fp_ok = float(cfg_get(self.cfg, "reward", "getup_fp_ok_mm",
                              default=40.0)) * 0.001
        fp_hi = float(cfg_get(self.cfg, "reward", "getup_fp_hi_mm",
                              default=120.0)) * 0.001
        f_fp = min(max((fp_hi - curl) / max(fp_hi - fp_ok, 1e-6),
                       0.0), 1.0)
        # No-flag fade on the pad-height SPREAD (highest minus lowest
        # pad, world z) — ground-reference-free, so it works from
        # arbitrary spawn poses where _pad_z_ref means nothing. The
        # mean-footprint factor above barely notices ONE straightened
        # leg; the video-confirmed flag poses ride 100-160 mm of
        # spread and fade to ~0 here, while honest gait swings
        # (~25-40 mm) keep exactly 1.0. Fade over [flag, 2*flag]
        # (PLANT_SPEC.flag_leg_mm 60 -> 120), never a hard zero.
        pad_z = np.array([float(self.data.xpos[b, 2])
                          for b in self._pad_bids])
        spread = float(np.max(pad_z) - np.min(pad_z))
        flag_m = float(cfg_get(self.cfg, "reward", "getup_flag_mm",
                               default=60.0)) * 0.001
        f_flag = min(max((2.0 * flag_m - spread) / max(flag_m, 1e-6),
                         0.0), 1.0)
        s_stand = f_h * f_feet * f_level * f_fp * f_flag

        # 3) Staged pipeline potential P (weighted SUM along untangle
        #    -> weight-on-feet -> supported stand; the curl itself is
        #    unpaid but never punished — the ratchet banks bests, and
        #    the "any" start distribution backward-chains across it) +
        #    one-shot ratchet income. The baseline seeds at the
        #    episode's FIRST tick so the spawn posture is never income
        #    (the _score_best convention).
        w_z = float(cfg_get(self.cfg, "reward", "getup_w_zero",
                            default=0.15))
        w_l = float(cfg_get(self.cfg, "reward", "getup_w_load",
                            default=0.25))
        w_s = float(cfg_get(self.cfg, "reward", "getup_w_stand",
                            default=0.60))
        unt_deg = float(cfg_get(self.cfg, "reward", "getup_untangle_deg",
                                default=60.0))
        q_now = np.asarray(self.data.qpos[self._qadr], dtype=float)
        mean_q_deg = float(np.mean(np.abs(q_now))) * 180.0 / math.pi
        f_unt = min(max(1.0 - mean_q_deg / max(unt_deg, 1e-6), 0.0), 1.0)
        # Middle stage = fraction of BODY WEIGHT carried by the feet
        # (measured ground reaction, saturating at 85% of m*g — the
        # honest plant's tripod imbalance never quite reads 100%).
        # Bank-measured to be the only scalar monotone along an honest
        # rise from the crouch on (0.07 belly -> 0.68 crouch-hold ->
        # 1.0 ramp/stand): footprint distance barely moves during the
        # curl (152->130 mm) and the reference crouch is joint-wise
        # FARTHER from the plant than the zero pose is (35 vs 30 deg),
        # so both of those metrics leave the honest path a reward
        # desert. Newton caps this one at m*g — pressing harder is not
        # a strategy, and carrying weight on feet IS the task.
        f_load = min(touch_sum_n / (0.85 * weight_n), 1.0)
        p_now = w_z * f_unt + w_l * f_load + w_s * s_stand
        if self._getup_best is None:
            self._getup_best = p_now
        d_p = max(p_now - self._getup_best, 0.0)
        if d_p > 0.0:
            self._getup_best = p_now
        k_prog = float(cfg_get(self.cfg, "reward", "getup_k_progress",
                               default=60.0))
        r_prog = k_prog * d_p

        # 4) Gated steady income. Zero-command ticks: quiet-stand pay,
        #    gated hard (S^3) so partial/flagged stands earn scraps
        #    with a slope. Commanded ticks: the walk kernel + linear
        #    progress, gated by a gait-tolerant stand score (a tripod
        #    of loaded feet is full credit mid-stride) TIMES achieved
        #    progress (the walk_kernel_prog_gate lesson, baked in from
        #    the start — a parked robot earns ~0). A belly-shuffle
        #    earns ~0 through f_h regardless of its progress.
        goal = self._current_goal()
        vx_ref = float(getattr(goal, "vx_ref", 0.0)) if goal else 0.0
        vy_ref = float(getattr(goal, "vy_ref", 0.0)) if goal else 0.0
        s_ref = float(np.hypot(vx_ref, vy_ref))
        r_hold = 0.0
        r_walk = 0.0
        if s_ref <= 1e-3:
            k_hold = float(cfg_get(self.cfg, "reward", "getup_k_hold",
                                   default=0.8))
            sig_qd = float(cfg_get(self.cfg, "reward",
                                   "still_sigma_rad_s", default=0.3))
            qd2 = float(np.mean(np.square(self._state.joint_velocity)))
            still = math.exp(-qd2 / (2.0 * sig_qd ** 2))
            r_hold = k_hold * (s_stand ** 3) * still
        else:
            s_gait = (f_h * f_level * f_fp * f_flag
                      * min(load_sat / 3.0, 1.0) ** 2)
            v = self._body_vel_xy()
            err = float(np.hypot(v[0] - vx_ref, v[1] - vy_ref))
            along = (v[0] * vx_ref + v[1] * vy_ref) / s_ref
            frac = along / s_ref
            kern = (K_WALK * math.exp(-(err ** 2) / (2.0 * SIGMA_V ** 2))
                    * min(max(frac, 0.0), 1.0))
            prog = K_PROG * min(frac, 1.25)
            r_walk = s_gait * (kern + prog)
            info["walk_vel_err"] = err
            info["walk_speed"] = float(np.hypot(v[0], v[1]))
            info["getup_gait_gate"] = s_gait

        reward += r_prog + r_hold + r_walk
        info["reward_getup_prog"] = r_prog
        info["reward_getup_hold"] = r_hold
        info["reward_getup_walk"] = r_walk
        info["getup_S"] = s_stand
        info["getup_P"] = p_now
        info["getup_best"] = float(self._getup_best)
        info["getup_feet_loaded"] = load_sat
        info["getup_f_load"] = f_load
        info["getup_f_height"] = f_h
        info["getup_f_level"] = f_level
        info["getup_f_footprint"] = f_fp
        info["getup_f_flag"] = f_flag
        return reward, info
