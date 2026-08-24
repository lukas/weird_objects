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
from rl_move.robot_state import DEG2RAD, N_JOINTS
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

# Explicit joystick command schedules. ``legacy`` preserves the historical
# independent resampler exactly; ``stress_mix`` selects one of the concrete
# schedules per episode.
WALK_CMD_SCHEDULES = (
    "random_hold", "flip_180", "sweep_circle", "square", "stop_go",
    "jitter",
)
WALK_CMD_MODE_IDS = {"legacy": 0, **{
    name: i + 1 for i, name in enumerate(WALK_CMD_SCHEDULES)
}}

# In-run command curriculum for stress_mix (08-17, operator-approved
# fb_20260817T005114 item 7): the from-scratch joystick recipe threw
# every schedule family at a newborn policy at once, four arms died
# without ever surviving takeoff. goal.walk_cmd_stage (default -1 =
# off, stress_mix draw-stream bit-exact) restricts which families
# stress_mix may draw, CUMULATIVE so earlier skills stay in the mix:
#   stage 0: forward/back stepping only (flip_180 + stop_go, heading
#            forced to 0) — learn to survive and reverse;
#   stage 1: + headings / circles / squares (random_hold,
#            sweep_circle, square), full heading scope;
#   stage >=2: the full stress_mix family set (adds jitter).
# Transitions inside an episode stay INSTANTANEOUS (blend cfg is
# untouched); ramp the stage with the sched.* in-run scheduler
# (sched.key=goal.walk_cmd_stage) so promotion is by global steps.
WALK_CMD_STAGE_FAMILIES = (
    ("flip_180", "stop_go"),
    ("random_hold", "sweep_circle", "square"),
    ("jitter",),
)


def walk_cmd_track_score(vx: float, vy: float, vx_ref: float,
                         vy_ref: float, stop_speed_m_s: float = 0.03
                         ) -> tuple[float, float, float]:
    """Normalized physical command score and its two components.

    At the requested speed and direction the score is +1. A parked body is
    -1, equal-speed cross-track motion is -2, and equal-speed motion exactly
    backward is -3. For a stop command, stillness is 0 and motion is charged
    by speed relative to ``stop_speed_m_s``.
    """
    s_ref = math.hypot(vx_ref, vy_ref)
    if s_ref <= 1e-6:
        speed = math.hypot(vx, vy)
        scale = max(float(stop_speed_m_s), 1e-6)
        return -speed / scale, 0.0, speed
    ux, uy = vx_ref / s_ref, vy_ref / s_ref
    along = vx * ux + vy * uy
    cross = abs(ux * vy - uy * vx)
    score = (along - abs(along - s_ref) - cross) / s_ref
    return score, along, cross


def walk_freeprog_score(vx: float, vy: float, vx_ref: float,
                        vy_ref: float, cap_m_s: float = 0.06
                        ) -> tuple[float, float, float]:
    """Direction-first command score with NO target speed.

    Operator order 2026-08-21 (from-scratch anti-slip walking): the
    policy must travel in the COMMANDED DIRECTION; it does not have to
    hit a commanded speed. So this score pays travel along the command
    direction, saturating at ``cap_m_s`` (faster earns the same, and is
    never charged here), and charges cross-track and backward travel on
    the same scale. +1 = along-command at or above the cap, 0 = parked,
    -1 = pure backward (or pure cross-track) at/above the cap. Unlike
    ``walk_cmd_track_score`` there is no ``|along - s_ref|`` band term:
    the commanded vector sets the DIRECTION only. For a stop command
    (``s_ref`` ~ 0) stillness is 0 and motion is charged up to -1.
    """
    cap = max(float(cap_m_s), 1e-6)
    s_ref = math.hypot(vx_ref, vy_ref)
    if s_ref <= 1e-6:
        speed = math.hypot(vx, vy)
        return -min(speed / cap, 1.0), 0.0, speed
    ux, uy = vx_ref / s_ref, vy_ref / s_ref
    along = vx * ux + vy * uy
    cross = abs(ux * vy - uy * vx)
    a = min(max(along / cap, -1.0), 1.0)
    c = min(cross / cap, 1.0)
    return a - c, along, cross


WALK_DIRECTION_MIN_SPEED_M_S = 0.01


def walk_direction_error_deg(
        vx: float, vy: float, vx_ref: float, vy_ref: float,
        min_speed_m_s: float = WALK_DIRECTION_MIN_SPEED_M_S,
) -> float | None:
    """Angle between actual and commanded planar velocity.

    Returns None for stop commands and near-stationary motion, where a
    velocity direction is undefined. Otherwise the result is in [0, 180].
    """
    cmd_speed = math.hypot(vx_ref, vy_ref)
    speed = math.hypot(vx, vy)
    if cmd_speed <= 1e-6 or speed < max(float(min_speed_m_s), 0.0):
        return None
    cos_err = (vx * vx_ref + vy * vy_ref) / (speed * cmd_speed)
    return math.degrees(math.acos(min(max(cos_err, -1.0), 1.0)))


def _add_walk_direction_info(
        info: dict, vx: float, vy: float, vx_ref: float, vy_ref: float,
        min_speed_m_s: float,
) -> None:
    """Emit active-command direction telemetry into an env info dict."""
    if math.hypot(vx_ref, vy_ref) <= 1e-3:
        return
    err = walk_direction_error_deg(
        vx, vy, vx_ref, vy_ref, min_speed_m_s=min_speed_m_s)
    info["walk_direction_valid"] = 1.0 if err is not None else 0.0
    if err is not None:
        info["walk_direction_err_deg"] = float(err)
        info["walk_direction_wrong_way"] = 1.0 if err > 90.0 else 0.0


# Learning-progress curriculum (goal.walk_lp_curriculum=1): commanded
# speed is drawn from one of these buckets instead of a single global
# uniform range. Bucket weights start uniform and are re-weighted during
# training by the LP callback in train_ppo_sim (sample where tracking is
# IMPROVING, not where it is solved or currently impossible — the manual
# global widenings to 0.07/0.08 both regressed). Default off = legacy.
LP_BUCKETS = ((0.02, 0.03), (0.03, 0.04), (0.04, 0.05), (0.05, 0.06),
              (0.06, 0.07), (0.07, 0.08), (0.08, 0.10), (0.10, 0.12))
# Adaptive competence+retention walk-command curriculum
# (goal.walk_curriculum=1; operator order 2026-08-18, run
# cw-dynrep-criticD-walkcurr1). Replaces the FIXED broad command
# sampling with a certification-gated frontier ladder: episodes draw a
# BUCKET (50% frontier / 25% weakest mastered / 15% uniform mastered /
# 10% the rung just prior to the frontier), never a locked future
# bucket, and never promote on time — only on a deterministic held-out
# certification pass of the frontier AND every retained bucket
# (apply_walkcurr_certification / walkcurr_update_admission, driven by
# the trainer exactly like the recover-mode ladder). Default 0 = off,
# bit-exact legacy: no rng draws, no randomizer swap, no cfg reads
# beyond __init__.
#   fields: s_lo/s_hi commanded speed band (m/s); head_lo/head_hi
#   heading magnitude band (rad, sign drawn ±; 0/0 = pure forward);
#   resample_s mid-episode command resampling period (0 = one command
#   held the whole episode = "long holds"); jitter/stop_frac/blend as
#   the legacy goal.walk_cmd_* keys; dr = this bucket's DR scale (the
#   env swaps its DomainRandomizer per episode); stop_gate = cert-time
#   max mean measured speed (m/s) during commanded-stop ticks (None =
#   bucket has no stop segments to gate).
WALKCURR_BUCKETS = (
    # B0 slow forward, long holds, DR0
    dict(name="fwd_slow", s_lo=0.04, s_hi=0.05, head_lo=0.0, head_hi=0.0,
         resample_s=0.0, jitter=0.0, stop_frac=0.0, blend_lo=1.0,
         blend_hi=1.0, dr=0.0, stop_gate=None),
    # B1 forward speed band widens
    dict(name="fwd_band", s_lo=0.03, s_hi=0.06, head_lo=0.0, head_hi=0.0,
         resample_s=0.0, jitter=0.0, stop_frac=0.0, blend_lo=1.0,
         blend_hi=1.0, dr=0.0, stop_gate=None),
    # B2-B4 heading cones open ±15/±30/±45 deg
    dict(name="head15", s_lo=0.03, s_hi=0.06, head_lo=0.0,
         head_hi=math.radians(15.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None),
    dict(name="head30", s_lo=0.03, s_hi=0.06, head_lo=0.0,
         head_hi=math.radians(30.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None),
    dict(name="head45", s_lo=0.03, s_hi=0.06, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None),
    # B5 blended front-cone transitions (no stops yet)
    dict(name="front_blend", s_lo=0.03, s_hi=0.06, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=6.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None),
    # B6 the joystick mix: 4s segments + jitter + stop/restart
    dict(name="stop_restart", s_lo=0.03, s_hi=0.06, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=0.015),
    # B7/B8 same commands under DR 0.1 then 0.3
    dict(name="dr01", s_lo=0.03, s_hi=0.06, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=1.0, blend_hi=1.0, dr=0.1,
         stop_gate=0.015),
    dict(name="dr03", s_lo=0.03, s_hi=0.06, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=1.0, blend_hi=1.0, dr=0.3,
         stop_gate=0.015),
    # B9/B10 lateral then rear/reverse — locked until every earlier
    # rung certifies AND retains (operator: "lateral/rear/reverse only
    # after retained passes").
    dict(name="lateral", s_lo=0.03, s_hi=0.06,
         head_lo=math.radians(45.0), head_hi=math.radians(90.0),
         resample_s=4.0, jitter=0.5, stop_frac=0.15, blend_lo=1.0,
         blend_hi=1.0, dr=0.3, stop_gate=0.015),
    dict(name="rear", s_lo=0.03, s_hi=0.06,
         head_lo=math.radians(90.0), head_hi=math.pi,
         resample_s=4.0, jitter=0.5, stop_frac=0.15, blend_lo=1.0,
         blend_hi=1.0, dr=0.3, stop_gate=0.015),
)

# WALKCURR_BUCKETS_V2 (walkcurr2, operator MCP note fb_20260818T060044,
# "figure out how to make a great run and then launch it"): corrects
# two root causes the matched walkcurr1-vs-40m1 data exposed in V1's
# B0/B1 (see cw-dynrep-criticD-40m1 triage + walkcurr1 in-flight reads).
#
# Root cause 1: V1's B0 command band (0.04-0.05 m/s, dead ahead) sits
# ENTIRELY inside SIGMA_V=0.05 (walk_task's velocity-tracking kernel
# width) of a PARKED (zero-output) robot — standing still nets
# exp(-(0.045/0.05)^2/2) = ~67% of peak reward, so PPO has little
# incentive to ever start walking before B0 can certify. V2's B0
# ("ignition") instead commands 0.08-0.12 m/s with a small heading
# spread from the very first bucket (a parked policy is now 1.6-2.4
# sigma off target, i.e. <15% of peak reward) — command diversity from
# step 0, not deferred to later rungs.
#
# Root cause 2: V1's gate (WALKCURR_GATE, train_ppo_transfer.py)
# applies slew_sat<=0.5 as a hard admission check on every bucket. The
# best real evidence of what a good policy at this budget looks like —
# cw-dynrep-criticD-40m1's retained 6M-best checkpoint, matched task,
# matched budget-ish, matched everything but curriculum — runs
# slew_sat~0.925, i.e. it would FAIL V1's own admission gate outright.
# A hard bar the best known-good policy cannot clear is not a quality
# floor, it is a wall between "parked" (low slew) and "walking" (high
# slew, because directional command changes cost joint-speed). V2
# raises slew_sat_max to 0.95 (WALKCURR_GATE_V2_*, monitored/
# selection-relevant like every other quality metric here, not
# vetoing) and otherwise keeps V1's floors (progress/slip/roll) intact
# — tightened, if anything, on later buckets via the per-bucket "gate"
# key (walkcurr_bucket_pass reads spec["gate"] when present).
#
# Buckets B2+ reuse V1's heading/resample/DR ladder verbatim (proven
# shape, not the thing that broke); only B0/B1 (ignition speed +
# immediate heading spread) and the gate calibration change.
WALKCURR_GATE_V2_IGNITION = dict(
    cmd_prog_frac_min=0.65, slip_per_m_max=2.5, peak_roll_deg_max=8.0,
    slew_sat_max=0.95, cross_track_frac_max=0.30,
    contact_sw_per_s_min=3.0, foot_sw_min_per_s_min=0.5)
WALKCURR_GATE_V2_QUALITY = dict(
    cmd_prog_frac_min=0.75, slip_per_m_max=2.0, peak_roll_deg_max=6.0,
    slew_sat_max=0.95, cross_track_frac_max=0.30,
    contact_sw_per_s_min=3.0, foot_sw_min_per_s_min=0.5)
WALKCURR_BUCKETS_V2 = (
    # B0 ignition: real speed + heading spread from step 0 (root cause 1)
    dict(name="ignition", s_lo=0.08, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(15.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None, gate=WALKCURR_GATE_V2_IGNITION),
    # B1 speed band widens toward the legacy range, heading unchanged
    dict(name="quality_band", s_lo=0.06, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(15.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None, gate=WALKCURR_GATE_V2_QUALITY),
    dict(name="full_band_head15", s_lo=0.03, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(15.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None, gate=WALKCURR_GATE_V2_QUALITY),
    dict(name="head30", s_lo=0.03, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(30.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None, gate=WALKCURR_GATE_V2_QUALITY),
    dict(name="head45", s_lo=0.03, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None, gate=WALKCURR_GATE_V2_QUALITY),
    # B5 blended front-cone transitions (no stops yet)
    dict(name="front_blend", s_lo=0.03, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=6.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None, gate=WALKCURR_GATE_V2_QUALITY),
    # B6 the joystick mix: 4s segments + jitter + stop/restart
    dict(name="stop_restart", s_lo=0.03, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V2_QUALITY),
    # B7/B8 same commands under DR 0.1 then 0.3
    dict(name="dr01", s_lo=0.03, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=1.0, blend_hi=1.0, dr=0.1,
         stop_gate=0.015, gate=WALKCURR_GATE_V2_QUALITY),
    dict(name="dr03", s_lo=0.03, s_hi=0.12, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=1.0, blend_hi=1.0, dr=0.3,
         stop_gate=0.015, gate=WALKCURR_GATE_V2_QUALITY),
    # B9/B10 lateral then rear/reverse — locked until every earlier
    # rung certifies AND retains
    dict(name="lateral", s_lo=0.03, s_hi=0.12,
         head_lo=math.radians(45.0), head_hi=math.radians(90.0),
         resample_s=4.0, jitter=0.5, stop_frac=0.15, blend_lo=1.0,
         blend_hi=1.0, dr=0.3, stop_gate=0.015,
         gate=WALKCURR_GATE_V2_QUALITY),
    dict(name="rear", s_lo=0.03, s_hi=0.12,
         head_lo=math.radians(90.0), head_hi=math.pi,
         resample_s=4.0, jitter=0.5, stop_frac=0.15, blend_lo=1.0,
         blend_hi=1.0, dr=0.3, stop_gate=0.015,
         gate=WALKCURR_GATE_V2_QUALITY),
)
# WALKCURR_BUCKETS_V3 ("bridge" ladder, operator order
# fb_20260818T102844_116d4c, walkcurr4 evidence-based correction):
# ignition must be ADJACENT TO THE TRANSPLANTED SOURCE SKILL, not to a
# park-proof speed band. The gaitinit canaries showed the two failure
# directions of V2's ignition on an actor-init recipe: a scratch actor
# reaches cmd_prog 0.74 but crouches/falls (canB-r1 at 2.1M), while the
# proven tall-walk actor keeps posture/safety but LOSES commanded
# travel chasing V2's 0.08-0.12 m/s band it never walked at
# (gaitinit-bcinit final cert: cmd_prog_frac 0.0575, height_factor
# 0.5847). V3's first two rungs are a SHORT BRIDGE at the source
# checkpoint's own operating point (ppo_goal_cw_dep_bcgait1_hard1
# walks ~0.05 m/s straight, height -12.7mm, slip 1.3, zero falls):
# B0 = straight forward 0.05-0.06 m/s, NO heading jitter / resampling /
# stops, DR0; B1 widens speed to 0.05-0.10 still dead straight; every
# later rung is V2's own direction/heading/stop/DR ladder verbatim, so
# the eventual multi-direction joystick goal is preserved. NOTE
# V3 B0 deliberately sits near the SIGMA_V park zone V2 removed — that
# is safe ONLY on an actor-init recipe whose init already walks
# (pre-PPO deterministic cert required: --walkcurr-cert-at-init), and
# is why V3 is not a scratch-acquisition ladder.
WALKCURR_GATE_V3_BRIDGE = dict(
    cmd_prog_frac_min=0.60, slip_per_m_max=2.0, peak_roll_deg_max=8.0,
    slew_sat_max=0.95, cross_track_frac_max=0.30,
    contact_sw_per_s_min=3.0, foot_sw_min_per_s_min=0.5)
WALKCURR_BUCKETS_V3 = (
    # B0 bridge: the source gait's own command, held all episode, DR0
    dict(name="bridge_fwd", s_lo=0.05, s_hi=0.06, head_lo=0.0,
         head_hi=0.0, resample_s=0.0, jitter=0.0, stop_frac=0.0,
         blend_lo=1.0, blend_hi=1.0, dr=0.0, stop_gate=None,
         gate=WALKCURR_GATE_V3_BRIDGE),
    # B1 bridge: speed band widens upward, still dead straight
    dict(name="bridge_band", s_lo=0.05, s_hi=0.10, head_lo=0.0,
         head_hi=0.0, resample_s=0.0, jitter=0.0, stop_frac=0.0,
         blend_lo=1.0, blend_hi=1.0, dr=0.0, stop_gate=None,
         gate=WALKCURR_GATE_V3_BRIDGE),
) + WALKCURR_BUCKETS_V2[2:]   # V2 direction/heading/stop/DR ladder

# WALKCURR_BUCKETS_V4: sustained joystick curriculum. V3 spends its
# first five rungs on one fixed command per 10-second episode, even
# though the observed deployment failure is falling only after several
# direction changes. V4 keeps one short source-skill bridge, introduces
# command changes immediately at B1, then holds the command distribution
# fixed while extending continuous survival to 20/40/60 seconds. Only
# after the 60-second joystick task is retained does it widen speed, add
# DR, and open lateral/rear travel. ``duration_s`` controls both training
# episode truncation and the deterministic certification horizon;
# ``min_command_changes`` is a fail-closed check that a cert actually
# exercised the intended joystick transitions.
WALKCURR_GATE_V4_BRIDGE = dict(
    cmd_prog_frac_min=0.60, cmd_prog_frac_p10_min=0.50,
    slip_per_m_max=2.0, peak_roll_deg_max=8.0, slew_sat_max=0.95,
    cross_track_frac_max=0.30, contact_sw_per_s_min=3.0,
    foot_sw_min_per_s_min=0.5, height_factor_min=0.80)
WALKCURR_GATE_V4_JOYSTICK = dict(
    cmd_prog_frac_min=0.65, cmd_prog_frac_p10_min=0.50,
    slip_per_m_max=2.0, peak_roll_deg_max=8.0, slew_sat_max=0.95,
    cross_track_frac_max=0.30, contact_sw_per_s_min=3.0,
    foot_sw_min_per_s_min=0.5, height_factor_min=0.80)

WALKCURR_BUCKETS_V4 = (
    # B0: prove the imported gait survives at its own operating point.
    dict(name="bridge_10s", duration_s=10.0, min_command_changes=0,
         s_lo=0.05, s_hi=0.06, head_lo=0.0, head_hi=0.0,
         resample_s=0.0, jitter=0.0, stop_frac=0.0, blend_lo=1.0,
         blend_hi=1.0, dr=0.0, stop_gate=None,
         gate=WALKCURR_GATE_V4_BRIDGE),
    # B1-B4: joystick changes happen immediately, then only duration grows.
    dict(name="joystick_10s", duration_s=10.0, min_command_changes=2,
         s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    dict(name="joystick_20s", duration_s=20.0, min_command_changes=4,
         s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    dict(name="joystick_40s", duration_s=40.0, min_command_changes=10,
         s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    dict(name="joystick_60s", duration_s=60.0, min_command_changes=15,
         s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    # B5: widen speed and make joystick timing less predictable.
    dict(name="full_band_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.03, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    # B6/B7: retain the full 60-second joystick task under DR.
    dict(name="dr01_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.03, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.1,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    dict(name="dr03_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.03, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.3,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    # B8/B9: lateral and rear commands remain locked behind retained
    # 60-second front-cone competence under DR0.3.
    dict(name="lateral_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.03, s_hi=0.10, head_lo=math.radians(45.0),
         head_hi=math.radians(90.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.3,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
    dict(name="rear_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.03, s_hi=0.10, head_lo=math.radians(90.0),
         head_hi=math.pi, resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.3,
         stop_gate=0.015, gate=WALKCURR_GATE_V4_JOYSTICK),
)

# WALKCURR_BUCKETS_V5 ("fast anti-skate" ladder, operator order
# fb_20260820T075230_4a90c6, recreated on the controller because the
# desktop commit 2cb2a7b7 could not be pushed): the fast-gait
# profile-headroom fork (steer6-fasttrack1 full-dose, steer7-middose1
# half-dose) FAILED on skating/slip and non-monotone speed control —
# both kept a single weakly-scaling cadence and paid slip 1.6-5.1/m.
# V5 is a fast-profile ladder ADJACENT to the proven tall-walk source
# (ppo_goal_cw_dep_bcgait1_hard1, walks ~0.05-0.06 m/s straight):
# B0 is a strict 10s bridge at the source's own operating point, B1/B2
# push the commanded band up to 0.08-0.10 / 0.06-0.10 with a small
# heading cone, B3-B5 are the V4-style joystick task (3s resamples,
# jitter, stops, partial blends) at 20/40/60 s, and only after the
# 60-second fast joystick task retains do B6/B7 add DR 0.1/0.3 and
# B8/B9 open lateral/rear. Gates are deliberately STRICT on
# direction/slip/height (the exact axes both steer forks failed):
# fast rungs demand cmd_prog>=0.70 (p10>=0.55), slip_per_m<=1.6,
# cross_track<=0.20, peak_roll<=8 deg, height_factor>=0.80 plus the
# usual all-feet contact/switch minima; the B0 bridge is slightly
# softer (cmd_prog>=0.65, p10>=0.50, slip<=1.8, cross_track<=0.22)
# so the warm start can certify at its own operating point before the
# band moves. V5 pairs with reward.k_loadslip_excess (direct loaded-
# slip charge, below) so skating is punished in training, not merely
# complained about at cert time.
# 2026-08-20 ~08:5x UTC alignment (q_20260820T0830Z review): the
# desktop branch (origin/codex/recover-retention @ 2cb2a7b7) is now
# fetchable. Diffed verbatim against the controller reconstruction
# above/below: the B0 bridge_10s bucket + WALKCURR_GATE_V5_BRIDGE the
# canaries actually exercised are BIT-IDENTICAL (confirming the
# fastnoslip1/midnoslip1 precert FAILs are genuine, not a
# reconstruction artifact) — but slew_sat_max was 0.95 here vs the
# authored 0.98, and B6-B9's min_command_changes/resample_s/jitter/
# stop_frac/blend/s_lo/head_hi diverged from the authored ladder.
# Realigned to the exact authored values below (never exercised by
# either canary — both died at B0 — so no in-flight run is affected).
WALKCURR_GATE_V5_BRIDGE = dict(
    cmd_prog_frac_min=0.65, cmd_prog_frac_p10_min=0.50,
    slip_per_m_max=1.8, peak_roll_deg_max=8.0, slew_sat_max=0.98,
    cross_track_frac_max=0.22, contact_sw_per_s_min=3.0,
    foot_sw_min_per_s_min=0.5, height_factor_min=0.80)
WALKCURR_GATE_V5_FAST = dict(
    cmd_prog_frac_min=0.70, cmd_prog_frac_p10_min=0.55,
    slip_per_m_max=1.6, peak_roll_deg_max=8.0, slew_sat_max=0.98,
    cross_track_frac_max=0.20, contact_sw_per_s_min=3.0,
    foot_sw_min_per_s_min=0.5, height_factor_min=0.80)

WALKCURR_BUCKETS_V5 = (
    # B0: prove the imported gait survives at its own operating point.
    dict(name="bridge_10s", duration_s=10.0, min_command_changes=0,
         s_lo=0.05, s_hi=0.06, head_lo=0.0, head_hi=0.0,
         resample_s=0.0, jitter=0.0, stop_frac=0.0, blend_lo=1.0,
         blend_hi=1.0, dr=0.0, stop_gate=None,
         gate=WALKCURR_GATE_V5_BRIDGE),
    # B1/B2: the fast band opens — straight first, then a small cone.
    dict(name="fast_08_10_10s", duration_s=10.0, min_command_changes=0,
         s_lo=0.08, s_hi=0.10, head_lo=0.0, head_hi=0.0,
         resample_s=0.0, jitter=0.0, stop_frac=0.0, blend_lo=1.0,
         blend_hi=1.0, dr=0.0, stop_gate=None,
         gate=WALKCURR_GATE_V5_FAST),
    dict(name="fast_06_10_head15", duration_s=20.0, min_command_changes=0,
         s_lo=0.06, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(15.0), resample_s=0.0, jitter=0.0,
         stop_frac=0.0, blend_lo=1.0, blend_hi=1.0, dr=0.0,
         stop_gate=None, gate=WALKCURR_GATE_V5_FAST),
    # B3-B5: fast joystick task, duration grows 20/40/60 s.
    dict(name="fast_joystick_20s", duration_s=20.0, min_command_changes=4,
         s_lo=0.06, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(30.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V5_FAST),
    dict(name="fast_joystick_40s", duration_s=40.0, min_command_changes=10,
         s_lo=0.06, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(30.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V5_FAST),
    dict(name="fast_joystick_60s", duration_s=60.0, min_command_changes=15,
         s_lo=0.06, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(30.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V5_FAST),
    # B6/B7: retain the 60-second fast joystick task under DR, the
    # heading cone opening to ±45 and the band widening down to 0.04.
    dict(name="fast_dr01_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.05, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.1,
         stop_gate=0.015, gate=WALKCURR_GATE_V5_FAST),
    dict(name="fast_dr03_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.05, s_hi=0.10, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.3,
         stop_gate=0.015, gate=WALKCURR_GATE_V5_FAST),
    # B8/B9: lateral and rear stay locked behind retained front-cone
    # competence under DR0.3.
    dict(name="lateral_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.04, s_hi=0.10, head_lo=math.radians(45.0),
         head_hi=math.radians(90.0), resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.3,
         stop_gate=0.015, gate=WALKCURR_GATE_V5_FAST),
    dict(name="rear_60s", duration_s=60.0, min_command_changes=9,
         s_lo=0.04, s_hi=0.10, head_lo=math.radians(90.0),
         head_hi=math.pi, resample_s=4.0, jitter=0.5,
         stop_frac=0.15, blend_lo=0.25, blend_hi=0.75, dr=0.3,
         stop_gate=0.015, gate=WALKCURR_GATE_V5_FAST),
)

# WALKCURR_BUCKETS_V6 ("hist16 full-circle joystick" ladder, operator
# order fb_20260823T220651_5c66e3): teach the hist16-dep1/c1 learned
# gait (16-frame deployment-contract walker, front-cone joystick only,
# heading_max 45 deg in training) to follow joystick commands in EVERY
# direction. Structure mirrors V4/V5: B0 is a strict 10-second bridge
# at the source checkpoint's own operating point
# (ppo_goal_cw_arch_hist16_dep1_c1 walks 0.05-0.06 m/s, front cone,
# DR0.5-trained), then the heading envelope opens in bands — front
# cone +-45 (20/60 s), side band 45-90 (20/60 s), rear bands 90-135
# (40 s) and 135-180 (60 s) — before B6 draws uniformly over the FULL
# CIRCLE for 60 s and B7/B8 retain the full-circle task under DR 0.2
# then DR 0.5 (the lineage's own training DR). One command
# distribution (3 s resamples, jitter, stops, partial blends) is held
# fixed across every joystick rung; only heading band, sustained
# horizon, and DR move. Gates are the V4 joystick bars (slip<=2.0 —
# inside the joystick-track slip band <=2.9 — cmd_prog>=0.65,
# all-feet cycling minima), bridge slightly softer, so promotion
# demands real command following in the newly opened band, not just
# survival.
WALKCURR_GATE_V6_BRIDGE = dict(
    cmd_prog_frac_min=0.60, cmd_prog_frac_p10_min=0.50,
    slip_per_m_max=2.0, peak_roll_deg_max=8.0, slew_sat_max=0.95,
    cross_track_frac_max=0.30, contact_sw_per_s_min=3.0,
    foot_sw_min_per_s_min=0.5, height_factor_min=0.80)
WALKCURR_GATE_V6_JOYSTICK = dict(
    cmd_prog_frac_min=0.65, cmd_prog_frac_p10_min=0.50,
    slip_per_m_max=2.0, peak_roll_deg_max=8.0, slew_sat_max=0.95,
    cross_track_frac_max=0.30, contact_sw_per_s_min=3.0,
    foot_sw_min_per_s_min=0.5, height_factor_min=0.80)

WALKCURR_BUCKETS_V6 = (
    # B0: prove the transplanted hist16 gait survives at its own
    # operating point (0.05-0.06 m/s straight, DR0) before anything
    # moves.
    dict(name="bridge_10s", duration_s=10.0, min_command_changes=0,
         s_lo=0.05, s_hi=0.06, head_lo=0.0, head_hi=0.0,
         resample_s=0.0, jitter=0.0, stop_frac=0.0, blend_lo=1.0,
         blend_hi=1.0, dr=0.0, stop_gate=None,
         gate=WALKCURR_GATE_V6_BRIDGE),
    # B1/B2: the front cone the lineage already knows (+-45 deg),
    # joystick churn immediately, horizon 20 then 60 s.
    dict(name="front45_20s", duration_s=20.0, min_command_changes=4,
         s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V6_JOYSTICK),
    dict(name="front45_60s", duration_s=60.0, min_command_changes=15,
         s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.radians(45.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V6_JOYSTICK),
    # B3/B4: the side band opens (45-90 deg), 20 then 60 s.
    dict(name="side90_20s", duration_s=20.0, min_command_changes=4,
         s_lo=0.04, s_hi=0.08, head_lo=math.radians(45.0),
         head_hi=math.radians(90.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V6_JOYSTICK),
    dict(name="side90_60s", duration_s=60.0, min_command_changes=15,
         s_lo=0.04, s_hi=0.08,
         head_lo=math.radians(45.0), head_hi=math.radians(90.0),
         resample_s=3.0, jitter=0.2, stop_frac=0.10, blend_lo=0.5,
         blend_hi=0.9, dr=0.0, stop_gate=0.015,
         gate=WALKCURR_GATE_V6_JOYSTICK),
    # B5/B6: rear bands 90-135 (40 s) then 135-180 (60 s).
    dict(name="rear135_40s", duration_s=40.0, min_command_changes=10,
         s_lo=0.04, s_hi=0.08, head_lo=math.radians(90.0),
         head_hi=math.radians(135.0), resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V6_JOYSTICK),
    dict(name="rear180_60s", duration_s=60.0, min_command_changes=15,
         s_lo=0.04, s_hi=0.08, head_lo=math.radians(135.0),
         head_hi=math.pi, resample_s=3.0, jitter=0.2,
         stop_frac=0.10, blend_lo=0.5, blend_hi=0.9, dr=0.0,
         stop_gate=0.015, gate=WALKCURR_GATE_V6_JOYSTICK),
    # B7: the goal task — uniform full-circle joystick, 60 s.
    dict(name="fullcircle_60s", duration_s=60.0, min_command_changes=15,
         s_lo=0.04, s_hi=0.08, head_lo=0.0, head_hi=math.pi,
         resample_s=3.0, jitter=0.2, stop_frac=0.10, blend_lo=0.5,
         blend_hi=0.9, dr=0.0, stop_gate=0.015,
         gate=WALKCURR_GATE_V6_JOYSTICK),
    # B8/B9: retain the full-circle task under DR 0.2 then the
    # lineage's own training DR 0.5.
    dict(name="fullcircle_dr02_60s", duration_s=60.0,
         min_command_changes=15, s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.pi, resample_s=3.0, jitter=0.2, stop_frac=0.10,
         blend_lo=0.5, blend_hi=0.9, dr=0.2, stop_gate=0.015,
         gate=WALKCURR_GATE_V6_JOYSTICK),
    dict(name="fullcircle_dr05_60s", duration_s=60.0,
         min_command_changes=15, s_lo=0.04, s_hi=0.08, head_lo=0.0,
         head_hi=math.pi, resample_s=3.0, jitter=0.2, stop_frac=0.10,
         blend_lo=0.5, blend_hi=0.9, dr=0.5, stop_gate=0.015,
         gate=WALKCURR_GATE_V6_JOYSTICK),
)

# WALKCURR_BUCKETS_V7 ("stress-diet" ladder, 08-24 assume-and-go
# repair): same heading/duration/DR/gate ladder as V6 (the certfreeze
# dig-in's own pre-registered if-false branch — "the damage is the
# b2+ practice diet itself... next lever is mixing stress_mix
# commands into bucket training, not freeze mechanics" — B2-B9's
# held-out joygate falls (over_current, 1/48 parent -> 6-7/48 across
# 3 independent freeze/no-freeze arms) never moved with the freeze
# mechanism, so the fix targets the DIET, not the supervisor). V6's
# own command sampler never draws an in-place turn (wz) or a full
# reversal — the held-out joygate's stress_mix family (flip_180,
# sweep_circle, square, jitter) does both every episode. V7 adds
# per-bucket `wz_max`/`wz_zero_frac` (in-place turning, same
# semantics as goal.walk_yaw_max_rad_s/walk_yaw_zero_frac) and
# `reversal_frac` (probability a resampled segment is a full
# instantaneous reversal, the bucket-diet analogue of flip_180) from
# front45_20s onward; the bridge rung (B0, proving the transplanted
# gait survives at all) stays untouched at 0/1.0/0.0. `_sample_walk_
# curr` reads these with `.get(..., 0.0)` defaults, so V1-V6 tables
# (missing the keys) are bit-exact unchanged.
WALKCURR_BUCKETS_V7 = tuple(
    (dict(b, wz_max=0.0, wz_zero_frac=1.0, reversal_frac=0.0)
     if b["name"] == "bridge_10s" else
     dict(b, wz_max=0.3, wz_zero_frac=0.5, reversal_frac=0.15))
    for b in WALKCURR_BUCKETS_V6
)

# Sampling mixture over unlocked buckets (operator spec): 50% frontier,
# 25% weakest mastered, 15% uniform over mastered, 10% the rung just
# prior to the frontier. Empty components fold back to the frontier.
WALKCURR_MIX = dict(frontier=0.50, weakest=0.25, uniform=0.15,
                    prior=0.10)
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

# Speed-coupled phase clock (2026-08-22, amp M2 speedrange root cause;
# same fingerprint as joystick phasedir9-seed17): the clock above
# advances at a FIXED rate whenever any linear velocity is commanded,
# so commanded speed can never reach the actor's cadence — a 3x
# command range compressed to ~0.10+/-0.02 m/s realized in BOTH the
# fastphase (hz 1.333->2.0) and fastphase-nostyle (style weight 0)
# probes: neither a faster constant clock nor removing AMP style
# widened it. With goal.walk_phase_speed_scale=k (default 0.0 = OFF,
# bit-exact legacy), the effective clock rate becomes
#   hz_eff = hz_base * (1 + k * (s_ref / s_nom - 1))
# i.e. k=1 -> cadence fully proportional to commanded speed, anchored
# so s_ref == s_nom reproduces hz_base exactly (the teacher's cadence
# at its natural ~0.08 m/s). goal.walk_phase_speed_nom sets the
# anchor speed; goal.walk_phase_hz_max (default 0 = no clamp) caps
# the rate so extreme commands cannot demand physically impossible
# stepping. Consumers stay consistent by construction: obs sin/cos,
# the k_phase_contact agreement reward, and (via sim_env's
# bc_anchor_phase_lock branch) the walk BC anchor all read the same
# advanced phase.
PHASE_SPEED_NOM_DEFAULT = 0.08   # m/s; teacher's natural speed


def phase_hz_effective(hz_base, s_ref, k_coup,
                       s_nom=PHASE_SPEED_NOM_DEFAULT, hz_max=0.0):
    """Effective phase-clock rate under speed coupling.

    k_coup <= 0 (the default) returns hz_base exactly — legacy
    fixed-rate clock, bit-exact. s_ref == s_nom also returns hz_base
    exactly for any k_coup (anchor point). hz_max > 0 clamps the top;
    the result is never negative.
    """
    if k_coup <= 0.0 or s_ref <= 0.0:
        return hz_base
    ratio = s_ref / max(float(s_nom), 1e-6)
    hz = hz_base * (1.0 + k_coup * (ratio - 1.0))
    if hz_max > 0.0:
        hz = min(hz, hz_max)
    return max(hz, 0.0)


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
    # recover (recover_to_plant, 08-15 operator directive
    # fb_20260815T165306_606974): reach a full-height quiet six-loaded
    # stand from any recoverable state, zero velocity command
    # throughout — command-wise it is the rise family.
    "recover": "rise",
    "quad": "quad",
    # quadwalk = quad-family locomotion (08-13, quad track "four-leg
    # WALKING" spec): the quad one-hot bit + non-zero vx/vy refs carry
    # the within-mode command, exactly the walk-vs-hold convention.
    "quadwalk": "quad",
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
    cmd_mode: str = "legacy"
    duration_steps: int | None = None
    command_changes: int = 0

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
                          "_foot_prev_xy", "_foot_prev_force",
                          "_foot_tan_slip_m", "_duty_hist", "_phase",
                          "_anchor_xy", "_anchor_prev_on",
                          "_walk_bucket", "_step_disp_bank",
                          "_ls_prev_xy", "_ls_prev_on",
                          "_ls_slip_m", "_ls_prog_m",
                          "_yaw_still_ema", "_yaw_prog_ema", "_stance_slip_acc",
                          "_walk_idle_ema", "_walk_idle_low_s",
                          "_walk_stop_cmd_s",
                          "_walk_qvel_ema", "_walk_course_ema",
                          "_walk_kernel_vema", "_walk_kernel_wz_ema",
                          "_gait_last_step", "_gait_cmd_tick",
                          "_gait_gate_qfactor", "_wp", "_vel_est")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Walk probability lives on the generator so the eval callback's
        # p_<mode> isolation mechanism works unchanged. 0.70: run 1's
        # 0.40 diluted the hard skill with tasks the lineage had solved.
        self._goal_gen.p_walk = 0.70
        # quadwalk (08-13, quad track): commanded walking on the four
        # support legs with goal.quad_lift_legs raised. Default 0 =
        # never sampled; the _sample_goal cdf gains an EMPTY interval
        # so every legacy rng stream is bit-exact unchanged. Enable
        # per-run via --goal-mix quadwalk=<p>. The attribute must
        # exist (not just a getattr default) so the eval harness's
        # p_<mode> forcing can isolate the mode.
        self._goal_gen.p_quadwalk = 0.0
        # recover_to_plant (08-15, operator directive fb_20260815T165306
        # _606974): universal recovery to a quiet six-loaded stand.
        # Default 0 = never sampled; the _sample_goal cdf gains an EMPTY
        # interval so every legacy rng stream is bit-exact unchanged.
        # Enable per-run via --goal-mix recover=<p>.
        self._goal_gen.p_recover = 0.0
        # Adaptive reset-bank curriculum state (recover mode only).
        # PERSISTENT across episodes (like _lp_weights, NOT in
        # SNAP_ATTRS).  _rec_stats is certification-only: stochastic
        # PPO rollouts are deliberately too noisy to certify the 0.5 s
        # six-foot hold, so they must never advance the ladder.
        # _rec_rollout_stats remains visible as diagnostic telemetry.
        self._rec_stats = {}
        self._rec_cert_rounds = {}
        self._rec_rollout_stats = {}
        self._rec_rollout_counts = {}
        # Global MJX training batches are folded into this per-bucket
        # terminal-shortfall EMA by the trainer callback.  It affects only
        # replay probability; deterministic certification remains the sole
        # authority for admission.
        self._rec_training_error_stats = {}
        self._rec_external_certification = bool(float(cfg_get(
            self.cfg, "goal", "recover_external_certification",
            default=0.0)))
        # Recovery must prove the easy six-foot correction before floor
        # starts enter the diet.  any1 started with families 1+2 and also
        # probed family 3, so there was no bucket-1 acquisition phase to
        # measure.
        # _rec_active_n is the MONOTONIC number of unlocked buckets.
        # _rec_focus_bucket is the hardest unlocked acquisition bucket;
        # _rec_weak_bucket is the weakest previously certified bucket and
        # receives extra spaced-replay pressure.  Keeping these concepts
        # separate prevents a noisy assay from deleting learned starts.
        self._rec_active_n = 1
        self._rec_focus_bucket = 0
        self._rec_weak_bucket = None
        # Swing-bonus bookkeeping (see step()): per-foot contact state
        # and world XY at the moment of liftoff.
        self._pad_bids = [self.model.body(f"L{i}_pad").id
                          for i in range(6)]
        self._foot_on = [True] * 6
        self._liftoff_xy = [None] * 6
        self._liftoff_step = [0] * 6
        self._foot_prev_xy = [None] * 6
        self._foot_prev_force = [0.0] * 6
        self._foot_tan_slip_m = [0.0] * 6
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
        self._yaw_prog_ema = 0.0
        # Leg-odometry velocity estimator (goal.walk_obs_body_vel=3
        # only; None in every other mode = zero overhead). Per-episode
        # stateful — recreated on reset in _augment_obs, snapshot via
        # MJX_SNAPSHOT_EXTRA (instances deep-copy cleanly by design;
        # estimator.py docstring).
        self._vel_est = None
        # Loaded-slip income gate bookkeeping (operator ruling 2026-08-09
        # §3/WALK-SLIP): episode-accumulated loaded foot-XY travel and
        # along-command body progress. NEVER reset by touchdown — only
        # at episode reset — so cadence cannot re-buy an allowance.
        self._ls_prev_xy = [None] * 6
        self._ls_prev_on = [False] * 6
        self._ls_slip_m = 0.0
        self._ls_prog_m = 0.0
        # Anti-park travel-floor EMA (reward.k_walk_idle_charge);
        # per-episode/per-segment, snapshot via MJX_SNAPSHOT_EXTRA.
        self._walk_idle_ema = 0.0
        # Sustained-idle termination counter + its OWN mean-
        # |joint-velocity| EMA (safety.walk_idle_terminate_s):
        # deliberately NOT body speed (a real dragging/skating
        # gait can sit near-zero BODY speed while its legs
        # cycle hard, and cutting that short was measured to
        # rob it of the full-episode loadslip charge the bank
        # relies on; a body-speed version also flagged genuine
        # wrong-direction travel as "idle"). Mean |qvel| across
        # the 18 actuated joints cleanly separates a literally
        # FROZEN policy output (park/belly-sit/tripod-lock:
        # ~5e-5 rad/s, pure settle jitter) from every other
        # scripted behavior including skate/stall (>=0.1 rad/s,
        # ~35x higher) -- see the WALKCURR_PF_IDLE_TERM bank.
        # Seconds low, consecutively; reset to 0 the instant
        # the EMA clears the floor.
        self._walk_idle_low_s = 0.0
        self._walk_qvel_ema = 0.0
        # Seconds since the current commanded-stop segment began
        # (reward.walk_stop_grace_s); 0 whenever s_ref > 1e-3
        # (walking commanded), increments by dt each stop tick.
        # Used only to ramp the stop-speed charge in over the
        # unavoidable deceleration transient -- see the charge
        # site below. Same lifecycle as _walk_idle_ema.
        self._walk_stop_cmd_s = 0.0
        # Structural stop-hold timer (goal.walk_stop_freeze_s);
        # same lifecycle as _walk_stop_cmd_s -- see
        # sim_env._walk_stop_freeze_override.
        self._walk_stop_freeze_cmd_s = 0.0
        # Commanded-course EMA (reward.k_walk_course); same lifecycle.
        self._walk_course_ema = [0.0, 0.0]
        # Stride-EMA velocity for the tracking kernel
        # (reward.walk_kernel_vel_ema); same lifecycle.
        self._walk_kernel_vema = [0.0, 0.0]
        # Stride-EMA yaw-rate for the yaw tracking kernel
        # (reward.walk_kernel_yaw_ema); same lifecycle.
        self._walk_kernel_wz_ema = 0.0
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
        # Drag-stance allowance RAMP (08-22, phasedir9-seed-lottery
        # dig-in / pd8 regime-gap follow-up): the det-calibrated
        # drag_stance_allow_mm cannot separate honest walking from a
        # drag cheat while PPO's action-noise std is still high early
        # in training (pd8_digin_regime: at std 0.135 the honest
        # clone's own per-stance travel needs an allowance >=48mm to
        # stay untaxed while the det drag cheat pays zero past 36mm) —
        # a FIXED tight allowance (24mm) taxes noisy honest exploration
        # far harder than the cheat before basin selection happens,
        # which the log-std anneal alone cannot fix (mirrors
        # bus.profile_ramp_steps' construction exactly: cfg-armed,
        # trainer-driven, default OFF = bit-exact legacy). When armed
        # (reward.drag_stance_allow_ramp_steps > 0), the ALLOWANCE
        # starts loose (reward.drag_stance_allow_ramp_mm, meant to sit
        # above the noisy-honest tail) and anneals down to the normal
        # reward.drag_stance_allow_mm target over that many GLOBAL env
        # steps — armed but never broadcast (apply_drag_allow_frac)
        # sits at the TARGET allowance, so eval_checkpoint/play/the
        # periodic C-env evals always judge the calibrated final
        # pricing even when the training cfg carries ramp keys, exactly
        # like the profile ramp.
        self._drag_allow_ramp: dict | None = None
        self._drag_allow_override_m: float | None = None
        _da_ramp_steps = int(float(cfg_get(
            self.cfg, "reward", "drag_stance_allow_ramp_steps",
            default=0) or 0))
        if _da_ramp_steps > 0:
            _da_target_mm = float(cfg_get(
                self.cfg, "reward", "drag_stance_allow_mm", default=6.0))
            _da_start_mm = float(cfg_get(
                self.cfg, "reward", "drag_stance_allow_ramp_mm",
                default=48.0))
            if _da_start_mm < _da_target_mm:
                raise ValueError(
                    "reward.drag_stance_allow_ramp_mm "
                    f"({_da_start_mm:g}) must be >= the target "
                    f"reward.drag_stance_allow_mm ({_da_target_mm:g}) "
                    "— the ramp only ever loosens, never tightens, "
                    "the allowance")
            self._drag_allow_ramp = {
                "steps": _da_ramp_steps,
                "start_m": _da_start_mm / 1000.0,
                "target_m": _da_target_mm / 1000.0,
                "frac": 1.0,
            }
        # Termination-penalty RAMP (08-22, freeprog-term400-stall dig-in
        # follow-up): term_penalty=400 correctly killed the suicide
        # exploit (cw-amp-m2-freeprog-{noamp,style05}FAIL, dig-in
        # 08-22) but the term400 fix pair's own dig-in found a NEW
        # tension — env/reward_walk_freeprog_pen sits at a sustained
        # ~-1.4 to -1.8/tick cross-track charge (six legs cycling but
        # marching in place, near-zero along-command travel) for the
        # WHOLE 2M budget with terminations mostly SURVIVED (truncated-
        # dominant), i.e. the policy is choosing a known, bounded-cost
        # micro-stepping basin over the unknown, front-loaded risk of
        # a -400 charge while it is still unskilled at real strides.
        # Symmetric fix to the drag-allow ramp above: start the
        # termination charge LOW (reward.term_penalty_ramp_init, cheap
        # to fall while still learning to move) and anneal it UP to
        # the validated reward.term_penalty target over
        # reward.term_penalty_ramp_steps global env steps — so early
        # exploration of real strides is not front-loaded with the
        # full anti-suicide cost, while the final regime still prices
        # death at the validated deterrent level. Same cfg-armed /
        # trainer-driven / default-OFF contract; armed-but-unbroadcast
        # sits at the TARGET (full) penalty, so eval_checkpoint / play
        # always judge the validated deterrent even mid-ramp cfg.
        self._term_penalty_ramp: dict | None = None
        self._term_penalty_override: float | None = None
        _tp_ramp_steps = int(float(cfg_get(
            self.cfg, "reward", "term_penalty_ramp_steps",
            default=0) or 0))
        if _tp_ramp_steps > 0:
            _tp_target = float(cfg_get(self.cfg, "reward",
                                       "term_penalty", default=0.0))
            _tp_start = float(cfg_get(
                self.cfg, "reward", "term_penalty_ramp_init",
                default=0.0))
            if _tp_start > _tp_target:
                raise ValueError(
                    "reward.term_penalty_ramp_init "
                    f"({_tp_start:g}) must be <= the target "
                    f"reward.term_penalty ({_tp_target:g}) — the ramp "
                    "only ever raises, never lowers, the termination "
                    "charge below the validated deterrent")
            self._term_penalty_ramp = {
                "steps": _tp_ramp_steps, "start": _tp_start,
                "target": _tp_target, "frac": 1.0,
            }
        # Dense walk-charge RAMP (08-23, walkcurr fwd1/fwd2 dig-in):
        # from-scratch PPO froze into a tilt-safe splayed crouch for
        # 2M steps in three straight rung-1 arms because the dense
        # per-step walk charge flow (~-4.7/step, loadslip-dominated
        # via the 0.03 m travel floor) both makes freezing the best
        # REACHABLE policy and instantly punishes the flailing that
        # exploration must pass through, while every income channel
        # requires competent locomotion before it pays (walk_prog
        # identically 0.0 across all three runs); no bank-legal
        # income dose can compete (fwd2-swing/swingterm800 FAIL,
        # 08-23). When armed (reward.walk_charge_ramp_steps > 0) the
        # three DISCOVERY-FRICTION walk charges — k_park_duty,
        # k_walk_idle_charge, k_walk_heading — are scaled by
        # walk_charge_ramp_min_frac at frac 0 and anneal linearly UP
        # to the full bank-proven dose at frac 1. k_loadslip_excess is
        # DELIBERATELY EXCLUDED (bank finding, same day: ramping it
        # down at the shared min_frac=0.15 made 'skate'/'shuffle' beat
        # every wrong-way/standing behavior — see
        # test_walkcurr_chargeramp_min_ranking_holds) and always
        # charges at full dose; the ramp only loosens the charges that
        # make REFUSING to move look falsely cheap, never the
        # anti-skate/anti-fall floor. Same cfg-armed /
        # trainer-driven / default-OFF contract as the term-penalty
        # and drag-allow ramps: default absent/0 is bit-exact legacy,
        # and armed-but-unbroadcast sits at the FULL charges so
        # eval_checkpoint / play / the periodic C-env evals always
        # judge the bank-proven pricing even mid-ramp cfg.
        self._walk_charge_ramp: dict | None = None
        self._walk_charge_override: float | None = None
        _wc_ramp_steps = int(float(cfg_get(
            self.cfg, "reward", "walk_charge_ramp_steps",
            default=0) or 0))
        if _wc_ramp_steps > 0:
            # Default floor 0.40, MEASURED (08-23 chargeramp floor
            # bank): at 0.15 the loosened charges invert the required
            # ranking (skate +131 out-earns sideways +52/reverse -1.3
            # and shuffle +228 becomes a strong positive rest point);
            # 0.40 is the lowest floor that keeps skate/shuffle priced
            # below every honest behavior while the top of the
            # landscape stays positive-sum (gait > stall > park, all
            # positive) — see test_task_semantics.py
            # WALKCURR_PF_CHARGERAMP_MIN_OVERRIDES.
            _wc_min = float(cfg_get(
                self.cfg, "reward", "walk_charge_ramp_min_frac",
                default=0.40))
            if not 0.0 <= _wc_min <= 1.0:
                raise ValueError(
                    "reward.walk_charge_ramp_min_frac "
                    f"({_wc_min:g}) must be in [0, 1] — the ramp "
                    "only ever anneals the charges UP to the "
                    "bank-proven full dose")
            self._walk_charge_ramp = {
                "steps": _wc_ramp_steps, "min_frac": _wc_min,
                "frac": 1.0,
            }
        # Loaded-slip-excess BOOTSTRAP (08-23, walkcurr fwd4 dig-in
        # follow-up): fwd1-fwd4 (four straight rung-1 arms — plain,
        # swing-income x2, charge-ramp, wider-init-noise x2) all froze
        # into a static splayed crouch with env/walk_freeprog_score
        # flat/negative for the full 2M budget; the walk-charge ramp
        # deliberately EXCLUDED k_loadslip_excess (bank finding: ramping
        # it down TOGETHER with the other three charges at their shared
        # min_frac=0.15 made 'skate'/'shuffle' beat every honest
        # standing/wrong-way behavior). This is a SEPARATE, narrower
        # lever: soften ONLY k_loadslip_excess, ONLY for a short early
        # bootstrap window, ANNEALING BACK UP to the full bank-proven
        # dose — distinct from the rejected permanent-low-floor design.
        # Same cfg-armed / trainer-driven / default-OFF contract as the
        # three ramps above (bit-exact when reward.
        # walk_loadslip_bootstrap_steps is 0/absent). Re-proven at ITS
        # OWN schedule against the WALKCURR_PF bank held at the
        # bootstrap's min_frac (loadslip alone loosened, the other three
        # discovery-friction charges at full dose — the opposite
        # combination from the rejected chargeramp-min design) — see
        # test_task_semantics.py WALKCURR_PF_LOADSLIP_BOOTSTRAP_MIN_OVERRIDES.
        # cfg: reward.walk_loadslip_bootstrap_steps (int, 0=off),
        # reward.walk_loadslip_bootstrap_min_frac (float in [0,1],
        # default 0.65 — MEASURED separately from the walk-charge
        # ramp's 0.40 floor: this charge, held alone at 0.40 while the
        # other three discovery-friction charges stay at FULL dose
        # (the least-favorable single-lever combination), lets
        # 'sideways' (-231) out-earn 'park' (-352) — a ranking
        # violation the bank catches
        # (test_walkcurr_loadslip_bootstrap_min_ranking_holds); 0.65
        # is the lowest floor in a 0.5-0.8 sweep that keeps
        # park/stall strictly above every wrong-way gait (margin
        # ~37) while still cutting the skate penalty by ~30% (-1339
        # -> -940) relative to full dose — see test_task_semantics.py
        # WALKCURR_PF_LOADSLIP_BOOTSTRAP_MIN_OVERRIDES.
        self._ls_bootstrap: dict | None = None
        self._ls_bootstrap_override: float | None = None
        _lsb_steps = int(float(cfg_get(
            self.cfg, "reward", "walk_loadslip_bootstrap_steps",
            default=0) or 0))
        if _lsb_steps > 0:
            _lsb_min = float(cfg_get(
                self.cfg, "reward", "walk_loadslip_bootstrap_min_frac",
                default=0.65))
            if not 0.0 <= _lsb_min <= 1.0:
                raise ValueError(
                    "reward.walk_loadslip_bootstrap_min_frac "
                    f"({_lsb_min:g}) must be in [0, 1] — the bootstrap "
                    "only ever anneals k_loadslip_excess UP to the "
                    "bank-proven full dose")
            self._ls_bootstrap = {
                "steps": _lsb_steps, "min_frac": _lsb_min, "frac": 1.0,
            }
        # All-support-legs gait gate bookkeeping (08-13, quad track,
        # reward.walk_gait_gate): per-leg COMMANDED-tick index of the
        # last completed real swing (liftoff -> >=2 ticks airborne ->
        # touchdown with XY stride >= gait_gate_stride_mm), plus the
        # commanded-tick clock itself and the per-tick factor stashed
        # for _quad_income's clear/plant gating. All three ride
        # MJX_SNAPSHOT_EXTRA (pool-restore lesson, commit 65edba7).
        self._gait_last_step = [0] * 6
        self._gait_cmd_tick = 0
        self._gait_gate_qfactor = 1.0
        # Learning-progress curriculum state: sampling weights over
        # LP_BUCKETS (None = uniform) and the bucket of the current
        # walk episode (surfaced in step info for the LP callback).
        self._lp_weights = None
        self._walk_bucket = None
        # Adaptive competence+retention walk-command curriculum state
        # (goal.walk_curriculum=1..4; see WALKCURR_BUCKETS*).
        # PERSISTENT across episodes like _lp_weights/_rec_* — never in
        # SNAP_ATTRS. _wc_results is certification-only: stochastic
        # rollouts must never move the frontier; the trainer broadcasts
        # deterministic held-out assay results via
        # apply_walkcurr_certification and promotes via
        # walkcurr_update_admission. version 2 (walkcurr2, operator MCP
        # note fb_20260818T060044) selects WALKCURR_BUCKETS_V2 (fixed
        # B0/B1 ignition band + per-bucket gate calibration) instead of
        # the original V1 table; version 1 stays bit-exact unchanged.
        wc_version = float(cfg_get(self.cfg, "goal", "walk_curriculum",
                                   default=0.0))
        self._wc_on = wc_version in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
        self._wc_version = int(wc_version) if self._wc_on else 0
        self._wc_table = (WALKCURR_BUCKETS_V7 if self._wc_version == 7
                          else WALKCURR_BUCKETS_V6
                          if self._wc_version == 6
                          else WALKCURR_BUCKETS_V5
                          if self._wc_version == 5
                          else WALKCURR_BUCKETS_V4
                          if self._wc_version == 4
                          else WALKCURR_BUCKETS_V3
                          if self._wc_version == 3
                          else WALKCURR_BUCKETS_V2
                          if self._wc_version == 2
                          else WALKCURR_BUCKETS)
        if self._wc_version in (4, 5, 6, 7):
            required_s = max(float(b["duration_s"])
                             for b in self._wc_table)
            available_s = self.episode_steps * self.dt
            if available_s + 0.5 * self.dt < required_s:
                raise ValueError(
                    f"walk curriculum V{self._wc_version} requires "
                    f"episode_seconds >= {required_s:g} (got "
                    f"{available_s:g}); long-horizon certification "
                    "must not be silently shortened")
        self._wc_active_n = 1
        self._wc_results: dict = {}   # bucket -> {passed, score, cert_round}
        self._wc_bucket = None        # this episode's curriculum bucket
        self._wc_randomizers: dict = {}   # dr scale -> DomainRandomizer
        if self._wc_on and float(cfg_get(
                self.cfg, "goal", "walk_lp_curriculum",
                default=0.0)) == 1.0:
            raise ValueError("goal.walk_curriculum and "
                             "goal.walk_lp_curriculum are mutually "
                             "exclusive command samplers")
        # goal.walk_pure (2026-08-18, operator order
        # fb_20260818T065930_03b422): pure-walk diet fixed at
        # CONSTRUCTION time — every p_<mode> on the goal generator is
        # zeroed and p_walk set to 1.0 before the first reset, so the
        # batched MJX vec envs (which build their shim envs internally
        # and mint reset pools immediately) can never sample a mixed
        # diet before a post-construction set_goal_mix lands. Default
        # 0 = off, bit-exact legacy (no draws, no attribute writes).
        if float(cfg_get(self.cfg, "goal", "walk_pure",
                         default=0.0)) > 0.0:
            gen = self._goal_gen
            for _name in dir(gen):
                if (_name.startswith("p_")
                        and isinstance(getattr(gen, _name),
                                       (int, float))):
                    setattr(gen, _name, 0.0)
            gen.p_walk = 1.0
        # In-env walk quality probe (measurement only, default OFF;
        # walkcurr MJX certification, fb_20260818T065930_03b422): when
        # walk_probe_on is set (VecEnv set_attr on cert envs), each
        # episode accumulates the eval_task quality metrics from the
        # same mirrored fields the reward stack reads (pad-body XY,
        # touch sensors, safety slew, IMU state, goal refs) and emits
        # them as info["walk_probe"] on the terminal tick. Never
        # affects obs, reward, termination or rng on any backend.
        self.walk_probe_on = bool(float(cfg_get(
            self.cfg, "goal", "walk_probe", default=0.0)) > 0.0)
        self._wp = None
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
        # Command-derived one-hot (obs.mode_onehot_cmd=1; the multitask
        # x arch transplant, 08-13). On the command-conditioned
        # generalist recipe every episode is mode "walk", so the
        # episode-constant one-hot above never routes the dual-core GRU
        # (gru_policy.DualGruActorCriticPolicy gates on the obs tail).
        # With this flag, walk-FAMILY ticks light the slot from the
        # LIVE blended command instead: a commanded stop (all of
        # |vx_ref|,|vy_ref| <= obs.mode_cmd_stop_m_s and |wz_ref| <=
        # obs.mode_cmd_stop_rad_s) lights "hold" (stance core), any
        # motion command lights "walk" (locomotion core). Non-walk
        # modes are untouched; no effect unless obs.mode_onehot=1.
        # Default OFF = bit-exact obs for every existing lineage.
        self._mode_cmd = float(cfg_get(self.cfg, "obs", "mode_onehot_cmd",
                                       default=0.0)) == 1.0
        # Recovery needs a task-stable pose frame.  q-q_nom is zero at
        # every reset because q_nom is the arbitrary settled bad pose, so
        # two very different tangles can otherwise begin with identical
        # joint-position observations.  Append q-q_plant at the frame tail
        # (recover ticks only); the pretrained dynamics adapter deliberately
        # consumes only the original first 59 proprio fields.
        self._recover_plant_q_obs = float(cfg_get(
            self.cfg, "obs", "recover_plant_q", default=0.0)) == 1.0
        # Fault-health obs (obs.fault_health=1; AMP brief sec8.2 M4
        # wiring — default OFF, bit-exact-when-off per every other obs
        # flag here). Appends EpisodeRandomization.fault_health() (18,
        # per-joint 1.0 healthy / 0.0 disabled-frozen / intermediate =
        # degraded strength) at the frame tail every tick, so the
        # policy can actually CONDITION ON a known joint fault instead
        # of only compensating blind (the faultsmoke1 pair's innate-
        # tolerance measurement). Same value all episode (fault draw is
        # per-reset, not per-tick) but recomputed every tick like
        # mode_onehot/wz_ref so it survives obs-history stacking and
        # pool-restore without a new SNAP_ATTRS entry — _ep_rand IS the
        # snapshotted state. All-ones (perfectly healthy) whenever
        # dr.fault_prob=0 or _ep_rand is unset (CPU C-path corner
        # before the first reset), so a fault-unaware DR-0 eval reads
        # as "everything healthy", the correct semantics.
        self._fault_obs = float(cfg_get(
            self.cfg, "obs", "fault_health", default=0.0)) == 1.0
        if _gym is not None:
            self.observation_space = self._obs_space_box(
                N_OBS - 6 + self.n_act + WALK_GOAL_DIM + N_VEL_OBS
                + (N_PHASE_OBS if self._phase_obs else 0)
                + (1 if self._yaw_cmd else 0)
                + (N_MODE_OBS if self._mode_obs else 0)
                + (N_JOINTS if self._recover_plant_q_obs else 0)
                + (N_JOINTS if self._fault_obs else 0))

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
        #   3.0 — DEPLOYABLE leg-odometry estimator
        #         (rl_move.estimator.LegOdometryVelocity, the probe-
        #         validated board-safe module): fed the OBSERVED state
        #         (DR-corrupted encoders + gyro + tilt — the robot's own
        #         view), so training sees hardware-realistic estimate
        #         error. Built 08-20 (fb_20260820T000059 item 3c audit:
        #         mode 2 carries ZERO body-velocity information by
        #         construction — the fast-profile canary could not even
        #         observe its own 2.5x overspeed; before 08-20 a cfg
        #         value of 3 silently fell into the privileged branch).
        #         Same obs WIDTH as every other mode.
        vel_mode = float(cfg_get(self.cfg, "goal", "walk_obs_body_vel",
                                 default=1.0))
        if vel_mode == 0.0:
            v = np.zeros(N_VEL_OBS)
        elif vel_mode == 2.0:
            goal = self._current_goal()
            v = (np.array([float(getattr(goal, "vx_ref", 0.0)),
                           float(getattr(goal, "vy_ref", 0.0))])
                 / VEL_SCALE) if goal is not None else np.zeros(N_VEL_OBS)
        elif vel_mode == 3.0:
            if reset or self._vel_est is None:
                from rl_move.estimator import LegOdometryVelocity
                self._vel_est = LegOdometryVelocity(dt=self.dt)
            st = self._state
            if st is not None:
                v_est = self._vel_est.update(
                    st.joint_position, st.imu_gyro,
                    st.imu_roll, st.imu_pitch)
            else:
                v_est = np.zeros(N_VEL_OBS)
            v = np.asarray(v_est, dtype=float) / VEL_SCALE
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
                run = s_ref > 1e-3
                if not run and float(cfg_get(
                        self.cfg, "goal", "walk_phase_run_on_yaw",
                        default=0.0)) > 0.0:
                    # Turn-in-place clock fix (amp M2-yaw, 08-22): the
                    # clock above only ran while a LINEAR velocity was
                    # commanded, so during turn-in-place segments
                    # (vx=vy=0, wz!=0) the phase obs FROZE — a
                    # phase-locked policy (every BC-clone lineage) then
                    # has no time-base to step with and parks. Measured:
                    # yawcmd + tip50-r2 + tip90 all show tip err ==
                    # |wz_ref| exactly (zero rotation at vx=0) while yaw
                    # WHILE TRANSLATING (clock running) did improve —
                    # exposure 0.5/0.9 gave zero dose-response because
                    # the skill was unlearnable with a frozen clock, not
                    # under-exposed. goal.walk_phase_run_on_yaw=1 also
                    # advances the clock while a yaw rate is commanded
                    # (default 0 = off, bit-exact legacy; pure-park
                    # segments with wz_ref=0 still freeze the clock).
                    run = abs(float(getattr(goal, "wz_ref", 0.0))
                              if goal is not None else 0.0) > 1e-3
                if run:
                    hz = float(cfg_get(self.cfg, "goal", "walk_phase_hz",
                                       default=PHASE_HZ_DEFAULT))
                    # Speed-coupled clock (default OFF = bit-exact
                    # legacy fixed rate; see phase_hz_effective).
                    k_coup = float(cfg_get(
                        self.cfg, "goal", "walk_phase_speed_scale",
                        default=0.0))
                    if k_coup > 0.0:
                        hz = phase_hz_effective(
                            hz, s_ref, k_coup,
                            s_nom=float(cfg_get(
                                self.cfg, "goal",
                                "walk_phase_speed_nom",
                                default=PHASE_SPEED_NOM_DEFAULT)),
                            hz_max=float(cfg_get(
                                self.cfg, "goal", "walk_phase_hz_max",
                                default=0.0)))
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
            mode = (getattr(self._goal_traj, "mode", "hold")
                    if self._goal_traj is not None else "hold")
            if (self._mode_cmd
                    and _MODE_FAMILY.get(str(mode), "hold") == "walk"):
                # Command-derived slot (obs.mode_onehot_cmd=1): follow
                # the LIVE blended command so the dual-core GRU routes
                # stop segments to the stance core. Derived per tick
                # from _current_goal() like the wz_ref tail above —
                # pool-restore safe by construction, no episode attr.
                # During the settle hold / ramp-from-zero the command
                # IS near zero, so those ticks route to the stance
                # core too — the commanded behavior there is standing.
                goal = self._current_goal()
                vx = float(getattr(goal, "vx_ref", 0.0)) \
                    if goal is not None else 0.0
                vy = float(getattr(goal, "vy_ref", 0.0)) \
                    if goal is not None else 0.0
                wz = float(getattr(goal, "wz_ref", 0.0)) \
                    if goal is not None else 0.0
                eps_v = float(cfg_get(self.cfg, "obs", "mode_cmd_stop_m_s",
                                      default=0.005))
                eps_w = float(cfg_get(self.cfg, "obs",
                                      "mode_cmd_stop_rad_s",
                                      default=0.02))
                stopped = (abs(vx) <= eps_v and abs(vy) <= eps_v
                           and abs(wz) <= eps_w)
                obs = np.concatenate(
                    [obs, mode_onehot("hold" if stopped else "walk")])
            else:
                obs = np.concatenate([obs, mode_onehot(mode)])
        if self._recover_plant_q_obs:
            mode = (getattr(self._goal_traj, "mode", "hold")
                    if self._goal_traj is not None else "hold")
            q_plant = np.zeros(N_JOINTS, dtype=float)
            if mode == "recover" and self._state is not None:
                qs = float(cfg_get(self.cfg, "obs", "q_scale",
                                   default=1.0))
                q_plant = ((self._state.joint_position
                            - self._plant_deg * DEG2RAD)
                           / max(qs, 1e-6))
            obs = np.concatenate([obs, q_plant])
        if self._fault_obs:
            er = getattr(self, "_ep_rand", None)
            fh = (er.fault_health() if er is not None
                  else np.ones(N_JOINTS, dtype=np.float32))
            obs = np.concatenate([obs, fh])
        return obs.astype(np.float32)

    def _reset_begin(self, seed: int | None = None):
        # State the obs hook reads must be reset BEFORE _reset_finalize
        # builds the first observation. Done in the pre-physics reset
        # hook so both the C path (reset()) and the batched MJX vec env
        # (which drives _reset_begin directly) get it.
        self._foot_on = [True] * 6
        self._liftoff_xy = [None] * 6
        self._foot_prev_xy = [None] * 6
        self._foot_prev_force = [0.0] * 6
        self._foot_tan_slip_m = [0.0] * 6
        self._duty_hist = []
        self._anchor_xy = [None] * 6
        self._anchor_prev_on = [False] * 6
        self._step_disp_bank = 0.0
        self._ls_prev_xy = [None] * 6
        self._ls_prev_on = [False] * 6
        self._ls_slip_m = 0.0
        self._ls_prog_m = 0.0
        # Anti-park travel-floor EMA (reward.k_walk_idle_charge);
        # per-episode/per-segment, snapshot via MJX_SNAPSHOT_EXTRA.
        self._walk_idle_ema = 0.0
        # Sustained-idle termination counter + its OWN mean-
        # |joint-velocity| EMA (safety.walk_idle_terminate_s):
        # deliberately NOT body speed (a real dragging/skating
        # gait can sit near-zero BODY speed while its legs
        # cycle hard, and cutting that short was measured to
        # rob it of the full-episode loadslip charge the bank
        # relies on; a body-speed version also flagged genuine
        # wrong-direction travel as "idle"). Mean |qvel| across
        # the 18 actuated joints cleanly separates a literally
        # FROZEN policy output (park/belly-sit/tripod-lock:
        # ~5e-5 rad/s, pure settle jitter) from every other
        # scripted behavior including skate/stall (>=0.1 rad/s,
        # ~35x higher) -- see the WALKCURR_PF_IDLE_TERM bank.
        # Seconds low, consecutively; reset to 0 the instant
        # the EMA clears the floor.
        self._walk_idle_low_s = 0.0
        self._walk_qvel_ema = 0.0
        # Seconds since the current commanded-stop segment began
        # (reward.walk_stop_grace_s); 0 whenever s_ref > 1e-3
        # (walking commanded), increments by dt each stop tick.
        # Used only to ramp the stop-speed charge in over the
        # unavoidable deceleration transient -- see the charge
        # site below. Same lifecycle as _walk_idle_ema.
        self._walk_stop_cmd_s = 0.0
        # Structural stop-hold timer (goal.walk_stop_freeze_s);
        # same lifecycle as _walk_stop_cmd_s -- see
        # sim_env._walk_stop_freeze_override.
        self._walk_stop_freeze_cmd_s = 0.0
        # Commanded-course EMA (reward.k_walk_course); same lifecycle.
        self._walk_course_ema = [0.0, 0.0]
        # Stride-EMA velocity for the tracking kernel
        # (reward.walk_kernel_vel_ema); same lifecycle.
        self._walk_kernel_vema = [0.0, 0.0]
        # Stride-EMA yaw-rate for the yaw tracking kernel
        # (reward.walk_kernel_yaw_ema); same lifecycle.
        self._walk_kernel_wz_ema = 0.0
        self._stance_slip_acc = [0.0] * 6
        self._gait_last_step = [0] * 6
        self._gait_cmd_tick = 0
        self._gait_gate_qfactor = 1.0
        self._phase = 0.0
        self._yaw_still_ema = 0.0
        self._yaw_prog_ema = 0.0
        self._vel_est = None
        if self._wc_on:
            # Bucket must be chosen BEFORE super() samples this
            # episode's DR (_ep_rand) so the bucket's dr scale applies
            # to the same episode. Off = zero rng draws, randomizer
            # untouched (bit-exact legacy).
            self._walkcurr_prepare_episode()
        return super()._reset_begin(seed)

    def _reset_finalize(self):
        obs, info = super()._reset_finalize()
        if self.walk_probe_on:
            self._walk_probe_start()
        return obs, info

    # ------------------------------------------------------------------
    # In-env walk quality probe (walk_probe_on; measurement only).
    # Same formulas as train_ppo_transfer.eval_task's external loop,
    # with ONE sourcing difference: foot XY comes from the pad BODIES
    # (self._pad_bids — mirrored per tick into the MJX shims' FakeData,
    # exactly what the reward stack reads) instead of the foot sites
    # (which the batched backend does not mirror). Identical code runs
    # on the C env and both MJX vec envs, so cert numbers are directly
    # comparable across backends.

    def _walk_probe_start(self) -> None:
        self._wp = dict(
            tr=tuple(self._tilt_ref0),
            sat_limit=0.98 * self.safety.max_dq,
            prev_cmd=self.safety._last_safe.copy(),
            prev_on=None, prev_xy=[None] * 6,
            peak_roll=0.0, peak_pitch=0.0, peak_gyro=0.0,
            slip=0.0, sw=0, sat_jt=0, sat_all=0,
            sw_foot=[0] * 6, on_ticks=0,
            h0=float(self.data.xpos[self._chassis_bid, 2]),
            xy0=self.data.xpos[self._chassis_bid, :2].copy(),
            h_sum=0.0, vx_se=0.0, vy_se=0.0, wz_se=0.0, vx_n=0,
            cmd_dist=0.0, prog_m=0.0, cross_m=0.0,
            stop_v_sum=0.0, stop_ticks=0,
            stop_v_sum_settled=0.0, stop_ticks_settled=0,
            stop_seg_s=0.0,
            head_ticks=int(round(2.0 / self.dt)),
            ret=0.0, n=0)

    def _walk_probe_tick(self, reward: float, term: bool, trunc: bool,
                         info: dict) -> None:
        w = self._wp
        w["ret"] += reward
        w["n"] += 1
        n = w["n"]
        st = self._state
        w["peak_roll"] = max(w["peak_roll"],
                             abs(st.imu_roll - w["tr"][0]))
        w["peak_pitch"] = max(w["peak_pitch"],
                              abs(st.imu_pitch - w["tr"][1]))
        w["peak_gyro"] = max(w["peak_gyro"],
                             float(np.max(np.abs(st.imu_gyro[:2]))))
        cmd = self.safety._last_safe
        n_sat = int(np.sum(np.abs(cmd - w["prev_cmd"])
                           >= w["sat_limit"]))
        w["sat_jt"] += n_sat
        w["sat_all"] += int(n_sat >= 6)
        w["prev_cmd"] = cmd.copy()
        on: list[bool] = []
        for f in range(6):
            adr = self._touch_adr[f]
            is_on = bool(adr >= 0
                         and float(self.data.sensordata[adr]) > 0.5)
            b = self._pad_bids[f]
            xy = self.data.xpos[b, :2].copy() if b >= 0 else None
            if w["prev_on"] is not None:
                if is_on != w["prev_on"][f]:
                    w["sw"] += 1
                    w["sw_foot"][f] += 1
                if (is_on and w["prev_on"][f] and xy is not None
                        and w["prev_xy"][f] is not None):
                    w["slip"] += float(np.hypot(*(xy - w["prev_xy"][f])))
            w["prev_xy"][f] = xy
            on.append(is_on)
            w["on_ticks"] += int(is_on)
        w["prev_on"] = on
        w["h_sum"] += float(self.data.xpos[self._chassis_bid, 2])
        vxr = getattr(self._goal_traj, "vx", None)
        if vxr is not None:
            j = min(max(n - 1, 0), len(vxr) - 1)
            vyr = getattr(self._goal_traj, "vy", None)
            wzr = getattr(self._goal_traj, "wz", None)
            vx_meas, vy_meas = self._body_vel_xy()
            vx_c = float(vxr[j])
            vy_c = float(vyr[j]) if vyr is not None else 0.0
            w["vx_se"] += (vx_c - float(vx_meas)) ** 2
            w["vy_se"] += (vy_c - float(vy_meas)) ** 2
            wz_c = float(wzr[j]) if wzr is not None else 0.0
            w["wz_se"] += (wz_c - float(self._body_wz())) ** 2
            w["vx_n"] += 1
            s_ref = float(np.hypot(vx_c, vy_c))
            w["cmd_dist"] += s_ref * self.dt
            if s_ref > 1e-6:
                w["prog_m"] += ((float(vx_meas) * vx_c
                                 + float(vy_meas) * vy_c) / s_ref
                                * self.dt)
                w["cross_m"] += ((float(vy_meas) * vx_c
                                  - float(vx_meas) * vy_c) / s_ref
                                 * self.dt)
                w["stop_seg_s"] = 0.0
            else:
                # Elapsed time already spent in THIS stop segment
                # before this tick (2026-08-24, joyfullcurr10
                # cert-methodology audit: the legacy `stop_v_sum`/
                # `stop_ticks` below count every stop tick from the
                # very first one, with no per-segment settle
                # exemption -- unlike the reward's own
                # `reward.walk_stop_grace_s`, which explicitly ramps
                # the stop charges' multiplier 0->1 over the first
                # grace window because that transient is an
                # unavoidable physical deceleration, not creep. That
                # asymmetry means dosing the reward charge can only
                # ever discipline the POST-grace ticks the cert
                # already weights identically to the pre-grace ones,
                # so a dose-insensitive `stop_speed_m_s` plateau does
                # not by itself prove the creep is unshapeable --
                # `goal.walk_stop_settle_s` (default 0.0, additive
                # metric only, legacy `stop_v_sum`/`stop_ticks` path
                # below is untouched) lets an offline read exclude the
                # first N seconds of each stop segment the same way,
                # surfacing `stop_speed_settled_m_s` for comparison.
                seg_s_before = w["stop_seg_s"]
                w["stop_seg_s"] = seg_s_before + self.dt
                if n > w["head_ticks"]:
                    w["stop_v_sum"] += float(np.hypot(vx_meas, vy_meas))
                    w["stop_ticks"] += 1
                    settle_s = float(cfg_get(
                        self.cfg, "goal", "walk_stop_settle_s",
                        default=0.0))
                    if seg_s_before >= settle_s:
                        w["stop_v_sum_settled"] += float(
                            np.hypot(vx_meas, vy_meas))
                        w["stop_ticks_settled"] += 1
        if term or trunc:
            info["walk_probe"] = self._walk_probe_summary(bool(term))

    def _walk_probe_summary(self, term: bool) -> dict:
        w, nan = self._wp, float("nan")
        n = max(w["n"], 1)
        rad2deg = 180.0 / math.pi
        xyN = self.data.xpos[self._chassis_bid, :2]
        vx_n = w["vx_n"]
        # Body-height telemetry vs the SAME anchor the reward gate uses
        # (z0 + goal.height_ref; walk_height_gate block below). Emitted
        # unconditionally so cert panels can show the crouch depth and
        # the income factor a gated run would keep, whether or not the
        # gate is enabled (operator order fb_20260818T085648_2a0a60:
        # body-height/height-factor on B0 cert + all W&B panels).
        goal = self._current_goal()
        h_ref = float(getattr(goal, "height_ref", 0.0))
        h_err_m = (w["h_sum"] / n - self._z0) - h_ref
        sig_m = float(cfg_get(self.cfg, "reward",
                              "walk_height_sigma_mm",
                              default=30.0)) / 1000.0
        height_factor = math.exp(
            -0.5 * (h_err_m / max(sig_m, 1e-6)) ** 2)
        return {
            "h_err_mm": h_err_m * 1000.0,
            "height_factor": height_factor,
            "return": w["ret"], "ep_len": float(w["n"]),
            "survival_s": float(w["n"] * self.dt),
            "command_changes": float(getattr(
                self._goal_traj, "command_changes", 0)),
            "early_term": float(term),
            "peak_roll_deg": w["peak_roll"] * rad2deg,
            "peak_pitch_deg": w["peak_pitch"] * rad2deg,
            "peak_gyro_dps": w["peak_gyro"] * rad2deg,
            "slip_m": w["slip"],
            "fwd_m": float(np.hypot(*(xyN - w["xy0"]))),
            "contact_sw_per_s": w["sw"] / max(w["n"] * self.dt, 1e-9),
            "slew_sat": w["sat_jt"] / max(w["n"] * 18, 1),
            "slew_sat_all": w["sat_all"] / n,
            "mean_h_m": w["h_sum"] / n,
            "dh_m": (float(self.data.xpos[self._chassis_bid, 2])
                     - w["h0"]),
            "vx_rmse": (float(np.sqrt(w["vx_se"] / vx_n))
                        if vx_n else nan),
            "vy_rmse": (float(np.sqrt(w["vy_se"] / vx_n))
                        if vx_n else nan),
            "wz_rmse_dps": (float(np.sqrt(w["wz_se"] / vx_n)) * rad2deg
                            if vx_n else nan),
            "cmd_prog_m": w["prog_m"],
            "cmd_prog_frac": (w["prog_m"] / w["cmd_dist"]
                              if w["cmd_dist"] > 0.01 else nan),
            "slip_per_m": w["slip"] / max(w["prog_m"], 0.05),
            "cross_track_frac": (abs(w["cross_m"]) / w["cmd_dist"]
                                 if w["cmd_dist"] > 0.01 else nan),
            "wrong_way": (float(w["prog_m"] < 0.0)
                          if w["cmd_dist"] > 0.01 else nan),
            "stop_speed_m_s": (w["stop_v_sum"] / w["stop_ticks"]
                               if w["stop_ticks"] else nan),
            "stop_speed_settled_m_s": (
                w["stop_v_sum_settled"] / w["stop_ticks_settled"]
                if w["stop_ticks_settled"] else nan),
            "stop_ticks_settled_frac": (
                w["stop_ticks_settled"] / w["stop_ticks"]
                if w["stop_ticks"] else nan),
            "foot_sw_min_per_s": (min(w["sw_foot"])
                                  / max(w["n"] * self.dt, 1e-9)),
            "duty_factor": w["on_ticks"] / max(w["n"] * 6, 1),
        }

    # ------------------------------------------------------------------
    # Adaptive competence+retention walk-command curriculum
    # (goal.walk_curriculum=1; WALKCURR_BUCKETS/WALKCURR_MIX above).
    # Same contract as the recover-mode ladder: sampling weights are
    # derived env-side from certification results, the trainer is the
    # only writer of those results (deterministic held-out assays,
    # broadcast to every env), stochastic rollouts can never move the
    # frontier, and locked future buckets are never trained.

    def _walkcurr_prepare_episode(self) -> None:
        gen = self._goal_gen
        if float(getattr(gen, "p_walk", 0.0)) != 1.0:
            raise ValueError(
                "goal.walk_curriculum=1 requires a pure walk diet "
                f"(p_walk=1.0, got {getattr(gen, 'p_walk', 0.0)}); the "
                "curriculum owns the whole command distribution")
        if float(cfg_get(self.cfg, "goal", "mode_seq",
                         default=0.0)) > 0.0:
            raise ValueError("goal.walk_curriculum is incompatible "
                             "with goal.mode_seq")
        force = getattr(self, "force_walk_curr_bucket", None)
        if force is not None:
            b = int(force)
            if not 0 <= b < len(self._wc_table):
                raise ValueError(f"force_walk_curr_bucket {b} out of "
                                 f"range 0..{len(self._wc_table) - 1}")
        else:
            b = self._walkcurr_draw_bucket()
        self._wc_bucket = b
        dr = float(self._wc_table[b]["dr"])
        self.randomizer = self._walkcurr_randomizer(dr)

    def _walkcurr_draw_bucket(self) -> int:
        w = self._walkcurr_weights()
        r = float(self.rng.random())
        return int(np.searchsorted(np.cumsum(w), r,
                                   side="right").clip(0, len(w) - 1))

    def _walkcurr_weights(self) -> np.ndarray:
        """Sampling mixture over UNLOCKED buckets only (operator spec):
        50% frontier, 25% weakest mastered, 15% uniform mastered, 10%
        the rung just prior to the frontier. With no mastered buckets
        every component folds back to the frontier. Locked buckets
        (index >= active_n) get exactly zero mass by construction."""
        n = max(1, int(self._wc_active_n))
        frontier = n - 1
        w = np.zeros(n, dtype=float)
        w[frontier] += WALKCURR_MIX["frontier"]
        mastered = list(range(frontier))
        if mastered:
            weakest = min(
                mastered,
                key=lambda b: self._wc_results.get(
                    b, {}).get("score", float("inf")))
            w[weakest] += WALKCURR_MIX["weakest"]
            for b in mastered:
                w[b] += WALKCURR_MIX["uniform"] / len(mastered)
            w[frontier - 1] += WALKCURR_MIX["prior"]
        else:
            w[frontier] += (WALKCURR_MIX["weakest"]
                            + WALKCURR_MIX["uniform"]
                            + WALKCURR_MIX["prior"])
        return w / w.sum()

    def _walkcurr_randomizer(self, scale: float):
        """Per-bucket DomainRandomizer (cached), with the same cfg
        dr.* absolute-override semantics as sim_env.__init__."""
        key = round(float(scale), 6)
        r = self._wc_randomizers.get(key)
        if r is None:
            from .domain_rand import DomainRandomizer
            r = DomainRandomizer.from_params(self.params, scale=key)
            for _k, _v in (self.cfg.get("dr") or {}).items():
                if not hasattr(r.ranges, _k):
                    raise ValueError(f"unknown DR override dr.{_k}")
                if isinstance(_v, str):
                    _parts = tuple(float(x) for x in _v.split(","))
                    _v = _parts[0] if len(_parts) == 1 else _parts
                setattr(r.ranges, _k, _v)
            self._wc_randomizers[key] = r
        return r

    def apply_walkcurr_certification(self, bucket: int, passed: bool,
                                     score: float,
                                     cert_round: int) -> dict:
        """Record one bucket's deterministic held-out assay (trainer
        broadcast; the ONLY write path into the curriculum). ``score``
        is a continuous competence proxy (cert cmd_prog_frac) used only
        to pick the weakest mastered bucket for replay pressure."""
        bucket = int(bucket)
        if not 0 <= bucket < len(self._wc_table):
            raise ValueError(f"unknown walkcurr bucket {bucket}")
        self._wc_results[bucket] = {
            "passed": bool(passed), "score": float(score),
            "cert_round": int(cert_round)}
        return dict(self._wc_results[bucket], bucket=bucket)

    def walkcurr_update_admission(self, cert_round: int) -> dict:
        """Promote ONLY if the frontier AND every retained bucket
        passed a FRESH assay of this cert round. Never time-based."""
        n = int(self._wc_active_n)
        frontier = n - 1
        rows = {}
        for b in range(n):
            row = self._wc_results.get(b)
            fresh = (row is not None
                     and row.get("cert_round") == int(cert_round))
            rows[b] = {"passed": bool(fresh and row["passed"]),
                       "fresh": bool(fresh),
                       "score": (float(row["score"]) if row is not None
                                 else float("nan"))}
        frontier_passed = rows[frontier]["passed"]
        retained_failed = [b for b in range(frontier)
                           if not rows[b]["passed"]]
        retention_passed = not retained_failed
        promoted = False
        if (frontier_passed and retention_passed
                and self._wc_active_n < len(self._wc_table)):
            self._wc_active_n += 1
            promoted = True
        return {
            "cert_round": int(cert_round),
            "frontier_bucket": frontier,
            "frontier_passed": bool(frontier_passed),
            "retention_passed": bool(retention_passed),
            "retained_failed_buckets": retained_failed,
            "promoted": bool(promoted),
            "active_n": int(self._wc_active_n),
            "buckets": rows,
        }

    def walkcurr_state(self) -> dict:
        """Serializable telemetry snapshot (weights + results)."""
        w = self._walkcurr_weights()
        frontier = self._wc_active_n - 1
        mastered = list(range(frontier))
        weakest = (min(mastered,
                       key=lambda b: self._wc_results.get(
                           b, {}).get("score", float("inf")))
                   if mastered else -1)
        return {
            "total_buckets": len(self._wc_table),
            "active_n": int(self._wc_active_n),
            "frontier_bucket": int(frontier),
            "weakest_mastered": int(weakest),
            "sample_probabilities": {str(b): float(p)
                                     for b, p in enumerate(w)},
            "results": {str(b): dict(r)
                        for b, r in self._wc_results.items()},
        }

    def apply_drag_allow_frac(self, frac: float) -> dict:
        """Move the live drag_stance allowance to ``frac`` of the ramp
        (0 = loose/noisy-safe start, 1 = the cfg target allowance);
        trainer-driven — see the ``reward.drag_stance_allow_ramp_steps``
        block in ``__init__``. Mirrors ``apply_profile_ramp_frac``'s
        contract exactly: raises when the ramp is not armed, so a
        broadcast that silently no-ops is never a hidden failure mode.
        """
        if self._drag_allow_ramp is None:
            raise RuntimeError(
                "apply_drag_allow_frac called but reward."
                "drag_stance_allow_ramp_steps is not set (>0) in this "
                "env's cfg — the drag-allow ramp is not armed")
        f = min(max(float(frac), 0.0), 1.0)
        s = self._drag_allow_ramp["start_m"]
        t = self._drag_allow_ramp["target_m"]
        self._drag_allow_override_m = s + f * (t - s)
        self._drag_allow_ramp["frac"] = f
        return {"frac": f, "allow_mm": self._drag_allow_override_m * 1000.0}

    def apply_term_penalty_frac(self, frac: float) -> dict:
        """Move the live termination penalty to ``frac`` of the ramp
        (0 = lenient start, 1 = the validated cfg target); trainer-
        driven — see the ``reward.term_penalty_ramp_steps`` block in
        ``__init__``. Mirrors ``apply_drag_allow_frac``'s contract
        exactly: raises when the ramp is not armed.
        """
        if self._term_penalty_ramp is None:
            raise RuntimeError(
                "apply_term_penalty_frac called but reward."
                "term_penalty_ramp_steps is not set (>0) in this "
                "env's cfg — the term-penalty ramp is not armed")
        f = min(max(float(frac), 0.0), 1.0)
        s = self._term_penalty_ramp["start"]
        t = self._term_penalty_ramp["target"]
        self._term_penalty_override = s + f * (t - s)
        self._term_penalty_ramp["frac"] = f
        return {"frac": f, "term_penalty": self._term_penalty_override}

    def apply_walk_charge_frac(self, frac: float) -> dict:
        """Move the live dense-walk-charge scale to ``frac`` of the
        ramp (0 = walk_charge_ramp_min_frac of every scaled charge,
        1 = the full bank-proven dose); trainer-driven — see the
        ``reward.walk_charge_ramp_steps`` block in ``__init__``.
        Mirrors ``apply_term_penalty_frac``'s contract exactly:
        raises when the ramp is not armed, so a broadcast that
        silently no-ops is never a hidden failure mode.
        """
        if self._walk_charge_ramp is None:
            raise RuntimeError(
                "apply_walk_charge_frac called but reward."
                "walk_charge_ramp_steps is not set (>0) in this "
                "env's cfg — the walk-charge ramp is not armed")
        f = min(max(float(frac), 0.0), 1.0)
        m = self._walk_charge_ramp["min_frac"]
        self._walk_charge_override = m + f * (1.0 - m)
        self._walk_charge_ramp["frac"] = f
        return {"frac": f, "charge_scale": self._walk_charge_override}

    def _walk_charge_scale(self) -> float:
        """Live scale on the three discovery-friction walk charges
        (k_park_duty, k_walk_idle_charge, k_walk_heading — see the
        walk-charge ramp block in ``__init__``; k_loadslip_excess is
        excluded and never scaled): 1.0 (bit-exact legacy / full
        bank-proven dose) unless the trainer has broadcast a ramp
        frac."""
        ov = self._walk_charge_override
        return 1.0 if ov is None else ov

    def apply_loadslip_bootstrap_frac(self, frac: float) -> dict:
        """Move the live k_loadslip_excess scale to ``frac`` of the
        bootstrap (0 = walk_loadslip_bootstrap_min_frac of the
        bank-proven charge, 1 = full dose); trainer-driven — see the
        ``reward.walk_loadslip_bootstrap_steps`` block in ``__init__``.
        Mirrors ``apply_walk_charge_frac``'s contract exactly: raises
        when the bootstrap is not armed, so a broadcast that silently
        no-ops is never a hidden failure mode."""
        if self._ls_bootstrap is None:
            raise RuntimeError(
                "apply_loadslip_bootstrap_frac called but reward."
                "walk_loadslip_bootstrap_steps is not set (>0) in "
                "this env's cfg — the loadslip bootstrap is not armed")
        f = min(max(float(frac), 0.0), 1.0)
        m = self._ls_bootstrap["min_frac"]
        self._ls_bootstrap_override = m + f * (1.0 - m)
        self._ls_bootstrap["frac"] = f
        return {"frac": f, "excess_scale": self._ls_bootstrap_override}

    def _loadslip_excess_scale(self) -> float:
        """Live scale on k_loadslip_excess (see the loadslip-bootstrap
        block in ``__init__``): 1.0 (bit-exact legacy / full
        bank-proven dose) unless the trainer has broadcast a bootstrap
        frac."""
        ov = self._ls_bootstrap_override
        return 1.0 if ov is None else ov

    def walkcurr_checkpoint_state(self) -> dict:
        """State paired with a promotion checkpoint (rollback/resume)."""
        return {"active_n": int(self._wc_active_n),
                "results": {str(b): dict(r)
                            for b, r in self._wc_results.items()}}

    def restore_walkcurr_checkpoint_state(self, state: dict) -> None:
        active_n = int(state["active_n"])
        if not 1 <= active_n <= len(self._wc_table):
            raise ValueError(f"invalid walkcurr active_n {active_n}")
        self._wc_active_n = active_n
        self._wc_results = {
            int(b): {"passed": bool(r["passed"]),
                     "score": float(r["score"]),
                     "cert_round": int(r["cert_round"])}
            for b, r in dict(state["results"]).items()}

    def _sample_walk_curr(self) -> WalkTrajectory:
        """Curriculum walk episode: the stashed bucket's spec fully
        defines the command distribution (legacy goal.walk_cmd_* keys
        are ignored while the curriculum owns sampling). Command
        grammar matches the legacy sampler: 1 s zero hold + 1 s ramp,
        then optional resampled segments with blends and stops."""
        b = self._wc_bucket
        if b is None:
            raise RuntimeError("walkcurr episode without a prepared "
                               "bucket (reset ordering bug)")
        spec = self._wc_table[b]
        n = self.episode_steps + 1
        rng = self.rng
        duration_steps = int(round(float(spec.get(
            "duration_s", self.episode_steps * self.dt)) / self.dt))
        command_n = min(n, duration_steps + 1)
        command_changes = 0

        def draw_cmd() -> tuple[float, float]:
            speed = float(rng.uniform(spec["s_lo"], spec["s_hi"]))
            if spec["head_hi"] <= 0.0:
                ang = 0.0
            else:
                mag = float(rng.uniform(spec["head_lo"],
                                        spec["head_hi"]))
                ang = mag if rng.random() < 0.5 else -mag
            return speed * math.cos(ang), speed * math.sin(ang)

        vx_t, vy_t = draw_cmd()
        hold_n = max(1, int(round(1.0 / self.dt)))
        ramp_n = max(1, int(round(1.0 / self.dt)))
        vx = np.full(n, vx_t)
        vy = np.full(n, vy_t)
        vx[:hold_n] = 0.0
        vy[:hold_n] = 0.0
        end = min(hold_n + ramp_n, n)
        vx[hold_n:end] = np.linspace(0.0, vx_t, end - hold_n)
        vy[hold_n:end] = np.linspace(0.0, vy_t, end - hold_n)
        rs_s = float(spec["resample_s"])
        # V7 stress-diet extras (walk_task.py WALKCURR_BUCKETS_V7
        # banner): missing on every V1-V6 bucket, so `.get` defaults
        # keep this branch a true no-op (zero extra rng draws) for
        # every pre-existing table.
        wz_max = float(spec.get("wz_max", 0.0))
        wz_zero_frac = float(spec.get("wz_zero_frac", 1.0))
        reversal_frac = float(spec.get("reversal_frac", 0.0))
        wz = np.zeros(n) if wz_max > 0.0 else None
        if rs_s > 0.0:
            jit = float(spec["jitter"])
            bl_lo, bl_hi = float(spec["blend_lo"]), float(spec["blend_hi"])

            def seg_len() -> int:
                s = rs_s if jit <= 0.0 \
                    else rs_s * float(rng.uniform(1.0 - jit, 1.0 + jit))
                return max(1, int(round(max(s, self.dt) / self.dt)))

            def blend_len() -> int:
                bl = bl_lo if bl_hi <= bl_lo \
                    else float(rng.uniform(bl_lo, bl_hi))
                if bl <= 0.0:
                    return 0
                return max(1, int(round(max(bl, self.dt) / self.dt)))

            cvx, cvy, cwz = vx_t, vy_t, 0.0
            i = hold_n + ramp_n + seg_len()
            while i < command_n:
                if rng.random() < float(spec["stop_frac"]):
                    nvx = nvy = 0.0
                elif reversal_frac > 0.0 and rng.random() < reversal_frac:
                    # Bucket-diet analogue of stress_mix's flip_180: an
                    # instantaneous full reversal of the current
                    # command (the joygate's dominant held-out failure
                    # mode, over_current on stop/reverse, is never
                    # PRACTICED by the plain V6 sampler).
                    nvx, nvy = -cvx, -cvy
                else:
                    nvx, nvy = draw_cmd()
                n_blend = blend_len()
                end_b = min(i + n_blend, command_n)
                if n_blend:
                    vx[i:end_b] = np.linspace(cvx, nvx, end_b - i)
                    vy[i:end_b] = np.linspace(cvy, nvy, end_b - i)
                vx[end_b:command_n] = nvx
                vy[end_b:command_n] = nvy
                cvx, cvy = nvx, nvy
                if wz is not None:
                    nwz = (0.0 if rng.random() < wz_zero_frac
                          else float(rng.uniform(-wz_max, wz_max)))
                    if n_blend:
                        wz[i:end_b] = np.linspace(cwz, nwz, end_b - i)
                    wz[end_b:command_n] = nwz
                    cwz = nwz
                command_changes += 1
                i += seg_len()
        zeros = np.zeros(n)
        self._walk_bucket = None
        traj = WalkTrajectory(mode="walk", roll=zeros, pitch=zeros,
                              height=zeros, unload_leg=None,
                              start_at="plant", vx=vx, vy=vy, wz=wz,
                              cmd_mode="walkcurr",
                              duration_steps=duration_steps,
                              command_changes=command_changes)
        return traj

    def set_walk_bucket_weights(self, w) -> None:
        """LP-curriculum hook (called via VecEnv.env_method)."""
        w = np.clip(np.asarray(w, dtype=float), 0.0, None)
        s = float(w.sum())
        self._lp_weights = (w / s) if s > 0 else None

    def _sample_walk(self) -> WalkTrajectory:
        if self._wc_on:
            # Adaptive curriculum owns the whole command distribution
            # (bucket stashed by _walkcurr_prepare_episode pre-DR).
            return self._sample_walk_curr()
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
        # Discrete commanded-heading SET (operator staged-curriculum
        # order 2026-08-22, fb_20260822T032514: forward-only first,
        # then a SMALL HEADING SET, then full fixed headings — never
        # full +-180 from tick zero). goal.walk_heading_set, a JSON
        # list of radians (--cfg-set 'goal.walk_heading_set=[0,0.7854,
        # -0.7854]') or a comma-separated string, draws the episode
        # heading uniformly FROM THE SET and overrides
        # walk_heading_max_rad; mid-episode resamples (draw_heading
        # below) use the same set. Default "" = off: no draw happens,
        # every legacy rng stream is bit-exact.
        h_set_raw = cfg_get(self.cfg, "goal", "walk_heading_set",
                            default="")
        if isinstance(h_set_raw, (list, tuple)):
            h_set = [float(x) for x in h_set_raw]
        elif isinstance(h_set_raw, str) and h_set_raw.strip():
            h_set = [float(x) for x in h_set_raw.split(",")]
        else:
            h_set = []
        if h_set:
            ang = h_set[int(rng.integers(len(h_set)))]
        elif h_max >= 0.0:
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
        cmd_mode = str(cfg_get(self.cfg, "goal", "walk_cmd_mode",
                               default="legacy")).strip().lower()
        # goal.walk_cmd_stage curriculum (see WALK_CMD_STAGE_FAMILIES
        # above): only shapes stress_mix draws; default -1 keeps the
        # legacy uniform family choice draw-stream bit-exact.
        stage_f = float(cfg_get(self.cfg, "goal", "walk_cmd_stage",
                                default=-1.0))
        stage0 = False
        if cmd_mode == "stress_mix":
            if stage_f >= 0.0:
                s = min(int(stage_f), len(WALK_CMD_STAGE_FAMILIES) - 1)
                fams = tuple(f for tier in WALK_CMD_STAGE_FAMILIES[:s + 1]
                             for f in tier)
                cmd_mode = str(rng.choice(fams))
                if s == 0:
                    stage0 = True
                    ang = 0.0    # pure forward/back stepping first
            else:
                cmd_mode = str(rng.choice(WALK_CMD_SCHEDULES))
        elif cmd_mode != "legacy" and cmd_mode not in WALK_CMD_SCHEDULES:
            raise ValueError(
                f"unknown goal.walk_cmd_mode={cmd_mode!r}; expected "
                f"legacy, stress_mix, or one of {WALK_CMD_SCHEDULES}")
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
                if stage0:
                    return 0.0   # stage-0 curriculum: fwd/back only
                if h_set:
                    return h_set[int(rng.integers(len(h_set)))]
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
                if b <= 0.0:
                    return 0
                return max(1, int(round(max(b, self.dt) / self.dt)))

            cvx, cvy = vx_t, vy_t
            cwz = float(wz[min(end, n - 1)]) if wz is not None else 0.0
            if cmd_mode == "sweep_circle":
                period_s = max(float(cfg_get(
                    self.cfg, "goal", "walk_cmd_sweep_period_s",
                    default=12.0)), self.dt)
                sweep_sign = -1.0 if rng.random() < 0.5 else 1.0
                idx = np.arange(max(n - end, 0), dtype=float)
                theta = ang + sweep_sign * 2.0 * math.pi * (
                    idx * self.dt / period_s)
                vx[end:] = speed * np.cos(theta)
                vy[end:] = speed * np.sin(theta)
            else:
                square_sign = None
                if cmd_mode == "square":
                    square_sign = -1.0 if rng.random() < 0.5 else 1.0
                stop_next = True
                i = hold_n + ramp_n + seg_len()
                while i < n:
                    if cmd_mode in ("legacy", "random_hold"):
                        if rng.random() < stop_frac:
                            nvx = nvy = 0.0
                        else:
                            s2 = float(rng.uniform(s_lo, s_hi))
                            a2 = draw_heading()
                            nvx = s2 * math.cos(a2)
                            nvy = s2 * math.sin(a2)
                    elif cmd_mode == "flip_180":
                        nvx, nvy = -cvx, -cvy
                    elif cmd_mode == "square":
                        a2 = math.atan2(cvy, cvx) + (
                            square_sign * math.pi / 2.0)
                        s2 = max(math.hypot(cvx, cvy), s_lo)
                        nvx = s2 * math.cos(a2)
                        nvy = s2 * math.sin(a2)
                    elif cmd_mode == "stop_go":
                        if stop_next:
                            nvx = nvy = 0.0
                        else:
                            s2 = float(rng.uniform(s_lo, s_hi))
                            a2 = draw_heading()
                            nvx = s2 * math.cos(a2)
                            nvy = s2 * math.sin(a2)
                        stop_next = not stop_next
                    else:  # jitter
                        a2 = math.atan2(cvy, cvx) + float(rng.uniform(
                            -float(cfg_get(
                                self.cfg, "goal", "walk_cmd_jitter_rad",
                                default=0.25)),
                            float(cfg_get(
                                self.cfg, "goal", "walk_cmd_jitter_rad",
                                default=0.25))))
                        s2 = float(rng.uniform(s_lo, s_hi))
                        nvx = s2 * math.cos(a2)
                        nvy = s2 * math.sin(a2)
                    n_blend = blend_len()
                    end_b = min(i + n_blend, n)
                    if n_blend:
                        vx[i:end_b] = np.linspace(cvx, nvx, end_b - i)
                        vy[i:end_b] = np.linspace(cvy, nvy, end_b - i)
                    vx[end_b:] = nvx
                    vy[end_b:] = nvy
                    cvx, cvy = nvx, nvy
                    if wz is not None:
                        nwz = draw_wz()
                        if n_blend:
                            wz[i:end_b] = np.linspace(
                                cwz, nwz, end_b - i)
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
            # Keep the head-replacement targets consistent: a gait
            # spawn drawn below must CONTINUE this turn-in-place
            # command, not resurrect the discarded linear draw
            # (pre-existing latent interaction — no prior config ran
            # tip_frac and gait_start_frac together).
            vx_t = 0.0
            vy_t = 0.0
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
            # Turn-state reset densification (08-23): when
            # goal.walk_gait_spawn_wz > 0 (default 0 = off, legacy
            # commands and rng streams bit-exact) the yaw command is
            # ALSO live from the start (same 0.3 s fast ramp), matching
            # the mid-rotation spawn pose sim_env builds from traj.wz —
            # otherwise the policy would wake up turning under a zero
            # yaw command it is then scored against.
            spawn_wz = float(cfg_get(self.cfg, "goal",
                                     "walk_gait_spawn_wz", default=0.0))
            if spawn_wz > 0.0 and wz is not None:
                wz_tgt = float(wz[min(head, n - 1)])
                wz[:head] = wz_tgt
                wz[:ramp_fast] = np.linspace(0.0, wz_tgt, ramp_fast)
            if h_off != 0.0:
                height[:head] = h_off
                height[:ramp_fast] = np.linspace(0.0, h_off, ramp_fast)
        return WalkTrajectory(mode="walk", roll=zeros, pitch=zeros,
                              height=height, unload_leg=None,
                              start_at=start_at, vx=vx, vy=vy, wz=wz,
                              cmd_mode=cmd_mode)

    def _sample_quadwalk(self) -> WalkTrajectory:
        """QUADWALK mode (quad track, 08-13 spec): commanded planar
        walking on the four support legs with the front pair
        (goal.quad_lift_legs) raised as hands.

        Command interface = walk's (vx/vy refs + the goal one-hot
        lighting the lift legs; obs width unchanged, mode one-hot
        family "quad"). The walk reward block prices it with lift-leg
        exemptions (park-duty window and step/swing credit skip the
        lift legs) PLUS the quad clear/plant income, so an honest
        rear-four gait out-earns a six-leg walk, a fronts-down drag
        and a freeze — pinned by the QUADWALK semantics bank.

        Discovery-scope defaults, all cfg-overridable: slower command
        band than walk (four feet, smaller support polygon), forward
        only, a longer settle head (the fronts must lift before the
        ramp — matches goal.quad_grace_s + ramp), heading-hold yaw,
        no mid-episode resample.
        """
        n = self.episode_steps + 1
        rng = self.rng
        s_lo = float(cfg_get(self.cfg, "goal", "quadwalk_speed_min_m_s",
                             default=0.02))
        s_hi = float(cfg_get(self.cfg, "goal", "quadwalk_speed_max_m_s",
                             default=0.05))
        h_max = float(cfg_get(self.cfg, "goal", "quadwalk_heading_max_rad",
                              default=0.0))
        hold_s = float(cfg_get(self.cfg, "goal", "quadwalk_hold_s",
                               default=2.0))
        speed = float(rng.uniform(s_lo, s_hi))
        ang = 0.0 if h_max <= 0.0 else float(rng.uniform(-h_max, h_max))
        vx_t = speed * math.cos(ang)
        vy_t = speed * math.sin(ang)
        hold_n = max(1, int(round(hold_s / self.dt)))
        ramp_n = max(1, int(round(1.0 / self.dt)))
        vx = np.full(n, vx_t)
        vy = np.full(n, vy_t)
        vx[:hold_n] = 0.0
        vy[:hold_n] = 0.0
        end = min(hold_n + ramp_n, n)
        vx[hold_n:end] = np.linspace(0.0, vx_t, end - hold_n)
        vy[hold_n:end] = np.linspace(0.0, vy_t, end - hold_n)
        zeros = np.zeros(n)
        wz = np.zeros(n) if self._yaw_cmd else None
        # Spawn kind (08-13, quad track, after cw-quadwalk1/2/3): from
        # the legacy six-foot "plant" start the warm-started six-leg
        # walk basin survives both 3x lift income AND a live per-tick
        # ground-contact charge worth ~40% of episode return (verdict
        # chain in rl_docs/runs/cw-quadwalk{1,2,3}.md) — pricing is an
        # exhausted lever class; the blocker is exploration. "quad"
        # spawns the episode ALREADY IN the fronts-tucked four-leg
        # stance (built env-side in sim_env._reset_begin, kind
        # "quadstance") so rear-four stepping is the natural thing to
        # try and six-leg walking requires actively planting the
        # charged fronts. Default "plant" = legacy bit-exact (same
        # start_at literal, no extra rng draw).
        start = str(cfg_get(self.cfg, "goal", "quadwalk_start",
                            default="plant"))
        if start not in ("plant", "quad"):
            raise ValueError(
                f"goal.quadwalk_start must be 'plant' or 'quad', "
                f"got {start!r}")
        return WalkTrajectory(mode="quadwalk", roll=zeros, pitch=zeros,
                              height=zeros, unload_leg=None,
                              lift_legs=tuple(getattr(
                                  self._goal_gen, "quad_legs", (0, 5))),
                              start_at=("quadstance" if start == "quad"
                                        else "plant"),
                              vx=vx, vy=vy, wz=wz)

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
            # goal.getup_forward_only=1 (RISE_WALK_NEXT_48H P1): the
            # minimal unified rise->walk task — commands are forward
            # or stop ONLY, no lateral/diagonal targets. The angle
            # draws still happen (and are discarded) so rng streams —
            # and hence start kinds, stop patterns, DR — are seed-
            # identical to the full task for A/B. Default 0 bit-exact.
            fwd_only = bool(int(cfg_get(self.cfg, "goal",
                                        "getup_forward_only",
                                        default=0)))
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
                    if fwd_only:
                        ang = 0.0
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

    # ---- recover_to_plant (08-15, operator directive
    # fb_20260815T165306_606974): reach a full-height, level, quiet
    # standing pose with ALL SIX feet loaded, from any recoverable
    # start, then HOLD it 0.5 s — the episode ends on held success.
    # Zero velocity command throughout (this is the recovery
    # specialist; walking is another mode's job). Start-state
    # curriculum = difficulty FAMILIES of start kinds, unlocked
    # monotonically from per-kind deterministic certification fractions
    # (bucket 0 alone first), with bucket-level spaced replay forever.
    # Reward is a potential DIFFERENCE (PBRS) on
    # bounded [0,1] features + one-shot success bonus + a
    # rate-normalized time tax — no occupancy/hold income, no alive
    # bonus (see _recover_reward / REWARD.md §4c).
    #
    # Backward curriculum from the goal boundary.  Buckets are
    # zero-indexed in telemetry and forced eval:
    #   B0 plant_catch: nominal plant + <=2 deg joint noise; hold it.
    #   B1 onefoot_micro: one foot perturbed 3-8 deg.
    #   B2 onefoot_mid:   one foot perturbed 8-15 deg.
    #   B3 onefoot:       one foot perturbed 15-30 deg.
    #   B4 park:          a full alternating tripod is lifted.
    #   B5-B7 shallow/medium/deep all-feet crouches.
    #   B8-B10 high/mid/low partial curls toward the belly-zero pose.
    #   B11 zero: belly-zero with small joint jitter.
    #   B12-B13 25/50% blends toward random legal tangles.
    #   B14-B15 full-height terminal repairs with one/two badly misplaced
    #   legs. These explicitly teach the failure seen at the old B14:
    #   upright and quiet, but parked forever on only four/five feet.
    #   B16-B20 60/70/80/90/100% random-legal tangle blends.
    #   B21 harvested on-path bank states; B22 side/back/upside-down drops.
    #   Sub-90-degree
    #   constructor tilts roll back upright during limp settle, so they
    #   are deliberately not represented as fake curriculum rungs.
    # Keeping the one-foot severities separate matters: the original
    # bucket 1 mixed a 12-30 deg single-foot correction with a tripod
    # park, and produced zero success despite millions of steps.
    RECOVER_FAMILIES = (("plant_catch",),
                        ("onefoot_micro",),
                        ("onefoot_mid",),
                        ("onefoot",),
                        ("park",),
                        ("crouch_shallow",),
                        ("crouch_mid",),
                        ("crouch_deep",),
                        ("partial_high",),
                        ("partial_mid",),
                        ("partial_low",),
                        ("zero",),
                        ("tangle_mild",),
                        ("tangle_mid",),
                        ("repair_one",),
                        ("repair_two",),
                        ("tangle_60",),
                        ("tangle_70",),
                        ("tangle_80",),
                        ("tangle_90",),
                        ("tangle",),
                        ("bank",),
                        ("flip",))
    RECOVER_KIND_IDS = {
        kind: i for i, kind in enumerate(
            kind for family in RECOVER_FAMILIES for kind in family)
    }
    RECOVER_KIND_BUCKETS = {
        kind: bucket for bucket, family in enumerate(RECOVER_FAMILIES)
        for kind in family
    }

    def _recover_family_kinds(self, bucket: int) -> list:
        """Available kinds in one bucket (bank requires a configured file)."""
        has_bank = cfg_get(self.cfg, "goal", "recover_start_bank",
                           default=None) is not None
        return [k for k in self.RECOVER_FAMILIES[bucket]
                if k != "bank" or has_bank]

    def _recover_active_kinds(self) -> list:
        """Kinds in every monotonically unlocked family."""
        kinds = []
        for bucket in range(self._rec_active_n):
            kinds += self._recover_family_kinds(bucket)
        return kinds

    def _recover_bucket_certification(self, bucket: int) -> dict | None:
        """Latest complete deterministic assay for one bucket."""
        kinds = self._recover_family_kinds(bucket)
        stats = [self._rec_stats.get(k, (0, 0)) for k in kinds]
        if not stats or any(episodes <= 0 for _successes, episodes in stats):
            return None
        successes = sum(v[0] for v in stats)
        episodes = sum(v[1] for v in stats)
        fractions = [s / n for s, n in stats]
        return {
            "success_fraction": successes / episodes,
            # Multi-kind buckets promote and remediate on their weakest
            # kind so an easy bank cannot hide a failing random tangle.
            "gate_fraction": min(fractions),
            "successes": successes,
            "episodes": episodes,
        }

    def _recover_refresh_weak_bucket(self) -> None:
        """Point replay pressure at the weakest certified old bucket."""
        candidates = []
        for bucket in range(self._rec_focus_bucket):
            row = self._recover_bucket_certification(bucket)
            if row is not None:
                candidates.append((row["gate_fraction"], -bucket, bucket))
        self._rec_weak_bucket = min(candidates)[2] if candidates else None

    def _recover_training_error_distribution(
            self, n: int | None = None) -> np.ndarray | None:
        """Evidence-weighted replay priority from training shortfall.

        Raw stochastic recovery success is a deliberately strict and noisy
        signal (exploration can break the continuous six-foot hold).  The
        sampler therefore uses terminal goal-potential shortfall instead,
        with safety terminations recorded as maximum error.  This signal can
        allocate a bounded replay slice but can never certify a bucket.
        """
        n = max(1, int(self._rec_active_n if n is None else n))
        min_episodes = max(1, int(float(cfg_get(
            self.cfg, "goal", "recover_training_error_min_episodes",
            default=8))))
        power = max(0.0, float(cfg_get(
            self.cfg, "goal", "recover_training_error_power",
            default=2.0)))
        priority = np.zeros(n, dtype=float)
        for bucket in range(n):
            error, episodes = self._rec_training_error_stats.get(
                bucket, (0.0, 0))
            if episodes < min_episodes:
                continue
            confidence = min(float(episodes) / min_episodes, 1.0)
            priority[bucket] = confidence * max(float(error), 0.0) ** power
        total = float(priority.sum())
        return priority / total if total > 0.0 else None

    def apply_recover_training_error_batch(self, rows: dict) -> None:
        """Fold global non-RSI training outcomes into sampler-only EMAs."""
        beta = float(np.clip(cfg_get(
            self.cfg, "goal", "recover_training_error_ema_beta",
            default=0.25), 0.0, 1.0))
        for raw_bucket, values in rows.items():
            bucket = int(raw_bucket)
            if not 0 <= bucket < len(self.RECOVER_FAMILIES):
                continue
            error_sum, episodes = values
            episodes = int(episodes)
            if episodes <= 0:
                continue
            batch_error = float(np.clip(
                float(error_sum) / episodes, 0.0, 1.0))
            old_error, old_n = self._rec_training_error_stats.get(
                bucket, (batch_error, 0))
            updated = (batch_error if old_n == 0 else
                       (1.0 - beta) * old_error + beta * batch_error)
            self._rec_training_error_stats[bucket] = (
                float(updated), int(old_n) + episodes)

    def _recover_bucket_weights(self) -> np.ndarray:
        """Spaced-replay probabilities over unlocked recovery buckets.

        The mass is assigned by BUCKET, not by start kind: 50% to the
        acquisition frontier, 25% geometrically over its three immediate
        predecessors, 15% to the weakest certified old bucket, and 10%
        uniformly over all remaining unlocked buckets. Empty components
        fall back to the frontier. A multi-kind family splits its bucket
        probability later, so adding a bank never doubles that level's
        training share.  Once enough terminal evidence exists, a bounded
        sampler-only slice is redistributed toward buckets with the largest
        terminal goal-potential shortfall.
        """
        n = max(1, int(self._rec_active_n))
        focus = min(max(int(self._rec_focus_bucket), 0), n - 1)
        w = np.zeros(n, dtype=float)
        focus_mass = float(cfg_get(
            self.cfg, "goal", "recover_focus_mix", default=0.50))
        recent_mass = float(cfg_get(
            self.cfg, "goal", "recover_recent_mix", default=0.25))
        weak_mass = float(cfg_get(
            self.cfg, "goal", "recover_weak_mix", default=0.15))
        uniform_mass = float(cfg_get(
            self.cfg, "goal", "recover_uniform_mix", default=0.10))
        masses = np.maximum(
            np.asarray([focus_mass, recent_mass, weak_mass, uniform_mass],
                       dtype=float), 0.0)
        if float(masses.sum()) <= 0.0:
            masses[0] = 1.0
        masses /= masses.sum()
        focus_mass, recent_mass, weak_mass, uniform_mass = masses
        w[focus] += focus_mass

        recent = [focus - d for d in range(1, 4) if focus - d >= 0]
        if recent:
            shape = np.asarray((0.50, 0.30, 0.20)[:len(recent)],
                               dtype=float)
            shape /= shape.sum()
            for bucket, share in zip(recent, shape):
                w[bucket] += recent_mass * share
        else:
            w[focus] += recent_mass

        weak = self._rec_weak_bucket
        if weak is not None and 0 <= int(weak) < n and int(weak) != focus:
            w[int(weak)] += weak_mass
        else:
            w[focus] += weak_mass

        reserved = {focus, *recent}
        if weak is not None and 0 <= int(weak) < n:
            reserved.add(int(weak))
        others = [bucket for bucket in range(n) if bucket not in reserved]
        if others:
            for bucket in others:
                w[bucket] += uniform_mass / len(others)
        else:
            w[focus] += uniform_mass
        w /= w.sum()
        error_distribution = self._recover_training_error_distribution(n)
        error_mix = float(np.clip(cfg_get(
            self.cfg, "goal", "recover_training_error_mix", default=0.10),
            0.0, 1.0))
        if error_distribution is not None and error_mix > 0.0:
            w = (1.0 - error_mix) * w + error_mix * error_distribution
        return w / w.sum()

    def _recover_kind_weights(self, kinds: list) -> np.ndarray:
        """Map bucket replay mass to kinds, splitting families evenly."""
        bucket_w = self._recover_bucket_weights()
        w = []
        for kind in kinds:
            bucket = self.RECOVER_KIND_BUCKETS[kind]
            family_n = len(self._recover_family_kinds(bucket))
            w.append(bucket_w[bucket] / max(family_n, 1))
        w = np.asarray(w, dtype=float)
        return w / w.sum()

    def _recover_admission_status(self,
                                  cert_round: int | None = None) -> dict:
        """Return the frontier and retention-suite gate state."""
        self._rec_focus_bucket = self._rec_active_n - 1
        admit_n = int(float(cfg_get(
            self.cfg, "goal", "recover_admit_n", default=4)))
        threshold = float(cfg_get(
            self.cfg, "goal", "recover_admit_fraction", default=0.8))
        bucket_rows = {}
        for bucket in range(self._rec_active_n):
            kinds = self._recover_family_kinds(bucket)
            passed = bool(kinds)
            fresh = bool(kinds)
            fractions = []
            for kind in kinds:
                successes, episodes = self._rec_stats.get(kind, (0, 0))
                fraction = successes / episodes if episodes else 0.0
                fractions.append(fraction)
                passed = (passed and episodes >= admit_n
                          and fraction >= threshold)
                if cert_round is not None:
                    fresh = (fresh and self._rec_cert_rounds.get(kind)
                             == int(cert_round))
            bucket_rows[bucket] = {
                "passed": bool(passed and fresh),
                "score_passed": bool(passed),
                "fresh": bool(fresh),
                "gate_fraction": (min(fractions) if fractions else 0.0),
            }
        focus = self._rec_focus_bucket
        frontier_passed = bool(bucket_rows.get(
            focus, {}).get("passed", False))
        retention = [bucket_rows[b] for b in range(focus)]
        retention_passed = all(row["passed"] for row in retention)
        failed = [bucket for bucket, row in bucket_rows.items()
                  if not row["passed"]]
        retention_failed = [bucket for bucket in range(focus)
                            if not bucket_rows[bucket]["passed"]]
        return {
            "cert_round": (-1 if cert_round is None else int(cert_round)),
            "frontier_bucket": int(focus),
            "frontier_passed": frontier_passed,
            "retention_passed": bool(retention_passed),
            "suite_passed": bool(frontier_passed and retention_passed),
            "retention_bucket_count": int(focus),
            "failed_buckets": failed,
            "retention_failed_buckets": retention_failed,
            "retention_min_gate_fraction": min(
                (row["gate_fraction"] for row in retention), default=1.0),
            "min_gate_fraction": min(
                (row["gate_fraction"] for row in bucket_rows.values()),
                default=0.0),
            "buckets": bucket_rows,
        }

    def _recover_update_admission(
            self, cert_round: int | None = None) -> dict:
        """Unlock only after frontier plus retention suite pass."""
        status = self._recover_admission_status(cert_round)
        before = self._rec_active_n
        if (self._rec_active_n < len(self.RECOVER_FAMILIES)
                and status["suite_passed"]):
            self._rec_active_n += 1
            self._rec_focus_bucket = self._rec_active_n - 1
        self._recover_refresh_weak_bucket()
        status.update({
            "active_before": int(before),
            "active_after": int(self._rec_active_n),
            "promoted": bool(self._rec_active_n > before),
        })
        return status

    def recover_score_state(self) -> dict:
        """Serializable deterministic curriculum/scoreboard snapshot."""
        self._recover_refresh_weak_bucket()
        bucket_w = self._recover_bucket_weights()
        error_priority = self._recover_training_error_distribution()
        if error_priority is None:
            error_priority = np.zeros(self._rec_active_n, dtype=float)
        buckets = {}
        for bucket in range(len(self.RECOVER_FAMILIES)):
            row = self._recover_bucket_certification(bucket)
            if row is not None:
                buckets[str(bucket)] = row
        return {
            "total_buckets": len(self.RECOVER_FAMILIES),
            "max_unlocked_bucket": self._rec_active_n - 1,
            "focus_bucket": self._rec_focus_bucket,
            "weakest_bucket": (-1 if self._rec_weak_bucket is None
                                else int(self._rec_weak_bucket)),
            "buckets": buckets,
            "sample_probabilities": {
                str(bucket): float(probability)
                for bucket, probability in enumerate(bucket_w)
            },
            "training_errors": {
                str(bucket): {
                    "ema": float(self._rec_training_error_stats.get(
                        bucket, (0.0, 0))[0]),
                    "episodes": int(self._rec_training_error_stats.get(
                        bucket, (0.0, 0))[1]),
                    "priority": float(error_priority[bucket]),
                }
                for bucket in range(self._rec_active_n)
            },
        }

    def recover_curriculum_checkpoint_state(self) -> dict:
        """State paired with a policy snapshot at a proven promotion."""
        return {
            "active_n": int(self._rec_active_n),
            "focus_bucket": int(self._rec_focus_bucket),
            "stats": dict(self._rec_stats),
            "cert_rounds": dict(self._rec_cert_rounds),
        }

    def restore_recover_curriculum_checkpoint_state(self,
                                                    state: dict) -> None:
        """Restore promotion-time curriculum state, retaining error debt."""
        active_n = int(state["active_n"])
        if not 1 <= active_n <= len(self.RECOVER_FAMILIES):
            raise ValueError(f"invalid recovery active_n {active_n}")
        self._rec_active_n = active_n
        self._rec_focus_bucket = active_n - 1
        self._rec_stats = {
            str(kind): (int(values[0]), int(values[1]))
            for kind, values in dict(state["stats"]).items()
        }
        self._rec_cert_rounds = {
            str(kind): int(cert_round)
            for kind, cert_round in dict(state["cert_rounds"]).items()
        }
        self._recover_refresh_weak_bucket()

    def apply_recover_certification(self, kind: str,
                                    outcomes: list[bool],
                                    update_admission: bool = True,
                                    cert_round: int | None = None) -> dict:
        """Apply deterministic same-backend outcomes to the curriculum.

        The MJX trainer calls this on every training env after a held-out
        deterministic certification pass.  Keeping this mutation here
        makes the admission contract identical for C, MJX and sharded
        host envs while ensuring ordinary stochastic rollout terminals
        cannot move the frontier.
        """
        kind = str(kind)
        if kind not in self.RECOVER_KIND_BUCKETS:
            raise ValueError(f"unknown recover certification kind {kind!r}")
        ys = [bool(v) for v in outcomes]
        if not ys:
            raise ValueError("recover certification needs at least one outcome")
        successes = sum(1 for ok in ys if ok)
        n = len(ys)
        fraction = successes / n
        # A certification is a fixed-size held-out assay. Store exactly
        # this batch, rather than blending it into an EMA whose numerator
        # and denominator cannot be interpreted from a chart.
        self._rec_stats[kind] = (successes, n)
        if cert_round is not None:
            self._rec_cert_rounds[kind] = int(cert_round)
        before = self._rec_active_n
        focus_before = self._rec_focus_bucket
        if update_admission:
            self._recover_update_admission(cert_round)
        return {"kind": kind, "success_fraction": float(fraction),
                "successes": int(successes), "episodes": int(n),
                "active_before": int(before),
                "active_after": int(self._rec_active_n),
                "focus_before": int(focus_before),
                "focus_after": int(self._rec_focus_bucket)}

    def _sample_recover(self) -> WalkTrajectory:
        """One recover_to_plant episode: adaptive start-kind draw, zero
        commands, mode 'recover', env-side 'any' spawn branch builds
        the pose (start_kind rides the trajectory, same contract as
        getup). Hook (bank/eval only, not a cfg key):
        force_recover_start pins the kind — the weight computation and
        the draw still happen, so rng streams are identical whether or
        not the hook is armed."""
        if float(cfg_get(self.cfg, "goal", "mode_seq",
                         default=0.0)) > 0.0:
            # The mode-seq frame probes call _place_at_plant before the
            # episode placement and would consume the flip-spawn
            # pending quat; the combination is also semantically
            # meaningless (recover episodes are single-goal). Refuse
            # loudly instead of training a corrupted diet.
            raise ValueError("goal.mode_seq is incompatible with the "
                             "recover mode (frame probes vs flip "
                             "spawns); run recover as a single-mode "
                             "diet")
        if not self._rec_external_certification:
            self._recover_update_admission()
        kinds = self._recover_active_kinds()
        w = self._recover_kind_weights(kinds)
        r = float(self.rng.random())
        kind = kinds[int(np.searchsorted(np.cumsum(w), r,
                                         side="right").clip(
                                             0, len(kinds) - 1))]
        force = getattr(self, "force_recover_start", None)
        if force is not None:
            kind = str(force)
        n = self.episode_steps + 1
        zeros = np.zeros(n)
        traj = WalkTrajectory(mode="recover", roll=zeros.copy(),
                              pitch=zeros.copy(), height=zeros.copy(),
                              unload_leg=None, start_at="any",
                              vx=zeros.copy(), vy=zeros.copy(), wz=None)
        traj.start_kind = kind
        # RECOVER RSI (08-16, zero-family mechanism fix after
        # cw-recover-any8/any9 both stalled on B11): with probability
        # goal.recover_rsi_frac, an episode whose kind was NATURALLY
        # drawn from goal.recover_rsi_kinds (default "zero") spawns ON
        # the demonstrated belly->plant path instead of the family
        # pose (sim_env._reset_begin builds the waypoint — the same
        # proven goal.rise_rsi_frac lever, extended to recover). The
        # decision lives HERE, goal-side, because only the sampler
        # knows whether the kind was FORCED: force_recover_start is
        # the deterministic CERT/eval path and must stay pure, so a
        # forced episode never carries the flag. Default 0.0 = off,
        # bit-exact (no extra rng draw).
        traj.recover_rsi = False
        _rsi_f = float(cfg_get(self.cfg, "goal", "recover_rsi_frac",
                               default=0.0))
        if _rsi_f > 0.0 and force is None:
            _rsi_kinds = [k.strip() for k in str(cfg_get(
                self.cfg, "goal", "recover_rsi_kinds",
                default="zero")).split(",") if k.strip()]
            if kind in _rsi_kinds and float(self.rng.random()) < _rsi_f:
                traj.recover_rsi = True
        # RECOVER RSI, HARVESTED-BANK variant (08-16, tangle-wall
        # mechanism fix after any7/any11/any12's 3rd matching miss on
        # curriculum-weight): the ref-path mechanism above is
        # hardcoded to the belly->plant rise trajectory (a single
        # monotonic-height reference), which has no equivalent for
        # tangle's non-monotonic untangling motion. This second,
        # independent axis instead samples a spawn pose from a
        # harvested bank of ON-PATH states from a checkpoint's OWN
        # successful recoveries of the target kind
        # (harvest_recover_rsi_bank.py), so a policy stuck on a hard
        # kind practices from states partway through the motion that
        # is already known to work sometimes, not just the family's
        # raw start pose. Fully independent cfg keys/kind-list from
        # the ref-path axis above (no interaction when the target
        # kinds don't overlap); default frac 0.0 = off, bit-exact (no
        # extra rng draw). `not traj.recover_rsi` keeps the two
        # mechanisms mutually exclusive on a single episode if a kind
        # is ever listed in both.
        traj.recover_rsi_bank = False
        _rsi_bank_f = float(cfg_get(self.cfg, "goal",
                                    "recover_rsi_bank_frac", default=0.0))
        if _rsi_bank_f > 0.0 and force is None and not traj.recover_rsi:
            _rsi_bank_kinds = [k.strip() for k in str(cfg_get(
                self.cfg, "goal", "recover_rsi_bank_kinds",
                default="")).split(",") if k.strip()]
            if (kind in _rsi_bank_kinds
                    and float(self.rng.random()) < _rsi_bank_f):
                traj.recover_rsi_bank = True
        return traj

    # ---- mode sequencing (goal.mode_seq; TRANSITIONS_DIRECTIVE item 1)
    #
    # Episode = K back-to-back mode segments following the operator's
    # command grammar rise -> {hold|walk} -> {walk|lower} -> (rise ...).
    # At each switch the env installs the new segment family's CANONICAL
    # settled frame (q_nom/_z0/pad refs from sim_env._seq_capture_frames
    # — the eval_handoff/reanchor_to() semantics; see the trans-dagger2
    # fix note on sim_env._seq_maybe_switch) and regenerates the refs;
    # this side samples the plan and
    # builds each segment's schedule. goal.mode_seq=0 (default) is
    # bit-exact legacy: no plan, no extra rng draws, no per-tick work
    # beyond one attr check. Keys:
    #   goal.mode_seq                sequence-episode probability
    #                                (0 = off/legacy, 1 = every episode,
    #                                0<p<1 = mixed diet; joint_walk only)
    #   goal.mode_seq_segment_s_min  segment length draw lo (s, 6.0)
    #   goal.mode_seq_segment_s_max  segment length draw hi (s, 8.0)
    #   goal.mode_seq_blend_s_min    per-switch ref blend lo (s, 0.5)
    #   goal.mode_seq_blend_s_max    per-switch ref blend hi (s, 1.0)
    #   goal.mode_seq_max_segments   plan length cap (5)
    # First segment uses the LEGACY samplers (full start-kind diversity:
    # rise keeps its flat/bridge/crouch mix, walk its park/gait spawn
    # draws, lower its belly-start draw) so a sequence may begin at any
    # start kind, compatibly by construction. Mid-sequence spawns
    # (sequence-RSI) are deliberately NOT in v1 (pre-registered
    # follow-up lever).
    SEQ_NEXT = {"rise": ("hold", "walk"), "hold": ("walk", "lower"),
                "walk": ("lower",), "lower": ("rise",)}

    def _sample_mode_seq(self):
        gen = self._goal_gen
        rng = self.rng
        dt = self.dt
        n = self.episode_steps + 1
        seg_lo = float(cfg_get(self.cfg, "goal", "mode_seq_segment_s_min",
                               default=6.0))
        seg_hi = float(cfg_get(self.cfg, "goal", "mode_seq_segment_s_max",
                               default=8.0))
        bl_lo = float(cfg_get(self.cfg, "goal", "mode_seq_blend_s_min",
                              default=0.5))
        bl_hi = float(cfg_get(self.cfg, "goal", "mode_seq_blend_s_max",
                              default=1.0))
        max_seg = int(cfg_get(self.cfg, "goal", "mode_seq_max_segments",
                              default=5))
        # First mode: the configured goal mix restricted to the four
        # sequence modes (renormalized; uniform fallback) — --goal-mix
        # keeps steering what sequences train on.
        modes4 = ("rise", "walk", "hold", "lower")
        p = np.array([gen.p_rise, float(getattr(gen, "p_walk", 0.0)),
                      gen.p_hold, gen.p_lower], dtype=float)
        p = (p / p.sum()) if p.sum() > 0 else np.full(4, 0.25)
        mode = str(modes4[int(rng.choice(4, p=p))])
        # Segment boundaries: cumulative U(seg_lo, seg_hi) draws until
        # the tail can no longer hold a useful (>=3 s) segment; the
        # last segment runs to the episode end. Guarantee >= 2 segments
        # (a 1-segment "sequence" tests nothing) by splitting at the
        # middle if the draw left no boundary.
        min_tail = int(round(3.0 / dt))
        ticks = [0]
        t = 0.0
        while len(ticks) < max_seg:
            t += float(rng.uniform(seg_lo, seg_hi))
            tk = int(round(t / dt))
            if tk > self.episode_steps - min_tail:
                break
            ticks.append(tk)
        if len(ticks) == 1:
            ticks.append(max(1, self.episode_steps // 2))
        plan = [{"mode": mode, "tick": 0, "blend": 0}]
        for tk in ticks[1:]:
            mode = str(rng.choice(self.SEQ_NEXT[mode]))
            bn = max(1, int(round(float(rng.uniform(bl_lo, bl_hi)) / dt)))
            plan.append({"mode": mode, "tick": tk, "blend": bn})
        self._seq_plan = plan
        self._seq_idx = 0
        self._seq_seg_end = (int(plan[1]["tick"]) if len(plan) > 1
                             else int(self.episode_steps))
        # First segment through the legacy samplers (start-kind
        # diversity lives there; reset() uses its start_at).
        first = str(plan[0]["mode"])
        if first == "walk":
            return self._sample_walk()
        return gen.sample(rng, n, dt, force_mode=first)

    def _seq_segment_traj(self, mode: str, tick: int):
        """Mid-episode segment schedule on the EPISODE clock: arrays are
        full-length with [0:tick] padded (never read again) so every
        existing _step_i-indexed consumer (goal reads, ref_quiet, the
        BC lookahead, end-posture) works unchanged. Heights are
        relative to the CANONICAL family _z0 the caller installs
        BEFORE calling (settled plant for walk/hold/lower, settled
        belly for rise — sim_env._seq_capture_frames). Returns
        (traj, h_target, ramp_i0)."""
        n = self.episode_steps + 1
        m = n - tick
        if mode == "walk":
            base = self._sample_walk()
            for name in ("roll", "pitch", "height", "vx", "vy", "wz"):
                arr = getattr(base, name, None)
                if arr is None:
                    continue
                arr = np.asarray(arr, dtype=float)
                out = np.empty(n, dtype=float)
                out[:tick] = arr[0]
                out[tick:] = arr[:m]
                setattr(base, name, out)
            base.start_at = "plant"    # reset-only hint, unused here
            return base, 0.0, 0
        # rise/hold/lower segment schedules moved to the shared base
        # (goal_task.SimHexapodGoalEnv._seq_segment_traj, 08-15,
        # stance-only sequencing) — identical statements, identical rng
        # draw order, so walk-task sequence streams are unchanged.
        return super()._seq_segment_traj(mode, tick)

    def _seq_reset_mode_state(self, mode: str, ramp_i0: int,
                              h_target: float) -> None:
        super()._seq_reset_mode_state(mode, ramp_i0, h_target)
        # Fresh-segment walk bookkeeping: every accumulator _reset_begin
        # zeroes gets a fresh segment (income/charge baselines must
        # never leak across a mode switch); prev-XY/contact latches go
        # back to None/False and re-latch on the next loaded tick.
        self._liftoff_step = [0] * 6
        self._foot_prev_xy = [None] * 6
        self._foot_prev_force = [0.0] * 6
        self._foot_tan_slip_m = [0.0] * 6
        self._duty_hist = []
        self._anchor_xy = [None] * 6
        self._anchor_prev_on = [False] * 6
        self._step_disp_bank = 0.0
        self._yaw_still_ema = 0.0
        self._yaw_prog_ema = 0.0
        self._ls_prev_xy = [None] * 6
        self._ls_prev_on = [False] * 6
        self._ls_slip_m = 0.0
        self._ls_prog_m = 0.0
        # Anti-park travel-floor EMA (reward.k_walk_idle_charge);
        # per-episode/per-segment, snapshot via MJX_SNAPSHOT_EXTRA.
        self._walk_idle_ema = 0.0
        # Sustained-idle termination counter + its OWN mean-
        # |joint-velocity| EMA (safety.walk_idle_terminate_s):
        # deliberately NOT body speed (a real dragging/skating
        # gait can sit near-zero BODY speed while its legs
        # cycle hard, and cutting that short was measured to
        # rob it of the full-episode loadslip charge the bank
        # relies on; a body-speed version also flagged genuine
        # wrong-direction travel as "idle"). Mean |qvel| across
        # the 18 actuated joints cleanly separates a literally
        # FROZEN policy output (park/belly-sit/tripod-lock:
        # ~5e-5 rad/s, pure settle jitter) from every other
        # scripted behavior including skate/stall (>=0.1 rad/s,
        # ~35x higher) -- see the WALKCURR_PF_IDLE_TERM bank.
        # Seconds low, consecutively; reset to 0 the instant
        # the EMA clears the floor.
        self._walk_idle_low_s = 0.0
        self._walk_qvel_ema = 0.0
        # Seconds since the current commanded-stop segment began
        # (reward.walk_stop_grace_s); 0 whenever s_ref > 1e-3
        # (walking commanded), increments by dt each stop tick.
        # Used only to ramp the stop-speed charge in over the
        # unavoidable deceleration transient -- see the charge
        # site below. Same lifecycle as _walk_idle_ema.
        self._walk_stop_cmd_s = 0.0
        # Structural stop-hold timer (goal.walk_stop_freeze_s);
        # same lifecycle as _walk_stop_cmd_s -- see
        # sim_env._walk_stop_freeze_override.
        self._walk_stop_freeze_cmd_s = 0.0
        # Commanded-course EMA (reward.k_walk_course); same lifecycle.
        self._walk_course_ema = [0.0, 0.0]
        # Stride-EMA velocity for the tracking kernel
        # (reward.walk_kernel_vel_ema); same lifecycle.
        self._walk_kernel_vema = [0.0, 0.0]
        # Stride-EMA yaw-rate for the yaw tracking kernel
        # (reward.walk_kernel_yaw_ema); same lifecycle.
        self._walk_kernel_wz_ema = 0.0
        self._stance_slip_acc = [0.0] * 6
        self._gait_last_step = [0] * 6
        self._gait_cmd_tick = 0
        self._gait_gate_qfactor = 1.0
        self._walk_bucket = None
        self._phase = 0.0

    def _sample_goal(self):
        # goal.mode_seq semantics (08-14, Arm 2 recipe: "retain ~25%
        # single-mode episodes"): 0 = off (bit-exact legacy, no draw),
        # 1 = every episode a sequence (bit-exact prior behavior, no
        # draw), 0<p<1 = this episode is a sequence with prob p, else
        # falls through to the legacy single-mode samplers (one extra
        # rng draw ONLY in the fractional case, so both endpoints keep
        # their historical rng streams).
        p_seq = float(cfg_get(self.cfg, "goal", "mode_seq",
                              default=0.0))
        if float(cfg_get(self.cfg, "goal", "mode_seq_stance",
                         default=0.0)) > 0.0:
            # The stance-only planner (goal_task, 08-15) belongs to the
            # joint_goal task; on joint_walk the base-mode fallback
            # below would start stance sequences mid-draw — refuse
            # loudly instead of training a misconfigured diet.
            raise ValueError(
                "goal.mode_seq_stance is a joint_goal-task key; use "
                "goal.mode_seq on the joint_walk task")
        if p_seq >= 1.0 or (p_seq > 0.0 and self.rng.random() < p_seq):
            return self._sample_mode_seq()
        gen = self._goal_gen
        p_walk = float(getattr(gen, "p_walk", 0.0))
        p_getup = float(getattr(gen, "p_getup", 0.0))
        p_qw = float(getattr(gen, "p_quadwalk", 0.0))
        p_rec = float(getattr(gen, "p_recover", 0.0))
        p_base = (gen.p_hold + gen.p_lean + gen.p_track + gen.p_unload
                  + gen.p_raise + gen.p_rise + gen.p_lower
                  + getattr(gen, "p_quad", 0.0))
        tot = p_walk + p_getup + p_qw + p_rec + p_base
        if tot <= 0:
            return self._sample_walk()
        # Single draw, walk-first cdf: with p_getup == p_quadwalk ==
        # p_recover == 0 (default) the draw and its use are
        # bit-identical to the legacy two-way split, so every existing
        # lineage's rng stream is unchanged (a zero-probability mode is
        # an empty interval).
        r = self.rng.random() * tot
        if r < p_walk:
            return self._sample_walk()
        if r < p_walk + p_getup:
            return self._sample_getup()
        if r < p_walk + p_getup + p_qw:
            return self._sample_quadwalk()
        if r < p_walk + p_getup + p_qw + p_rec:
            return self._sample_recover()
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
        if self._step_i >= self._active_episode_steps():
            trunc = True
        if (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "")
                in ("walk", "quadwalk")):
            # quadwalk (08-13, quad track) shares the entire walk
            # pricing stack — kernel, progress, every income gate and
            # slip/drag charge — with exactly two lift-leg exemptions
            # below (park-duty window, step/swing credit) plus the
            # quad clear/plant income at the end. `lift` is empty in
            # walk mode, so walk pricing is bit-exact unchanged.
            mode_q = (getattr(self._goal_traj, "mode", "")
                      == "quadwalk")
            goal = self._current_goal()
            lift = (tuple(goal.lift_legs)
                    if (mode_q and goal.lift_legs) else ())
            v = self._body_vel_xy()
            err = float(np.hypot(v[0] - goal.vx_ref, v[1] - goal.vy_ref))
            # Stride-EMA kernel error (phasedir7/7b dig-in, 2026-08-22;
            # cfg reward.walk_kernel_vel_ema=1, default 0 = bit-exact
            # legacy). MEASURED defect the flag repairs: the kernel's
            # INSTANTANEOUS 2D velocity error taxes honest stride sway
            # (the accepted ~35 deg tick-level sway floor), so under
            # the phasedir7 stack a 0.059 m/s low-sway gait out-earned
            # the 0.0716 m/s clone on the kernel itself (357.1 vs
            # 325.6/ep) — the sway tax (-31/ep) cancelled k_prog's
            # linear speed payment (+34/ep), leaving income FLAT in
            # realized speed across [0.059, 0.08]. Any travel-
            # proportional charge then pins speed below the gate floor
            # at ANY dose (the pd7 step function). With the flag on,
            # the kernel error uses an EMA of the body velocity over
            # reward.walk_kernel_vel_tau_s (default 0.75 s = one
            # teacher gait period, the k_walk_course convention) so
            # zero-mean sway averages out and the kernel pays for the
            # STRIDE-AVERAGED velocity the gate actually scores.
            # Update is unconditional while the flag is on (stop
            # segments included: the EMA lags a stop by ~tau for every
            # candidate behavior equally).
            if float(cfg_get(self.cfg, "reward", "walk_kernel_vel_ema",
                             default=0.0)) > 0.0:
                tau_kv = max(float(cfg_get(self.cfg, "reward",
                                           "walk_kernel_vel_tau_s",
                                           default=0.75)), self.dt)
                a_kv = self.dt / tau_kv
                self._walk_kernel_vema[0] += a_kv * (
                    float(v[0]) - self._walk_kernel_vema[0])
                self._walk_kernel_vema[1] += a_kv * (
                    float(v[1]) - self._walk_kernel_vema[1])
                err = float(np.hypot(
                    self._walk_kernel_vema[0] - goal.vx_ref,
                    self._walk_kernel_vema[1] - goal.vy_ref))
                info["walk_kernel_vel_ema_err"] = err
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
            # Direction-first, SPEED-TARGET-FREE walk income (operator
            # order 2026-08-21, from-scratch anti-slip walking line).
            # The Gaussian kernel above and the linear progress term
            # both price a commanded SPEED; the operator's task says the
            # policy must move in the commanded DIRECTION and actually
            # travel, but does NOT have to track a speed. When
            # reward.k_walk_freeprog > 0 both are REPLACED by
            # walk_freeprog_score (see its docstring): saturating
            # along-command credit, cross-track/backward charge, no
            # band term. The positive part rides every income gate
            # below (anchor / loadslip / height / gait), the negative
            # part is added ungated at the total so a gate can never
            # shrink a penalty. Pair it with reward.walk_loadslip_gate +
            # reward.k_loadslip_excess (structural loaded-slip
            # accounting) and reward.k_walk_idle_charge (anti-park
            # travel floor); do NOT enable walk_kernel_prog_gate with
            # it — that gate re-introduces a speed target by scaling
            # income with achieved fraction of the commanded speed.
            # Default 0 = off, legacy bit-exact (no new info keys).
            # cfg: reward.k_walk_freeprog, reward.walk_freeprog_cap_m_s.
            k_free = float(cfg_get(self.cfg, "reward",
                                   "k_walk_freeprog", default=0.0))
            r_free_pen = 0.0
            if k_free > 0.0 and s_ref > 1e-3:
                # Stride-EMA input (08-22, freeprog-term400-stall dig-in
                # follow-up): a second concurrent cycle's std-anneal
                # arm (cw-amp-m2-freeprog-term400-stdanneal, FAIL)
                # root-caused the marching-in-place plateau as a
                # REWARD-SHAPE defect, not exploration noise —
                # walk_freeprog_score's INSTANTANEOUS along-command
                # velocity nets to ~0 for a symmetric back-and-forth
                # stepping gait (push-off and recovery lobes cancel in
                # the per-tick average) exactly the same way the
                # phasedir7 kernel dig-in found for the Gaussian
                # kernel term above — so it cannot distinguish "no net
                # travel" from "just starting to walk slowly" by
                # construction. Reuse of the SAME already-validated
                # fix (reward.walk_kernel_vel_ema, phasedir7/7b/8):
                # when both flags are on, freeprog scores the
                # STRIDE-AVERAGED velocity (self._walk_kernel_vema,
                # already updated unconditionally above whenever the
                # flag is set) instead of the raw instantaneous one,
                # so a true net-zero marcher still nets ~0 (unchanged)
                # but a policy that starts converting oscillation into
                # real forward drift gets a cleaner, less noise-
                # cancelled gradient toward it. Default (flag off) is
                # bit-exact legacy (raw v, unchanged).
                if float(cfg_get(self.cfg, "reward", "walk_kernel_vel_ema",
                                 default=0.0)) > 0.0:
                    fv0, fv1 = (self._walk_kernel_vema[0],
                               self._walk_kernel_vema[1])
                else:
                    fv0, fv1 = float(v[0]), float(v[1])
                f_score, f_along, f_cross = walk_freeprog_score(
                    fv0, fv1, goal.vx_ref, goal.vy_ref,
                    float(cfg_get(self.cfg, "reward",
                                  "walk_freeprog_cap_m_s",
                                  default=0.06)))
                along = f_along
                r_walk = k_free * max(f_score, 0.0)
                r_free_pen = k_free * min(f_score, 0.0)
                r_prog = 0.0
                info["walk_freeprog_score"] = f_score
                info["walk_freeprog_cross"] = f_cross
            # Simple physical joystick objective (default off). Unlike the
            # historical Gaussian/proxy stack, this is negative for parking,
            # cross-track travel, and wrong-way travel by construction.
            k_cmd_track = max(0.0, float(cfg_get(
                self.cfg, "reward", "k_walk_cmd_track", default=0.0)))
            r_cmd_track = 0.0
            cmd_cross = 0.0
            if k_cmd_track > 0.0:
                cmd_score, cmd_along, cmd_cross = walk_cmd_track_score(
                    float(v[0]), float(v[1]), goal.vx_ref, goal.vy_ref,
                    stop_speed_m_s=float(cfg_get(
                        self.cfg, "goal", "walk_speed_min_m_s",
                        default=0.03)))
                along = cmd_along
                r_cmd_track = k_cmd_track * cmd_score
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
                # Stride-EMA yaw kernel (08-23, hold/forward income-
                # dominance audit, probe_walk_income yawcmd0 stack): the
                # SAME sway-tax defect the walk_kernel_vel_ema fix
                # (phasedir7/7b/8) repaired for the linear-velocity
                # kernel exists here too, unfixed — a genuine full-stop
                # hold command pins wz near 0 with almost no variance
                # (nothing is moving), while an honest walking/turning
                # gait's body yaw-rate oscillates stride-to-stride around
                # its achieved mean even when perfectly on-command, so
                # the INSTANTANEOUS Gaussian never sits at its peak for
                # real motion the way it does for standing still.
                # Measured (probe_walk_income, ypfix1 checkpoint, full
                # 15s pinned segments): reward_walk_yaw hold=374 vs
                # forward=203 vs tip_left/right=132/147 — a real
                # richness gap on TOP of the k_yaw_prog/k_yaw_still
                # progress bonuses those segments already collect, part
                # of the measured hold/forward income dominance
                # (q_20260823T0240Z item b) that eroded turn+push
                # tracking under extra budget (tipfrac05-acq1). Fix
                # mirrors walk_kernel_vel_ema exactly in scope: ONLY the
                # kernel's err term is smoothed; the g_yaw/g_hold
                # achieved-rotation/achieved-progress gates below and
                # k_yaw_prog/k_yaw_still still use the RAW instantaneous
                # wz (those are correctness gates on real behavior, not
                # a shaping kernel, and must not be desensitized).
                # cfg reward.walk_kernel_yaw_ema > 0 turns it on; default
                # 0.0 = bit-exact legacy (raw wz, unchanged). Update is
                # unconditional while the flag is on (matches
                # walk_kernel_vel_ema's stop-segments-included
                # convention: the EMA lags any transition by ~tau for
                # every candidate behavior equally, so it cannot be
                # gamed by timing a segment boundary).
                if float(cfg_get(self.cfg, "reward", "walk_kernel_yaw_ema",
                                 default=0.0)) > 0.0:
                    tau_kw = max(float(cfg_get(
                        self.cfg, "reward", "walk_kernel_yaw_tau_s",
                        default=0.75)), self.dt)
                    a_kw = self.dt / tau_kw
                    self._walk_kernel_wz_ema += a_kw * (
                        wz - self._walk_kernel_wz_ema)
                    wz_kernel = self._walk_kernel_wz_ema
                    info["walk_kernel_wz_ema"] = wz_kernel
                else:
                    wz_kernel = wz
                yaw_err = wz - goal.wz_ref
                yaw_err_kernel = wz_kernel - goal.wz_ref
                r_yaw = k_yaw * math.exp(
                    -(yaw_err_kernel ** 2) / (2.0 * sig_w ** 2))
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
                        # OVER-SPIN FARM FIX (08-23 income audit,
                        # probe_walk_income yawcmd0 stack): the legacy
                        # clip's +1.25 headroom makes OVER-rotation
                        # strictly dominant — the gradient points to
                        # ratio 1.25, not 1.0, and the eroded acq1-r2
                        # policy measurably farmed it (yaw_progress
                        # ratio 1.78, yaw_prog income +14% over the
                        # accurate champion; also explains yawprice3x
                        # getting WORSE with 3x income). cfg
                        # reward.yaw_prog_overshoot_decay > 0 makes
                        # income PEAK at ratio 1.0 and decay linearly
                        # past it (never below 0 on the overshoot
                        # side, so gyro noise is not punished);
                        # default 0.0 = bit-exact legacy clip.
                        # DC-vs-AC (same defect class as the 08-11
                        # yaw_still fix): pricing the INSTANTANEOUS wz
                        # pays the smooth fast spinner and fines the
                        # honest gait's zero-mean stride oscillation
                        # (measured 08-23: a 0.95-ratio tracker earned
                        # NEGATIVE yaw_prog while a 2.0-ratio spinner
                        # earned +). cfg reward.yaw_prog_avg_s = EMA
                        # time constant (s) for the wz used in the
                        # ratio; default 0 = legacy instantaneous.
                        # Per-episode EMA state rides
                        # MJX_SNAPSHOT_EXTRA like _yaw_still_ema.
                        tau_p = float(cfg_get(self.cfg, "reward",
                                              "yaw_prog_avg_s",
                                              default=0.0))
                        if tau_p > 0.0:
                            a_ema = min(self.dt / tau_p, 1.0)
                            self._yaw_prog_ema += a_ema * (
                                wz_now - self._yaw_prog_ema)
                            wz_p = self._yaw_prog_ema
                            info["yaw_prog_wz_avg"] = wz_p
                        else:
                            wz_p = wz_now
                        ratio = wz_p / goal.wz_ref
                        decay = float(cfg_get(
                            self.cfg, "reward",
                            "yaw_prog_overshoot_decay", default=0.0))
                        if decay > 0.0 and ratio > 1.0:
                            val = max(1.0 - decay * (ratio - 1.0), 0.0)
                        else:
                            val = min(max(ratio, -1.5), 1.25)
                        r_yp = k_yp * val
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
                # Direct loaded-slip excess penalty (operator order
                # fb_20260820T075230_4a90c6, fast anti-skate V5): the
                # loadslip GATE can only zero income — a skating
                # policy that also collects non-velocity reward is
                # "complained about", never charged. This term charges
                # the EPISODE-ACCUMULATED loaded-slip ratio's excess
                # over loadslip_ok per second while a velocity is
                # commanded: r -= k * max(ratio - loadslip_ok, 0) * dt
                # (2026-08-20 ~08:5x realign, q_20260820T0830Z: the
                # authored desktop commit 2cb2a7b7 scales by self.dt
                # like every other per-second reward charge in this
                # file, e.g. c_time above — the controller
                # reconstruction had dropped the dt factor, which
                # would have made this term ~1/dt = 25x too strong
                # for the intended k=6.0 dose the moment any dose ever
                # reaches PPO; fixed before any training exercises it,
                # both canaries died at the B0 precert before this
                # term ever ran). Same ratio the gate and the eval
                # harness score (no touchdown resets it); a policy
                # that keeps skating keeps paying every tick, one that
                # walks clean pays nothing. Additive penalty — never
                # shrunk by income gates. Default 0 = off, bit-exact
                # legacy (no new info keys). cfg: reward.k_loadslip_excess.
                # NOT scaled by the walk-charge ramp below (walkcurr
                # bank finding, 2026-08-23,
                # test_walkcurr_chargeramp_min_ranking_holds): the
                # ramp exists to lower DISCOVERY FRICTION (park/idle/
                # heading charges that make refusing-to-move look
                # falsely safe), never the anti-skate/anti-fall floor.
                # Scaling k_loadslip_excess by the same shared
                # min_frac (0.15) made 'skate' (+130.6) and the swing-
                # farming 'shuffle' twin (+227.9) BOTH beat every
                # wrong-way/standing behavior at the ramp's minimum —
                # exactly the "high-slip/skate/fall is the floor"
                # invariant the track rule (STATUS.md, operator 08-23)
                # forbids trading away, and precisely the window
                # (early, high-exploration training) where a
                # from-scratch policy is most likely to find and lock
                # onto it. loadslip stays at its full bank-proven dose
                # at every ramp frac; only k_walk_heading/
                # k_walk_idle_charge/k_park_duty (discovery friction,
                # bank-proven safe to loosen — see the same test file)
                # ramp.
                k_lse = float(cfg_get(self.cfg, "reward",
                                      "k_loadslip_excess",
                                      default=0.0))
                # walkcurr loadslip-BOOTSTRAP (08-23, fwd4 dig-in
                # follow-up; see the __init__ block + apply_loadslip_
                # bootstrap_frac): scales this charge only, only while
                # reward.walk_loadslip_bootstrap_steps is armed; 1.0
                # (no-op, bit-exact) otherwise.
                k_lse *= self._loadslip_excess_scale()
                if k_lse > 0.0:
                    r_lse = -k_lse * max(ratio - ls_ok, 0.0) * self.dt
                    reward += r_lse
                    info["reward_loadslip_excess"] = r_lse
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
            # All-support-legs gait gate (08-13, quad track; the
            # STRUCTURAL close of the leg-sacrifice loophole after
            # cw-quadwalk1-5 measured pricing exhausted for BOTH cheat
            # families: fronts-down paid a -575/ep lift-contact charge
            # (~40% of return) and kept walking on six (quadwalk3);
            # mid-leg-park ignored a 6x k_park_duty reprice outright
            # (quadwalk5). Additive charges are payable fines, and the
            # anchor gate's fraction spans LOADED feet only — a leg
            # parked in the AIR silently drops out of its denominator.
            # Same lesson as the prog/anchor/loadslip gates: make the
            # cheat worth less BY CONSTRUCTION. Velocity income
            # (kernel + positive progress; quadwalk's clear/plant
            # income in _quad_income rides the same factor) is
            # multiplied by the MIN over commanded SUPPORT legs of a
            # per-leg "recently completed a real swing" score: 1.0 if
            # the leg finished a liftoff -> >=2-ticks-airborne ->
            # touchdown swing with XY stride >= gait_gate_stride_mm
            # within the trailing gait_gate_window_s of COMMANDED
            # ticks, fading linearly to 0 over gait_gate_fade_s after
            # that (a fade, not a hard zero — the holdstill1
            # zero-gradient lesson). MIN, not mean: quadwalk3/5
            # measured that any fractional discount is simply paid;
            # sacrificing ANY subset of support legs must collapse
            # transport income to the (1-g) floor. Episode start
            # counts as "just stepped" (window+fade of commanded
            # grace); lift legs are exempt (they must NOT step);
            # penalties are never shrunk. Default 0 = off, legacy
            # bit-exact. cfg: reward.walk_gait_gate in [0,1],
            # reward.gait_gate_window_s (2.0), gait_gate_fade_s (2.0),
            # gait_gate_stride_mm (10.0).
            g_gait = float(cfg_get(self.cfg, "reward",
                                   "walk_gait_gate", default=0.0))
            self._gait_gate_qfactor = 1.0
            if g_gait > 0.0 and s_ref > 1e-3:
                self._gait_cmd_tick += 1
                n_gwin = max(1, int(round(float(cfg_get(
                    self.cfg, "reward", "gait_gate_window_s",
                    default=2.0)) / self.dt)))
                n_gfade = int(round(float(cfg_get(
                    self.cfg, "reward", "gait_gate_fade_s",
                    default=2.0)) / self.dt))
                g_score = 1.0
                for f in range(6):
                    if f in lift:
                        continue
                    since = self._gait_cmd_tick - self._gait_last_step[f]
                    if since <= n_gwin:
                        sc = 1.0
                    elif n_gfade > 0:
                        sc = max(1.0 - (since - n_gwin) / n_gfade, 0.0)
                    else:
                        sc = 0.0
                    g_score = min(g_score, sc)
                gt_factor = (1.0 - g_gait) + g_gait * g_score
                r_walk *= gt_factor
                if r_prog > 0.0:
                    r_prog *= gt_factor
                if r_cmd_track > 0.0:
                    r_cmd_track *= gt_factor
                self._gait_gate_qfactor = gt_factor
                info["walk_gait_min"] = g_score
                info["walk_gait_gate_factor"] = gt_factor
            reward = float(reward) + r_walk + r_prog + r_cmd_track \
                + r_free_pen
            if r_free_pen != 0.0:
                info["reward_walk_freeprog_pen"] = r_free_pen
            info["reward_walk"] = r_walk
            info["reward_walk_prog"] = r_prog
            if k_cmd_track > 0.0:
                info["reward_walk_cmd_track"] = r_cmd_track
            info["walk_vel_err"] = err
            info["walk_speed"] = float(np.hypot(*v))
            # Fast-profile command-tracking charges (08-20, operator
            # note fb_20260820T000059 item 3b — the steer5-fastprof1
            # canary: under the raised servo profile every checkpoint
            # runs 0.13-0.16 m/s against a 0.05-0.06 command and drifts
            # 50-60 deg off heading; the Gaussian kernel saturates ~2
            # sigma out and the progress cap 1.25 still PAYS overspeed,
            # so nothing prices the exceedance itself). Two opt-in
            # CHARGES, walk/quadwalk block only, commanded-motion ticks
            # only (s_ref > 1e-3 — stop/settle segments are priced by
            # the stop->stance-hold contract, never charged here), added
            # AFTER the income gates so penalties are never shrunk
            # (gait-gate rule). Both default 0 = bit-exact legacy.
            #  - reward.k_walk_overspeed: -k * min(over/s_ref, 3) where
            #    over = max(0, |v| - (1+walk_overspeed_tol)*s_ref) —
            #    fractional exceedance of the commanded band (tol
            #    default 0.10 ~ the eval prog_ratio band), linear so the
            #    gradient never saturates, capped at 3x for bounded
            #    per-tick cost. At the canary's operating point
            #    (0.14 m/s vs 0.06 cmd) k=2 charges ~2.4/tick — real
            #    money vs the ~2/tick kernel income.
            #  - reward.k_walk_heading: -k * (1 - cos(heading err)) on
            #    ticks actually moving (|v| >=
            #    reward.walk_heading_min_speed_m_s, default 0.01 —
            #    heading is undefined near zero speed; parking is
            #    priced by the prog gates, not here). Smooth, bounded
            #    [0, 2k]; the canary's ~55 deg drift costs ~0.43k/tick,
            #    perfect heading costs 0.
            k_over = float(cfg_get(self.cfg, "reward",
                                   "k_walk_overspeed", default=0.0))
            k_head = float(cfg_get(self.cfg, "reward",
                                   "k_walk_heading", default=0.0)) \
                * self._walk_charge_scale()
            if (k_over > 0.0 or k_head > 0.0) and s_ref > 1e-3:
                spd_now = float(np.hypot(v[0], v[1]))
                if k_over > 0.0:
                    tol = float(cfg_get(self.cfg, "reward",
                                        "walk_overspeed_tol",
                                        default=0.10))
                    over = max(0.0, spd_now - (1.0 + tol) * s_ref)
                    info["walk_overspeed_m_s"] = over
                    if over > 0.0:
                        r_over = -k_over * min(over / s_ref, 3.0)
                        reward = float(reward) + r_over
                        info["reward_walk_overspeed"] = r_over
                if k_head > 0.0:
                    v_min = float(cfg_get(
                        self.cfg, "reward",
                        "walk_heading_min_speed_m_s", default=0.01))
                    if spd_now >= v_min:
                        cos_h = max(-1.0, min(1.0, float(
                            v[0] * goal.vx_ref + v[1] * goal.vy_ref)
                            / (spd_now * s_ref)))
                        r_head = -k_head * (1.0 - cos_h)
                        reward = float(reward) + r_head
                        info["reward_walk_heading"] = r_head
                        info["walk_heading_cos"] = cos_h
            # Anti-park TRAVEL FLOOR charge (operator order 2026-08-21,
            # from-scratch anti-slip walking): every prior from-scratch
            # gait arm collapsed into freeze or march-in-place, and a
            # slip-per-metre objective is meaningless at ~zero travel.
            # This charges, per second of COMMANDED motion, the
            # shortfall of the body's smoothed along-command speed below
            # reward.walk_idle_speed_m_s: r -= k * clip((floor - ema)/
            # floor, 0, 1) * dt. A parked or marching-in-place body pays
            # the full k per second; a body creeping at half the floor
            # pays half; anything travelling at or above the floor pays
            # nothing, and there is no upper bound to hit (this is a
            # FLOOR, not a speed band). The EMA (reward.walk_idle_tau_s,
            # default 1 s) is what makes it smooth and gradient-bearing:
            # a real gait's per-tick along speed oscillates through zero
            # every stride and must not be charged for that. Added AFTER
            # the income gates so no gate can shrink it. Default 0 =
            # off, legacy bit-exact (no new info keys, no state use).
            # cfg: reward.k_walk_idle_charge, reward.walk_idle_speed_m_s,
            # reward.walk_idle_tau_s.
            k_idle = float(cfg_get(self.cfg, "reward",
                                   "k_walk_idle_charge", default=0.0)) \
                * self._walk_charge_scale()
            if k_idle > 0.0 and s_ref > 1e-3:
                tau = max(float(cfg_get(self.cfg, "reward",
                                        "walk_idle_tau_s", default=1.0)),
                          self.dt)
                self._walk_idle_ema += (self.dt / tau) * (
                    float(along) - self._walk_idle_ema)
                floor_v = max(float(cfg_get(
                    self.cfg, "reward", "walk_idle_speed_m_s",
                    default=0.03)), 1e-6)
                shortfall = min(max(
                    (floor_v - self._walk_idle_ema) / floor_v, 0.0), 1.0)
                r_idle = -k_idle * shortfall * self.dt
                reward = float(reward) + r_idle
                info["walk_along_ema_m_s"] = self._walk_idle_ema
                info["walk_idle_shortfall"] = shortfall
                info["reward_walk_idle"] = r_idle
            # Commanded-STOP speed charge (2026-08-24, joyfullcurr6
            # dig-in): during commanded-stop segments NOTHING in this
            # stack prices residual body speed except the shallow
            # Gaussian kernel (stillness 2.0/tick vs a 0.04 m/s creep's
            # 1.45/tick, +0.55/tick margin) — every other walk term
            # (prog gate, course, park-duty, step/drag, idle) is guarded
            # by s_ref > 1e-3 and pays/charges NOTHING on stop ticks.
            # The V6 ladder's cert bar (mean stop-tick speed <= 0.015
            # m/s) therefore had no matching reward optimum, and the
            # 40M joyfullcurr6 run converged to a ~0.04 m/s creep that
            # failed the b1 stop cert ~79 rounds in a row while reward
            # sat converged. This charges, on stop ticks only (s_ref ~
            # 0 and NOT a commanded turn-in-place — that motion is the
            # command), the instantaneous body speed against
            # reward.walk_stop_scale_m_s (default = the 0.015 cert
            # bar): r -= k * min(speed/scale, cap). Stillness pays 0,
            # the observed creep pays ~2.7k/tick, walking through a
            # stop pays the cap — so true stillness is the optimum by
            # construction, matching the cert exactly. Added AFTER the
            # income gates so no gate can shrink it. NOT part of the
            # walk_charge_ramp trio (that ramp loosens charges that
            # make refusal falsely cheap; this one prices obedience to
            # an explicit stop command and must never loosen). Default
            # 0 = off, legacy bit-exact (no new info keys).
            # cfg: reward.k_walk_stop_charge, reward.walk_stop_scale_m_s,
            # reward.walk_stop_charge_cap, reward.walk_stop_grace_s.
            k_stopc = float(cfg_get(self.cfg, "reward",
                                    "k_walk_stop_charge", default=0.0))
            # Read the stop-CURRENT gain here too: both stop charges
            # share the _walk_stop_cmd_s grace timer, so the timer must
            # tick whenever EITHER is armed (k_stopcur=0 default keeps
            # the guard identical to the pre-current-charge code).
            k_stopcur = float(cfg_get(self.cfg, "reward",
                                      "k_walk_stop_current", default=0.0))
            if k_stopc > 0.0 or k_stopcur > 0.0:
                # Elapsed time since the current stop segment began
                # (reset the instant translation is commanded again).
                # Tracked whenever the charge is active at all so the
                # ramp below is correct from the very first stop tick,
                # independent of the turn-in-place exemption.
                if s_ref > 1e-3:
                    self._walk_stop_cmd_s = 0.0
                else:
                    self._walk_stop_cmd_s += self.dt
            if (k_stopc > 0.0 and s_ref <= 1e-3
                    and not (self._yaw_cmd
                             and abs(float(getattr(goal, "wz_ref", 0.0)
                                           or 0.0)) > 1e-3)):
                scale_sc = max(float(cfg_get(
                    self.cfg, "reward", "walk_stop_scale_m_s",
                    default=0.015)), 1e-6)
                cap_sc = float(cfg_get(self.cfg, "reward",
                                       "walk_stop_charge_cap",
                                       default=4.0))
                # Settle-grace (2026-08-24, joyfullcurr7 dig-in): the
                # plain charge above priced the UNAVOIDABLE physical
                # deceleration transient right after a stop command as
                # harshly as sustained creep, and joyfullcurr7 (k=1.0,
                # no grace) measured the consequence directly — every
                # fall in its held-out joygate session was an
                # over_current safety trip, not roll/tilt: the policy
                # was braking hard enough to spike actuator current.
                # grace_s > 0 linearly ramps the charge's MULTIPLIER
                # from 0 at the instant a stop begins to 1.0 at
                # grace_s seconds in (then holds at 1.0) so the
                # transient pays little/nothing while SUSTAINED creep
                # past the grace window still pays the full charge —
                # the actual cert-bar violation. Default 0 = off,
                # bit-exact (grace_mult stays exactly 1.0, identical
                # to the pre-grace formula).
                grace_s = float(cfg_get(self.cfg, "reward",
                                        "walk_stop_grace_s", default=0.0))
                grace_mult = 1.0
                if grace_s > 1e-6:
                    grace_mult = min(max(
                        self._walk_stop_cmd_s / grace_s, 0.0), 1.0)
                sp_sc = float(np.hypot(float(v[0]), float(v[1])))
                r_stopc = -k_stopc * grace_mult * min(sp_sc / scale_sc,
                                                       cap_sc)
                reward = float(reward) + r_stopc
                info["walk_stop_speed_m_s"] = sp_sc
                info["reward_walk_stop"] = r_stopc
                info["walk_stop_grace_mult"] = grace_mult
            # Commanded-STOP actuator-current charge (2026-08-24,
            # joyfullcurr8 dig-in): joyfullcurr7 (stop-speed charge,
            # no grace) and joyfullcurr8 (+0.4s grace ramp) BOTH ended
            # with 100% of held-out joygate falls = over_current; the
            # grace ramp moved slip/dir_err but over_current not at
            # all. Root cause: the SafetyLayer trips on servo current
            # > 2.5 A SUSTAINED for 0.8 s through a ~0.1 s low-pass
            # (safety.py / sim_env._read_state) -- that is not a
            # braking transient, it is a sustained isometric fight
            # (stiff position targets pressing against contact) held
            # through the stop stance. NOTHING in the stack prices
            # that fight on stop ticks (k_current_hot is global and
            # OFF in this lineage), so speed-based stop pricing pushes
            # the policy INTO it: braking/freezing hard is the
            # cheapest way to zero the speed charge. This charges, on
            # stop ticks only (same s_ref/turn-in-place scoping as the
            # speed charge above), per-servo current quadratically
            # above a headroom threshold (default 1.5 A = 1.0 A under
            # the trip): r -= k * grace_mult * min(sum(max(|I|-thr,0)^2),
            # cap). A settled deadband stance draws ~0 A (sim_env
            # firmware dead-zone), so RELAXED stillness pays nothing
            # -- the optimum is stop-without-fighting, exactly what
            # the joygate demands. Level charge, not current-rate: the
            # trip fires on sustained level, and the 0.1 s LPF already
            # erases spikes a rate term would price. Shares
            # walk_stop_grace_s (same timer) so the unavoidable
            # braking current of the first grace window pays little.
            # Added AFTER the income gates (gait-gate rule). Default
            # 0 = off, legacy bit-exact (no new info keys).
            # cfg: reward.k_walk_stop_current, reward.walk_stop_current_a,
            # reward.walk_stop_current_cap (+ shared walk_stop_grace_s).
            if (k_stopcur > 0.0 and s_ref <= 1e-3
                    and not (self._yaw_cmd
                             and abs(float(getattr(goal, "wz_ref", 0.0)
                                           or 0.0)) > 1e-3)):
                cur_sc = getattr(self._state, "servo_current", None)
                if cur_sc is not None:
                    thr_cur = float(cfg_get(self.cfg, "reward",
                                            "walk_stop_current_a",
                                            default=1.5))
                    cap_cur = float(cfg_get(self.cfg, "reward",
                                            "walk_stop_current_cap",
                                            default=4.0))
                    grace_s_cur = float(cfg_get(self.cfg, "reward",
                                                "walk_stop_grace_s",
                                                default=0.0))
                    gm_cur = 1.0
                    if grace_s_cur > 1e-6:
                        gm_cur = min(max(
                            self._walk_stop_cmd_s / grace_s_cur, 0.0), 1.0)
                    over_cur = np.maximum(
                        np.abs(np.asarray(cur_sc, dtype=float)) - thr_cur,
                        0.0)
                    r_stopcur = -k_stopcur * gm_cur * min(
                        float(np.sum(over_cur ** 2)), cap_cur)
                    reward = float(reward) + r_stopcur
                    info["walk_stop_current_max_a"] = float(
                        np.max(np.abs(cur_sc)))
                    info["reward_walk_stop_current"] = r_stopcur
            # Commanded-COURSE charge (operator reward-alignment order
            # 2026-08-22, fb_20260822T032514 item 3 — the phasedir1
            # fix): price wrong-way / off-heading TRAVEL on the
            # STRIDE-AVERAGED velocity, never the per-tick one.
            # phasedir1's PPO drifted to a generic stable fast walk
            # (dir_err med 35.6 -> 67.3 deg, rear headings collapsed)
            # because no active term priced course; the existing
            # per-tick k_walk_heading is the wrong tool because a
            # clean tripod stride sways the instantaneous velocity
            # ~35 deg (CURRENT_TRUTHS metric fact) and would charge
            # honest gait mechanics. This EMAs the body-frame planar
            # velocity over reward.walk_course_tau_s (default 0.75 s =
            # one teacher gait period, so stride sway averages out
            # over exactly one cycle) and charges
            # -k * (1 - cos(course err)) per tick once the smoothed
            # speed clears reward.walk_course_min_speed_m_s (course is
            # undefined near zero travel; parking/refusal is priced by
            # k_walk_idle_charge + the prog gates, not here).
            # Commanded-motion ticks only; added AFTER the income
            # gates so no gate can shrink it (gait-gate rule). Default
            # 0 = off: no state update, no info keys, legacy bit-exact.
            # cfg: reward.k_walk_course, reward.walk_course_tau_s,
            # reward.walk_course_min_speed_m_s.
            k_course = float(cfg_get(self.cfg, "reward",
                                     "k_walk_course", default=0.0))
            if k_course > 0.0 and s_ref > 1e-3:
                tau_c = max(float(cfg_get(self.cfg, "reward",
                                          "walk_course_tau_s",
                                          default=0.75)), self.dt)
                a_c = self.dt / tau_c
                self._walk_course_ema[0] += a_c * (
                    float(v[0]) - self._walk_course_ema[0])
                self._walk_course_ema[1] += a_c * (
                    float(v[1]) - self._walk_course_ema[1])
                ex, ey = self._walk_course_ema
                spd_c = math.hypot(ex, ey)
                v_min_c = float(cfg_get(self.cfg, "reward",
                                        "walk_course_min_speed_m_s",
                                        default=0.01))
                if spd_c >= v_min_c:
                    cos_c = max(-1.0, min(1.0, (
                        ex * goal.vx_ref + ey * goal.vy_ref)
                        / (spd_c * s_ref)))
                    r_course = -k_course * (1.0 - cos_c)
                    reward = float(reward) + r_course
                    info["walk_course_cos"] = cos_c
                    info["reward_walk_course"] = r_course
                # EMA-speed overspeed band charge (same order, same
                # preflight: the INSTANT-speed k_walk_overspeed cannot
                # separate the 1.3-1.7x attractor from the honest gait
                # — a clean tripod's instantaneous speed pulses through
                # the band every stride, so a tight instant band taxes
                # obedience while a loose one lets the measured 0.139
                # attractor keep out-earning obey (preflight bank
                # 08-22: overspeed 504 vs obey 430 under
                # k_walk_overspeed=2/tol=0.10). Charging the STRIDE-
                # AVERAGED |v| prices sustained overspeed only:
                # -k * min(over/s_ref, 3), over = max(0, |v_ema| -
                # (1+tol)*s_ref). Default 0 = off, bit-exact.
                # cfg: reward.k_walk_course_overspeed,
                # reward.walk_course_overspeed_tol.
                k_cover = float(cfg_get(self.cfg, "reward",
                                        "k_walk_course_overspeed",
                                        default=0.0))
                if k_cover > 0.0:
                    tol_co = float(cfg_get(
                        self.cfg, "reward",
                        "walk_course_overspeed_tol", default=0.05))
                    # Along-command projection variant (2026-08-22
                    # phasedir3 reprice): |v_ema| is biased UPWARD by
                    # zero-mean exploration sway (norm of a noisy
                    # vector), so at PPO std=0.36 the band charge fired
                    # flat (~-3/tick) on a clone whose ALONG-command
                    # travel was under the band. When
                    # reward.walk_course_overspeed_along=1 the
                    # exceedance uses the unbiased along-command
                    # projection of the same course EMA instead —
                    # sustained directed overspeed (the 0.139 m/s
                    # attractor) still pays in full. Default 0 = off,
                    # |v_ema| pricing, bit-exact legacy.
                    spd_over = spd_c
                    if float(cfg_get(self.cfg, "reward",
                                     "walk_course_overspeed_along",
                                     default=0.0)) == 1.0:
                        spd_over = (ex * goal.vx_ref
                                    + ey * goal.vy_ref) / s_ref
                    # Reference floor (2026-08-22 phasedir8 dig-in):
                    # during a command RAMP s_ref sweeps up from ~0,
                    # so BOTH the band threshold (1+tol)*s_ref and the
                    # fractional-exceedance denominator shrink toward
                    # zero — millimetre-per-second EMA drift on early
                    # ramp ticks paid the full -3k clip (measured on
                    # cw-dep-bcgait4-phasedir8: W&B mean charge
                    # -2.38/charged-tick against mean exceedance
                    # 0.002 m/s, only possible at s_ref of a few mm/s;
                    # the charge taught "lag the ramp", capping mean
                    # speed). Flooring the reference at
                    # reward.walk_course_overspeed_ref_floor_m_s prices
                    # overspeed against at least the floor speed:
                    # ramp drift under (1+tol)*floor pays nothing;
                    # sustained overspeed at the full command (s_ref >=
                    # floor) is bit-identical. Default 0 = legacy
                    # exact (s_den == s_ref).
                    s_den = max(s_ref, float(cfg_get(
                        self.cfg, "reward",
                        "walk_course_overspeed_ref_floor_m_s",
                        default=0.0)))
                    over_c = max(0.0, spd_over - (1.0 + tol_co) * s_den)
                    if over_c > 0.0:
                        r_cover = -k_cover * min(over_c / s_den, 3.0)
                        reward = float(reward) + r_cover
                        info["walk_course_overspeed_m_s"] = over_c
                        info["reward_walk_course_overspeed"] = r_cover
            _add_walk_direction_info(
                info, float(v[0]), float(v[1]),
                float(goal.vx_ref), float(goal.vy_ref),
                min_speed_m_s=float(cfg_get(
                    self.cfg, "goal", "walk_direction_min_speed_m_s",
                    default=WALK_DIRECTION_MIN_SPEED_M_S)))
            # Raw commanded-direction speed telemetry (08-15, operator
            # directive fb_20260815T114414: judge command-following by
            # RAW SIGNED m/s along the requested direction, never by
            # clipped gate factors or total speed; SIMPLIFIED by
            # fb_20260815T115650: NO per-heading bin keys in training —
            # uniform [-pi,pi] heading sampling + the signed average
            # already zeroes out command-ignorant motion, and fixed
            # 8/12-direction panels belong in held-out EVAL only). cfg
            # goal.walk_cmd_metrics=1; default 0 = no new info keys,
            # legacy info dict bit-exact. Emitted ONLY on
            # active-command ticks (s_ref > 1e-3), so the trainers'
            # info-scalar means are per-ACTIVE-tick by construction.
            if (s_ref > 1e-3 and float(cfg_get(
                    self.cfg, "goal", "walk_cmd_metrics",
                    default=0.0)) == 1.0):
                ux, uy = goal.vx_ref / s_ref, goal.vy_ref / s_ref
                info["v_along_cmd_m_s"] = float(along)
                info["v_cross_abs_m_s"] = (
                    cmd_cross if k_cmd_track > 0.0 else
                    abs(float(ux * v[1] - uy * v[0])))
                info["cmd_speed_m_s"] = s_ref
                info["wrong_way"] = 1.0 if along < 0.0 else 0.0
                info["walk_cmd_mode_id"] = WALK_CMD_MODE_IDS.get(
                    getattr(self._goal_traj, "cmd_mode", "legacy"), 0)
                # Walk-direction telemetry (operator directive
                # fb_20260815T192912): angular error in DEGREES between
                # the achieved planar velocity and the commanded
                # direction. Direction is undefined near zero speed, so
                # ticks are VALID only when actual speed >= 5 mm/s;
                # walk_dir_valid is emitted on every active tick (its
                # mean = valid fraction) while the deg key is emitted
                # only on valid ticks (its mean = error over valid
                # ticks). Same walk_cmd_metrics gate as above: the
                # default (0) keeps the legacy info dict bit-exact.
                spd = float(np.hypot(*v))
                dir_valid = spd >= 5e-3
                info["walk_dir_valid"] = 1.0 if dir_valid else 0.0
                if dir_valid:
                    cosang = max(-1.0, min(1.0, float(along) / spd))
                    info["walk_direction_err_deg"] = math.degrees(
                        math.acos(cosang))
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
                                   default=0.0)) \
                * self._walk_charge_scale()
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
            # Ramp override (see __init__/apply_drag_allow_frac): None
            # unless reward.drag_stance_allow_ramp_steps is armed AND
            # the trainer has broadcast at least one frac — bit-exact
            # legacy cfg lookup otherwise.
            if self._drag_allow_override_m is not None:
                allow_m = self._drag_allow_override_m
            else:
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
            # gait_gate_stride_mm: min completed-swing XY stride the
            # walk_gait_gate above counts as "this leg is cycling"
            # (below the step credit's 10 mm ALONG-command bar on
            # purpose — the gate scores gait legality, not progress;
            # direction is priced by the kernel/progress terms).
            gait_stride_m = float(cfg_get(self.cfg, "reward",
                                          "gait_gate_stride_mm",
                                          default=10.0)) / 1000.0
            # Tangential planted-foot slip charge and contact diagnostics
            # (2026-08-23 Stage-A probe). This is deliberately lighter
            # than the episode-level loadslip ratio above: it only
            # charges foot XY velocity while the same foot has meaningful
            # contact on consecutive ticks, averages across measured
            # feet instead of summing all six, applies a deadband/cap,
            # and is default-off. cfg: reward.k_foot_slip_tangent,
            # reward.foot_slip_contact_n,
            # reward.foot_slip_deadband_m_s,
            # reward.foot_slip_max_m_s; goal.walk_contact_diagnostics.
            k_tslip = float(cfg_get(self.cfg, "reward",
                                    "k_foot_slip_tangent", default=0.0))
            contact_diag = bool(float(cfg_get(
                self.cfg, "goal", "walk_contact_diagnostics",
                default=0.0)) > 0.0)
            tslip_contact_n = float(cfg_get(
                self.cfg, "reward", "foot_slip_contact_n", default=1.0))
            tslip_deadband = float(cfg_get(
                self.cfg, "reward", "foot_slip_deadband_m_s",
                default=0.015))
            tslip_cap = float(cfg_get(
                self.cfg, "reward", "foot_slip_max_m_s", default=0.25))
            if (k_swing > 0.0 or k_step > 0.0 or k_drag > 0.0
                    or k_park > 0.0 or k_ds > 0.0
                    or g_gait > 0.0 or k_tslip > 0.0
                    or contact_diag) and s_ref > 1e-3:
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
                r_tslip = 0.0
                contacts = [False] * 6
                contact_forces = [0.0] * 6
                meaningful_contacts = 0
                touchdown_flags = [False] * 6
                liftoff_flags = [False] * 6
                swinging_flags = [False] * 6
                air_times_s = [0.0] * 6
                tangent_vels = [0.0] * 6
                measured_tangent_vels = []
                tangent_excess = []
                for f in range(6):
                    adr = self._touch_adr[f]
                    force = (max(0.0, float(self.data.sensordata[adr]))
                             if adr >= 0 else 0.0)
                    contact_forces[f] = force
                    on = force > 0.5
                    contacts[f] = on
                    meaningful = on and force >= tslip_contact_n
                    if meaningful:
                        meaningful_contacts += 1
                    xy = self.data.xpos[self._pad_bids[f], :2]
                    if on and not self._foot_on[f]:
                        touchdown_flags[f] = True
                        # Touchdown: a new stance period earns a fresh
                        # slip allowance (k_drag_stance bookkeeping).
                        self._stance_slip_acc[f] = 0.0
                    if self._foot_on[f] and not on:
                        liftoff_flags[f] = True
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
                        # Lift-leg exemption (quadwalk): a commanded-
                        # lifted front never earns swing/step credit —
                        # stepping with the "hands" is the six-leg
                        # cheat, not the task. Charges below still
                        # apply to it (a dragging front pays).
                        # walk_gait_gate bookkeeping: a completed real
                        # swing marks this leg "cycling" on the
                        # commanded-tick clock (lift legs excluded —
                        # their stepping is the six-leg cheat).
                        if g_gait > 0.0 and air >= 2 \
                                and stride >= gait_stride_m \
                                and f not in lift:
                            self._gait_last_step[f] = self._gait_cmd_tick
                        if k_swing > 0.0 and stride >= 0.015 \
                                and air >= 2 and f not in lift:
                            r_swing += k_swing
                        if k_step > 0.0 and air >= 2 and f not in lift:
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
                            and self._foot_prev_xy[f] is not None:
                        slip = float(np.linalg.norm(
                            xy - self._foot_prev_xy[f]))
                        prev_meaningful = (
                            self._foot_prev_force[f] >= tslip_contact_n)
                        if meaningful and prev_meaningful:
                            tv = slip / max(self.dt, 1e-9)
                            tangent_vels[f] = tv
                            measured_tangent_vels.append(tv)
                            self._foot_tan_slip_m[f] += slip
                            ex = max(tv - tslip_deadband, 0.0)
                            if tslip_cap > 0.0:
                                ex = min(ex, tslip_cap)
                            tangent_excess.append(ex)
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
                    if not on and self._liftoff_xy[f] is not None:
                        swinging_flags[f] = True
                        air_times_s[f] = (self._step_i
                                          - self._liftoff_step[f]) * self.dt
                    self._foot_prev_xy[f] = xy.copy()
                    self._foot_prev_force[f] = force
                    self._foot_on[f] = on
                if k_tslip > 0.0:
                    if tangent_excess:
                        r_tslip = -k_tslip * float(np.mean(tangent_excess))
                    reward += r_tslip
                    info["reward_foot_slip_tangent"] = r_tslip
                if k_tslip > 0.0 or contact_diag:
                    info["walk_contact_feet"] = float(sum(contacts))
                    info["walk_contact_meaningful_feet"] = float(
                        meaningful_contacts)
                    if measured_tangent_vels:
                        info["walk_tangent_contact_vel_mean_m_s"] = float(
                            np.mean(measured_tangent_vels))
                        info["walk_tangent_contact_vel_max_m_s"] = float(
                            max(measured_tangent_vels))
                    else:
                        info["walk_tangent_contact_vel_mean_m_s"] = 0.0
                        info["walk_tangent_contact_vel_max_m_s"] = 0.0
                    info["walk_touchdown_count"] = float(
                        sum(touchdown_flags))
                    info["walk_liftoff_count"] = float(sum(liftoff_flags))
                    info["walk_swinging_feet"] = float(sum(swinging_flags))
                if contact_diag:
                    for f in range(6):
                        info[f"walk_foot{f}_contact"] = (
                            1.0 if contacts[f] else 0.0)
                        info[f"walk_foot{f}_meaningful_contact"] = (
                            1.0 if contact_forces[f] >= tslip_contact_n
                            and contacts[f] else 0.0)
                        info[f"walk_foot{f}_contact_force"] = (
                            contact_forces[f])
                        info[f"walk_foot{f}_tangent_vel_m_s"] = (
                            tangent_vels[f])
                        info[f"walk_foot{f}_tangent_slip_m_total"] = (
                            self._foot_tan_slip_m[f])
                        info[f"walk_foot{f}_swinging"] = (
                            1.0 if swinging_flags[f] else 0.0)
                        info[f"walk_foot{f}_touchdown"] = (
                            1.0 if touchdown_flags[f] else 0.0)
                        info[f"walk_foot{f}_liftoff"] = (
                            1.0 if liftoff_flags[f] else 0.0)
                        info[f"walk_foot{f}_air_time_s"] = air_times_s[f]
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
                        # Lift-leg exemption (quadwalk): the window
                        # spans only the support legs — a permanently
                        # lifted front is the COMMAND, not a park
                        # (audit 08-13: all six spanned meant an
                        # honest quad stance paid ~0.2k every tick).
                        # `lift` empty (walk mode) = original array,
                        # bit-exact.
                        if lift:
                            duty = duty[[f for f in range(6)
                                         if f not in lift]]
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
            # Hip-yaw limit-margin charge (2026-08-19; operator order
            # fb_20260818T152717 lineage — the direction-switch tangle).
            # probe_dirswitch_tangle measured the tangle PRECURSOR:
            # after abrupt command switches the walker rides hip-yaw
            # joints within ~2 deg of (and into) the hard stop for
            # 1-9% of ticks (yaw_sat_frac 0.013-0.093, margin min to
            # -0.65 deg), while rot60 sector crossings were exonerated.
            # Three exposure/schedule/blend levers (steer1-hard20m1,
            # steer2-hard20m1-r1, steer2-blend1) moved the symptom
            # without curing it, so this prices the precursor
            # directly: per tick, each leg whose hip-yaw sits within
            # reward.yaw_margin_allow_deg of either hard limit pays
            # k_yaw_margin scaled linearly by depth into the band
            # (margin >= allow -> 0, margin <= 0 [pressed into the
            # stop] -> full k). WALK-mode block only; charged on every
            # walk tick regardless of the commanded speed (saturation
            # during a stop dwell is the same tangle precursor). The
            # honest tall gait rides ~10-20+ deg of margin and pays
            # ~0 by construction (semantics bank). Reads only
            # data.qpos + model constants, so C env and MJX FakeData
            # backends price identically. Default 0.0 = off, block
            # skipped, bit-exact legacy.
            k_yawm = float(cfg_get(self.cfg, "reward", "k_yaw_margin",
                                   default=0.0))
            if k_yawm > 0.0:
                jr = getattr(self, "_yaw_margin_jrng", None)
                if jr is None:
                    jr = []
                    for leg in range(6):
                        j = self.model.joint(f"L{leg}_yaw")
                        jr.append((int(np.asarray(j.qposadr).item()),
                                   float(np.degrees(j.range[0])),
                                   float(np.degrees(j.range[1]))))
                    self._yaw_margin_jrng = jr
                allow_deg = float(cfg_get(self.cfg, "reward",
                                          "yaw_margin_allow_deg",
                                          default=3.0))
                r_yawm = 0.0
                for adr, lo, hi in jr:
                    q = float(np.degrees(self.data.qpos[adr]))
                    margin = min(q - lo, hi - q)
                    if margin < allow_deg:
                        depth = 1.0 - max(margin, 0.0) / allow_deg
                        r_yawm -= k_yawm * depth
                if r_yawm:
                    reward += r_yawm
                info["reward_yaw_margin"] = r_yawm
            if mode_q:
                # quadwalk: the lifted-fronts income (clear/plant,
                # same cfg keys and grace as quad hold) rides on top
                # of the walk stack — fronts-down forfeits it by
                # construction, which is what makes the six-leg and
                # drag cheats under-earn (QUADWALK semantics bank).
                reward, info = self._quad_income(float(reward), info)
        elif (self._goal_traj is not None
                and getattr(self._goal_traj, "mode", "") == "quad"):
            reward, info = self._quad_income(float(reward), info)
        elif getattr(self, "_is_getup", False):
            reward, info = self._getup_reward(float(reward), info)
        elif getattr(self, "_is_recover", False):
            reward, term, info = self._recover_reward(
                float(reward), term, trunc, info)
        # reward.term_penalty (2026-08-18, fb_20260818T065930_03b422):
        # cfg-gated in-env twin of train_ppo_transfer's TRAINING-ONLY
        # _term_penalty_wrapper (one-time charge on early termination,
        # the dynrep pilots' anti-suicide term) so the batched MJX
        # trainers — which construct shim envs internally and cannot
        # wrap them — can train on the exact walkcurr2 reward contract.
        # Default 0.0 = bit-exact legacy. Eval/cert envs leave it 0
        # (evals run the raw reward, same rule as the transfer trainer).
        if term and not trunc:
            if self._term_penalty_override is not None:
                _tp = self._term_penalty_override
            else:
                _tp = float(cfg_get(self.cfg, "reward", "term_penalty",
                                    default=0.0))
            # reward.walk_idle_terminate_penalty (2026-08-24): the
            # sustained-idle boundary (safety.walk_idle_terminate_s)
            # fires on the SAME code path as every other termination,
            # which by default pays the full anti-suicide term_penalty
            # (sized, correctly, to make a fast tilt-death the worst
            # outcome in the bank). Reusing that same large lump for
            # idle-eviction was measured to collapse the fine per-tick
            # ranking the WALKCURR_PF bank requires among the
            # non-progressing behaviors (park/stall/reverse/sideways/
            # skate) -- once ALL of them also pay the ~1200 lump, only
            # their few pre-termination ticks of charge differ, and
            # a slow idle death can end up cheaper than the fast
            # topple death purely by paying fewer accumulated ticks,
            # inverting the required floor. A distinct, independently
            # sized penalty (default -1 sentinel = fall back to
            # reward.term_penalty, bit-exact unless set) lets a
            # walkcurr rung price "keep failing to progress" as
            # strictly worse than every live behavior WITHOUT
            # swamping the graded ranking among the live ones.
            if info.get("termination_reason") == "walk_idle_terminate":
                _tp_idle = float(cfg_get(
                    self.cfg, "reward", "walk_idle_terminate_penalty",
                    default=-1.0))
                if _tp_idle >= 0.0:
                    _tp = _tp_idle
            if _tp > 0.0:
                reward = float(reward) - _tp
        if self.walk_probe_on and self._wp is not None:
            # Measurement-only walk quality probe (walkcurr MJX cert,
            # fb_20260818T065930_03b422): accumulate AFTER the full
            # reward stack so per-episode return matches what the
            # trainer sees; on the terminal tick the summary rides the
            # info dict. Zero effect on obs/reward/rng.
            self._walk_probe_tick(float(reward), bool(term),
                                  bool(trunc), info)
        return obs, reward, term, trunc, info

    def _quad_income(self, reward: float, info: dict) -> tuple:
        """Quad-family lifted-fronts shaping, shared by the quad HOLD
        mode and quadwalk (08-13; pure code motion from the quad elif
        in _post_step — behavior identical for quad mode).

        Quad-hold shaping (feasibility GO, c57; all cfg-gated,
        default 0 = mode earns only the base kernels). Two income
        terms, no new charges — the level kernel, current charge
        and tilt trip already price the failure modes:
          k_quad_clear: pay each LIFT leg's height above its
            episode-start pad z, clipped at quad_clear_cap_mm,
            and only while that foot is OFF the ground (a loaded
            "lifted" leg earns nothing by construction).
          k_quad_plant: pay the loaded fraction of the four
            support legs — the four-planted half of the task.
        A grace window (quad_grace_s) keeps the settle + lift
        transient unpaid so income starts only once the hold
        could actually be happening.

        k_quad_still (08-13, quad track: the turn1-r1 dig-in measured
        the learned quad HOLD stance creeping ~0.33 m/15 s — stillness
        was never priced; hold_still_gate is scoped hold/track and
        exempts quad by design): per-tick charge on body planar speed
        above quad_still_floor_m_s, applied ONLY while no velocity is
        commanded (s_ref ~ 0) so it can never fight a quadwalk
        command. Default 0 = off, legacy exact.

        k_quad_lift_contact (08-13, quad track, after cw-quadwalk1/2:
        pure lift-INCOME pricing is a closed lever — 3x clear/plant
        income moved front-leg tail contact duty only 1.0 -> 0.62/0.32,
        never under the 0.15 fronts_lifted bar; six-leg walking from
        the warm start pays too well to abandon for a side bonus):
        per-tick CHARGE on the fraction of commanded LIFT legs in
        ground contact after the grace window. Makes keeping the
        fronts down strictly unprofitable instead of merely less
        profitable; the honest lifted form pays ~0 by construction.
        Applies to the whole quad family (quad hold's honest form has
        the fronts off the ground, so it is uncharged). Default 0 =
        off, legacy exact.
        """
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
            # walk_gait_gate (08-13): on commanded quadwalk ticks the
            # clear/plant income rides the SAME all-support-legs
            # factor as the kernel — otherwise "sit fronts-up and
            # scoot" keeps its ~2.25/tick sitting income while the
            # gate zeroes only transport pay (quadwalk4/5's cheat was
            # funded by exactly this stream). Quad HOLD mode and
            # quadwalk stillness segments are untouched (the factor
            # is reset to 1.0 on every non-commanded tick and the
            # mode check below skips hold outright); gate off =
            # factor 1.0 = bit-exact.
            qgt = 1.0
            if getattr(self._goal_traj, "mode", "") == "quadwalk":
                qgt = float(getattr(self, "_gait_gate_qfactor", 1.0))
            r_qc = k_qc * clear_sum / max(len(lift), 1) * qgt
            r_qp = k_qp * n_on / max(len(support), 1) * qgt
            reward = float(reward) + r_qc + r_qp
            info["reward_quad_clear"] = r_qc
            info["reward_quad_plant"] = r_qp
            info["quad_clear_mm"] = clear_mm / max(len(lift), 1)
            info["quad_fronts_off"] = fronts_off / max(len(lift), 1)
            info["quad_planted_frac"] = n_on / max(len(support), 1)
        k_ql = float(cfg_get(self.cfg, "reward", "k_quad_lift_contact",
                             default=0.0))
        if lift and k_ql > 0.0 and self._step_i > grace_n:
            n_lift_on = sum(
                1 for f in lift
                if self._touch_adr[f] >= 0
                and float(self.data.sensordata[self._touch_adr[f]]) > 0.5)
            if n_lift_on:
                r_ql = -k_ql * n_lift_on / max(len(lift), 1)
                reward = float(reward) + r_ql
                info["reward_quad_lift_contact"] = r_ql
        k_qs = float(cfg_get(self.cfg, "reward", "k_quad_still",
                             default=0.0))
        if lift and k_qs > 0.0 and self._step_i > grace_n:
            s_ref = float(np.hypot(getattr(goal, "vx_ref", 0.0),
                                   getattr(goal, "vy_ref", 0.0)))
            if s_ref <= 1e-3:
                sp = float(np.hypot(*self._body_vel_xy()))
                floor = float(cfg_get(self.cfg, "reward",
                                      "quad_still_floor_m_s",
                                      default=0.005))
                r_qs = -k_qs * max(sp - floor, 0.0)
                reward = float(reward) + r_qs
                info["reward_quad_still"] = r_qs
                info["quad_body_speed"] = sp
        return reward, info

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
        # Recalibrated 08-22 (getup_honest_ordering dig-in,
        # test_task_semantics.py's GETUP bank): at the old default 60
        # a genuine partial rise (holding a real mid-ramp crouch, feet
        # loaded) earned LESS than freezing at the untouched spawn
        # pose (partial -12.16 vs freeze -11.26, bank-measured 3
        # seeds) — not a values-need-remeasuring drift, a real
        # under-pricing: the one-shot ratchet credit for the honest
        # partial-stand's potential gain (~+10 at k=60) was smaller
        # than the ADDITIONAL regularizer (gyro/action/current) cost
        # of actually executing the rise motion the freeze never pays
        # (measured other-than-prog totals -21.6 partial vs -11.7
        # freeze, seed 11). Those regularizers are intentionally left
        # untouched (they price real physics, not this task's shape;
        # see the strip comment above) so the fix is sizing the
        # progress ratchet to clear that physical cost with margin,
        # not editing the physics charges. 200 gives partial +10.8 >
        # freeze -11.5 (>=20 margin) while leaving every other GETUP
        # bank ordering intact (replay/flagleg/stilt/thrash all keep
        # their required gaps at this value, swept 60->500).
        k_prog = float(cfg_get(self.cfg, "reward", "getup_k_progress",
                               default=200.0))
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
            _add_walk_direction_info(
                info, float(v[0]), float(v[1]), vx_ref, vy_ref,
                min_speed_m_s=float(cfg_get(
                    self.cfg, "goal", "walk_direction_min_speed_m_s",
                    default=WALK_DIRECTION_MIN_SPEED_M_S)))
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

    @staticmethod
    def _rec_gate(x: float) -> float:
        """Smooth gate g(x): smoothstep on [0,1] — C1, monotone, g(0)=0,
        g(1)=1. The directive requires smooth gates (no hard
        thresholds inside the potential)."""
        x = min(max(x, 0.0), 1.0)
        return x * x * (3.0 - 2.0 * x)

    def _recover_reward(self, reward: float, term: bool, trunc: bool,
                        info: dict) -> tuple[float, bool, dict]:
        """recover_to_plant pricing (08-15 directive
        fb_20260815T165306_606974; REWARD.md §4c).

        r = kP*(gamma*Phi(s') - Phi(s)) + B*first_held_success
            - c_time*dt (until termination, incl. the success hold)
            - fail_cost at timeout/safety termination without success
        plus the base regularizers (gyro/action/current — physics
        pricing stays; the tracking kernel + tilt shaping are stripped
        like getup ticks). Phi uses bounded [0,1] features with smooth
        gates:
          U  uprightness ((1+cos(tilt))/2 — full gradient from
             upside-down)
          L  mean six-foot load saturation
          H  supported stand-height progress (belly->z_full, the
             compliance-calibrated real stand height; overshoot above
             the FK plant fades to 0 like getup — stilt pops price
             themselves)
          M  SMOOTH-MIN per-foot load — one unloaded foot stays
             visible; the getup3-c2/getup4 mean-average plateau cannot
             recur by construction
          P  nominal-footprint closeness (the getup f_footprint fade)
          Phi = wU*U + wL*g(U)*L + wH*g(U)*L*H + wM*g(U)*g(H)*M
                + wP*g(U)*g(H)*P            (defaults .15/.15/.30/.30/.10)
        Success = 0.5 s CONTINUOUS hold of: |z - z_full| <= 15 mm,
        tilt <= 6 deg, every foot's load fraction >= rec_load_min AND
        pad spread small (all six near the ground and loaded — no
        mean-only loophole), footprint closeness >= 0.5 (support
        proxy), low joint/body velocity, and no current violation.
        Falls are NOT terminal; the episode ends only on held success
        (one-shot bonus, term=True), timeout, or the safety envelope.
        A non-success termination pays fail_cost >= the maximum
        remaining time tax, so early abort can never out-earn trying.
        PBRS telescopes over the episode, so the spawn potential is
        never income and re-farming a feature pays 0 by construction.
        """
        # 1) Strip kernel + tilt shaping (same rationale as getup).
        r_strip = (info.get("reward_task", 0.0)
                   + info.get("reward_roll", 0.0)
                   + info.get("reward_pitch", 0.0))
        if r_strip != 0.0:
            reward -= r_strip
            info["reward_task"] = 0.0
            info["reward_roll"] = 0.0
            info["reward_pitch"] = 0.0

        # 2) Bounded features.
        load_n = float(cfg_get(self.cfg, "reward", "rec_load_n",
                               default=1.0))
        x = np.zeros(6)
        for f in range(6):
            adr = self._touch_adr[f]
            if adr >= 0:
                t_n = max(float(self.data.sensordata[adr]), 0.0)
                x[f] = min(t_n / max(load_n, 1e-6), 1.0)
        feat_l = float(np.mean(x))
        tau = float(cfg_get(self.cfg, "reward", "rec_min_tau",
                            default=0.15))
        feat_m = float(min(max(
            -tau * math.log(float(np.mean(np.exp(-x / tau)))),
            0.0), 1.0))
        t_roll, t_pitch = self._true_roll_pitch()
        cos_t = math.cos(t_roll) * math.cos(t_pitch)
        feat_u = min(max((1.0 + cos_t) / 2.0, 0.0), 1.0)
        z_plant, _weight_n = self._getup_geom()
        z_belly = float(cfg_get(self.cfg, "reward", "getup_z_belly_mm",
                                default=38.0)) * 0.001
        z_frac = float(cfg_get(self.cfg, "reward", "getup_z_full_frac",
                               default=0.80))
        z_full = z_belly + z_frac * max(z_plant - z_belly, 1e-3)
        z = float(self.data.xpos[self._chassis_bid, 2])
        feat_h = min(max((z - z_belly) / max(z_full - z_belly, 1e-3),
                         0.0), 1.0)
        over = z - (z_plant + 0.02)
        if over > 0.0:
            feat_h *= min(max(1.0 - over / 0.06, 0.0), 1.0)
        curl = self._curl_dist()
        fp_ok = float(cfg_get(self.cfg, "reward", "getup_fp_ok_mm",
                              default=40.0)) * 0.001
        fp_hi = float(cfg_get(self.cfg, "reward", "getup_fp_hi_mm",
                              default=120.0)) * 0.001
        feat_p = min(max((fp_hi - curl) / max(fp_hi - fp_ok, 1e-6),
                         0.0), 1.0)
        g_u = self._rec_gate(feat_u)
        g_h = self._rec_gate(feat_h)
        w_u = float(cfg_get(self.cfg, "reward", "rec_w_u", default=0.15))
        w_l = float(cfg_get(self.cfg, "reward", "rec_w_l", default=0.15))
        w_h = float(cfg_get(self.cfg, "reward", "rec_w_h", default=0.30))
        w_m = float(cfg_get(self.cfg, "reward", "rec_w_m", default=0.30))
        w_p = float(cfg_get(self.cfg, "reward", "rec_w_p", default=0.10))
        phi = (w_u * feat_u + w_l * g_u * feat_l
               + w_h * g_u * feat_l * feat_h
               + w_m * g_u * g_h * feat_m
               + w_p * g_u * g_h * feat_p)

        # 3) Potential difference (PBRS). Seeded at the first
        #    post-settle tick — no income for the spawn posture.
        k_pot = float(cfg_get(self.cfg, "reward", "rec_k_pot",
                              default=20.0))
        gam = float(cfg_get(self.cfg, "reward", "rec_gamma",
                            default=0.995))
        r_pot = 0.0
        if self._rec_phi_prev is not None:
            r_pot = k_pot * (gam * phi - self._rec_phi_prev)
        self._rec_phi_prev = phi
        reward += r_pot

        # 4) Success detection + 0.5 s continuous hold.
        h_tol = float(cfg_get(self.cfg, "reward", "rec_h_tol_mm",
                              default=15.0)) * 0.001
        lev_deg = float(cfg_get(self.cfg, "reward", "rec_level_deg",
                                default=6.0))
        load_min = float(cfg_get(self.cfg, "reward", "rec_load_min",
                                 default=0.35))
        spread_max = float(cfg_get(self.cfg, "reward",
                                   "rec_pad_spread_mm",
                                   default=30.0)) * 0.001
        qd_max = float(cfg_get(self.cfg, "reward", "rec_qd_max_rad_s",
                               default=0.7))
        v_max = float(cfg_get(self.cfg, "reward", "rec_v_max_m_s",
                              default=0.08))
        cur_max = float(cfg_get(self.cfg, "reward", "rec_cur_max_a",
                                default=3.0))
        tilt_deg = max(abs(t_roll), abs(t_pitch)) * 180.0 / math.pi
        pad_z = np.array([float(self.data.xpos[b, 2])
                          for b in self._pad_bids])
        spread = float(np.max(pad_z) - np.min(pad_z))
        qd_rms = float(np.sqrt(np.mean(
            np.square(self._state.joint_velocity))))
        v = self._body_vel_xy()
        cur = getattr(self._state, "servo_current", None)
        cur_ok = (cur is None or cur_max <= 0.0
                  or float(np.max(cur)) <= cur_max)
        ok = (abs(z - z_full) <= h_tol
              and tilt_deg <= lev_deg
              and float(np.min(x)) >= load_min
              and spread <= spread_max
              and feat_p >= 0.5
              and qd_rms <= qd_max
              and float(np.hypot(v[0], v[1])) <= v_max
              and cur_ok)
        self._rec_hold_n = self._rec_hold_n + 1 if ok else 0
        hold_need = max(int(round(float(cfg_get(
            self.cfg, "reward", "rec_hold_s", default=0.5))
            / self.dt)), 1)
        success = self._rec_hold_n >= hold_need

        # 5) Time tax — every tick until termination, INCLUDING the
        #    success hold (the directive's speed incentive; a normal
        #    ~4 s recovery costs ~8% of the success bonus at defaults).
        c_time = float(cfg_get(self.cfg, "reward", "rec_c_time",
                               default=1.0))
        reward -= c_time * self.dt

        # 6) Terminal handling.
        r_bonus = 0.0
        if success:
            r_bonus = float(cfg_get(self.cfg, "reward",
                                    "rec_b_success", default=50.0))
            reward += r_bonus
            term = True
            info["termination_reason"] = "recover_success"
        elif term or trunc:
            # timeout / safety-envelope end without success: pay at
            # least the maximum remaining time tax so aborting early
            # (or coasting into the horizon) never beats recovering.
            fail = float(cfg_get(self.cfg, "reward", "rec_fail_cost",
                                 default=0.0))
            if fail <= 0.0:
                fail = 1.25 * c_time * self.episode_steps * self.dt
            reward -= fail
            info["reward_recover_fail"] = -fail
        if term or trunc:
            # Stochastic rollout bookkeeping is diagnostic only.  The
            # adaptive sampler/admission state is updated exclusively by
            # apply_recover_certification() from deterministic MJX passes.
            kind = getattr(self._goal_traj, "start_kind", "?")
            bucket = self.RECOVER_KIND_BUCKETS.get(kind, -1)
            if (getattr(self._goal_traj, "recover_rsi", False)
                    or getattr(self._goal_traj, "recover_rsi_bank",
                              False)):
                # RSI episodes (ref-path goal.recover_rsi_frac OR
                # harvested-bank goal.recover_rsi_bank_frac) practice
                # on-path waypoints, not the family's own start: keep
                # them OUT of the rollout EMA/counters and the
                # C-trainer self-cert stats so no curriculum or
                # diagnostic signal is inflated by easier on-path
                # spawns. Each logs under its own suffix.
                suffix = ("_rsi" if getattr(self._goal_traj,
                                            "recover_rsi", False)
                         else "_rsibank")
                info[f"recover_episode_{kind}{suffix}"] = 1.0
                info[f"recover_success_{kind}{suffix}"] = (
                    1.0 if success else 0.0)
            else:
                # Do not use the strict stochastic success bit as replay
                # error: exploration noise can interrupt an otherwise good
                # 0.5 s hold.  Terminal potential shortfall preserves a
                # graded signal; a true safety termination is maximal error.
                training_error = (0.0 if success else
                                  (1.0 if term and not trunc else
                                   float(np.clip(1.0 - phi, 0.0, 1.0))))
                info["recover_training_error"] = training_error
                ema, n = self._rec_rollout_stats.get(kind, (0.5, 0))
                beta = float(cfg_get(self.cfg, "goal",
                                     "recover_ema_beta", default=0.25))
                updated = (
                    (1.0 - beta) * ema
                    + beta * (1.0 if success else 0.0),
                    n + 1)
                self._rec_rollout_stats[kind] = updated
                successes, episodes = self._rec_rollout_counts.get(
                    kind, (0, 0))
                self._rec_rollout_counts[kind] = (
                    successes + int(success), episodes + 1)
                # Preserve the legacy self-certified curriculum for the
                # C trainer.  MJX recovery runs opt into external
                # certification in train_ppo_mjx._env_kwargs, so their
                # noisy PPO actions can never mutate _rec_stats.
                if not self._rec_external_certification:
                    cert_successes, cert_episodes = self._rec_stats.get(
                        kind, (0, 0))
                    self._rec_stats[kind] = (
                        cert_successes + int(success), cert_episodes + 1)
                    if bucket >= 0:
                        self.apply_recover_training_error_batch({
                            bucket: (training_error, 1)})
                info[f"recover_episode_{kind}"] = 1.0
                info[f"recover_success_{kind}"] = (
                    1.0 if success else 0.0)
                if bucket >= 0:
                    info[f"recover_episode_bucket_{bucket}"] = 1.0
                    info[f"recover_success_bucket_{bucket}"] = (
                        1.0 if success else 0.0)

        info["reward_recover_pot"] = r_pot
        info["reward_recover_bonus"] = r_bonus
        info["reward_recover_time"] = -c_time * self.dt
        info["recover_phi"] = phi
        info["recover_U"] = feat_u
        info["recover_L"] = feat_l
        info["recover_H"] = feat_h
        info["recover_M"] = feat_m
        info["recover_P"] = feat_p
        info["recover_hold_n"] = float(self._rec_hold_n)
        info["recover_success"] = 1.0 if success else 0.0
        info["recover_rsi_episode"] = (
            1.0 if getattr(self._goal_traj, "recover_rsi", False)
            else 0.0)
        info["recover_rsi_bank_episode"] = (
            1.0 if getattr(self._goal_traj, "recover_rsi_bank", False)
            else 0.0)
        info["recover_min_load"] = float(np.min(x))
        info["recover_tilt_deg"] = tilt_deg
        kind = getattr(self._goal_traj, "start_kind", "?")
        bucket = self.RECOVER_KIND_BUCKETS.get(kind, -1)
        info["recover_start_kind_id"] = float(
            self.RECOVER_KIND_IDS.get(kind, -1))
        info["recover_start_bucket"] = float(bucket)
        info["recover_active_families"] = float(self._rec_active_n)
        info["recover_frontier_bucket"] = float(self._rec_active_n - 1)
        info["recover_max_unlocked_bucket"] = float(
            self._rec_active_n - 1)
        info["recover_focus_bucket"] = float(self._rec_focus_bucket)
        info["recover_weakest_bucket"] = float(
            -1 if self._rec_weak_bucket is None else self._rec_weak_bucket)
        info[f"recover_reset_height_mm_{kind}"] = float(
            self._rec_reset_height_mm)
        info[f"recover_reset_tilt_deg_{kind}"] = float(
            self._rec_reset_tilt_deg)
        info[f"recover_reset_min_load_n_{kind}"] = float(
            self._rec_reset_min_load_n)
        info[f"recover_reset_pad_spread_mm_{kind}"] = float(
            self._rec_reset_pad_spread_mm)
        for rec_kind in self._recover_active_kinds():
            cert_successes, cert_episodes = self._rec_stats.get(
                rec_kind, (0, 0))
            cert_fraction = (cert_successes / cert_episodes
                             if cert_episodes else 0.0)
            info[f"recover_curriculum_fraction_{rec_kind}"] = float(
                cert_fraction)
            info[f"recover_curriculum_successes_{rec_kind}"] = float(
                cert_successes)
            info[f"recover_curriculum_episodes_{rec_kind}"] = float(
                cert_episodes)
            roll_ema, roll_n = self._rec_rollout_stats.get(
                rec_kind, (0.5, 0))
            info[f"recover_rollout_ema_{rec_kind}"] = float(roll_ema)
            info[f"recover_rollout_n_{rec_kind}"] = float(roll_n)
            roll_successes, roll_episodes = self._rec_rollout_counts.get(
                rec_kind, (0, 0))
            info[f"recover_rollout_fraction_{rec_kind}"] = float(
                roll_successes / roll_episodes if roll_episodes else 0.0)
        error_priority = self._recover_training_error_distribution()
        if error_priority is None:
            error_priority = np.zeros(self._rec_active_n, dtype=float)
        for rec_bucket in range(self._rec_active_n):
            stats = [self._rec_stats.get(k, (0, 0))
                     for k in self._recover_family_kinds(rec_bucket)]
            if stats:
                successes = sum(v[0] for v in stats)
                episodes = sum(v[1] for v in stats)
                info[
                    f"recover_curriculum_bucket_{rec_bucket}_success_fraction"
                ] = float(successes / episodes if episodes else 0.0)
                info[f"recover_curriculum_bucket_{rec_bucket}_successes"] = (
                    float(successes))
                info[f"recover_curriculum_bucket_{rec_bucket}_episodes"] = (
                    float(episodes))
            error_ema, error_episodes = (
                self._rec_training_error_stats.get(rec_bucket, (0.0, 0)))
            info[f"recover_training_error_ema_bucket_{rec_bucket}"] = (
                float(error_ema))
            info[f"recover_training_error_n_bucket_{rec_bucket}"] = (
                float(error_episodes))
            info[f"recover_training_error_priority_bucket_{rec_bucket}"] = (
                float(error_priority[rec_bucket]))
        for rec_bucket, probability in enumerate(
                self._recover_bucket_weights()):
            info[f"recover_sample_probability_bucket_{rec_bucket}"] = (
                float(probability))
        return reward, term, info
