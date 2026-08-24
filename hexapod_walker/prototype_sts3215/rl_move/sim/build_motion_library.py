"""build_motion_library.py — AMP track M1: from-scratch motion-prior
dataset (rl_docs/AMP_LOCOMOTION.md §4 "Motion-Prior Dataset").

Source (§4.3, "a clean hand-designed alternating-tripod kinematic
generator"): the hardware-proven scripted teacher
(linux_control/tripod_gait.py TripodGait), driven through REAL MuJoCo
physics at the measured tibia-150 plant (CURRENT_TRUTHS.md: 0.06-0.10
m/s x 4 headings, zero falls, slip/m 1.4-2.9) — not a bare kinematic
replay, so joint velocities/contacts/orientation come from the actual
simulator the policy will train in.

Command coverage (§4.2), all produced by directly commanding the
teacher's existing vx/vy/omega inputs (no mirroring transform needed —
the simulator is left/right symmetric, so a negative vy or omega
command already produces the true left/right or CW/CCW trajectory):
forward x3 speeds, backward x2 speeds, lateral left/right, turn
CW/CCW, forward-while-turning, diagonal, accel-from-rest,
decel-to-rest.

Discriminator feature set recorded per tick (§3.6): joint positions
relative to a per-clip neutral pose (the post-reset spawn stance),
joint velocities, base angular velocity (body-frame gyro), projected
gravity (world -z rotated into the body frame), and foot positions
relative to the body frame — concatenated as ``obs_style`` (60-dim:
18+18+3+3+18). Transitions are adjacent ticks of the same clip
(§4.5); this script stores per-tick ``obs_style`` and the loader
takes obs_style[i], obs_style[i+1] pairs within a clip's index range
(never across a clip boundary — see ``clip_starts``/``clip_lens``).

Validation/rejection (§4.4) reuses the SAME slip/m and fall
vocabulary the eval harness scores elsewhere in this repo (no new
physics, no new pass/fail definitions): a clip is rejected if the
episode terminated (fall) or if its loaded-foot slip/m exceeds
``--reject-slip-per-m`` (default 3.5, above the teacher's own
measured 1.4-2.9 band to allow for the coarser per-clip estimator
here, but well below the RL slip-financed-progress cheat band
4.5-6.8 documented in the joystick track's phasedir5/6 findings).

ASSUMPTION (recorded per the "assume-and-go" directive, open item for
whoever builds the discriminator): "neutral pose" is defined per-clip
as the post-reset spawn stance (index 0 of that clip), not a single
global constant — this keeps the dataset self-consistent even if the
plant/stance geometry changes later, but the discriminator training
code must pick ONE convention (global vs per-clip neutral) consistent
with whatever the POLICY's own actor observation uses, and this
script's choice may need to change to match. Filed as
OPERATOR_QUESTIONS.md q_20260822T0900Z with this default.

CPU-only (pure MuJoCo rollout via the existing joint/walk env
machinery, no GPU/Warp) — run on the controller, not a train pod.

Usage:
    uv run python -m rl_move.sim.build_motion_library \
        --out rl_move/sim/motion_library/teacher_v1
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import mujoco  # noqa: E402

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

WALK_PLANT = (20.0, 80.0)   # hip/knee deg, matches the tibia-150 measured plant
CTRL_HZ = 25.0
GRAVITY_WORLD = np.array([0.0, 0.0, -1.0])
TURN_RADIUS_APPROX_M = 0.09  # nominal foot-to-yaw-axis radius for the
                            # slip-metric's rotation-as-speed proxy


def _make_env(seed: int, episode_seconds: float):
    """A bare walk env: no reward-mechanism cfg matters here (we only
    read physics/contacts), just the plant/servo/DR baseline every
    other bank test in this repo already agrees on."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _quat_rot_world_to_body(quat_wxyz: np.ndarray, v_world: np.ndarray) -> np.ndarray:
    """Rotate a world-frame vector into the body frame (inverse of the
    body->world rotation MuJoCo's xquat encodes)."""
    qinv = np.zeros(4)
    mujoco.mju_negQuat(qinv, quat_wxyz)
    out = np.zeros(3)
    mujoco.mju_rotVecQuat(out, v_world, qinv)
    return out


def _clip_transition_pairs(obs_style: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return obs_style[:-1], obs_style[1:]


# --------------------------------------------------------------------------
# Command coverage (§4.2). Each entry: name, a function t -> (vx, vy, wz),
# and a clip duration. Speeds/rates are the teacher's own measured/tested
# operating points (CURRENT_TRUTHS teacher band 0.06-0.10 m/s; TURN_CMD_WZ
# 0.25 rad/s from the turn bank in test_task_semantics.py).
RAMP_S = 1.0


def _ramp_hold(cmd, t, t0=RAMP_S):
    return cmd * min(max(t / t0, 0.0), 1.0)


def _families(clip_s: float = 6.0, turn_wz: float = 0.25):
    fam = {}
    for vx in (0.06, 0.08, 0.10):
        fam[f"forward_{vx:.2f}"] = (
            lambda t, vx=vx: (_ramp_hold(vx, t), 0.0, 0.0), clip_s)
    for vx in (-0.06, -0.08):
        fam[f"backward_{abs(vx):.2f}"] = (
            lambda t, vx=vx: (_ramp_hold(vx, t), 0.0, 0.0), clip_s)
    for vy, tag in ((0.05, "left"), (-0.05, "right")):
        fam[f"lateral_{tag}"] = (
            lambda t, vy=vy: (0.0, _ramp_hold(vy, t), 0.0), clip_s)
    # turn_wz: commanded rate for the PURE turn-in-place clips. 0.25 is
    # the legacy teacher_v1/v2 value (turn bank TURN_CMD_WZ). The 08-23
    # yaw-authority dig-in measured the untrimmed tripod teacher
    # ACHIEVING only 0.13-0.14 rad/s at cmd 0.25-0.30 (ratio ~0.5, the
    # exact price-invariant ratio every tipfrac05 policy is pinned at:
    # the discriminator sees base_angular_velocity unconditioned on
    # command, so demo wz IS the style ceiling). teacher_v3 builds with
    # turn_wz 0.30 + turn stance scales (below) = measured max
    # ~0.176 rad/s.
    for wz, tag in ((turn_wz, "ccw"), (-turn_wz, "cw")):
        fam[f"turn_{tag}"] = (
            lambda t, wz=wz: (0.0, 0.0, _ramp_hold(wz, t)), clip_s)
    for wz, tag in ((0.20, "ccw"), (-0.20, "cw")):
        fam[f"forward_turn_{tag}"] = (
            lambda t, wz=wz: (_ramp_hold(0.07, t), 0.0, _ramp_hold(wz, t)),
            clip_s)
    for vy, tag in ((0.04, "fl"), (-0.04, "fr")):
        fam[f"diagonal_{tag}"] = (
            lambda t, vy=vy: (_ramp_hold(0.06, t), _ramp_hold(vy, t), 0.0),
            clip_s)

    def _accel(t):
        return (min(t / 3.0, 1.0) * 0.08, 0.0, 0.0)
    fam["accel_from_rest"] = (_accel, 4.0)

    def _decel(t, t_dec=4.0, t_total=6.0):
        if t < t_dec:
            return (0.08, 0.0, 0.0)
        frac = max(0.0, 1.0 - (t - t_dec) / (t_total - t_dec))
        return (0.08 * frac, 0.0, 0.0)
    fam["decel_to_rest"] = (_decel, 6.0)
    return fam


def run_clip(name: str, cmd_fn, clip_s: float, seed: int, *,
             controller: str = "tripod", cpg_params=None,
             yaw_trim: bool = False,
             turn_scales: tuple[float, float] | None = None,
             cmd_cond: bool = False) -> dict:
    """Roll the scripted teacher through real physics for one command
    profile; return per-tick records + validation metrics.

    `controller="tripod"` (default) is byte-for-byte the original
    teacher_v1/teacher_v2 generator, unchanged. `controller="se2cpg"`
    (cpg track, 08-23) drives the SAME command families through the
    CPG-search `SE2FootGait` winner instead, for a controlled
    teacher_v2-vs-CPG motion-library A/B (`rl_docs/tracks/cpg/STATUS.md`
    Next item 3) -- everything else (env, physics, obs_style layout,
    validation) is identical so a downstream A/B compares only the
    GAIT, not the harness.
    """
    # FRAME FIX (08-22, fb_20260822T145428 audit): import through the
    # ONE sim-side knee-convention boundary (sim_gait_compat), NOT raw
    # tripod_gait. Since 30660b51 the raw class speaks the hardware's
    # ABSOLUTE-tibia convention: feeding its desired_deg() straight to
    # q_rad_to_action mis-poses every knee, and WALK_PLANT (20, 80) is
    # the SIM-RELATIVE canonical plant, which the raw
    # sync_plant_stance misreads as absolute. Measured divergence of
    # the raw stream vs the verified compat stream at this plant:
    # knee up to 15.7 deg (mean 80.6 vs 85.2), coxa up to 4.9 deg —
    # teacher_v1.npz clips are physically valid (15/15 accepted, low
    # slip) but are NOT the verified teacher's gait. Rebuilds from
    # this script now produce the true convention-correct motion
    # (default --out bumped to teacher_v2; v1 kept append-only).
    if controller == "se2cpg":
        from sim_gait_compat import SE2FootGait
        gait = SE2FootGait(
            gait=cpg_params.gait, period=cpg_params.period,
            swing_frac=cpg_params.swing_frac, lift=cpg_params.lift_m,
            cmd_tau=cpg_params.cmd_tau,
            workspace_margin=cpg_params.workspace_margin)
    else:
        from sim_gait_compat import TripodGait
        gait = TripodGait(vx=0.0, lift=0.025)

    env = _make_env(seed, episode_seconds=clip_s + 1.0)
    env.reset()
    gait.sync_plant_stance(*WALK_PLANT)
    # Turn-in-place stance-geometry scaling (08-23 yaw-authority
    # dig-in): the tripod's yaw rate SATURATES ~0.15 rad/s at default
    # stride/period no matter the commanded omega (measured 0.30 ->
    # 0.144, 0.60 -> 0.163, 0.80 -> 0.127 = degrading). Foot sweep per
    # stride is dx ~ omega*r*period/2*stride_scale, so stride 1.4 x
    # period 1.2 lifts the measured ceiling to ~0.176 rad/s at slip
    # proxy ~2.1 (best probed point). Tripod-only, turn_* clips only,
    # default None = bit-exact legacy build.
    if turn_scales is not None and controller == "tripod":
        gait.set_scales(stride_scale=turn_scales[0],
                        period_scale=turn_scales[1])
    gait.reset_phase() if controller != "se2cpg" else gait.reset_phase(t=0.0)

    dt = env.dt
    n = int(round(clip_s / dt))
    neutral_q = env.data.qpos[env._qadr].copy()
    # Closed-loop yaw trim (only meaningful for a PURE turn-in-place
    # clip -- name starts with "turn_"; forward+turn/diagonal clips are
    # untouched, matching the verified eval_cpg_gate.py scope): see
    # rl_move/sim/yaw_trim.py for why an open-loop CPG gait needs this.
    trim_active = (yaw_trim and controller == "se2cpg"
                   and name.startswith("turn_"))
    trim_scale = 1.0
    trim_ref_yaw = 0.0
    yaw_cum = 0.0
    yaw_prev = None
    if trim_active:
        from rl_move.sim.eval_cpg_gate import _body_yaw, _wrap_angle
        from rl_move.sim.yaw_trim import update_trim
        yaw_prev = _body_yaw(env)

    joint_pos, joint_vel, base_quat, base_angvel = [], [], [], []
    foot_pos_body, proj_grav, phase_label, cmd_hist = [], [], [], []
    prev_on = [False] * 6
    prev_xy = [None] * 6
    ls_slip_m, ls_prog_m = 0.0, 0.0
    max_delta_q_deg = 0.0
    fell = False

    for step in range(n):
        t = step * dt
        vx, vy, wz = cmd_fn(t)
        cmd_hist.append((vx, vy, wz))
        gait.set_velocity(vx=vx, vy=vy, omega=wz * trim_scale
                          if trim_active else wz)
        if trim_active:
            yaw_now = _body_yaw(env)
            if step > 0:
                yaw_cum += _wrap_angle(yaw_now - yaw_prev)
            yaw_prev = yaw_now
            period_steps = max(1, int(round(1.0 / dt)))
            if step > 0 and step % period_steps == 0:
                measured_wz = (yaw_cum - trim_ref_yaw) / (period_steps * dt)
                trim_ref_yaw = yaw_cum
                trim_scale = update_trim(trim_scale, measured_wz, wz)
        q_prev = env.data.qpos[env._qadr].copy()
        act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        _obs, _r, term, trunc, _info = env.step(act)
        q_now = env.data.qpos[env._qadr].copy()
        max_delta_q_deg = max(max_delta_q_deg,
                              float(np.max(np.abs(q_now - q_prev))) / DEG2RAD)

        joint_pos.append(q_now.copy())
        joint_vel.append(env.data.qvel[env._vadr].copy())
        quat = env.data.xquat[env._chassis_bid].copy()
        base_quat.append(quat)
        gyro = env.data.sensordata[
            env._gyro_adr:env._gyro_adr + 3].copy()
        base_angvel.append(gyro)
        proj_grav.append(_quat_rot_world_to_body(quat, GRAVITY_WORLD))
        chassis_xyz = env.data.xpos[env._chassis_bid].copy()
        feet = []
        # Include a rotation term so pure turn-in-place clips still get a
        # non-zero along-command "speed" denominator (leg-radius proxy,
        # matching the walk env's own s_ref convention) -- without this,
        # turn_cw/turn_ccw clips always read slip/m=0 (vacuous check).
        s_cmd = math.hypot(vx, vy) + abs(wz) * TURN_RADIUS_APPROX_M
        for f in range(6):
            xy_world = env.data.xpos[env._pad_bids[f], :2].copy()
            z_world = env.data.xpos[env._pad_bids[f], 2]
            rel_world = np.array([xy_world[0] - chassis_xyz[0],
                                  xy_world[1] - chassis_xyz[1],
                                  z_world - chassis_xyz[2]])
            rel_body = _quat_rot_world_to_body(quat, rel_world)
            feet.append(rel_body)
            adr = env._touch_adr[f]
            on = bool(adr >= 0 and env.data.sensordata[adr] > 0.5)
            if s_cmd > 1e-3:
                if prev_on[f] and prev_xy[f] is not None:
                    ls_slip_m += float(np.linalg.norm(xy_world - prev_xy[f]))
                prev_xy[f] = xy_world.copy()
                prev_on[f] = on
        if s_cmd > 1e-3:
            ls_prog_m += s_cmd * dt
        foot_pos_body.append(np.stack(feet))
        # Which tripod group (PHASE_TRIPOD_A vs. the other) the gait
        # clock currently commands into stance -- alternates every
        # half period by construction, unlike an agreement/mismatch
        # bit which stays ~1 the whole clip for a clean honest gait.
        if controller == "se2cpg":
            # SE2FootGait has no single sinusoidal phase scalar (its
            # scheduler assigns per-group swing WINDOWS over
            # cpg_params.period) -- this field is descriptive metadata
            # only (not consumed by any training/discriminator code
            # today, grep-confirmed), so a period-locked proxy is
            # sufficient to keep the array's shape/semantics.
            phase_label.append(
                1 if math.sin(2 * math.pi * t / cpg_params.period) >= 0.0
                else 0)
        else:
            phase_label.append(1 if math.sin(gait._phase) >= 0.0 else 0)
        if term:
            fell = True
            break
        if trunc:
            break
    env.close()

    n_got = len(joint_pos)
    joint_pos = np.asarray(joint_pos)
    joint_vel = np.asarray(joint_vel)
    base_quat = np.asarray(base_quat)
    base_angvel = np.asarray(base_angvel)
    proj_grav = np.asarray(proj_grav)
    foot_pos_body = np.asarray(foot_pos_body)
    phase_label = np.asarray(phase_label, dtype=np.int8)
    cmd_hist = np.asarray(cmd_hist)
    joint_pos_rel_neutral = joint_pos - neutral_q[None, :]

    # 08-23 yaw-authority follow-up (command-conditioned discriminator,
    # see amp_features.obs_style_from_data's cmd= docstring): default
    # False = bit-exact legacy 60-dim obs_style. True appends this
    # clip's per-tick (vx, vy, wz) command history (identical values
    # already computed above as ``cmd_hist``, and the SAME order
    # sim_env.py's live cmd-cond path uses: vx_ref, vy_ref, wz_ref) —
    # 63-dim. A library built one way can only pair with a live run
    # configured the same way; a dim mismatch is a loud AMPDiscriminator
    # shape error, never silent.
    style_parts = [
        joint_pos_rel_neutral,
        joint_vel,
        base_angvel,
        proj_grav,
        foot_pos_body.reshape(n_got, -1),
    ]
    if cmd_cond:
        style_parts.append(cmd_hist)
    obs_style = (np.concatenate(style_parts, axis=1) if n_got > 0
                else np.zeros((0, 63 if cmd_cond else 60)))

    slip_per_m = ls_slip_m / max(ls_prog_m, 0.05)
    # Measured steady-state body yaw rate over the final 3 s (08-23
    # yaw-authority dig-in): the manifest must record what a clip
    # actually ROTATES at, not just what was commanded — teacher_v2's
    # turn clips are labeled 0.25 rad/s but embody ~0.134, and that gap
    # (invisible until now) was the AMP style ceiling that pinned every
    # tipfrac05 policy at ratio ~0.5 of the commanded rate. Pure
    # measurement: no behavior/validation change.
    measured_wz = 0.0
    if n_got > 5:
        yaws = np.array([math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])) for q in base_quat])
        yaw_unwrapped = np.unwrap(yaws)
        k = min(int(round(3.0 / dt)), n_got - 1)
        measured_wz = float(
            (yaw_unwrapped[-1] - yaw_unwrapped[-1 - k]) / (k * dt))
    return dict(
        name=name, seed=seed, n_ticks=n_got, dt=dt,
        joint_position=joint_pos, joint_position_rel_neutral=joint_pos_rel_neutral,
        joint_velocity=joint_vel, base_orientation=base_quat,
        base_angular_velocity=base_angvel, projected_gravity=proj_grav,
        foot_positions=foot_pos_body, phase_label=phase_label,
        command=cmd_hist, obs_style=obs_style,
        fell=fell, slip_per_m=slip_per_m, max_delta_q_deg=max_delta_q_deg,
        neutral_pose=neutral_q, measured_wz=measured_wz,
    )


def validate(clip: dict, reject_slip_per_m: float, min_ticks: int) -> tuple[bool, str]:
    if clip["fell"]:
        return False, "terminated (fall)"
    if clip["n_ticks"] < min_ticks:
        return False, f"too short ({clip['n_ticks']} ticks)"
    if clip["slip_per_m"] > reject_slip_per_m:
        return False, f"dragging (slip/m {clip['slip_per_m']:.2f})"
    if clip["max_delta_q_deg"] > 8.0:
        return False, f"joint discontinuity ({clip['max_delta_q_deg']:.1f} deg/tick)"
    return True, "ok"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="rl_move/sim/motion_library/teacher_v2")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--clip-seconds", type=float, default=6.0)
    ap.add_argument("--reject-slip-per-m", type=float, default=3.5)
    ap.add_argument("--min-ticks", type=int, default=50)
    ap.add_argument("--turn-wz", type=float, default=0.25,
                    help="commanded yaw rate for the pure turn-in-place"
                         " clips (default 0.25 = legacy teacher_v1/v2)")
    ap.add_argument("--turn-stride-scale", type=float, default=1.0,
                    help="TripodGait stride_scale applied ONLY to "
                         "turn_* clips (tripod controller only; "
                         "default 1.0 = legacy). 08-23 probe: 1.4 with "
                         "--turn-period-scale 1.2 lifts achieved turn "
                         "rate 0.134 -> 0.176 rad/s")
    ap.add_argument("--turn-period-scale", type=float, default=1.0,
                    help="TripodGait period_scale applied ONLY to "
                         "turn_* clips (tripod controller only; "
                         "default 1.0 = legacy)")
    ap.add_argument("--controller", choices=["tripod", "se2cpg"],
                    default="tripod",
                    help="tripod (default, unchanged teacher_v1/v2 "
                         "generator) or se2cpg (cpg-track winner, "
                         "cpg/STATUS.md Next item 3)")
    ap.add_argument("--cpg-params-from", default="",
                    help="required with --controller se2cpg: a "
                         "paper_cpg_search report JSON (uses "
                         "best.params) or an eval_cpg_gate --export "
                         "controller artifact JSON (uses .params)")
    ap.add_argument("--no-yaw-trim", action="store_true",
                    help="with --controller se2cpg, skip the "
                         "closed-loop yaw trim on turn_* clips "
                         "(default: trim ON, matching the verified "
                         "robust-gate recipe)")
    ap.add_argument("--cmd-cond", action="store_true",
                    help="append each clip's per-tick (vx, vy, wz) "
                         "command history to obs_style (60 -> 63 dim) "
                         "for a command-conditioned AMP discriminator "
                         "(08-23 yaw-authority follow-up; default off "
                         "= bit-exact legacy 60-dim library). Pair "
                         "only with a live run that sets "
                         "goal.amp_style_cmd_cond=1.")
    args = ap.parse_args()

    cpg_params = None
    if args.controller == "se2cpg":
        if not args.cpg_params_from:
            print("--controller se2cpg requires --cpg-params-from")
            sys.exit(1)
        from rl_move.sim.paper_cpg_search import GaitParams, _clip_params
        raw = json.loads(Path(args.cpg_params_from).read_text())
        params_dict = raw["params"] if "params" in raw else raw["best"]["params"]
        cpg_params = _clip_params(GaitParams(**params_dict))

    families = _families(args.clip_seconds, turn_wz=args.turn_wz)
    turn_scales = None
    if (args.turn_stride_scale != 1.0 or args.turn_period_scale != 1.0):
        turn_scales = (args.turn_stride_scale, args.turn_period_scale)
    source = ("scripted_tripod_gait_teacher via sim_gait_compat "
              "knee-convention boundary (linux_control/tripod_gait.py"
              " + linux_control/sim_gait_compat.py)"
              " @ tibia-150 plant, real MuJoCo physics, no RL"
              if args.controller == "tripod" else
              "cpg_search_se2_foot_gait winner (params from "
              f"{args.cpg_params_from}) via sim_gait_compat.SE2FootGait"
              " @ tibia-150 plant, real MuJoCo physics, no RL"
              f"{'' if args.no_yaw_trim else ' + closed-loop yaw trim on turn_* clips (rl_move/sim/yaw_trim.py)'}")
    manifest = {"families": sorted(families), "clips": [], "source": source,
                "controller": args.controller,
                "turn_wz": args.turn_wz,
                "turn_stride_scale": args.turn_stride_scale,
                "turn_period_scale": args.turn_period_scale,
                "cpg_params": (dict(gait=cpg_params.gait,
                                    period=cpg_params.period,
                                    swing_frac=cpg_params.swing_frac,
                                    lift_m=cpg_params.lift_m,
                                    cmd_tau=cpg_params.cmd_tau,
                                    workspace_margin=cpg_params.workspace_margin)
                              if cpg_params is not None else None),
                "cmd_cond": args.cmd_cond,
                "obs_style_layout": ("joint_pos_rel_neutral(18) + "
                "joint_velocity(18) + base_angular_velocity(3) + "
                "projected_gravity(3) + foot_positions_rel_body(18) + "
                "command_vx_vy_wz(3) = 63" if args.cmd_cond else
                "joint_pos_rel_neutral(18) + "
                "joint_velocity(18) + base_angular_velocity(3) + "
                "projected_gravity(3) + foot_positions_rel_body(18) = 60"),
                "neutral_convention": "per-clip: qpos at clip tick 0 "
                "(post-reset spawn stance) -- see script docstring "
                "ASSUMPTION / OPERATOR_QUESTIONS.md q_20260822T0900Z"}

    accepted, rejected = [], []
    for name, (cmd_fn, clip_s) in families.items():
        for seed in args.seeds:
            clip = run_clip(name, cmd_fn, clip_s, seed,
                            controller=args.controller,
                            cpg_params=cpg_params,
                            yaw_trim=not args.no_yaw_trim,
                            turn_scales=(turn_scales
                                         if name.startswith("turn_")
                                         else None),
                            cmd_cond=args.cmd_cond)
            ok, reason = validate(clip, args.reject_slip_per_m, args.min_ticks)
            entry = dict(name=name, seed=seed, n_ticks=clip["n_ticks"],
                        slip_per_m=round(clip["slip_per_m"], 3),
                        measured_wz=round(clip["measured_wz"], 3),
                        max_delta_q_deg=round(clip["max_delta_q_deg"], 2),
                        fell=clip["fell"], accepted=ok, reason=reason,
                        quality_score=round(
                            (0.0 if not ok else
                             1.0 / (1.0 + max(clip["slip_per_m"] - 1.0, 0.0))),
                            3))
            manifest["clips"].append(entry)
            if ok:
                accepted.append(clip)
            else:
                rejected.append(entry)
            print(f"[{'OK ' if ok else 'REJ'}] {name} seed={seed} "
                  f"n={clip['n_ticks']} slip/m={clip['slip_per_m']:.2f} "
                  f"maxdq={clip['max_delta_q_deg']:.1f}deg reason={reason}")

    if not accepted:
        print("NO CLIPS ACCEPTED -- nothing written.")
        sys.exit(1)

    clip_lens = [c["n_ticks"] for c in accepted]
    clip_starts = np.cumsum([0] + clip_lens[:-1])
    total_ticks = sum(clip_lens)
    print(f"\n{len(accepted)}/{len(manifest['clips'])} clips accepted, "
          f"{total_ticks} ticks ({total_ticks / CTRL_HZ:.1f}s @ {CTRL_HZ}Hz)")

    def _cat(key):
        return np.concatenate([c[key] for c in accepted], axis=0)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        str(out_path) + ".npz",
        joint_position=_cat("joint_position"),
        joint_position_rel_neutral=_cat("joint_position_rel_neutral"),
        joint_velocity=_cat("joint_velocity"),
        base_orientation=_cat("base_orientation"),
        base_angular_velocity=_cat("base_angular_velocity"),
        projected_gravity=_cat("projected_gravity"),
        foot_positions=_cat("foot_positions"),
        phase_label=_cat("phase_label"),
        command=_cat("command"),
        obs_style=_cat("obs_style"),
        clip_starts=clip_starts,
        clip_lens=np.asarray(clip_lens),
        clip_names=np.asarray([c["name"] for c in accepted]),
        clip_seeds=np.asarray([c["seed"] for c in accepted]),
        dt=accepted[0]["dt"],
    )
    with open(str(out_path) + "_manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"wrote {out_path}.npz + {out_path}_manifest.json")


if __name__ == "__main__":
    main()
