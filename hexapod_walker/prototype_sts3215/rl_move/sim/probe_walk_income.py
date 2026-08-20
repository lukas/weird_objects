"""probe_walk_income.py — term-by-term WALK income decomposition.

WHY (RL_PLAN queue 2.1 / rl_docs/TURN.md, 08-11): three omni arms each
collapsed into a DIFFERENT degenerate gait (mirror1-r1 freeze,
mirror2/dr02 leg-sacrifice tripod, trans1 paddle-stall) while the
task-semantics bank kept passing — the bank compares full-command gait
vs march-in-place vs park, but the real attractors are INTERMEDIATE:
partial progress with degenerate leg usage. Before any new omni arm,
find which reward channel still pays for that.

WHAT IT DOES: rolls policies in the exact reward stack of a named run
(trans1 = the current deliverable stack; mirror2 = + yaw set) with a
pinned command, and accumulates EVERY per-term reward component the
env emits (info["reward_*"]) plus the income-gate factors, progress
ratio, and per-leg duty/swing fingerprints. Policies are scripted
references (honest gait, half-speed gait, 1/3-leg sacrifice, paddle,
freeze — built to match the video fingerprints) and/or real
checkpoints (ckpt:<path>).

    python -m rl_move.sim.probe_walk_income --stack trans1 \
        --policies gait,gait_slow,sac1,sac3,paddle,freeze,ckpt:rl_move/sim/policies/ppo_goal_cw_omni_trans1.zip \
        --dirs forward,backward,crab_left,diag_back_right \
        --seeds 0,1,2 --jobs 8 --out logs/probe_walk_income/trans1.json

Residual = episode return − Σ(labelled terms): non-zero residual is the
BASE stack (task kernel, stance/clearance/still, penalties) — printed
as its own row so no income channel can hide.

CLEAN-vs-DIRTY VERDICT (operator RISE_WALK_NEXT_48H directive, 08-13:
"why does the dirty/crouched/sliding policy earn more return than the
clean teacher?" — answer it BEFORE any coefficient sweep): every
rollout now also measures ACTUATOR FEASIBILITY under the fitted servo
model (requested joint-target speed vs the effective per-joint clamp
min(write_speed, vel_max_deg_s); fraction of joint-ticks cruising AT
the clamp; simultaneous-saturation ticks; profile backlog; command-vs-
measured tracking error), and `--clean <pol> --dirty <pol>` prints a
matched per-term return decomposition (identical seeds + pinned
command schedule) plus termination economics, then classifies the gap:

  A. reward economics prefer sliding (income channels dominate, clean
     physically keeps up);
  B. actuator constraints make clean stepping infeasible/expensive
     (clean saturates the clamp and undertracks);
  C. neither channel explains the gap (architecture/other — escalate).

    python -m rl_move.sim.probe_walk_income --stack vref1 \
        --policies noslip_clean,ckpt:rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --dirs forward --seeds 0,1,2 \
        --clean noslip_clean \
        --dirty ckpt:rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --out logs/probe_walk_income/clean_vs_dirty_vref1.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

WALK_PLANT = (20.0, 80.0)
CMD_V = 0.055           # mid of the trained 0.05-0.06 band
EPISODE_S = 15.0

# Exact env-relevant cfg of cw-omni-trans1 (ledger extra_args 08-11) —
# the CURRENT deliverable stack (turn de-scoped).
TRANS1_STACK = {
    ("reward", "k_step_event"): 1.0,
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_park_duty"): 1.0,
    ("goal", "walk_speed_min_m_s"): 0.05,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("goal", "walk_park_start_frac"): 0.25,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("goal", "walk_heading_max_rad"): 3.14159,
    ("goal", "walk_cmd_resample_s"): 1.5,
    ("goal", "walk_cmd_resample_jitter"): 0.6,
    ("goal", "walk_cmd_blend_s_min"): 0.1,
    ("goal", "walk_cmd_blend_s_max"): 1.0,
    ("goal", "walk_stop_frac"): 0.2,
    ("obs", "history_frames"): 16,
    ("goal", "walk_obs_body_vel"): 2.0,
    ("safety", "max_roll_deg"): 25.0,
    ("safety", "max_pitch_deg"): 25.0,
    ("reward", "k_current"): 0.0,
}
# cw-omni-mirror2 adds the full yaw set (ledger extra_args 08-11).
MIRROR2_STACK = dict(TRANS1_STACK)
MIRROR2_STACK.update({
    ("goal", "walk_yaw_cmd"): 1,
    ("goal", "walk_yaw_max_rad_s"): 0.3,
    ("goal", "walk_yaw_zero_frac"): 0.5,
    ("goal", "walk_turn_in_place_frac"): 0.30,
    ("reward", "k_walk_yaw"): 1.0,
    ("reward", "walk_yaw_kernel_gate"): 1.0,
    ("reward", "k_yaw_prog"): 1.0,
    ("reward", "k_yaw_still"): 50.0,
    ("reward", "walk_kernel_yaw_gate"): 1.0,
})
# mirror2 + the 08-11 latent-defect fixes (walk_task.py): heading-hold
# yaw income gated on achieved linear progress, drift charge on the wz
# EMA instead of the instantaneous oscillation. The stack any future
# turn arm must train with (TURN_OVERRIDES in test_task_semantics.py).
MIRROR2FIX_STACK = dict(MIRROR2_STACK)
MIRROR2FIX_STACK.update({
    ("reward", "walk_yaw_hold_prog_gate"): 1.0,
    ("reward", "yaw_still_avg_s"): 1.0,
})
# Exact env-relevant cfg of cw-dep-vref1-r1 (ledger extra_args) — THE
# hardware walk champion's own training stack (RL_PLAN queue -0.5 P0,
# operator 08-11: "is the reward accurate?"). Note vs trans1: NO
# hist16, NO k_step_event, NO omni heading/resample set — defaults
# apply. Purpose: replay the tape-proven scripted gait AT PLANT HEIGHT
# through the exact stack the crouch-paddling champion trained on and
# compare full returns (income + penalties + termination risk).
VREF1_STACK = {
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_park_duty"): 1.0,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("goal", "walk_park_start_frac"): 0.25,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("goal", "walk_speed_min_m_s"): 0.05,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("goal", "walk_obs_body_vel"): 2.0,
    ("safety", "max_roll_deg"): 25.0,
    ("safety", "max_pitch_deg"): 25.0,
}
STACKS = {"trans1": TRANS1_STACK, "mirror2": MIRROR2_STACK,
          "mirror2fix": MIRROR2FIX_STACK, "vref1": VREF1_STACK}

DIRS = {
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "crab_left": (0.0, 1.0),
    "diag_back_right": (-0.707, -0.707),
}

# Video fingerprints being priced (rl_docs/TURN.md):
#   sac3   mirror2/dr02: one tripod held planted, body barely moves
#   sac1   the classic single flag-leg walk (leg pinned at plant)
#   paddle trans1: legs 1,4 planted 90-99% duty, other four rapid
#          ~0.01 m strides, partial progress
#   driftride  the yawcmd1/yawgate2/turnfix1 fingerprint: track the
#          linear command but rotate at the structural ~+0.09 rad/s
#          left drift, collecting the yaw kernel's heading-hold band.
#          Scripted omega 0.25 calibrated to ACHIEVE ~0.088 rad/s
#          while translating (the gait realizes ~40% of commanded
#          omega — measured, test_task_semantics.py DRIFT_RIDE_WZ).
#   noslip / noslip_slow / noslip_clean: the step-then-shift
#          world-anchored gait (linux_control/noslip_gait.py) —
#          quasi-static, zero commanded foot drag. noslip = default
#          timing (realizes ~0.02-0.03 m/s, BELOW the trained 0.05-0.06
#          band); noslip_slow = an early slow timing (~0.005 m/s);
#          noslip_clean = NoSlipGait.CLAMP_FIT_KW, the 08-12 sweep's
#          cleanest timing under the fitted 31 deg/s clamp (zero true
#          scrub, travel ratio 0.96 at 13 mm/s commanded).
SCRIPTED = ("gait", "gait_slow", "sac1", "sac3", "paddle", "freeze",
            "driftride")
PIN = {"sac1": (0,), "sac3": (0, 2, 4), "paddle": (1, 4)}
VEL_SCALE = {"gait_slow": 0.5, "paddle": 0.3}
DRIFT_RIDE_OMEGA = 0.25

# gate-factor / diagnostic means worth reporting alongside term sums
FACTOR_KEYS = ("walk_prog_factor", "walk_anchor_frac",
               "walk_loadslip_factor", "walk_height_factor",
               "walk_yaw_gate_factor", "walk_yaw_kernel_factor",
               "walk_yaw_hold_factor")


def make_env(seed: int, stack: dict, dr_scale: float = 0.0,
             extra_sets: tuple = ()):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in stack.items():
        cfg.setdefault(sec, {})[leaf] = val
    # --set overrides land AFTER the named stack (audit tool: replay a
    # run's exact ledger cfg on top of the nearest base stack).
    for (sec, leaf), val in extra_sets:
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=dr_scale > 0.0,
        dr_scale=dr_scale, episode_seconds=EPISODE_S, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def pin_command(env, vx: float, vy: float) -> None:
    """Deterministic command: hold 1 s, ramp 1 s, then constant."""
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = vx
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = vx * ramp
    traj.vy[:] = vy
    traj.vy[:hold_n] = 0.0
    traj.vy[hold_n:hold_n + ramp_n] = vy * ramp
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0


def parse_policy_spec(policy: str) -> tuple[str, float]:
    """Split '<name>[@p<scale>]' -> (base_name, tripod period_scale).

    Fast-cadence lever (operator order 08-20): 'gait@p0.75' rolls the
    scripted TripodGait with its clock period scaled by 0.75
    (0.5625 s cycle instead of 0.75 s). No suffix = 1.0 = bit-exact
    legacy behavior. Only meaningful for TripodGait-backed policies."""
    base, _, sfx = policy.partition("@")
    if not sfx:
        return base, 1.0
    if not sfx.startswith("p"):
        raise SystemExit(f"bad policy suffix {policy!r}; want "
                         f"<name>@p<scale>, e.g. gait@p0.75")
    return base, float(sfx[1:])


def rollout(policy: str, direction: str, seed: int, stack_name: str,
            deterministic: bool = True, dr_scale: float = 0.0,
            extra_sets: tuple = (), cmd: float = CMD_V) -> dict:
    from tripod_gait import TripodGait

    base_pol, period_scale = parse_policy_spec(policy)
    stack = STACKS[stack_name]
    env = make_env(seed, stack, dr_scale, extra_sets)
    obs, _ = env.reset()
    ux, uy = DIRS[direction]
    vx, vy = ux * cmd, uy * cmd
    pin_command(env, vx, vy)
    traj = env._goal_traj
    n = len(traj.vx)

    model = None
    gait = None
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    if base_pol.startswith("ckpt:"):
        from stable_baselines3 import PPO
        model = PPO.load(str(ROOT / base_pol[5:]), device="cpu")
    elif base_pol.startswith("noslip"):
        from noslip_gait import NoSlipGait
        if base_pol == "noslip_clean":
            gait = NoSlipGait.clamp_fit()
        else:
            kw = (dict(period=8.0, shift_frac=0.20, swing_frac=0.26)
                  if base_pol == "noslip_slow" else {})
            gait = NoSlipGait(**kw)
        gait.sync_plant_stance(*WALK_PLANT)
    else:
        gait = TripodGait(vx=0.0, period_scale=period_scale)
        gait.sync_plant_stance(*WALK_PLANT)
        gait.reset_phase()

    term_sums: dict[str, float] = defaultdict(float)
    factor_sums: dict[str, float] = defaultdict(float)
    factor_n: dict[str, int] = defaultdict(int)
    contact_hist = []
    # Fast-cadence preflight metrics (operator order 08-20): loaded
    # foot-XY slip (same accounting as eval_checkpoint slip_m_total:
    # pad motion between ticks counted when the pad was in contact at
    # the earlier tick) and chassis height, so the raw-teacher check
    # (tall / no falls / reasonable slip / real travel) reads off one
    # probe table.
    z_hist: list[float] = []
    slip_m = 0.0
    pad_prev = None
    contact_prev: list[bool] | None = None
    total, step, cmd_dist, along_dist = 0.0, 0, 0.0, 0.0
    wz_sum, wz_n = 0.0, 0
    term_reason = None
    scale = VEL_SCALE.get(base_pol, 1.0)

    # Actuator-feasibility accumulators (clean-vs-dirty directive):
    # requested target speed |Δq_cmd|/dt vs the profile's effective
    # clamp, cruise-at-clamp saturation, backlog, tracking error. All
    # read AFTER env.step so post-SafetyLayer commands are measured —
    # the same signal the hardware SafetyLayer clamp analysis uses.
    prev_cmd = env._cmd.copy()
    req_speed: list[np.ndarray] = []      # per-tick (18,) rad/s
    sat_counts: list[int] = []            # joints cruising at clamp
    lag: list[float] = []                 # max |goal-target| rad
    track_err: list[np.ndarray] = []      # per-tick (18,) rad
    vel_clamp = np.minimum(
        env._profile._vel_default,
        float(env.write_speed_deg_s) * DEG2RAD)

    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if model is not None:
            act, _ = model.predict(obs, deterministic=deterministic)
        elif base_pol == "freeze":
            act = q_rad_to_action(plant_rad)
        else:
            omega = (DRIFT_RIDE_OMEGA if base_pol == "driftride"
                     and t >= 2.0 else 0.0)
            gait.set_velocity(vx=float(traj.vx[i]) * scale,
                              vy=float(traj.vy[i]) * scale, omega=omega)
            q = np.asarray(gait.desired_deg(t)) * DEG2RAD
            for leg in PIN.get(base_pol, ()):
                q[3 * leg:3 * leg + 3] = plant_rad[3 * leg:3 * leg + 3]
            act = q_rad_to_action(q)
        obs, r, term, trunc, info = env.step(act)
        total += float(r)
        for k, v in info.items():
            if k.startswith("reward_"):
                term_sums[k] += float(v)
            elif k in FACTOR_KEYS:
                factor_sums[k] += float(v)
                factor_n[k] += 1
        g = env._current_goal()
        if g is not None:
            s_ref = math.hypot(g.vx_ref, g.vy_ref)
            if s_ref > 1e-3:
                v_b = env._body_vel_xy()
                cmd_dist += s_ref * env.dt
                along_dist += ((v_b[0] * g.vx_ref + v_b[1] * g.vy_ref)
                               / s_ref) * env.dt
                wz_sum += env._body_wz()
                wz_n += 1
        contact_hist.append([
            float(env.data.sensordata[adr]) > 0.5
            for adr in env._touch_adr])
        z_hist.append(float(env.data.xpos[env._chassis_bid, 2]))
        pad_now = env.data.xpos[env._pad_bids, :2].copy()
        if pad_prev is not None:
            moved = np.linalg.norm(pad_now - pad_prev, axis=1)
            slip_m += float(moved[np.asarray(contact_prev, bool)].sum())
        pad_prev = pad_now
        contact_prev = contact_hist[-1]
        cmd_now = env._cmd.copy()
        req_speed.append(np.abs(cmd_now - prev_cmd) / env.dt)
        prev_cmd = cmd_now
        prof = env._profile
        sat_counts.append(int(np.sum(
            np.abs(prof._v) >= 0.999 * np.maximum(prof._vel_now, 1e-9))))
        lag.append(float(np.max(np.abs(prof.goal - prof.target))))
        track_err.append(np.abs(env._state.joint_position - cmd_now))
        step += 1
        if term or trunc:
            term_reason = info.get("termination_reason")
            break
    dt_env = env.dt
    env.close()

    rs = np.asarray(req_speed) / DEG2RAD          # (T, 18) deg/s
    te = np.asarray(track_err) / DEG2RAD          # (T, 18) deg
    sat = np.asarray(sat_counts, dtype=float)
    feas = {
        "vel_clamp_deg_s_mean": float(np.mean(vel_clamp) / DEG2RAD),
        "req_speed_deg_s_p95": float(np.percentile(rs, 95)),
        "req_speed_deg_s_max": float(np.max(rs)) if rs.size else 0.0,
        # joint-ticks demanding more than the joint's effective clamp
        "req_over_clamp_frac": float(np.mean(
            rs > (vel_clamp / DEG2RAD)[None, :])),
        # joint-ticks cruising AT the clamp (profile velocity pinned)
        "sat_joint_frac": float(np.mean(sat / 18.0)),
        # ticks with >= 6 joints simultaneously slew-saturated
        "sat6_tick_frac": float(np.mean(sat >= 6.0)),
        "lag_deg_p95": float(np.percentile(np.asarray(lag) / DEG2RAD, 95)
                             if lag else 0.0),
        "track_rmse_deg": float(np.sqrt(np.mean(te ** 2))),
    }

    contact = np.asarray(contact_hist, dtype=bool)
    duty = contact.mean(axis=0)
    swings = [int(np.sum(np.diff(contact[:, f].astype(int)) == -1))
              for f in range(6)]
    labelled = float(sum(term_sums.values()))
    return {
        "policy": policy, "dir": direction, "seed": seed,
        "stack": stack_name, "dr_scale": dr_scale, "ticks": step, "return": total,
        "terms": dict(term_sums),
        "residual_base": total - labelled,
        "factors": {k: factor_sums[k] / max(factor_n[k], 1)
                    for k in factor_sums},
        "progress_ratio": along_dist / cmd_dist if cmd_dist > 0 else 0.0,
        "wz_mean": wz_sum / wz_n if wz_n else 0.0,
        # ruled skating metric: loaded foot-XY travel per meter of
        # along-command progress (eval_checkpoint convention)
        "slip_per_m": round(slip_m / max(along_dist, 0.05), 3),
        "slip_m_total": round(slip_m, 3),
        # chassis height after the 2 s hold+ramp prefix (tall-check)
        "height_mm_mean": round(1000.0 * float(np.mean(
            z_hist[min(len(z_hist) - 1, int(round(2.0 / dt_env))):])), 1)
        if z_hist else None,
        "duty": [round(float(d), 3) for d in duty],
        "swings": swings,
        "feas": feas,
        "terminated": term_reason if term_reason not in (None, "time")
        else None,
    }


def _job(args):
    return rollout(*args)


def summarize(records: list[dict]) -> None:
    by_pol: dict[str, list[dict]] = defaultdict(list)
    for r in records:
        by_pol[r["policy"]].append(r)
    pols = list(by_pol)
    all_terms = sorted({t for r in records for t in r["terms"]})

    def pname(p):
        if not p.startswith("ckpt:"):
            return p
        # distinguish multiple checkpoints: last path piece, trimmed
        base = p[5:].rsplit("/", 1)[-1]
        base = base.replace("ppo_goal_cw_", "").replace(".zip", "")
        return base[-13:]

    hdr = f"{'':28s}" + "".join(f"{pname(p):>14s}" for p in pols)
    print("\n=== per-episode MEAN income by term (all dirs/seeds) ===")
    print(hdr)
    for t in all_terms + ["residual_base", "TOTAL_RETURN"]:
        row = f"{t:28s}"
        for p in pols:
            rs = by_pol[p]
            if t == "TOTAL_RETURN":
                v = np.mean([r["return"] for r in rs])
            elif t == "residual_base":
                v = np.mean([r["residual_base"] for r in rs])
            else:
                v = np.mean([r["terms"].get(t, 0.0) for r in rs])
            row += f"{v:14.1f}"
        print(row)
    print("\n--- behavior fingerprints (mean) ---")
    for lab, key in (("progress_ratio", "progress_ratio"),
                     ("wz_mean", "wz_mean"),
                     ("slip_per_m", "slip_per_m"),
                     ("height_mm_mean", "height_mm_mean")):
        row = f"{lab:28s}"
        for p in pols:
            vals = [r[key] for r in by_pol[p]
                    if r.get(key) is not None]
            row += (f"{np.mean(vals):14.2f}" if vals else f"{'-':>14s}")
        print(row)
    row = f"{'terminated_eps':28s}"
    for p in pols:
        row += f"{sum(1 for r in by_pol[p] if r['terminated']):14d}"
    print(row)
    print("\n--- actuator feasibility (mean; fitted servo clamp) ---")
    for fk in ("req_speed_deg_s_p95", "req_over_clamp_frac",
               "sat_joint_frac", "sat6_tick_frac", "lag_deg_p95",
               "track_rmse_deg"):
        row = f"{fk:28s}"
        for p in pols:
            vals = [r["feas"][fk] for r in by_pol[p] if "feas" in r]
            row += (f"{np.mean(vals):14.3f}" if vals else f"{'-':>14s}")
        print(row)
    for fk in FACTOR_KEYS:
        vals = {p: [r["factors"][fk] for r in by_pol[p]
                    if fk in r["factors"]] for p in pols}
        if not any(vals.values()):
            continue
        row = f"{fk:28s}"
        for p in pols:
            row += (f"{np.mean(vals[p]):14.2f}" if vals[p]
                    else f"{'-':>14s}")
        print(row)
    print("\n--- per-direction TOTAL return ---")
    dirs = sorted({r["dir"] for r in records})
    for d in dirs:
        row = f"{d:28s}"
        for p in pols:
            rs = [r for r in by_pol[p] if r["dir"] == d]
            row += (f"{np.mean([r['return'] for r in rs]):14.1f}"
                    if rs else f"{'-':>14s}")
        print(row)


# Directive channel map: every reward_* term the stack can emit, folded
# into the report categories of RISE_WALK_NEXT_48H "P1 — Understand why
# PPO prefers the dirty gait". Unlisted terms land in "other" and are
# still printed individually — no channel can hide.
CHANNELS = {
    "progress_tracking": (
        "reward_task", "reward_walk", "reward_walk_prog",
        "reward_step_event", "reward_swing", "reward_walk_yaw",
        "reward_yaw_prog", "reward_getup_prog", "reward_getup_hold",
        "reward_getup_walk"),
    "slip_drag": ("reward_drag", "reward_drag_stance",
                  "reward_drag_trans"),
    "effort_current": ("reward_current", "reward_current_max",
                       "reward_current_hot", "reward_effort"),
    "posture_height": (
        "reward_height", "reward_roll", "reward_pitch", "reward_stance",
        "reward_clearance", "reward_flag_leg", "reward_support_margin",
        "reward_load_even", "reward_end_posture"),
    "action_smoothness": ("reward_action", "reward_action_delta",
                          "reward_gyro", "reward_still",
                          "reward_yaw_still"),
    "contact": ("reward_phase_contact", "reward_park_duty",
                "reward_quad_clear", "reward_quad_plant",
                "reward_quad_lift_contact", "reward_quad_still"),
    "termination": ("reward_termination", "reward_alive"),
}
_TERM_TO_CHANNEL = {t: c for c, ts in CHANNELS.items() for t in ts}


def verdict(records: list[dict], clean: str, dirty: str) -> dict:
    """Matched clean-vs-dirty decomposition + hypothesis classification.

    Uses only (dir, seed) cells BOTH policies ran, so the comparison is
    the directive's "matched states/commands": identical resets and
    identical pinned command schedules.
    """
    def cells(pol):
        return {(r["dir"], r["seed"]): r for r in records
                if r["policy"] == pol}

    rc, rd = cells(clean), cells(dirty)
    keys = sorted(set(rc) & set(rd))
    if not keys:
        print(f"\nVERDICT: no matched (dir, seed) cells for "
              f"clean={clean!r} dirty={dirty!r} — nothing to compare")
        return {}
    cs = [rc[k] for k in keys]
    ds = [rd[k] for k in keys]

    def mean_terms(rs):
        out: dict[str, float] = defaultdict(float)
        for r in rs:
            for t, v in r["terms"].items():
                out[t] += v / len(rs)
            out["residual_base"] += r["residual_base"] / len(rs)
        return out

    tc, td = mean_terms(cs), mean_terms(ds)
    all_terms = sorted(set(tc) | set(td))
    ret_c = float(np.mean([r["return"] for r in cs]))
    ret_d = float(np.mean([r["return"] for r in ds]))
    gap = ret_d - ret_c

    chan_delta: dict[str, float] = defaultdict(float)
    print(f"\n=== VERDICT: dirty − clean, matched over {len(keys)} "
          f"(dir, seed) cells ===")
    print(f"{'term':28s}{'clean':>12s}{'dirty':>12s}{'delta':>12s}")
    rows = []
    for t in all_terms:
        d = td.get(t, 0.0) - tc.get(t, 0.0)
        rows.append((t, tc.get(t, 0.0), td.get(t, 0.0), d))
        chan_delta[_TERM_TO_CHANNEL.get(t, "other")] += d
    for t, c, dv, d in sorted(rows, key=lambda r: -abs(r[3])):
        if abs(d) < 0.05 and abs(c) < 0.05 and abs(dv) < 0.05:
            continue
        print(f"{t:28s}{c:12.1f}{dv:12.1f}{d:12.1f}")
    print(f"{'TOTAL_RETURN':28s}{ret_c:12.1f}{ret_d:12.1f}{gap:12.1f}")

    # Termination economics: early ends forfeit the survivor's mean
    # per-tick income for the remaining ticks — report it explicitly so
    # "fall risk was cheap/expensive" is a number, not a feeling.
    ticks_c = float(np.mean([r["ticks"] for r in cs]))
    ticks_d = float(np.mean([r["ticks"] for r in ds]))
    full_ticks = max(max(r["ticks"] for r in cs + ds), 1)
    inc_c = ret_c / max(ticks_c, 1.0)
    inc_d = ret_d / max(ticks_d, 1.0)
    forfeits_c = (full_ticks - ticks_c) * inc_c
    forfeits_d = (full_ticks - ticks_d) * inc_d
    n_term_c = sum(1 for r in cs if r["terminated"])
    n_term_d = sum(1 for r in ds if r["terminated"])
    print(f"\ntermination economics: clean {n_term_c}/{len(cs)} eps "
          f"terminated (mean {ticks_c:.0f} ticks, ~{forfeits_c:.1f} "
          f"return forfeited) | dirty {n_term_d}/{len(ds)} "
          f"(mean {ticks_d:.0f} ticks, ~{forfeits_d:.1f} forfeited)")

    def mfeas(rs, k):
        return float(np.mean([r["feas"][k] for r in rs if "feas" in r]))

    feas_keys = ("req_speed_deg_s_p95", "req_over_clamp_frac",
                 "sat_joint_frac", "sat6_tick_frac", "track_rmse_deg")
    fc = {k: mfeas(cs, k) for k in feas_keys}
    fd = {k: mfeas(ds, k) for k in feas_keys}
    print("\nactuator feasibility (fitted servo-speed clamp):")
    for k in feas_keys:
        print(f"  {k:26s} clean {fc[k]:8.3f}   dirty {fd[k]:8.3f}")

    print("\nchannel deltas (dirty − clean):")
    for c, d in sorted(chan_delta.items(), key=lambda kv: -abs(kv[1])):
        print(f"  {c:26s}{d:10.1f}")

    # Mechanical classification (evidence summary, not a decree):
    #   B if the clean gait is pinned at the clamp much harder than the
    #     dirty one AND tracks its own commands worse;
    #   A if the gap is dominated by income channels while clean keeps
    #     up physically;
    #   C/other if neither pattern holds.
    clean_saturates = (fc["sat_joint_frac"]
                       > max(1.5 * fd["sat_joint_frac"], 0.10)
                       or fc["req_over_clamp_frac"]
                       > max(1.5 * fd["req_over_clamp_frac"], 0.10))
    clean_undertracks = fc["track_rmse_deg"] > fd["track_rmse_deg"] * 1.5
    top = sorted(chan_delta.items(), key=lambda kv: -abs(kv[1]))[:2]
    top_str = ", ".join(f"{c} {d:+.1f}" for c, d in top)
    print("\n--- verdict ---")
    print(f"dirty out-earns clean by {gap:+.1f} return/episode; "
          f"largest channels: {top_str}")
    if gap <= 0:
        klass = "clean_wins"
        print("the clean policy already out-earns the dirty one under "
            "this stack — the dirty attractor is NOT the paid optimum "
            "here. If PPO still converges to it, the failure is "
            "optimization/exploration (or a training-env pricing "
            "mismatch, e.g. the measured MJX-vs-C anchoring gap), not "
            "reward economics. Compare the same pair under the MJX "
            "training pricing before concluding.")
    elif clean_saturates and clean_undertracks:
        klass = "B"
        print("hypothesis B evidence: the clean gait demands speeds the "
              f"fitted clamp cannot deliver (sat_joint_frac "
              f"{fc['sat_joint_frac']:.2f} vs {fd['sat_joint_frac']:.2f}"
              f", track RMSE {fc['track_rmse_deg']:.2f} vs "
              f"{fd['track_rmse_deg']:.2f} deg) — clean stepping is "
              "infeasible/expensive under the current actuator model. "
            "Intervention: fix/verify the clamp fit (sysid loaded runs) "
            "or retime the clean gait under the clamp before repricing "
            "anything.")
    elif abs(chan_delta.get("progress_tracking", 0.0)) >= 0.5 * abs(gap) \
            and gap > 0:
        klass = "A"
        print("hypothesis A evidence: the gap is dominated by "
            "progress/tracking income the dirty gait collects while its "
            f"slip/drag charges move only {chan_delta.get('slip_drag', 0.0):+.1f}"
            " — reward economics underprice sliding relative to "
            "velocity-tracking income. Intervention: raise ONLY the "
            "implicated slip/drag price enough to reverse this measured "
            "advantage (preregister the budget), not a blind sweep.")
    else:
        klass = "C/other"
        print("neither the actuator-infeasibility nor the "
            "income-dominance pattern explains the gap — check "
            "observation/control architecture (hypothesis C) or an "
            "unlisted mechanism before touching coefficients.")
    return {"matched_cells": len(keys), "return_clean": ret_c,
            "return_dirty": ret_d, "gap": gap,
            "channel_delta": dict(chan_delta),
            "term_clean": dict(tc), "term_dirty": dict(td),
            "feas_clean": fc, "feas_dirty": fd,
            "termination": {"clean_eps": n_term_c, "dirty_eps": n_term_d,
                            "clean_forfeit": forfeits_c,
                            "dirty_forfeit": forfeits_d},
            "class": klass}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stack", choices=list(STACKS), default="trans1")
    ap.add_argument("--policies", default=",".join(SCRIPTED))
    ap.add_argument("--dirs", default=",".join(DIRS))
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--stochastic", action="store_true")
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--set", action="append", default=[],
                    dest="sets", metavar="SEC.KEY=VAL",
                    help="cfg override applied AFTER the stack; "
                         "repeatable (audit: replay a run's ledger cfg)")
    ap.add_argument("--cmd", type=float, default=CMD_V,
                    help=f"pinned command speed m/s (default {CMD_V})")
    ap.add_argument("--clean", default=None,
                    help="policy name (as in --policies) of the CLEAN "
                         "reference for the verdict block")
    ap.add_argument("--dirty", default=None,
                    help="policy name of the DIRTY policy for the "
                         "verdict block")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    extra_sets = []
    for s in a.sets:
        key, _, val = s.partition("=")
        sec, _, leaf = key.partition(".")
        if not (sec and leaf and _):
            raise SystemExit(f"bad --set {s!r}, want SEC.KEY=VAL")
        try:
            v = float(val)
        except ValueError:
            v = val
        extra_sets.append(((sec, leaf), v))
    extra_sets = tuple(extra_sets)

    pols = [p for p in a.policies.split(",") if p]
    dirs = [d for d in a.dirs.split(",") if d]
    seeds = [int(s) for s in a.seeds.split(",") if s != ""]
    jobs = [(p, d, s, a.stack, not a.stochastic, a.dr_scale,
             extra_sets, a.cmd)
            for p in pols for d in dirs for s in seeds]
    print(f"probe_walk_income: stack={a.stack} dr={a.dr_scale} "
          f"cmd={a.cmd} sets={a.sets} {len(jobs)} rollouts "
          f"({len(pols)} policies x {len(dirs)} dirs x {len(seeds)} seeds)")
    if a.jobs > 1:
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            records = list(ex.map(_job, jobs))
    else:
        records = [_job(j) for j in jobs]
    summarize(records)
    vd = None
    if a.clean and a.dirty:
        vd = verdict(records, a.clean, a.dirty)
    if a.out:
        out = ROOT / a.out
        out.parent.mkdir(parents=True, exist_ok=True)
        payload = ({"records": records, "verdict": vd}
                   if vd is not None else records)
        out.write_text(json.dumps(payload, indent=1))
        print(f"\nWROTE {out}")


if __name__ == "__main__":
    main()
