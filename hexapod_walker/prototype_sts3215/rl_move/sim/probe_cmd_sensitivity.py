"""Command-plumbing / policy-sensitivity probe for unified walk policies.

Operator kick 2026-08-28 (MCP 20260828T153954Z): after the manual-drive
report called the unified standwalk policy "direction-deaf", decide
BUG vs NO-BUG on the command path with cheap local diagnostics instead
of more training budget. Three questions, three parts:

1. ``sweep`` — obs plumbing + frozen-state action sensitivity. Drive
   the policy (state threaded correctly — see gru_policy.
   RecurrentPredictor, the 08-28 runner-bug fix) to a mid-walk steady
   state, FREEZE physics and the GRU hidden state, and rebuild the
   observation under a swept command set (stop/+x/-x/+y/-y/diagonals).
   Reports (a) which obs indices change and by how much (command
   channels present and correctly scaled), and (b) the policy action
   delta per command at identical physical state + hidden state (does
   the actor READ the command at all).

2. ``response`` — behavioral command response. One fixed command per
   session (identical seed, DR-0, deterministic, same rise), measure
   net displacement over the walk window projected on the command.
   The per-command response matrix distinguishes "follows weakly in
   every direction" from "only forward works" (training draw gap:
   this lineage's walk_heading_max_rad=0.0 means lateral commands only
   ever appeared via the square/sweep schedule families).

3. ``--obs-cmd zero|rot90`` controls on ``response`` — same physics
   command script, but the OBS command channels (both the goal
   channels and, in this lineage's walk_obs_body_vel=2 contract, the
   ref-copy velocity channels — found empirically by obs diff) are
   zeroed / rotated before the policy sees them. A command-following
   policy must degrade / rotate its response; if metrics barely move,
   the harness was measuring gait survival, not command following.

Read-only: no env/reward semantics touched; commands are injected into
the live WalkTrajectory arrays exactly like manual_drive_session.

    uv run python -m rl_move.sim.probe_cmd_sensitivity <ckpt.zip> \
        --cfg-set k=v ... [--mode sweep|response|both] \
        [--obs-cmd normal,zero,rot90] [--out logs/manual_drive/…]
"""
from __future__ import annotations

import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import copy
import json
import math
import time
from pathlib import Path

import numpy as np

_PROTO = Path(__file__).resolve().parents[2]

SPEED = 0.08
DIAG = SPEED / math.sqrt(2.0)
SWEEP_CMDS = [
    ("stop", 0.0, 0.0), ("fwd", SPEED, 0.0), ("back", -SPEED, 0.0),
    ("left", 0.0, SPEED), ("right", 0.0, -SPEED),
    ("diag_fl", DIAG, DIAG), ("diag_fr", DIAG, -DIAG),
    ("diag_bl", -DIAG, DIAG), ("diag_br", -DIAG, -DIAG),
]
RESPONSE_CMDS = [("fwd", SPEED, 0.0), ("back", -SPEED, 0.0),
                 ("left", 0.0, SPEED), ("right", 0.0, -SPEED),
                 ("stop", 0.0, 0.0),
                 # Added 2026-08-28 (hdgset1 staged-curriculum canary):
                 # a heading-SET recipe (goal.walk_heading_set=[0,+-pi/4])
                 # never draws pure lateral (+-90deg) in training, so the
                 # fair in-distribution response check is the +-45deg
                 # diagonal a policy actually practiced, not left/right.
                 # (DIAG, DIAG) = +45deg (fwd-left), matching the SWEEP_CMDS
                 # convention above one-for-one.
                 ("diag_fl", DIAG, DIAG), ("diag_fr", DIAG, -DIAG)]
RISE_S = 20.0        # measured: this lineage needs ~14-16 s to stand
SETTLE_S = 2.0       # walk-segment settle before the command engages
CRUISE_S = 12.0      # forward cruise before the frozen-state sweep
RESP_S = 20.0        # response measurement window per command


def make_env(cfg_sets, seed):
    from rl_move.config import load_config
    from .train_ppo_sim import _parse_cfg_set
    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for key, parsed in _parse_cfg_set(cfg_sets).items():
        sect, name = key.split(".", 1)
        cfg.setdefault(sect, {})[name] = parsed
    cfg.setdefault("goal", {})["mode_seq"] = 1.0
    cfg["goal"]["rise_rsi_frac"] = 0.0
    cfg.setdefault("dr", {})["tipped_start_prob"] = 0.0
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg), randomize=False,
        dr_scale=0.0, episode_seconds=RISE_S + SETTLE_S + CRUISE_S
        + RESP_S + 10.0, seed=seed, render_mode=None, cfg=cfg)
    gen = env._goal_gen
    gen.p_rise = 1.0
    for m in ("walk", "hold", "lean", "track", "unload", "raise",
              "lower", "recover"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 0.0)
    gen.force_rise_start = "bridge"
    return env


def start_session(env, model):
    """reset + install a rise->walk plan; return first obs."""
    obs, _ = env.reset()
    if hasattr(model, "reset"):
        model.reset()
    dt = env.dt
    plan = [{"mode": "rise", "tick": 0, "blend": 0},
            {"mode": "walk", "tick": int(round(RISE_S / dt)),
             "blend": int(round(0.75 / dt))}]
    env._seq_plan = plan
    env._seq_idx = 0
    env._seq_seg_end = plan[1]["tick"]
    return obs


def counterfactual_obs(env, vx, vy):
    """Rebuild THIS tick's obs with only the commanded velocity changed.

    Overrides the live WalkTrajectory arrays at the current step (and
    restores them) so BOTH command consumers see the counterfactual:
    WalkGoal.as_obs's goal channels via build_obs AND _augment_obs's
    walk_obs_body_vel=2 ref-copy velocity channels (which re-read
    env._current_goal() internally).
    """
    from rl_move.env import build_obs
    tr = env._goal_traj
    assert tr is not None and hasattr(tr, "vx"), "not in a walk segment"
    i = min(max(env._step_i, 0), len(tr.vx) - 1)
    vx0, vy0 = float(tr.vx[i]), float(tr.vy[i])
    tr.vx[i], tr.vy[i] = float(vx), float(vy)
    # _augment_obs mutates only the phase clock when reset=False and a
    # command is active; snapshot+restore so repeated calls are pure.
    phase0 = env._phase
    try:
        goal = env._current_goal()
        base = build_obs(env.cfg, env._state, env._q_nom,
                         env._prev_action, goal=goal,
                         tilt_ref=env._tilt_ref0)
        out = env._augment_obs(base, reset=False).astype(np.float32)
    finally:
        tr.vx[i], tr.vy[i] = vx0, vy0
        env._phase = phase0
    assert env._hist_n <= 1, "obs history stacking not supported here"
    return out


def run_sweep(env, model, out):
    obs = start_session(env, model)
    dt = env.dt
    walk_tick = env._seq_plan[1]["tick"]
    n_cruise = walk_tick + int(round((SETTLE_S + CRUISE_S) / dt))
    for i in range(n_cruise):
        a, _ = model.predict(obs, deterministic=True)
        obs, _r, term, trunc, info = env.step(a)
        if env._seq_idx == 1 and hasattr(env._goal_traj, "vx"):
            tr = env._goal_traj
            if float(tr.vx[-1]) != SPEED:
                k0 = walk_tick + int(round(SETTLE_S / dt))
                tr.vx[:k0] = 0.0
                tr.vy[:] = 0.0
                tr.vx[k0:] = SPEED
        if term or trunc:
            return {"error": f"terminated during cruise at t="
                    f"{env._step_i * dt:.1f}s: "
                    f"{info.get('termination_reason')}"}
    # frozen point: same physics, same hidden state for every command
    state_snap = (copy.deepcopy(model._state)
                  if hasattr(model, "_state") else None)

    def frozen_action(o):
        if state_snap is not None:
            model._state = copy.deepcopy(state_snap)
            model._episode_start = np.zeros((1,), dtype=bool)
        a, _ = model.predict(o, deterministic=True)
        return np.asarray(a, dtype=float)

    base_obs = counterfactual_obs(env, SPEED, 0.0)   # the cruise command
    a_base = frozen_action(base_obs)
    rows = []
    for name, vx, vy in SWEEP_CMDS:
        o = counterfactual_obs(env, vx, vy)
        d_idx = np.where(np.abs(o - base_obs) > 1e-9)[0]
        a = frozen_action(o)
        rows.append({
            "cmd": name, "vx": vx, "vy": vy,
            "obs_diff_idx": d_idx.tolist(),
            "obs_diff_max": round(float(np.max(np.abs(o - base_obs)))
                                  if len(d_idx) else 0.0, 6),
            "act_delta_l2": round(float(np.linalg.norm(a - a_base)), 6),
            "act_delta_max": round(float(np.max(np.abs(a - a_base))), 6),
        })
    if state_snap is not None:
        model._state = copy.deepcopy(state_snap)
    return {"rows": rows,
            "note": "act deltas vs the fwd cruise command at IDENTICAL "
                    "physics + hidden state; action units = normalized "
                    "joint offsets in [-1,1]"}


def _obs_cmd_indices(env):
    """Empirical command-channel indices (goal channels + any ref-copy).

    Derived from fwd-vs-back (vx) and left-vs-right (vy) pairs: both
    members of each pair are MOVING commands, so the walk phase clock
    advances identically and its sin/cos channels cannot leak into the
    index sets (first probe version diffed against the STOP command,
    whose frozen clock put the phase channels in both sets — the
    zero/rot90 controls then fed an invalid sin=cos=0 phase, which by
    itself parks a phase-locked policy).
    """
    fwd = counterfactual_obs(env, SPEED, 0.0)
    back = counterfactual_obs(env, -SPEED, 0.0)
    left = counterfactual_obs(env, 0.0, SPEED)
    right = counterfactual_obs(env, 0.0, -SPEED)
    vx_ix = np.where(np.abs(fwd - back) > 1e-9)[0].tolist()
    vy_ix = np.where(np.abs(left - right) > 1e-9)[0].tolist()
    assert not set(vx_ix) & set(vy_ix), (
        f"vx/vy channel sets overlap: {vx_ix} vs {vy_ix}")
    return vx_ix, vy_ix


def run_response(env, model, obs_cmd, out):
    results = []
    for name, vx, vy in RESPONSE_CMDS:
        obs = start_session(env, model)
        dt = env.dt
        walk_tick = env._seq_plan[1]["tick"]
        k_go = walk_tick + int(round(SETTLE_S / dt))
        n_end = k_go + int(round(RESP_S / dt))
        vx_ix = vy_ix = None
        xy0 = None
        term_reason = None
        injected = False
        for i in range(n_end):
            if vx_ix is not None and obs_cmd != "normal":
                o = obs.copy()
                cvx = obs[vx_ix[0]] if vx_ix else 0.0
                cvy = obs[vy_ix[0]] if vy_ix else 0.0
                if obs_cmd == "zero":
                    o[vx_ix] = 0.0
                    o[vy_ix] = 0.0
                elif obs_cmd == "rot90":  # (vx,vy) -> (-vy, vx)
                    o[vx_ix] = -cvy
                    o[vy_ix] = cvx
                a, _ = model.predict(o, deterministic=True)
            else:
                a, _ = model.predict(obs, deterministic=True)
            obs, _r, term, trunc, info = env.step(a)
            if env._seq_idx == 1 and not injected \
                    and hasattr(env._goal_traj, "vx"):
                tr = env._goal_traj
                tr.vx[:k_go] = 0.0
                tr.vy[:k_go] = 0.0
                tr.vx[k_go:] = vx
                tr.vy[k_go:] = vy
                injected = True
                # Only needed to steer the zero/rot90 obs-command
                # controls (used a few lines up, gated on `obs_cmd !=
                # "normal"`); computing it unconditionally crashed
                # plain `--obs-cmd normal` response runs on any
                # walk_obs_body_vel=3 (leg-odometry) checkpoint, where
                # the fwd/back and left/right velocity-ESTIMATE
                # channels both react to a command flip on either axis
                # (found 2026-08-28 triaging cw-standwalk-unified1-
                # joyfix-velobs3-c1) and so legitimately overlap for
                # this heuristic — a real index-detection limitation of
                # the zero/rot90 controls under mode 3, not a reason to
                # lose the (unaffected) plain response numbers too.
                if obs_cmd != "normal":
                    vx_ix, vy_ix = _obs_cmd_indices(env)
            if env._step_i == k_go:
                xy0 = env.data.xpos[env._chassis_bid, :2].copy()
            if term or trunc:
                term_reason = info.get("termination_reason")
                break
        xy1 = env.data.xpos[env._chassis_bid, :2].copy()
        disp = (xy1 - xy0) if xy0 is not None else np.zeros(2)
        t_meas = max((min(env._step_i, n_end) - k_go) * env.dt, 1e-6)
        s_cmd = math.hypot(vx, vy)
        along = ((disp[0] * vx + disp[1] * vy) / s_cmd
                 if s_cmd > 1e-6 else 0.0)
        cross = (abs(vx * disp[1] - vy * disp[0]) / s_cmd
                 if s_cmd > 1e-6 else float(np.hypot(*disp)))
        results.append({
            "cmd": name, "vx": vx, "vy": vy, "obs_cmd": obs_cmd,
            "disp_x_m": round(float(disp[0]), 4),
            "disp_y_m": round(float(disp[1]), 4),
            "along_m": round(float(along), 4),
            "cross_m": round(float(cross), 4),
            "speed_ratio": round(float(along / (s_cmd * t_meas)), 3)
            if s_cmd > 1e-6 else None,
            "meas_s": round(t_meas, 1),
            "terminated": term_reason,
        })
    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt", type=Path)
    ap.add_argument("--cfg-set", action="append", default=[])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mode", default="both",
                    choices=["sweep", "response", "both"])
    ap.add_argument("--obs-cmd", default="normal",
                    help="comma list of controls for response mode: "
                         "normal,zero,rot90")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    env = make_env(args.cfg_set, args.seed)
    from .gru_policy import load_checkpoint_auto, wrap_recurrent_predictor
    model = load_checkpoint_auto(args.ckpt, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} vs env "
        f"{env.observation_space.shape} — pass the run's own cfg stack")
    model = wrap_recurrent_predictor(model)

    report = {"ckpt": str(args.ckpt), "seed": args.seed,
              "recurrent": hasattr(model, "_state")}
    if args.mode in ("sweep", "both"):
        report["sweep"] = run_sweep(env, model, None)
    if args.mode in ("response", "both"):
        report["response"] = []
        for oc in args.obs_cmd.split(","):
            report["response"] += run_response(env, model, oc.strip(),
                                               None)
    out = args.out or (_PROTO / "logs" / "manual_drive" /
                       f"cmd_sensitivity_{args.ckpt.stem}_"
                       f"{time.strftime('%H%M%S')}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print(f"[probe_cmd_sensitivity] wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
