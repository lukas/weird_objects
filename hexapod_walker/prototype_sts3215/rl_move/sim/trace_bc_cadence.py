"""Per-tick BC-anchor cadence trace (measurement only, no training).

WHAT THIS ANSWERS (phasedir9 dig-in, 08-22): every phasedir9 seed shows
train/bc_anchor_loss_walk ~converged (1e-4..7e-4) yet realized
swing_s_mean ~30% longer than the teacher/clone (0.34-0.36 s vs
0.25-0.27 s). Is the policy's CYCLE RATE actually slower than its
bc_target's (supervision/clock mismatch), or does it cycle at the same
rate with a longer AIRBORNE FRACTION per cycle (contact-timing gap the
action-space MSE cannot price)? Three legs, same env, DR-0 det:

  1. a policy checkpoint rollout      (records bc_target + realized)
  2. optionally a second checkpoint   (e.g. the raw BC clone control)
  3. a scripted RAW TEACHER rollout   (un-phase-locked TripodGait on
                                       the wall clock, knee_abs dialect)

Per rollout it reports, per leg family:
  - joint-space cycle period of the realized coxa/yaw joints
    (rising-zero-crossing intervals, mean-removed)
  - same period for the bc_target signal and for the policy's action
  - contact-based metrics with the harness definition: swing duration,
    duty, lift-to-lift cycle period, stride, swing count
  - the phase-obs clock period actually observed (d(phase)/dt)
  - action-vs-bc_target MSE + per-joint-family |err| and best xcorr lag

Usage (on the run's pod, from the PROTO dir):
  python3 -m rl_move.sim.trace_bc_cadence rl_move/sim/policies/X.zip \
      [--control rl_move/sim/policies/CLONE.zip] --scripted-teacher \
      --episodes 2 --episode-seconds 15 --seed 0 \
      --cfg-set goal.walk_phase_obs=1 ... --out logs/ckpt_eval/trace_x

Measurement-only: builds the same env the eval harness builds, never
touches training code or shared defaults.
"""
from __future__ import annotations

import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (_PROTO, _PROTO / "linux_control"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .servo_model import SimServoParams          # noqa: E402
from .walk_task import SimHexapodJointWalkEnv    # noqa: E402
from .joint_task import q_rad_to_action          # noqa: E402

DEG2RAD = math.pi / 180.0
COXA_IDX = [0, 3, 6, 9, 12, 15]      # per-leg yaw joints
FEMUR_IDX = [1, 4, 7, 10, 13, 16]
TIBIA_IDX = [2, 5, 8, 11, 14, 17]
CONTACT_N = 0.5


def _zero_cross_period(sig: np.ndarray, dt: float) -> float | None:
    """Median interval between rising zero crossings of a mean-removed
    1-D signal, seconds. None when < 3 crossings (no cycling)."""
    s = np.asarray(sig, dtype=float)
    if len(s) < 8:
        return None
    s = s - s.mean()
    ris = np.where((s[:-1] < 0) & (s[1:] >= 0))[0]
    if len(ris) < 3:
        return None
    return float(np.median(np.diff(ris)) * dt)


def _periods_over_legs(mat: np.ndarray, idx: list[int],
                       dt: float) -> list[float]:
    out = []
    for j in idx:
        p = _zero_cross_period(mat[:, j], dt)
        if p is not None:
            out.append(p)
    return out


def _contact_metrics(contact: np.ndarray, pad_xy: np.ndarray,
                     dt: float) -> dict:
    duty, swing_s, cyc_s, strides, counts = [], [], [], [], []
    for f in range(6):
        c = contact[:, f]
        duty.append(float(c.mean()))
        d = np.diff(c.astype(int))
        lifts = np.where(d == -1)[0]
        downs = np.where(d == 1)[0]
        counts.append(int(len(lifts)))
        if len(lifts) >= 3:
            cyc_s.extend(np.diff(lifts) * dt)
        for lo in lifts:
            hi_c = downs[downs > lo]
            if len(hi_c):
                hi = int(hi_c[0])
                swing_s.append((hi - lo) * dt)
                strides.append(float(np.linalg.norm(
                    pad_xy[hi, f] - pad_xy[lo, f])))
    return {
        "duty_mean": round(float(np.mean(duty)), 3),
        "duty_per_leg": [round(x, 2) for x in duty],
        "swing_count_per_leg": counts,
        "swing_s_mean": (round(float(np.mean(swing_s)), 3)
                         if swing_s else None),
        "cycle_s_lift2lift_med": (round(float(np.median(cyc_s)), 3)
                                  if cyc_s else None),
        "stride_m_mean": (round(float(np.mean(strides)), 3)
                          if strides else None),
    }


class _ScriptedTeacher:
    """Raw un-phase-locked TripodGait driven by the wall clock and the
    live blended command (knee_abs dialect = the phase-clone lineage's
    own action dialect; matches _make_walk_bc_gait knee_abs=1)."""

    def __init__(self, env):
        from tripod_gait import TripodGait
        self.env = env
        self.gait = TripodGait(vx=0.0)
        self.gait.sync_plant_stance(20.0, 80.0)
        self.gait.reset_phase()

    def act(self) -> np.ndarray:
        env = self.env
        goal = env._current_goal()
        if goal is not None:
            self.gait.set_velocity(
                vx=float(goal.vx_ref), vy=float(goal.vy_ref),
                omega=float(getattr(goal, "wz_ref", 0.0) or 0.0))
        else:
            self.gait.set_velocity(vx=0.0, vy=0.0, omega=0.0)
        t = env._step_i * env.dt          # WALL clock, never gated
        q_deg = np.asarray(self.gait.desired_deg(t))
        return q_rad_to_action(q_deg * DEG2RAD).astype(np.float32)


def _rollout(env, actor, deterministic: bool, max_ticks: int) -> dict:
    obs, _ = env.reset()
    rec = {k: [] for k in ("t", "phase", "q", "act", "bc", "bc_mask",
                           "contact", "pad_xy", "cmd")}
    term = trunc = False
    ticks = 0
    while not (term or trunc) and ticks < max_ticks:
        if hasattr(actor, "predict"):
            action, _ = actor.predict(obs, deterministic=deterministic)
        else:
            action = actor.act()
        obs, _r, term, trunc, info = env.step(action)
        goal = env._current_goal()
        s_ref = (float(np.hypot(goal.vx_ref, goal.vy_ref))
                 if goal is not None else 0.0)
        rec["t"].append(env._step_i * env.dt)
        rec["phase"].append(float(getattr(env, "_phase", 0.0)))
        rec["q"].append(np.asarray(
            env._state.joint_position, dtype=float).copy())
        rec["act"].append(np.asarray(action, dtype=float).copy())
        bc = info.get("bc_target")
        rec["bc"].append(np.zeros(18) if bc is None
                         else np.asarray(bc, dtype=float))
        rec["bc_mask"].append(bc is not None)
        rec["contact"].append([
            float(env.data.sensordata[adr]) > CONTACT_N
            for adr in env._touch_adr])
        rec["pad_xy"].append(
            env.data.xpos[env._pad_bids, :2].copy())
        rec["cmd"].append(s_ref)
        ticks += 1
    out = {k: np.asarray(v) for k, v in rec.items()}
    out["terminated"] = bool(term)
    return out


def _analyze(tag: str, roll: dict, dt: float) -> dict:
    mask = roll["bc_mask"].astype(bool)
    cmd = roll["cmd"] > 1e-3
    q = roll["q"]
    act = roll["act"]
    bc = roll["bc"]
    # phase-obs clock period from unwrapped phase on commanded ticks
    ph = np.unwrap(roll["phase"])
    dph = np.diff(ph)[cmd[1:]]
    phase_period = (round(float(2 * math.pi * dt / np.median(dph)), 3)
                    if len(dph) and np.median(dph) > 1e-6 else None)
    res = {
        "tag": tag,
        "ticks": int(len(q)),
        "terminated": roll["terminated"],
        "commanded_frac": round(float(cmd.mean()), 3),
        "bc_tick_frac": round(float(mask.mean()), 3),
        "phase_obs_period_s": phase_period,
        "coxa_period_s": {
            "realized_q": _periods_over_legs(q, COXA_IDX, dt),
            "action": _periods_over_legs(act, COXA_IDX, dt),
            "bc_target": (_periods_over_legs(bc[mask], COXA_IDX, dt)
                          if mask.sum() > 20 else []),
        },
        "contact": _contact_metrics(
            roll["contact"].astype(bool), roll["pad_xy"], dt),
    }
    if mask.sum() > 20:
        # action-space anchor error on bc ticks (eval-regime analog of
        # train/bc_anchor_loss_walk)
        err = act[mask] - bc[mask]
        res["anchor_mse_action"] = round(float((err ** 2).mean()), 6)
        res["anchor_abs_err"] = {
            "coxa": round(float(np.abs(err[:, COXA_IDX]).mean()), 4),
            "femur": round(float(np.abs(err[:, FEMUR_IDX]).mean()), 4),
            "tibia": round(float(np.abs(err[:, TIBIA_IDX]).mean()), 4),
        }
        # best xcorr lag (ticks) of realized q vs bc_target, coxa 0,
        # converted to action space for scale invariance
        qa = q_rad_to_action(q[mask])
        lags = []
        for j in COXA_IDX:
            a = qa[:, j] - qa[:, j].mean()
            b = bc[mask][:, j] - bc[mask][:, j].mean()
            if a.std() < 1e-6 or b.std() < 1e-6:
                continue
            n = len(a)
            best, best_l = -2.0, 0
            for lag in range(0, min(40, n // 3)):
                c = float(np.dot(a[lag:], b[:n - lag])
                          / (n - lag) / (a.std() * b.std()))
                if c > best:
                    best, best_l = c, lag
            lags.append((best_l, round(best, 3)))
        res["realized_vs_bc_lag_ticks"] = lags
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--control", type=Path, default=None,
                    help="second checkpoint (e.g. the BC clone)")
    ap.add_argument("--scripted-teacher", action="store_true")
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--stochastic", action="store_true")
    ap.add_argument("--cfg-set", action="append", default=None)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    from .train_ppo_sim import _parse_cfg_set
    cfg_kw = {}
    if args.cfg_set:
        from rl_move.config import load_config
        cfg = load_config()
        for key, parsed in _parse_cfg_set(args.cfg_set).items():
            sect, name = key.split(".", 1)
            cfg.setdefault(sect, {})[name] = parsed
        cfg_kw["cfg"] = cfg

    def make_env():
        env = SimHexapodJointWalkEnv(
            params=SimServoParams.from_cfg(cfg_kw.get("cfg")),
            randomize=False, dr_scale=0.0,
            episode_seconds=args.episode_seconds, seed=args.seed,
            render_mode=None, **cfg_kw)
        gen = env._goal_gen
        for m in ("hold", "lean", "track", "unload", "raise", "rise",
                  "lower", "walk", "quad", "getup", "quadwalk",
                  "recover"):
            if hasattr(gen, f"p_{m}"):
                setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
        return env

    from .gru_policy import load_checkpoint_auto
    actors = [("policy", load_checkpoint_auto(args.checkpoint,
                                              device="cpu"))]
    if args.control is not None:
        actors.append(("control", load_checkpoint_auto(args.control,
                                                       device="cpu")))
    max_ticks = int(args.episode_seconds * 25) + 50
    results = []
    for tag, actor in actors:
        for k in range(args.episodes):
            env = make_env()
            env.reset(seed=args.seed + k)
            r = _rollout(env, actor, not args.stochastic, max_ticks)
            results.append(_analyze(f"{tag}_ep{k}", r, env.dt))
            env.close()
    if args.scripted_teacher:
        for k in range(args.episodes):
            env = make_env()
            env.reset(seed=args.seed + k)
            r = _rollout(env, _ScriptedTeacher(env), True, max_ticks)
            results.append(_analyze(f"teacher_raw_ep{k}", r, env.dt))
            env.close()

    args.out.mkdir(parents=True, exist_ok=True)
    with open(args.out / "trace.json", "w") as f:
        json.dump({"checkpoint": str(args.checkpoint),
                   "control": str(args.control),
                   "cfg_set": args.cfg_set, "seed": args.seed,
                   "results": results}, f, indent=1)
    for r in results:
        cp = r["coxa_period_s"]
        med = (lambda v: round(float(np.median(v)), 3) if v else None)
        print(f"[{r['tag']}] ticks={r['ticks']} term={r['terminated']} "
              f"phase_T={r['phase_obs_period_s']} "
              f"coxaT q={med(cp['realized_q'])} act={med(cp['action'])} "
              f"bc={med(cp['bc_target'])} | "
              f"swing={r['contact']['swing_s_mean']} "
              f"cycT={r['contact']['cycle_s_lift2lift_med']} "
              f"duty={r['contact']['duty_mean']} "
              f"stride={r['contact']['stride_m_mean']} "
              f"mse={r.get('anchor_mse_action')} "
              f"lag={r.get('realized_vs_bc_lag_ticks')}")
    print(f"TRACE-DONE artifacts: {args.out}")


if __name__ == "__main__":
    main()
