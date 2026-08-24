"""Berkeley-style low-dimensional gait search for the MuJoCo hexapod.

The Yang/Wang/Calandra/Contreras/Levine/Pister microrobot paper trains
walking by optimizing a small CPG controller, not by training a neural
policy. This runner ports that idea to our current sim stack:

* controller: SE2FootGait via sim_gait_compat, so knees are in the
  MuJoCo-relative frame at the one sim boundary;
* search space: a small set of gait parameters instead of 18 joint
  targets or PPO weights;
* objective: the same measured rollout metrics we care about at eval
  time: commanded progress, cross-track error, loaded-foot slip, falls,
  height/tilt, and effort.

Use a short straight-ahead run first, then widen to contextual headings:

    uv run python -m rl_move.sim.paper_cpg_search --iterations 50 --suite straight
    uv run python -m rl_move.sim.paper_cpg_search --iterations 250 --suite contextual
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.verify_noslip import (  # noqa: E402
    HOLD_S, PLANT_HIP_DEG, PLANT_KNEE_DEG, _make_env,
)


@dataclass(frozen=True)
class Command:
    name: str
    vx: float
    vy: float
    wz: float = 0.0


@dataclass(frozen=True)
class GaitParams:
    gait: str
    period: float
    swing_frac: float
    lift_m: float
    cmd_tau: float
    workspace_margin: float


_BOUNDS = {
    "tetrapod": {
        "period": (2.0, 12.0),
        "swing_frac": (0.10, 0.30),
        "lift_m": (0.012, 0.035),
        "cmd_tau": (0.10, 0.90),
        "workspace_margin": (0.72, 1.00),
    },
    "wave": {
        "period": (5.0, 24.0),
        "swing_frac": (0.05, 0.15),
        "lift_m": (0.010, 0.032),
        "cmd_tau": (0.10, 1.20),
        "workspace_margin": (0.72, 1.00),
    },
}


def command_suite(name: str, speed: float, wz: float) -> list[Command]:
    """Small fixed command panels, from straight to contextual headings."""
    s = float(speed)
    if name == "straight":
        return [Command("fwd", s, 0.0, 0.0)]
    if name == "front3":
        angles = (0.0, math.radians(25.0), math.radians(-25.0))
    elif name == "contextual":
        angles = (
            0.0, math.radians(35.0), math.radians(-35.0),
            math.radians(70.0), math.radians(-70.0),
        )
    elif name == "omni":
        angles = tuple(math.radians(a) for a in (0, 45, -45, 90, -90, 180))
    else:
        raise ValueError(f"unknown suite {name!r}")
    out = [
        Command(f"head{int(round(math.degrees(a))):+03d}",
                s * math.cos(a), s * math.sin(a), 0.0)
        for a in angles
    ]
    if abs(wz) > 1e-6 and name in ("contextual", "omni"):
        out.extend([
            Command("turn_left", 0.0, 0.0, abs(wz)),
            Command("turn_right", 0.0, 0.0, -abs(wz)),
        ])
    return out


def _sample_params(rng: np.random.Generator, gait: str | None = None
                   ) -> GaitParams:
    gait = gait or str(rng.choice(("tetrapod", "wave"), p=(0.7, 0.3)))
    b = _BOUNDS[gait]

    def u(key: str) -> float:
        lo, hi = b[key]
        return float(rng.uniform(lo, hi))

    return GaitParams(
        gait=gait,
        period=u("period"),
        swing_frac=u("swing_frac"),
        lift_m=u("lift_m"),
        cmd_tau=u("cmd_tau"),
        workspace_margin=u("workspace_margin"),
    )


def _clip_params(p: GaitParams) -> GaitParams:
    b = _BOUNDS[p.gait]

    def c(key: str, value: float) -> float:
        lo, hi = b[key]
        return float(np.clip(value, lo, hi))

    return GaitParams(
        gait=p.gait,
        period=c("period", p.period),
        swing_frac=c("swing_frac", p.swing_frac),
        lift_m=c("lift_m", p.lift_m),
        cmd_tau=c("cmd_tau", p.cmd_tau),
        workspace_margin=c("workspace_margin", p.workspace_margin),
    )


def _jitter_params(p: GaitParams, rng: np.random.Generator,
                   frac: float) -> GaitParams:
    b = _BOUNDS[p.gait]

    def j(key: str, value: float) -> float:
        lo, hi = b[key]
        return float(value + rng.normal(0.0, frac * (hi - lo)))

    gait = p.gait
    if rng.random() < 0.08:
        gait = "wave" if gait == "tetrapod" else "tetrapod"
        return _sample_params(rng, gait=gait)
    return _clip_params(GaitParams(
        gait=gait,
        period=j("period", p.period),
        swing_frac=j("swing_frac", p.swing_frac),
        lift_m=j("lift_m", p.lift_m),
        cmd_tau=j("cmd_tau", p.cmd_tau),
        workspace_margin=j("workspace_margin", p.workspace_margin),
    ))


def _vectorize(p: GaitParams) -> list[float]:
    b = _BOUNDS[p.gait]

    def n(key: str, value: float) -> float:
        lo, hi = b[key]
        return (float(value) - lo) / max(hi - lo, 1e-9)

    return [
        0.0 if p.gait == "tetrapod" else 1.0,
        n("period", p.period),
        n("swing_frac", p.swing_frac),
        n("lift_m", p.lift_m),
        n("cmd_tau", p.cmd_tau),
        n("workspace_margin", p.workspace_margin),
    ]


def _normal_pdf(z: np.ndarray) -> np.ndarray:
    return np.exp(-0.5 * z * z) / math.sqrt(2.0 * math.pi)


def _normal_cdf(z: np.ndarray) -> np.ndarray:
    return 0.5 * (1.0 + np.vectorize(math.erf)(z / math.sqrt(2.0)))


def _matern52(xa: np.ndarray, xb: np.ndarray,
              length_scale: float = 0.35) -> np.ndarray:
    """Small Matern-5/2 kernel for dependency-free BO."""
    d = xa[:, None, :] - xb[None, :, :]
    r = np.sqrt(np.sum((d / max(length_scale, 1e-9)) ** 2, axis=2))
    sr5 = math.sqrt(5.0) * r
    return (1.0 + sr5 + 5.0 * r * r / 3.0) * np.exp(-sr5)


def _gp_ei_numpy(history: list[dict], pool: list[GaitParams]) -> GaitParams:
    """Dependency-free GP expected improvement over a candidate pool."""
    x = np.asarray([_vectorize(GaitParams(**r["params"]))
                    for r in history], dtype=float)
    y_raw = np.asarray([float(r["score"]) for r in history], dtype=float)
    y_mean = float(np.mean(y_raw))
    y_std = float(np.std(y_raw)) or 1.0
    y = (y_raw - y_mean) / y_std
    k = _matern52(x, x) + np.eye(len(x)) * 1e-5
    try:
        l = np.linalg.cholesky(k)
    except np.linalg.LinAlgError:
        k = k + np.eye(len(x)) * 1e-3
        l = np.linalg.cholesky(k)
    alpha = np.linalg.solve(l.T, np.linalg.solve(l, y))
    xp = np.asarray([_vectorize(p) for p in pool], dtype=float)
    ks = _matern52(x, xp)
    mu = ks.T @ alpha
    v = np.linalg.solve(l, ks)
    var = np.maximum(1.0 - np.sum(v * v, axis=0), 1e-9)
    std = np.sqrt(var)
    best = float(np.max(y))
    z = (mu - best - 0.01) / std
    ei = (mu - best - 0.01) * _normal_cdf(z) + std * _normal_pdf(z)
    return pool[int(np.argmax(ei))]


def propose_next(history: list[dict], rng: np.random.Generator, *,
                 init_random: int, pool_size: int,
                 optimizer: str) -> tuple[GaitParams, str]:
    """Suggest the next candidate.

    Uses dependency-free Gaussian-process expected improvement after
    the initial random design, then falls back to elite jitter only if
    the GP solve becomes ill-conditioned.
    """
    if len(history) < init_random or optimizer == "random":
        return _sample_params(rng), "random"
    scores = np.asarray([float(r["score"]) for r in history], dtype=float)
    elites = [
        GaitParams(**history[i]["params"])
        for i in np.argsort(scores)[-max(3, min(8, len(history))):]
    ]
    if optimizer in ("auto", "gp"):
        pool = [_sample_params(rng) for _ in range(pool_size)]
        # Bias half the acquisition pool around known good gaits.
        for _ in range(pool_size // 2):
            pool.append(_jitter_params(
                elites[int(rng.integers(0, len(elites)))],
                rng, frac=0.10))
        try:
            return _gp_ei_numpy(history, pool), "gp_ei_numpy"
        except Exception as exc:
            if optimizer == "gp":
                raise
            method = f"elite_jitter:{exc.__class__.__name__}"
    else:
        method = "elite_jitter"
    frac = max(0.04, 0.18 * (0.97 ** len(history)))
    return _jitter_params(elites[int(rng.integers(0, len(elites)))],
                          rng, frac=frac), method


def _body_yaw(env) -> float:
    r = env.data.xmat[env._chassis_bid].reshape(3, 3)
    return math.atan2(float(r[1, 0]), float(r[0, 0]))


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _body_command_dir_world(env, vx: float, vy: float) -> np.ndarray:
    r = env.data.xmat[env._chassis_bid].reshape(3, 3)
    v = r @ np.asarray([vx, vy, 0.0], dtype=float)
    xy = np.asarray(v[:2], dtype=float)
    n = float(np.linalg.norm(xy))
    return xy / max(n, 1e-9)


def rollout(env, params: GaitParams, command: Command, *,
            walk_s: float, seed: int) -> dict:
    """Run one gait/command in MuJoCo and return eval-aligned metrics."""
    from sim_gait_compat import SE2FootGait

    try:
        env.reset(seed=seed)
    except TypeError:
        env.reset()

    gait = SE2FootGait(
        gait=params.gait,
        period=params.period,
        swing_frac=params.swing_frac,
        lift=params.lift_m,
        cmd_tau=params.cmd_tau,
        workspace_margin=params.workspace_margin,
    )
    gait.sync_plant_stance(PLANT_HIP_DEG, PLANT_KNEE_DEG)
    gait.reset_phase(t=0.0)
    gait.set_velocity(vx=command.vx, vy=command.vy, omega=command.wz)

    plant_rad = np.asarray(
        [0.0, PLANT_HIP_DEG, PLANT_KNEE_DEG] * 6, dtype=float) * DEG2RAD
    hold_steps = int(round(HOLD_S / env.dt))
    walk_steps = int(round(walk_s / env.dt))

    xy0 = None
    yaw0 = None
    yaw_prev = None
    yaw_cum = 0.0
    u_world = None
    pad_prev = None
    touch_prev = None
    slip_m = 0.0
    currents = []
    heights = []
    rolls = []
    pitches = []
    terminated = ""
    steps = 0

    for step in range(hold_steps + walk_steps):
        walking = step >= hold_steps
        if walking:
            t = (step - hold_steps) * env.dt
            q_rad = np.asarray(gait.desired_deg(t), dtype=float) * DEG2RAD
        else:
            q_rad = plant_rad
        _obs, _r, term, trunc, info = env.step(q_rad_to_action(q_rad))
        steps += 1
        if walking:
            if xy0 is None:
                xy0 = env.data.xpos[env._chassis_bid, :2].copy()
                yaw0 = _body_yaw(env)
                yaw_prev = yaw0
                if math.hypot(command.vx, command.vy) > 1e-6:
                    u_world = _body_command_dir_world(
                        env, command.vx, command.vy)
            else:
                # Unwrapped cumulative yaw: per-step deltas are far
                # below pi, so summing wrapped increments never aliases.
                # (A 20 s turn at 0.2 rad/s targets 4 rad > pi; wrapping
                # the endpoint difference reported near-perfect turns as
                # sign-inverted.)
                yaw_now = _body_yaw(env)
                yaw_cum += _wrap_angle(yaw_now - yaw_prev)
                yaw_prev = yaw_now
            touch = np.asarray([
                float(env.data.sensordata[a]) > 0.5
                for a in env._touch_adr
            ], dtype=bool)
            pad_now = env.data.xpos[env._pad_bids, :2].copy()
            if pad_prev is not None and touch_prev is not None:
                moved = np.linalg.norm(pad_now - pad_prev, axis=1)
                slip_m += float(moved[touch_prev].sum())
            pad_prev = pad_now
            touch_prev = touch
            st = env._state
            heights.append(float(env.data.xpos[env._chassis_bid, 2]))
            rolls.append(abs(float(st.imu_roll)) * 180.0 / math.pi)
            pitches.append(abs(float(st.imu_pitch)) * 180.0 / math.pi)
            if st.servo_current is not None:
                currents.append(np.asarray(st.servo_current, dtype=float))
        if term or trunc:
            terminated = str(info.get("termination_reason") or "trunc")
            break

    xy1 = env.data.xpos[env._chassis_bid, :2].copy()
    delta = np.zeros(2) if xy0 is None else xy1 - xy0
    speed_cmd = math.hypot(command.vx, command.vy)
    cmd_dist = speed_cmd * walk_s
    progress = 0.0
    cross = float(np.linalg.norm(delta))
    if u_world is not None:
        progress = float(np.dot(delta, u_world))
        cross = abs(float(u_world[0] * delta[1] - u_world[1] * delta[0]))

    yaw_delta = 0.0 if yaw0 is None else yaw_cum
    yaw_target = command.wz * walk_s
    yaw_along = (yaw_delta * np.sign(command.wz)
                 if abs(command.wz) > 1e-6 else 0.0)
    cur = np.vstack(currents) if currents else np.zeros((1, 18))

    # Normalize slip by the commanded-equivalent path: translation
    # progress plus the foot-arc length of the achieved rotation
    # (mean stance foot radius ~0.17 m). Pure-turn commands otherwise
    # divide by ~zero translation and saturate the slip penalty no
    # matter how clean the pivot is.
    slip_den = max(progress + 0.17 * abs(yaw_delta), 0.05)
    slip_per_m = slip_m / slip_den
    return {
        "command": asdict(command),
        "steps": steps,
        "terminated": terminated,
        "cmd_dist_m": cmd_dist,
        "progress_m": progress,
        "progress_frac": progress / cmd_dist if cmd_dist > 1e-6 else None,
        "cross_m": cross,
        "cross_frac": cross / max(cmd_dist, 0.05),
        "slip_m": slip_m,
        "slip_per_m": slip_per_m,
        "speed_m_s": progress / max(walk_s, 1e-9),
        "yaw_delta_rad": yaw_delta,
        "yaw_target_rad": yaw_target,
        "yaw_along_frac": (
            yaw_along / abs(yaw_target) if abs(yaw_target) > 1e-6 else None),
        "yaw_err_rad": (
            abs(yaw_delta - yaw_target)
            if abs(yaw_target) > 1e-6 else None),
        "height_mean_m": float(np.mean(heights)) if heights else None,
        "roll_peak_deg": float(max(rolls)) if rolls else 0.0,
        "pitch_peak_deg": float(max(pitches)) if pitches else 0.0,
        "current_mean_a": float(np.mean(cur)),
        "current_p95_a": float(np.percentile(cur, 95)),
        "command_scale": float(gait.last_command_scale),
        "ik_failures": int(gait.ik_failures),
        "limit_clips": int(gait.limit_clips),
        "support_margin_min_m": (
            None if gait.min_support_margin == float("inf")
            else float(gait.min_support_margin)),
    }


def rollout_score(row: dict, *, slip_weight: float) -> float:
    """Eval-aligned scalar for one rollout; higher is better."""
    progress_frac = row.get("progress_frac")
    if progress_frac is None:
        progress_score = 0.0
    else:
        progress_score = min(float(progress_frac), 1.25)
    yaw_frac = row.get("yaw_along_frac")
    if yaw_frac is not None:
        progress_score += 0.5 * min(max(float(yaw_frac), -1.0), 1.25)

    cross_pen = 0.75 * min(float(row.get("cross_frac") or 0.0), 3.0)
    slip_pen = slip_weight * min(float(row.get("slip_per_m") or 0.0), 5.0)
    tilt_pen = 0.025 * max(float(row.get("roll_peak_deg") or 0.0),
                           float(row.get("pitch_peak_deg") or 0.0))
    effort_pen = 0.015 * float(row.get("current_p95_a") or 0.0)
    yaw_err = row.get("yaw_err_rad")
    yaw_pen = 0.0 if yaw_err is None else 0.5 * min(abs(float(yaw_err)), 3.0)
    term_pen = 5.0 if row.get("terminated") else 0.0
    ik_pen = 0.1 * (int(row.get("ik_failures") or 0)
                    + int(row.get("limit_clips") or 0))
    return progress_score - cross_pen - slip_pen - tilt_pen - effort_pen \
        - yaw_pen - term_pen - ik_pen


def score_rollouts(rows: Iterable[dict], *, slip_weight: float = 0.70
                   ) -> dict:
    rows = list(rows)
    scores = [rollout_score(r, slip_weight=slip_weight) for r in rows]

    def mean_key(key: str):
        vals = [r.get(key) for r in rows if r.get(key) is not None]
        return float(np.mean(vals)) if vals else None

    return {
        "score": float(np.mean(scores)) if scores else float("-inf"),
        "score_min": float(np.min(scores)) if scores else float("-inf"),
        "progress_frac_mean": mean_key("progress_frac"),
        "cross_frac_mean": mean_key("cross_frac"),
        "slip_per_m_mean": mean_key("slip_per_m"),
        "falls": int(sum(1 for r in rows if r.get("terminated"))),
        "speed_m_s_mean": mean_key("speed_m_s"),
        "current_p95_a_mean": mean_key("current_p95_a"),
        "command_scale_min": (
            float(min(r["command_scale"] for r in rows))
            if rows and all("command_scale" in r for r in rows) else None),
    }


def evaluate_candidate(env, params: GaitParams, commands: list[Command], *,
                       walk_s: float, seed_base: int,
                       slip_weight: float) -> dict:
    rows = [
        rollout(env, params, c, walk_s=walk_s,
                seed=seed_base + 1009 * i)
        for i, c in enumerate(commands)
    ]
    summary = score_rollouts(rows, slip_weight=slip_weight)
    return {
        "params": asdict(params),
        "score": summary["score"],
        "summary": summary,
        "rollouts": rows,
    }


def evaluate_candidate_panel(envs: list[tuple[float, object]],
                             params: GaitParams, commands: list[Command], *,
                             walk_s: float, seed_base: int,
                             slip_weight: float) -> dict:
    """Robustness-panel scoring: mean candidate score across friction envs.

    Motivated by the 08-23 eval_cpg_gate result: the contextual-250
    winner passes DR-0/loaded/mu1.2 but overshoots turns ~33% at mu0.8
    (open-loop yaw scale is friction-dependent). Scoring each candidate
    across the same friction panel puts that failure axis INTO the
    search objective — reward stays == eval.
    """
    per_mu = {}
    for mu, env in envs:
        rec = evaluate_candidate(env, params, commands, walk_s=walk_s,
                                 seed_base=seed_base,
                                 slip_weight=slip_weight)
        per_mu[f"mu{mu:g}"] = rec
    scores = [r["score"] for r in per_mu.values()]
    return {
        "params": asdict(params),
        "score": float(np.mean(scores)),
        "score_worst_mu": float(np.min(scores)),
        "summary": {
            "score": float(np.mean(scores)),
            "per_mu": {k: v["summary"] for k, v in per_mu.items()},
            "falls": int(sum(v["summary"]["falls"]
                             for v in per_mu.values())),
            "progress_frac_mean": float(np.mean([
                v["summary"]["progress_frac_mean"]
                for v in per_mu.values()
                if v["summary"]["progress_frac_mean"] is not None])),
            "slip_per_m_mean": float(np.mean([
                v["summary"]["slip_per_m_mean"]
                for v in per_mu.values()
                if v["summary"]["slip_per_m_mean"] is not None])),
        },
        "rollouts_per_mu": {k: v["rollouts"] for k, v in per_mu.items()},
    }


def _write_report(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-name", default="paper-cpg-search")
    ap.add_argument("--iterations", type=int, default=50)
    ap.add_argument("--init-random", type=int, default=10)
    ap.add_argument("--optimizer", choices=("auto", "gp", "jitter", "random"),
                    default="auto")
    ap.add_argument("--pool-size", type=int, default=256)
    ap.add_argument("--suite", choices=("straight", "front3", "contextual",
                                        "omni"),
                    default="straight")
    ap.add_argument("--speed", type=float, default=0.04)
    ap.add_argument("--wz", type=float, default=0.20)
    ap.add_argument("--walk-s", type=float, default=20.0,
                    help="measured walking seconds; 20s matches the "
                         "referenced repo's normal.py 400 x 50ms trials")
    ap.add_argument("--slip-weight", type=float, default=0.70)
    ap.add_argument("--seed", type=int, default=20260822)
    ap.add_argument("--mu", type=float, default=0.0)
    ap.add_argument("--mu-list", default="",
                    help="comma-separated friction panel (0 = XML "
                         "default); scores each candidate as the MEAN "
                         "over one env per mu. Empty = single --mu env "
                         "(bit-exact legacy behavior)")
    ap.add_argument("--servo-params", default="",
                    help="'' = air fit, 'loaded' = loaded bench fit")
    ap.add_argument("--write-speed", type=int, default=1500)
    ap.add_argument("--write-acc", type=int, default=80)
    ap.add_argument("--vel-max", type=float, default=None,
                    help="servo cruise ceiling deg/s; default matches "
                         "write speed, 0 keeps fitted clamp")
    ap.add_argument("--warm-json", default="",
                    help="JSON list of GaitParams dicts evaluated as the "
                         "first trials (warm-starts the GP history, e.g. "
                         "with the straight-suite winner)")
    ap.add_argument("--replay-json", default="",
                    help="JSON GaitParams dict; skips the search and "
                         "re-evaluates these exact params on "
                         "--replay-seeds held-out seed bases")
    ap.add_argument("--replay-seeds", type=int, default=5)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)

    rng = np.random.default_rng(args.seed)
    commands = command_suite(args.suite, args.speed, args.wz)
    out = args.out or (
        ROOT / "logs" / "paper_cpg_search" / f"{args.run_name}.json")
    report = {
        "run_name": args.run_name,
        "created_unix": time.time(),
        "args": vars(args) | {"out": str(out)},
        "commands": [asdict(c) for c in commands],
        "history": [],
        "best": None,
    }

    mu_list = ([float(x) for x in args.mu_list.split(",") if x.strip()]
               if args.mu_list else [])

    def _env(mu: float):
        return _make_env(
            mu, args.servo_params, args.seed,
            episode_s=HOLD_S + args.walk_s + 1.0,
            render=False,
            write_speed=args.write_speed,
            write_acc=args.write_acc,
            vel_max_deg_s=args.vel_max,
        )

    if mu_list:
        panel = [(mu, _env(mu)) for mu in mu_list]
        env = panel[0][1]

        def _eval(params, seed_base):
            return evaluate_candidate_panel(
                panel, params, commands, walk_s=args.walk_s,
                seed_base=seed_base, slip_weight=args.slip_weight)
    else:
        env = _env(args.mu)
        panel = [(args.mu, env)]

        def _eval(params, seed_base):
            return evaluate_candidate(
                env, params, commands, walk_s=args.walk_s,
                seed_base=seed_base, slip_weight=args.slip_weight)

    try:
        if args.replay_json:
            # Held-out verification of one exact parameter set: the
            # search scored each trial on a single seed base, so a
            # winner must reproduce on fresh seeds before promotion.
            params = GaitParams(**json.loads(args.replay_json))
            report["replay_params"] = asdict(params)
            for k in range(args.replay_seeds):
                seed_base = args.seed + k * 100_003
                rec = _eval(params, seed_base)
                rec["iteration"] = k + 1
                rec["seed_base"] = seed_base
                rec["proposal_method"] = "replay"
                report["history"].append(rec)
                _write_report(out, report)
                s = rec["summary"]
                print(
                    f"[replay {k + 1:02d}/{args.replay_seeds}] "
                    f"seed_base={seed_base} score={rec['score']:+.3f} "
                    f"prog={s['progress_frac_mean']} "
                    f"slip/m={s['slip_per_m_mean']} falls={s['falls']}",
                    flush=True,
                )
            scores = [r["score"] for r in report["history"]]
            report["best"] = {
                "params": asdict(params),
                "score": float(np.mean(scores)),
                "summary": {
                    "score": float(np.mean(scores)),
                    "score_min": float(np.min(scores)),
                    "score_std": float(np.std(scores)),
                    "progress_frac_mean": float(np.mean([
                        r["summary"]["progress_frac_mean"]
                        for r in report["history"]
                        if r["summary"]["progress_frac_mean"] is not None
                    ])),
                    "slip_per_m_mean": float(np.mean([
                        r["summary"]["slip_per_m_mean"]
                        for r in report["history"]
                        if r["summary"]["slip_per_m_mean"] is not None
                    ])),
                    "falls": int(sum(r["summary"]["falls"]
                                     for r in report["history"])),
                },
            }
            _write_report(out, report)
            print(f"[paper_cpg_search] wrote {out}")
            print("[paper_cpg_search] replay aggregate:")
            print(json.dumps(report["best"]["summary"], indent=2,
                             sort_keys=True))
            return 0
        warm = ([_clip_params(GaitParams(**d))
                 for d in json.loads(args.warm_json)]
                if args.warm_json else [])
        for i in range(args.iterations):
            if i < len(warm):
                params, method = warm[i], "warm"
            else:
                params, method = propose_next(
                    report["history"], rng,
                    init_random=args.init_random,
                    pool_size=args.pool_size,
                    optimizer=args.optimizer,
                )
            rec = _eval(params, args.seed + i * 100_003)
            rec["iteration"] = i + 1
            rec["proposal_method"] = method
            report["history"].append(rec)
            if report["best"] is None or rec["score"] > report["best"]["score"]:
                report["best"] = rec
            _write_report(out, report)
            s = rec["summary"]
            print(
                f"[{i + 1:03d}/{args.iterations}] {method:18s} "
                f"score={rec['score']:+.3f} "
                f"best={report['best']['score']:+.3f} "
                f"prog={s['progress_frac_mean']} "
                f"slip/m={s['slip_per_m_mean']} "
                f"falls={s['falls']} "
                f"{rec['params']}",
                flush=True,
            )
    finally:
        for _mu, e in panel:
            e.close()
    print(f"[paper_cpg_search] wrote {out}")
    if report["best"]:
        print("[paper_cpg_search] best:")
        print(json.dumps({
            "score": report["best"]["score"],
            "params": report["best"]["params"],
            "summary": report["best"]["summary"],
        }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
