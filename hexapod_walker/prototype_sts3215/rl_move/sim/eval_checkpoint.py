"""Exact-path checkpoint evaluation — the exact-path eval harness
(spec: archive/RL_PLAN_NEXT.md §2; current plan: RL_PLAN.md §11).

Loads a checkpoint through the IDENTICAL env/reset path automated eval
uses and runs per-mode deterministic episodes with:

- telemetry-overlay videos (MP4 @ 25 fps) + 8-frame PNG strips
- per-servo current stats: max, p95, time above soft threshold,
  cross-leg imbalance, hottest servo (RL_PLAN_NEXT §4)
- gait metrics for walk episodes: per-foot duty cycle, swing count/
  duration, stride length, slip-while-loaded, forward distance
  (RL_PLAN_NEXT §5)
- rise/lower stats split by start kind (pooled numbers lie)
- optional stochastic pass at the checkpoint's own std (the IK-line
  lesson: a skill that only works deterministically erodes in any
  future fine-tune)

Usage:
  python -m rl_move.sim.eval_checkpoint POLICY.zip --task joint_goal \
      --per-mode 6 --dr-scale 0.2 --seed 0 [--modes rise lower] \
      [--stochastic] [--no-video]

Output: logs/ckpt_eval/<ckpt>_<ts>/ with report.json, videos, strips.
A checkpoint that scores well but LOOKS wrong is a failed checkpoint —
fix the metric before training anything else.
"""
from __future__ import annotations

import os

# Cap math-library thread pools BEFORE numpy/torch import them.
# Default OpenBLAS/OMP pools size to VISIBLE cores (70+ on CoreWeave
# nodes) per process; 20 concurrent gate evals on the controller =
# ~1400 spinning threads, node load 300, and the four trainers
# co-located on g142d86 ran 5x slow (2026-08-09 evening incident).
# Eval is MuJoCo-stepping bound; 2 threads is plenty.
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (_PROTO, _PROTO / "linux_control"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .servo_model import SimServoParams  # noqa: E402
from .goal_task import SimHexapodGoalEnv  # noqa: E402
from .joint_task import SimHexapodJointGoalEnv  # noqa: E402
from .walk_task import SimHexapodJointWalkEnv  # noqa: E402

ENV_CLASSES = {"goal": SimHexapodGoalEnv,
               "joint_goal": SimHexapodJointGoalEnv,
               "joint_walk": SimHexapodJointWalkEnv}

ALL_MODES = ("hold", "lean", "track", "unload", "raise", "rise",
             "lower", "walk")
SOFT_CURRENT_A = 1.5      # sustained above this = hot-servo flag
CONTACT_N = 0.5           # touch-sensor force that counts as contact
FPS = 25


# ---------------------------------------------------------------------------
# per-episode collection


def _start_kind(traj) -> str:
    start_at = getattr(traj, "start_at", "plant")
    if start_at == "crouch":
        return "crouch"
    if getattr(traj, "start_curl", 0.0) > 0:
        return "bridge"
    if start_at == "zero":
        return "flat"
    return "plant"


# End-posture check (operator directive 2026-08-08 ~20:40Z): a rise/lower
# "success" that ends with a leg flagged in the air is the same blind-spot
# class as the walk shuffle. Stand-ending modes must finish with all six
# feet at the support surface; lower must finish with no leg elevated
# (tucked is fine, a vertical flag leg ~130+ mm is not). Eval-side only,
# independent of reward terms. Gate wiring is behind --end-posture-gate
# until the champions are baselined (report the baseline first).
STAND_END_MODES = ("rise", "raise", "hold", "lean", "track")
END_CLEAR_STAND_MM = 20.0
END_CLEAR_BELLY_MM = 60.0


def _success(mode: str, term: bool, ep: dict,
             end_posture_gate: bool = False) -> bool:
    if term:
        return False
    if end_posture_gate and mode in STAND_END_MODES + ("lower",) \
            and ep.get("end_posture_ok") is False:
        return False
    if mode in ("rise", "lower"):
        return ep["height_err_end_mm"] is not None \
            and ep["height_err_end_mm"] <= 15.0
    if mode == "raise":
        return ep["height_err_end_mm"] is not None \
            and ep["height_err_end_mm"] <= 5.0
    if mode == "walk":
        # gait_valid: no persistently sacrificed leg (see gait-validity
        # gate below); tracking alone has repeatedly hidden the exploit.
        return (ep.get("vel_err_mean", 1e9) <= 0.03
                and ep.get("gait_valid", True))
    if mode in ("lean", "track", "hold"):
        return ep.get("track_err_mean_deg", 1e9) <= 1.5
    if mode == "unload":
        return True   # survival; force stats reported separately
    return True


def run_episode(env, model, *, deterministic: bool, video: bool,
                annotate, end_posture_gate: bool = False) -> tuple[dict, list]:
    obs, info0 = env.reset()
    mode = info0.get("goal_mode", "?")
    kind = _start_kind(env._goal_traj) if env._goal_traj else "plant"
    pads = [env.model.body(f"L{i}_pad").id for i in range(6)]

    frames = []
    cur_hist = []            # (T, 18) per-servo current
    contact_hist = []        # (T, 6) bool
    pad_xy_hist = []         # (T, 6, 2) world
    track_errs, vel_errs, speeds = [], [], []
    cmd_dist_m, along_dist_m = 0.0, 0.0
    h_err = None
    chassis0 = env.data.xpos[env._chassis_bid, :2].copy()
    ret, safety_flags, term = 0.0, 0, False

    done = False
    while not done:
        a, _ = model.predict(obs, deterministic=deterministic)
        obs, r, term, trunc, info = env.step(a)
        ret += float(r)
        done = term or trunc

        st = env._state
        if st.servo_current is not None:
            cur_hist.append(st.servo_current.copy())
        contact_hist.append([
            float(env.data.sensordata[adr]) > CONTACT_N
            for adr in env._touch_adr])
        pad_xy_hist.append(
            [env.data.xpos[b].copy() for b in pads])
        if not info.get("safety_ok", True):
            safety_flags += 1
        if "track_err_deg" in info:
            track_errs.append(abs(float(info["track_err_deg"])))
        if "height_err_mm" in info:
            h_err = abs(float(info["height_err_mm"]))
        if "walk_vel_err" in info:
            vel_errs.append(float(info["walk_vel_err"]))
            speeds.append(float(info["walk_speed"]))
        if mode == "walk":
            # progress_ratio bookkeeping (operator ruling 2026-08-09
            # WALK-DISTANCE-GATE): along-command body displacement vs
            # commanded displacement, integrated over commanded ticks.
            g = env._current_goal()
            if g is not None:
                s_ref = math.hypot(g.vx_ref, g.vy_ref)
                if s_ref > 1e-3:
                    v = env._body_vel_xy()
                    cmd_dist_m += s_ref * env.dt
                    along_dist_m += ((v[0] * g.vx_ref
                                      + v[1] * g.vy_ref)
                                     / s_ref) * env.dt

        if video:
            frame = env.render()
            if frame is not None:
                cur = cur_hist[-1] if cur_hist else np.zeros(18)
                hot = int(np.argmax(cur))
                lines = [
                    f"{mode} ({kind})  t={env._step_i * env.dt:5.2f}s  "
                    f"R={ret:+.1f}",
                    f"tilt {info.get('roll_deg', 0):+.1f}/"
                    f"{info.get('pitch_deg', 0):+.1f}deg  "
                    f"h_err {info.get('height_err_mm', 0):+.0f}mm",
                    f"I max {cur.max():.2f}A (servo {hot})  "
                    f"mean {cur.mean():.2f}A",
                    "feet " + "".join(
                        "#" if c else "." for c in contact_hist[-1]),
                ]
                if vel_errs:
                    g = env._current_goal()
                    lines.append(
                        f"v {speeds[-1]:.3f} vs ref "
                        f"{math.hypot(g.vx_ref, g.vy_ref):.3f} m/s "
                        f"(err {vel_errs[-1]:.3f})")
                if term:
                    lines.append(
                        f"TERMINATED: {info.get('termination_reason')}")
                frames.append(annotate(frame, lines))

    cur = np.asarray(cur_hist) if cur_hist else np.zeros((1, 18))
    leg_mean = cur.reshape(len(cur), 6, 3).mean(axis=(0, 2))  # per leg
    contact = np.asarray(contact_hist, dtype=bool)
    pad_xyz = np.asarray(pad_xy_hist)          # (T, 6, 3) world
    pad_xy = pad_xyz[:, :, :2]

    # Gait metrics: swing = contiguous no-contact run; slip = XY motion
    # of a pad while in contact.
    duty = contact.mean(axis=0)
    swings, swing_len_s, strides, slips = [], [], [], []
    for f in range(6):
        c = contact[:, f]
        d = np.diff(c.astype(int))
        lifts = np.where(d == -1)[0]
        downs = np.where(d == 1)[0]
        swings.append(len(lifts))
        for lo in lifts:
            hi_c = downs[downs > lo]
            if len(hi_c):
                hi = int(hi_c[0])
                swing_len_s.append((hi - lo) * env.dt)
                strides.append(float(np.linalg.norm(
                    pad_xy[hi, f] - pad_xy[lo, f])))
        moved = np.linalg.norm(np.diff(pad_xy[:, f], axis=0), axis=1)
        slips.append(float(moved[c[:-1]].sum()))

    ep = {
        "mode": mode, "start_kind": kind,
        "return": round(ret, 2),
        "terminated": bool(term),
        "term_reason": info.get("termination_reason", ""),
        "safety_flags": int(safety_flags),
        "height_err_end_mm": None if h_err is None else round(h_err, 1),
        "track_err_mean_deg": (round(float(np.mean(track_errs)), 2)
                               if track_errs else None),
        # currents (RL_PLAN_NEXT §4)
        "cur_max_a": round(float(cur.max()), 2),
        "cur_p95_a": round(float(np.percentile(cur, 95)), 2),
        "cur_hot_servo": int(np.argmax(cur.max(axis=0))),
        "cur_s_above_soft": round(
            float((cur > SOFT_CURRENT_A).any(axis=1).mean()
                  * len(cur) * env.dt), 2),
        "cur_leg_imbalance": round(
            float(leg_mean.max() / max(leg_mean.mean(), 1e-6)), 2),
        # gait (RL_PLAN_NEXT §5)
        "duty_cycle": [round(float(x), 2) for x in duty],
        "swing_count": swings,
        "swing_s_mean": (round(float(np.mean(swing_len_s)), 2)
                         if swing_len_s else 0.0),
        "stride_m_mean": (round(float(np.mean(strides)), 3)
                          if strides else 0.0),
        "slip_m_total": round(float(np.sum(slips)), 3),
        "forward_dist_m": round(float(np.linalg.norm(
            env.data.xpos[env._chassis_bid, :2] - chassis0)), 3),
    }
    if vel_errs:
        ep["vel_err_mean"] = round(float(np.mean(vel_errs)), 3)
        ep["speed_mean_m_s"] = round(float(np.mean(speeds)), 3)
    if mode == "walk":
        # Gait-validity gate (guardrails, external review §5b): a walking
        # checkpoint is INVALID if any leg is persistently sacrificed,
        # regardless of velocity error. Sacrificed = airborne essentially
        # the whole episode (parked flag leg) or grounded the whole
        # episode with zero swings (dragged anchor). Permanent eval-side
        # detection, independent of any reward term.
        ep["sacrificed_legs"] = [
            f for f in range(6)
            if duty[f] < 0.10 or (duty[f] > 0.95 and swings[f] == 0)]
        ep["gait_valid"] = not ep["sacrificed_legs"]
        # Ruled walk metrics (operator rulings 2026-08-09 §3/§6):
        # progress_ratio = along-command displacement / commanded
        # displacement (promotion band 0.75-1.25; >1.25 = overspeed);
        # slip_per_m = episode loaded foot-XY travel per meter of
        # along-command progress (primary skating metric, never
        # touchdown-reset by construction of slip_m_total).
        ep["along_dist_m"] = round(along_dist_m, 3)
        ep["cmd_dist_m"] = round(cmd_dist_m, 3)
        ep["progress_ratio"] = (round(along_dist_m / cmd_dist_m, 3)
                                if cmd_dist_m > 1e-6 else None)
        ep["slip_per_m"] = round(
            float(np.sum(slips)) / max(along_dist_m, 0.05), 3)
    if mode in STAND_END_MODES + ("lower",) \
            and getattr(env, "_pad_z_ref", None) is not None:
        # Mean pad clearance over the final 0.5 s (single-frame contact
        # flicker lies; a parked flag leg does not).
        tail = max(1, int(round(0.5 / env.dt)))
        clear_mm = (pad_xyz[-tail:, :, 2].mean(axis=0)
                    - env._pad_z_ref) * 1000.0
        thr = END_CLEAR_BELLY_MM if mode == "lower" else END_CLEAR_STAND_MM
        ep["end_clear_mm"] = [round(float(c), 1) for c in clear_mm]
        ep["end_posture_ok"] = bool((clear_mm <= thr).all())
    ep["success"] = _success(mode, term, ep, end_posture_gate)
    return ep, frames


# ---------------------------------------------------------------------------


STRIP_FRAMES = 10


def _save_video(frames: list, path: Path) -> None:
    if not frames:
        return
    import imageio
    imageio.mimsave(path.with_suffix(".mp4"), frames, fps=FPS,
                    macro_block_size=1)
    # Film strip for quick embedding in reports/chat.
    idx = np.linspace(0, len(frames) - 1, STRIP_FRAMES).astype(int)
    strip = np.concatenate([frames[i] for i in idx], axis=1)
    imageio.imwrite(path.with_suffix(".png"), strip)


def _save_contact_sheet(strip_paths: list[Path], out: Path) -> None:
    """Stack one strip per mode into a single review image.

    One glance answers 'what is this checkpoint actually doing' across
    every skill — the unit of systematic video review (VIDEO_REVIEW.md).
    """
    import imageio.v2 as imageio
    rows = [imageio.imread(p) for p in strip_paths if p.exists()]
    if not rows:
        return
    w = max(r.shape[1] for r in rows)
    rows = [np.pad(r, ((0, 0), (0, w - r.shape[1]), (0, 0)))
            for r in rows]
    imageio.imwrite(out / "contact_sheet.png",
                    np.concatenate(rows, axis=0))
    print(f"[eval_checkpoint] contact sheet → {out / 'contact_sheet.png'}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--task", choices=sorted(ENV_CLASSES),
                    default="joint_goal")
    ap.add_argument("--modes", nargs="*", default=None,
                    help="default: env's EVAL_MODES")
    ap.add_argument("--per-mode", type=int, default=6)
    ap.add_argument("--dr-scale", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--stochastic", action="store_true",
                    help="add a pass sampling at the checkpoint's std")
    # Default ON since 2026-08-08 (champion baseline recorded first, per
    # directive: stance champ lower 0/12, rise 5/12, hold 12/12). All
    # rise/lower success counts after this date are posture-strict;
    # earlier logged numbers are height-only and NOT comparable.
    ap.add_argument("--end-posture-gate", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="wire end_posture_ok into success for stand/"
                         "belly-ending modes")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--video-every", type=int, default=3,
                    help="record every Nth episode per mode (1st always)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cfg-set", action="append", default=None,
                    help="config override, e.g. "
                         "goal.walk_speed_max_m_s=0.06 (repeatable)")
    args = ap.parse_args()

    from stable_baselines3 import PPO
    from .train_ppo_sim import _annotate_frame

    env_cls = ENV_CLASSES[args.task]
    # Apply --cfg-set BEFORE construction: overrides can change obs
    # WIDTH (e.g. goal.walk_phase_obs), which is baked in __init__ —
    # post-hoc env.cfg mutation silently kept the legacy width (found
    # evaluating the cw-walk-phase smoke, cycle 11).
    cfg_kw = {}
    if args.cfg_set:
        from rl_move.config import load_config
        cfg = load_config()
        for spec in args.cfg_set:
            key, val = spec.split("=", 1)
            sect, name = key.split(".", 1)
            try:
                parsed: float | str = float(val)
            except ValueError:
                parsed = val.strip()   # e.g. goal.walk_park_bank=PATH
            cfg.setdefault(sect, {})[name] = parsed
        cfg_kw["cfg"] = cfg
    # dr.<field> cfg overrides need the randomizer alive even at
    # --dr-scale 0 (payload/latency-axis arms: scale 0 = nominal sim +
    # ONLY the overridden field randomized). Without this the override
    # silently evaluated as plain DR0 (cycle 49).
    _has_dr_ov = bool(cfg_kw.get("cfg", {}).get("dr"))
    env = env_cls(params=SimServoParams.load(),
                  randomize=(args.dr_scale > 0 or _has_dr_ov),
                  dr_scale=args.dr_scale,
                  episode_seconds=args.episode_seconds, seed=args.seed,
                  render_mode=None if args.no_video else "rgb_array",
                  **cfg_kw)
    model = PPO.load(args.checkpoint, device="cpu")
    std = float(np.exp(model.policy.log_std.detach().numpy().mean()))

    modes = args.modes or list(getattr(env_cls, "EVAL_MODES",
                                       ("hold", "track", "rise")))
    gen = env._goal_gen
    out = args.out or (_PROTO / "logs" / "ckpt_eval" /
                       f"{args.checkpoint.stem}_{time.strftime('%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)

    passes = [("det", True)] + ([("sto", False)] if args.stochastic else [])
    report = {"checkpoint": str(args.checkpoint), "task": args.task,
              "dr_scale": args.dr_scale, "seed": args.seed,
              "policy_std": round(std, 3), "episodes": {}}
    sheet_strips: list[Path] = []

    for tag, det in passes:
        for mode in modes:
            for m in ALL_MODES:
                if hasattr(gen, f"p_{m}"):
                    setattr(gen, f"p_{m}", 1.0 if m == mode else 0.0)
            eps = []
            for k in range(args.per_mode):
                # sto passes get video too: an unwatched success cannot
                # support a PASS (guardrails: watched_modes=all_verdict_modes)
                # walk mode renders EVERY episode so gait-invalid episodes
                # (tripod park / wander) always leave watchable video —
                # 2026-08-09: a 15 s sto tripod park landed between the
                # every-Nth slots and was scalar-only. Non-scheduled valid
                # episodes are rendered but not saved.
                scheduled = (k == 0 or k % args.video_every == 0)
                video = (not args.no_video and (scheduled or mode == "walk"))
                ep, frames = run_episode(
                    env, model, deterministic=det, video=video,
                    annotate=_annotate_frame,
                    end_posture_gate=args.end_posture_gate)
                eps.append(ep)
                if frames and (scheduled or not ep.get("gait_valid", True)):
                    _save_video(frames, out / f"{mode}_{tag}_{k}")
                    if det and k == 0:
                        sheet_strips.append(
                            (out / f"{mode}_{tag}_{k}").with_suffix(".png"))
            report["episodes"][f"{mode}/{tag}"] = eps
            n_ok = sum(e["success"] for e in eps)
            hot = max(e["cur_max_a"] for e in eps)
            kinds = ""
            if mode in ("rise", "lower"):
                by = {}
                for e in eps:
                    ok, tot = by.get(e["start_kind"], (0, 0))
                    by[e["start_kind"]] = (ok + e["success"], tot + 1)
                kinds = " [" + " ".join(
                    f"{k}:{a}/{b}" for k, (a, b) in sorted(by.items())) + "]"
            extra = ""
            if any("vel_err_mean" in e for e in eps):
                ve = [e["vel_err_mean"] for e in eps if "vel_err_mean" in e]
                sp = [e["speed_mean_m_s"] for e in eps
                      if "speed_mean_m_s" in e]
                extra = (f" | vel_err {np.mean(ve):.3f} "
                         f"speed {np.mean(sp):.3f} m/s")
            if mode == "walk":
                n_valid = sum(bool(e.get("gait_valid")) for e in eps)
                sac = sorted({f for e in eps
                              for f in e.get("sacrificed_legs", [])})
                extra += (f" | gait_valid {n_valid}/{len(eps)}"
                          + (f" sacrificed legs {sac}" if sac else ""))
                pr = [e["progress_ratio"] for e in eps
                      if e.get("progress_ratio") is not None]
                spm = [e["slip_per_m"] for e in eps
                       if e.get("slip_per_m") is not None]
                if pr:
                    extra += (f" | prog_ratio {np.mean(pr):.2f} "
                              f"slip/m {np.mean(spm):.2f}")
            if any("end_posture_ok" in e for e in eps):
                n_ep = sum(bool(e.get("end_posture_ok")) for e in eps)
                worst = max(max(e["end_clear_mm"]) for e in eps
                            if "end_clear_mm" in e)
                extra += (f" | end_posture {n_ep}/{len(eps)}"
                          f" worst_clear {worst:.0f}mm")
            print(f"[{tag}] {mode:6s}: {n_ok}/{len(eps)}{kinds} | "
                  f"Imax {hot:.2f}A | imbal "
                  f"{max(e['cur_leg_imbalance'] for e in eps):.2f}"
                  f"{extra}")

    if sheet_strips:
        _save_contact_sheet(sheet_strips, out)
    (out / "report.json").write_text(json.dumps(report, indent=2))
    print(f"[eval_checkpoint] artifacts → {out}")
    env.close()


if __name__ == "__main__":
    main()
