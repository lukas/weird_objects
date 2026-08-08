"""Pre-training sanity check: render every task mode, audit the reward.

Run this BEFORE every training run (the agent workflow is: run preflight,
actually LOOK at the frames, only then launch PPO). It catches the class
of bug that killed earlier runs — detached feet, phantom tilt, reward
terms that pay for freezing — before burning training time.

For each goal mode (hold / lean / track / unload / rise) it runs:

- a FROZEN episode (zero action — the shortcut policy), and
- a SCRIPTED episode (feedforward: push the commanded refs straight into
  the body-IK action channels; curl in first for rise),

saves frame strips to ``logs/preflight/<ts>/`` as PNGs, and prints the
per-step reward component means. The gate:

    scripted return  >>  frozen return  >>  failure return

If frozen ≈ scripted, the reward pays for doing nothing and PPO WILL find
that. A FAILURE episode (hard constant lean, expected to trip safety)
calibrates the bottom of the scale.

Usage (from prototype_sts3215/):
    ../../.venv/bin/python -m rl_move.sim.preflight --dr-scale 0.2
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rl_move.config import cfg_get, load_config  # noqa: E402

from .goal_task import SimHexapodGoalEnv  # noqa: E402
from .servo_model import SimServoParams  # noqa: E402

MODES = ("hold", "lean", "track", "unload", "rise")
N_ACT = 6


def _force_mode(env: SimHexapodGoalEnv, mode: str) -> None:
    g = env._goal_gen
    for m in MODES:
        setattr(g, f"p_{m}", 1.0 if m == mode else 0.0)


def _scripted_action(env: SimHexapodGoalEnv, step: int,
                     cfg: dict) -> np.ndarray:
    """Feedforward controller: put the refs straight into the IK channels."""
    goal = env._goal_traj.at(step)
    max_roll = math.radians(float(
        cfg_get(cfg, "actions", "max_roll_deg", default=5.0)))
    max_pitch = math.radians(float(
        cfg_get(cfg, "actions", "max_pitch_deg", default=5.0)))
    max_h = float(cfg_get(cfg, "actions", "max_height_mm", default=80.0)) \
        * 0.001
    a = np.zeros(N_ACT)
    a[0] = goal.roll_ref / max(max_roll, 1e-9)
    a[1] = goal.pitch_ref / max(max_pitch, 1e-9)
    a[2] = goal.height_ref / max(max_h, 1e-9)
    if env._goal_traj.start_at == "zero":
        # Full curl rate; the IK ratchet integrates it to the plant
        # footprint in ~2.5 s, inside the hold window.
        a[5] = 1.0
    elif env._goal_traj.unload_leg is not None:
        # Weight shift: translate AND lean away from the target leg —
        # the lean is what actually opens the foot's contact.
        az = (env._goal_traj.unload_leg + 0.5) * math.pi / 3.0
        ramp = min(1.0, step * env.dt / 1.5)
        a[3] = -math.cos(az) * ramp
        a[4] = -math.sin(az) * ramp
        a[0] = -math.sin(az) * ramp
        a[1] = math.cos(az) * ramp
    return np.clip(a, -1.0, 1.0)


def _run_episode(env: SimHexapodGoalEnv, mode: str, policy: str,
                 cfg: dict, out_dir: Path, n_frames: int = 8) -> dict:
    _force_mode(env, mode)
    obs, info = env.reset()
    del obs
    assert info.get("goal_mode") == mode, info.get("goal_mode")
    parts_acc: dict[str, list[float]] = {}
    frames, ret, step, term = [env.render()], 0.0, 0, False
    track_errs, h_errs = [], []
    while True:
        if policy == "frozen":
            a = np.zeros(N_ACT)
        elif policy == "failure":
            a = np.zeros(N_ACT)
            a[0] = 1.0  # hard constant max lean — should trip safety
        else:
            a = _scripted_action(env, step, cfg)
        _obs, r, term, trunc, info = env.step(a)
        ret += float(r)
        step += 1
        for k, v in info.items():
            if k.startswith("reward_"):
                parts_acc.setdefault(k, []).append(float(v))
        if "track_err_deg" in info:
            track_errs.append(float(info["track_err_deg"]))
        if "height_err_mm" in info:
            h_errs.append(abs(float(info["height_err_mm"])))
        frames.append(env.render())
        if term or trunc:
            break
    # Save an evenly spaced strip of frames.
    idx = np.linspace(0, len(frames) - 1, n_frames).astype(int)
    strip = np.concatenate([frames[i] for i in idx], axis=1)
    try:
        from PIL import Image
        Image.fromarray(strip).save(out_dir / f"{mode}_{policy}.png")
    except ImportError:
        pass
    return {
        "mode": mode, "policy": policy, "return": ret, "steps": step,
        "terminated": bool(term),
        "term_reason": info.get("termination_reason", ""),
        "track_err_deg": float(np.mean(track_errs)) if track_errs else None,
        "height_err_mm": float(np.mean(h_errs)) if h_errs else None,
        "parts": {k: float(np.mean(v)) for k, v in parts_acc.items()},
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dr-scale", type=float, default=0.2)
    ap.add_argument("--no-dr", action="store_true")
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args(argv)

    cfg = load_config()
    params = SimServoParams.load()
    out_dir = (_PROTO / "logs" / "preflight"
               / time.strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)

    env = SimHexapodGoalEnv(
        params=params, randomize=not args.no_dr, dr_scale=args.dr_scale,
        episode_seconds=args.episode_seconds, seed=args.seed,
        render_mode="rgb_array", mesh_visuals=False)

    results = []
    for mode in MODES:
        for policy in ("frozen", "scripted"):
            results.append(_run_episode(env, mode, policy, cfg, out_dir))
    results.append(_run_episode(env, "lean", "failure", cfg, out_dir))
    env.close()

    print(f"\npreflight frames: {out_dir}\n")
    hdr = (f"{'mode':<8}{'policy':<10}{'return':>9}{'steps':>7}"
           f"{'trk err°':>10}{'h err mm':>10}  end")
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        te = f"{r['track_err_deg']:.2f}" if r['track_err_deg'] is not None \
            else "-"
        he = f"{r['height_err_mm']:.1f}" if r['height_err_mm'] is not None \
            else "-"
        end = r["term_reason"] if r["terminated"] else "ok"
        print(f"{r['mode']:<8}{r['policy']:<10}{r['return']:>9.1f}"
              f"{r['steps']:>7}{te:>10}{he:>10}  {end}")

    print("\nreward components (mean/step):")
    keys = sorted({k for r in results for k in r["parts"]})
    print(f"{'mode':<8}{'policy':<10}" + "".join(
        f"{k.removeprefix('reward_'):>13}" for k in keys))
    for r in results:
        print(f"{r['mode']:<8}{r['policy']:<10}" + "".join(
            f"{r['parts'].get(k, 0.0):>13.4f}" for k in keys))

    # The gate: scripted must clearly beat frozen on every goal mode.
    print("\ngate (scripted - frozen return, per mode):")
    ok = True
    by = {(r["mode"], r["policy"]): r for r in results}
    for mode in MODES:
        d = by[(mode, "scripted")]["return"] - by[(mode, "frozen")]["return"]
        flag = "OK" if d > 5.0 else ("~" if d > 0 else "FAIL")
        if d <= 0:
            ok = False
        print(f"  {mode:<8} {d:+8.1f}  {flag}")
    print("\npreflight:", "PASS" if ok else
          "FAIL — the reward pays for freezing somewhere; fix before "
          "training")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
