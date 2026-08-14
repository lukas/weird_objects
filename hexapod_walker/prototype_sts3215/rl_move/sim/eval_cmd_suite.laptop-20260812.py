"""Fixed retained-command suite runner (multitask track, MULTITASK.md).

The multitask verdicts are per-command, never one aggregate return:
"per-command tracking error, fall rate, progress, slip/contact, current,
det+sto — recorded SEPARATELY". eval_drive/eval_yaw build their own
scenario panels from a speed envelope and cannot probe exact (vx, vy,
wz) triples, so acquisition/erosion across fine-tune checkpoints was
unmeasurable. This runner drives EXACT commanded triples:

  retained suite   stand, forward, yaw-left, yaw-right, mixed
                   (mixed = fwd -> stop -> yaw -> fwd transitions)
  zero-shot probes vx=0.037 wz=0.07 | vx=0.025 vy=0.012 |
                   vx=0.05 wz=-0.11   (never sampled exactly in
                   training; interpolation evidence)
  --cmd vx,vy,wz[,label]  any extra exact triple (repeatable) — the
                   Phase-2 "new downstream command" probes.

Per command x {det, sto}: median |vx/vy/wz error| (blend-skipped),
falls, net progress (or drift for stand), loaded slip ratio, current.
GRU/dual-GRU checkpoints load via load_checkpoint_auto with hidden
state threaded across steps; a checkpoint that is N_MODE_OBS wider
than the env auto-enables obs.mode_onehot + obs.mode_onehot_cmd (the
dual-core transplant routes on the live command — a stopped command
must light the stance core here exactly as in training).

    python3 -m rl_move.sim.eval_cmd_suite <ckpt.zip> \
        [--speed 0.05] [--wz 0.1] [--cmd 0.0,0.0,-0.3,big-right] \
        [--cfg-set k=v ...] [--out suite.json]

Always exit 0 unless --max-falls is given (orchestrator gate use).
Video stays eval_checkpoint's job; this tool is the numbers half.
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
from pathlib import Path

import numpy as np

# Interpolation probes fixed by MULTITASK.md ("commands never sampled
# exactly"): keep them frozen so every arm/checkpoint is comparable.
ZERO_SHOT = [
    ("interp-fwd-yaw",  0.037, 0.0,   0.07),
    ("interp-fwd-lat",  0.025, 0.012, 0.0),
    ("interp-fwd-yawR", 0.05,  0.0,  -0.11),
]


def retained_suite(s: float, w: float) -> list[tuple[str, list]]:
    """(name, [(seconds, vx, vy, wz), ...]) — the fixed retained set."""
    return [
        ("stand",     [(8.0, 0.0, 0.0, 0.0)]),
        ("forward",   [(1.0, 0, 0, 0), (8.0,  s, 0.0, 0.0)]),
        ("yaw-left",  [(1.0, 0, 0, 0), (8.0,  s, 0.0,  w)]),
        ("yaw-right", [(1.0, 0, 0, 0), (8.0,  s, 0.0, -w)]),
        ("mixed",     [(1.0, 0, 0, 0), (4.0, s, 0.0, 0.0),
                       (3.0, 0.0, 0.0, 0.0), (4.0, s, 0.0, w),
                       (4.0, s, 0.0, 0.0)]),
    ]


class _StatefulPredictor:
    """Thread recurrent hidden state across steps (zero-state predict
    on a GRU checkpoint evaluates a memory-less lobotomy, not the
    policy). Also the uniform interface for plain PPO."""

    def __init__(self, model):
        self._policy = model.policy
        self._recurrent = getattr(model.policy, "lstm_actor",
                                  None) is not None
        self.reset()

    def reset(self) -> None:
        self._state = None
        self._start = np.ones((1,), dtype=bool)

    def predict(self, obs, deterministic: bool):
        if self._recurrent:
            a, self._state = self._policy.predict(
                obs, state=self._state, episode_start=self._start,
                deterministic=deterministic)
            self._start = np.zeros((1,), dtype=bool)
            return a
        a, _ = self._policy.predict(obs, deterministic=deterministic)
        return a


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--speed", type=float, default=0.05,
                    help="forward command for the retained suite (m/s)")
    ap.add_argument("--wz", type=float, default=0.1,
                    help="small-yaw command for the retained suite "
                         "(rad/s)")
    ap.add_argument("--cmd", action="append", default=None,
                    metavar="VX,VY,WZ[,LABEL]",
                    help="extra exact command triple (repeatable)")
    ap.add_argument("--cmd-seconds", type=float, default=8.0,
                    help="hold time for --cmd and zero-shot triples")
    ap.add_argument("--no-interp", action="store_true",
                    help="skip the fixed zero-shot probes")
    ap.add_argument("--sto-episodes", type=int, default=1,
                    help="stochastic repeats per command (0 = det only)")
    ap.add_argument("--blend-skip-s", type=float, default=1.5,
                    help="seconds ignored after each command change "
                         "(training blends commands up to 1 s)")
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-falls", type=int, default=None,
                    help="exit 1 if total falls exceed this (gate use)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V")
    args = ap.parse_args()

    from rl_move.config import load_config

    from .gru_policy import load_checkpoint_auto
    from .servo_model import SimServoParams
    from .walk_task import N_MODE_OBS, SimHexapodJointWalkEnv

    cfg = load_config()
    if args.cfg_set:
        for spec in args.cfg_set:
            key, val = spec.split("=", 1)
            sect, name = key.split(".", 1)
            try:
                parsed: float | str = float(val)
            except ValueError:
                parsed = val.strip()
            cfg.setdefault(sect, {})[name] = parsed

    def make_env(c):
        env = SimHexapodJointWalkEnv(
            params=SimServoParams.from_cfg(c), cfg=c,
            randomize=args.dr_scale > 0, dr_scale=args.dr_scale,
            episode_seconds=600.0, seed=args.seed)
        gen = env._goal_gen
        gen.p_walk = 1.0
        for m in ("hold", "lean", "track", "unload", "raise", "rise",
                  "lower"):
            setattr(gen, f"p_{m}", 0.0)
        return env

    env = make_env(cfg)
    model = load_checkpoint_auto(args.checkpoint, device="cpu")
    n_model = int(model.observation_space.shape[0])
    n_env = int(env.observation_space.shape[0])
    if n_model == n_env + N_MODE_OBS:
        # Dual-core GRU transplant checkpoint: route on the LIVE
        # command, exactly as it trained (walk_task obs.mode_onehot_cmd).
        print(f"[cmd_suite] checkpoint obs {n_model} = env {n_env} + "
              f"{N_MODE_OBS}: enabling obs.mode_onehot + mode_onehot_cmd")
        env.close()
        cfg.setdefault("obs", {})["mode_onehot"] = 1.0
        cfg["obs"].setdefault("mode_onehot_cmd", 1.0)
        env = make_env(cfg)
    elif n_model != n_env:
        raise SystemExit(
            f"checkpoint obs width {n_model} does not fit the env "
            f"({n_env}); wrong --cfg-set for this checkpoint?")
    predictor = _StatefulPredictor(model)

    def run_schedule(phases, *, deterministic: bool, seed: int) -> dict:
        obs, _ = env.reset(seed=seed)
        predictor.reset()
        falls = 0
        errs_vx, errs_vy, errs_wz = [], [], []
        cur_mean, cur_max = [], 0.0
        slip_m = prog_m = 0.0
        prev_slip = prev_prog = 0.0
        p0 = np.asarray(env.data.qpos[:2]).copy()
        path_m, prev_p = 0.0, p0.copy()
        for seconds, vx, vy, wz in phases:
            t_seg = 0.0
            for _ in range(max(1, int(seconds / env.dt))):
                traj = env._goal_traj
                if traj is not None and hasattr(traj, "vx"):
                    traj.vx[:] = vx
                    traj.vy[:] = vy
                    if getattr(traj, "wz", None) is not None:
                        traj.wz[:] = wz
                    else:
                        traj.wz = np.full_like(np.asarray(traj.vx), wz)
                a = predictor.predict(obs, deterministic=deterministic)
                obs, _r, term, trunc, info = env.step(a)
                t_seg += env.dt
                p = np.asarray(env.data.qpos[:2])
                path_m += float(np.linalg.norm(p - prev_p))
                prev_p = p.copy()
                slip_m += max(float(env._ls_slip_m) - prev_slip, 0.0)
                prog_m += max(float(env._ls_prog_m) - prev_prog, 0.0)
                prev_slip = float(env._ls_slip_m)
                prev_prog = float(env._ls_prog_m)
                if t_seg >= args.blend_skip_s:
                    v = env._body_vel_xy()
                    errs_vx.append(abs(float(v[0]) - vx))
                    errs_vy.append(abs(float(v[1]) - vy))
                    errs_wz.append(abs(env._body_wz() - wz))
                    if "mean_current_a" in info:
                        cur_mean.append(info["mean_current_a"])
                        cur_max = max(cur_max, info["max_current_a"])
                if term or trunc:
                    falls += 1
                    obs, _ = env.reset()
                    predictor.reset()
                    prev_slip = prev_prog = 0.0
                    prev_p = np.asarray(env.data.qpos[:2]).copy()
        net_m = float(np.linalg.norm(np.asarray(env.data.qpos[:2]) - p0))

        def med(x):
            return round(float(np.median(x)), 4) if x else None
        return {
            "falls": falls,
            "vx_err_med": med(errs_vx),
            "vy_err_med": med(errs_vy),
            "wz_err_med": med(errs_wz),
            "net_disp_m": round(net_m, 4),
            "path_m": round(path_m, 4),
            "slip_ratio": (round(slip_m / prog_m, 4)
                           if prog_m > 1e-4 else None),
            "current_a_med": med(cur_mean),
            "current_a_max": round(float(cur_max), 3),
        }

    commands = retained_suite(args.speed, args.wz)
    if not args.no_interp:
        commands += [(name, [(1.0, 0, 0, 0), (args.cmd_seconds, vx, vy, wz)])
                     for name, vx, vy, wz in ZERO_SHOT]
    for i, spec in enumerate(args.cmd or []):
        parts = spec.split(",")
        vx, vy, wz = (float(p) for p in parts[:3])
        label = parts[3].strip() if len(parts) > 3 else f"cmd-{i}"
        commands.append(
            (label, [(1.0, 0, 0, 0), (args.cmd_seconds, vx, vy, wz)]))

    results: dict[str, dict] = {}
    total_falls = 0
    for name, phases in commands:
        entry: dict[str, dict] = {}
        entry["det"] = run_schedule(phases, deterministic=True,
                                    seed=args.seed)
        total_falls += entry["det"]["falls"]
        for k in range(args.sto_episodes):
            r = run_schedule(phases, deterministic=False,
                             seed=args.seed + 1000 + k)
            entry[f"sto{k}" if args.sto_episodes > 1 else "sto"] = r
            total_falls += r["falls"]
        results[name] = entry
        d = entry["det"]
        print(f"{name:16s} det: falls={d['falls']} "
              f"vx_err={d['vx_err_med']} wz_err={d['wz_err_med']} "
              f"net={d['net_disp_m']}m slip={d['slip_ratio']} "
              f"cur={d['current_a_med']}A")

    print(f"total falls = {total_falls}")
    report = {
        "checkpoint": str(args.checkpoint),
        "speed": args.speed, "wz": args.wz,
        "blend_skip_s": args.blend_skip_s, "seed": args.seed,
        "total_falls": total_falls,
        "commands": results,
    }
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=1))
        print(f"wrote {args.out}")
    if args.max_falls is not None and total_falls > args.max_falls:
        print(f"FALL GATE: FAIL ({total_falls} > {args.max_falls})")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
