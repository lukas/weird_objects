"""Fixed retained-command suite: exact (vx, vy, wz) triples, det+sto.

Multitask track instrument (tracks/multitask/STATUS.md "Next",
[CODE, when wave 1 lands] — wave 1 landed 08-12/08-13). Neither
eval_drive.py (named linear scenarios, no wz) nor eval_yaw.py (fixed
yaw panel) can probe an ARBITRARY exact command triple, so wave-2 /
transfer-test retention comparisons ("same command list, before vs
after fine-tune") had no runner. This drives the policy through an
explicit list of exact (vx, vy, wz) commands — the identical list
every time — and reports per command, per pass (det AND sto):

  falls              terminations during the measured window
  v_err_med          median |v_xy − v_ref| (m/s), plus per-axis
  wz_err_med         median |wz − wz_ref| (rad/s)
  slip_per_m         loaded-foot XY travel per meter of along-command
                     body progress (harness definition; stance
                     commands report raw slip meters instead)
  cur_mean/p95_a     servo current over the measured window

Output is machine-readable JSON (--out): one row per (command, pass),
medians across --episodes episodes. NOT a gate — this is a
measurement suite for retained-command erosion; it always exits 0
unless it errors. MLP PPO checkpoints only (the multitask lineage).

    cd prototype_sts3215 && python3 -m rl_move.sim.eval_cmd_suite \
        <ckpt.zip> --cmd 0.05,0,0 --cmd 0,0,0.3 [--suite cmds.json] \
        [--cfg-set k=v ...] [--out out.json]

--suite file: JSON list of [vx, vy, wz] or {"name":..., "vx":...,
"vy":..., "wz":...}. Without --cmd/--suite a default panel is used
(fwd/back/left/right/stop + arcs/tips at --speed / --wz-max).
"""
from __future__ import annotations

import os

# Cap math threads before numpy import (same reason as
# eval_checkpoint: unbounded per-process pools thrashed the
# controller's node, 08-09).
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
from pathlib import Path

import numpy as np

CONTACT_N = 0.5   # touch-sensor force that counts as contact (harness)


def default_panel(s: float, w: float) -> list[tuple[str, float, float, float]]:
    return [
        ("fwd",       s,   0.0,  0.0),
        ("back",     -s,   0.0,  0.0),
        ("left",      0.0,  s,   0.0),
        ("right",     0.0, -s,   0.0),
        ("stop",      0.0,  0.0, 0.0),
        ("arc-left",  s,   0.0,  w / 2),
        ("arc-right", s,   0.0, -w / 2),
        ("tip-left",  0.0,  0.0,  w),
        ("tip-right", 0.0,  0.0, -w),
    ]


def _cmd_name(vx: float, vy: float, wz: float) -> str:
    return f"vx{vx:+.3f}_vy{vy:+.3f}_wz{wz:+.2f}"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--cmd", action="append", default=None,
                    metavar="VX,VY,WZ",
                    help="exact command triple (repeatable)")
    ap.add_argument("--suite", type=Path, default=None,
                    help="JSON list of [vx,vy,wz] or "
                         "{name,vx,vy,wz} objects")
    ap.add_argument("--speed", type=float, default=0.05,
                    help="default-panel linear speed (m/s)")
    ap.add_argument("--wz-max", type=float, default=0.3,
                    help="default-panel yaw rate (rad/s)")
    ap.add_argument("--episodes", type=int, default=2,
                    help="episodes per command per pass")
    ap.add_argument("--seconds", type=float, default=6.0,
                    help="command hold time per episode")
    ap.add_argument("--settle-s", type=float, default=1.0,
                    help="zero-command settle before each hold")
    ap.add_argument("--blend-skip-s", type=float, default=1.5,
                    help="seconds ignored after the command switch "
                         "(training blends commands up to 1 s)")
    ap.add_argument("--det-only", action="store_true",
                    help="skip the stochastic pass")
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V", help="env cfg overrides (own-cfg eval)")
    args = ap.parse_args()

    cmds: list[tuple[str, float, float, float]] = []
    if args.suite:
        for item in json.loads(args.suite.read_text()):
            if isinstance(item, dict):
                vx, vy, wz = (float(item.get("vx", 0.0)),
                              float(item.get("vy", 0.0)),
                              float(item.get("wz", 0.0)))
                cmds.append((str(item.get("name") or _cmd_name(vx, vy, wz)),
                             vx, vy, wz))
            else:
                vx, vy, wz = (float(item[0]), float(item[1]),
                              float(item[2]))
                cmds.append((_cmd_name(vx, vy, wz), vx, vy, wz))
    if args.cmd:
        for spec in args.cmd:
            vx, vy, wz = (float(x) for x in spec.split(","))
            cmds.append((_cmd_name(vx, vy, wz), vx, vy, wz))
    if not cmds:
        cmds = default_panel(args.speed, args.wz_max)

    from stable_baselines3 import PPO

    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    # Same cfg-before-construction rule as eval_checkpoint (cycle 11:
    # overrides can change obs WIDTH, baked in __init__).
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
                parsed = val.strip()
            cfg.setdefault(sect, {})[name] = parsed
        cfg_kw["cfg"] = cfg
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg_kw.get("cfg")),
        randomize=args.dr_scale > 0,
        dr_scale=args.dr_scale, episode_seconds=600.0, seed=args.seed,
        **cfg_kw)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        setattr(gen, f"p_{m}", 0.0)
    model = PPO.load(args.checkpoint, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} vs env "
        f"{env.observation_space.shape} — wrong --cfg-set for this ckpt?")

    pads = [env.model.body(f"L{i}_pad").id for i in range(6)]

    def run_episode(vx: float, vy: float, wz: float, det: bool,
                    seed: int) -> dict:
        obs, _ = env.reset(seed=seed)
        falls = 0
        verr, vxerr, vyerr, wzerr = [], [], [], []
        cur_meas: list[np.ndarray] = []
        contact_hist: list[list[bool]] = []
        pad_xy_hist: list[np.ndarray] = []
        pos0 = None
        t = 0.0
        n_settle = max(1, int(args.settle_s / env.dt))
        n_hold = max(1, int(args.seconds / env.dt))
        for i in range(n_settle + n_hold):
            in_hold = i >= n_settle
            cvx, cvy, cwz = (vx, vy, wz) if in_hold else (0.0, 0.0, 0.0)
            traj = env._goal_traj
            if traj is not None and hasattr(traj, "vx"):
                traj.vx[:] = cvx
                traj.vy[:] = cvy
                if getattr(traj, "wz", None) is not None:
                    traj.wz[:] = cwz
                elif abs(cwz) > 1e-9:
                    traj.wz = np.full_like(np.asarray(traj.vx), cwz)
            a, _ = model.predict(obs, deterministic=det)
            obs, _r, term, trunc, _info = env.step(a)
            t += env.dt
            measured = in_hold and (i - n_settle) * env.dt >= args.blend_skip_s
            if measured:
                if pos0 is None:
                    pos0 = env.data.xpos[env._chassis_bid, :2].copy()
                v = env._body_vel_xy()
                vxerr.append(abs(float(v[0]) - cvx))
                vyerr.append(abs(float(v[1]) - cvy))
                verr.append(math.hypot(float(v[0]) - cvx,
                                       float(v[1]) - cvy))
                wzerr.append(abs(env._body_wz() - cwz))
                st = env._state
                if st.servo_current is not None:
                    cur_meas.append(st.servo_current.copy())
                contact_hist.append([
                    float(env.data.sensordata[adr]) > CONTACT_N
                    for adr in env._touch_adr])
                pad_xy_hist.append(np.asarray(
                    [env.data.xpos[b][:2].copy() for b in pads]))
            if term or trunc:
                falls += 1
                obs, _ = env.reset()
                pos0 = None   # progress across a fall is meaningless

        # loaded-foot slip over the measured window (harness definition)
        slip_m = 0.0
        if len(pad_xy_hist) > 1:
            contact = np.asarray(contact_hist, dtype=bool)
            pad_xy = np.asarray(pad_xy_hist)
            for f in range(6):
                moved = np.linalg.norm(
                    np.diff(pad_xy[:, f], axis=0), axis=1)
                slip_m += float(moved[contact[:-1, f]].sum())
        # along-command progress (falls reset pos0; last unbroken run)
        s_ref = math.hypot(vx, vy)
        prog_m = 0.0
        if pos0 is not None and s_ref > 1e-3:
            d = env.data.xpos[env._chassis_bid, :2] - pos0
            prog_m = float(np.dot(d, [vx / s_ref, vy / s_ref]))
        cur = (np.asarray(cur_meas) if cur_meas
               else np.zeros((1, 18)))
        return {
            "falls": falls,
            "v_err_med": round(float(np.median(verr)), 4) if verr else None,
            "vx_err_med": (round(float(np.median(vxerr)), 4)
                           if vxerr else None),
            "vy_err_med": (round(float(np.median(vyerr)), 4)
                           if vyerr else None),
            "wz_err_med": (round(float(np.median(wzerr)), 4)
                           if wzerr else None),
            "slip_m": round(slip_m, 4),
            "prog_m": round(prog_m, 4),
            "slip_per_m": (round(slip_m / max(prog_m, 0.05), 3)
                           if s_ref > 1e-3 else None),
            "cur_mean_a": round(float(cur.mean()), 3),
            "cur_p95_a": round(float(np.percentile(cur, 95)), 3),
        }

    passes = ["det"] if args.det_only else ["det", "sto"]
    rows = []
    for name, vx, vy, wz in cmds:
        for p in passes:
            eps = [run_episode(vx, vy, wz, det=(p == "det"),
                               seed=args.seed + 1000 * e)
                   for e in range(args.episodes)]
            agg = {"cmd": name, "vx": vx, "vy": vy, "wz": wz, "pass": p,
                   "episodes": len(eps),
                   "falls": int(sum(e["falls"] for e in eps))}
            for k in ("v_err_med", "vx_err_med", "vy_err_med",
                      "wz_err_med", "slip_m", "prog_m", "slip_per_m",
                      "cur_mean_a", "cur_p95_a"):
                vals = [e[k] for e in eps if e[k] is not None]
                agg[k] = round(float(np.median(vals)), 4) if vals else None
            rows.append(agg)
            spm = ("-" if agg["slip_per_m"] is None
                   else f"{agg['slip_per_m']:.2f}")
            wzs = ("-" if agg["wz_err_med"] is None
                   else f"{agg['wz_err_med']:.3f}")
            print(f"{name:12s} [{p}] falls={agg['falls']} "
                  f"v_err={agg['v_err_med']} wz_err={wzs} "
                  f"slip/m={spm} cur={agg['cur_mean_a']}A")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(
            {"checkpoint": str(args.checkpoint),
             "dr_scale": args.dr_scale, "seed": args.seed,
             "episodes_per_cmd": args.episodes,
             "seconds": args.seconds,
             "blend_skip_s": args.blend_skip_s,
             "rows": rows}, indent=1))
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
