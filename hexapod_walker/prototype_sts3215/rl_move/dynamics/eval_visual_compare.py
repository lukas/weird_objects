"""Matched visual A/B comparison of two (or more) walk checkpoints.

Operator request fb_20260818T022818_d54f8e: compare the frozen-critic-D
40M run's saved best-locomotion checkpoint against its newest periodic
checkpoint by DRIVING BOTH through the identical command-rich scripted
sequence in the exact trained env/obs contract (train_ppo_transfer
make_task_env: walk task, hist16, same --goal-set keys), deterministic
actions, identical fresh seeds, and WATCHING the result — not CSV-only.

Per (checkpoint, dr_scale, seed) it runs ONE long episode through a
fixed segment schedule (settle, fwd, +45deg diag, lateral, backward,
stop, restart, abrupt fwd->back flip; NO commanded yaw — this policy
has no yaw channel), forcing the command into the episode's
WalkTrajectory arrays every tick exactly like eval_drive/eval_cmd_suite
so the policy sees commands the way training delivered them. A fall
(termination) is recorded with its segment, the env resets (same-seed
family) and the schedule continues, so videos stay time-synchronized.

Outputs under --out:
    <label>_dr<D>_s<S>.mp4         25 fps follow-cam video, one per ep
    <label>_dr<D>_s<S>_strip.png   1 fps contact-sheet strip
    sbs_dr<D>_s<S>.mp4             side-by-side (label order) sync video
    report.json                    per-segment + per-episode metrics
    summary.txt                    A-vs-B table, human-readable

Per segment: cmd, achieved mean speed, progress along command (m and
fraction of commanded distance), net cross-track drift (m), loaded-foot
slip (m and per progress meter), peak/tail roll vs episode tilt ref,
falls, contact switches/s + feet cycling, slew saturation (per-joint
and >=6-joints-simultaneous), mean chassis height, servo current
mean/p95. Episode-level aggregate feeds the trainer's pre-registered
locomotion_quality composite for a headline number.

    python3 -m rl_move.dynamics.eval_visual_compare \
        --ckpt best6M=models/ppo_..._best.zip \
        --ckpt ck22M=models/ppo_..._ck22000000.zip \
        --dr-scales 0,0.3 --seeds 100,101,102 \
        --goal-set walk_cmd_resample_s=4.0 \
        --goal-set walk_cmd_resample_jitter=0.5 \
        --goal-set walk_stop_frac=0.15 \
        --out logs/ckpt_eval/viscompare1

Eval-only probe code: no training semantics touched.
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np

FPS = 25


def segments(speed: float, smoke: bool = False) -> list[tuple[str, float, float, float]]:
    """(name, seconds, vx, vy) — the fixed command-rich schedule."""
    s = speed
    t = 1.0 if smoke else 5.0
    return [
        ("settle",      1.0 if smoke else 1.5, 0.0, 0.0),
        ("fwd",         t,  s,          0.0),
        ("diag45",      t,  s * 0.7071, s * 0.7071),
        ("lat-left",    t,  0.0,        s),
        ("back",        t, -s,          0.0),
        ("stop",        1.0 if smoke else 3.0, 0.0, 0.0),
        ("restart-fwd", 1.0 if smoke else 4.0, s, 0.0),
        ("flip-back",   1.0 if smoke else 4.0, -s, 0.0),
    ]


def _md5(p: Path) -> str:
    h = hashlib.md5()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_episode(env, model, sched, video_path: Path | None,
                strip_path: Path | None, seed: int) -> dict:
    """One deterministic pass through the schedule; returns metrics."""
    import mujoco
    sids = [mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE,
                              f"L{i}_foot_site") for i in range(6)]
    rad2deg = 180.0 / math.pi
    obs, _ = env.reset(seed=seed)
    tr = getattr(env, "_tilt_ref0", (0.0, 0.0))
    sat_limit = 0.98 * env.safety.max_dq
    prev_cmd = env.safety._last_safe.copy()

    writer = None
    if video_path is not None:
        import imageio
        writer = imageio.get_writer(video_path, fps=FPS,
                                    macro_block_size=1)
    strip: list = []
    next_strip_t = 0.0

    seg_rows = []
    falls_total = 0
    t_global = 0.0
    # episode-level accumulators (for locomotion_quality)
    ep = dict(vx_se=0.0, vy_se=0.0, v_n=0, prog=0.0, cmd_dist=0.0,
              slip=0.0, sw=0, sat_jt=0, sat_all=0, n=0,
              peak_roll=0.0)
    for name, seconds, vx, vy in sched:
        n_ticks = max(1, int(round(seconds / env.dt)))
        prev_on: list[bool] | None = None
        prev_xy: list = [None] * 6
        sw = sat_jt = sat_all = 0
        slip = 0.0
        rolls: list[float] = []
        speeds: list[float] = []
        vx_se = vy_se = 0.0
        prog = xtrack = 0.0
        h_sum = 0.0
        cur: list[np.ndarray] = []
        duty = np.zeros(6)
        sw_per_foot = np.zeros(6, dtype=int)
        falls_seg = 0
        s_ref = math.hypot(vx, vy)
        u = (vx / s_ref, vy / s_ref) if s_ref > 1e-9 else (0.0, 0.0)
        for _ in range(n_ticks):
            traj = env._goal_traj
            if traj is not None and hasattr(traj, "vx"):
                traj.vx[:] = vx
                traj.vy[:] = vy
                wzr = getattr(traj, "wz", None)
                if wzr is not None:
                    wzr[:] = 0.0
            act, _ = model.predict(obs, deterministic=True)
            obs, _r, term, trunc, _info = env.step(act)
            t_global += env.dt
            st = env._state
            rolls.append(abs(st.imu_roll - tr[0]) * rad2deg)
            v = env._body_vel_xy()
            vmx, vmy = float(v[0]), float(v[1])
            speeds.append(math.hypot(vmx, vmy))
            vx_se += (vmx - vx) ** 2
            vy_se += (vmy - vy) ** 2
            if s_ref > 1e-9:
                prog += (vmx * u[0] + vmy * u[1]) * env.dt
                xtrack += (-vmx * u[1] + vmy * u[0]) * env.dt
            h_sum += float(env.data.xpos[env._chassis_bid, 2])
            sc = getattr(st, "servo_current", None)
            if sc is not None:
                cur.append(np.abs(np.asarray(sc, dtype=float)))
            cmdv = env.safety._last_safe
            n_sat = int(np.sum(np.abs(cmdv - prev_cmd) >= sat_limit))
            sat_jt += n_sat
            sat_all += int(n_sat >= 6)
            prev_cmd = cmdv.copy()
            on: list[bool] = []
            for f in range(6):
                adr = env._touch_adr[f]
                is_on = bool(adr >= 0
                             and float(env.data.sensordata[adr]) > 0.5)
                xy = (env.data.site_xpos[sids[f], :2].copy()
                      if sids[f] >= 0 else None)
                if prev_on is not None:
                    if is_on != prev_on[f]:
                        sw += 1
                        sw_per_foot[f] += 1
                    if (is_on and prev_on[f] and xy is not None
                            and prev_xy[f] is not None):
                        slip += float(np.hypot(*(xy - prev_xy[f])))
                prev_xy[f] = xy
                duty[f] += float(is_on)
                on.append(is_on)
            prev_on = on
            if writer is not None:
                frame = env.render()
                writer.append_data(frame)
                if strip_path is not None and t_global >= next_strip_t:
                    strip.append(frame)
                    next_strip_t += 1.0
            if term or trunc:
                falls_seg += int(bool(term))
                falls_total += int(bool(term))
                obs, _ = env.reset(seed=seed + 1000 * falls_total)
                tr = getattr(env, "_tilt_ref0", tr)
                prev_cmd = env.safety._last_safe.copy()
                prev_on = None
                prev_xy = [None] * 6
        n = n_ticks
        seg_t = n * env.dt
        tail_n = max(1, int(round(1.0 / env.dt)))
        cur_a = (np.concatenate(cur) if cur else np.zeros(1))
        cmd_dist = s_ref * seg_t
        seg_rows.append({
            "segment": name, "vx_cmd": vx, "vy_cmd": vy,
            "seconds": round(seg_t, 2),
            "speed_mean": round(float(np.mean(speeds)), 4),
            "prog_m": round(prog, 4),
            "cmd_prog_frac": (round(prog / cmd_dist, 3)
                              if cmd_dist > 1e-6 else None),
            "xtrack_m": round(xtrack, 4),
            "vxy_rmse": round(math.hypot(math.sqrt(vx_se / n),
                                         math.sqrt(vy_se / n)), 4),
            "slip_m": round(slip, 4),
            "slip_per_m": (round(slip / max(prog, 0.05), 2)
                           if s_ref > 1e-9 else None),
            "peak_roll_deg": round(float(np.max(rolls)), 1),
            "tail_roll_deg": round(float(np.mean(rolls[-tail_n:])), 2),
            "falls": falls_seg,
            "contact_sw_per_s": round(sw / seg_t, 2),
            "feet_cycling": int(np.sum(sw_per_foot >= 2)),
            "duty": [round(float(d) / n, 2) for d in duty],
            "slew_sat": round(sat_jt / (n * 18), 3),
            "slew_sat_all": round(sat_all / n, 3),
            "mean_h_m": round(h_sum / n, 4),
            "cur_mean_a": round(float(cur_a.mean()), 3),
            "cur_p95_a": round(float(np.percentile(cur_a, 95)), 3),
        })
        # episode-level (moving + stop segments all count, like training)
        ep["vx_se"] += vx_se
        ep["vy_se"] += vy_se
        ep["v_n"] += n
        ep["prog"] += prog
        ep["cmd_dist"] += cmd_dist
        ep["slip"] += slip
        ep["sw"] += sw
        ep["sat_jt"] += sat_jt
        ep["sat_all"] += sat_all
        ep["n"] += n
        ep["peak_roll"] = max(ep["peak_roll"], float(np.max(rolls)))
    if writer is not None:
        writer.close()
    if strip_path is not None and strip:
        import imageio.v2 as iio2
        iio2.imwrite(strip_path, np.hstack(strip))

    from rl_move.dynamics.train_ppo_transfer import locomotion_quality
    ep_secs = ep["n"] * env.dt
    ep_metrics = {
        "early_term_rate": falls_total / max(len(sched), 1),
        "cmd_prog_frac": ep["prog"] / max(ep["cmd_dist"], 1e-6),
        "vx_rmse": math.sqrt(ep["vx_se"] / ep["v_n"]),
        "vy_rmse": math.sqrt(ep["vy_se"] / ep["v_n"]),
        "wz_rmse_dps": 0.0,   # no yaw channel on this lineage
        "slip_per_m": ep["slip"] / max(ep["prog"], 0.05),
        "peak_roll_deg": ep["peak_roll"],
        "slew_sat": ep["sat_jt"] / max(ep["n"] * 18, 1),
        "contact_sw_per_s": ep["sw"] / max(ep_secs, 1e-9),
    }
    return {"falls": falls_total,
            "loco_quality": round(locomotion_quality(ep_metrics), 2),
            "episode": {k: (round(v, 4) if isinstance(v, float) else v)
                        for k, v in ep_metrics.items()},
            "segments": seg_rows}


def side_by_side(paths: list[Path], labels: list[str], out: Path) -> None:
    """Streamed hstack of same-length MP4s with a label banner."""
    import imageio
    readers = [imageio.get_reader(p) for p in paths]
    writer = imageio.get_writer(out, fps=FPS, macro_block_size=1)
    try:
        import cv2
    except Exception:
        cv2 = None
    for frames in zip(*(r.iter_data() for r in readers)):
        row = np.hstack(frames)
        banner = np.zeros((28, row.shape[1], 3), dtype=np.uint8)
        if cv2 is not None:
            w = frames[0].shape[1]
            for i, lab in enumerate(labels):
                cv2.putText(banner, lab, (i * w + 8, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 1, cv2.LINE_AA)
        writer.append_data(np.vstack([banner, row]))
    for r in readers:
        r.close()
    writer.close()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ckpt", action="append", required=True,
                    metavar="LABEL=PATH")
    ap.add_argument("--dr-scales", default="0,0.3")
    ap.add_argument("--seeds", default="100,101,102")
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--goal-set", action="append", default=None,
                    metavar="K=V", help="goal.* cfg overrides — pass the "
                    "run's OWN training keys (own-cfg rule)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--smoke", action="store_true",
                    help="1 s segments, first seed/dr only, no strict use")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--encoder", type=Path, default=None,
                    help="local path of the frozen dynamics encoder .pt "
                    "— overrides the absolute pod path baked into a "
                    "condition-D checkpoint's policy_kwargs (critic "
                    "only; actions never touch it)")
    args = ap.parse_args()

    from rl_move.dynamics.train_ppo_transfer import make_task_env

    ckpts: list[tuple[str, Path]] = []
    for spec in args.ckpt:
        lab, _, p = spec.partition("=")
        ckpts.append((lab, Path(p)))
    drs = [float(x) for x in args.dr_scales.split(",")]
    seeds = [int(x) for x in args.seeds.split(",")]
    if args.smoke:
        drs, seeds = drs[:1], seeds[:1]
    goal_set = None
    if args.goal_set:
        goal_set = {}
        for spec in args.goal_set:
            k, _, v = spec.partition("=")
            try:
                goal_set[k] = float(v)
            except ValueError:
                goal_set[k] = v
    sched = segments(args.speed, smoke=args.smoke)
    args.out.mkdir(parents=True, exist_ok=True)

    from stable_baselines3 import PPO

    def load_ppo(p: Path):
        if args.encoder is None:
            return PPO.load(p, device="cpu")
        from stable_baselines3.common.save_util import load_from_zip_file
        data, _, _ = load_from_zip_file(p, device="cpu")
        pk = dict(data.get("policy_kwargs") or {})
        if "predictor_ckpt" in pk:
            pk["predictor_ckpt"] = str(args.encoder)
        return PPO.load(p, device="cpu",
                        custom_objects={"policy_kwargs": pk})

    report: dict = {
        "request": "fb_20260818T022818_d54f8e",
        "speed": args.speed, "dr_scales": drs, "seeds": seeds,
        "goal_set": goal_set,
        "schedule": [dict(zip(("segment", "seconds", "vx", "vy"), s))
                     for s in sched],
        "checkpoints": {lab: {"path": str(p), "md5": _md5(p),
                              "size": p.stat().st_size}
                        for lab, p in ckpts},
        "episodes": []}
    models = {}
    ep_secs = sum(s[1] for s in sched)
    for dr in drs:
        for seed in seeds:
            vids, labs = [], []
            for lab, p in ckpts:
                env = make_task_env("walk", seed, dr, ep_secs + 30.0,
                                    goal_set=goal_set)
                env.render_mode = None if args.no_video else "rgb_array"
                if lab not in models:
                    models[lab] = load_ppo(p)
                    assert (models[lab].observation_space.shape
                            == env.observation_space.shape), (
                        f"obs mismatch {lab}: policy "
                        f"{models[lab].observation_space.shape} vs env "
                        f"{env.observation_space.shape}")
                tag = f"{lab}_dr{dr:g}_s{seed}"
                vp = None if args.no_video else args.out / f"{tag}.mp4"
                sp = (None if args.no_video
                      else args.out / f"{tag}_strip.png")
                t0 = time.time()
                m = run_episode(env, models[lab], sched, vp, sp, seed)
                env.close()
                m.update(label=lab, dr_scale=dr, seed=seed,
                         video=str(vp) if vp else None,
                         wall_s=round(time.time() - t0, 1))
                report["episodes"].append(m)
                print(f"[{tag}] falls={m['falls']} "
                      f"loco_quality={m['loco_quality']} "
                      f"({m['wall_s']}s)", flush=True)
                if vp is not None:
                    vids.append(vp)
                    labs.append(lab)
            if len(vids) >= 2:
                sbs = args.out / f"sbs_dr{dr:g}_s{seed}.mp4"
                side_by_side(vids, labs, sbs)
                print(f"  side-by-side -> {sbs}", flush=True)

    (args.out / "report.json").write_text(json.dumps(report, indent=1))

    # human summary: per (dr, seed, segment) A|B columns
    lines = []
    labs = [lab for lab, _ in ckpts]
    keys = ("cmd_prog_frac", "speed_mean", "xtrack_m", "slip_per_m",
            "peak_roll_deg", "tail_roll_deg", "falls",
            "contact_sw_per_s", "feet_cycling", "slew_sat",
            "mean_h_m", "cur_mean_a")
    for dr in drs:
        for seed in seeds:
            eps = {e["label"]: e for e in report["episodes"]
                   if e["dr_scale"] == dr and e["seed"] == seed}
            lines.append(f"== dr{dr:g} seed{seed} — loco_quality: " +
                         "  ".join(f"{l}={eps[l]['loco_quality']}"
                                   for l in labs))
            for i, seg in enumerate(sched):
                name = seg[0]
                row = [f"  {name:12s}"]
                for k in keys:
                    vals = [eps[l]["segments"][i].get(k) for l in labs]
                    row.append(f"{k}=" + "|".join(str(v) for v in vals))
                lines.append(" ".join(row))
    (args.out / "summary.txt").write_text("\n".join(lines) + "\n")
    print(f"WROTE {args.out}/report.json + summary.txt", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
