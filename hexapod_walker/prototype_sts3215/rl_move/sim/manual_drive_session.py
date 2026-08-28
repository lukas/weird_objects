"""Scripted human-like manual-drive session for ONE unified policy.

Operator request 2026-08-28 (MCP lane 20260828T140802Z): "load the best
current passed unified rising+walking policy into MuJoCo and drive it
around like a human would, then report how it feels." There is no
interactive viewer on the pods (drive_policy.py needs a cv2 window +
keyboard), so this is the sanctioned fallback: a SCRIPTED human-like
session, env-native (goal.mode_seq machinery, the same protocol the
policy trained and was gated on — no reanchor wrapper help).

Session (all in the trained SEQ_NEXT grammar rise->hold->walk->lower):

  flat belly start -> rise -> hold (settle, then height down/up play)
  -> walk driven by a hand-authored joystick script (gentle fwd, stop,
  crab, diagonal, gradual sweep, reverse, rapid stop/restart, a
  deliberately out-of-envelope half-speed probe) -> stop -> lower.

Command injection follows the drive_policy.py pattern: the walk
segment's WalkTrajectory arrays are overwritten ONCE at the segment
switch with the precomputed script (same obs slot / scaling as
training); hold heights likewise. Blend continuity at every mode
switch is preserved (script phases start/end at command zero).

Pure orchestration: no env/reward code, no shared defaults touched.

    uv run python -m rl_move.sim.manual_drive_session \
        rl_move/sim/policies/<ckpt>.zip --cfg-set k=v ... \
        [--seed 0] [--out-dir logs/manual_drive/X] [--no-video]
"""
from __future__ import annotations

import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

_PROTO = Path(__file__).resolve().parents[2]

SPEED = 0.08          # this lineage's exact trained speed (min=max=0.08)
DIAG = SPEED / math.sqrt(2.0)

# ---- session plan (seconds) -------------------------------------------
# Rise gets 20 s: measured (s1 det, seed 0) the policy lags its 6 s
# ramp by ~2x and needs ~14-16 s to actually reach stand height.
RISE_END_S = 20.0     # rise segment: flat belly -> stand
HOLD_END_S = 42.0     # hold segment: settle + gentle height play
WALK_END_S = 142.0    # walk segment: 100 s joystick script
EPISODE_S = 152.0     # lower runs to episode end
BLEND_S = 0.75

# Walk script: (t_rel_start_s, vx, vy, label). Blend 0.5 s between
# setpoints except phases marked instant (the rapid stop/restart run).
WALK_SCRIPT = [
    (0.0,   0.0,    0.0,   "settle-stop"),
    (4.0,   SPEED,  0.0,   "forward-cruise"),
    (18.0,  0.0,    0.0,   "full-stop"),
    (22.0,  0.0,   -SPEED, "crab-right"),
    (32.0,  DIAG,   DIAG,  "diag-fwd-left"),
    (42.0,  None,   None,  "sweep"),           # special: 18 s rotation
    (60.0,  0.0,    0.0,   "stop-after-sweep"),
    (64.0, -SPEED,  0.0,   "reverse"),
    (74.0,  0.0,    0.0,   "rapid-stop-1"),
    (75.5,  SPEED,  0.0,   "rapid-go-1"),
    (77.0,  0.0,    0.0,   "rapid-stop-2"),
    (78.5,  SPEED,  0.0,   "rapid-go-2"),
    (80.0,  SPEED,  0.0,   "forward-again"),
    (88.0,  0.04,   0.0,   "half-speed-probe (OUT-OF-ENVELOPE)"),
    (94.0,  0.0,    0.0,   "final-stop"),
]
SWEEP_START_S, SWEEP_END_S = 42.0, 60.0   # heading 45deg -> -135deg
RAPID = {"rapid-stop-1", "rapid-go-1", "rapid-stop-2", "rapid-go-2"}

# Hold-height script (t_rel_s, target_mm), rate-limited 15 mm/s (the
# trained hold_height_cmd_rate_mm_s); trained range [-40, +20] mm.
# Kept GENTLE (+-10 mm): the -40 mm hold_low_height kill line measures
# ABSOLUTE height vs the plant frame, and the s1 policy overshoots
# down-commands by ~20 mm (measured, seed 0) — a -20 mm command dies.
HOLD_HEIGHT_SCRIPT = [(0.0, 0.0), (8.0, -10.0), (12.0, -10.0),
                      (14.0, 10.0), (18.0, 10.0), (20.0, 0.0)]
HOLD_RATE_MM_S = 15.0


def build_walk_cmds(n: int, dt: float, seg_tick: int):
    """Full-episode-clock vx/vy arrays + per-tick label list."""
    vx = np.zeros(n)
    vy = np.zeros(n)
    labels = [""] * n
    events = []   # (tick, label, tgt_vx, tgt_vy) for latency scoring

    def tick_of(t_rel):
        return min(n - 1, seg_tick + int(round(t_rel / dt)))

    cur = (0.0, 0.0)
    for i, (t0, cvx, cvy, lab) in enumerate(WALK_SCRIPT):
        k0 = tick_of(t0)
        k1 = tick_of(WALK_SCRIPT[i + 1][0]) if i + 1 < len(WALK_SCRIPT) \
            else n
        if lab == "sweep":
            idx = np.arange(k0, k1)
            frac = (idx - k0) * dt / max(SWEEP_END_S - SWEEP_START_S, dt)
            theta = math.radians(45.0) - frac * math.radians(180.0)
            vx[k0:k1] = SPEED * np.cos(theta)
            vy[k0:k1] = SPEED * np.sin(theta)
            cur = (float(vx[k1 - 1]), float(vy[k1 - 1]))
            events.append((k0, lab, None, None))
        else:
            bl = 0 if lab in RAPID else int(round(0.5 / dt))
            kb = min(k0 + bl, k1)
            if bl:
                vx[k0:kb] = np.linspace(cur[0], cvx, kb - k0)
                vy[k0:kb] = np.linspace(cur[1], cvy, kb - k0)
            vx[kb:k1] = cvx
            vy[kb:k1] = cvy
            cur = (cvx, cvy)
            events.append((k0, lab, cvx, cvy))
        for k in range(k0, k1):
            labels[k] = lab
    vx[tick_of(WALK_SCRIPT[-1][0]):] = 0.0
    vy[tick_of(WALK_SCRIPT[-1][0]):] = 0.0
    return vx, vy, labels, events


def build_hold_height(n: int, dt: float, seg_tick: int):
    """Rate-limited height schedule (meters, plant-frame-relative)."""
    h = np.zeros(n)
    cur = 0.0
    step = HOLD_RATE_MM_S * dt   # mm per tick max
    targets = np.zeros(n)
    for i, (t0, mm) in enumerate(HOLD_HEIGHT_SCRIPT):
        k0 = min(n - 1, seg_tick + int(round(t0 / dt)))
        k1 = min(n, seg_tick + int(round(HOLD_HEIGHT_SCRIPT[i + 1][0] / dt))) \
            if i + 1 < len(HOLD_HEIGHT_SCRIPT) else n
        targets[k0:k1] = mm
    vals = np.zeros(n)
    for k in range(seg_tick, n):
        tgt = targets[k]
        cur += float(np.clip(tgt - cur, -step, step))
        vals[k] = cur
    h[seg_tick:] = vals[seg_tick:] / 1000.0
    return h


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt", type=Path)
    ap.add_argument("--cfg-set", action="append", default=[])
    # Default bridge: the s1 policy's cold FLAT rise is knife-edge
    # bistable (a 3e-8 action perturbation — BLAS kernel selection by
    # import order — flips full stand vs a stable ~40 mm half-risen
    # park, each branch reproducible across seeds/threads; measured
    # 08-28). Bridge/crouch/rsi starts are robust.
    ap.add_argument("--rise-start", default="bridge",
                    choices=["flat", "bridge", "crouch"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rise-s", type=float, default=RISE_END_S,
                    help="rise segment length (trained cadence: 8-12 s)")
    ap.add_argument("--hold-s", type=float, default=HOLD_END_S - RISE_END_S,
                    help="hold segment length; height script scales to it")
    # Mid-episode rise->hold entry reliably trips hold_min_load in this
    # branch (EMA restarts at 0 with no segment-local grace + the
    # policy leaves a foot unloaded through the switch; the isolated
    # gate shows the same terms 2/6 det, 4/6 sto). rise->walk is also
    # in the trained grammar.
    ap.add_argument("--skip-hold", action="store_true")
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--stochastic", action="store_true")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--render-every", type=int, default=4)
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()

    from rl_move.config import load_config
    from .train_ppo_sim import _parse_cfg_set
    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for key, parsed in _parse_cfg_set(args.cfg_set).items():
        sect, name = key.split(".", 1)
        cfg.setdefault(sect, {})[name] = parsed
    # Session forcing (eval-time only): every episode a sequence, clean
    # deterministic flat rise start (no RSI, no tipped spawn).
    cfg.setdefault("goal", {})["mode_seq"] = 1.0
    cfg["goal"]["rise_rsi_frac"] = 0.0
    cfg.setdefault("dr", {})["tipped_start_prob"] = 0.0

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg),
        randomize=args.dr_scale > 0, dr_scale=args.dr_scale,
        episode_seconds=EPISODE_S, seed=args.seed,
        render_mode=None if args.no_video else "rgb_array", cfg=cfg)
    gen = env._goal_gen
    gen.p_rise = 1.0
    for m in ("walk", "hold", "lean", "track", "unload", "raise", "lower",
              "recover"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 0.0)
    gen.force_rise_start = args.rise_start

    from .gru_policy import load_checkpoint_auto, wrap_recurrent_predictor
    model = load_checkpoint_auto(args.ckpt, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} vs env "
        f"{env.observation_space.shape} — pass the run's own cfg stack")
    # RUNNER BUG FIX (08-28, second operator kick 20260828T153954Z):
    # the first manual-drive report ran GRU checkpoints through the
    # stateless model.predict path — ZERO hidden state every tick, the
    # exact "memory-less lobotomy" eval_checkpoint._RecurrentPredictor
    # exists to prevent. Every feel finding measured before this line
    # landed evaluated that lobotomy, not the policy.
    model = wrap_recurrent_predictor(model)

    rise_end = args.rise_s
    hold_end = rise_end + args.hold_s
    walk_end = hold_end + (WALK_END_S - HOLD_END_S)
    # scale the hold height script into the requested hold window
    hh_scale = args.hold_s / (HOLD_END_S - RISE_END_S)
    global HOLD_HEIGHT_SCRIPT
    HOLD_HEIGHT_SCRIPT = [(t * hh_scale, mm) for t, mm in HOLD_HEIGHT_SCRIPT]

    obs, _ = env.reset()
    dt = env.dt
    n = env.episode_steps + 1

    def tk(s):
        return int(round(s / dt))

    bl = tk(BLEND_S)
    if args.skip_hold:
        hold_end = rise_end
        walk_end = hold_end + (WALK_END_S - HOLD_END_S)
        plan = [
            {"mode": "rise", "tick": 0, "blend": 0},
            {"mode": "walk", "tick": tk(hold_end), "blend": bl},
            {"mode": "lower", "tick": tk(walk_end), "blend": bl},
        ]
    else:
        plan = [
            {"mode": "rise", "tick": 0, "blend": 0},
            {"mode": "hold", "tick": tk(rise_end), "blend": bl},
            {"mode": "walk", "tick": tk(hold_end), "blend": bl},
            {"mode": "lower", "tick": tk(walk_end), "blend": bl},
        ]
    env._seq_plan = plan
    env._seq_idx = 0
    env._seq_seg_end = plan[1]["tick"]

    walk_tick = next(p["tick"] for p in plan if p["mode"] == "walk")
    walk_vx, walk_vy, walk_labels, walk_events = build_walk_cmds(
        n, dt, walk_tick)
    hold_h = build_hold_height(n, dt, plan[1]["tick"])

    out = args.out_dir or (_PROTO / "logs" / "manual_drive" /
                           f"{args.ckpt.stem}_{time.strftime('%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)

    frames = []
    rows = []
    seg_idx_prev = 0
    term_reason = None
    deterministic = not args.stochastic
    t_wall = time.time()
    for i in range(env.episode_steps):
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, _r, term, trunc, info = env.step(action)
        # segment switch bookkeeping (switch happens inside step)
        if env._seq_idx != seg_idx_prev:
            seg_idx_prev = env._seq_idx
            mode = plan[seg_idx_prev]["mode"]
            i0 = plan[seg_idx_prev]["tick"]
            traj = env._goal_traj
            if mode == "walk" and hasattr(traj, "vx"):
                traj.vx = walk_vx.copy()
                traj.vy = walk_vy.copy()
            elif mode == "hold":
                # keep the switch blend window, then the script
                traj.height[i0 + bl:] = hold_h[i0 + bl:]
        v = env.unwrapped._body_vel_xy()
        mode_now = plan[env._seq_idx]["mode"]
        step_i = env._step_i
        rows.append({
            "t": round(step_i * dt, 3), "mode": mode_now,
            "label": walk_labels[min(step_i, n - 1)]
            if mode_now == "walk" else mode_now,
            "cmd_vx": float(walk_vx[min(step_i, n - 1)])
            if mode_now == "walk" else 0.0,
            "cmd_vy": float(walk_vy[min(step_i, n - 1)])
            if mode_now == "walk" else 0.0,
            "act_vx": float(v[0]), "act_vy": float(v[1]),
            "height_mm": info.get("height_mm"),
            "height_ref_mm": info.get("height_ref_mm"),
            "mean_current_a": info.get("mean_current_a"),
            "max_current_a": info.get("max_current_a"),
            "slip_m": float(getattr(env, "_ls_slip_m", 0.0)),
            "prog_m": float(getattr(env, "_ls_prog_m", 0.0)),
        })
        if not args.no_video and i % args.render_every == 0:
            fr = env.render()
            try:
                import cv2
                fr = np.ascontiguousarray(fr)
                r0 = rows[-1]
                lab = r0["label"]
                txt = (f"t={r0['t']:6.1f}s  {mode_now.upper():5s}  {lab}  "
                       f"cmd({r0['cmd_vx']:+.2f},{r0['cmd_vy']:+.2f}) "
                       f"act({r0['act_vx']:+.2f},{r0['act_vy']:+.2f})")
                cv2.putText(fr, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (255, 255, 40), 1, cv2.LINE_AA)
            except Exception:
                pass
            frames.append(fr)
        if term or trunc:
            term_reason = info.get("termination_reason") if term else None
            break

    wall = time.time() - t_wall
    # ---- aggregate per phase ------------------------------------------
    phases = {}
    for r in rows:
        phases.setdefault(r["label"], []).append(r)
    summary = {"ckpt": str(args.ckpt), "seed": args.seed,
               "deterministic": deterministic,
               "dr_scale": args.dr_scale,
               "terminated": term_reason is not None,
               "termination_reason": term_reason,
               "sim_seconds": rows[-1]["t"] if rows else 0.0,
               "wall_seconds": round(wall, 1), "phases": []}
    for lab, rs in phases.items():
        cvx = np.mean([r["cmd_vx"] for r in rs])
        cvy = np.mean([r["cmd_vy"] for r in rs])
        avx = np.mean([r["act_vx"] for r in rs])
        avy = np.mean([r["act_vy"] for r in rs])
        cmd_sp = math.hypot(cvx, cvy)
        act_sp = math.hypot(avx, avy)
        dir_err = None
        if cmd_sp > 0.01 and act_sp > 0.005:
            dir_err = round(math.degrees(abs(math.atan2(
                cvx * avy - cvy * avx, cvx * avx + cvy * avy))), 1)
        hs = [r["height_mm"] for r in rs if r["height_mm"] is not None]
        cs = [r["mean_current_a"] for r in rs
              if r["mean_current_a"] is not None]
        summary["phases"].append({
            "label": lab, "t0": rs[0]["t"], "t1": rs[-1]["t"],
            "cmd_speed": round(cmd_sp, 3),
            "act_speed_mean": round(float(np.mean(
                [math.hypot(r["act_vx"], r["act_vy"]) for r in rs])), 4),
            "along_cmd_mean": round(float(
                (avx * cvx + avy * cvy) / cmd_sp), 4) if cmd_sp > 0.01
            else None,
            "dir_err_deg": dir_err,
            "height_mm_mean": round(float(np.mean(hs)), 1) if hs else None,
            "height_mm_minmax": [round(float(np.min(hs)), 1),
                                 round(float(np.max(hs)), 1)] if hs else None,
            "current_a_mean": round(float(np.mean(cs)), 2) if cs else None,
        })
    # latency per command change: time to reach 50 % of the new command
    # along its direction (or |v|<0.02 for stops)
    lat = []
    for (k0, lab, tvx, tvy) in walk_events:
        if tvx is None:
            continue
        sp = math.hypot(tvx, tvy)
        t0 = k0 * dt
        found = None
        for r in rows:
            if r["t"] < t0:
                continue
            if r["t"] > t0 + 6.0:
                break
            if sp < 0.01:
                if math.hypot(r["act_vx"], r["act_vy"]) < 0.02:
                    found = round(r["t"] - t0, 2)
                    break
            else:
                along = (r["act_vx"] * tvx + r["act_vy"] * tvy) / sp
                if along >= 0.5 * sp:
                    found = round(r["t"] - t0, 2)
                    break
        lat.append({"label": lab, "t_cmd": round(t0, 1),
                    "latency_s": found})
    summary["latency"] = lat
    # walk-segment slip
    wrs = [r for r in rows if r["mode"] == "walk"]
    if wrs:
        summary["walk_slip_m"] = round(wrs[-1]["slip_m"], 3)
        summary["walk_prog_m"] = round(wrs[-1]["prog_m"], 3)
        summary["walk_slip_per_m"] = round(
            wrs[-1]["slip_m"] / max(wrs[-1]["prog_m"], 0.05), 2)

    (out / "session.json").write_text(json.dumps(summary, indent=2))
    (out / "ticks.json").write_text(json.dumps(rows[::10]))
    if frames:
        from .eval_checkpoint import _save_video
        _save_video(frames, out / "drive.mp4")
    print(json.dumps(summary, indent=2))
    print(f"[manual_drive] artifacts in {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
