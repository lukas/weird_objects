"""probe_dirswitch_tangle.py — why does sustained joystick steering tangle
the tall walker's legs? (operator fb_20260818T152717_278879)

The strong tall walker `cw-dep-bcgait1-hard1` trained on 15 s episodes
with ONE fixed velocity command each — no mid-episode command changes.
Driving it around by joystick (rot60 wrapper ON by default) eventually
tangles the legs and it falls. Two candidate triggers:

  (a) rot60 SECTOR CROSSINGS: changing the commanded heading across a
      hexagon sector boundary relabels the leg permutation in one tick —
      obs and action maps jump discontinuously while the physical legs
      stay where they were. Mid-gait that hands the policy a leg
      configuration it can never see from its own on-policy state
      distribution.
  (b) PLAIN COMMAND SWITCHES: any abrupt command change is an untrained
      transition; accumulated yaw-limit saturation / phase error tangles
      the legs regardless of the wrapper.

This probe runs MATCHED long direction-switch schedules (same seed →
identical command arrays) with rot60 ON vs OFF and logs, per tick:
command-change ticks, rot60 sector k (shadow-computed when OFF so event
alignment is identical), per-leg ground contact, per-leg yaw-limit
margin (deg to nearest hard limit), min inter-foot pad distance + min
adjacent foot azimuth gap (leg-crossing proxies), and the fall tick.
If falls cluster within a couple seconds of SECTOR changes only when
rot60 is ON → (a); if ON≈OFF and falls follow plain switches / creeping
yaw saturation → (b).

Schedules (families, all abrupt, no blending): random full-circle
steering, 180-deg back-and-forth, continuous circle sweep, square,
stop/go, heading jitter. Dwells drawn irregularly in ~2-20 s.

DR-0, deterministic, flat spawn (tipped starts off — the question is
steering, not takeoff). Env cfg = hard1's exact eval stack.

    uv run python -m rl_move.sim.probe_dirswitch_tangle \
        rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip \
        --seconds 120 --out logs/probe_dirswitch/hard1.json
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
from pathlib import Path

import numpy as np

# hard1's exact eval stack (ops.sh evalcmd cw-dep-bcgait1-hard1), minus
# tipped starts (probe is about steering, not takeoff).
HARD1_CFG = {
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
    ("reward", "walk_height_gate"): 1.0,
    ("reward", "walk_height_sigma_mm"): 30.0,
    ("dr", "tipped_start_prob"): 0.0,
}

FAMILIES = ("random", "flip180", "circle", "square", "stopgo", "jitter")
S_LO, S_HI = 0.05, 0.06        # hard1's trained speed band
DWELL_LO, DWELL_HI = 2.0, 20.0
YAW_SAT_DEG = 2.0              # "saturated" margin threshold
NEAR_S = 2.0                   # fall-near-event window (seconds)


def build_schedule(family: str, rng: np.random.Generator, n: int,
                   dt: float) -> tuple[np.ndarray, np.ndarray, list[int]]:
    """(vx, vy, change_ticks) — abrupt, irregular-dwell command arrays."""
    vx = np.zeros(n)
    vy = np.zeros(n)
    changes: list[int] = []
    hold = max(1, int(round(1.0 / dt)))          # 1 s settle at zero cmd

    def dwell(lo=DWELL_LO, hi=DWELL_HI) -> int:
        return max(1, int(round(float(rng.uniform(lo, hi)) / dt)))

    def spd() -> float:
        return float(rng.uniform(S_LO, S_HI))

    if family == "circle":
        period = float(rng.uniform(15.0, 30.0))
        sign = -1.0 if rng.random() < 0.5 else 1.0
        ang0 = float(rng.uniform(-math.pi, math.pi))
        s = spd()
        idx = np.arange(n - hold, dtype=float)
        th = ang0 + sign * 2.0 * math.pi * idx * dt / period
        vx[hold:] = s * np.cos(th)
        vy[hold:] = s * np.sin(th)
        changes.append(hold)                     # continuous after start
        return vx, vy, changes

    i = hold
    if family == "flip180":
        ang = float(rng.uniform(-math.pi, math.pi))
    elif family == "square":
        ang = float(rng.uniform(-math.pi, math.pi))
        sq_sign = -1.0 if rng.random() < 0.5 else 1.0
    elif family == "jitter":
        ang = float(rng.uniform(-math.pi, math.pi))
    go = True
    first = True
    while i < n:
        if family == "random":
            if not first and rng.random() < 0.15:
                cvx = cvy = 0.0
            else:
                ang = float(rng.uniform(-math.pi, math.pi))
                s = spd()
                cvx, cvy = s * math.cos(ang), s * math.sin(ang)
        elif family == "flip180":
            if not first:
                ang += math.pi
            s = spd()
            cvx, cvy = s * math.cos(ang), s * math.sin(ang)
        elif family == "square":
            if not first:
                ang += sq_sign * math.pi / 2.0
            s = spd()
            cvx, cvy = s * math.cos(ang), s * math.sin(ang)
        elif family == "stopgo":
            if go:
                ang = float(rng.uniform(-math.pi, math.pi))
                s = spd()
                cvx, cvy = s * math.cos(ang), s * math.sin(ang)
            else:
                cvx = cvy = 0.0
            go = not go
        else:                                    # jitter
            if not first:
                ang += float(rng.uniform(-0.25, 0.25))
            s = spd()
            cvx, cvy = s * math.cos(ang), s * math.sin(ang)
        d = dwell(2.0, 8.0) if family == "jitter" else dwell()
        if family == "stopgo" and cvx == 0.0 and cvy == 0.0:
            d = dwell(2.0, 6.0)
        vx[i:i + d] = cvx
        vy[i:i + d] = cvy
        changes.append(i)
        i += d
        first = False
    return vx, vy, changes


def run_episode(env, model, rot_on: bool, vx: np.ndarray, vy: np.ndarray,
                changes: list[int], tilt_scale: float,
                deterministic: bool = True,
                ep_seed: int | None = None) -> dict:
    import mujoco

    from .rot60 import Rot60Policy, sector_from_cmd

    # ep_seed re-seeds the env so a rot60 ON/OFF pair draws the SAME
    # DR physics + spawn — matched-pair comparison under randomize.
    obs, _ = env.reset(seed=ep_seed)
    pol = Rot60Policy(model, tilt_scale=tilt_scale) if rot_on else model
    if hasattr(pol, "reset"):
        pol.reset()
    traj = env._goal_traj
    n = min(len(vx), len(traj.vx))
    traj.vx[:n] = vx[:n]
    traj.vy[:n] = vy[:n]
    traj.vx[n:] = vx[n - 1]
    traj.vy[n:] = vy[n - 1]
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0

    m = env.model
    yaw_j = []
    for leg in range(6):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"L{leg}_yaw")
        yaw_j.append((int(m.jnt_qposadr[j]),
                      float(np.degrees(m.jnt_range[j][0])),
                      float(np.degrees(m.jnt_range[j][1]))))
    home_az = np.radians((np.arange(6) + 0.5) * 60.0)

    shadow_k = 0
    ks: list[int] = []
    sector_changes: list[int] = []
    yaw_min_t: list[float] = []                  # min-over-legs margin
    contact_t: list[int] = []                    # bitmask
    foot_min_t: list[float] = []                 # min pairwise pad dist mm
    azgap_min_t: list[float] = []                # min adjacent azimuth gap deg
    verr_t: list[float] = []
    swings = [0] * 6
    prev_c = [True] * 6
    fall_tick = -1
    ticks = n
    for i in range(n):
        a, _ = pol.predict(obs, deterministic=deterministic)
        obs, _r, term, trunc, _info = env.step(a)
        k = pol.k if rot_on else sector_from_cmd(
            float(vx[i]), float(vy[i]), shadow_k)
        shadow_k = k
        if ks and k != ks[-1]:
            sector_changes.append(i)
        ks.append(k)
        margins = [min(v - lo, hi - v) for adr, lo, hi in yaw_j
                   for v in (float(np.degrees(env.data.qpos[adr])),)]
        yaw_min_t.append(min(margins))
        body = env.data.xpos[env._chassis_bid]
        R = env.data.xmat[env._chassis_bid].reshape(3, 3)
        cmask = 0
        pads = []
        for f in range(6):
            adr = env._touch_adr[f]
            on = adr >= 0 and float(env.data.sensordata[adr]) > 0.5
            if on:
                cmask |= 1 << f
            if on and not prev_c[f]:
                pass
            if prev_c[f] and not on:
                swings[f] += 1
            prev_c[f] = on
            pads.append(R.T @ (env.data.xpos[env._pad_bids[f]] - body))
        contact_t.append(cmask)
        pads = np.asarray(pads)
        dmin = min(float(np.hypot(*(pads[a2][:2] - pads[b2][:2])))
                   for a2 in range(6) for b2 in range(a2 + 1, 6))
        foot_min_t.append(dmin * 1000.0)
        az = np.arctan2(pads[:, 1], pads[:, 0])
        gaps = np.degrees((az[(np.arange(6) + 1) % 6] - az) % (2 * math.pi))
        azgap_min_t.append(float(gaps.min()))
        v = env._body_vel_xy()
        verr_t.append(float(math.hypot(v[0] - vx[i], v[1] - vy[i])))
        if term:
            fall_tick = i
            ticks = i + 1
            break
        if trunc:
            ticks = i + 1
            break

    dt = env.dt
    fell = fall_tick >= 0

    def last_before(events: list[int], t: int) -> float:
        prior = [e for e in events if e <= t]
        return (t - prior[-1]) * dt if prior else float("inf")

    yaw_min = np.asarray(yaw_min_t)
    # per-command-change: legs still cycling + tracking recovery
    swing_after, recover_s = [], []
    for c in changes[1:]:
        if c >= ticks:
            continue
        w0, w1 = c, min(c + int(2.5 / dt), ticks)
        legs_sw = 0
        for f in range(6):
            trans = sum(1 for t2 in range(w0 + 1, w1)
                        if (contact_t[t2 - 1] >> f & 1)
                        and not (contact_t[t2] >> f & 1))
            legs_sw += trans > 0
        swing_after.append(legs_sw)
        rec = next((t2 for t2 in range(c, ticks)
                    if verr_t[t2] < 0.03), None)
        if rec is not None:
            recover_s.append((rec - c) * dt)
    return {
        "rot60": rot_on,
        "seconds": round(ticks * dt, 1),
        "fell": fell,
        "fall_t_s": round(fall_tick * dt, 2) if fell else None,
        "fall_dt_cmd_change_s": (round(last_before(changes, fall_tick), 2)
                                 if fell else None),
        "fall_dt_sector_change_s": (
            round(last_before(sector_changes, fall_tick), 2)
            if fell else None),
        "n_cmd_changes": len([c for c in changes if c < ticks]),
        "n_sector_changes": len(sector_changes),
        "yaw_margin_min_deg": round(float(yaw_min.min()), 2),
        "yaw_margin_mean_deg": round(float(yaw_min.mean()), 2),
        "yaw_sat_frac": round(float((yaw_min < YAW_SAT_DEG).mean()), 4),
        "foot_dist_min_mm": round(min(foot_min_t), 1),
        "azgap_min_deg": round(min(azgap_min_t), 1),
        "legs_cycling_after_change_min": (min(swing_after)
                                          if swing_after else None),
        "track_recover_s_mean": (round(float(np.mean(recover_s)), 2)
                                 if recover_s else None),
        "swings_per_leg": swings,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--families", default=",".join(FAMILIES))
    ap.add_argument("--seeds", type=int, default=2,
                    help="schedules per family (each run rot60 ON + OFF)")
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--dr-scale", type=float, default=0.0,
                    help="own-DR physics randomization (matched draw "
                         "per rot60 ON/OFF pair via per-episode seed)")
    ap.add_argument("--stochastic", action="store_true",
                    help="sample actions instead of deterministic")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V")
    args = ap.parse_args()

    from stable_baselines3 import PPO

    from rl_move.config import cfg_get, load_config

    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sect, name), val in HARD1_CFG.items():
        cfg.setdefault(sect, {})[name] = val
    if args.cfg_set:
        for spec in args.cfg_set:
            key, val = spec.split("=", 1)
            sect, name = key.split(".", 1)
            try:
                parsed: float | str = float(val)
            except ValueError:
                parsed = val.strip()
            cfg.setdefault(sect, {})[name] = parsed

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg),
        randomize=args.dr_scale > 0.0,
        dr_scale=args.dr_scale, episode_seconds=args.seconds + 5.0,
        seed=args.seed0, cfg=cfg)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for mm in ("hold", "lean", "track", "unload", "raise", "rise",
               "lower"):
        setattr(gen, f"p_{mm}", 0.0)
    model = PPO.load(args.checkpoint, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} vs env "
        f"{env.observation_space.shape}")
    ts = float(cfg_get(env.cfg, "obs", "tilt_scale", default=0.2))

    n = int(round(args.seconds / env.dt))
    episodes = []
    fams = [f.strip() for f in args.families.split(",") if f.strip()]
    for fam in fams:
        for s in range(args.seeds):
            seed = args.seed0 + 1000 + 17 * s
            rng = np.random.default_rng(abs(hash((fam, seed))) % 2**31)
            vx, vy, changes = build_schedule(fam, rng, n, env.dt)
            for rot_on in (True, False):
                r = run_episode(env, model, rot_on, vx, vy, changes, ts,
                                deterministic=not args.stochastic,
                                ep_seed=seed)
                r["family"] = fam
                r["sched_seed"] = seed
                episodes.append(r)
                print(f"[{fam}:{s} rot60={'ON ' if rot_on else 'off'}] "
                      f"fell={r['fell']} t={r['seconds']}s "
                      f"dt_cmd={r['fall_dt_cmd_change_s']} "
                      f"dt_sec={r['fall_dt_sector_change_s']} "
                      f"yaw_min={r['yaw_margin_min_deg']} "
                      f"sat={r['yaw_sat_frac']} "
                      f"foot_min={r['foot_dist_min_mm']}mm "
                      f"cyc={r['legs_cycling_after_change_min']}",
                      flush=True)

    def agg(rot_on: bool) -> dict:
        eps = [e for e in episodes if e["rot60"] == rot_on]
        falls = [e for e in eps if e["fell"]]
        near_sec = [e for e in falls
                    if e["fall_dt_sector_change_s"] is not None
                    and e["fall_dt_sector_change_s"] <= NEAR_S]
        return {
            "episodes": len(eps),
            "falls": len(falls),
            "falls_within_2s_of_sector_change": len(near_sec),
            "yaw_sat_frac_mean": round(float(np.mean(
                [e["yaw_sat_frac"] for e in eps])), 4),
            "yaw_margin_min_deg": min(e["yaw_margin_min_deg"]
                                      for e in eps),
            "foot_dist_min_mm": min(e["foot_dist_min_mm"] for e in eps),
        }

    summary = {"rot60_on": agg(True), "rot60_off": agg(False),
               "checkpoint": str(args.checkpoint),
               "seconds": args.seconds, "families": fams,
               "episodes": episodes}
    print("\nSUMMARY rot60 ON :", json.dumps(summary["rot60_on"]))
    print("SUMMARY rot60 off:", json.dumps(summary["rot60_off"]))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(summary, indent=1))
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
