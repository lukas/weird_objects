"""probe_mirror_turn.py — zero-training reflection probe for turning.

WHY (RL_PLAN queue 0.2 step 2, 08-11 — the rot60 lesson applied to the
left-right mirror): every walk-lineage policy carries a
command-invariant ~+0.09 rad/s LEFT yaw drift; three reward-mechanism
families failed to move it because it is baked into the learned gait's
chirality, not into any price. The hexagon also has a (near-exact,
pinwheel-approximate) sagittal mirror symmetry, and mirror.py already
encodes the full obs/action relabeling. Reflecting the policy
(mirror.MirrorPolicy) should produce a RIGHT-drifter with identical
gait competence — so selecting naked-vs-mirrored by desired turn sign
is commanded ARC turning, and alternating on a heading error cancels
the drift for straight driving, all with ZERO training.

WHAT IT MEASURES (per checkpoint, deterministic, N seeds):

  naked        forward walk, the policy as deployed: yaw drift rate
               (rad/s), heading at end, travel along command. This IS
               "steer in the drift direction" (left today).
  mirror       the reflected policy on the same command: the mirror
               hypothesis predicts wz ~ -naked with matched travel.
               This IS "steer the other way" (right today) — together
               the two rows are commanded arc turning by selection.
  heading-hold alternate naked/mirrored on the accumulated heading
               (bang-bang, hysteresis): final |heading| and travel vs
               naked's runaway drift — the "drive straight" row.

PASS reading (pre-registered): mirror-fwd wz within 50% of -1x
naked-fwd wz with travel >= 0.7x naked; heading-hold final |heading|
< 0.5x naked's. If mirror-fwd travel collapses or wz keeps the naked
sign, the pinwheel asymmetry dominates and reflection is refuted as a
turning mechanism (then mirror-symmetry TRAINING, queue 0.2 step 3,
is the remaining lever).

    uv run python -m rl_move.sim.probe_mirror_turn \
        rl_move/sim/policies/ppo_goal_cw_dep_vref1_r1.zip \
        --seeds 0,1,2 --seconds 12 --out logs/mirror_turn/vref1r1.json
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

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

CMD_V = 0.05          # champion band forward command
HYST_RAD = math.radians(4.0)   # heading-hold switch hysteresis


def make_env(seed: int, episode_seconds: float, dr_scale: float = 0.0):
    from rl_move.config import load_config
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    # Deployment contract of the walk deliverable (cw-dep-vref1 lineage):
    # meas := ref velocity obs, 25 deg tilt envelope. Reward keys are
    # irrelevant here — this probe only reads behavior.
    cfg = load_config()
    cfg.setdefault("goal", {})["walk_obs_body_vel"] = 2.0
    cfg.setdefault("safety", {})["max_roll_deg"] = 25.0
    cfg["safety"]["max_pitch_deg"] = 25.0
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=dr_scale > 0.0,
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def pin_forward(env) -> None:
    traj = env._goal_traj
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = CMD_V
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = CMD_V * ramp
    traj.vy[:] = 0.0
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0


def body_heading(env) -> float:
    R = env.data.xmat[env._chassis_bid].reshape(3, 3)
    return math.atan2(R[1, 0], R[0, 0])


def rollout(policies: dict, mode: str, seed: int,
            episode_seconds: float, drift_sign: int = +1,
            dr_scale: float = 0.0) -> dict:
    """policies: {'naked': model, 'mirror': MirrorPolicy}. mode selects
    which/when; drift_sign = sign of the NAKED policy's measured drift
    (steers the heading-hold selector). Returns drift + travel
    fingerprints."""
    env = make_env(seed, episode_seconds, dr_scale)
    obs, _ = env.reset()
    pin_forward(env)
    h0 = body_heading(env)
    p0 = env.data.xpos[env._chassis_bid][:2].copy()

    settle_n = int(round(2.0 / env.dt))
    wzs: list[float] = []
    active = "naked"
    switches = 0
    step, term_reason = 0, None
    heading_unwrapped, h_prev = 0.0, h0
    while True:
        if mode == "heading-hold":
            # Bang-bang on the accumulated heading, hysteresis so the
            # selector cannot chatter every tick: veered too far in the
            # naked drift's direction -> run the OTHER chirality.
            want = active
            if heading_unwrapped > HYST_RAD:
                want = "mirror" if drift_sign > 0 else "naked"
            elif heading_unwrapped < -HYST_RAD:
                want = "naked" if drift_sign > 0 else "mirror"
            if want != active:
                active, switches = want, switches + 1
        elif mode in policies:
            active = mode
        a, _ = policies[active].predict(obs, deterministic=True)
        obs, _r, term, trunc, info = env.step(a)
        h = body_heading(env)
        d = (h - h_prev + math.pi) % (2 * math.pi) - math.pi
        heading_unwrapped += d
        h_prev = h
        if step >= settle_n:
            wzs.append(env._body_wz())
        step += 1
        if term or trunc:
            term_reason = info.get("termination_reason")
            break
    p1 = env.data.xpos[env._chassis_bid][:2]
    travel = float(p1[0] - p0[0])
    travel_abs = float(np.hypot(*(p1 - p0)))
    env.close()
    return {
        "mode": mode, "seed": seed,
        "wz_mean": float(np.mean(wzs)) if wzs else 0.0,
        "wz_med": float(np.median(wzs)) if wzs else 0.0,
        "heading_end_deg": math.degrees(heading_unwrapped),
        "travel_x_m": round(travel, 4),
        "travel_abs_m": round(travel_abs, 4),
        "switches": switches,
        "terminated": term_reason if term_reason not in (None, "time")
        else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    from stable_baselines3 import PPO

    from rl_move.sim.mirror import MirrorPolicy

    model = PPO.load(args.checkpoint, device="cpu")
    policies = {"naked": model, "mirror": MirrorPolicy(model)}
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]

    records: list[dict] = []
    drift_sign = +1
    # naked first: its measured drift sign steers the selector modes
    for mode in ("naked", "mirror", "heading-hold"):
        for s in seeds:
            r = rollout(policies, mode, s, args.seconds, drift_sign,
                        args.dr_scale)
            records.append(r)
            print(f"{mode:13s} seed {s}: wz_mean {r['wz_mean']:+.4f} "
                  f"rad/s  heading_end {r['heading_end_deg']:+7.1f} deg  "
                  f"travel_x {r['travel_x_m']:+.3f} m  "
                  f"switches {r['switches']}"
                  + (f"  TERMINATED({r['terminated']})"
                     if r["terminated"] else ""))
        if mode == "naked":
            naked_wz = float(np.mean(
                [r["wz_mean"] for r in records if r["mode"] == "naked"]))
            drift_sign = +1 if naked_wz >= 0 else -1

    def agg(mode, key):
        return float(np.mean([r[key] for r in records
                              if r["mode"] == mode]))

    naked_wz = agg("naked", "wz_mean")
    mirror_wz = agg("mirror", "wz_mean")
    naked_tv = agg("naked", "travel_x_m")
    mirror_tv = agg("mirror", "travel_x_m")
    hh_head = float(np.mean([abs(r["heading_end_deg"]) for r in records
                             if r["mode"] == "heading-hold"]))
    nk_head = float(np.mean([abs(r["heading_end_deg"]) for r in records
                             if r["mode"] == "naked"]))
    falls = sum(1 for r in records if r["terminated"])

    flipped = (naked_wz * mirror_wz < 0
               and 0.5 <= abs(mirror_wz / naked_wz) <= 2.0
               ) if abs(naked_wz) > 1e-4 else False
    travel_ok = mirror_tv >= 0.7 * naked_tv
    hold_ok = hh_head < 0.5 * nk_head if nk_head > 2.0 else True
    verdict = ("PASS" if flipped and travel_ok and hold_ok and falls == 0
               else "FAIL")

    print(f"\nnaked   wz {naked_wz:+.4f} rad/s, travel {naked_tv:+.3f} m")
    print(f"mirror  wz {mirror_wz:+.4f} rad/s, travel {mirror_tv:+.3f} m")
    print(f"drift flipped: {flipped}  travel_ok: {travel_ok}  "
          f"heading-hold |end| {hh_head:.1f} deg vs naked {nk_head:.1f} "
          f"deg (hold_ok: {hold_ok})  falls: {falls}")
    print(f"REFLECTION PROBE: {verdict}")

    if args.out:
        out = ROOT / args.out if not args.out.is_absolute() else args.out
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(
            {"checkpoint": str(args.checkpoint), "verdict": verdict,
             "naked_wz": naked_wz, "mirror_wz": mirror_wz,
             "naked_travel": naked_tv, "mirror_travel": mirror_tv,
             "heading_hold_end_deg": hh_head,
             "naked_end_deg": nk_head, "falls": falls,
             "records": records}, indent=1))
        print(f"wrote {out}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
