"""Rise+hold specialist -> walk champion HANDOFF composition eval.

THE QUESTION (RL_PLAN queue 2.3 follow-up, the plan's next named
composition milestone): the robot now has an honest learned stand-up +
quiet hold (`ppo_goal_cw_stand_holdbc1_hard1`) and a proven walk
champion (`ppo_goal_cw_walk_longdist_r2`). On the joystick robot the
stand specialist will bring the body up, then the walk champion takes
over. Does the walk champion stumble on the specialist's EXACT settled
pose/state, or is the handoff clean without the scripted blend the old
stance-champion path (play.py key 7) needed?

Three arms, N episodes each per rise start kind (flat/bridge/crouch):

  direct    specialist runs a genuine training-distribution rise episode
            (the env's own goal generator, RSI off) to a settled hold;
            control then switches to the walk champion ON THE
            SPECIALIST'S EXACT PHYSICAL STATE. Episode bookkeeping is
            re-anchored to a clean plant frame (goal refs/height anchor
            = the walk champion's training frame, same trick as play.py
            re_anchor) but qpos/qvel/ctrl and the safety slew state
            carry over unchanged — physically nothing is teleported.
  blend     identical, plus the deployed fallback's scripted 1.5 s
            joint-target blend to the walk plant pose between the two
            policies (the proven key-7 path) — the incumbent to beat.
  plant     control: walk champion from its own clean plant reset, same
            drive schedule — its training distribution; defines the
            noise band for the drive metrics.

Per episode: rise outcome (valid_plant + detail at the handoff tick),
then a scripted drive schedule (1 s settle, 6 s forward, 2 s stop) with
falls / tracking error / distance / stumble-window (first 2 s) max tilt
and height dip.

    python3 -m rl_move.sim.eval_handoff \
        --stand rl_move/sim/policies/ppo_goal_cw_stand_holdbc1_hard1.zip \
        --walk  rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --episodes 4 --out logs/ckpt_eval/handoff.json \
        [--cfg-set bus.servo_params=loaded ...]

Verdict rule of thumb: direct is a PASS if its falls == 0 and its drive
metrics sit inside the plant arm's band; blend-vs-direct tells us
whether the scripted blend is still needed at all.
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

# Specialist rise profile: worst-case pre-ramp hold (rise_hold_s, 3 s)
# + ramp (rise_ramp_s, 6 s) + settle slack. 12.5 s stays inside the
# specialist's 15 s training episodes.
PHASE_A_S = 12.5
BLEND_S = 1.5            # the key-7 scripted blend duration (play.py)
STUMBLE_S = 2.0          # window after handoff scored for the stumble
SCHEDULE = lambda v: [(1.0, 0.0, 0.0), (6.0, v, 0.0), (2.0, 0.0, 0.0)]


def _set_mix(gen, **p) -> None:
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_walk = 0.0
    for k, v in p.items():
        setattr(gen, f"p_{k}", v)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0])
    ap.add_argument("--stand", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_stand_holdbc1_hard1.zip"))
    ap.add_argument("--walk", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_walk_longdist_r2.zip"))
    ap.add_argument("--episodes", type=int, default=4,
                    help="episodes per start kind per arm")
    ap.add_argument("--start-kinds", default="flat,bridge,crouch")
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rise-height-mm", default="108,114",
                    help="specialist's trained plant band")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--strips", type=Path, default=None,
                    help="dir for 1 fps frame-strip PNGs (first episode "
                         "of each arm x start kind)")
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V")
    args = ap.parse_args()

    import mujoco
    from stable_baselines3 import PPO

    from rl_move.config import load_config
    from rl_move.env import build_obs
    from .joint_task import q_rad_to_action
    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    # cfg BEFORE construction (obs width / servo params are baked in).
    cfg = load_config()
    lo, hi = (float(x) for x in args.rise_height_mm.split(","))
    cfg.setdefault("actions", {})["max_height_mm"] = hi + 1.0
    g = cfg.setdefault("goal", {})
    g["rise_height_mm"] = [lo, hi]
    g["rise_ramp_s"] = 6.0
    g["rise_rsi_frac"] = 0.0          # eval = real belly starts, no RSI
    for spec in (args.cfg_set or []):
        key, val = spec.split("=", 1)
        sect, name = key.split(".", 1)
        try:
            parsed: float | str = float(val)
        except ValueError:
            parsed = val.strip()
        cfg.setdefault(sect, {})[name] = parsed

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg), cfg=cfg, randomize=False,
        episode_seconds=20.0, seed=args.seed,
        render_mode="rgb_array" if args.strips else None)
    stand = PPO.load(args.stand, device="cpu")
    walk = PPO.load(args.walk, device="cpu")
    n_stand = int(stand.observation_space.shape[0])
    n_env = int(env.observation_space.shape[0])
    assert walk.observation_space.shape[0] == n_env, (
        f"walk policy obs {walk.observation_space.shape} != env {n_env}")
    assert n_stand < n_env, (
        "stand policy obs must be a prefix of the walk env obs "
        f"(got {n_stand} vs {n_env})")

    gen = env._goal_gen
    dt = env.dt
    qadr = env._qadr

    def chassis_z() -> float:
        return float(env.data.xpos[env._chassis_bid, 2])

    strip_frames: list = []

    def grab(final: bool = False) -> None:
        if args.strips is None:
            return
        # ~1 fps strip; cheap enough to call every tick.
        if final or (grab.n % max(1, int(round(1.0 / dt)))) == 0:
            strip_frames.append(env.render())
        grab.n += 1
    grab.n = 0

    def save_strip(name: str) -> None:
        if args.strips is None or not strip_frames:
            return
        import imageio.v2 as imageio   # same writer the harness uses
        args.strips.mkdir(parents=True, exist_ok=True)
        imageio.imwrite(args.strips / f"{name}.png",
                        np.hstack(strip_frames))
        strip_frames.clear()

    # One clean plant reset up front: the blend arm's target joint pose
    # (play.py captures q_plant the same way).
    _set_mix(gen, walk=1.0)
    env.reset()
    q_plant = env.data.qpos[qadr].copy()

    def reanchor_keep_state():
        """Fresh plant-frame walk episode, specialist's physics kept.

        reset() re-anchors every episode reference (goal height frame,
        pad-z, q_nom, tilt ref, safety nominal) at a clean plant — the
        walk champion's training frame — then we put back the exact
        physical state the specialist left: qpos/qvel/act/ctrl AND the
        safety layer's stateful slew memory (on hardware the slew state
        carries across a policy switch; re-initializing it at the plant
        nominal would yank any joint >1 slew step from nominal).
        """
        d = env.data
        keep_qpos, keep_qvel = d.qpos.copy(), d.qvel.copy()
        keep_ctrl = d.ctrl.copy()
        keep_act = d.act.copy() if d.act.size else None
        keep_safe = env.safety._last_safe.copy()
        _set_mix(gen, walk=1.0)
        env.reset()
        d.qpos[:] = keep_qpos
        d.qvel[:] = keep_qvel
        d.ctrl[:] = keep_ctrl
        if keep_act is not None:
            d.act[:] = keep_act
        env.safety._last_safe = keep_safe
        mujoco.mj_forward(env.model, env.data)
        env._state = env._read_state()
        return env._final_obs(
            build_obs(env.cfg, env._state, env._q_nom,
                      env._prev_action, goal=env._current_goal(),
                      tilt_ref=env._tilt_ref0), reset=True)

    def drive(obs) -> dict:
        """Walk-champion drive schedule on the CURRENT episode."""
        traj = env._goal_traj
        r = {"fall": None, "trk_err": 0.0, "dist_m": 0.0,
             "stumble_max_tilt_deg": 0.0, "stumble_min_height_mm": 1e9}
        n_err, t = 0, 0.0
        p0 = np.array(env.data.qpos[:2], dtype=float)
        for seconds, vx, vy in SCHEDULE(args.speed):
            for _ in range(max(1, int(round(seconds / dt)))):
                if hasattr(traj, "vx"):
                    traj.vx[:] = vx
                    traj.vy[:] = vy
                if getattr(traj, "wz", None) is not None:
                    traj.wz[:] = 0.0
                a, _ = walk.predict(obs, deterministic=True)
                obs, _rw, term, trunc, info = env.step(a)
                grab()
                t += dt
                if t <= STUMBLE_S:
                    tr, tp = env._true_roll_pitch()
                    r["stumble_max_tilt_deg"] = max(
                        r["stumble_max_tilt_deg"],
                        math.degrees(max(abs(tr), abs(tp))))
                    r["stumble_min_height_mm"] = min(
                        r["stumble_min_height_mm"], chassis_z() * 1000.0)
                v = env._body_vel_xy()
                r["trk_err"] += math.hypot(v[0] - vx, v[1] - vy)
                n_err += 1
                if term or trunc:
                    r["fall"] = str(
                        info.get("termination_reason") or "episode_end")
                    break
            if r["fall"]:
                break
        r["trk_err"] = round(r["trk_err"] / max(n_err, 1), 4)
        r["dist_m"] = round(float(np.hypot(
            *(np.array(env.data.qpos[:2], dtype=float) - p0))), 3)
        r["stumble_max_tilt_deg"] = round(r["stumble_max_tilt_deg"], 1)
        r["stumble_min_height_mm"] = round(r["stumble_min_height_mm"], 1)
        return r

    def rise_phase(kind: str) -> tuple[dict, bool]:
        """Specialist rise+settle. Returns (report, alive)."""
        _set_mix(gen, rise=1.0)
        gen.force_rise_start = kind
        obs, _ = env.reset()
        gen.force_rise_start = None
        for _ in range(int(round(PHASE_A_S / dt))):
            a, _ = stand.predict(obs[:n_stand], deterministic=True)
            obs, _rw, term, trunc, info = env.step(a)
            grab()
            if term or trunc:
                return ({"rise_fall": str(
                    info.get("termination_reason") or "end")}, False)
        h_err = chassis_z() - (env._z0 + env._h_target)
        ok, detail = env.plant_report(height_err_m=h_err)
        rep = {"rise_valid_plant": bool(ok),
               "rise_fail": [k for k, v in detail.items()
                             if k.endswith("_ok") and not v],
               "rise_height_err_mm": round(h_err * 1000.0, 1),
               "rise_max_clear_mm": detail.get("max_clear_mm")}
        return rep, True

    def blend_phase() -> bool:
        """Scripted 1.5 s joint blend to the walk plant pose (key-7)."""
        q_from = env.data.qpos[qadr].copy()
        n = int(round(BLEND_S / dt))
        for i in range(n):
            s = (i + 1) / n
            a = q_rad_to_action((1.0 - s) * q_from + s * q_plant)
            _obs, _rw, term, trunc, _info = env.step(a)
            grab()
            if term or trunc:
                return False
        return True

    kinds = [k.strip() for k in args.start_kinds.split(",") if k.strip()]
    results: dict = {"cfg_set": args.cfg_set or [],
                     "stand": str(args.stand), "walk": str(args.walk),
                     "speed": args.speed, "episodes": []}

    for arm in ("direct", "blend", "plant"):
        for kind in (kinds if arm != "plant" else ["plant"]):
            for ep in range(args.episodes):
                rec = {"arm": arm, "start": kind, "ep": ep}
                name = f"{arm}_{kind}_{ep}"
                want_strip = args.strips is not None and ep == 0
                strip_frames.clear()   # drop any prior episode's frames
                if arm == "plant":
                    _set_mix(gen, walk=1.0)
                    obs, _ = env.reset()
                    # walk episodes always start at the plant here
                    # (goal.walk_park_start_frac defaults 0).
                    rec.update(drive(obs))
                else:
                    rep, alive = rise_phase(kind)
                    rec.update(rep)
                    if alive and arm == "blend":
                        alive = blend_phase()
                        if not alive:
                            rec["rise_fall"] = "fell_during_blend"
                    if alive:
                        obs = reanchor_keep_state()
                        rec.update(drive(obs))
                    else:
                        rec["fall"] = "before_handoff"
                if want_strip:
                    grab(final=True)
                    save_strip(name)
                results["episodes"].append(rec)
                print(f"[{arm:6s}] {kind:7s} ep{ep} "
                      f"rise_ok={rec.get('rise_valid_plant', '-')} "
                      f"fall={rec.get('fall')} "
                      f"trk={rec.get('trk_err', '-')} "
                      f"dist={rec.get('dist_m', '-')} "
                      f"tilt2s={rec.get('stumble_max_tilt_deg', '-')} "
                      f"hmin2s={rec.get('stumble_min_height_mm', '-')}")

    # ---- summary ------------------------------------------------------
    def band(arm: str, key: str) -> list:
        vals = [e[key] for e in results["episodes"]
                if e["arm"] == arm and key in e and e.get("fall") is None]
        return ([round(float(min(vals)), 3), round(float(max(vals)), 3)]
                if vals else [])

    summary = {}
    for arm in ("direct", "blend", "plant"):
        eps = [e for e in results["episodes"] if e["arm"] == arm]
        handed = [e for e in eps if e.get("fall") != "before_handoff"]
        summary[arm] = {
            "episodes": len(eps),
            "rise_valid_plant": sum(
                1 for e in eps if e.get("rise_valid_plant")),
            "rise_failed_pre_handoff": sum(
                1 for e in eps if e.get("fall") == "before_handoff"),
            "handoff_falls": sum(
                1 for e in handed if e.get("fall")),
            "trk_err_band": band(arm, "trk_err"),
            "dist_band": band(arm, "dist_m"),
            "stumble_tilt_band": band(arm, "stumble_max_tilt_deg"),
        }
    results["summary"] = summary
    print(json.dumps(summary, indent=1))
    direct_ok = (summary["direct"]["handoff_falls"] == 0
                 and summary["direct"]["rise_failed_pre_handoff"]
                 < summary["direct"]["episodes"])
    print("HANDOFF (direct, no blend):",
          "CLEAN — no falls after switching on the specialist's pose"
          if direct_ok else "NOT CLEAN — see per-episode records")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(results, indent=1))
        print(f"wrote {args.out}")
    return 0 if direct_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
