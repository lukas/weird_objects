"""Walk champion -> stop -> specialist LOWER (sit) REVERSE handoff eval.

THE QUESTION (RL_PLAN queue 2.3, the composition milestone after the
forward handoff PASSED 08-11): on the joystick robot the walk champion
(`ppo_goal_cw_walk_longdist_r2`) drives, the operator lets go of the
stick, the robot stops — then the rise/hold specialist
(`ppo_goal_cw_stand_holdbc1_hard1`) takes over to lower the body to
belly rest (sit). Two sub-questions, answered by three arms:

  (1) Is the specialist's LOWER skill even intact post-holdbc1? Its
      lineage trained lower at 45% goal-mix but the quality was never
      harness-verified after the hold-BC change (bc1@2M showed a 166mm
      flag-leg lower; holdbc1's gates only covered hold+rise).
  (2) Does the specialist lower cleanly from the WALKER'S exact stopped
      pose/state (gait residue in qpos/qvel + slew memory), not just
      from its own clean plant reset?

Arms, N episodes each:

  spec      control: specialist runs its own training-distribution
            LOWER episode from a clean plant reset. Defines the
            specialist's intrinsic lower quality + the noise band.
  direct    walk champion drives a scripted schedule (1 s settle, 4 s
            forward, 2 s stop) from a clean plant; control switches to
            the specialist ON THE WALKER'S EXACT PHYSICAL STATE with
            episode bookkeeping re-anchored to a fresh LOWER episode
            (same reanchor trick as eval_handoff.py: goal refs at the
            clean-plant frame; qpos/qvel/ctrl + safety slew carried).
  scripted  incumbent: same walk+stop, then the hardware go_zero("sit")
            analog — a scripted 6 s linear joint glide to the zero
            (belly) pose + 2 s quiet hold. No specialist. The deployable
            fallback if the learned lower is broken.

Per episode: walk-phase fall, lower-phase termination, final height err
vs the lower ref (spec/direct), tail-0.5s per-pad clearance vs the
harness's 60 mm belly allowance (the posture-strict lower rule),
max tilt + peak descent speed (bang proxy) during the lower.

    python3 -m rl_move.sim.eval_handoff_reverse \
        --stand rl_move/sim/policies/ppo_goal_cw_stand_holdbc1_hard1.zip \
        --walk  rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --episodes 6 --out logs/ckpt_eval/handoff_rev.json \
        [--cfg-set bus.servo_params=loaded ...]

Verdict rule of thumb: `spec` tells us whether the learned lower exists
at all (posture-strict: no term, |h_err|<=15mm, all pads <=60mm);
`direct` is a PASS if it matches the spec band with zero falls; the
`scripted` arm is the bar a broken learned lower must be replaced by.
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

WALK_SCHEDULE = lambda v: [(1.0, 0.0, 0.0), (4.0, v, 0.0), (2.0, 0.0, 0.0)]
# Lower profile: lower_hold_s (1 s) + lower_ramp_s (5 s) + quiet settle.
LOWER_PHASE_S = 10.0
GLIDE_S = 6.0            # hardware go_zero("sit") glide analog
GLIDE_HOLD_S = 2.0       # quiet hold at the zero pose before scoring
TAIL_S = 0.5             # harness tail-mean window for end posture
END_CLEAR_BELLY_MM = 60.0   # eval_checkpoint.py posture-strict lower
HEIGHT_ERR_OK_MM = 15.0     # eval_checkpoint.py lower success rule


def _set_mix(gen, **p) -> None:
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_walk = 0.0
    for k, v in p.items():
        setattr(gen, f"p_{k}", v)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stand", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_stand_holdbc1_hard1.zip"))
    ap.add_argument("--walk", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_walk_longdist_r2.zip"))
    ap.add_argument("--episodes", type=int, default=6,
                    help="episodes per arm")
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--strips", type=Path, default=None,
                    help="dir for 1 fps frame-strip PNGs (first episode "
                         "of each arm)")
    ap.add_argument("--strip-all", action="store_true",
                    help="strip EVERY episode, not just ep0 (failure "
                         "review)")
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
    # Same frame as eval_handoff.py; lower_* stays at the specialist's
    # trained defaults (lower_height_mm [25,55], 1 s hold + 5 s ramp).
    cfg = load_config()
    cfg.setdefault("actions", {})["max_height_mm"] = 115.0
    g = cfg.setdefault("goal", {})
    g["rise_rsi_frac"] = 0.0
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
    tail_n = max(1, int(round(TAIL_S / dt)))

    def chassis_z() -> float:
        return float(env.data.xpos[env._chassis_bid, 2])

    def pad_z() -> np.ndarray:
        return np.array([float(env.data.xpos[b, 2])
                         for b in env._pad_bids])

    strip_frames: list = []

    def grab(final: bool = False) -> None:
        if args.strips is None:
            return
        if final or (grab.n % max(1, int(round(1.0 / dt)))) == 0:
            strip_frames.append(env.render())
        grab.n += 1
    grab.n = 0

    def save_strip(name: str) -> None:
        if args.strips is None or not strip_frames:
            return
        import imageio.v2 as imageio
        args.strips.mkdir(parents=True, exist_ok=True)
        imageio.imwrite(args.strips / f"{name}.png",
                        np.hstack(strip_frames))
        strip_frames.clear()

    def reanchor_lower_keep_state():
        """Fresh clean-plant LOWER episode, walker's physics kept.

        Same trick as eval_handoff.reanchor_keep_state: reset()
        re-anchors every episode reference (height frame _z0, pad-z ref,
        q_nom, tilt ref, safety nominal) at a clean plant, then we put
        back the exact physical state the walker stopped in — qpos/qvel/
        act/ctrl AND the safety layer's stateful slew memory.
        """
        d = env.data
        keep_qpos, keep_qvel = d.qpos.copy(), d.qvel.copy()
        keep_ctrl = d.ctrl.copy()
        keep_act = d.act.copy() if d.act.size else None
        keep_safe = env.safety._last_safe.copy()
        _set_mix(gen, lower=1.0)
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

    def walk_phase() -> tuple:
        """Clean-plant walk episode + drive schedule. (obs, alive)."""
        _set_mix(gen, walk=1.0)
        obs, _ = env.reset()
        traj = env._goal_traj
        for seconds, vx, vy in WALK_SCHEDULE(args.speed):
            for _ in range(max(1, int(round(seconds / dt)))):
                if hasattr(traj, "vx"):
                    traj.vx[:] = vx
                    traj.vy[:] = vy
                if getattr(traj, "wz", None) is not None:
                    traj.wz[:] = 0.0
                a, _ = walk.predict(obs, deterministic=True)
                obs, _rw, term, trunc, info = env.step(a)
                grab()
                if term or trunc:
                    return (str(info.get("termination_reason")
                                or "episode_end"), False)
        return obs, True

    def score_tail(rec: dict, pad_hist: list, h_err_mm) -> None:
        clear_mm = (np.asarray(pad_hist[-tail_n:]).mean(axis=0)
                    - env._pad_z_ref) * 1000.0
        rec["end_clear_mm"] = [round(float(c), 1) for c in clear_mm]
        rec["end_posture_ok"] = bool(
            (clear_mm <= END_CLEAR_BELLY_MM).all())
        rec["height_err_end_mm"] = (None if h_err_mm is None
                                    else round(abs(h_err_mm), 1))
        rec["success"] = bool(
            rec.get("fall") is None and rec["end_posture_ok"]
            and (h_err_mm is None
                 or abs(h_err_mm) <= HEIGHT_ERR_OK_MM))

    def lower_phase(obs, rec: dict) -> None:
        """Specialist runs the current LOWER episode to belly rest."""
        pad_hist, h_err_mm = [], None
        z_prev = chassis_z()
        for _ in range(int(round(LOWER_PHASE_S / dt))):
            a, _ = stand.predict(obs[:n_stand], deterministic=True)
            obs, _rw, term, trunc, info = env.step(a)
            grab()
            pad_hist.append(pad_z())
            if "height_err_mm" in info:
                h_err_mm = float(info["height_err_mm"])
            tr, tp = env._true_roll_pitch()
            rec["max_tilt_deg"] = round(max(
                rec.get("max_tilt_deg", 0.0),
                math.degrees(max(abs(tr), abs(tp)))), 1)
            z = chassis_z()
            rec["peak_down_mps"] = round(max(
                rec.get("peak_down_mps", 0.0), (z_prev - z) / dt), 3)
            z_prev = z
            if term or trunc:
                rec["fall"] = str(info.get("termination_reason")
                                  or "episode_end")
                break
        if pad_hist:
            score_tail(rec, pad_hist, h_err_mm)
        else:
            rec["success"] = False

    def glide_phase(rec: dict) -> None:
        """Scripted go_zero('sit') analog inside the walk episode."""
        q_from = env.data.qpos[qadr].copy()
        q_zero = np.zeros_like(q_from)
        n = int(round(GLIDE_S / dt))
        pad_hist = []
        z_prev = chassis_z()
        for i in range(n + int(round(GLIDE_HOLD_S / dt))):
            s = min(1.0, (i + 1) / n)
            a = q_rad_to_action((1.0 - s) * q_from + s * q_zero)
            _obs, _rw, term, trunc, info = env.step(a)
            grab()
            pad_hist.append(pad_z())
            tr, tp = env._true_roll_pitch()
            rec["max_tilt_deg"] = round(max(
                rec.get("max_tilt_deg", 0.0),
                math.degrees(max(abs(tr), abs(tp)))), 1)
            z = chassis_z()
            rec["peak_down_mps"] = round(max(
                rec.get("peak_down_mps", 0.0), (z_prev - z) / dt), 3)
            z_prev = z
            if term or trunc:
                rec["fall"] = str(info.get("termination_reason")
                                  or "episode_end")
                break
        if pad_hist:
            # No height goal on the scripted path: posture-only rule.
            score_tail(rec, pad_hist, None)
        else:
            rec["success"] = False

    results: dict = {"cfg_set": args.cfg_set or [],
                     "stand": str(args.stand), "walk": str(args.walk),
                     "speed": args.speed, "episodes": []}

    for arm in ("spec", "direct", "scripted"):
        for ep in range(args.episodes):
            rec = {"arm": arm, "ep": ep}
            name = f"rev_{arm}_{ep}"
            want_strip = args.strips is not None and (
                args.strip_all or ep == 0)
            strip_frames.clear()
            if arm == "spec":
                _set_mix(gen, lower=1.0)
                obs, _ = env.reset()
                lower_phase(obs, rec)
            else:
                out, alive = walk_phase()
                if not alive:
                    rec["fall"] = "during_walk:" + out
                    rec["success"] = False
                elif arm == "direct":
                    obs = reanchor_lower_keep_state()
                    lower_phase(obs, rec)
                else:
                    glide_phase(rec)
            if want_strip:
                grab(final=True)
                save_strip(name)
            results["episodes"].append(rec)
            print(f"[{arm:8s}] ep{ep} success={rec.get('success')} "
                  f"fall={rec.get('fall')} "
                  f"h_err={rec.get('height_err_end_mm', '-')} "
                  f"max_clear={max(rec['end_clear_mm']) if rec.get('end_clear_mm') else '-'} "
                  f"tilt={rec.get('max_tilt_deg', '-')} "
                  f"vdown={rec.get('peak_down_mps', '-')}")

    # ---- summary ------------------------------------------------------
    summary = {}
    for arm in ("spec", "direct", "scripted"):
        eps = [e for e in results["episodes"] if e["arm"] == arm]
        handed = [e for e in eps
                  if not str(e.get("fall", "")).startswith("during_walk")]
        clears = [max(e["end_clear_mm"]) for e in eps
                  if e.get("end_clear_mm")]
        summary[arm] = {
            "episodes": len(eps),
            "walk_falls_pre_handoff": len(eps) - len(handed),
            "lower_falls": sum(1 for e in handed if e.get("fall")),
            "success": sum(1 for e in eps if e.get("success")),
            "end_posture_ok": sum(1 for e in eps
                                  if e.get("end_posture_ok")),
            "worst_clear_band_mm": ([round(min(clears), 1),
                                     round(max(clears), 1)]
                                    if clears else []),
            "max_tilt_band_deg": ([round(min(e["max_tilt_deg"]
                                          for e in handed), 1),
                                   round(max(e["max_tilt_deg"]
                                             for e in handed), 1)]
                                  if handed else []),
        }
    results["summary"] = summary
    print(json.dumps(summary, indent=1))
    spec_ok = summary["spec"]["success"] == summary["spec"]["episodes"]
    direct_ok = (summary["direct"]["lower_falls"] == 0
                 and summary["direct"]["success"]
                 >= max(1, summary["direct"]["episodes"] - 1))
    print("SPEC lower (training distribution):",
          "INTACT" if spec_ok else "NOT posture-strict — see episodes")
    print("REVERSE HANDOFF (direct):",
          "CLEAN" if direct_ok else "NOT CLEAN — see per-episode records")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(results, indent=1))
        print(f"wrote {args.out}")
    return 0 if (spec_ok and direct_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
