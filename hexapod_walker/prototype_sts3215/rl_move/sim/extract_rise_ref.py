"""Extract a rise reference trajectory from a champion checkpoint.

Stage-II scaffold for the unified-policy rise (operator 08-10; see
RL_PLAN.md "rise" section). The stand-up literature (HumanUP, HoST,
RSS 2025) never learns a deployable get-up from a height reward in one
shot: a discovery stage finds the motion, then the deployable policy
TRACKS it. Our discovery stage is already done — the stance champion
(ppo_goal_cw_stance_dr10) performs a learned belly-rise. This script
rolls it out deterministically on the nominal sim (DR0), verifies the
episode actually ends standing ON ITS FEET (posture-strict: every pad
within --end-clear-mm of its grounded z), and saves the joint
trajectory as the tracking reference consumed by
``reward.k_rise_ref_track`` / ``reward.rise_ref_path`` in sim_env.py.

The npz stores ``q_rad`` (T,18), ``dt`` and ``ramp_i0`` (first tick the
height ref leaves zero) — training episodes time-align to the ramp
start, so jittered hold windows and crouch/bridge starts all join the
same reference mid-flight.

Usage (plain python, laptop or pod; C env, no GPU needed):

    uv run python -m rl_move.sim.extract_rise_ref \
        rl_move/sim/policies/ppo_goal_cw_stance_dr10.zip \
        --out rl_move/sim/refs/rise_ref_stance_dr10.npz

Tries seeds until one episode passes the posture + height check
(the stance champion's posture-strict rise is ~5/12, so expect a few
rejects); refuses to write a reference that does not stand.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rl_move.robot_state import DEG2RAD  # noqa: E402

from .servo_model import SimServoParams  # noqa: E402
from .train_ppo_sim import ENV_CLASSES  # noqa: E402


def _blend_pose_ik(q_from_rad: np.ndarray, q_plant_rad: np.ndarray,
                   s: float) -> np.ndarray:
    """Foot-anchored (Cartesian) blend between two model-relative poses.

    08-22 tibia-150 finding: a raw joint-ANGLE lerp between the
    champion's crouch-stand and the env plant pose falls on every seed
    at the corrected (longer) tibia, even though both endpoints are
    fine standing poses individually -- the linearly-interpolated
    INTERMEDIATE joint angles do not correspond to a linearly moving
    foot, so the mid-blend pose can transiently over-reach or dig a
    foot into the ground. Fix: per leg, interpolate the FOOT position
    in the leg's sagittal plane (reach, height) -- a straight-line
    foot path is an ordinary, physically valid motion -- and IK back
    to joint angles at every blend step, using the same
    absolute-tibia FK/IK (``tripod_gait.foot_rz_from_hip_knee`` /
    ``_leg_ik``) the hardware gait stack already trusts at this
    geometry. Coxa (yaw) keeps the plain lerp: it only rotates the
    leg's reach direction and was never implicated.
    """
    import tripod_gait as _tg
    q_from = np.asarray(q_from_rad, dtype=float).reshape(18)
    q_plant = np.asarray(q_plant_rad, dtype=float).reshape(18)
    q_out = (1.0 - s) * q_from + s * q_plant
    for leg in range(6):
        hip_j, knee_j = 3 * leg + 1, 3 * leg + 2
        hip_from_deg = math.degrees(q_from[hip_j])
        hip_plant_deg = math.degrees(q_plant[hip_j])
        # model-relative knee -> absolute-tibia knee (knee_abs = hip + knee_rel).
        knee_abs_from_deg = math.degrees(q_from[knee_j] + q_from[hip_j])
        knee_abs_plant_deg = math.degrees(q_plant[knee_j] + q_plant[hip_j])
        r_from, z_from = _tg.foot_rz_from_hip_knee(hip_from_deg,
                                                     knee_abs_from_deg)
        r_plant, z_plant = _tg.foot_rz_from_hip_knee(hip_plant_deg,
                                                       knee_abs_plant_deg)
        r_s = (1.0 - s) * r_from + s * r_plant
        z_s = (1.0 - s) * z_from + s * z_plant
        ik = _tg._leg_ik((r_s, 0.0, z_s))
        if ik is None:
            continue  # keep the joint-space lerp fallback for this leg/tick
        hip_rad, knee_abs_rad = ik
        q_out[hip_j] = hip_rad
        q_out[knee_j] = knee_abs_rad - hip_rad  # abs -> model-relative
    return q_out


def _validate_open_loop_robustness(qs: np.ndarray, ramp_i0: int, env_cls,
                                   seeds, margin_deg: float) -> tuple[bool, str]:
    """Replay the candidate reference open-loop into FRESH resets at
    OTHER seeds than the one it was extracted from, mirroring exactly
    how the consumers (test_task_semantics._rise_rollout, and any
    reward.k_rise_ref_track training rollout) align it: hold
    ``q_rad[0]`` until the new env's own ramp-start tick
    (``env._rise_ramp_i0``), then play ``q_rad[ramp_i0:]`` verbatim.

    08-22 finding: a candidate that stands cleanly when replayed into
    the EXACT env instance it was recorded from (same seed, same
    starting attitude) can still be one open-loop absolute-joint
    trajectory blindly applied over a DIFFERENT starting attitude —
    with no closed-loop correction, small attitude differences at the
    start compound and can tip a marginal candidate over the hard
    safety cutoff mid-blend. A reference only safe for its own
    extraction seed is not safe to ship (PPO/the bank replay it
    against arbitrary resets) -- this is the check that catches that
    before it ships, not after a bank regression.
    """
    from .joint_task import q_rad_to_action
    total = len(qs)
    dt_guess = 0.04
    for seed in seeds:
        env = env_cls(params=SimServoParams.from_cfg(None),
                      randomize=False, dr_scale=0.0,
                      episode_seconds=total * dt_guess + 2.0, seed=seed)
        gen = env._goal_gen
        for m in ("hold", "lean", "track", "unload", "raise", "rise",
                  "lower", "quad"):
            if hasattr(gen, f"p_{m}"):
                setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
        gen.force_rise_start = "flat"
        obs, _ = env.reset()
        peak = 0.0
        term = trunc = False
        step = 0
        while True:
            j = ramp_i0 + (step - env._rise_ramp_i0)
            act = q_rad_to_action(qs[min(max(j, 0), total - 1)])
            obs, _r, term, trunc, info = env.step(act)
            peak = max(peak, abs(info.get("pitch_deg", 0.0)),
                       abs(info.get("roll_deg", 0.0)))
            step += 1
            if term or trunc or step >= total + 40:
                break
        env.close()
        if term or peak > margin_deg:
            return False, (f"validate seed {seed}: term={term} "
                           f"peak_tilt={peak:.1f}deg (margin {margin_deg})")
    return True, "ok (%d validation seeds)" % len(seeds)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--task", choices=sorted(ENV_CLASSES),
                    default="joint_goal")
    ap.add_argument("--start", choices=["flat", "bridge", "crouch"],
                    default="flat",
                    help="rise start kind to pin (flat = operator "
                         "placement, the full belly-to-stand motion)")
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--seeds", type=int, nargs=2, default=[0, 24],
                    metavar=("LO", "HI"), help="seed range to try")
    ap.add_argument("--height-tol-mm", type=float, default=12.0,
                    help="max |height err| at episode end")
    ap.add_argument("--end-clear-mm", type=float, default=20.0,
                    help="max pad clearance over grounded z at end")
    ap.add_argument("--blend-to-plant", action="store_true",
                    help="append a scripted joint blend from the rise's "
                         "end pose to the env plant pose plus a short "
                         "hold, so the reference ends in the WALKABLE "
                         "plant stance instead of the champion's "
                         "crouch-stand (operator 08-10: 'we couldn't "
                         "walk from that'). The blend is the same "
                         "recipe play.py's 7-key uses, validated to "
                         "hand off into the walk champion.")
    ap.add_argument("--blend-mode", choices=["ik", "lerp"], default="ik",
                    help="ik (default, 08-22 fix) = foot-anchored "
                         "Cartesian blend per leg via FK/IK; lerp = the "
                         "original raw joint-angle interpolation, kept "
                         "only for regression comparison (falls on "
                         "every seed at tibia-150).")
    ap.add_argument("--blend-s", type=float, default=1.5)
    ap.add_argument("--plant-hold-s", type=float, default=1.0)
    ap.add_argument("--validate-seeds", type=str, default="500,501,502",
                    help="comma-separated seeds (disjoint from --seeds "
                         "by convention) to open-loop replay the "
                         "candidate reference into fresh resets before "
                         "accepting it -- catches an extraction-seed-"
                         "only-safe trajectory (08-22 finding). Empty "
                         "string disables (not recommended).")
    ap.add_argument("--validate-margin-deg", type=float, default=8.0,
                    help="max peak |pitch|/|roll| deg tolerated during "
                         "validation replay (below the ~10deg hard "
                         "safety cutoff, so a real training rollout at "
                         "yet another seed still has headroom).")
    ap.add_argument("--out", type=Path,
                    default=_RL / "sim" / "refs" / "rise_ref.npz")
    args = ap.parse_args()

    from stable_baselines3 import PPO

    env_cls = ENV_CLASSES[args.task]
    model = PPO.load(args.checkpoint, device="cpu")

    if args.blend_to_plant:
        from .joint_task import q_rad_to_action

    for seed in range(args.seeds[0], args.seeds[1] + 1):
        # With the blend appended the episode must outlast rise + blend
        # + hold; without it, keep the legacy exact budget.
        extra_s = ((args.blend_s + args.plant_hold_s + 1.0)
                   if args.blend_to_plant else 0.0)
        env = env_cls(params=SimServoParams.from_cfg(None),
                      randomize=False, dr_scale=0.0,
                      episode_seconds=args.episode_seconds + extra_s,
                      seed=seed)
        gen = env._goal_gen
        for m in ("hold", "lean", "track", "unload", "raise", "rise",
                  "lower", "quad"):
            if hasattr(gen, f"p_{m}"):
                setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
        gen.force_rise_start = args.start

        obs, _ = env.reset()
        chassis_bid = env.model.body("chassis").id
        z_start = float(env.data.xpos[chassis_bid, 2])
        qs = [env.data.qpos[env._qadr].copy()]
        hs = [0.0]   # chassis height above start, per tick (RSI schedules)
        term = False
        info: dict = {}
        n_rise = int(round(args.episode_seconds / env.dt))
        for _ in range(n_rise):
            act, _ = model.predict(obs, deterministic=True)
            obs, _r, term, trunc, info = env.step(act)
            qs.append(env.data.qpos[env._qadr].copy())
            hs.append(float(env.data.xpos[chassis_bid, 2]) - z_start)
            if term or trunc:
                break

        def clear_mm_now() -> float:
            return max(
                (float(env.data.xpos[b, 2]) - z) * 1000.0
                for b, z in zip(env._pad_bids, env._pad_z_ref) if b >= 0)

        h = np.asarray(env._goal_traj.height)
        target_mm = env._h_target * 1000.0
        h_end_mm = info.get("height_mm", float("nan"))
        err_mm = abs(h_end_mm - target_mm)
        clear_mm = clear_mm_now()
        ok = (not term and err_mm <= args.height_tol_mm
              and clear_mm <= args.end_clear_mm)
        print(f"[seed {seed}] target {target_mm:+.0f}mm "
              f"end {h_end_mm:+.0f}mm (err {err_mm:.0f}) "
              f"worst_clear {clear_mm:.0f}mm term={term} "
              f"-> {'ACCEPT' if ok else 'reject'}")
        if not ok:
            continue

        if args.blend_to_plant:
            q_plant = env._plant_deg * DEG2RAD
            q_from = env.data.qpos[env._qadr].copy()
            n_blend = max(1, int(round(args.blend_s / env.dt)))
            n_hold = int(round(args.plant_hold_s / env.dt))
            fell = False
            for k in range(n_blend + n_hold):
                s = min((k + 1) / n_blend, 1.0)
                if args.blend_mode == "ik":
                    q_blend = _blend_pose_ik(q_from, q_plant, s)
                else:
                    q_blend = (1.0 - s) * q_from + s * q_plant
                act = q_rad_to_action(q_blend)
                obs, _r, term, trunc, info = env.step(act)
                qs.append(env.data.qpos[env._qadr].copy())
                hs.append(float(env.data.xpos[chassis_bid, 2]) - z_start)
                if term or trunc:
                    fell = True
                    break
            clear_mm = clear_mm_now()
            h_rel_mm = (float(env.data.xpos[chassis_bid, 2])
                        - z_start) * 1000.0
            q_err_deg = float(np.sqrt(np.mean(
                (env.data.qpos[env._qadr] - q_plant) ** 2))) / DEG2RAD
            ok = (not fell and clear_mm <= args.end_clear_mm
                  and q_err_deg <= 10.0)
            print(f"[seed {seed}] blend: rel height {h_rel_mm:+.0f}mm "
                  f"worst_clear {clear_mm:.0f}mm q_rms_err "
                  f"{q_err_deg:.1f}deg fell={fell} "
                  f"-> {'ACCEPT' if ok else 'reject'}")
            if not ok:
                continue

        nz = np.nonzero(np.abs(h) > 1e-12)[0]
        ramp_i0 = int(nz[0]) if len(nz) else 0

        if args.blend_to_plant and args.validate_seeds.strip():
            vseeds = [int(v) for v in args.validate_seeds.split(",") if v.strip()]
            v_ok, v_detail = _validate_open_loop_robustness(
                np.asarray(qs), ramp_i0, env_cls, vseeds,
                args.validate_margin_deg)
            print(f"[seed {seed}] robustness validation: "
                  f"{'PASS' if v_ok else 'FAIL'} ({v_detail})")
            if not v_ok:
                continue

        h_rel_end_m = (float(env.data.xpos[chassis_bid, 2]) - z_start)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        np.savez(args.out, q_rad=np.asarray(qs), dt=env.dt,
                 ramp_i0=ramp_i0, h_rel_end_m=h_rel_end_m,
                 h_rel_m=np.asarray(hs))
        print(f"[extract_rise_ref] wrote {args.out} "
              f"(T={len(qs)}, dt={env.dt}s, ramp_i0={ramp_i0}, "
              f"seed={seed}, start={args.start}, "
              f"blend_to_plant={args.blend_to_plant}, "
              f"ckpt={args.checkpoint.name}); reference ends "
              f"{h_rel_end_m * 1000:+.0f}mm above its start pose — "
              f"command goal.rise_height_mm near that for the "
              f"tracking arm.")
        return

    sys.exit("[extract_rise_ref] no seed in range produced a clean "
             "standing rise — reference NOT written. Try --start crouch "
             "(easier), widen --seeds, or a different checkpoint.")


if __name__ == "__main__":
    main()
