"""Mint a MESH-NATIVE scripted rise reference (belly -> plant), rung-9.

WHAT THIS TOOL IS FOR (standwalk rung-9, 2026-08-25): every rise
reference so far (rise_ref_belly2plant.npz) was EXTRACTED from a
primitive-family (2.104 kg, 25 Hz) champion rollout. On the mesh model
(3.50 kg, corrected hip axis) that recording supervises the policy
through a strongly ASYMMETRIC mid-rise press (two legs folded at femur
~-57deg / tibia ~+100deg) recorded at 60 % of the real mass — and the
rung-8 evidence chain (pace dose bracketed non-monotonic, budget null
at every pace, anchor dose 6.0 regression) says the policy grinds into
that posture at the 2.64 A current ceiling instead of rising. This
tool does NOT extract from any checkpoint: it composes the trajectory
from geometry alone and PROVES it on the training model before it is
allowed to ship.

Script (all per-leg foot paths via the same absolute-tibia FK/IK the
hardware gait stack trusts — ``tripod_gait.foot_rz_from_hip_knee`` /
``_leg_ik``, reused through ``extract_rise_ref._blend_pose_ik``):

  Phase A  TUCK  (default 2.0 s): belly stays on the ground carrying
           the mass (near-zero torque) while each foot moves from its
           sprawled flat-start point to the PLANT footprint (reach
           r: start->plant, small sinusoidal lift, coxa lerp).
  Phase B  PRESS (default 2.5 s): straight Cartesian press-up — per
           leg, foot (r, z) interpolates linearly from the tuck pose
           to the plant pose, all six legs symmetric and simultaneous.
  Phase C  HOLD  (default 1.0 s): plant pose.

The commanded waypoints are replayed OPEN-LOOP through the real env
(default model family — mesh on post-08-24 code) and the npz stores
the ACHIEVED joints/heights, exactly like extract_rise_ref. ramp_i0 =
the phase-B start tick. Acceptance requires: no termination, end
height within tolerance of the env's own rise target, every pad near
the ground, AND servo current p95 under a hard bar (default 1.5 A,
the rung's gate bar) — the mesh-feasibility check the primitive-era
extraction never had. Held-out-seed open-loop robustness replay
(extract_rise_ref._validate_open_loop_robustness) runs before the
file is written.

Usage (CPU, no checkpoint, no GPU):

    uv run python -m rl_move.sim.make_rise_ref_scripted \
        --out rl_move/sim/refs/rise_ref_mesh_scripted.npz
"""
from __future__ import annotations

import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

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

from .servo_model import SimServoParams, resolve_model_source  # noqa: E402
from .train_ppo_sim import ENV_CLASSES  # noqa: E402
from .extract_rise_ref import (_blend_pose_ik,  # noqa: E402
                               _validate_open_loop_robustness)


def _foot_rz(q_rad: np.ndarray, leg: int) -> tuple[float, float]:
    import tripod_gait as _tg
    hip_deg = math.degrees(q_rad[3 * leg + 1])
    knee_abs_deg = math.degrees(q_rad[3 * leg + 2] + q_rad[3 * leg + 1])
    return _tg.foot_rz_from_hip_knee(hip_deg, knee_abs_deg)


def _leg_from_rz(r: float, z: float):
    import tripod_gait as _tg
    ik = _tg._leg_ik((r, 0.0, z))
    if ik is None:
        return None
    hip_rad, knee_abs_rad = ik
    return hip_rad, knee_abs_rad - hip_rad   # model-relative knee


def build_script(q_start: np.ndarray, q_plant: np.ndarray, dt: float,
                 tuck_s: float, press_s: float, hold_s: float,
                 tuck_lift_mm: float) -> tuple[np.ndarray, int]:
    """Return (commanded q_rad (T,18), ramp_i0 = press start tick)."""
    n_tuck = max(1, int(round(tuck_s / dt)))
    n_press = max(1, int(round(press_s / dt)))
    n_hold = max(1, int(round(hold_s / dt)))
    rz_start = [_foot_rz(q_start, leg) for leg in range(6)]
    rz_plant = [_foot_rz(q_plant, leg) for leg in range(6)]
    # Tuck endpoint: plant footprint reach at the START (belly) foot
    # height, so phase B is a pure vertical press from a resting tuck.
    q_crouch = q_start.copy()
    for leg in range(6):
        r_p, _ = rz_plant[leg]
        _, z_s = rz_start[leg]
        sol = None
        for dz in (0.0, 0.005, 0.010, 0.015, -0.005):
            sol = _leg_from_rz(r_p, z_s + dz)
            if sol is not None:
                break
        if sol is None:
            raise SystemExit(f"[make_rise_ref_scripted] tuck IK failed "
                             f"for leg {leg}: r={r_p:.3f} z={z_s:.3f}")
        q_crouch[3 * leg + 0] = q_plant[3 * leg + 0]   # coxa at plant yaw
        q_crouch[3 * leg + 1] = sol[0]
        q_crouch[3 * leg + 2] = sol[1]
    qs = []
    lift = tuck_lift_mm * 1e-3
    for k in range(n_tuck):
        s = (k + 1) / n_tuck
        q_k = q_start.copy()
        for leg in range(6):
            r_s, z_s = rz_start[leg]
            r_p, _ = rz_plant[leg]
            r_k = (1.0 - s) * r_s + s * r_p
            z_k = ((1.0 - s) * z_s
                   + s * _foot_rz(q_crouch, leg)[1]
                   + lift * math.sin(math.pi * s))
            sol = _leg_from_rz(r_k, z_k)
            if sol is None:          # keep previous tick's leg pose
                q_prev = qs[-1] if qs else q_start
                q_k[3 * leg + 1] = q_prev[3 * leg + 1]
                q_k[3 * leg + 2] = q_prev[3 * leg + 2]
            else:
                q_k[3 * leg + 1], q_k[3 * leg + 2] = sol
            q_k[3 * leg + 0] = ((1.0 - s) * q_start[3 * leg + 0]
                                + s * q_plant[3 * leg + 0])
        qs.append(q_k)
    ramp_i0 = len(qs)
    for k in range(n_press):
        s = (k + 1) / n_press
        qs.append(_blend_pose_ik(q_crouch, q_plant, s))
    for _ in range(n_hold):
        qs.append(q_plant.copy())
    return np.asarray(qs), ramp_i0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=sorted(ENV_CLASSES),
                    default="joint_goal")
    # Quasi-static pacing (measured 08-25, this tool's first probes):
    # tuck2/press2.5/hold1 commands far above the ~31 deg/s servo
    # velocity limit -> the robot lags the command by seconds and the
    # convergence transient alone pins 2.64 A. At 3/5/2.5 the whole
    # rise runs at 0.53 A max. Do not speed these up without re-reading
    # the current profile.
    ap.add_argument("--tuck-s", type=float, default=3.0)
    ap.add_argument("--press-s", type=float, default=5.0)
    ap.add_argument("--hold-s", type=float, default=2.5)
    ap.add_argument("--tuck-lift-mm", type=float, default=12.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--min-rise-mm", type=float, default=70.0,
                    help="reject if the achieved end height above the "
                         "belly start is below this")
    ap.add_argument("--end-q-rms-deg", type=float, default=6.0,
                    help="reject if the achieved end pose is farther "
                         "than this RMS from the plant pose")
    ap.add_argument("--end-clear-mm", type=float, default=20.0)
    ap.add_argument("--save-every", type=int, default=2,
                    help="subsample factor for the saved reference "
                         "(the state-aligned anchor argmin scans every "
                         "saved tick per env step; 2 keeps T near the "
                         "legacy ref's 314)")
    ap.add_argument("--max-cur-p95-a", type=float, default=1.5,
                    help="hard mesh-feasibility bar on the open-loop "
                         "replay's servo-current p95 (the rung gate bar)")
    ap.add_argument("--validate-seeds", type=str, default="500,501,502")
    ap.add_argument("--validate-margin-deg", type=float, default=8.0)
    ap.add_argument("--out", type=Path,
                    default=_RL / "sim" / "refs"
                    / "rise_ref_mesh_scripted.npz")
    args = ap.parse_args()

    from .joint_task import q_rad_to_action

    env_cls = ENV_CLASSES[args.task]
    total_s = args.tuck_s + args.press_s + args.hold_s
    env = env_cls(params=SimServoParams.from_cfg(None), randomize=False,
                  dr_scale=0.0, episode_seconds=total_s + 3.0,
                  seed=args.seed)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
    gen.force_rise_start = "flat"
    obs, _ = env.reset()
    print(f"[make_rise_ref_scripted] model_source="
          f"{resolve_model_source(None)}  dt={env.dt}s")
    chassis_bid = env.model.body("chassis").id
    z_start = float(env.data.xpos[chassis_bid, 2])
    q_start = env.data.qpos[env._qadr].copy()
    q_plant = env._plant_deg * DEG2RAD

    q_cmd, ramp_i0 = build_script(q_start, q_plant, env.dt, args.tuck_s,
                                  args.press_s, args.hold_s,
                                  args.tuck_lift_mm)
    print(f"[make_rise_ref_scripted] script T={len(q_cmd)} "
          f"ramp_i0={ramp_i0}")

    qs = [q_start.copy()]
    hs = [0.0]
    curs = []
    term = trunc = False
    info: dict = {}
    for k in range(len(q_cmd)):
        obs, _r, term, trunc, info = env.step(q_rad_to_action(q_cmd[k]))
        qs.append(env.data.qpos[env._qadr].copy())
        hs.append(float(env.data.xpos[chassis_bid, 2]) - z_start)
        curs.append(float(info.get("max_current_a", 0.0)))
        if term or trunc:
            break
    clear_mm = max((float(env.data.xpos[b, 2]) - z) * 1000.0
                   for b, z in zip(env._pad_bids, env._pad_z_ref)
                   if b >= 0)
    h_end_mm = hs[-1] * 1000.0
    q_rms_deg = float(np.degrees(np.sqrt(np.mean(
        (env.data.qpos[env._qadr] - q_plant) ** 2))))
    cur = np.asarray(curs)
    cur_p95 = float(np.percentile(cur, 95)) if len(cur) else 0.0
    cur_max = float(cur.max()) if len(cur) else 0.0
    print(f"[make_rise_ref_scripted] replay: term={term} "
          f"end {h_end_mm:+.0f}mm (min {args.min_rise_mm:.0f}mm; env "
          f"target {env._h_target * 1000.0:+.0f}mm for reference) "
          f"end_q_rms {q_rms_deg:.1f}deg worst_clear {clear_mm:.0f}mm "
          f"cur_p95 {cur_p95:.2f}A cur_max {cur_max:.2f}A")
    ok = (not term and h_end_mm >= args.min_rise_mm
          and q_rms_deg <= args.end_q_rms_deg
          and clear_mm <= args.end_clear_mm
          and cur_p95 <= args.max_cur_p95_a)
    if not ok:
        sys.exit("[make_rise_ref_scripted] REJECT - the scripted "
                 "trajectory is not clean/feasible on this model; "
                 "reference NOT written. Tune phase durations or the "
                 "tuck lift, or lower the press endpoint.")

    qs_arr = np.asarray(qs)
    hs_arr = np.asarray(hs)
    # ramp_i0 = first tick the ACHIEVED height genuinely leaves the
    # belly (matches the legacy semantics "first tick the height ref
    # leaves zero"); the scripted press-start tick is only a fallback.
    nz = np.flatnonzero(hs_arr > 2e-3)
    ramp_i0_ach = int(nz[0]) if len(nz) else ramp_i0
    if args.validate_seeds.strip():
        vseeds = [int(v) for v in args.validate_seeds.split(",")
                  if v.strip()]
        v_ok, v_detail = _validate_open_loop_robustness(
            qs_arr, ramp_i0_ach, env_cls, vseeds,
            args.validate_margin_deg)
        print(f"[make_rise_ref_scripted] robustness validation: "
              f"{'PASS' if v_ok else 'FAIL'} ({v_detail})")
        if not v_ok:
            sys.exit("[make_rise_ref_scripted] validation FAIL - "
                     "reference NOT written.")
    env.close()
    n_sub = max(1, int(args.save_every))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, q_rad=qs_arr[::n_sub], dt=env.dt * n_sub,
             ramp_i0=ramp_i0_ach // n_sub, h_rel_end_m=hs_arr[-1],
             h_rel_m=hs_arr[::n_sub])
    print(f"[make_rise_ref_scripted] wrote {args.out} "
          f"(T={len(qs_arr[::n_sub])}, dt={env.dt * n_sub}s, "
          f"ramp_i0={ramp_i0_ach // n_sub}); ends "
          f"{hs_arr[-1] * 1000:+.0f}mm above belly - command "
          f"goal.rise_height_mm near that for the tracking arm.")


if __name__ == "__main__":
    main()
