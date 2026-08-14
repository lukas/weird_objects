"""Measure foot slip of the step-then-shift gait (vs the drag gait).

Replays ``linux_control/noslip_gait.NoSlipGait`` through the sim env's
full servo/safety stack at 25 Hz and measures, per foot:

  TRUE SCRUB  integral of the tangential velocity of the contacting
              material point (via the contact Jacobian). This is the
              physical definition of slipping; rolling of the 7 mm
              sphere foot contributes ZERO here.
  net drift   world XY displacement of the foot site over each contact
              episode (includes rolling, ~2.5 mm/stance, and touchdown
              settling — an upper bound, not all of it is slip).

Also reports commanded vs actual body travel.

    .venv/bin/python -m rl_move.sim.verify_noslip
    .venv/bin/python -m rl_move.sim.verify_noslip --compare   # + drag gait
    .venv/bin/python -m rl_move.sim.verify_noslip --mu 0.6    # calibrated mu
    .venv/bin/python -m rl_move.sim.verify_noslip --video out.mp4

Context: the classic body-frame drag gait (tripod_gait.TripodGait)
measured 0.50-0.51 travel ratio on hardware (tape, 2026-08-10); the
whole point of NoSlipGait is that planted feet are commanded to fixed
world anchors, so slip should collapse to servo-tracking residuals.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402

PLANT_HIP_DEG = 20.0
PLANT_KNEE_DEG = 80.0
HOLD_S = 1.5
TOUCH_N = 0.5      # N; feet carry ~3.4 N each at rest


def _make_env(mu: float, servo_params: str, seed: int,
              episode_s: float, render: bool,
              write_speed: int = 1500, write_acc: int = 80,
              vel_max_deg_s: float | None = None):
    import dataclasses

    from rl_move.sim.joint_task import SimHexapodJointGoalEnv
    from rl_move.sim.servo_model import SimServoParams

    cfg = load_config()
    if servo_params:
        cfg.setdefault("bus", {})["servo_params"] = servo_params
    if mu > 0:
        cfg.setdefault("env", {})["foot_friction_slide"] = float(mu)
    # Servo write profile. The tape runs / drag gait used (1500, 30);
    # acc 30 (~264 deg/s^2) cannot follow a sub-second swing sweep, so
    # the default here is a still-conservative acc 80 — the STS3215 acc
    # register goes to 254 and the swing legs are unloaded.
    cfg.setdefault("bus", {})["write_speed"] = int(write_speed)
    cfg["bus"]["write_acc"] = int(write_acc)
    cfg.setdefault("safety", {})["max_delta_q_deg"] = 8.0
    params = SimServoParams.from_cfg(cfg)
    # The fitted vel_max_deg_s is the profile CRUISE SPEED the system-ID
    # test happened to write (350 counts/s = 30.8 deg/s), NOT the
    # servo's capability — with it in place every write_speed above 350
    # is silently clamped (calibrate_slip's 1500 included). On hardware
    # the profile cruises at the written speed, so default to matching
    # vel_max to write_speed; pass --vel-max 0 to keep the fitted clamp.
    if vel_max_deg_s is None:
        vel_max_deg_s = write_speed * 360.0 / 4096.0
    if vel_max_deg_s > 0:
        params.axes = {
            ax: dataclasses.replace(p, vel_max_deg_s=float(vel_max_deg_s))
            for ax, p in params.axes.items()}
    env = SimHexapodJointGoalEnv(
        cfg=cfg, params=params, randomize=False,
        dr_scale=0.0, episode_seconds=episode_s, seed=seed,
        plant_deg=[0.0, PLANT_HIP_DEG, PLANT_KNEE_DEG] * 6,
        render_mode="rgb_array" if render else None)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "hold" else 0.0)
    return env


def _foot_site_ids(env) -> list[int]:
    import mujoco
    sids = []
    for i in range(6):
        sid = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_SITE, f"L{i}_foot_site")
        assert sid >= 0, f"missing site L{i}_foot_site"
        sids.append(sid)
    return sids


def _scrub_speeds(env, foot_gids: list[int], floor_gid: int) -> np.ndarray:
    """True per-foot slip: |tangential material-point velocity| (m/s).

    Rolling of the 7 mm sphere foot moves the sphere CENTRE without any
    scrubbing, so site displacement over-counts; this evaluates the
    contact-point velocity through the Jacobian instead (rolling -> 0).
    """
    import mujoco
    out = np.zeros(6)
    nv = env.model.nv
    jacp = np.zeros((3, nv))
    for c in range(env.data.ncon):
        con = env.data.contact[c]
        g1, g2 = con.geom1, con.geom2
        if floor_gid not in (g1, g2):
            continue
        other = g2 if g1 == floor_gid else g1
        if other not in foot_gids:
            continue
        i = foot_gids.index(other)
        bid = env.model.geom_bodyid[other]
        mujoco.mj_jac(env.model, env.data, jacp, None, con.pos, bid)
        v = jacp @ env.data.qvel
        n = con.frame[:3]
        vt = v - np.dot(v, n) * n
        out[i] = max(out[i], float(np.linalg.norm(vt[:2])))
    return out


def _geom_ids(env) -> tuple[list[int], int]:
    import mujoco
    foot_gids = [mujoco.mj_name2id(
        env.model, mujoco.mjtObj.mjOBJ_GEOM, f"L{i}_foot") for i in range(6)]
    floor_gid = mujoco.mj_name2id(
        env.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    assert floor_gid >= 0 and all(g >= 0 for g in foot_gids)
    return foot_gids, floor_gid


def rollout(env, gait, walk_s: float, *, cmd_dist_fn,
            video_writer=None) -> dict:
    """Hold plant, then run ``gait.desired_deg`` for walk_s; measure slip."""
    from rl_move.sim.joint_task import q_rad_to_action

    env.reset()
    sids = _foot_site_ids(env)
    foot_gids, floor_gid = _geom_ids(env)
    plant_rad = np.array([0.0, PLANT_HIP_DEG, PLANT_KNEE_DEG] * 6) * DEG2RAD
    hold_steps = int(round(HOLD_S / env.dt))
    walk_steps = int(round(walk_s / env.dt))

    gross = np.zeros(6)           # sum of |per-tick| motion in contact
    net = np.zeros(6)             # sum of |net displacement| per episode
    scrub = np.zeros(6)           # integral of tangential contact speed
    ep_start = [None] * 6         # site xy at contact-episode start
    stance_ticks = np.zeros(6)
    max_tick_slip = 0.0
    prev_xy = None
    prev_touch = None
    xy0 = None
    terminated = ""
    for step in range(hold_steps + walk_steps):
        walking = step >= hold_steps
        if not walking:
            act = q_rad_to_action(plant_rad)
        else:
            t = (step - hold_steps) * env.dt
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        _obs, _r, term, trunc, info = env.step(act)
        if walking:
            if xy0 is None:
                xy0 = env.data.xpos[env._chassis_bid, :2].copy()
            xy = env.data.site_xpos[sids][:, :2].copy()
            touch = np.array([env.data.sensordata[a]
                              for a in env._touch_adr])
            inc = touch > TOUCH_N
            scrub += _scrub_speeds(env, foot_gids, floor_gid) * env.dt
            for i in range(6):
                if inc[i] and ep_start[i] is None:
                    ep_start[i] = xy[i].copy()
                elif not inc[i] and ep_start[i] is not None:
                    net[i] += float(np.linalg.norm(
                        prev_xy[i] - ep_start[i]))
                    ep_start[i] = None
            if prev_xy is not None:
                both = inc & (prev_touch > TOUCH_N)
                d = np.linalg.norm(xy - prev_xy, axis=1)
                gross += np.where(both, d, 0.0)
                stance_ticks += both
                if both.any():
                    max_tick_slip = max(max_tick_slip,
                                        float(d[both].max()))
            prev_xy, prev_touch = xy, touch
            if video_writer is not None:
                import cv2
                video_writer.write(
                    cv2.cvtColor(env.render(), cv2.COLOR_RGB2BGR))
        if term or trunc:
            terminated = info.get("termination_reason") or "trunc"
            break
    for i in range(6):
        if ep_start[i] is not None and prev_xy is not None:
            net[i] += float(np.linalg.norm(prev_xy[i] - ep_start[i]))
    xy_end = env.data.xpos[env._chassis_bid, :2].copy()
    travel_m = float(np.linalg.norm(xy_end - xy0)) if xy0 is not None else 0.0
    cmd_m = float(cmd_dist_fn())
    return {
        "travel_m": travel_m,
        "cmd_m": cmd_m,
        "ratio": travel_m / cmd_m if cmd_m > 1e-9 else float("nan"),
        "slip_mm": net * 1000.0,
        "slip_total_mm": float(net.sum()) * 1000.0,
        "gross_mm": float(gross.sum()) * 1000.0,
        "scrub_mm": scrub * 1000.0,
        "scrub_total_mm": float(scrub.sum()) * 1000.0,
        "slip_per_m": (float(net.sum()) / travel_m * 1000.0
                       if travel_m > 1e-6 else float("nan")),
        "max_tick_slip_mm": max_tick_slip * 1000.0,
        "stance_duty": float(stance_ticks.mean()) / max(walk_steps - 1, 1),
        "terminated": terminated,
    }


def _report(name: str, r: dict) -> None:
    per_foot = " ".join(f"{v:5.1f}" for v in r["slip_mm"])
    per_scrub = " ".join(f"{v:5.1f}" for v in r["scrub_mm"])
    print(f"\n{name}")
    print(f"  travel {r['travel_m']:.3f} m of {r['cmd_m']:.3f} m commanded"
          f"  -> ratio {r['ratio']:.3f}")
    print(f"  TRUE SCRUB (tangential contact-point motion, rolling"
          f" excluded): {r['scrub_total_mm']:.1f} mm total")
    print(f"    per foot (mm): [{per_scrub}]")
    print(f"  net foot-centre drift in contact: {r['slip_total_mm']:.1f} mm"
          f"  ({r['slip_per_m']:.1f} mm per m traveled;"
          f" gross incl. vibration {r['gross_mm']:.1f} mm)")
    print(f"    per foot (mm): [{per_foot}]"
          f"   max single tick: {r['max_tick_slip_mm']:.2f} mm")
    print(f"  mean stance duty {r['stance_duty']:.2f}"
          + (f"   TERMINATED: {r['terminated']}" if r["terminated"] else ""))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--vx", type=float, default=0.02)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--omega", type=float, default=0.0)
    ap.add_argument("--alpha", type=float, default=0.0,
                    help="body-motion overlap: 0 = step-then-shift, "
                    "1 = continuous body drift (world anchors either way)")
    ap.add_argument("--period", type=float, default=3.2)
    ap.add_argument("--shift-frac", type=float, default=0.24)
    ap.add_argument("--swing-frac", type=float, default=0.22)
    ap.add_argument("--lift-mm", type=float, default=28.0)
    ap.add_argument("--walk-s", type=float, default=16.0)
    ap.add_argument("--write-speed", type=int, default=1500)
    ap.add_argument("--write-acc", type=int, default=80)
    ap.add_argument("--vel-max", type=float, default=None,
                    help="servo cruise ceiling deg/s (default: match "
                    "write-speed; 0 = keep the fitted 30.8 clamp)")
    ap.add_argument("--mu", type=float, default=0.0,
                    help="foot-ground slide friction (0 = XML default 2.0)")
    ap.add_argument("--servo-params", default="",
                    help="'' = air fit, 'loaded' = 08-10 loaded bench fit")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--compare", action="store_true",
                    help="also replay the classic drag gait (tripod_gait)")
    ap.add_argument("--video", type=Path, default=None,
                    help="write an mp4 of the noslip rollout")
    args = ap.parse_args()

    from noslip_gait import NoSlipGait

    episode_s = HOLD_S + args.walk_s + 1.0
    writer = None
    env = _make_env(args.mu, args.servo_params, args.seed, episode_s,
                    render=args.video is not None,
                    write_speed=args.write_speed, write_acc=args.write_acc,
                    vel_max_deg_s=args.vel_max)
    if args.video is not None:
        import cv2
        writer = cv2.VideoWriter(
            str(args.video), cv2.VideoWriter_fourcc(*"mp4v"),
            int(round(1.0 / env.dt)), (640, 480))

    gait = NoSlipGait(period=args.period, lift=args.lift_mm * 1e-3,
                      shift_frac=args.shift_frac, swing_frac=args.swing_frac,
                      vx=args.vx, vy=args.vy, omega=args.omega,
                      alpha=args.alpha)
    gait.sync_plant_stance(PLANT_HIP_DEG, PLANT_KNEE_DEG)

    def cmd_dist():
        x, y, _ = gait.commanded_pose()
        return float(np.hypot(x, y))

    r = rollout(env, gait, args.walk_s, cmd_dist_fn=cmd_dist,
                video_writer=writer)
    _report(f"noslip alpha={args.alpha}  vx={args.vx} vy={args.vy} "
            f"omega={args.omega} period={args.period}s "
            f"lift={args.lift_mm}mm  walk {args.walk_s}s "
            f"mu={'XML(2.0)' if args.mu <= 0 else args.mu}", r)
    if writer is not None:
        writer.release()
        print(f"  video: {args.video}")
    env.close()

    if args.compare:
        from tripod_gait import TripodGait

        env2 = _make_env(args.mu, args.servo_params, args.seed, episode_s,
                         render=False, write_speed=args.write_speed,
                         write_acc=args.write_acc,
                         vel_max_deg_s=args.vel_max)
        drag = TripodGait(vx=args.vx, vy=args.vy, omega=args.omega)
        drag.sync_plant_stance(PLANT_HIP_DEG, PLANT_KNEE_DEG)
        drag.set_lift_mm(args.lift_mm)
        drag.reset_phase()
        r2 = rollout(env2, drag, args.walk_s,
                     cmd_dist_fn=lambda: np.hypot(args.vx, args.vy)
                     * args.walk_s)
        _report("classic drag gait (tripod_gait.TripodGait), same settings",
                r2)
        env2.close()


if __name__ == "__main__":
    main()
