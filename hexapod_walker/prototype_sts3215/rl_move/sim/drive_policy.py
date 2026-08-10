"""Interactive drive-around for a trained walk policy (OpenCV window).

Uses the env's own offscreen renderer (the same battle-tested path the
training videos use) + cv2.imshow — NO mujoco.viewer / mjpython, which
segfaults intermittently on macOS. Plain python works:

    cd hexapod_walker/prototype_sts3215
    .venv/bin/python -m rl_move.sim.drive_policy \
        rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

Controls (the "hexapod drive" window must have focus):
    I / Up      more forward speed      K / Down    less / backward
    J / Left    strafe left             L / Right   strafe right
    0 / Space   full stop               R           reset episode
    4           toggle QUAD mode (lift both fronts, stand on four —
                only meaningful on a quad-trained checkpoint, e.g. the
                cw-walk-joyquad* / cw-quad-hold* lineages; on older
                checkpoints the command bits are out-of-distribution
                inputs and the gait may wobble)
    Q / Esc     quit
    (policy has NO yaw command — steering = changing crab direction)

The commanded velocity is written into the episode's WalkTrajectory
arrays every tick, so the policy sees it exactly the way training
commands arrived (same obs slot, same scaling).
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

_STEP = 0.01          # m/s per keypress
_SPEED_MAX = 0.12     # trained command ceiling (goal.walk_speed_max default)

# cv2.waitKeyEx codes: macOS arrow keys.
_UP, _DOWN, _LEFT, _RIGHT = 63232, 63233, 63234, 63235


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--episode-seconds", type=float, default=600.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--realtime", type=float, default=1.0,
                    help="1.0 = wall-clock speed, 2.0 = 2x, 0 = uncapped")
    args = ap.parse_args()

    import cv2
    from stable_baselines3 import PPO

    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.load(),
        randomize=args.dr_scale > 0, dr_scale=args.dr_scale,
        episode_seconds=args.episode_seconds, seed=args.seed,
        render_mode="rgb_array")
    # Force every episode to be a walk episode.
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        setattr(gen, f"p_{m}", 0.0)

    model = PPO.load(args.checkpoint, device="cpu")
    assert model.observation_space.shape == env.observation_space.shape, (
        f"obs mismatch: policy {model.observation_space.shape} "
        f"vs env {env.observation_space.shape} — wrong cfg for this ckpt?")

    vx = vy = 0.0
    quad = False          # QUAD mode: lift fronts 0+5, stand on four

    def clamp(v: float) -> float:
        return float(np.clip(v, -_SPEED_MAX, _SPEED_MAX))

    def apply_cmd() -> None:
        traj = env._goal_traj
        if traj is not None and hasattr(traj, "vx"):
            traj.vx[:] = vx
            traj.vy[:] = vy
            # Quad command rides the goal one-hot (TaskGoal.lift_legs),
            # exactly the training-time encoding: both front bits hot.
            traj.lift_legs = (0, 5) if quad else None

    obs, _ = env.reset()
    apply_cmd()
    print(__doc__.split("Controls")[1].split("The commanded")[0])
    print("driving. trained speed band 0.02-0.06 m/s; "
          "tap I/Up 3-4x for the ~0.04 sweet spot.")

    win = "hexapod drive"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 720)
    msg = ""
    while True:
        t0 = time.monotonic()
        apply_cmd()
        action, _ = model.predict(obs, deterministic=True)
        obs, _r, term, trunc, info = env.step(action)
        if term or trunc:
            msg = f"[{info.get('termination_reason') or 'episode end'}] reset"
            print(msg)
            # Zero the command on reset: re-applying a held command to a
            # fresh standing start caused fall -> instant re-fall cascades
            # (17 consecutive tilt_roll resets, 2026-08-09 drive session).
            vx = vy = 0.0
            quad = False
            obs, _ = env.reset()
            apply_cmd()

        frame = env.render()
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        v = env.unwrapped._body_vel_xy()

        # --- HUD -----------------------------------------------------------
        spd = float(np.hypot(vx, vy))
        # Direction label: what the current command MEANS. Taps are +-0.01
        # each, so e.g. one K tap from +0.05 fwd is +0.04 — STILL FORWARD;
        # riders kept expecting one tap = reverse (2026-08-09 session).
        if quad:
            direction = "QUAD (fronts up)"
        elif spd < 0.005:
            direction = "STOP"
        else:
            parts = []
            if vx > 0.005:
                parts.append("FWD")
            elif vx < -0.005:
                parts.append("REVERSE")
            if vy > 0.005:
                parts.append("STRAFE-L")
            elif vy < -0.005:
                parts.append("STRAFE-R")
            direction = "+".join(parts)
        heading = float(np.degrees(np.arctan2(vy, vx))) if spd >= 0.005 else 0.0
        in_env = (vx >= -0.005 and abs(heading) <= 50 and spd <= 0.075)
        env_txt = ("IN trained envelope" if in_env else
                   "OUTSIDE trained envelope (fwd<=0.06, heading<=45deg)"
                   " - may stumble/fall")

        for i, (line, color) in enumerate((
                (f"CMD  {direction}   {spd:.3f} m/s @ {heading:+.0f} deg",
                 (40, 240, 40)),
                (f"     vx {vx:+.3f} (I/K +-0.01/tap)   "
                 f"vy {vy:+.3f} (J/L)", (40, 240, 40)),
                (f"ACTUAL  vx {v[0]:+.3f}  vy {v[1]:+.3f} m/s   "
                 f"(tracking err {np.hypot(v[0]-vx, v[1]-vy):.3f})",
                 (200, 200, 40)),
                (env_txt, (40, 200, 40) if in_env else (0, 60, 255)),
                ("I/K fwd/back  J/L strafe  0/space stop  4 quad  "
                 "R reset  Q quit",
                 (180, 180, 180)))):
            cv2.putText(img, line, (10, 24 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

        # Top-down compass, top-right: thick arrow = command, thin = actual.
        cx, cy, R = img.shape[1] - 90, 90, 70
        cv2.circle(img, (cx, cy), R, (90, 90, 90), 1, cv2.LINE_AA)
        cv2.putText(img, "fwd", (cx - 14, cy - R - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1)
        # 45deg trained wedge (forward hemisphere)
        for a in (-45, 45):
            ex = cx - int(R * np.sin(np.radians(a)))
            ey = cy - int(R * np.cos(np.radians(a)))
            cv2.line(img, (cx, cy), (ex, ey), (70, 120, 70), 1, cv2.LINE_AA)
        def _arrow(wx, wy, color, thick):
            n = np.hypot(wx, wy)
            if n < 0.004:
                return
            scale = R * min(n / _SPEED_MAX, 1.0) / n
            # env frame: +vx = forward = screen up; +vy = left = screen left
            cv2.arrowedLine(img, (cx, cy),
                            (cx - int(wy * scale), cy - int(wx * scale)),
                            color, thick, cv2.LINE_AA, tipLength=0.25)
        _arrow(vx, vy, (40, 240, 40) if in_env else (0, 60, 255), 3)
        _arrow(float(v[0]), float(v[1]), (200, 200, 40), 1)

        if msg:
            cv2.putText(img, msg, (10, 140), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.imshow(win, img)

        k = cv2.waitKeyEx(1)
        if k in (ord("i"), ord("I"), _UP):
            vx = clamp(vx + _STEP); msg = ""
        elif k in (ord("k"), ord("K"), _DOWN):
            vx = clamp(vx - _STEP); msg = ""
        elif k in (ord("j"), ord("J"), _LEFT):
            vy = clamp(vy + _STEP); msg = ""
        elif k in (ord("l"), ord("L"), _RIGHT):
            vy = clamp(vy - _STEP); msg = ""
        elif k in (ord("0"), ord(" ")):
            vx = vy = 0.0; msg = ""
        elif k == ord("4"):
            quad = not quad
            if quad:
                vx = vy = 0.0   # quad-hold trains at zero velocity
                msg = "QUAD: lifting fronts (0+5), standing on four"
            else:
                msg = "quad off - back on six"
        elif k in (ord("r"), ord("R")):
            obs, _ = env.reset()
            apply_cmd()
            msg = "manual reset"
        elif k in (ord("q"), ord("Q"), 27):
            break
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break

        if args.realtime > 0:
            dt = env.dt / args.realtime - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    cv2.destroyAllWindows()
    print("closed — clean exit")


if __name__ == "__main__":
    main()
