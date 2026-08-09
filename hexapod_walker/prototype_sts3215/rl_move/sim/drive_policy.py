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

    def clamp(v: float) -> float:
        return float(np.clip(v, -_SPEED_MAX, _SPEED_MAX))

    def apply_cmd() -> None:
        traj = env._goal_traj
        if traj is not None and hasattr(traj, "vx"):
            traj.vx[:] = vx
            traj.vy[:] = vy

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
            obs, _ = env.reset()
            apply_cmd()

        frame = env.render()
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        v = env.unwrapped._body_vel_xy()
        for i, line in enumerate((
                f"cmd  vx {vx:+.2f}  vy {vy:+.2f} m/s",
                f"vel  vx {v[0]:+.2f}  vy {v[1]:+.2f} m/s",
                "I/K fwd/back  J/L strafe  0 stop  R reset  Q quit")):
            cv2.putText(img, line, (10, 24 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (40, 240, 40), 1, cv2.LINE_AA)
        # Most checkpoints train forward-hemisphere only (heading <= ~45
        # deg, speed <= ~0.06): sustained reverse tips them (tilt_roll,
        # 2026-08-09). Warn instead of silently extrapolating.
        if vx < -0.01 or abs(vy) > abs(vx) + 0.02 or np.hypot(vx, vy) > 0.085:
            cv2.putText(img, "OUTSIDE TRAINED ENVELOPE (fwd <=0.06,"
                        " heading <=45deg) - may fall",
                        (10, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 60, 255), 1, cv2.LINE_AA)
        if msg:
            cv2.putText(img, msg, (10, 100), cv2.FONT_HERSHEY_SIMPLEX,
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
