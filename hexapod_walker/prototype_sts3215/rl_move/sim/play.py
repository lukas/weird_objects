"""BOTH champions in one sim: stand up, walk around, sit down.

    ./sim_play.sh          # from prototype_sts3215/  (plain python, NO mjpython)

One physics env (the walk env) + two policies:

- STANCE policy (ppo_goal_cw_stance_dr10, obs 68) is active while no
  velocity is commanded: stand up / sit down / lean / height.
- WALK policy (ppo_goal_cw_walk_longdist_r2, obs 72) takes over the
  moment you command a velocity, and hands back on stop.

This works because the walk env's obs is the stance obs (59 proprio +
9 goal = 68) with 4 walk dims appended (vx/vy ref + measured vx/vy) —
the stance policy just reads the first 68 dims of the same observation.

Rendered offscreen into an OpenCV window: unlike mujoco.viewer there
are NO built-in key bindings, so every key below is ours and nothing
else ever fires.

The two champions stand in DIFFERENT poses: the stance line's belly
rise ends in a ~72 mm crouch-stand while the walk line lives around the
~142 mm plant stance (joint gap up to 104 deg). Key 7 therefore runs a
3-phase auto sequence: stance-policy rise (~9 s) -> scripted 1.5 s
joint-target blend to the plant pose -> re-anchor the episode in the
plant frame. After that, walking / stopping / sitting are all exactly
the walk champion's training distribution.

Keys (window must have focus):
    7           stand up from belly (fully automatic, ~11 s total)
    8           sit down (crouch; from a belly episode: back to floor)
    9 / R       reset to standing (plant, at the origin)
    B           reset belly-down (then 7 to stand)
    I/K Up/Down forward / backward speed (+-0.01 m/s, engages WALK)
    J/L L/R     strafe left / right
    0 / Space   stop -> STANCE policy holds
    = / -       body height +/- 5 mm (stance)
    Q / Esc     quit
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np

from rl_move.env import TaskGoal
from .view import _InteractiveTraj
from .walk_task import SimHexapodJointWalkEnv, WalkGoal

_STEP = 0.01          # m/s per keypress
_SPEED_MAX = 0.06     # champion's trained command band tops out here
_UP, _DOWN, _LEFT, _RIGHT = 63232, 63233, 63234, 63235   # macOS cv2 arrows


class _PlayTraj(_InteractiveTraj):
    """Interactive stance goals + a live velocity command.

    Velocity ramps at ~0.06 m/s^2 toward the keyed target — training
    commands eased in over ~1 s, so instant steps are avoided the same
    way the tilt/height refs are ramped.
    """

    VEL_RATE = 0.06

    def __init__(self, dt: float = 0.04):
        super().__init__(dt)
        self.vx = 0.0           # user targets (keyboard writes here)
        self.vy = 0.0
        self._pvx = 0.0         # published (ramped) command
        self._pvy = 0.0

    def reset_published(self) -> None:
        super().reset_published()
        self._pvx = self._pvy = 0.0

    def at(self, step: int) -> WalkGoal:
        n = max(step - self._last_step, 0)
        dt = n * self._dt
        base = super().at(step)             # ramps tilt/height refs
        self._pvx = self._toward(self._pvx, self.vx, self.VEL_RATE * dt)
        self._pvy = self._toward(self._pvy, self.vy, self.VEL_RATE * dt)
        return WalkGoal(roll_ref=base.roll_ref, pitch_ref=base.pitch_ref,
                        height_ref=base.height_ref,
                        unload_leg=base.unload_leg,
                        vx_ref=self._pvx, vy_ref=self._pvy)


class _PlayEnv(SimHexapodJointWalkEnv):
    def __init__(self, *a, **kw):
        self.traj = _PlayTraj()
        super().__init__(*a, **kw)

    def _sample_goal(self):
        return self.traj


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stance", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_stance_dr10.zip"))
    ap.add_argument("--walk", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_walk_longdist_r2.zip"))
    ap.add_argument("--realtime", type=float, default=1.0)
    args = ap.parse_args()

    import cv2
    import mujoco
    from stable_baselines3 import PPO

    from .joint_task import q_rad_to_action
    from .servo_model import SimServoParams

    env = _PlayEnv(params=SimServoParams.load(), randomize=False,
                   episode_seconds=3600.0, render_mode="rgb_array")
    stance = PPO.load(args.stance, device="cpu")
    walk = PPO.load(args.walk, device="cpu")
    n_stance = int(stance.observation_space.shape[0])
    assert walk.observation_space.shape == env.observation_space.shape, (
        f"walk policy obs {walk.observation_space.shape} != env "
        f"{env.observation_space.shape}")
    assert n_stance < int(env.observation_space.shape[0]), (
        "stance policy obs must be a prefix of the walk env obs")

    traj = env.traj
    chassis_bid = env.model.body("chassis").id
    msg = ""
    # phases of the 7-key auto stand: None | ("rise", steps) | ("blend", k)
    auto: list | None = None

    def chassis_z() -> float:
        return float(env.data.xpos[chassis_bid, 2])

    def q_now() -> np.ndarray:
        return env.data.qpos[7:25].copy()

    def do_reset(start: str, h_goal: float, note: str) -> None:
        nonlocal obs, msg, auto
        auto = None
        traj.start_at = start
        traj.goal = TaskGoal()
        traj.goal.height_ref = h_goal
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        obs, _ = env.reset()
        msg = note

    # First reset is a plant stand: capture the walk line's standing
    # joint pose — the blend target for the auto stand-up.
    traj.start_at = "plant"
    obs, _ = env.reset()
    q_plant = q_now()

    RISE_S = 9.5                          # curl 5 s + rise ramp, plus slack
    BLEND_N = int(round(1.5 / env.dt))    # scripted crouch->plant blend
    q_blend_from = q_plant

    def re_anchor_plant() -> None:
        # The robot is physically AT the plant pose now; reset the episode
        # so goal frames (height 0 = standing tall) and both policies see
        # the training distribution, then put the body back where it was.
        nonlocal obs
        keep = env.data.qpos[:7].copy()
        traj.start_at = "plant"
        traj.goal = TaskGoal()
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        obs, _ = env.reset()
        env.data.qpos[:2] = keep[:2]      # keep XY; reset's height/quat
        mujoco.mj_forward(env.model, env.data)

    win = "hexapod play"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 720)

    while True:
        t0 = time.monotonic()
        cmd_speed = float(np.hypot(traj.vx, traj.vy))
        walking = cmd_speed > 1e-3 and auto is None
        if auto is not None and auto[0] == "rise":
            action, _ = stance.predict(obs[:n_stance], deterministic=True)
            auto[1] += 1
            if auto[1] * env.dt >= RISE_S:
                if chassis_z() > 0.06:
                    q_blend_from = q_now()
                    auto = ["blend", 0]
                    msg = "blending to the walk stance..."
                else:
                    auto = None
                    msg = "stand-up failed - press 9 to reset standing"
        elif auto is not None and auto[0] == "blend":
            auto[1] += 1
            s = min(auto[1] / BLEND_N, 1.0)
            action = q_rad_to_action(
                (1.0 - s) * q_blend_from + s * q_plant)
            if auto[1] >= BLEND_N:
                re_anchor_plant()
                auto = None
                msg = "standing tall - I/K/J/L to walk, 8 to sit"
        elif walking:
            action, _ = walk.predict(obs, deterministic=True)
        else:
            action, _ = stance.predict(obs[:n_stance], deterministic=True)
        obs, _r, term, trunc, info = env.step(action)
        if term or trunc:
            do_reset("plant", 0.0,
                     f"[{info.get('termination_reason') or 'episode end'}]"
                     " auto-reset standing")

        # --- HUD ---------------------------------------------------------
        frame = env.render()
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        v = env._body_vel_xy()
        g = traj.goal
        if auto is not None:
            mode_txt = ("STANDING UP (auto): "
                        + ("curl + rise..." if auto[0] == "rise"
                           else "blend to walk stance..."))
            mode_col = (0, 200, 255)
        elif walking:
            mode_txt, mode_col = "WALK policy", (40, 240, 40)
        else:
            mode_txt, mode_col = "STANCE policy", (240, 200, 40)
        lines = [
            (f"{mode_txt}   height {chassis_z() * 1000:.0f}mm", mode_col),
            (f"CMD vx {traj.vx:+.3f} vy {traj.vy:+.3f} m/s   "
             f"ACTUAL vx {v[0]:+.3f} vy {v[1]:+.3f}", (200, 200, 40)),
            (f"goal height {g.height_ref * 1000:+.0f}mm  "
             f"roll {math.degrees(g.roll_ref):+.1f}  "
             f"pitch {math.degrees(g.pitch_ref):+.1f}", (200, 200, 200)),
            ("7 stand up   8 sit   9 reset standing   B belly reset",
             (180, 180, 180)),
            ("I/K fwd/back  J/L strafe  0/space stop  =/- height  Q quit",
             (180, 180, 180)),
        ]
        if msg:
            lines.append((msg, (0, 200, 255)))
        for i, (line, color) in enumerate(lines):
            cv2.putText(img, line, (10, 24 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1,
                        cv2.LINE_AA)
        cv2.imshow(win, img)

        # --- keys (all ours: cv2 has no built-in bindings) ------------------
        k = cv2.waitKeyEx(1)

        def engage_walk() -> bool:
            # The walk champion lives around the ~142 mm plant stance;
            # walking from the floor or the 72 mm crouch-stand collapses
            # into a belly shuffle (measured) — make the user stand first.
            nonlocal msg
            if auto is not None:
                return False
            if chassis_z() < 0.09:
                msg = "too low to walk - press 7 to stand up first"
                return False
            # Walk champion trained at height/tilt refs = 0: snap the
            # published stance refs to nominal so its obs is in-distribution.
            traj.goal.roll_ref = traj.goal.pitch_ref = 0.0
            traj.goal.height_ref = 0.0
            traj._pub.roll_ref = traj._pub.pitch_ref = 0.0
            traj._pub.height_ref = 0.0
            msg = ""
            return True

        if k in (ord("i"), ord("I"), _UP):
            if engage_walk():
                traj.vx = float(np.clip(traj.vx + _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("k"), ord("K"), _DOWN):
            if engage_walk():
                traj.vx = float(np.clip(traj.vx - _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("j"), ord("J"), _LEFT):
            if engage_walk():
                traj.vy = float(np.clip(traj.vy + _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("l"), ord("L"), _RIGHT):
            if engage_walk():
                traj.vy = float(np.clip(traj.vy - _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("0"), ord(" ")):
            traj.vx = traj.vy = 0.0
            msg = "stopped - stance policy holding"
        elif k == ord("7"):
            do_reset("zero", 0.045,
                     "STAND: curl ~5s, rise, then blend - hands off")
            auto = ["rise", 0]
        elif k == ord("8"):
            traj.vx = traj.vy = 0.0
            # Height refs are relative to the episode's start pose:
            # belly-start episodes sit at 0, plant-start ones crouch.
            traj.goal.height_ref = (0.0 if traj.start_at in ("zero", "belly")
                                    else -0.06)
            msg = "sitting back down"
        elif k in (ord("9"), ord("r"), ord("R")):
            do_reset("plant", 0.0, "reset standing")
        elif k in (ord("b"), ord("B")):
            do_reset("zero", 0.0, "reset belly-down (7 to stand)")
        elif k == ord("="):
            traj.goal.height_ref = min(traj.goal.height_ref + 0.005, 0.06)
        elif k == ord("-"):
            traj.goal.height_ref = max(traj.goal.height_ref - 0.005, -0.06)
        elif k in (ord("q"), ord("Q"), 27):
            break
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break

        if args.realtime > 0:
            dt = env.dt / args.realtime - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
