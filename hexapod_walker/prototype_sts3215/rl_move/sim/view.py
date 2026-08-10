"""Open the hexapod sim in the interactive MuJoCo viewer.

Two ways to play:

  # 1. Pose & poke (default): servos hold the plant stance. Drag the
  #    robot around (double-click a body, then ctrl+drag = apply force,
  #    ctrl+right-drag = torque), move joints from the Control tab.
  ../../.venv/bin/mjpython -m rl_move.sim.view
  ../../.venv/bin/mjpython -m rl_move.sim.view --pose zero   # belly down

  # 2. Drive the trained policy YOURSELF (default when --policy given):
  #    you type goals, the policy executes them, you can also shove it.
  ../../.venv/bin/mjpython -m rl_move.sim.view \
      --policy rl_move/sim/policies/ppo_goal.zip

  #    Keys (click the viewer window first so it has focus):
  #      arrows      lean target: left/right = roll, up/down = pitch (0.5°)
  #      U / J       body height target up / down (5 mm)
  #      1..6        unload leg L0..L5 (press again to clear)
  #      0 or C      clear all goals (plain hold)
  #      R           reset episode at the plant stance
  #      B           reset belly-down at the zero pose (then U-U-U to
  #                  command a rise — the policy must curl its legs in
  #                  and stand, exactly like the training task)

  # 3. Or watch it run training-style sampled episodes:
  ../../.venv/bin/mjpython -m rl_move.sim.view \
      --policy rl_move/sim/policies/ppo_goal.zip --mode rise

Plain ``python`` also works — on macOS the script re-execs itself under
the venv's mjpython (the GL UI cannot run on a non-main thread there).
Episodes loop forever in policy mode; close the window to quit.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# Single-threaded torch/OpenMP: the 128x128 MLP gains nothing from
# worker threads, and under mjpython every OpenMP spawn triggers macOS
# "Task policy set failed: 4" kernel spam (affinity calls are rejected).
os.environ.setdefault("OMP_NUM_THREADS", "1")

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
if str(_PROTO) not in sys.path:
    sys.path.insert(0, str(_PROTO))

MODES = ("hold", "lean", "track", "unload", "raise", "rise", "lower")


def _reexec_under_mjpython() -> None:
    """macOS: the viewer needs mjpython's main-thread event loop.

    Detection MUST use MJPYTHON_BIN (set by the mjpython launcher):
    sys.executable still reports the plain python path under mjpython,
    and checking it caused an infinite re-exec loop (a new process every
    ~200 ms, each logging one "Task policy set failed: 4" kernel line,
    and no window ever appearing).
    """
    if sys.platform != "darwin" or os.environ.get("MJPYTHON_BIN"):
        return
    mjpython = Path(sys.executable).with_name("mjpython")
    if not mjpython.is_file():
        sys.exit("mujoco viewer on macOS needs mjpython — run:\n"
                 f"  {Path(sys.executable).parent}/mjpython {__file__}")
    os.execv(str(mjpython), [str(mjpython), str(Path(__file__).resolve()),
                             *sys.argv[1:]])


def _pose_and_poke(args) -> None:
    import mujoco.viewer
    import numpy as np
    from rl_move.sim.sim_env import SimHexapodBalanceEnv

    plant = ([0.0] * 18 if args.pose == "zero" else None)
    env = SimHexapodBalanceEnv(randomize=False, plant_deg=plant,
                               mesh_visuals=args.meshes)
    env.reset()
    print("viewer: double-click a body, ctrl+drag to shove it; "
          "Control tab drives the servos. Close the window to quit.")
    # launch() runs its own physics loop; ctrl already holds the settled
    # pose, so the robot stands there waiting to be poked.
    mujoco.viewer.launch(env.model, env.data)


class _InteractiveTraj:
    """A 'trajectory' the keyboard mutates live; the env polls ``at()``
    every tick, so key presses take effect on the next control step.

    The PUBLISHED reference ramps toward the user's target at training-
    like rates (tilt ~4°/s, height ~12 mm/s — the rise ramp). The policy
    only ever saw ramped references; an instant +40 mm height step from
    the belly is out-of-distribution and the policy just sits there
    (measured). Ramping also gives the zero-pose rise its curl window,
    exactly like the training task's hold-then-ramp profile.
    """

    TILT_RATE = 4.0 * 3.141592653589793 / 180.0   # rad/s
    HEIGHT_RATE = 0.012                            # m/s
    # Belly starts: keep the height ref pinned at 0 this long, matching
    # the training rise profile (rise_hold_s) — the policy curls its legs
    # under itself during this window. Ramping immediately makes it fight
    # from the flat pose and trip the sustained over-current breaker.
    BELLY_HOLD_S = 5.0

    def __init__(self, dt: float = 0.04):
        import numpy as np
        from rl_move.env import TaskGoal
        self.mode = "interactive"
        self.start_at = "plant"
        self.crouch_dz = 0.0
        self.height = np.zeros(1)   # reset() peeks max() for milestones
        self.goal = TaskGoal()      # user TARGET (keyboard writes here)
        self._pub = TaskGoal()      # ramped ref the policy actually sees
        self._dt = dt
        self._last_step = 0

    def reset_published(self) -> None:
        from rl_move.env import TaskGoal
        self._pub = TaskGoal()
        self._last_step = 0

    @staticmethod
    def _toward(cur: float, tgt: float, step: float) -> float:
        if cur < tgt:
            return min(cur + step, tgt)
        return max(cur - step, tgt)

    def at(self, step):
        n = max(step - self._last_step, 0)
        self._last_step = step
        dt = n * self._dt
        self._pub.roll_ref = self._toward(
            self._pub.roll_ref, self.goal.roll_ref, self.TILT_RATE * dt)
        self._pub.pitch_ref = self._toward(
            self._pub.pitch_ref, self.goal.pitch_ref, self.TILT_RATE * dt)
        # Curl window: on belly starts an upward height ref is held back
        # for BELLY_HOLD_S of episode time (keyed to `step`, so it also
        # re-arms after every auto-reset).
        h_goal = self.goal.height_ref
        if (self.start_at in ("zero", "belly") and h_goal > 0.0
                and step * self._dt < self.BELLY_HOLD_S):
            h_goal = 0.0
        self._pub.height_ref = self._toward(
            self._pub.height_ref, h_goal, self.HEIGHT_RATE * dt)
        self._pub.unload_leg = self.goal.unload_leg  # one-hot: no ramp
        return self._pub


def _run_policy(args) -> None:
    import math
    import mujoco.viewer
    import torch
    torch.set_num_threads(1)
    from stable_baselines3 import PPO
    from rl_move.sim.goal_task import SimHexapodGoalEnv
    from rl_move.sim.joint_task import SimHexapodJointGoalEnv

    interactive = args.mode == "interactive"
    base_cls = (SimHexapodJointGoalEnv if args.task == "joint_goal"
                else SimHexapodGoalEnv)

    class InteractiveGoalEnv(base_cls):
        def __init__(self, *a, **kw):
            self.traj = _InteractiveTraj()
            super().__init__(*a, **kw)

        def _sample_goal(self):
            return self.traj

    env_cls = InteractiveGoalEnv if interactive else base_cls
    env = env_cls(randomize=args.dr_scale > 0,
                  dr_scale=args.dr_scale,
                  episode_seconds=(3600.0 if interactive
                                   else args.episode_seconds),
                  mesh_visuals=args.meshes)

    # --mode may be a comma list ("rise,lower"): episodes cycle through
    # the modes in order — e.g. watch it stand up, then descend, repeat.
    cycle = ([] if interactive or args.mode == "random"
             else args.mode.split(","))

    def set_mode(m: str) -> None:
        gen = env._goal_gen
        for attr in [a for a in vars(gen) if a.startswith("p_")]:
            setattr(gen, attr, 0.0)
        setattr(gen, f"p_{m}", 1.0)

    if cycle:
        set_mode(cycle[0])
    model = PPO.load(args.policy, device="cpu")
    obs, info = env.reset()

    DEG = math.pi / 180.0
    # viewer thread -> step loop: (start_pose, height_goal_after_reset)
    reset_req: list[tuple[str, float]] = []

    def show() -> None:
        g = env.traj.goal
        leg = "-" if g.unload_leg is None else f"L{g.unload_leg}"
        print(f"goal: roll {g.roll_ref / DEG:+.1f}°  "
              f"pitch {g.pitch_ref / DEG:+.1f}°  "
              f"height {g.height_ref * 1000:+.0f} mm  unload {leg}")

    def key_cb(keycode: int) -> None:
        # Letters are all claimed by MuJoCo's own vis/rnd toggles (D
        # hides the floor, S flips shadows, U draws actuators, ...), so
        # our bindings live on digits, - / = and the arrow keys only.
        if 320 <= keycode <= 329:          # GLFW numpad 0-9 -> digits
            keycode = ord("0") + keycode - 320
        g = env.traj.goal
        if keycode == 263:    g.roll_ref -= 0.5 * DEG      # left
        elif keycode == 262:  g.roll_ref += 0.5 * DEG      # right
        elif keycode == 265:  g.pitch_ref += 0.5 * DEG     # up
        elif keycode == 264:  g.pitch_ref -= 0.5 * DEG     # down
        elif keycode == ord("="):
            g.height_ref += 0.005
        elif keycode == ord("-"):
            g.height_ref -= 0.005
        elif ord("1") <= keycode <= ord("6"):
            leg = keycode - ord("1")
            g.unload_leg = None if g.unload_leg == leg else leg
        elif keycode == ord("0"):
            g.roll_ref = g.pitch_ref = g.height_ref = 0.0
            g.unload_leg = None
        elif keycode == ord("7"):
            # One-button stand: belly start, the curl window and height
            # ramp then play out automatically (~9 s total).
            reset_req.append(("zero", 0.045))
            return
        elif keycode == ord("8"):
            g.height_ref = 0.0      # gentle descent back down
            show()
            return
        elif keycode == ord("9"):
            reset_req.append(("plant", 0.0))
            return
        else:
            return
        g.roll_ref = min(max(g.roll_ref, -4 * DEG), 4 * DEG)
        g.pitch_ref = min(max(g.pitch_ref, -4 * DEG), 4 * DEG)
        g.height_ref = min(max(g.height_ref, -0.06), 0.06)
        show()

    if interactive:
        print(f"policy {Path(args.policy).name} | YOU set the goals "
              "(click the window first):\n"
              "  7 = STAND UP (belly start, fully automatic, ~9 s)\n"
              "  8 = go back down gently\n"
              "  9 = reset standing | 0 = clear goals\n"
              "  arrows = lean, = / - = height +/-5mm, 1-6 = unload leg.\n"
              "  (letters are MuJoCo's own display toggles - avoid them)\n"
              "  Ctrl+drag shoves it. Close the window to quit.")
    else:
        print(f"policy {Path(args.policy).name} | goal={info['goal_mode']}"
              " | shove the robot: double-click a body, ctrl+drag. "
              "Close the window to quit.")

    hud_status = [""]  # last event line, shown live in the window

    _KEYS_HELP = ("7 stand up (auto ~9s)\n"
                  "8 sit back down\n"
                  "9 reset standing\n"
                  "0 clear goals\n"
                  "arrows lean\n"
                  "= / -  height +/-5mm\n"
                  "1..6 unload leg\n"
                  "Ctrl+drag = shove")

    def update_hud(viewer) -> None:
        # In-window overlay: terminal output is invisible when the viewer
        # has focus, so everything important is painted here every step.
        set_texts = getattr(viewer, "set_texts", None)
        if set_texts is None:      # older mujoco: no overlay API
            return
        import mujoco as _mj
        texts = []
        if interactive:
            g = env.traj.goal
            leg = "-" if g.unload_leg is None else f"L{g.unload_leg}"
            goal_line = (f"goal  roll {g.roll_ref / DEG:+.1f}deg  "
                         f"pitch {g.pitch_ref / DEG:+.1f}deg  "
                         f"height {g.height_ref * 1000:+.0f}mm  "
                         f"unload {leg}")
            texts.append((_mj.mjtFontScale.mjFONTSCALE_150,
                          _mj.mjtGridPos.mjGRID_BOTTOMLEFT,
                          "KEYS (click window first)", _KEYS_HELP))
            texts.append((_mj.mjtFontScale.mjFONTSCALE_150,
                          _mj.mjtGridPos.mjGRID_TOPRIGHT,
                          Path(args.policy).name,
                          goal_line + ("\n" + hud_status[0]
                                       if hud_status[0] else "")))
        else:
            texts.append((_mj.mjtFontScale.mjFONTSCALE_150,
                          _mj.mjtGridPos.mjGRID_TOPRIGHT,
                          Path(args.policy).name, hud_status[0]))
        set_texts(texts)

    with mujoco.viewer.launch_passive(
            env.model, env.data,
            key_callback=key_cb if interactive else None) as viewer:
        t_next = time.monotonic()
        ret = 0.0
        while viewer.is_running():
            update_hud(viewer)
            if reset_req:
                start, h_goal = reset_req.pop()
                env.traj.start_at = start
                env.traj.goal.roll_ref = 0.0
                env.traj.goal.pitch_ref = 0.0
                env.traj.goal.height_ref = h_goal
                env.traj.goal.unload_leg = None
                env.traj.reset_published()
                obs, info = env.reset()
                ret = 0.0
                if h_goal > 0:
                    hud_status[0] = (f"STAND: curl ~5s, then rise to "
                                     f"+{h_goal * 1000:.0f}mm - hands off")
                else:
                    hud_status[0] = f"reset at {start} pose - goals cleared"
                print(hud_status[0])
                viewer.sync()
                t_next = time.monotonic()
                continue
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, inf = env.step(action)
            ret += float(r)
            viewer.sync()
            if term or trunc:
                end = (f"TERMINATED: {inf.get('termination_reason')}"
                       if term else "survived")
                hud_status[0] = (f"episode done: goal={inf.get('goal_mode')} "
                                 f"return={ret:+.1f} | {end}")
                print(hud_status[0])
                if interactive:
                    env.traj.reset_published()  # refs re-ramp from zero
                if cycle:
                    cycle.append(cycle.pop(0))
                    set_mode(cycle[0])
                obs, info = env.reset()
                ret = 0.0
                if not interactive:
                    print(f"new episode: goal={info['goal_mode']}")
                else:
                    print("auto-reset — goals kept")
                    show()
                viewer.sync()
                t_next = time.monotonic()
            t_next += env.dt
            time.sleep(max(0.0, t_next - time.monotonic()))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", type=Path, default=None,
                    help="run this trained policy live (default: just "
                         "pose & poke the passive model)")
    ap.add_argument("--task", choices=("goal", "joint_goal"),
                    default="goal",
                    help="env the policy was trained on: 'goal' = 6-dim "
                         "body-IK actions, 'joint_goal' = raw 18-joint "
                         "actions (e.g. the cw-* CoreWeave runs)")
    ap.add_argument("--mode", default="interactive",
                    help="goal mode for --policy episodes: 'interactive' "
                         "(default) = you set goals with the keyboard, "
                         "'random' or a task name = training-style "
                         "sampled episodes; a comma list like "
                         "'rise,lower' cycles modes episode by episode")
    ap.add_argument("--pose", choices=("plant", "zero"), default="plant",
                    help="start pose for pose-&-poke mode")
    ap.add_argument("--dr-scale", type=float, default=0.0,
                    help="domain randomization scale for --policy mode "
                         "(0 = calibrated nominal sim)")
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--meshes", action="store_true",
                    help="render STL meshes instead of the collision "
                         "primitives. Default is primitives — they ARE "
                         "what physics simulates; the June 2026 STL "
                         "re-export has stale visual offsets (feet render "
                         "detached), so meshes look broken even though "
                         "the physics is fine")
    args = ap.parse_args()
    valid = set(MODES) | {"random", "interactive"}
    bad = [m for m in args.mode.split(",") if m not in valid]
    if bad:
        ap.error(f"unknown mode(s) {bad}; valid: {sorted(valid)}")

    _reexec_under_mjpython()
    if args.policy:
        _run_policy(args)
    else:
        _pose_and_poke(args)


if __name__ == "__main__":
    main()
