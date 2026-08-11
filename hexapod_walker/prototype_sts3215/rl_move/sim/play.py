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

Gamepad (recommended; hot-plugs any time — just turn the pad on):
    Left stick  analog walk: forward/back + strafe (engages WALK)
    A           stand up IN PLACE (belly: auto rise ~11 s; crouch: rises)
    B           sit down
    Y           reset to standing        X   reset belly-down
    LB / RB     body height -/+ 5 mm (stance)
    D-pad U/D   cycle STANCE model       D-pad L/R  cycle WALK model

Input is read with pygame/SDL (`SDL_VIDEODRIVER=dummy`, no window):
SDL's game-controller database normalizes Xbox/PS/Switch pads, and a
gamepad can never collide with key bindings — which is also why this
player renders into an OpenCV window instead of mujoco.viewer (cv2 has
NO built-in bindings; in mujoco.viewer nearly every letter is a toggle).

Model selection: every non-`*_steps` checkpoint in the policies dir is
classified at startup by observation width read straight from the sb3
zip's JSON metadata (no torch load): obs 68 -> STANCE slot, obs 72 ->
WALK slot, anything else is not playable here. A clickable panel on the
right of the window (like the robot webui's policy picker) lists both
groups — click a row to load it (~1 s stall); `[ ]` / `, .` and the
pad d-pad cycle the same lists. The current pair is highlighted.

Keys (window must have focus; all still work without a pad — the cv2
window owns every key, so no modifier is needed to avoid collisions):
    Arrows      HOLD to drive (fwd/back/strafe at 0.05 m/s), release to
                stop. Diagonals work (hold two arrows). Detected via OS
                key auto-repeat: no repeat for ~0.65 s = released.
    7           stand up IN PLACE - no reset/teleport. From a crouch it
                just rises; from the belly it re-anchors the episode
                where the robot is and runs the auto rise (~11 s).
    8           sit down (crouch; from a belly episode: back to floor)
    9 / R       reset to standing (plant, at the origin - a true reset)

On a tip (episode termination) there is NO auto-reset: the robot
freezes where it fell and waits — 7 tries an in-place recovery,
9 does a true reset. Same stop-and-wait rule as the real robot.
    B           reset belly-down (then 7 to stand)
    I/K J/L     persistent cruise trim (+-0.01 m/s per tap, engages WALK)
    0 / Space   stop -> STANCE policy holds
    = / -       body height +/- 5 mm (stance)
    [ / ]       cycle STANCE model      , / .  cycle WALK model
    Q / Esc     quit
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import zipfile
from pathlib import Path

import numpy as np

from rl_move.env import TaskGoal
from .view import _InteractiveTraj
from .walk_task import SimHexapodJointWalkEnv, WalkGoal

_STEP = 0.01          # m/s per keypress
_SPEED_MAX = 0.06     # champion's trained command band tops out here
_CRUISE = 0.05        # hold-to-drive speed (inside the trained band)
# cv2 can't see key-up events, but macOS auto-repeats a held arrow key.
# "No repeat for _HOLD_S" therefore means "released" — a dead-man switch.
# Must exceed the OS initial-repeat delay (default ~0.5 s).
_HOLD_S = 0.65
_UP, _DOWN, _LEFT, _RIGHT = 63232, 63233, 63234, 63235   # macOS cv2 arrows

# Playable obs widths (see module docstring / sim_viewer/README.md).
_ROLE_OBS = {68: "stance", 72: "walk"}

# Checkpoints currently deployed on the physical robot (source: the
# `meta` blocks of linux_control/rl_policy_weights.json and
# rl_walk_weights.json). Sorted to the top of each panel list.
_ON_ROBOT = {"ppo_goal_cw_stand_holdbc1_hard1", "ppo_goal_cw_dep_vref1_r1"}

# Panel order: robot-deployed first, then the checkpoints most worth
# trying (champions / hardened driving recipes per SKILLS.md), then the
# rest alphabetically.
_PROMOTED = [
    # stance group
    "ppo_goal_cw_stand_holdbc1_hard1",
    "ppo_goal_cw_stance_dr10",
    "ppo_goal_cw_stand_crouchrise1",
    "ppo_goal_cw_stance_raisefix",
    # walk group
    "ppo_goal_cw_dep_vref1_r1",
    "ppo_goal_cw_walk_joyheadfric",
    "ppo_goal_cw_walk_joyheadfric_payload_r1",
    "ppo_goal_cw_walk_longdist_r2",
    "ppo_goal_cw_walk_wander30",
    "ppo_goal_cw_walk_slow2",
    "ppo_goal_cw_walk_anchorgate",
]

# One-liners distilled from RL_LOG.md / rl_docs / archive (2026-08-11).
_DESC = {
    # --- stance group (obs 68) ---------------------------------------
    "ppo_goal_cw_friction": "rise 5/5 w/ friction DR 0.3-1.6x; sim2real proof",
    "ppo_goal_cw_long5m": "rise 5/5 @DR0.4; raise stuck ~1/2 at 5M",
    "ppo_goal_cw_lower": "stand-flat round trip solved; early flagship",
    "ppo_goal_cw_stance_clear": "clearance pen broke tripod; raise 0/6 FAIL",
    "ppo_goal_cw_stance_dr08": "even-stance line DR 0.8 PASS, raise 5/6",
    "ppo_goal_cw_stance_dr10": "STANCE CHAMPION, solved at DR 1.0",
    "ppo_goal_cw_stance_even": "hot-current fix partial; tripod unchanged",
    "ppo_goal_cw_stance_raisefix": "raise exempt from clearance; gate PASS",
    "ppo_goal_cw_stand_dr05": "plain stand line, DR 0.5 rung PASS 6/6",
    "ppo_goal_cw_stand_dr08": "plain stand line, DR 0.8 rung PASS",
    "ppo_goal_cw_stand_dr10": "plain stand DR 1.0 PASS; was hw candidate",
    "ppo_goal_cw_stand_crouchrise1":
        "crouch-stand FIX 16/16 (vs 0/8) but hold broke",
    "ppo_goal_cw_stand_holdbc1_hard1":
        "ON ROBOT (stance): stand/sit/hold 10M run",
    "ppo_joint_goal": "local default-name ckpt; 512-step smoke overwrote",
    "ppo_joint_goal_bc": "BC-from-IK-teacher variant (no log entry)",
    "ppo_joint_goal_bc2m": "BC warm 2M net negative: rise 3/4 + overcurrent",
    "ppo_joint_goal_scratch2m": "scratch 2M, belly-rise 5/5; raw joints proof",
    # --- walk group (obs 72) -----------------------------------------
    "ppo_goal_cw_dep_quad1_c2": "quad-hold +12M ext PASS; height err 2.9mm",
    "ppo_goal_cw_dep_tip1": "tipped-start DR FAIL; 0 recovery, gait hurt",
    "ppo_goal_cw_dep_vref1_r1":
        "ON ROBOT (walk): meas:=ref contract, 0.05-0.06",
    "ppo_goal_cw_walk2_gait": "swing bonus: stride 2x, tracking still 0/6",
    "ppo_goal_cw_walk_anchorgate": "ex walk champ; income gate, less slip",
    "ppo_goal_cw_walk_curr08": "widen 0.02-0.08 unconsolidated; missed gate",
    "ppo_goal_cw_walk_dr04": "DR 0.2->0.4 near-miss, 3/6 stop @0.031",
    "ppo_goal_cw_walk_fresh_gait": "fresh-init ablation; same skate, refuted",
    "ppo_goal_cw_walk_longdist_r2":
        "SIM WALK CHAMPION; paddle-slide, not hw-ready",
    "ppo_goal_cw_walk_prog3": "3x progress reward refuted; motion, 0 track",
    "ppo_goal_cw_walk_slow": "slow band 0.02-0.06; first tracking gain",
    "ppo_goal_cw_walk_slow2": "slow-band consolidation; gate 5/6 @0.028",
    "ppo_goal_cw_walk_w08": "widen to 0.08 regressed 1/6; rise eroded",
    "ppo_goal_cw_walk_w08_s1": "INVALID seed twin; bit-identical, seed bug",
    "ppo_goal_cw_walk_wander30": "30s drive endurance PASS; speedband base",
    "ppo_mjx_joint_walk": "MJX trainer default-name artifact (no log)",
    "ppo_goal_cw_walk_joyheadfric":
        "widest driving env: +-90deg steer, fric, 3-seed",
    "ppo_goal_cw_walk_joyheadfric_payload_r1":
        "joyheadfric + payload 1.0-1.4x; gate PASS",
}


def _obs_width(path: Path) -> int | None:
    """Obs width of an sb3 checkpoint WITHOUT loading it (no torch).

    sb3 zips carry a JSON ``data`` member whose observation_space entry
    includes a plain ``_shape`` list next to the pickled payload.
    """
    try:
        with zipfile.ZipFile(path) as z:
            data = json.loads(z.read("data"))
        shape = data["observation_space"]["_shape"]
        return int(shape[0]) if shape else None
    except Exception:
        return None


def scan_policies(pdir: Path) -> dict[str, list[Path]]:
    """Classify checkpoints in ``pdir`` into stance (68) / walk (72) lists.

    ``*_steps.zip`` autosaves are skipped — there are hundreds and the
    named finals are the ones worth cycling through.
    """
    out: dict[str, list[Path]] = {"stance": [], "walk": []}
    for p in sorted(pdir.glob("*.zip")):
        if p.stem.endswith("_steps"):
            continue
        role = _ROLE_OBS.get(_obs_width(p) or -1)
        if role:
            out[role].append(p.resolve())
    def rank(p: Path):
        stem = p.stem
        return (_PROMOTED.index(stem) if stem in _PROMOTED
                else len(_PROMOTED), stem)

    for lst in out.values():
        lst.sort(key=rank)
    return out


class _Gamepad:
    """Xbox-style pad via pygame/SDL, headless (no pygame window).

    SDL's controller database normalizes most pads: axes 0/1 = left
    stick, buttons 0..3 = A/B/X/Y (south/east/west/north), d-pad = hat 0
    (a few pads report it as buttons 11..14 — both are handled).
    Hot-plug works: turn the pad on whenever, it attaches on the fly.
    """

    DEADZONE = 0.15
    _BTN = {0: "A", 1: "B", 2: "X", 3: "Y", 4: "LB", 5: "RB",
            11: "UP", 12: "DOWN", 13: "LEFT", 14: "RIGHT"}

    def __init__(self):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        import pygame
        self._pg = pygame
        pygame.init()
        pygame.joystick.init()
        self._js = None
        self._down: dict[int, bool] = {}
        self._hat = (0, 0)
        self._attach()

    def _attach(self) -> None:
        if self._pg.joystick.get_count() > 0:
            self._js = self._pg.joystick.Joystick(0)
            self._down = {}
            self._hat = (0, 0)

    @property
    def name(self) -> str | None:
        return self._js.get_name() if self._js else None

    def _axis(self, i: int) -> float:
        v = float(self._js.get_axis(i)) if i < self._js.get_numaxes() else 0.0
        if abs(v) < self.DEADZONE:
            return 0.0
        s = (abs(v) - self.DEADZONE) / (1.0 - self.DEADZONE)
        return math.copysign(min(s, 1.0), v)

    def poll(self):
        """-> (lx, ly, pressed_edges: set[str]) or None if no pad."""
        pg = self._pg
        for ev in pg.event.get():
            if ev.type == pg.JOYDEVICEADDED and self._js is None:
                self._attach()
            elif (ev.type == pg.JOYDEVICEREMOVED and self._js is not None
                    and ev.instance_id == self._js.get_instance_id()):
                self._js = None
        if self._js is None:
            return None
        js = self._js
        pressed: set[str] = set()
        for idx, name in self._BTN.items():
            if idx >= js.get_numbuttons():
                continue
            down = bool(js.get_button(idx))
            if down and not self._down.get(idx):
                pressed.add(name)
            self._down[idx] = down
        if js.get_numhats() > 0:
            hx, hy = js.get_hat(0)
            phx, phy = self._hat
            if hy > 0 >= phy:
                pressed.add("UP")
            if hy < 0 <= phy:
                pressed.add("DOWN")
            if hx < 0 <= phx:
                pressed.add("LEFT")
            if hx > 0 >= phx:
                pressed.add("RIGHT")
            self._hat = (hx, hy)
        return self._axis(0), self._axis(1), pressed


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

    # --- checkpoint slots: one STANCE (obs 68) + one WALK (obs 72) ------
    cats = scan_policies(args.stance.parent)
    stance_list, walk_list = cats["stance"], cats["walk"]

    def ensure_listed(lst: list[Path], p: Path, want: int) -> int:
        w = _obs_width(p)
        if w != want:
            raise SystemExit(f"{p}: obs width {w}, need {want}")
        p = p.resolve()
        if p not in lst:
            lst.insert(0, p)
        return lst.index(p)

    si = ensure_listed(stance_list, args.stance, 68)
    wi = ensure_listed(walk_list, args.walk, 72)

    stance = PPO.load(stance_list[si], device="cpu")
    walk = PPO.load(walk_list[wi], device="cpu")
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

    def set_stance(i: int) -> None:
        nonlocal stance, si, n_stance, msg
        si = i % len(stance_list)
        m = PPO.load(stance_list[si], device="cpu")
        if m.action_space.shape != env.action_space.shape:
            msg = f"{stance_list[si].stem}: action space mismatch - skipped"
            return
        stance = m
        n_stance = int(m.observation_space.shape[0])
        msg = f"stance model -> {stance_list[si].stem}"

    def set_walk(i: int) -> None:
        nonlocal walk, wi, msg
        wi = i % len(walk_list)
        m = PPO.load(walk_list[wi], device="cpu")
        if m.action_space.shape != env.action_space.shape:
            msg = f"{walk_list[wi].stem}: action space mismatch - skipped"
            return
        walk = m
        msg = f"walk model -> {walk_list[wi].stem}"

    try:
        pad = _Gamepad()
        pad_err = ""
    except Exception as e:                      # pygame missing/broken
        pad, pad_err = None, f"gamepad disabled ({e})"
    stick_live = False
    held: dict[int, float] = {}      # arrow keycode -> last press/repeat time
    arrows_live = False
    downed = False   # episode terminated (tip etc.): freeze, wait for user

    def chassis_z() -> float:
        return float(env.data.xpos[chassis_bid, 2])

    def q_now() -> np.ndarray:
        return env.data.qpos[7:25].copy()

    def do_reset(start: str, h_goal: float, note: str) -> None:
        nonlocal obs, msg, auto, downed
        auto = None
        downed = False
        held.clear()
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

    def do_stand() -> None:
        # Realistic stand: never teleport. Crouched in a plant frame ->
        # just raise the height ref. Otherwise re-anchor a belly episode
        # IN PLACE (restore full qpos/qvel after the bookkeeping reset,
        # same trick as re_anchor_plant) and run the auto rise sequence.
        nonlocal obs, msg, auto, downed
        if auto is not None:
            return
        if not downed and traj.start_at == "plant" and chassis_z() > 0.09:
            traj.goal.height_ref = 0.0
            msg = "standing back up (in place)"
            return
        keep_q = env.data.qpos.copy()
        keep_v = env.data.qvel.copy()
        downed = False
        held.clear()
        traj.start_at = "zero"
        traj.goal = TaskGoal()
        traj.goal.height_ref = 0.045
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        obs, _ = env.reset()
        env.data.qpos[:] = keep_q
        env.data.qvel[:] = keep_v
        mujoco.mj_forward(env.model, env.data)
        auto = ["rise", 0]
        msg = "STAND (in place): curl ~5s, rise, then blend - hands off"

    # --- clickable model panel (like the robot webui's policy picker) ---
    # Fixed-size canvas + WINDOW_AUTOSIZE so mouse coords map 1:1 to
    # pixels; sim view is upscaled to VIEW_W x VIEW_H, panel sits right.
    VIEW_W, VIEW_H, PANEL_W = 960, 720, 540
    n_rows = len(stance_list) + len(walk_list) + 2
    ROW_H = int(min(20, max(14, (VIEW_H - 60) / n_rows)))
    layout: list[tuple[int, str, int]] = []      # (y_top, role, index)

    def _lay_rows(role: str, lst: list[Path], y: int) -> int:
        layout.append((y, "hdr:" + role, -1))
        y += ROW_H + 4
        for i in range(len(lst)):
            layout.append((y, role, i))
            y += ROW_H
        return y + 14

    _lay_rows("walk", walk_list, _lay_rows("stance", stance_list, 8))

    mouse = {"hover": (-1, -1), "clicks": []}

    def on_mouse(ev, mx, my, _flags, _param) -> None:
        if ev == cv2.EVENT_LBUTTONDOWN:
            mouse["clicks"].append((mx, my))
        elif ev == cv2.EVENT_MOUSEMOVE:
            mouse["hover"] = (mx, my)

    def row_at(mx: int, my: int):
        if mx < VIEW_W + 4:
            return None
        for y, role, i in layout:
            if not role.startswith("hdr") and y <= my < y + ROW_H:
                return role, i
        return None

    def draw_panel() -> np.ndarray:
        panel = np.full((VIEW_H, PANEL_W, 3), 26, np.uint8)
        hov = row_at(*mouse["hover"])
        for y, role, i in layout:
            if role.startswith("hdr"):
                txt = ("STANCE models (obs 68)" if role.endswith("stance")
                       else "WALK models (obs 72)")
                cv2.putText(panel, txt + "  - click to load, * = on robot",
                            (10, y + ROW_H - 6), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (150, 150, 150), 1, cv2.LINE_AA)
                continue
            lst, cur = ((stance_list, si) if role == "stance"
                        else (walk_list, wi))
            stem = lst[i].stem
            sel = i == cur
            if hov == (role, i):
                panel[y:y + ROW_H] = 48
            if sel:
                panel[y:y + ROW_H] = ((48, 42, 20) if role == "stance"
                                      else (20, 48, 20))
            color = (((240, 200, 40) if role == "stance" else (40, 240, 40))
                     if sel else (205, 205, 205))
            mark = (">" if sel else " ") + ("*" if stem in _ON_ROBOT
                                            else " ")
            disp = stem.removeprefix("ppo_goal_").removeprefix("ppo_")
            cv2.putText(panel, f"{mark} {disp}", (8, y + ROW_H - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1,
                        cv2.LINE_AA)
            cv2.putText(panel, _DESC.get(stem, ""), (208, y + ROW_H - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                        (140, 190, 190) if stem in _ON_ROBOT
                        else (135, 135, 135), 1, cv2.LINE_AA)
        return panel

    win = "hexapod play"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        t0 = time.monotonic()
        cmd_speed = float(np.hypot(traj.vx, traj.vy))
        walking = cmd_speed > 1e-3 and auto is None and not downed
        if downed:
            # No auto-reset: freeze the joints where they are and wait
            # for the operator (7 = try to stand in place, 9 = reset) —
            # same "stop and wait" rule as the real robot.
            action = q_rad_to_action(q_now())
        elif auto is not None and auto[0] == "rise":
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
        if (term or trunc) and not downed:
            downed = True
            auto = None
            held.clear()
            traj.vx = traj.vy = 0.0
            msg = (f"[{info.get('termination_reason') or 'episode end'}] "
                   "DOWN - 7 stand in place, 9 reset standing")

        # --- HUD ---------------------------------------------------------
        frame = env.render()
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (VIEW_W, VIEW_H),
                         interpolation=cv2.INTER_LINEAR)
        v = env._body_vel_xy()
        g = traj.goal
        if downed:
            mode_txt = "DOWN (episode terminated) - 7 stand, 9 reset"
            mode_col = (60, 60, 255)
        elif auto is not None:
            mode_txt = ("STANDING UP (auto): "
                        + ("curl + rise..." if auto[0] == "rise"
                           else "blend to walk stance..."))
            mode_col = (0, 200, 255)
        elif walking:
            mode_txt, mode_col = "WALK policy", (40, 240, 40)
        else:
            mode_txt, mode_col = "STANCE policy", (240, 200, 40)
        pad_name = pad.name if pad else None
        pad_txt = (f"pad: {pad_name}" if pad_name
                   else pad_err or "no gamepad - turn it on, it hot-plugs")
        lines = [
            (f"{mode_txt}   height {chassis_z() * 1000:.0f}mm", mode_col),
            (f"CMD vx {traj.vx:+.3f} vy {traj.vy:+.3f} m/s   "
             f"ACTUAL vx {v[0]:+.3f} vy {v[1]:+.3f}", (200, 200, 40)),
            (f"goal height {g.height_ref * 1000:+.0f}mm  "
             f"roll {math.degrees(g.roll_ref):+.1f}  "
             f"pitch {math.degrees(g.pitch_ref):+.1f}", (200, 200, 200)),
            (f"stance[{si + 1}/{len(stance_list)}] {stance_list[si].stem}   "
             f"walk[{wi + 1}/{len(walk_list)}] {walk_list[wi].stem}",
             (230, 160, 230)),
            (pad_txt + "   L-stick walk  A stand  B sit  Y reset  X belly",
             (120, 220, 220) if pad_name else (140, 140, 140)),
            ("pad: dpad U/D stance model  L/R walk model  LB/RB height",
             (120, 220, 220) if pad_name else (140, 140, 140)),
            ("keys: HOLD arrows to drive (release = stop)   "
             "7 stand  8 sit  9 reset  B belly", (180, 180, 180)),
            ("keys: I/K/J/L cruise trim  0/space stop  =/- height  "
             "[ ] stance model  , . walk model  Q quit", (180, 180, 180)),
        ]
        if msg:
            lines.append((msg, (0, 200, 255)))
        for i, (line, color) in enumerate(lines):
            cv2.putText(img, line, (10, 24 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1,
                        cv2.LINE_AA)
        cv2.imshow(win, np.hstack([img, draw_panel()]))

        # --- keys (all ours: cv2 has no built-in bindings) ------------------
        k = cv2.waitKeyEx(1)

        # --- panel clicks: load the model that was hit ---------------------
        while mouse["clicks"]:
            hit = row_at(*mouse["clicks"].pop())
            if hit is not None:
                role, i = hit
                (set_stance if role == "stance" else set_walk)(i)

        def engage_walk() -> bool:
            # The walk champion lives around the ~142 mm plant stance;
            # walking from the floor or the 72 mm crouch-stand collapses
            # into a belly shuffle (measured) — make the user stand first.
            nonlocal msg
            if auto is not None:
                return False
            if downed:
                msg = "robot is down - 7 to stand or 9 to reset first"
                return False
            if chassis_z() < 0.09:
                msg = "too low to walk - press 7 (pad: A) to stand up first"
                return False
            # Walk champion trained at height/tilt refs = 0: snap the
            # published stance refs to nominal so its obs is in-distribution.
            traj.goal.roll_ref = traj.goal.pitch_ref = 0.0
            traj.goal.height_ref = 0.0
            traj._pub.roll_ref = traj._pub.pitch_ref = 0.0
            traj._pub.height_ref = 0.0
            if msg.startswith("too low to walk"):
                msg = ""
            return True

        # --- gamepad (edge-triggered buttons, analog left stick) ----------
        state = pad.poll() if pad else None
        if state is not None:
            lx, ly, pressed = state
            if lx != 0.0 or ly != 0.0:
                # Stick up = forward (+vx); stick left = strafe left (+vy).
                if engage_walk():
                    traj.vx = -ly * _SPEED_MAX
                    traj.vy = -lx * _SPEED_MAX
                stick_live = True
            elif stick_live:
                traj.vx = traj.vy = 0.0     # released -> stance holds
                stick_live = False
            if "A" in pressed:
                do_stand()
            elif "B" in pressed:
                traj.vx = traj.vy = 0.0
                traj.goal.height_ref = (0.0 if traj.start_at
                                        in ("zero", "belly") else -0.06)
                msg = "sitting back down"
            elif "Y" in pressed:
                do_reset("plant", 0.0, "reset standing")
            elif "X" in pressed:
                do_reset("zero", 0.0, "reset belly-down (A to stand)")
            if "RB" in pressed:
                traj.goal.height_ref = min(traj.goal.height_ref + 0.005,
                                           0.06)
            elif "LB" in pressed:
                traj.goal.height_ref = max(traj.goal.height_ref - 0.005,
                                           -0.06)
            if "UP" in pressed:
                set_stance(si + 1)
            elif "DOWN" in pressed:
                set_stance(si - 1)
            if "RIGHT" in pressed:
                set_walk(wi + 1)
            elif "LEFT" in pressed:
                set_walk(wi - 1)

        if k in (_UP, _DOWN, _LEFT, _RIGHT):
            # Hold-to-drive: keep the direction alive while repeats arrive.
            if engage_walk():
                held[k] = time.monotonic()
        elif k in (ord("i"), ord("I")):
            if engage_walk():
                traj.vx = float(np.clip(traj.vx + _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("k"), ord("K")):
            if engage_walk():
                traj.vx = float(np.clip(traj.vx - _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("j"), ord("J")):
            if engage_walk():
                traj.vy = float(np.clip(traj.vy + _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("l"), ord("L")):
            if engage_walk():
                traj.vy = float(np.clip(traj.vy - _STEP,
                                        -_SPEED_MAX, _SPEED_MAX))
        elif k in (ord("0"), ord(" ")):
            held.clear()
            traj.vx = traj.vy = 0.0
            msg = "stopped - stance policy holding"
        elif k == ord("7"):
            do_stand()
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
        elif k == ord("]"):
            set_stance(si + 1)
        elif k == ord("["):
            set_stance(si - 1)
        elif k == ord("."):
            set_walk(wi + 1)
        elif k == ord(","):
            set_walk(wi - 1)
        elif k in (ord("q"), ord("Q"), 27):
            break

        # --- hold-to-drive: evaluate which arrows are still held ---------
        if held:
            now = time.monotonic()
            for kk in [kk for kk, t in held.items() if now - t >= _HOLD_S]:
                del held[kk]
        if held:
            traj.vx = _CRUISE * ((_UP in held) - (_DOWN in held))
            traj.vy = _CRUISE * ((_LEFT in held) - (_RIGHT in held))
            arrows_live = True
        elif arrows_live:
            traj.vx = traj.vy = 0.0     # all arrows released -> stance holds
            arrows_live = False

        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break

        if args.realtime > 0:
            dt = env.dt / args.realtime - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
