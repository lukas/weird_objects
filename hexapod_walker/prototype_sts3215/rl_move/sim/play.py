"""BOTH champions in one sim: rise, walk around, lower to the ground.

    ./sim_play.sh          # from prototype_sts3215/  (plain python, NO mjpython)

One physics env (the walk env) + two policies:

- STANCE policy (ppo_goal_cw_stance_dr10, obs 68) is active while no
  velocity is commanded: rise / hold / lower / lean / height.
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
    A           rise IN PLACE (belly: auto rise ~11 s; crouch: rises)
    B           lower to the ground
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

--phase-obs: enables the walk env's phase clock (goal.walk_phase_obs=1,
sin/cos appended at the obs tail -> obs 74) so the PHASE-CLOCK no-slip
RL checkpoints (obs 74; cw-arch-noslipphase1 line, e.g.
ppo_goal_cw_arch_noslipphase1_r4) become playable in the WALK slot.
--phase-hz sets the clock rate (default 1/6 Hz = one revolution per
6 s clamp-fit gait cycle, what that line was trained on). Legacy 72-obs
walk champions still work in this mode — the phase dims sit AFTER the
velocity tail, so they are fed obs[:72] (their exact layout); the
stance policy always reads obs[:68].

The WALK list additionally ends with two non-checkpoint rows, flagged
`S`: the SCRIPTED no-slip gait (linux_control/noslip_gait.py —
rules-based, no RL) at alpha=0 (the original step-then-shift) and at
alpha=0.5 (the middle of the overlap continuum: half the body travel
rides a constant drift through swings and dwells instead of the shift
pulses). Select one and the usual drive inputs run that gait instead
of a policy; stopping still hands back to the stance policy. Both are
slower than the RL band (clamped 0.04/0.035 m/s; ~0.01 m/s realized
here, since this env keeps the default clamped servo cruise rather
than verify_noslip's unclamped hardware profile) but planted feet are
commanded to fixed world anchors at every alpha (zero commanded scrub,
see verify_noslip --alpha), and unlike the walk champions it can TURN:
U/O trim a yaw rate +-0.05 rad/s per tap, including turn-in-place.

Keys (window must have focus; all still work without a pad — the cv2
window owns every key, so no modifier is needed to avoid collisions):
    Arrows      HOLD to drive (fwd/back/strafe at 0.05 m/s), release to
                stop. Diagonals work (hold two arrows). Detected via OS
                key auto-repeat: no repeat for ~0.65 s = released.
    7           rise IN PLACE - no reset/teleport. From a crouch it
                just rises; from the belly it re-anchors the episode
                where the robot is and runs the auto rise (~11 s).
    8           lower FOR REAL: trained lower to the crouch, then a
                torque-off limp settle onto the belly (the robot's own
                lower-then-limp choreography). Stays parked until 7.
    9 / R       reset to standing (plant, at the origin - a true reset)

On a tip (episode termination) there is NO auto-reset: the robot
freezes where it fell and waits — 7 tries an in-place recovery,
9 does a true reset. Same stop-and-wait rule as the real robot.
    B           reset belly-down (then 7 to rise)
    I/K J/L     persistent cruise trim (+-0.01 m/s per tap, engages WALK)
    U / O       turn left/right trim (scripted no-slip gait only)
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
import sys
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
_STEP_W = 0.05        # rad/s per keypress (scripted gait turn)

# Sentinel rows in the WALK panel: not checkpoints — they select the
# scripted no-slip gait (linux_control/noslip_gait.py) as the walk
# driver, at different alpha (body-motion overlap): 0.0 = the original
# step-then-shift, 0.5 = the midpoint of the continuum (half the body
# travel rides a constant drift through swings/dwells). Slower than the
# RL band (~0.02 m/s realized here) but stance feet are commanded to
# fixed world anchors at every alpha, and it turns (U/O). The clampfit
# row is NoSlipGait.CLAMP_FIT_KW — the 08-12 sweep's cleanest timing
# under this env's fitted ~31 deg/s servo clamp (alpha=1, swing-heavy
# 6 s cycle; zero true scrub, 4x less loaded foot drift).
_NOSLIP = Path("noslip_scripted_gait")
_NOSLIP_MID = Path("noslip_hybrid_a50")
_NOSLIP_CLEAN = Path("noslip_clampfit_gait")
_SCRIPTED_ALPHA = {_NOSLIP: 0.0, _NOSLIP_MID: 0.5, _NOSLIP_CLEAN: 1.0}
# cv2 can't see key-up events, but macOS auto-repeats a held arrow key.
# "No repeat for _HOLD_S" therefore means "released" — a dead-man switch.
# Must exceed the OS initial-repeat delay (default ~0.5 s).
_HOLD_S = 0.65
_UP, _DOWN, _LEFT, _RIGHT = 63232, 63233, 63234, 63235   # macOS cv2 arrows

# Playable obs widths (see module docstring / sim_viewer/README.md).
_ROLE_OBS = {68: "stance", 72: "walk"}

# Checkpoints currently deployed on the physical robot — live slot
# files AND the selectable linux_control/policies/ picker entries
# (source: GET /api/rl/policies). Sorted to the top of each panel list.
_ON_ROBOT = {
    "ppo_goal_cw_stand_holdbc1_hard1",     # live stance slot
    "ppo_goal_cw_stand_footlow2_hard1",    # stand/lower roles
    "ppo_goal_cw_stance_dr10",             # picker fallback
    "ppo_goal_cw_dep_vref1_r1",            # live walk slot
    "ppo_goal_cw_dep_tip1",                # picker
    "ppo_goal_cw_dep_quad1_c2",            # picker
    "ppo_goal_cw_arch_noslipphase1_r4",    # picker (obs 74, 08-13)
}

# Panel order: robot-deployed first, then the checkpoints most worth
# trying (champions / hardened driving recipes per SKILLS.md), then the
# rest alphabetically.
_PROMOTED = [
    # stance group
    "ppo_goal_cw_stand_holdbc1_hard1",
    "ppo_goal_cw_stand_footlow2_hard1",
    "ppo_goal_cw_stance_dr10",
    "ppo_goal_cw_stand_crouchrise1",
    "ppo_goal_cw_stance_raisefix",
    # walk group
    "ppo_goal_cw_dep_vref1_r1",
    "ppo_goal_cw_arch_noslipphase1_r4",
    "ppo_goal_cw_dep_tip1",
    "ppo_goal_cw_walk_joyheadfric",
    "ppo_goal_cw_walk_joyheadfric_payload_r1",
    "ppo_goal_cw_walk_longdist_r2",
    "ppo_goal_cw_walk_wander30",
    "ppo_goal_cw_walk_slow2",
    "ppo_goal_cw_walk_anchorgate",
]

# Plain-English one-liners (facts from RL_LOG.md / rl_docs; keep each
# under ~58 chars so it fits the 330px description column).
_DESC = {
    # --- stance group (obs 68) ---------------------------------------
    "ppo_goal_cw_friction": "stands up reliably even on slick or grippy floors",
    "ppo_goal_cw_long5m": "stands fine; raising taller works about half the time",
    "ppo_goal_cw_lower": "first model to stand up AND lie back down cleanly",
    "ppo_goal_cw_stance_clear": "failed experiment - can't raise height; skip",
    "ppo_goal_cw_stance_dr08": "solid stand/sit; raises height most of the time",
    "ppo_goal_cw_stance_dr10":
        "old sim champion for stand/sit; didn't transfer to hw",
    "ppo_goal_cw_stance_even": "tried to stop servos running hot; barely helped",
    "ppo_goal_cw_stance_raisefix": "fixed height-raise; all stance moves pass",
    "ppo_goal_cw_stand_dr05": "basic stand-up, easy physics setting",
    "ppo_goal_cw_stand_dr08": "basic stand-up, medium physics setting",
    "ppo_goal_cw_stand_dr10": "basic stand-up, hardest physics; once a hw pick",
    "ppo_goal_cw_stand_crouchrise1":
        "learned to rise from a crouch but wobbles standing",
    "ppo_goal_cw_stand_crouchrise3":
        "crouch-rise retry; leaves two legs parked - broken",
    "ppo_goal_cw_stand_holdbc1_hard1":
        "ON ROBOT: rock-steady stand, but tips when sitting here",
    "ppo_goal_cw_stand_footlow2_hard1":
        "ON ROBOT: full stand-up AND sit both work; default here",
    "ppo_joint_goal": "leftover smoke-test file; ignore",
    "ppo_joint_goal_bc": "imitation-learning tryout; never documented",
    "ppo_joint_goal_bc2m": "imitation warm-start that made things worse",
    "ppo_joint_goal_scratch2m": "early proof raw joint control can learn to stand",
    # --- walk group (obs 72) -----------------------------------------
    "ppo_goal_cw_dep_quad1_c2": "holds a four-leg stance very precisely",
    "ppo_goal_cw_dep_tip1": "tried to learn tip-over recovery; never did",
    "ppo_goal_cw_dep_vref1_r1":
        "ON ROBOT: the walk the real robot runs today",
    "ppo_goal_cw_walk2_gait": "longer strides but still can't hold a speed",
    "ppo_goal_cw_walk_anchorgate": "walks with less foot slip than the sim champ",
    "ppo_goal_cw_walk_curr08": "wider speed-range attempt; fell short",
    "ppo_goal_cw_walk_dr04": "tougher-physics attempt; almost but not quite",
    "ppo_goal_cw_walk_fresh_gait": "control experiment; same foot-skating",
    "ppo_goal_cw_walk_longdist_r2":
        "fast sim walker but it skates its feet; sim-only",
    "ppo_goal_cw_walk_prog3": "reward tweak: lots of motion, zero speed control",
    "ppo_goal_cw_walk_slow": "first model to actually follow a speed command",
    "ppo_goal_cw_walk_slow2": "dependable slow walker",
    "ppo_goal_cw_walk_w08": "faster-speed attempt that got worse",
    "ppo_goal_cw_walk_w08_s1": "accidental exact copy of walk_w08; ignore",
    "ppo_goal_cw_walk_wander30": "drives around for 30s straight without falling",
    "ppo_mjx_joint_walk": "leftover from the GPU trainer; ignore",
    "noslip_scripted_gait":
        "hand-coded gait: feet never slide; U/O to turn",
    "noslip_hybrid_a50":
        "hand-coded, smoother body glide, still no slide",
    "noslip_clampfit_gait":
        "hand-coded, tuned to real servo speed; smoothest",
    "ppo_goal_cw_arch_noslipphase1_r4":
        "ON ROBOT: best RL walk, near-zero slip; needs --phase-obs",
    "ppo_goal_cw_bcnoslip_phase2_init":
        "imitation copy of the hand-coded gait; just a seed",
    "ppo_goal_cw_arch_noslipphase1_r1":
        "overtrained sibling; feet started sliding again",
    "ppo_goal_cw_arch_noslipphase1_r3":
        "shorter-trained sibling; just missed the bar",
    "ppo_goal_cw_dep_bcnoslip2":
        "failed: rocks its body, never found the rhythm",
    "ppo_goal_cw_walk_joyheadfric":
        "steers hard left/right, handles varied floors",
    "ppo_goal_cw_walk_joyheadfric_payload_r1":
        "same steering but also carries extra weight",
}


# Trained goal-ramp profiles (hold_s / ramp_s / target_m) per stance
# checkpoint, read from the exported robot weights JSONs
# (linux_control/policies/*.json meta["profile"] — the sb3 zips don't
# carry them). Driving a model with a DIFFERENT ramp than it trained on
# is out-of-distribution: holdbc1_hard1 (hold 5s, ramp 6s to +111mm)
# stalls its rise at ~55mm when fed the old +45mm @ 12mm/s recipe —
# right under the player's 60mm success gate, i.e. "7 sometimes doesn't
# stand" (operator report 08-13). Models without an export fall back to
# the stance_dr10-era constants the player always used.
_LEGACY_PROFILE = {
    "stand": {"hold_s": 5.0, "ramp_s": 4.0, "target_m": 0.045},
    "lower": {"hold_s": 0.0, "ramp_s": 5.0, "target_m": -0.060},
}


def _load_profiles() -> dict[str, dict]:
    out: dict[str, dict] = {}
    pdir = Path(__file__).resolve().parents[2] / "linux_control" / "policies"
    for f in sorted(pdir.glob("*.json")):
        try:
            meta = json.loads(f.read_text())["meta"]
        except Exception:
            continue
        stem = Path(meta.get("source", "")).stem
        prof = meta.get("profile")
        if stem and isinstance(prof, dict):
            out[stem] = prof
    return out


def _sim_only_obs(role: str, stem: str) -> bool:
    """True if the checkpoint trained on data the real robot can't sense.

    Walk-env checkpoints train by default with PRIVILEGED simulator
    body velocity in the obs (walk_task walk_obs_body_vel=1.0); the
    board has no velocity estimate, so those can never run honestly on
    hardware. The dep-* line AND the noslip phase line train with
    meas:=ref (mode 2.0) — the robot's exact contract. Stance obs (68)
    are encoders/IMU/goal only, all measurable on the robot."""
    return role == "walk" and "_dep" not in stem and "noslip" not in stem


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
    # footlow2: the only stance model that both rises to FULL height and
    # sits cleanly in this env (holdbc1 tips tilt_pitch on every sit
    # here — measured 08-13, see its panel note).
    ap.add_argument("--stance", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_stand_footlow2_hard1.zip"))
    ap.add_argument("--walk", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_walk_longdist_r2.zip"))
    ap.add_argument("--realtime", type=float, default=1.0)
    ap.add_argument("--phase-obs", action="store_true",
                    help="enable the walk env's phase clock (+2 obs) so "
                         "74-obs phase-clock checkpoints are playable "
                         "(cw-arch-noslipphase1 line)")
    ap.add_argument("--phase-hz", type=float, default=0.1666667,
                    help="phase clock rate; default 1/6 Hz = one "
                         "revolution per 6 s clamp-fit gait cycle")
    args = ap.parse_args()

    import cv2
    import mujoco
    from stable_baselines3 import PPO

    from .joint_task import q_rad_to_action
    from .servo_model import SimServoParams

    lc = Path(__file__).resolve().parents[2] / "linux_control"
    if str(lc) not in sys.path:
        sys.path.insert(0, str(lc))
    from noslip_gait import NoSlipGait

    env_kw: dict = {}
    walk_widths = (72,)
    if args.phase_obs:
        from ..config import load_config
        cfg = load_config()
        cfg.setdefault("goal", {})["walk_phase_obs"] = 1.0
        cfg["goal"]["walk_phase_hz"] = args.phase_hz
        env_kw["cfg"] = cfg
        # 74-obs phase-clock checkpoints join the WALK panel; the phase
        # dims are appended after the vel tail, so 72-obs champions
        # keep working on obs[:72].
        _ROLE_OBS[74] = "walk"
        walk_widths = (72, 74)
    env = _PlayEnv(params=SimServoParams.load(), randomize=False,
                   episode_seconds=3600.0, render_mode="rgb_array",
                   **env_kw)

    # --- checkpoint slots: one STANCE (obs 68) + one WALK (obs 72) ------
    cats = scan_policies(args.stance.parent)
    stance_list, walk_list = cats["stance"], cats["walk"]

    def ensure_listed(lst: list[Path], p: Path, want: tuple[int, ...],
                      ) -> int:
        w = _obs_width(p)
        if w not in want:
            raise SystemExit(f"{p}: obs width {w}, need one of {want}")
        p = p.resolve()
        if p not in lst:
            lst.insert(0, p)
        return lst.index(p)

    si = ensure_listed(stance_list, args.stance, (68,))
    wi = ensure_listed(walk_list, args.walk, walk_widths)
    # Scripted-gait rows (alpha 0 and 0.5), bottom of the panel.
    walk_list.extend(_SCRIPTED_ALPHA)

    stance = PPO.load(stance_list[si], device="cpu")
    walk = PPO.load(walk_list[wi], device="cpu")
    n_stance = int(stance.observation_space.shape[0])
    n_walk = int(walk.observation_space.shape[0])
    n_env = int(env.observation_space.shape[0])
    assert n_walk <= n_env, (
        f"walk policy obs {n_walk} wider than env {n_env} "
        "(74-obs phase-clock checkpoints need --phase-obs)")
    assert n_stance < n_env, (
        "stance policy obs must be a prefix of the walk env obs")

    traj = env.traj
    chassis_bid = env.model.body("chassis").id
    msg = ""
    # phases of the 7-key auto stand: None | ("rise", steps) | ("blend", k)
    auto: list | None = None

    # Scripted no-slip gait state (live while the WALK slot is one of the
    # _SCRIPTED_ALPHA rows — then ``walk is None``). Rebuilt every time
    # driving engages from a stop, so its world-pinned foot anchors are
    # re-pinned under wherever the robot is actually standing.
    gait = None
    gait_t = 0.0
    om_cmd = 0.0        # rad/s turn command (scripted gait only)

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
        nonlocal walk, wi, msg, gait, n_walk
        wi = i % len(walk_list)
        if walk_list[wi] in _SCRIPTED_ALPHA:
            walk = None                 # scripted driver, no checkpoint
            gait = None                 # re-pinned when driving engages
            msg = ("walk driver -> SCRIPTED no-slip gait "
                   + ("clamp-fit preset "
                      if walk_list[wi] is _NOSLIP_CLEAN else
                      f"alpha={_SCRIPTED_ALPHA[walk_list[wi]]:.1f} ")
                   + "(U/O to turn)")
            return
        m = PPO.load(walk_list[wi], device="cpu")
        if m.action_space.shape != env.action_space.shape:
            msg = f"{walk_list[wi].stem}: action space mismatch - skipped"
            return
        if int(m.observation_space.shape[0]) > n_env:
            msg = (f"{walk_list[wi].stem}: obs "
                   f"{int(m.observation_space.shape[0])} > env {n_env} "
                   "- needs --phase-obs; skipped")
            return
        walk = m
        n_walk = int(m.observation_space.shape[0])
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
    sitting = False  # sat via 8: freeze on the ground until 7/9/driving

    def chassis_z() -> float:
        return float(env.data.xpos[chassis_bid, 2])

    def q_now() -> np.ndarray:
        return env.data.qpos[7:25].copy()

    def do_reset(start: str, h_goal: float, note: str) -> None:
        nonlocal obs, msg, auto, downed, gait, om_cmd, sitting
        auto = None
        downed = False
        sitting = False
        held.clear()
        gait = None
        om_cmd = 0.0
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
    z_plant = chassis_z()
    q_sit = q_plant  # pose held while sitting (captured at settle)

    def new_gait() -> "NoSlipGait":
        # Sync the gait's stance geometry to THIS env's plant pose (leg 0
        # hip/knee — all legs plant identically), so its neutral feet sit
        # where the robot is actually standing when driving engages.
        if walk_list[wi] is _NOSLIP_CLEAN:
            g = NoSlipGait.clamp_fit()
        else:
            g = NoSlipGait(alpha=_SCRIPTED_ALPHA.get(walk_list[wi], 0.0))
        g.sync_plant_stance(math.degrees(q_plant[1]),
                            math.degrees(q_plant[2]))
        return g

    # Scripted crouch->plant blend, PACED like a real motion: the old
    # fixed 1.5 s popped a 70 mm crouch-stand up at ~48 mm/s — 2.6x any
    # trained rise ramp, i.e. "jumps into standing" (operator 08-13).
    # Duration now scales with the height gap at ~20 mm/s, and a rise
    # that already reached plant height gets only a token joint-align.
    BLEND_RATE = 0.020                    # m/s vertical, physical pace
    q_blend_from = q_plant

    def blend_ticks() -> int:
        gap = max(z_plant - chassis_z(), 0.0)
        return int(round(min(max(gap / BLEND_RATE, 0.5), 4.0) / env.dt))

    profiles = _load_profiles()

    def stance_profile(kind: str) -> dict:
        """Trained goal ramp of the ACTIVE stance model ('stand'/'lower')."""
        prof = profiles.get(stance_list[si].stem, {})
        return {**_LEGACY_PROFILE[kind], **prof.get(kind, {})}

    def apply_ramp(kind: str) -> dict:
        """Point the traj's ref ramp at the active model's trained rate."""
        prof = stance_profile(kind)
        traj.HEIGHT_RATE = abs(prof["target_m"]) / max(prof["ramp_s"], 0.1)
        traj.BELLY_HOLD_S = float(prof["hold_s"]) if kind == "stand" else 0.0
        return prof

    def restore_phys(keep_q: np.ndarray, keep_v: np.ndarray) -> None:
        # Put the robot back EXACTLY where it physically was after a
        # bookkeeping env.reset(), and re-arm the command-side state to
        # the restored joints: the reset left the servo profile's
        # goal/target AND the safety layer's rate-limit anchor
        # (_last_safe) at the RESET pose, so the first ticks would slew
        # the actuators from that pose to the real one — from a sit
        # that sweep passes through the stilt extension and pops the
        # body +35mm (measured 08-13).
        env.data.qpos[:] = keep_q
        env.data.qvel[:] = keep_v
        mujoco.mj_forward(env.model, env.data)
        env._profile.reset(q_now())
        env.safety.set_nominal(q_now())

    def re_anchor_plant() -> None:
        # The robot is physically AT the plant pose now; reset the episode
        # so goal frames (height 0 = standing tall) and both policies see
        # the training distribution, then put the robot back EXACTLY
        # where it physically was — full qpos/qvel, same trick as
        # do_stand. (Used to keep only XY and snap height/quat to the
        # reset's ideal upright, a small unrequested teleport; operator
        # rule 08-13: only an explicit 9/B may move the body.)
        nonlocal obs
        keep_q = env.data.qpos.copy()
        keep_v = env.data.qvel.copy()
        traj.start_at = "plant"
        traj.goal = TaskGoal()
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        obs, _ = env.reset()
        restore_phys(keep_q, keep_v)

    def re_anchor_belly() -> None:
        # Episode re-anchor after the SIT fold: belly-frame bookkeeping
        # reset, robot stays exactly where it physically is (full
        # qpos/qvel restore) — 7 then runs the normal belly stand-up.
        nonlocal obs
        keep_q = env.data.qpos.copy()
        keep_v = env.data.qvel.copy()
        traj.start_at = "zero"
        traj.goal = TaskGoal()
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        obs, _ = env.reset()
        restore_phys(keep_q, keep_v)

    def do_stand() -> None:
        # Realistic stand: never teleport. Crouched in a plant frame ->
        # just raise the height ref. Otherwise re-anchor a belly episode
        # IN PLACE (restore full qpos/qvel after the bookkeeping reset,
        # same trick as re_anchor_plant) and run the auto rise sequence
        # along the ACTIVE stance model's trained ramp profile.
        nonlocal obs, msg, auto, downed, gait, om_cmd, sitting
        if auto is not None:
            if auto[0] == "lower":
                auto = None        # cancel the sit, stand instead
            else:
                msg = ("rise already running - hands off (9 aborts)"
                       if auto[0] == "rise"
                       else "scripted transition - one moment")
                return
        sitting = False
        prof = apply_ramp("stand")
        if not downed and traj.start_at == "plant" and chassis_z() > 0.09:
            traj.goal.height_ref = 0.0
            msg = "rising back up (in place)"
            return
        keep_q = env.data.qpos.copy()
        keep_v = env.data.qvel.copy()
        downed = False
        gait = None
        om_cmd = 0.0
        held.clear()
        traj.start_at = "zero"
        traj.goal = TaskGoal()
        traj.goal.height_ref = float(prof["target_m"])
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        obs, _ = env.reset()
        restore_phys(keep_q, keep_v)
        rise_total = float(prof["hold_s"]) + float(prof["ramp_s"]) + 1.5
        auto = ["rise", 0, rise_total]
        msg = (f"RISE (in place): curl ~{prof['hold_s']:.0f}s, rise to "
               f"{prof['target_m'] * 1000:+.0f}mm, blend - hands off")

    def do_sit() -> None:
        # A REAL sit, exactly the robot's lower choreography: the active
        # model's trained LOWER ramp takes the body to its crouch (that
        # is ALL the height-ref interface can command — the -60mm clamp
        # keeps a "sit" a subtle 45mm dip, the 08-13 "not sitting" bug),
        # then torque-off LIMP settling sags the body onto its belly,
        # like the robot's lower-then-limp. It stays frozen there
        # (`sitting`; no policy — the stance models have no trained
        # "rest on the ground" behavior and stand right back up) until
        # 7 stands it up. Cancels a running rise.
        nonlocal msg, auto, om_cmd
        if downed:
            msg = "robot is down - 7 to rise or 9 to reset first"
            return
        if sitting:
            msg = "already lowered - 7 to rise"
            return
        if auto is not None and auto[0] in ("blend", "fold"):
            msg = "scripted transition - one moment, then 8 again"
            return
        auto = None                     # cancels a running rise
        traj.vx = traj.vy = 0.0
        om_cmd = 0.0
        prof = apply_ramp("lower")
        if traj.start_at in ("zero", "belly"):
            # Belly-frame episode (cancelled rise): the policy cannot
            # track back DOWN here (measured: ref 0 still stands) — go
            # straight to the limp settle.
            auto = ["fold", 0, int(6.0 / env.dt), chassis_z()]
            msg = "LOWER: settling to the ground (torque off) - hands off"
            return
        traj.goal.height_ref = float(prof["target_m"])
        total = float(prof["hold_s"]) + float(prof["ramp_s"]) + 1.5
        auto = ["lower", 0, total]
        msg = (f"LOWER: {prof['target_m'] * 1000:+.0f}mm crouch, then "
               "settle to the ground - hands off")

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
                       else "WALK models (obs "
                       + "/".join(str(w) for w in walk_widths) + ")")
                cv2.putText(panel,
                            txt + "  R = on robot  * = sim-only  S = scripted",
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
            flag = ("R" if stem in _ON_ROBOT
                    else "S" if lst[i] in _SCRIPTED_ALPHA
                    else "*" if _sim_only_obs(role, stem) else " ")
            mark = (">" if sel else " ") + flag
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
        scripted = walk is None
        walking = ((cmd_speed > 1e-3 or (scripted and abs(om_cmd) > 1e-3))
                   and auto is None and not downed and not sitting)
        if not walking and gait is not None:
            gait = None      # stale anchors: re-pin on the next engage
        if downed:
            # No auto-reset: freeze the joints where they are and wait
            # for the operator (7 = try to stand in place, 9 = reset) —
            # same "stop and wait" rule as the real robot.
            action = q_rad_to_action(q_now())
        elif auto is not None and auto[0] == "rise":
            action, _ = stance.predict(obs[:n_stance], deterministic=True)
            auto[1] += 1
            if auto[1] * env.dt >= auto[2]:
                if chassis_z() > 0.06:
                    q_blend_from = q_now()
                    n_blend = blend_ticks()
                    auto = ["blend", 0, n_blend]
                    msg = (f"aligning to the walk stance "
                           f"({n_blend * env.dt:.1f}s)...")
                else:
                    auto = None
                    msg = "rise failed - press 9 to reset standing"
        elif auto is not None and auto[0] == "blend":
            auto[1] += 1
            s = min(auto[1] / auto[2], 1.0)
            action = q_rad_to_action(
                (1.0 - s) * q_blend_from + s * q_plant)
            if auto[1] >= auto[2]:
                re_anchor_plant()
                auto = None
                msg = "up at walk stance - I/K/J/L to walk, 8 to lower"
        elif auto is not None and auto[0] == "lower":
            action, _ = stance.predict(obs[:n_stance], deterministic=True)
            auto[1] += 1
            if auto[1] * env.dt >= auto[2]:
                auto = ["fold", 0, int(6.0 / env.dt), chassis_z()]
                msg = "settling to the ground (torque off)..."
        elif auto is not None and auto[0] == "fold":
            # Torque-off limp settle — the sim twin of the robot's
            # lower-then-limp sit. Raw physics ticks, no env.step (a
            # position action would hold the stilts up, and the stance
            # policy would stand right back up).
            env._advance(limp=True)
            action = None
            auto[1] += 1
            z = chassis_z()
            settled = (auto[1] * env.dt > 1.0 and abs(z - auto[3]) < 2e-5)
            auto[3] = z
            if settled or auto[1] >= auto[2]:
                re_anchor_belly()
                auto = None
                sitting = True
                q_sit = q_now()
                msg = "lowered, parked on the ground - 7 to rise"
        elif sitting:
            # Parked on the ground: hold the pose captured at settle
            # (deadband makes this ~zero torque). A fixed target — NOT
            # q_now() re-read each tick, which self-chases and ratchets
            # the body back up (measured +1.5mm/tick, 08-13).
            action = q_rad_to_action(q_sit)
        elif walking and scripted:
            if gait is None:
                gait = new_gait()
                gait_t = 0.0
            gait.set_velocity(vx=traj.vx, vy=traj.vy, omega=om_cmd)
            action = q_rad_to_action(np.radians(gait.desired_deg(gait_t)))
            gait_t += env.dt
        elif walking:
            action, _ = walk.predict(obs[:n_walk], deterministic=True)
        else:
            action, _ = stance.predict(obs[:n_stance], deterministic=True)
        if action is not None:
            obs, _r, term, trunc, info = env.step(action)
            if (term or trunc) and not downed:
                downed = True
                auto = None
                held.clear()
                traj.vx = traj.vy = 0.0
                om_cmd = 0.0
                msg = (f"[{info.get('termination_reason') or 'episode end'}]"
                       " DOWN - 7 rise in place, 9 reset standing")

        # --- HUD ---------------------------------------------------------
        frame = env.render()
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (VIEW_W, VIEW_H),
                         interpolation=cv2.INTER_LINEAR)
        v = env._body_vel_xy()
        g = traj.goal
        if downed:
            mode_txt = "DOWN (episode terminated) - 7 rise, 9 reset"
            mode_col = (60, 60, 255)
        elif auto is not None:
            mode_txt = {
                "rise": "RISING (auto): curl + rise...",
                "blend": "RISING (auto): align to walk stance...",
                "lower": "LOWERING (auto): trained lower to crouch...",
                "fold": "LOWERING (auto): limp settle to the ground...",
            }[auto[0]]
            mode_col = (0, 200, 255)
        elif sitting:
            mode_txt = "LOWERED (parked) - 7 to rise, 9 to reset standing"
            mode_col = (0, 200, 255)
        elif walking and scripted:
            mode_txt = ("WALK: scripted no-slip gait "
                        f"a={_SCRIPTED_ALPHA.get(walk_list[wi], 0.0):.1f}  "
                        f"[{gait.phase_name() if gait else '-'}]")
            mode_col = (40, 240, 240)
        elif walking:
            mode_txt, mode_col = "WALK policy", (40, 240, 40)
        else:
            mode_txt, mode_col = "STANCE policy", (240, 200, 40)
        pad_name = pad.name if pad else None
        pad_txt = (f"pad: {pad_name}" if pad_name
                   else pad_err or "no gamepad - turn it on, it hot-plugs")
        lines = [
            (f"{mode_txt}   height {chassis_z() * 1000:.0f}mm", mode_col),
            (f"CMD vx {traj.vx:+.3f} vy {traj.vy:+.3f} m/s"
             + (f" om {om_cmd:+.2f} rad/s" if scripted else "")
             + f"   ACTUAL vx {v[0]:+.3f} vy {v[1]:+.3f}", (200, 200, 40)),
            (f"goal height {g.height_ref * 1000:+.0f}mm  "
             f"roll {math.degrees(g.roll_ref):+.1f}  "
             f"pitch {math.degrees(g.pitch_ref):+.1f}", (200, 200, 200)),
            (f"stance[{si + 1}/{len(stance_list)}] {stance_list[si].stem}   "
             f"walk[{wi + 1}/{len(walk_list)}] {walk_list[wi].stem}",
             (230, 160, 230)),
            (pad_txt + "   L-stick walk  A rise  B lower  Y reset  X belly",
             (120, 220, 220) if pad_name else (140, 140, 140)),
            ("pad: dpad U/D stance model  L/R walk model  LB/RB height",
             (120, 220, 220) if pad_name else (140, 140, 140)),
            ("keys: HOLD arrows to drive (release = stop)   "
             "7 rise  8 lower  9 reset  B belly", (180, 180, 180)),
            ("keys: I/K/J/L cruise trim  U/O turn (scripted gait)  "
             "0/space stop  =/- height  "
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
                # Never swallow input silently (08-13: "why can't I walk
                # coming out of a stand or sit" — drive keys pressed
                # during the auto choreography just did nothing).
                msg = {"rise": "rising - drive keys work when the "
                               "rise finishes",
                       "blend": "almost up - drive keys work in "
                                "a moment",
                       "lower": "lowering - 7 cancels and rises "
                                "back up",
                       "fold": "settling to the ground - 7 to rise "
                               "when parked",
                       }[auto[0]]
                return False
            if downed:
                msg = "robot is down - 7 to rise or 9 to reset first"
                return False
            if sitting:
                msg = "lowered - press 7 (pad: A) to rise first"
                return False
            if chassis_z() < 0.09:
                msg = "too low to walk - press 7 (pad: A) to rise first"
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
                om_cmd = 0.0
                stick_live = False
            if "A" in pressed:
                do_stand()
            elif "B" in pressed:
                do_sit()
            elif "Y" in pressed:
                do_reset("plant", 0.0, "reset standing")
            elif "X" in pressed:
                do_reset("zero", 0.0, "reset belly-down (A to rise)")
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
        elif k in (ord("u"), ord("U"), ord("o"), ord("O")):
            if not scripted:
                msg = "U/O turn needs the scripted no-slip walk driver (, .)"
            elif engage_walk():
                d = _STEP_W if k in (ord("u"), ord("U")) else -_STEP_W
                om_cmd = float(np.clip(om_cmd + d, -0.30, 0.30))
        elif k in (ord("0"), ord(" ")):
            held.clear()
            traj.vx = traj.vy = 0.0
            om_cmd = 0.0
            msg = "stopped - HOLD (stance policy holding still)"
        elif k == ord("7"):
            do_stand()
        elif k == ord("8"):
            do_sit()
        elif k in (ord("9"), ord("r"), ord("R")):
            do_reset("plant", 0.0, "reset standing")
        elif k in (ord("b"), ord("B")):
            do_reset("zero", 0.0, "reset belly-down (7 to rise)")
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
            om_cmd = 0.0
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
