"""On-robot RL policy runner: STAND UP / LOWER / WALK buttons (web UI).

Runs the sim-trained raw-joint PPO policies (exported to plain numpy
weights by ``rl_move/sim/export_policy_np.py`` — no torch on the board)
in the exact conventions they were trained with. Two weight files:
``rl_policy_weights.json`` = stance champion (stand/lower, obs 68),
``rl_walk_weights.json`` = walk champion (obs 72; see the walk-mode
constants below for its hardware caveats). Walk files may also be
obs 74 = obs 72 + [sin, cos] of a phase clock the runner keeps
(advances at meta["phase_hz"] while velocity is commanded, frozen at
zero command — the sim's goal.walk_phase_obs=1 contract; the
cw-arch-noslipphase1 no-slip line). Phase policies run naked: no
rot-60 / mirror (they train all headings, no wedge).

- 25 Hz loop; obs = build_obs(q, qd, tilt-rel-to-start, gyro,
  prev_action(18), goal(9)) with q_nom = the pose read at arm time.
- action in [-1,1]^18 -> absolute joint targets via the AXIS_LIMITS_DEG
  center/half-range map (same as sim joint_task.action_to_q_rad).
- goal height ramps mirror the training GoalGenerator. The shape
  (hold_s / ramp_s / target_m / total_s) comes from the weight file's
  OWN meta["profile"] (export_policy_np.py --extra-meta) so it always
  matches the config the checkpoint trained with; legacy files without
  a profile get the stance_dr10-era constants:
  rise  = hold 0 for 5 s (curl window), ramp to +50 mm over 4 s, hold.
  lower = hold 0 for 1 s, ramp to -45 mm over 5 s, hold, then limp.
  The rise+hold specialist (ppo_goal_cw_stand_holdbc1_hard1, 08-11 —
  the sim-proven learned stand-up, rl_docs/RISE.md) ships hold 5 s,
  ramp 6 s, target +111 mm, total 12.5 s; its settled stand is in the
  walk champion's start distribution (handoff eval: 12/12 rises handed
  off with zero falls, scripted blend adds nothing), so the joystick
  chain on hardware is stand -> walk -> go_zero("sit").
- every command goes through rl_move.safety.SafetyLayer: 1.5 deg/tick
  rate clamp, joint limits, relative-tilt trip (10 deg for stand/lower,
  25 deg for walk — see WALK_MAX_TILT_DEG), sustained 2.5 A trip,
  temp/load trips. Trip => immediate limp (do not fight a fall).

Post-2026-08-06 rules baked in: NO motion unless every preflight gate
passes — all 18 servo IDs answering, IMU ok, tilt < 12 deg, and the
present pose near the expected start (flat/belly for stand, captured
plant for lower). The operator must be watching; the web button is the
explicit order.
"""
from __future__ import annotations

import csv
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

# rl_move lives one level above linux_control on the robot and in repo.
_HERE = Path(__file__).resolve().parent
for _p in (_HERE.parent, _HERE, _HERE / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import cfg_get, load_config            # noqa: E402
from rl_move.env import TaskGoal, build_obs                # noqa: E402
from rl_move.robot_state import (                          # noqa: E402
    DEG2RAD, N_JOINTS, RAD2DEG, RobotStateEstimator,
)
from rl_move.safety import AXIS_LIMITS_DEG, SafetyLayer    # noqa: E402

# Rot-60 canonicalizer (08-11, RL_PLAN queue 2.1 deploy-side port).
# numpy-only module, shipped by deploy_adb.sh. The wrapper is an exact
# no-op (k=0) for commands within the trained +/-30 deg forward wedge,
# and covers the FULL CIRCLE of headings by the robot's exact hexagonal
# symmetry (rl_move/sim/rot60.py docstring; proved by test_rot60.py).
# If the module is missing on the board, walk falls back to refusing
# any command outside the trained wedge instead of running a heading
# the naked policy is known to freeze/degenerate on.
try:
    from rl_move.sim.rot60 import Rot60Policy              # noqa: E402
    _ROT60_OK = True
except Exception:                                          # pragma: no cover
    Rot60Policy = None
    _ROT60_OK = False

# Sagittal-mirror chirality selection (08-12, TURN.md deploy port).
# Every walk-lineage champion carries a command-invariant ~+0.09 rad/s
# LEFT yaw drift baked into its gait chirality; reflecting the policy
# (mirror.MirrorPolicy, numpy-only like rot60) produces a RIGHT-drifter
# with the same gait competence — sim-proven PASS 08-11
# (probe_mirror_turn: drift flips sign, travel matched, heading-hold
# 2-4 deg vs 38 deg naked drift over 12 s). Selecting naked-vs-mirrored
# by desired turn sign = commanded ARC turning (~2 deg/s, slow);
# alternating on the accumulated heading = drive straight. If the
# module is missing on-board, turn requests are refused (the default
# turn=None walk never needs it).
try:
    from rl_move.sim.mirror import MirrorPolicy            # noqa: E402
    _MIRROR_OK = True
except Exception:                                          # pragma: no cover
    MirrorPolicy = None
    _MIRROR_OK = False

WEIGHTS_PATH = _HERE / "rl_policy_weights.json"        # stance (obs 68)
WALK_WEIGHTS_PATH = _HERE / "rl_walk_weights.json"     # walk (obs 72)
HZ = 25.0
DT = 1.0 / HZ

# LEGACY trained trajectory shapes (stance_dr10-era rl_move/config.yaml
# goal section). Policies exported since 08-11 carry their own trained
# goal profile in meta["profile"] (export_policy_np.py --extra-meta) so
# runner constants can never drift from the config a checkpoint was
# actually trained with — see policy_profile(). These constants are the
# fallback for weight files exported before that (e.g. stance_dr10).
RISE_HOLD_S = 5.0      # curl window: height ref pinned at 0
RISE_RAMP_S = 4.0
RISE_TARGET_M = 0.050  # mid of the trained 30-70 mm range
RISE_TOTAL_S = 16.0    # hold + ramp + ~7 s stabilise, then hold pose
LOWER_HOLD_S = 1.0
LOWER_RAMP_S = 5.0
LOWER_TARGET_M = -0.045
LOWER_TOTAL_S = 11.0   # ends resting on the belly -> limp

# mode -> legacy profile; meta["profile"][mode] overrides key-by-key.
_LEGACY_PROFILE = {
    "stand": {"hold_s": RISE_HOLD_S, "ramp_s": RISE_RAMP_S,
              "target_m": RISE_TARGET_M, "total_s": RISE_TOTAL_S},
    "lower": {"hold_s": LOWER_HOLD_S, "ramp_s": LOWER_RAMP_S,
              "target_m": LOWER_TARGET_M, "total_s": LOWER_TOTAL_S},
}


def policy_profile(policy: "NumpyPolicy", mode: str) -> dict:
    """Goal-ramp shape for ``mode``: the policy's own trained profile
    (meta["profile"][mode], written by export_policy_np.py) over the
    legacy constants. The rise+hold specialist
    (ppo_goal_cw_stand_holdbc1_hard1) trained hold 5 s / ramp 6 s /
    target 108-114 mm — NOT the legacy 5/4/50 shape; feeding it the
    legacy ramp would command a half-height stand."""
    prof = dict(_LEGACY_PROFILE[mode])
    prof.update((policy.meta.get("profile") or {}).get(mode) or {})
    return prof

PREFLIGHT_MAX_TILT_DEG = 12.0
# Start-pose gates (max per-joint |delta| from the expected pose).
STAND_START_TOL_DEG = 30.0   # near flat belly pose (logical zero-ish)
LOWER_START_TOL_DEG = 25.0   # near the captured plant stance

# Walk mode (ppo_goal_cw_dep_vref1_r1, obs 72) — the deployment-contract
# champion: trained with goal.walk_obs_body_vel=2, i.e. vx/vy_meas := ref
# IS the training contract, bit-identical to what this runner feeds
# (verdict PASS 08-10, no erosion vs parent). Still an operator-supervised
# experiment, tightly bounded:
# - starts ONLY from the captured plant stance (same gate as lower);
# - command ramps 0 -> v over 1 s after a 1 s settle (training profile),
#   holds, then ramps back to 0 for the last second and HOLDS the pose;
# - speed clamped to the trained band; duration clamped to 20 s;
# - the 4 walk obs dims are [vx_ref, vy_ref, vx_meas, vy_meas]/0.15,
#   with meas := ref exactly as in training (contract-exact);
# - FULL-CIRCLE headings via the rot-60 exact-equivariance
#   canonicalizer (rl_move/sim/rot60.py, 08-11): obs rotated + legs
#   relabeled into the trained +/-30 deg wedge, action un-relabeled.
#   Exact no-op (k=0) for forward-wedge commands, so the proven
#   forward contract is bit-identical; off-wedge commands are REFUSED
#   if the canonicalizer is disabled/missing (naked policy freezes or
#   degenerates there — sim-proven, logs/rot60/).
WALK_VEL_SCALE = 0.15
WALK_SPEED_MAX = 0.06        # trained command band is 0.05-0.06 m/s
WALK_HOLD_S = 1.0
WALK_RAMP_S = 1.0
WALK_MAX_TOTAL_S = 20.0
WALK_START_TOL_DEG = 25.0    # near the captured plant stance
# Walk-mode relative-tilt trip. A WORKING gait rocks +-10-20 deg in roll
# and pitch (operator-measured, scripted gait, 08-09 night); the config's
# 10 deg trip would terminate exactly the weight transfer a real gait
# needs. Walk-mode arms train AND deploy with a 25 deg envelope
# (cw-dep-vref1-r1: --cfg-set safety.max_roll_deg=25 / max_pitch_deg=25).
# Stand/lower keep the config's 10 deg trip — the stance champion
# (stance_dr10) trained with it, and its episodes should sit at +-1 deg.
WALK_MAX_TILT_DEG = 25.0

# Chirality selection (turn= on walk moves). The naked champion drifts
# LEFT (+wz, probe_mirror_turn 08-11); the mirrored policy drifts right
# at the same rate. If a future champion drifts right, flip this sign —
# the selector and the left/right mapping both key off it.
NAKED_DRIFT_SIGN = +1
# Heading-hold bang-bang hysteresis: the sim probe's 4 deg. Below it
# the selector keeps the current chirality, so it cannot chatter.
TURN_HYST_RAD = math.radians(4.0)


class ChiralitySelector:
    """naked/mirror selection for a walk episode (TURN.md deploy port).

    turn="left"/"right": constant chirality — the one whose drift turns
    the commanded way. turn="hold": bang-bang on the accumulated
    heading (integrated gyro z, rad) with TURN_HYST_RAD hysteresis —
    veered too far one way -> run the chirality that drifts back.
    Mirrors probe_mirror_turn.rollout's selector exactly; locked by
    tests/test_mirror_runner.py.
    """

    def __init__(self, turn: str, drift_sign: int = NAKED_DRIFT_SIGN):
        assert turn in ("left", "right", "hold")
        self.turn = turn
        self.drift_sign = +1 if drift_sign >= 0 else -1
        left = "naked" if self.drift_sign > 0 else "mirror"
        right = "mirror" if self.drift_sign > 0 else "naked"
        self.active = {"left": left, "right": right,
                       "hold": "naked"}[turn]
        self.switches = 0
        self.heading = 0.0

    def update(self, gyro_z: float, dt: float) -> str:
        """Integrate heading, return the chirality for this tick."""
        self.heading += float(gyro_z) * dt
        if self.turn != "hold":
            return self.active
        want = self.active
        if self.heading > TURN_HYST_RAD:
            want = "mirror" if self.drift_sign > 0 else "naked"
        elif self.heading < -TURN_HYST_RAD:
            want = "naked" if self.drift_sign > 0 else "mirror"
        if want != self.active:
            self.active = want
            self.switches += 1
        return self.active

# Drive session (MuJoCo-viewer-style held-key driving, operator 08-11).
# The browser holds arrow keys -> POST /api/rl/drive/cmd heartbeats carry
# the live (vx, vy); release -> (0, 0) -> the hold model. The loop NEVER
# trusts a stale command: refs decay to zero unless a heartbeat younger
# than DRIVE_CMD_TIMEOUT_S says otherwise, so a closed tab / dropped
# WiFi degrades to "stand still and hold", not "keep walking".
DRIVE_CMD_TIMEOUT_S = 0.6    # heartbeats at ~5 Hz; 3 misses = stop
DRIVE_IDLE_END_S = 120.0     # no heartbeat at all -> end session (hold)
DRIVE_MAX_SESSION_S = 300.0  # hard cap per session (decel + hold)
DRIVE_HOLD_SWITCH_S = 1.5    # zero-cmd dwell before flipping to the
                             # hold model (quick taps stay on walk)


class DriveCommand:
    """Thread-safe live command mailbox: HTTP handler -> drive loop.

    ``set()`` is called from web request threads (must never touch the
    bus); the 25 Hz loop reads with ``get()`` and publishes a UI
    snapshot via ``live``.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._vx = 0.0
        self._vy = 0.0
        # Counts as a heartbeat so the idle-end clock starts at session
        # birth instead of firing instantly (refs are zero until the
        # browser actually sends commands).
        self._t_cmd = time.monotonic()
        self._stop = False
        self._live: dict = {}

    def set(self, vx: float, vy: float) -> None:
        spd = math.hypot(vx, vy)
        if spd > WALK_SPEED_MAX:
            s = WALK_SPEED_MAX / spd
            vx, vy = vx * s, vy * s
        with self._lock:
            self._vx, self._vy = float(vx), float(vy)
            self._t_cmd = time.monotonic()

    def request_stop(self) -> None:
        with self._lock:
            self._stop = True

    def get(self) -> tuple[float, float, float, bool]:
        """(vx, vy, seconds_since_heartbeat, stop_requested)."""
        with self._lock:
            return (self._vx, self._vy,
                    time.monotonic() - self._t_cmd, self._stop)

    @property
    def live(self) -> dict:
        with self._lock:
            return dict(self._live)

    def publish(self, snap: dict) -> None:
        with self._lock:
            self._live = snap

_CENTER_RAD = np.array([
    (AXIS_LIMITS_DEG[j % 3][0] + AXIS_LIMITS_DEG[j % 3][1]) * 0.5 * DEG2RAD
    for j in range(N_JOINTS)])
_HALF_RAD = np.array([
    (AXIS_LIMITS_DEG[j % 3][1] - AXIS_LIMITS_DEG[j % 3][0]) * 0.5 * DEG2RAD
    for j in range(N_JOINTS)])


class NumpyPolicy:
    """Deterministic SB3 MlpPolicy actor: tanh MLP + linear head."""

    def __init__(self, path: Path = WEIGHTS_PATH):
        d = json.loads(Path(path).read_text())
        self.meta = d["meta"]
        self.W1 = np.array(d["W1"]); self.b1 = np.array(d["b1"])
        self.W2 = np.array(d["W2"]); self.b2 = np.array(d["b2"])
        self.Wo = np.array(d["Wout"]); self.bo = np.array(d["bout"])

    def act(self, obs: np.ndarray) -> np.ndarray:
        h = np.tanh(self.W1 @ obs + self.b1)
        h = np.tanh(self.W2 @ h + self.b2)
        return np.clip(self.Wo @ h + self.bo, -1.0, 1.0)


class _Rot60ModelShim:
    """Adapts NumpyPolicy.act to the SB3 ``predict()`` Rot60Policy calls.

    NumpyPolicy is deterministic; the flag is accepted and ignored.
    """

    def __init__(self, policy: NumpyPolicy):
        self._policy = policy

    def predict(self, obs, deterministic: bool = True, **_kw):
        return self._policy.act(np.asarray(obs, dtype=float)), None


def make_walk_canonicalizer(policy: NumpyPolicy, cfg: dict):
    """Rot-60 wrapper EXACTLY as the walk loop uses it (None if absent).

    Single source of truth: this wraps rl_move.sim.rot60.Rot60Policy
    itself (no ported copy to drift). It reads vx/vy_ref straight from
    obs indices 68:70 — the same contract the sim evals run — and keeps
    per-episode sector state with hysteresis + zero-command hold.
    tests/test_rot60_runner.py locks this path against rot60.py.
    """
    if not _ROT60_OK:
        return None
    ts = float(cfg_get(cfg, "obs", "tilt_scale", default=0.2))
    return Rot60Policy(_Rot60ModelShim(policy), tilt_scale=ts)


def make_walk_mirror(policy: NumpyPolicy, cfg: dict, *, rot60: bool):
    """Reflected stack for chirality selection (None if mirror absent).

    Mirror OUTERMOST: reflect the world's obs, run the SAME shipped
    stack (rot60 canonicalizer + policy — its own instance, so its
    sector hysteresis state never sees the other chirality's frames),
    reflect the action back. Wrapping outside rot60 keeps the
    composition correct for any heading: the reflected command selects
    the reflected sector by construction. numpy-only end to end.
    """
    if not _MIRROR_OK:
        return None
    inner = (make_walk_canonicalizer(policy, cfg) if rot60 and _ROT60_OK
             else _Rot60ModelShim(policy))
    return MirrorPolicy(inner, walk=True,
                        obs_dim=int(policy.meta.get("obs_dim", 72)))


def heading_in_trained_wedge(vx: float, vy: float,
                             wedge_deg: float = 30.0) -> bool:
    """True if the commanded heading is inside the trained +/-30 deg
    forward wedge (zero command counts as inside)."""
    if math.hypot(vx, vy) < 1e-6:
        return True
    return abs(math.degrees(math.atan2(vy, vx))) <= wedge_deg


def _height_ref(prof: dict, t: float) -> float:
    """Height reference at time t for a stand/lower goal profile:
    hold 0 for hold_s, ramp to target_m over ramp_s, then hold."""
    if t < prof["hold_s"]:
        return 0.0
    f = min(1.0, (t - prof["hold_s"]) / prof["ramp_s"])
    return f * prof["target_m"]


def _walk_vel_ref(t: float, total_s: float,
                  vx: float, vy: float) -> tuple[float, float]:
    """Training-shaped command: settle, 1 s ramp in, hold, 1 s ramp out."""
    if t < WALK_HOLD_S:
        f = 0.0
    elif t < WALK_HOLD_S + WALK_RAMP_S:
        f = (t - WALK_HOLD_S) / WALK_RAMP_S
    elif t > total_s - WALK_RAMP_S:
        f = max(0.0, (total_s - t) / WALK_RAMP_S)
    else:
        f = 1.0
    return f * vx, f * vy


class _EpisodeLog:
    """Every RL episode leaves a full local trace in ``logs/``.

    ``rl_<mode>_<stamp>.csv``  — one row per 25 Hz tick: attitude, gyro,
    goal refs, measured q (18), commanded q (18), raw action (18), and
    per-servo current when full feedback is available.
    ``rl_<mode>_<stamp>_summary.json`` — params + final result.
    Start/end also land in events.jsonl (kind ``rl_episode``).
    Pull with receive_robot_logs.py / scp for offline analysis.
    """

    def __init__(self, mode: str, params: dict, obs_dim: int = 0):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        d = _HERE / "logs"
        d.mkdir(exist_ok=True)
        self.mode = mode
        self.params = params
        self.obs_dim = int(obs_dim)
        self.csv_path = d / f"rl_{mode}_{stamp}.csv"
        self.sum_path = d / f"rl_{mode}_{stamp}_summary.json"
        self.started_iso = time.strftime("%Y-%m-%dT%H:%M:%S")
        self._n = 0
        self._f = self.csv_path.open("w", newline="")
        self._w = csv.writer(self._f)
        # ``phase`` and obs columns added 08-10 after the dep-tip1 fall
        # debug: the robot tipped AFTER "walk done" (episode ends
        # holding the last stance) with zero data past the last tick,
        # and without the obs vector there was no way to tell "policy
        # saw the roll and didn't act" from "obs pipeline fed it
        # garbage". phase=run rows carry everything; phase=tail rows
        # (post-episode, ~3 s at 10 Hz, no commands sent) carry
        # attitude/gyro/q/currents only. Logged obs lets the exact
        # policy be replayed offline: action mismatch = obs/weights
        # bug, action match = behavior/contact story.
        self._w.writerow(
            ["t_s", "phase", "roll_deg", "pitch_deg",
             "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
             "height_ref_mm", "vx_ref_mps", "vy_ref_mps", "max_cur_a"]
            + [f"q{j}_deg" for j in range(N_JOINTS)]
            + [f"cmd{j}_deg" for j in range(N_JOINTS)]
            + [f"act{j}" for j in range(N_JOINTS)]
            + [f"cur{j}_a" for j in range(N_JOINTS)]
            + [f"obs{k}" for k in range(self.obs_dim)]
            # appended LAST so all prior column indices stay stable for
            # existing offline parsers. Walk mode: rot-60 sector index
            # (obs columns hold the REAL-frame obs; replaying them
            # through make_walk_canonicalizer must reproduce act* —
            # the offline replay-parity contract).
            # "mirror": 1 when the mirrored chirality drove this tick
            # (turn= walk moves), 0 naked, "" no selector. Appended
            # after rot60_k for the same index-stability reason.
            + ["rot60_k", "mirror"])
        try:
            from event_log import emit
            emit("rl_episode", f"{mode} started ({self.csv_path.name})",
                 src="rl_policy", data=params)
        except Exception:
            pass

    def tick(self, t: float, state, action, q_cmd_rad, goal,
             vx_r: float, vy_r: float, max_cur: float,
             obs=None, phase: str = "run", rot60_k=None,
             mirror_on=None) -> None:
        cur = (state.servo_current.tolist()
               if state.servo_current is not None else [None] * N_JOINTS)
        obs_cols = ([round(float(o), 4) for o in obs]
                    if obs is not None else [""] * self.obs_dim)
        self._w.writerow(
            [round(t, 3), phase,
             round(state.imu_roll * RAD2DEG, 2),
             round(state.imu_pitch * RAD2DEG, 2)]
            + [round(float(g) * RAD2DEG, 2) for g in state.imu_gyro]
            + [round(goal.height_ref * 1000, 1) if goal is not None
               else "",
               round(vx_r, 4), round(vy_r, 4), round(max_cur, 3)]
            + [round(float(q) * RAD2DEG, 2) for q in state.joint_position]
            + ([round(float(q) * RAD2DEG, 2) for q in q_cmd_rad]
               if q_cmd_rad is not None else [""] * N_JOINTS)
            + ([round(float(a), 4) for a in action]
               if action is not None else [""] * N_JOINTS)
            + ["" if c is None else round(float(c), 3) for c in cur]
            + obs_cols
            + ["" if rot60_k is None else int(rot60_k),
               "" if mirror_on is None else int(mirror_on)])
        self._n += 1
        if self._n % 25 == 0:      # survive a mid-run kill: flush each ~1 s
            self._f.flush()

    def close(self, result: dict) -> str:
        try:
            self._f.close()
        except Exception:
            pass
        try:
            self.sum_path.write_text(json.dumps(
                {"started": self.started_iso, "csv": self.csv_path.name,
                 "ticks_logged": self._n, "params": self.params,
                 "result": result}, indent=1))
        except Exception:
            pass
        try:
            from event_log import emit
            emit("rl_episode",
                 f"{self.mode} " + ("done" if result.get("ok")
                                    else f"FAILED: {result.get('error')}"),
                 src="rl_policy",
                 level="info" if result.get("ok") else "warn",
                 data={"csv": self.csv_path.name, **result})
        except Exception:
            pass
        return self.csv_path.name


def _read_q_deg(bus) -> tuple[np.ndarray | None, str]:
    pos = bus.read_all_positions()
    if not isinstance(pos, dict):
        return None, "bus read failed"
    missing = [j for j in range(N_JOINTS) if j not in pos]
    if missing:
        return None, f"servo IDs not answering: joints {missing}"
    return np.array([float(pos[j]) for j in range(N_JOINTS)]), ""


def _expected_start_deg(mode: str) -> tuple[np.ndarray | None, str]:
    if mode == "stand":
        # Rise training starts belly-down at logical zero (legs straight
        # out). Partial curls were also trained, so the gate is loose.
        return np.zeros(N_JOINTS), ""
    # lower AND walk both start from the captured plant stance.
    try:
        from feetech_bus import load_plant_pose
        plant = load_plant_pose()
        joints = plant.get("joints_deg")
        if joints is None or len(joints) != 18:
            return None, ("no captured plant pose (joints_deg) — stand "
                          "first or run capture_plant from a good stance")
        return np.asarray(joints, dtype=float), ""
    except Exception as e:  # pragma: no cover
        return None, f"plant pose unavailable: {e}"


def preflight(bus, mode: str) -> tuple[bool, str, dict]:
    """All checks are read-only. Returns (ok, reason, details)."""
    q_deg, err = _read_q_deg(bus)
    if q_deg is None:
        return False, err, {}
    try:
        imu = bus.read_imu(apply_calib=True)
    except Exception:
        imu = None
    if not isinstance(imu, dict) or "ax_g" not in imu:
        return False, "IMU not answering", {}
    mag = math.sqrt(imu["ax_g"] ** 2 + imu["ay_g"] ** 2
                    + imu["az_g"] ** 2)
    if not 0.5 <= mag <= 1.5:
        # A dead/asleep MPU reads zeros; atan2(0,0)=0 would false-pass
        # the tilt gate. At rest |accel| must be ~1 g.
        return False, f"IMU reading implausible (|g|={mag:.2f})", {}
    roll = math.degrees(math.atan2(imu["ay_g"], imu["az_g"]))
    pitch = math.degrees(math.atan2(-imu["ax_g"],
                                    math.hypot(imu["ay_g"], imu["az_g"])))
    exp, err = _expected_start_deg(mode)
    if exp is None:
        return False, err, {}
    dq = np.abs(q_deg - exp)
    tol = {"stand": STAND_START_TOL_DEG,
           "walk": WALK_START_TOL_DEG}.get(mode, LOWER_START_TOL_DEG)
    details = {
        "roll_deg": round(roll, 1), "pitch_deg": round(pitch, 1),
        "max_pose_delta_deg": round(float(np.max(dq)), 1),
        "pose_tol_deg": tol,
    }
    if mode in ("stand", "walk") and (abs(roll) > PREFLIGHT_MAX_TILT_DEG
                                      or abs(pitch) > PREFLIGHT_MAX_TILT_DEG):
        # Name the 08-11 failure mode when it is the likely cause: a
        # tipped body over a folded knee. safe_zero knows how to untrap
        # (low-torque fold) — pointing there beats a bare refusal that
        # scripts answer by retrying stand/walk against the pin.
        hint = ""
        try:
            from pinned_tip import classify_pinned_tip
            v = classify_pinned_tip([float(x) for x in q_deg], roll, pitch)
            details["pinned_tip"] = v
            if v.get("pinned"):
                hint = (" — pinned-leg tip suspected "
                        f"({', '.join(c['name'] for c in v['candidates'])});"
                        " run safe_zero, it untraps first")
        except Exception:
            pass
        return False, (f"tilt too high for start "
                       f"(roll {roll:+.1f} pitch {pitch:+.1f}){hint}"
                       ), details
    if float(np.max(dq)) > tol:
        worst = int(np.argmax(dq))
        want = ("belly-down, legs straight out (logical zero)"
                if mode == "stand" else "the captured standing plant")
        return False, (f"pose is not {want}: joint {worst} is "
                       f"{dq[worst]:.0f} deg from expected (tol {tol:.0f})"
                       ), details
    return True, "", details


def run_policy_move(drive, mode: str, *, on_progress=None,
                    abort_check=None, vx: float = 0.03, vy: float = 0.0,
                    duration_s: float = 6.0, rot60: bool = True,
                    turn: str | None = None,
                    weights_path: Path | None = None,
                    tilt_trip_deg: float | None = None,
                    extra_hold_s: float = 0.0) -> dict:
    """Blocking policy episode. Call from a worker thread.

    ``drive`` is web_drive's DriveController (bus + arm state).
    ``mode`` is "stand", "lower" or "walk". Walk extras: body-frame
    vx/vy (m/s, clamped to the trained band; ANY heading with the
    rot-60 canonicalizer, else the trained +/-30 deg wedge only) and
    duration_s. ``rot60=False`` runs the naked policy (A/B baseline
    for a hardware parity session) — wedge headings only.
    ``turn`` (walk only): None = today's naked path, bit-identical;
    "left"/"right" = constant-chirality arc turn (~2 deg/s, the
    gait's own drift steered by naked-vs-mirrored selection);
    "hold" = heading hold, alternating chirality on the integrated
    gyro-z heading (4 deg hysteresis). Zero training; sim-proven
    (probe_mirror_turn PASS 08-11). Requires rl_move/sim/mirror.py
    on the board or the request is refused up front.
    ``weights_path`` overrides the default slot file (role registry,
    bench_api.rl_roles); obs-dim checks still apply.
    ``tilt_trip_deg`` (stand/lower only, clamped 5..30): operator-
    requested aggressive-tip test envelope — the 08-12 bench trips at
    10.3 deg mid-curl left "would it have stood?" unanswered. Walk
    keeps its own 25 deg envelope.
    ``extra_hold_s`` (stand/lower only, clamped 0..15): extends the
    episode past the profile's total_s; the height ref simply holds
    the target, so the extension is a longer settled hold.
    """
    assert mode in ("stand", "lower", "walk")
    if turn is not None and mode != "walk":
        return {"ok": False, "error": "turn= is walk-only"}
    if turn is not None and turn not in ("left", "right", "hold"):
        return {"ok": False,
                "error": f"bad turn {turn!r} (left / right / hold)"}
    on_progress = on_progress or (lambda p: None)
    abort_check = abort_check or (lambda: False)
    bus = drive.bus
    if bus is None or drive.dry_run:
        return {"ok": False, "error": "no bus"}

    cfg = load_config(str(_HERE.parent / "rl_move" / "config.yaml"))
    canon = None
    mirror = None
    selector = None
    if mode == "walk":
        wpath = weights_path or WALK_WEIGHTS_PATH
        policy = NumpyPolicy(wpath)
        walk_obs = policy.meta.get("obs_dim")
        if walk_obs not in (72, 74):
            return {"ok": False,
                    "error": (f"{Path(wpath).name} is not a walk policy "
                              f"(obs {walk_obs} not 72/74)")}
        # obs 74 = walk + phase clock (cw-arch-noslipphase1 no-slip
        # line): the runner appends [sin, cos] of a clock that advances
        # at meta["phase_hz"] while a velocity is commanded — the exact
        # contract of the sim's goal.walk_phase_obs=1. That line trains
        # ALL headings (no wedge) and has no rot-60/mirror machinery,
        # so it always runs naked. phase_hz MUST come from the export
        # meta: the sim default (1.0 Hz) is NOT this line's clock
        # (0.1666667 Hz) and a wrong clock is a silently broken gait.
        phase_hz = 0.0
        if walk_obs == 74:
            if "phase_hz" not in policy.meta:
                return {"ok": False,
                        "error": (f"{Path(wpath).name} is obs-74 but has "
                                  "no phase_hz in meta — re-export with "
                                  "--extra-meta phase_hz=<trained hz>")}
            phase_hz = float(policy.meta["phase_hz"])
            if turn is not None:
                return {"ok": False,
                        "error": ("turn= is not supported for phase-"
                                  "clock (obs 74) walk policies")}
        spd = math.hypot(vx, vy)
        if spd > WALK_SPEED_MAX:
            s = WALK_SPEED_MAX / spd
            vx, vy = vx * s, vy * s
        total_s = min(max(float(duration_s), 3.0), WALK_MAX_TOTAL_S)
        if rot60 and walk_obs == 72:
            canon = make_walk_canonicalizer(policy, cfg)
        if turn is not None:
            mirror = make_walk_mirror(policy, cfg, rot60=rot60)
            if mirror is None:
                return {"ok": False,
                        "error": ("turn= requested but the mirror "
                                  "module is unavailable "
                                  "(rl_move/sim/mirror.py not "
                                  "deployed)")}
            selector = ChiralitySelector(turn)
        if (canon is None and walk_obs == 72
                and not heading_in_trained_wedge(vx, vy)):
            # Naked, the policy freezes/degenerates off-wedge (sim-
            # proven, rot60.py docstring) — refuse rather than wander.
            return {"ok": False,
                    "error": ("command heading outside the trained "
                              "+/-30 deg wedge and the rot-60 "
                              "canonicalizer is "
                              + ("disabled" if _ROT60_OK else
                                 "unavailable (rl_move/sim/rot60.py "
                                 "not deployed)"))}
    else:
        wpath = weights_path or WEIGHTS_PATH
        policy = NumpyPolicy(wpath)
        if policy.meta.get("obs_dim") != 68:
            return {"ok": False,
                    "error": (f"{Path(wpath).name} is not a stance/"
                              "goal policy (obs "
                              f"{policy.meta.get('obs_dim')} != 68)")}
        prof = policy_profile(policy, mode)
        total_s = float(prof["total_s"]) + min(
            max(float(extra_hold_s or 0.0), 0.0), 15.0)

    ok, reason, details = preflight(bus, mode)
    if not ok:
        return {"ok": False, "error": f"preflight: {reason}", **details}

    def limp():
        try:
            bus.enable_all_torque(False)
        except Exception:
            try:
                drive._torque_all(False)
            except Exception:
                pass

    # --- arm: torque on, hold the PRESENT pose (never yank) ---
    with drive._lock:
        drive.mode = "demo"
        try:
            drive.gait.stop()
        except Exception:
            pass
        if not drive.armed:
            drive._torque_all(True)
            drive.armed = True

    est = RobotStateEstimator(bus, cfg)
    safety = SafetyLayer(cfg)
    if mode == "walk":
        # Match the walk policy's trained tilt envelope (see
        # WALK_MAX_TILT_DEG). The config's 10 deg stays for stand/lower.
        safety.max_roll = math.radians(WALK_MAX_TILT_DEG)
        safety.max_pitch = math.radians(WALK_MAX_TILT_DEG)
    elif tilt_trip_deg:
        # Operator aggressive-tip test envelope (see docstring). The
        # 35 deg fell detector and the current/temp trips stay as-is.
        t_deg = min(max(float(tilt_trip_deg), 5.0), 30.0)
        safety.max_roll = math.radians(t_deg)
        safety.max_pitch = math.radians(t_deg)
    tilt_trip_deg = round(math.degrees(safety.max_roll), 1)
    write_speed = int(cfg_get(cfg, "bus", "write_speed", default=400))
    write_acc = int(cfg_get(cfg, "bus", "write_acc", default=20))

    # Settle reads (no motion): q_nom = the pose we actually start from,
    # tilt reference = the attitude we actually start at (sim-identical).
    state = None
    for _ in range(5):
        state = est.update(want_full_feedback=True)
        time.sleep(DT)
    if state is None or not state.bus_ok:
        limp()
        return {"ok": False, "error": "bus dropped during settle"}
    q_nom = state.joint_position.copy()
    est.set_commanded(q_nom)
    bus.write_all((q_nom * RAD2DEG).tolist(), speed=write_speed,
                  acc=write_acc)
    est.reset_episode_filters()
    for _ in range(3):
        state = est.update()
        time.sleep(DT)
    tilt_ref0 = (state.imu_roll, state.imu_pitch)
    safety.set_nominal(q_nom)
    safety.set_tilt_reference(*tilt_ref0)

    prev_action = np.zeros(N_JOINTS, dtype=float)
    vx_r = vy_r = 0.0
    n_ticks = int(round(total_s * HZ))
    overruns = 0
    max_cur = 0.0
    tilt_rel_max = 0.0
    t_end = 0.0
    t_next = time.monotonic()
    result: dict = {"ok": True, "mode": mode}
    elog = _EpisodeLog(mode, obs_dim=int(policy.meta.get("obs_dim", 0)),
                       params={
        "mode": mode, "total_s": round(total_s, 1), "hz": HZ,
        "policy": dict(policy.meta),
        "q_nom_deg": [round(float(q) * RAD2DEG, 2) for q in q_nom],
        "tilt_ref_deg": [round(tilt_ref0[0] * RAD2DEG, 2),
                         round(tilt_ref0[1] * RAD2DEG, 2)],
        "tilt_trip_deg": tilt_trip_deg,
        "preflight": details,
        **({"vx": round(vx, 3), "vy": round(vy, 3),
            "rot60": canon is not None,
            **({"turn": turn} if turn else {})}
           if mode == "walk" else {}),
    })

    phase = 0.0        # walk phase clock (obs-74 policies only)
    for i in range(n_ticks):
        if abort_check():
            # Operator stop: HOLD pose (torque stays on); X still limps.
            result.update(ok=False, error="aborted",
                          held_pose=True, ticks=i)
            break
        t = i * DT
        if mode == "walk":
            goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0,
                            height_ref=0.0, unload_leg=None)
            vx_r, vy_r = _walk_vel_ref(t, total_s, vx, vy)
            obs = build_obs(cfg, state, q_nom, prev_action, goal=goal,
                            tilt_ref=tilt_ref0)
            # Sim obs tail: [vx_ref, vy_ref, vx_meas, vy_meas]/scale.
            # No velocity estimate on the board -> meas := ref
            # (open-loop; see module notes).
            # float32 = the training/export dtype; also keeps the
            # rot-60 path (rot60.obs_transform casts to float32) BIT-
            # IDENTICAL to the naked path at k=0.
            obs = np.concatenate(
                [obs, np.array([vx_r, vy_r, vx_r, vy_r]) / WALK_VEL_SCALE]
            ).astype(np.float32)
            if walk_obs == 74:
                # Phase tail, then advance while a velocity is
                # commanded — the sim's clock gate (walk_task
                # _augment_obs: s_ref > 1e-3). Episode starts at 0,
                # matching an env reset.
                obs = np.concatenate(
                    [obs, np.array([math.sin(phase), math.cos(phase)])]
                ).astype(np.float32)
                if math.hypot(vx_r, vy_r) > 1e-3:
                    phase = (phase + 2.0 * math.pi * phase_hz * DT) \
                        % (2.0 * math.pi)
        else:
            goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0,
                            height_ref=_height_ref(prof, t),
                            unload_leg=None)
            obs = build_obs(cfg, state, q_nom, prev_action, goal=goal,
                            tilt_ref=tilt_ref0)
        chirality = None
        if selector is not None:
            # Chirality selection (turn=): integrate the heading from
            # gyro z and pick naked vs mirrored for THIS tick. Each
            # chirality runs its own rot60 instance (sector state
            # never sees the other's frames); obs/prev_action/logs
            # stay REAL-frame for both — the wrappers permute
            # internally.
            chirality = selector.update(float(state.imu_gyro[2]), DT)
        if chirality == "mirror":
            raw_act, _ = mirror.predict(obs)
        elif canon is not None:
            # Canonicalize the REAL-frame obs into the trained wedge,
            # un-relabel the action back to real legs (rot60.py).
            # prev_action / logs stay REAL-frame — same contract as
            # the sim evals (Rot60Policy permutes them internally).
            raw_act, _ = canon.predict(obs)
        else:
            raw_act = policy.act(obs)
        action, bad = safety.validate_action(raw_act, n_act=N_JOINTS)
        if action is None:
            limp()
            result.update(ok=False, error=f"bad action: {bad}", ticks=i)
            break
        q_prop = _CENTER_RAD + action * _HALF_RAD
        q_safe, status = safety.filter(q_prop, state, action=action)
        if status.terminate:
            limp()
            result.update(ok=False, error=f"safety trip: {status.reason}"
                          + (f" ({status.detail})" if status.detail else ""),
                          limped=True, ticks=i)
            break
        est.set_commanded(q_safe)
        bus.write_all((q_safe * RAD2DEG).tolist(), speed=write_speed,
                      acc=write_acc)
        prev_action = action.copy()

        t_next += DT
        lag = time.monotonic() - t_next
        if lag > 0:
            overruns += 1
            t_next = time.monotonic()
        else:
            time.sleep(-lag)
        state = est.update()
        if state.servo_current is not None:
            max_cur = max(max_cur,
                          float(np.max(np.abs(state.servo_current))))
        tilt_rel_max = max(
            tilt_rel_max,
            abs(state.imu_roll - tilt_ref0[0]) * RAD2DEG,
            abs(state.imu_pitch - tilt_ref0[1]) * RAD2DEG)
        t_end = t + DT
        elog.tick(t, state, action, q_safe, goal, vx_r, vy_r, max_cur,
                  obs=obs,
                  rot60_k=(canon.k if canon is not None else None),
                  mirror_on=(None if chirality is None
                             else chirality == "mirror"))
        if i % 5 == 0:
            if mode == "walk":
                phase = ("settle" if t < WALK_HOLD_S else
                         "decel" if t > total_s - WALK_RAMP_S else
                         "ramp" if t < WALK_HOLD_S + WALK_RAMP_S
                         else "walk")
                ref_txt = (f"v=({vx_r * 1000:+.0f},{vy_r * 1000:+.0f})mm/s")
                if canon is not None and canon.k:
                    ref_txt += f" sec={canon.k:+d}"
                if selector is not None:
                    ref_txt += (f" {selector.active[:3]}"
                                f" hd={math.degrees(selector.heading):+.0f}")
            else:
                phase = ("curl" if mode == "stand" and t < prof["hold_s"]
                         else "ramp" if t < prof["hold_s"] + prof["ramp_s"]
                         else "hold")
                ref_txt = f"href={goal.height_ref * 1000:+.0f}mm"
            on_progress({
                "msg": (f"{mode} {phase} t={t:4.1f}s "
                        f"{ref_txt} "
                        f"maxI={max_cur:.2f}A"),
                "t_s": round(t, 2), "phase": phase,
                "height_ref_mm": round(goal.height_ref * 1000, 1),
                "roll_deg": round((state.imu_roll - tilt_ref0[0])
                                  * RAD2DEG, 2),
                "pitch_deg": round((state.imu_pitch - tilt_ref0[1])
                                   * RAD2DEG, 2),
                "max_current_a": round(max_cur, 2),
                "overruns": overruns,
            })
    else:
        result["ticks"] = n_ticks

    if result.get("ok") and mode == "lower":
        # Finished on the belly: go limp, that's the safe rest state.
        limp()
        result["limped"] = True
    if mode == "walk":
        # Walk ends holding the final stance (torque on) after the
        # decel ramp — the operator decides what happens next.
        result.update(vx_cmd=round(vx, 3), vy_cmd=round(vy, 3),
                      duration_s=round(total_s, 1),
                      rot60=canon is not None,
                      rot60_k_end=(canon.k if canon is not None
                                   else None))
        if selector is not None:
            result.update(
                turn=turn, turn_switches=selector.switches,
                heading_end_deg=round(
                    math.degrees(selector.heading), 1))

    # Post-episode tail (08-10, dep-tip1 fall debug): the robot tipped
    # AFTER "walk done" — the episode ended holding a ~15° lean and
    # the log stopped with it. Keep READING (never commanding) for a
    # few seconds so a tip-over during the end-of-episode hold (or
    # the collapse after a trip limp) is in the trace.
    TAIL_S = 3.0
    tail_tilt_max = 0.0
    for k in range(int(TAIL_S * 10)):
        time.sleep(0.1)
        try:
            state = est.update()
        except Exception:
            break
        if state is None or not state.bus_ok:
            break
        tail_tilt_max = max(
            tail_tilt_max,
            abs(state.imu_roll - tilt_ref0[0]) * RAD2DEG,
            abs(state.imu_pitch - tilt_ref0[1]) * RAD2DEG)
        elog.tick(t_end + (k + 1) * 0.1, state, None, None, None,
                  0.0, 0.0, max_cur, phase="tail")

    result.update(
        max_current_a=round(max_cur, 2), overruns=overruns,
        tilt_ref_deg=[round(tilt_ref0[0] * RAD2DEG, 2),
                      round(tilt_ref0[1] * RAD2DEG, 2)],
        # Attitude bookkeeping, all relative to the episode tilt ref:
        # the summary alone should answer "did it stay level, and did
        # it go over after the episode?" without opening the CSV.
        tilt_rel_max_deg=round(tilt_rel_max, 1),
        roll_rel_end_deg=round(
            (state.imu_roll - tilt_ref0[0]) * RAD2DEG, 1)
        if state is not None else None,
        pitch_rel_end_deg=round(
            (state.imu_pitch - tilt_ref0[1]) * RAD2DEG, 1)
        if state is not None else None,
        tail_s=TAIL_S,
        tail_tilt_max_deg=round(tail_tilt_max, 1),
        # >35° relative during run or tail ≈ on its side/nose: the
        # 25° walk trip would have fired first in-run, so this is a
        # post-episode fall detector.
        fell=bool(max(tail_tilt_max, tilt_rel_max) > 35.0),
    )
    result["log"] = elog.close(result)
    return result


def run_drive_session(drive, cmd: DriveCommand, *, on_progress=None,
                      abort_check=None, rot60: bool = True,
                      walk_weights: Path | None = None,
                      hold_weights: Path | None = None) -> dict:
    """Blocking persistent drive session (MuJoCo-viewer-style driving).

    Same conventions as run_policy_move mode="walk" — plant-stance
    start gate, 25 Hz, walk obs contract with meas := ref, rot-60
    canonicalizer, 25 deg tilt trip — but the command is LIVE: the
    browser streams body-frame (vx, vy) heartbeats into ``cmd`` while
    arrow keys are held. Refs slew toward the target at the trained
    ramp rate (0 -> full band in WALK_RAMP_S), so a key press feels
    like the training ramp and a release decays to the trained stop.

    Hold model: with ``hold_weights=None`` the walk policy itself
    holds at zero refs (its trained stop). A separate hold policy
    (obs 68 stance at height_ref 0, or another obs-72 walk file at
    zero refs) takes over after DRIVE_HOLD_SWITCH_S of zero command;
    any new command flips back to walk instantly. Every model switch
    re-anchors the episode frame (q_nom := present pose,
    prev_action := 0) — the same episode re-anchor the sim viewer's
    play.py does on policy handoff.

    Ends by: operator stop (decel then HOLD pose), abort (hold),
    heartbeat silence > DRIVE_IDLE_END_S (browser gone -> hold),
    session cap DRIVE_MAX_SESSION_S (hold), or safety trip (limp).
    """
    on_progress = on_progress or (lambda p: None)
    abort_check = abort_check or (lambda: False)
    bus = drive.bus
    if bus is None or drive.dry_run:
        return {"ok": False, "error": "no bus"}

    cfg = load_config(str(_HERE.parent / "rl_move" / "config.yaml"))
    wpath = walk_weights or WALK_WEIGHTS_PATH
    walk_policy = NumpyPolicy(wpath)
    walk_obs = walk_policy.meta.get("obs_dim")
    if walk_obs not in (72, 74):
        return {"ok": False,
                "error": (f"{Path(wpath).name} is not a walk policy "
                          f"(obs {walk_obs} not 72/74)")}
    # obs 74 = phase-clock walk (see run_policy_move): all-heading
    # training, no rot-60/mirror, phase_hz required in export meta.
    phase_hz = 0.0
    if walk_obs == 74:
        if "phase_hz" not in walk_policy.meta:
            return {"ok": False,
                    "error": (f"{Path(wpath).name} is obs-74 but has no "
                              "phase_hz in meta — re-export with "
                              "--extra-meta phase_hz=<trained hz>")}
        phase_hz = float(walk_policy.meta["phase_hz"])
    hold_policy = None
    hold_obs = None
    if hold_weights is not None:
        hold_policy = NumpyPolicy(hold_weights)
        hold_obs = hold_policy.meta.get("obs_dim")
        if hold_obs not in (68, 72, 74):
            return {"ok": False,
                    "error": (f"{Path(hold_weights).name} fits no hold "
                              f"role (obs {hold_obs}, need 68/72/74)")}
        if hold_obs == 74 and "phase_hz" not in hold_policy.meta:
            return {"ok": False,
                    "error": (f"{Path(hold_weights).name} is obs-74 but "
                              "has no phase_hz in meta")}

    canon = (make_walk_canonicalizer(walk_policy, cfg)
             if rot60 and walk_obs == 72 else None)
    hold_canon = (make_walk_canonicalizer(hold_policy, cfg)
                  if rot60 and hold_policy is not None and hold_obs == 72
                  else None)
    if canon is None and walk_obs == 72 and not _ROT60_OK:
        # Without the canonicalizer only the trained forward wedge is
        # safe; a live joystick can't be trusted to stay inside it.
        # (obs-74 phase policies trained all headings — naked is fine.)
        return {"ok": False,
                "error": ("drive session needs the rot-60 canonicalizer "
                          "(rl_move/sim/rot60.py not deployed)")}

    ok, reason, details = preflight(bus, "walk")
    if not ok:
        return {"ok": False, "error": f"preflight: {reason}", **details}

    def limp():
        try:
            bus.enable_all_torque(False)
        except Exception:
            try:
                drive._torque_all(False)
            except Exception:
                pass

    with drive._lock:
        drive.mode = "demo"
        try:
            drive.gait.stop()
        except Exception:
            pass
        if not drive.armed:
            drive._torque_all(True)
            drive.armed = True

    est = RobotStateEstimator(bus, cfg)
    safety = SafetyLayer(cfg)
    safety.max_roll = math.radians(WALK_MAX_TILT_DEG)
    safety.max_pitch = math.radians(WALK_MAX_TILT_DEG)
    write_speed = int(cfg_get(cfg, "bus", "write_speed", default=400))
    write_acc = int(cfg_get(cfg, "bus", "write_acc", default=20))

    state = None
    for _ in range(5):
        state = est.update(want_full_feedback=True)
        time.sleep(DT)
    if state is None or not state.bus_ok:
        limp()
        return {"ok": False, "error": "bus dropped during settle"}
    q_nom = state.joint_position.copy()
    est.set_commanded(q_nom)
    bus.write_all((q_nom * RAD2DEG).tolist(), speed=write_speed,
                  acc=write_acc)
    est.reset_episode_filters()
    for _ in range(3):
        state = est.update()
        time.sleep(DT)
    tilt_ref0 = (state.imu_roll, state.imu_pitch)
    safety.set_nominal(q_nom)
    safety.set_tilt_reference(*tilt_ref0)

    prev_action = np.zeros(N_JOINTS, dtype=float)
    vx_r = vy_r = 0.0
    phase = 0.0     # walk phase clock (obs-74 policies only); like the
                    # sim it starts at 0 and freezes at zero command
    dv_max = WALK_SPEED_MAX * DT / WALK_RAMP_S   # trained ramp rate
    active = "walk"
    zero_since = 0.0            # session time when refs+target went zero
    stopping = None             # reason string once winding down
    overruns = 0
    max_cur = 0.0
    tilt_rel_max = 0.0
    t = 0.0
    i = 0
    t_next = time.monotonic()
    result: dict = {"ok": True, "mode": "drive"}
    elog = _EpisodeLog("drive", obs_dim=int(walk_obs), params={
        "mode": "drive", "hz": HZ,
        "policy": dict(walk_policy.meta),
        "hold_policy": (dict(hold_policy.meta)
                        if hold_policy is not None else None),
        "q_nom_deg": [round(float(q) * RAD2DEG, 2) for q in q_nom],
        "tilt_ref_deg": [round(tilt_ref0[0] * RAD2DEG, 2),
                         round(tilt_ref0[1] * RAD2DEG, 2)],
        "tilt_trip_deg": WALK_MAX_TILT_DEG,
        "preflight": details, "rot60": canon is not None,
    })

    def reanchor():
        """Episode re-anchor on model switch (q frame + prev_action)."""
        nonlocal q_nom, prev_action
        q_nom = state.joint_position.copy()
        safety.set_nominal(q_nom)
        prev_action = np.zeros(N_JOINTS, dtype=float)

    while True:
        if abort_check():
            result.update(ok=False, error="aborted", held_pose=True,
                          ticks=i)
            break
        t = i * DT
        vx_t, vy_t, hb_age, stop_req = cmd.get()
        if stop_req and stopping is None:
            stopping = "stopped"
        if hb_age > DRIVE_IDLE_END_S and stopping is None:
            stopping = "no command from browser — session ended"
        if t > DRIVE_MAX_SESSION_S and stopping is None:
            stopping = f"session cap {DRIVE_MAX_SESSION_S:.0f}s reached"
        if stopping is not None or hb_age > DRIVE_CMD_TIMEOUT_S:
            vx_t = vy_t = 0.0
        # Slew refs toward the target at the trained ramp rate.
        vx_r += max(-dv_max, min(dv_max, vx_t - vx_r))
        vy_r += max(-dv_max, min(dv_max, vy_t - vy_r))
        moving = math.hypot(vx_r, vy_r) > 1e-4 or math.hypot(vx_t, vy_t) > 0
        if stopping is not None and not moving:
            # Graceful end: refs decayed to zero, robot HOLDS the pose.
            result.update(ticks=i, ended=stopping)
            break

        # Hold-model handoff (only when a separate hold policy is set).
        if hold_policy is not None:
            if moving:
                zero_since = t
                if active != "walk":
                    active = "walk"
                    reanchor()
            elif (active == "walk"
                  and t - zero_since >= DRIVE_HOLD_SWITCH_S):
                active = "hold"
                reanchor()

        goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0, height_ref=0.0,
                        unload_leg=None)
        base_obs = build_obs(cfg, state, q_nom, prev_action, goal=goal,
                             tilt_ref=tilt_ref0)
        need_obs = walk_obs if active == "walk" else hold_obs
        if need_obs in (72, 74):
            obs = np.concatenate(
                [base_obs,
                 np.array([vx_r, vy_r, vx_r, vy_r]) / WALK_VEL_SCALE]
            ).astype(np.float32)
            if need_obs == 74:
                obs = np.concatenate(
                    [obs, np.array([math.sin(phase), math.cos(phase)])]
                ).astype(np.float32)
            if active == "walk":
                raw_act, _ = (canon.predict(obs) if canon is not None
                              else (walk_policy.act(obs), None))
            else:
                raw_act, _ = (hold_canon.predict(obs)
                              if hold_canon is not None
                              else (hold_policy.act(obs), None))
        else:   # 68-obs stance hold at height_ref 0
            obs = base_obs
            raw_act = hold_policy.act(obs)
        # Phase clock advance (obs-74 policies): after obs, gated on a
        # live velocity ref — the sim's walk_task gate (s_ref > 1e-3).
        # At zero command the clock freezes, matching the trained hold.
        if walk_obs == 74 and math.hypot(vx_r, vy_r) > 1e-3:
            phase = (phase + 2.0 * math.pi * phase_hz * DT) \
                % (2.0 * math.pi)
        action, bad = safety.validate_action(raw_act, n_act=N_JOINTS)
        if action is None:
            limp()
            result.update(ok=False, error=f"bad action: {bad}", ticks=i)
            break
        q_prop = _CENTER_RAD + action * _HALF_RAD
        q_safe, status = safety.filter(q_prop, state, action=action)
        if status.terminate:
            limp()
            result.update(ok=False, error=f"safety trip: {status.reason}"
                          + (f" ({status.detail})" if status.detail else ""),
                          limped=True, ticks=i)
            break
        est.set_commanded(q_safe)
        bus.write_all((q_safe * RAD2DEG).tolist(), speed=write_speed,
                      acc=write_acc)
        prev_action = action.copy()

        t_next += DT
        lag = time.monotonic() - t_next
        if lag > 0:
            overruns += 1
            t_next = time.monotonic()
        else:
            time.sleep(-lag)
        state = est.update()
        if state.servo_current is not None:
            max_cur = max(max_cur,
                          float(np.max(np.abs(state.servo_current))))
        tilt_rel_max = max(
            tilt_rel_max,
            abs(state.imu_roll - tilt_ref0[0]) * RAD2DEG,
            abs(state.imu_pitch - tilt_ref0[1]) * RAD2DEG)
        # Hold-68 obs would misalign the fixed 72-wide obs columns —
        # blank them for those ticks (walk replay parity is what the
        # offline contract needs).
        elog.tick(t, state, action, q_safe, goal, vx_r, vy_r, max_cur,
                  obs=(obs if len(obs) == 72 else None),
                  phase=("stopping" if stopping else active),
                  rot60_k=(canon.k if canon is not None
                           and active == "walk" else None))
        snap = {
            "t_s": round(t, 1), "model": active,
            "vx_ref": round(vx_r, 3), "vy_ref": round(vy_r, 3),
            "vx_cmd": round(vx_t, 3), "vy_cmd": round(vy_t, 3),
            "roll_deg": round((state.imu_roll - tilt_ref0[0]) * RAD2DEG,
                              1),
            "pitch_deg": round((state.imu_pitch - tilt_ref0[1])
                               * RAD2DEG, 1),
            "max_current_a": round(max_cur, 2),
            "rot60_k": canon.k if canon is not None else None,
            "stopping": stopping, "overruns": overruns,
        }
        cmd.publish(snap)
        if i % 5 == 0:
            on_progress({
                "msg": (f"drive {active} t={t:5.1f}s "
                        f"v=({vx_r * 1000:+.0f},{vy_r * 1000:+.0f})mm/s "
                        f"maxI={max_cur:.2f}A"
                        + (f" · {stopping}" if stopping else "")),
                **snap})
        i += 1
        t = i * DT

    # Same post-episode observation tail as run_policy_move: keep
    # reading (never commanding) so a fall during the end-of-session
    # hold is in the trace.
    TAIL_S = 3.0
    tail_tilt_max = 0.0
    for k in range(int(TAIL_S * 10)):
        time.sleep(0.1)
        try:
            state = est.update()
        except Exception:
            break
        if state is None or not state.bus_ok:
            break
        tail_tilt_max = max(
            tail_tilt_max,
            abs(state.imu_roll - tilt_ref0[0]) * RAD2DEG,
            abs(state.imu_pitch - tilt_ref0[1]) * RAD2DEG)
        elog.tick(t + (k + 1) * 0.1, state, None, None, None,
                  0.0, 0.0, max_cur, phase="tail")

    result.update(
        duration_s=round(t, 1),
        max_current_a=round(max_cur, 2), overruns=overruns,
        tilt_ref_deg=[round(tilt_ref0[0] * RAD2DEG, 2),
                      round(tilt_ref0[1] * RAD2DEG, 2)],
        tilt_rel_max_deg=round(tilt_rel_max, 1),
        tail_s=TAIL_S,
        tail_tilt_max_deg=round(tail_tilt_max, 1),
        fell=bool(max(tail_tilt_max, tilt_rel_max) > 35.0),
    )
    result["log"] = elog.close(result)
    return result
