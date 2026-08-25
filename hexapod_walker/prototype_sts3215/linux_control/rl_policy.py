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
rot-60 / mirror (they train all headings, no wedge). AMP walk files use
obs 93 = obs 74 + yaw-rate command + 18 fault-health entries; hardware
currently feeds an all-healthy fault vector.

- policy loop runs at the selected policy's trained control_hz
  (legacy/no-rate exports = 25 Hz); obs = build_obs(q, qd, tilt-rel-to-start, gyro,
  prev_action(18), goal(9)) with q_nom = the pose read at arm time.
  The runner may stream interpolated servo targets faster between policy
  decisions, but the learned brain still sees exactly the trained
  cadence.
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
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

# rl_move lives one level above linux_control on the robot and in repo.
_HERE = Path(__file__).resolve().parent
for _p in (_HERE.parent, _HERE, _HERE / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import cfg_get, load_config            # noqa: E402
from rl_move.env import TaskGoal, build_obs                # noqa: E402
from rl_move.joint_frame import (                          # noqa: E402
    FRAME_ROBOT_ABS, normalize_joint_frame,
    policy_joint_frame_from_meta, policy_rad_to_robot_abs_rad,
    robot_abs_rad_to_policy_rad,
)
from rl_move.robot_state import (                          # noqa: E402
    DEG2RAD, N_JOINTS, RAD2DEG, RobotState, RobotStateEstimator,
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
# New training uses rl_move/config.yaml control.hz, but playback adapts per
# policy. Legacy exports without explicit rate metadata are 25 Hz brains;
# explicit exports run at meta.control_hz / meta.policy_hz. control.inner_hz
# is only the optional faster servo-stream layer between policy decisions.
LEGACY_POLICY_HZ = 25.0
try:
    _RUNNER_CFG = load_config(str(_HERE.parent / "rl_move" / "config.yaml"))
except Exception:                                          # pragma: no cover
    _RUNNER_CFG = {}
HZ = float(cfg_get(_RUNNER_CFG, "control", "hz",
                   default=LEGACY_POLICY_HZ) or LEGACY_POLICY_HZ)
DT = 1.0 / HZ
MAX_INNER_STEPS = 8


def policy_control_hz_error(meta: dict, name: str = "policy") -> str | None:
    """Compatibility hook: playback now adapts instead of rejecting."""
    _ = (meta, name)
    return None

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


def policy_joint_frame(policy: "NumpyPolicy", cfg: dict | None = None) -> str:
    """Joint frame for policy observations/actions.

    New/default exports use the robot's logical absolute-tibia frame.
    Older MuJoCo-native checkpoints must carry meta["joint_frame"] =
    "model_rel" (or cfg compat.policy_joint_frame=model_rel) so the
    runner converts both observations and bus commands.
    """
    return policy_joint_frame_from_meta(policy.meta, cfg)


def _state_for_policy_frame(state, joint_frame: str):
    """Return a RobotState view in the policy's declared joint frame."""
    frame = normalize_joint_frame(joint_frame)
    if frame == FRAME_ROBOT_ABS:
        return state
    return replace(
        state,
        joint_position=robot_abs_rad_to_policy_rad(
            state.joint_position, frame),
        joint_velocity=robot_abs_rad_to_policy_rad(
            state.joint_velocity, frame),
        commanded_position=robot_abs_rad_to_policy_rad(
            state.commanded_position, frame),
    )


def _finite_float(value, default: float) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float(default)
    return out if math.isfinite(out) else float(default)


def _positive_float(value, default: float) -> float:
    out = _finite_float(value, default)
    return out if out > 0.0 else float(default)


def _clamped_int(value, default: int, lo: int, hi: int) -> int:
    try:
        out = int(round(float(value)))
    except (TypeError, ValueError):
        out = int(default)
    return max(lo, min(hi, out))


@dataclass(frozen=True)
class PolicyTiming:
    """Per-episode timing chosen from the selected policy's contract."""

    policy_hz: float
    policy_dt: float
    trained_control_hz: float
    trained_control_hz_explicit: bool
    runner_config_hz: float

    @property
    def adapted(self) -> bool:
        return abs(self.policy_hz - self.runner_config_hz) > 1e-6


def _policy_control_hz(policy: "NumpyPolicy") -> tuple[float, bool]:
    """Return (trained policy decision Hz, explicit_in_meta).

    Legacy exports did not carry this key. Those are treated as 25 Hz
    because every metadata-free policy deployed before the 100 Hz switch
    was trained under the old contract.
    """
    meta = policy.meta or {}
    for key in ("control_hz", "policy_hz"):
        if key in meta:
            return _positive_float(meta[key], HZ), True
    control = meta.get("control")
    if isinstance(control, dict) and "hz" in control:
        return _positive_float(control["hz"], HZ), True
    return LEGACY_POLICY_HZ, False


def _policy_timing(policy: "NumpyPolicy") -> PolicyTiming:
    trained_hz, explicit = _policy_control_hz(policy)
    policy_hz = _positive_float(trained_hz, LEGACY_POLICY_HZ)
    return PolicyTiming(
        policy_hz=policy_hz,
        policy_dt=1.0 / policy_hz,
        trained_control_hz=trained_hz,
        trained_control_hz_explicit=explicit,
        runner_config_hz=HZ,
    )


def _check_policy_control_hz(policy: "NumpyPolicy", role: str) -> str | None:
    """Compatibility hook: playback now adapts instead of rejecting."""
    _ = role
    _policy_timing(policy)
    return None


def _policy_bus_profile(policy: "NumpyPolicy", cfg: dict) -> tuple[int, int]:
    """Bus profile requested by policy metadata, with config fallback."""
    meta = policy.meta or {}
    speed = meta.get("bus_write_speed",
                     cfg_get(cfg, "bus", "write_speed", default=400))
    acc = meta.get("bus_write_acc",
                   cfg_get(cfg, "bus", "write_acc", default=20))
    return (_clamped_int(speed, 400, 0, 4095),
            _clamped_int(acc, 20, 0, 254))


def _policy_safety_max_delta_q_deg(policy: "NumpyPolicy", cfg: dict,
                                   policy_hz: float) -> tuple[float, bool]:
    """Policy-tick joint slew, preserving old policies under new config.

    A metadata value is a trained per-policy-tick cap and wins. Without
    metadata, convert the config's per-tick cap at config control.hz into
    the same deg/s envelope at the selected policy cadence; this maps the
    current 100 Hz 0.375 deg/tick config back to 1.5 deg/tick for 25 Hz
    legacy policies.
    """
    cfg_dq = _positive_float(
        cfg_get(cfg, "safety", "max_delta_q_deg", default=1.5), 1.5)
    cfg_hz = _positive_float(
        cfg_get(cfg, "control", "hz", default=HZ), HZ)
    fallback = cfg_dq * cfg_hz / _positive_float(
        policy_hz, LEGACY_POLICY_HZ)
    meta = policy.meta or {}
    for key in ("safety_max_delta_q_deg", "max_delta_q_deg"):
        if key in meta:
            return _positive_float(meta[key], fallback), True
    safety = meta.get("safety")
    if isinstance(safety, dict) and "max_delta_q_deg" in safety:
        return _positive_float(safety["max_delta_q_deg"], fallback), True
    return fallback, False


def _apply_policy_safety_timing(safety: SafetyLayer, policy: "NumpyPolicy",
                                cfg: dict, timing: PolicyTiming
                                ) -> tuple[float, bool]:
    max_dq_deg, explicit = _policy_safety_max_delta_q_deg(
        policy, cfg, timing.policy_hz)
    safety.max_dq = math.radians(max_dq_deg)
    safety._hz = timing.policy_hz
    trip_s = _positive_float(
        cfg_get(cfg, "safety", "over_current_trip_s", default=0.8), 0.8)
    safety._over_current_trip_ticks = max(
        1, int(round(trip_s * timing.policy_hz)))
    return max_dq_deg, explicit


def _inner_stream_plan(policy: "NumpyPolicy", cfg: dict,
                       policy_hz: float | None = None
                       ) -> tuple[int, float, float]:
    """How to split one policy target into servo-stream substeps."""
    base_hz = _positive_float(policy_hz if policy_hz is not None else HZ, HZ)
    base_dt = 1.0 / base_hz
    meta = policy.meta or {}
    raw_hz = None
    for key in ("inner_stream_hz", "servo_inner_hz", "inner_hz"):
        if key in meta:
            raw_hz = meta[key]
            break
    if raw_hz is None:
        raw_hz = cfg_get(cfg, "control", "inner_hz", default=base_hz)
    requested_hz = _positive_float(raw_hz, base_hz)
    if requested_hz <= base_hz:
        return 1, base_hz, base_dt
    steps = max(1, min(MAX_INNER_STEPS, int(round(requested_hz / base_hz))))
    actual_hz = base_hz * steps
    return steps, actual_hz, 1.0 / actual_hz


def _stream_state_is_stale(state) -> bool:
    timing = getattr(state, "timing", {}) or {}
    return bool(timing.get("stale_feedback"))


def _stale_stream_state(last_good_state, stale_ticks: int, q_cmd=None):
    """Return a last-known-good RobotState marked as stale feedback."""
    if last_good_state is None:
        return None
    timing = dict(getattr(last_good_state, "timing", {}) or {})
    timing.update({
        "stale_feedback": True,
        "stale_ticks": int(stale_ticks),
    })
    kwargs = {
        "timestamp": time.monotonic(),
        "bus_ok": True,
        "timing": timing,
    }
    if q_cmd is not None and hasattr(last_good_state, "commanded_position"):
        kwargs["commanded_position"] = np.asarray(
            q_cmd, dtype=float).copy()
    return replace(last_good_state, **kwargs)


def _stream_target(bus, est: RobotStateEstimator,
                   q_from_robot: np.ndarray, q_to_robot: np.ndarray, *,
                   t_next: float, inner_steps: int, inner_dt: float,
                   write_speed: int, write_acc: int,
                   abort_check, last_good_state=None,
                   stale_ticks: int = 0,
                   max_stale_ticks: int = 0
                   ) -> tuple[object | None, float, int, str, int, int]:
    """Write interpolated servo targets up to q_to_robot.

    The caller still runs the policy once per selected policy tick. This
    helper only smooths the command sent to the MCU/bus and returns the
    latest sampled robot state at the end of that policy tick.
    """
    state_robot = last_good_state
    overruns = 0
    stale_samples = 0
    stale_ticks = max(0, int(stale_ticks))
    max_stale_ticks = max(0, int(max_stale_ticks))
    q_from = np.asarray(q_from_robot, dtype=float)
    q_to = np.asarray(q_to_robot, dtype=float)
    steps = max(1, int(inner_steps))
    step_all = getattr(bus, "step_all", None)
    can_step_all = callable(step_all)
    stream_firmware = bool(getattr(bus, "has_stream", False))
    for sub in range(1, steps + 1):
        if abort_check():
            return state_robot, t_next, overruns, "aborted", stale_ticks, stale_samples
        alpha = sub / steps
        q_cmd = q_to if sub == steps else q_from + (q_to - q_from) * alpha
        est.set_commanded(q_cmd)
        q_cmd_deg = (q_cmd * RAD2DEG).tolist()
        state_robot = None
        step_all_attempted = False
        if can_step_all:
            step_all_attempted = True
            try:
                snap = step_all(q_cmd_deg, speed=write_speed,
                                acc=write_acc)
            except Exception:
                snap = None
            if snap is not None:
                update_from_snapshot = getattr(est, "update_from_snapshot",
                                               None)
                if callable(update_from_snapshot):
                    state_robot = update_from_snapshot(snap)
                else:
                    state_robot = est.update()
            elif not stream_firmware:
                step_all_attempted = False
        if not step_all_attempted:
            bus.write_all(q_cmd_deg, speed=write_speed, acc=write_acc)

        t_next += inner_dt
        lag = time.monotonic() - t_next
        if lag > 0:
            overruns += 1
            t_next = time.monotonic()
        else:
            time.sleep(-lag)

        if state_robot is None and not step_all_attempted:
            state_robot = est.update()
        if state_robot is None or not state_robot.bus_ok:
            stale_ticks += 1
            stale_samples += 1
            if last_good_state is not None:
                state_robot = _stale_stream_state(
                    last_good_state, stale_ticks, q_cmd=q_cmd)
            if (last_good_state is not None
                    and stale_ticks <= max_stale_ticks):
                continue
            return (state_robot, t_next, overruns,
                    "feedback stale during stream", stale_ticks,
                    stale_samples)
        last_good_state = state_robot
        stale_ticks = 0
    return state_robot, t_next, overruns, "", stale_ticks, stale_samples



def _set_weight_bearing_torque(bus) -> None:
    """Best-effort full torque limit + torque-enable for RL body support."""
    if bus is None:
        return
    try:
        from feetech_bus import joint_to_servo_id as _sid_for_joint
    except Exception:  # pragma: no cover - deployed buses use feetech_bus
        _sid_for_joint = lambda j: j + 2
    pkt = getattr(bus, "pkt", None)
    if pkt is not None:
        for joint in range(N_JOINTS):
            try:
                pkt.write2ByteTxRx(
                    _sid_for_joint(joint), ADDR_TORQUE_LIMIT,
                    RL_HOLD_TORQUE_LIMIT)
            except Exception:
                pass
    try:
        bus.enable_all_torque(True)
    except Exception:
        pass


PREFLIGHT_MAX_TILT_DEG = 12.0
# Start-pose gates (max per-joint |delta| from the expected pose).
STAND_START_TOL_DEG = 30.0   # near flat belly pose (logical zero-ish)
LOWER_START_TOL_DEG = 25.0   # near the sim-default walk-ready stance

# Walk mode (ppo_goal_cw_dep_vref1_r1, obs 72) — the deployment-contract
# champion: trained with goal.walk_obs_body_vel=2, i.e. vx/vy_meas := ref
# IS the training contract, bit-identical to what this runner feeds
# (verdict PASS 08-10, no erosion vs parent). Still an operator-supervised
# experiment, tightly bounded:
# - starts from the sim-default walk-ready stance. Stand Up owns the
#   STEP -> walk-start settle; Start Driving never hides it.
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
WALK_YAW_SCALE = 0.5         # sim walk_task.WZ_SCALE
WALK_OBS_DIMS = (72, 74, 93)
WALK_PHASE_OBS_DIMS = (74, 93)
WALK_SPEED_MIN = 0.05        # deployed dep-vref walk policies are OOD below
                             # this; the UI may select slower values for sims,
                             # but hardware live-drive clamps to the trained band.
WALK_SPEED_MAX = 0.06        # trained command band is 0.05-0.06 m/s
WALK_HOLD_S = 1.0
WALK_RAMP_S = 1.0
WALK_MAX_TOTAL_S = 20.0
WALK_START_TOL_DEG = 25.0    # near the sim-default walk-ready stance
WALK_STEP_START_TOL_DEG = 35.0  # explicit compatibility hook only
DRIVE_HOLD_REFRESH_S = 0.25     # low-rate active refresh for joint-hold
RL_HOLD_TORQUE_LIMIT = 1000     # weight-bearing hold torque limit
ADDR_TORQUE_LIMIT = 48          # STS3215 SRAM max torque/current register
DRIVE_START_REFRESH_S = 0.45    # re-hold sim walk start through drive arming
DRIVE_START_REFRESH_SPEED = 260
DRIVE_START_REFRESH_ACC = 35
DRIVE_START_DRIFT_TOL_DEG = 8.0  # refuse instead of latching a sagged pose
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
# the live (vx, vy); release -> (0, 0) -> the hold path. The loop NEVER
# trusts a stale command: refs decay to zero unless a heartbeat younger
# than DRIVE_CMD_TIMEOUT_S says otherwise, so a closed tab / dropped
# WiFi degrades to "stand still and hold", not "keep walking". With no
# learned hold policy installed, "hold" is a direct joint hold of the last
# safe commanded pose. It must not run the walk policy at zero command:
# several good walking policies were never trained to be still at vx=vy=0.
DRIVE_CMD_TIMEOUT_S = 0.6    # heartbeats at ~5 Hz; 3 misses = stop
DRIVE_IDLE_END_S = 120.0     # no heartbeat at all -> end session (hold)
DRIVE_MAX_SESSION_S = 300.0  # hard cap per session (decel + hold)
DRIVE_HOLD_SWITCH_S = 1.5    # zero-cmd dwell before flipping to the
                             # hold model (quick taps stay on walk)
DRIVE_WALK_ENGAGE_S = 0.35   # require a real held direction before gait
DRIVE_WALK_ACTION_RAMP_S = 1.5  # blend first learned targets from stance
DRIVE_STREAM_STALE_TICKS = 10  # tolerate ~100 ms snapshot gaps at 100 Hz
DRIVE_MOVE_EPS_MPS = 1e-4
DRIVE_YAW_EPS_RAD_S = 1e-4


def _drive_clamp_translation(vx: float, vy: float) -> tuple[float, float]:
    """Clamp nonzero hardware-drive translation into the trained band."""
    spd = math.hypot(vx, vy)
    if spd <= DRIVE_MOVE_EPS_MPS:
        return 0.0, 0.0
    want = max(WALK_SPEED_MIN, min(WALK_SPEED_MAX, spd))
    s = want / spd
    return vx * s, vy * s


def _drive_command_is_moving(vx_target: float, vy_target: float,
                             wz_target: float, walk_obs: int = 72) -> bool:
    """Whether the live drive session should engage the walk policy."""
    if math.hypot(vx_target, vy_target) > DRIVE_MOVE_EPS_MPS:
        return True
    # Only AMP/yaw-command policies understand yaw-only drive. Legacy obs-72
    # policies ignore wz, so yaw-only at zero translation must stay in hold.
    return walk_obs == 93 and abs(wz_target) > DRIVE_YAW_EPS_RAD_S


def _drive_uses_learned_policy(active: str, hold_policy) -> bool:
    """False for the built-in neutral joint hold fallback."""
    return active == "walk" or hold_policy is not None


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
        self._wz = 0.0
        # Counts as a heartbeat so the idle-end clock starts at session
        # birth instead of firing instantly (refs are zero until the
        # browser actually sends commands).
        self._t_cmd = time.monotonic()
        self._stop = False
        self._live: dict = {}

    def set(self, vx: float, vy: float, wz: float = 0.0) -> None:
        spd = math.hypot(vx, vy)
        if spd > WALK_SPEED_MAX:
            s = WALK_SPEED_MAX / spd
            vx, vy = vx * s, vy * s
        wz = max(-WALK_YAW_SCALE, min(WALK_YAW_SCALE, float(wz)))
        with self._lock:
            self._vx, self._vy, self._wz = float(vx), float(vy), float(wz)
            self._t_cmd = time.monotonic()

    def request_stop(self) -> None:
        with self._lock:
            self._stop = True

    def get(self) -> tuple[float, float, float, float, bool]:
        """(vx, vy, wz, seconds_since_heartbeat, stop_requested)."""
        with self._lock:
            return (self._vx, self._vy, self._wz,
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


def _walk_obs_tail(walk_obs: int, vx_r: float, vy_r: float, phase: float,
                   wz_r: float = 0.0) -> np.ndarray:
    """The deploy-side tail for sim walk observations."""
    tail = [vx_r / WALK_VEL_SCALE, vy_r / WALK_VEL_SCALE,
            vx_r / WALK_VEL_SCALE, vy_r / WALK_VEL_SCALE]
    if walk_obs in WALK_PHASE_OBS_DIMS:
        tail.extend([math.sin(phase), math.cos(phase)])
    if walk_obs == 93:
        tail.append(wz_r / WALK_YAW_SCALE)
        tail.extend([1.0] * N_JOINTS)
    return np.asarray(tail, dtype=np.float32)


def _walk_phase_runs(walk_obs: int, vx_r: float, vy_r: float,
                     wz_r: float = 0.0, *, phase_run_on_yaw: bool = False
                     ) -> bool:
    if walk_obs not in WALK_PHASE_OBS_DIMS:
        return False
    if math.hypot(vx_r, vy_r) > 1e-3:
        return True
    return phase_run_on_yaw and abs(wz_r) > 1e-3


def _json_safe(value):
    """Best-effort JSON conversion for numpy-heavy live-run diagnostics."""
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def _state_debug(state, *, q_cmd_rad=None, target_robot=None) -> dict:
    if state is None:
        return {"state": None}
    timing = dict(getattr(state, "timing", {}) or {})
    q = np.asarray(state.joint_position, dtype=float)
    out = {
        "bus_ok": bool(getattr(state, "bus_ok", False)),
        "imu_ok": bool(getattr(state, "imu_ok", False)),
        "roll_deg": round(float(state.imu_roll) * RAD2DEG, 2),
        "pitch_deg": round(float(state.imu_pitch) * RAD2DEG, 2),
        "gyro_dps": [round(float(x) * RAD2DEG, 2)
                     for x in np.asarray(state.imu_gyro, dtype=float)],
        "q_deg": [round(float(x) * RAD2DEG, 2) for x in q],
        "stale_feedback": bool(timing.get("stale_feedback")),
        "stale_ticks": timing.get("stale_ticks"),
        "fallback": timing.get("fallback"),
        "timing_ms": {
            "pos": round(float(timing.get("t_pos") or 0.0) * 1000.0, 2),
            "imu": round(float(timing.get("t_imu") or 0.0) * 1000.0, 2),
            "fb": round(float(timing.get("t_fb") or 0.0) * 1000.0, 2),
            "total": round(float(timing.get("t_total") or 0.0)
                           * 1000.0, 2),
            "full_feedback": bool(timing.get("full_feedback")),
        },
    }
    if q_cmd_rad is not None:
        cmd = np.asarray(q_cmd_rad, dtype=float)
        dq = np.abs(q - cmd) * RAD2DEG
        j = int(np.argmax(dq)) if len(dq) else 0
        out["cmd_err_max_deg"] = round(float(dq[j]) if len(dq) else 0.0, 2)
        out["cmd_err_joint"] = j
    if target_robot is not None:
        target = np.asarray(target_robot, dtype=float)
        dq = np.abs(q - target) * RAD2DEG
        j = int(np.argmax(dq)) if len(dq) else 0
        out["target_err_max_deg"] = round(float(dq[j]) if len(dq) else 0.0, 2)
        out["target_err_joint"] = j
    cur = getattr(state, "servo_current", None)
    if cur is not None:
        cur_arr = np.asarray(cur, dtype=float)
        j = int(np.argmax(np.abs(cur_arr))) if len(cur_arr) else 0
        out["current_peak_a"] = round(float(abs(cur_arr[j])), 3)
        out["current_peak_joint"] = j
    return out


class _RunDebug:
    """JSONL flight recorder for live RL attempts, including early failures."""

    def __init__(self, mode: str, context: dict | None = None):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        d = _HERE / "logs"
        d.mkdir(exist_ok=True)
        self.mode = mode
        self.path = d / f"rl_{mode}_{stamp}_debug.jsonl"
        self.name = self.path.name
        self._t0 = time.monotonic()
        self._f = self.path.open("w")
        self._closed = False
        self.event("debug_start", context=context or {})

    def event(self, name: str, **data) -> None:
        if self._closed:
            return
        rec = {
            "event": name,
            "t_s": round(time.monotonic() - self._t0, 4),
            "mono": round(time.monotonic(), 6),
            "mode": self.mode,
        }
        rec.update(_json_safe(data))
        try:
            self._f.write(json.dumps(rec, separators=(",", ":")) + "\n")
            self._f.flush()
        except Exception:
            pass
        try:
            from event_log import emit
            emit("rl_debug", f"{self.mode}: {name}", src="rl_policy",
                 data={"debug_log": self.name, **_json_safe(data)})
        except Exception:
            pass

    def attach(self, result: dict) -> dict:
        result.setdefault("debug_log", self.name)
        return result

    def close(self, result: dict | None = None) -> None:
        if self._closed:
            return
        if result is not None:
            self.event("debug_end", result=result)
        self._closed = True
        try:
            self._f.close()
        except Exception:
            pass


def _finish_debug(debug: _RunDebug | None, result: dict) -> dict:
    if debug is not None:
        debug.attach(result)
        debug.close(result)
    return result


class _EpisodeLog:
    """Every RL episode leaves a full local trace in ``logs/``.

    ``rl_<mode>_<stamp>.csv``  — one row per 25 Hz tick: attitude, gyro,
    goal refs, measured q (18), commanded q (18), raw action (18), and
    per-servo current when full feedback is available.
    ``rl_<mode>_<stamp>_summary.json`` — params + final result.
    Start/end also land in events.jsonl (kind ``rl_episode``).
    Pull with receive_robot_logs.py / scp for offline analysis.
    """

    def __init__(self, mode: str, params: dict, obs_dim: int = 0,
                 debug: _RunDebug | None = None):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        d = _HERE / "logs"
        d.mkdir(exist_ok=True)
        self.mode = mode
        self.params = params
        self.obs_dim = int(obs_dim)
        self.debug = debug
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
            + ["rot60_k", "mirror",
               "bus_ok", "imu_ok", "stale_feedback", "stale_ticks",
               "t_pos_ms", "t_imu_ms", "t_fb_ms", "t_total_ms",
               "q_cmd_err_max_deg", "q_cmd_err_joint", "action_abs_max"])
        try:
            from event_log import emit
            emit("rl_episode", f"{mode} started ({self.csv_path.name})",
                 src="rl_policy", data=params)
        except Exception:
            pass
        if self.debug is not None:
            self.debug.event("episode_csv_started",
                             csv=self.csv_path.name,
                             summary=self.sum_path.name,
                             params=params)

    def tick(self, t: float, state, action, q_cmd_rad, goal,
             vx_r: float, vy_r: float, max_cur: float,
             obs=None, phase: str = "run", rot60_k=None,
             mirror_on=None) -> None:
        cur = (state.servo_current.tolist()
               if state.servo_current is not None else [None] * N_JOINTS)
        timing = dict(getattr(state, "timing", {}) or {})
        obs_cols = ([round(float(o), 4) for o in obs]
                    if obs is not None else [""] * self.obs_dim)
        if q_cmd_rad is not None:
            q_err = np.abs(np.asarray(state.joint_position, dtype=float)
                           - np.asarray(q_cmd_rad, dtype=float)) * RAD2DEG
            q_err_j = int(np.argmax(q_err)) if len(q_err) else 0
            q_err_max = round(float(q_err[q_err_j]) if len(q_err) else 0.0, 2)
        else:
            q_err_j = ""
            q_err_max = ""
        act_abs = (round(float(np.max(np.abs(action))), 4)
                   if action is not None else "")
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
               "" if mirror_on is None else int(mirror_on),
               int(bool(getattr(state, "bus_ok", False))),
               int(bool(getattr(state, "imu_ok", False))),
               int(bool(timing.get("stale_feedback"))),
               timing.get("stale_ticks", ""),
               round(float(timing.get("t_pos") or 0.0) * 1000.0, 3),
               round(float(timing.get("t_imu") or 0.0) * 1000.0, 3),
               round(float(timing.get("t_fb") or 0.0) * 1000.0, 3),
               round(float(timing.get("t_total") or 0.0) * 1000.0, 3),
               q_err_max, q_err_j, act_abs])
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
                 "debug_log": self.debug.name if self.debug else None,
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
    vals: list[float | None] = [None] * N_JOINTS
    errors: list[str] = []
    try:
        pos = bus.read_all_positions()
    except Exception as e:
        pos = None
        errors.append(str(e))
    if isinstance(pos, dict):
        for j, v in pos.items():
            jj = int(j)
            if 0 <= jj < N_JOINTS:
                vals[jj] = float(v)
    for j in range(N_JOINTS):
        if vals[j] is not None:
            continue
        try:
            v = bus.read_position_deg(j)
        except Exception as e:
            errors.append(f"j{j}: {e}")
            v = None
        if v is not None:
            vals[j] = float(v)
    missing = [j for j, v in enumerate(vals) if v is None]
    if missing:
        suffix = f" ({'; '.join(errors[:3])})" if errors else ""
        return None, f"servo IDs not answering: joints {missing}{suffix}"
    return np.array([float(v) for v in vals], dtype=float), ""


def _step_stand_final_deg() -> np.ndarray | None:
    try:
        data = json.loads((_HERE / "standup_modes.json").read_text())
        q = data["modes"]["step"]["keyframes"][-1]["q_deg"]
        if len(q) != N_JOINTS:
            return None
        return np.asarray(q, dtype=float)
    except Exception:
        return None


def _expected_start_options_deg(
        mode: str, *, allow_step_stand: bool = False
        ) -> tuple[list[tuple[str, np.ndarray, float]] | None, str]:
    if mode == "stand":
        # Rise training starts belly-down at logical zero (legs straight
        # out). Partial curls were also trained, so the gate is loose.
        return [("logical_zero", np.zeros(N_JOINTS),
                 STAND_START_TOL_DEG)], ""
    # Lower and normal walk start from the simulator's normal walk reset pose,
    # not from plant_pose.json. The saved plant is a calibration artifact;
    # letting it redefine q_nom made the hardware drive into a too-low stance.
    options: list[tuple[str, np.ndarray, float]] = []
    try:
        from rl_walk_start import walk_start_pose_degrees
        options.append(("sim_walk_start",
                        np.asarray(walk_start_pose_degrees(), dtype=float),
                        WALK_START_TOL_DEG if mode == "walk"
                        else LOWER_START_TOL_DEG))
    except Exception as e:  # pragma: no cover
        return None, f"sim walk start unavailable: {e}"
    if mode == "walk" and allow_step_stand:
        step = _step_stand_final_deg()
        if step is not None:
            options.append(("step_stand", step, WALK_STEP_START_TOL_DEG))
    return options, ""


def preflight(bus, mode: str, *, allow_step_stand: bool = False
              ) -> tuple[bool, str, dict]:
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
    options, err = _expected_start_options_deg(
        mode, allow_step_stand=allow_step_stand)
    if options is None:
        return False, err, {}
    checks = []
    for name, exp, tol_i in options:
        dq_i = np.abs(q_deg - exp)
        checks.append((float(np.max(dq_i)), int(np.argmax(dq_i)),
                       name, float(tol_i), dq_i))
    best_delta, best_joint, best_name, best_tol, best_dq = min(
        checks, key=lambda x: x[0])
    details = {
        "roll_deg": round(roll, 1), "pitch_deg": round(pitch, 1),
        "max_pose_delta_deg": round(best_delta, 1),
        "pose_tol_deg": best_tol,
        "start_pose": best_name,
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
    if best_delta > best_tol:
        worst = best_joint
        want = ("belly-down, legs straight out (logical zero)"
                if mode == "stand" else
                "an upright sim walk start (or STEP stand)"
                if allow_step_stand and mode == "walk"
                else "the sim walk-ready start")
        return False, (f"pose is not {want}: joint {worst} is "
                       f"{best_dq[worst]:.0f} deg from expected "
                       f"(tol {best_tol:.0f})"
                       ), details
    return True, "", details


def _preflight_start_target_deg(
        mode: str, details: dict, *,
        allow_step_stand: bool = False
        ) -> tuple[np.ndarray | None, str]:
    """Return the exact pose that the accepted preflight matched."""
    start_pose = str(details.get("start_pose") or "")
    options, err = _expected_start_options_deg(
        mode, allow_step_stand=allow_step_stand)
    if options is None:
        return None, err
    for name, expected, _tol in options:
        if name == start_pose:
            return expected.copy(), ""
    return None, f"preflight start pose {start_pose!r} no longer available"


def _max_pose_delta_deg(q_robot_rad: np.ndarray,
                        target_robot_rad: np.ndarray
                        ) -> tuple[float, int]:
    dq = np.abs((np.asarray(q_robot_rad, dtype=float)
                 - np.asarray(target_robot_rad, dtype=float)) * RAD2DEG)
    worst = int(np.argmax(dq)) if len(dq) else 0
    return float(dq[worst]) if len(dq) else 0.0, worst


def _direct_start_state(bus, target_robot: np.ndarray,
                        refresh: dict) -> RobotState | None:
    """Build a start RobotState from direct reads when stream snapshots stall."""
    q_deg, err = _read_q_deg(bus)
    if q_deg is None:
        refresh["fallback_error"] = err
        return None
    try:
        imu = bus.read_imu(apply_calib=True)
    except Exception as e:
        imu = None
        refresh["fallback_imu_error"] = str(e)
    imu_ok = isinstance(imu, dict) and "ax_g" in imu
    if imu_ok:
        ax = float(imu.get("ax_g", 0.0))
        ay = float(imu.get("ay_g", 0.0))
        az = float(imu.get("az_g", 0.0))
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.hypot(ay, az))
        gyro = np.array([
            float(imu.get("gx_dps", 0.0)) * DEG2RAD,
            float(imu.get("gy_dps", 0.0)) * DEG2RAD,
            float(imu.get("gz_dps", 0.0)) * DEG2RAD,
        ], dtype=float)
        accel = np.array([ax * 9.80665, ay * 9.80665, az * 9.80665],
                         dtype=float)
    else:
        roll = pitch = 0.0
        gyro = np.zeros(3, dtype=float)
        accel = np.zeros(3, dtype=float)
    refresh["fallback"] = "direct_position_read"
    return RobotState(
        timestamp=time.monotonic(),
        joint_position=np.asarray(q_deg, dtype=float) * DEG2RAD,
        joint_velocity=np.zeros(N_JOINTS, dtype=float),
        imu_roll=float(roll),
        imu_pitch=float(pitch),
        imu_yaw=0.0,
        imu_gyro=gyro,
        imu_accel=accel,
        commanded_position=np.asarray(target_robot, dtype=float).copy(),
        bus_ok=True,
        imu_ok=bool(imu_ok),
        dt=0.0,
        timing={"fallback": "direct_position_read",
                "stale_feedback": True},
    )


def _refresh_verified_start_pose(
        bus, est: RobotStateEstimator, target_deg: np.ndarray, *,
        timing, write_speed: int, write_acc: int, abort_check,
        label: str, debug: _RunDebug | None = None
        ) -> tuple[object | None, dict, str]:
    """Actively re-hold the accepted start pose and refuse if it sags away."""
    target_deg = np.asarray(target_deg, dtype=float)
    target_robot = target_deg * DEG2RAD
    refresh = {
        "pose": label,
        "target_deg": [round(float(x), 2) for x in target_deg],
    }
    speed = min(int(write_speed), DRIVE_START_REFRESH_SPEED)
    acc = min(int(write_acc), DRIVE_START_REFRESH_ACC)
    state_robot = None
    if debug is not None:
        debug.event("start_refresh_begin", pose=label, speed=speed,
                    acc=acc, target_deg=target_deg.tolist(),
                    duration_s=DRIVE_START_REFRESH_S)
    try:
        est.set_commanded(target_robot)
        bus.write_all(target_deg.tolist(), speed=speed, acc=acc)
    except Exception as e:
        refresh["write_error"] = str(e)
        if debug is not None:
            debug.event("start_refresh_write_failed", error=str(e),
                        refresh=refresh)
        return None, refresh, f"start pose refresh failed: {e}"

    deadline = time.monotonic() + DRIVE_START_REFRESH_S
    snapshot_samples = 0
    stale_samples = 0
    while time.monotonic() < deadline:
        if abort_check():
            return state_robot, refresh, "aborted"
        time.sleep(min(float(timing.policy_dt),
                       max(0.0, deadline - time.monotonic())))
        try:
            sampled = est.update(want_full_feedback=False)
        except Exception:
            sampled = None
        if sampled is not None and sampled.bus_ok:
            state_robot = sampled
            snapshot_samples += 1
        else:
            stale_samples += 1

    try:
        sampled = est.update(want_full_feedback=True)
    except Exception:
        sampled = None
    if sampled is not None and sampled.bus_ok:
        state_robot = sampled
        snapshot_samples += 1
    else:
        stale_samples += 1
    refresh.update({
        "snapshot_samples": snapshot_samples,
        "stale_samples": stale_samples,
    })
    if state_robot is None or not state_robot.bus_ok:
        state_robot = _direct_start_state(bus, target_robot, refresh)
        if debug is not None:
            debug.event("start_refresh_fallback",
                        refresh=refresh,
                        state=_state_debug(state_robot,
                                           target_robot=target_robot))
    if state_robot is None or not state_robot.bus_ok:
        if debug is not None:
            debug.event("start_refresh_failed", refresh=refresh)
        return (state_robot, refresh,
                "feedback unavailable during start refresh")

    delta, joint = _max_pose_delta_deg(state_robot.joint_position,
                                       target_robot)
    refresh.update({
        "max_pose_delta_deg": round(delta, 1),
        "worst_joint": joint,
        "tol_deg": DRIVE_START_DRIFT_TOL_DEG,
        "speed": speed,
        "acc": acc,
    })
    if delta > DRIVE_START_DRIFT_TOL_DEG:
        # Leave the servos actively commanded to the plant. The bug we are
        # preventing is treating the sagged feedback pose as the new nominal.
        try:
            est.set_commanded(target_robot)
            bus.write_all(target_deg.tolist(), speed=speed, acc=acc)
        except Exception:
            pass
        if debug is not None:
            debug.event("start_refresh_drift_failed",
                        refresh=refresh,
                        state=_state_debug(state_robot,
                                           target_robot=target_robot))
        return (
            state_robot, refresh,
            f"start pose drifted {delta:.1f} deg from {label} "
            f"on joint {joint} during drive arming")
    if debug is not None:
        debug.event("start_refresh_ok", refresh=refresh,
                    state=_state_debug(state_robot,
                                       target_robot=target_robot))
    return state_robot, refresh, ""


def run_policy_move(drive, mode: str, *, on_progress=None,
                    abort_check=None, vx: float = 0.03, vy: float = 0.0,
                    duration_s: float = 6.0, rot60: bool = True,
                    turn: str | None = None,
                    weights_path: Path | None = None,
                    tilt_trip_deg: float | None = None,
                    extra_hold_s: float = 0.0,
                    allow_step_stand_start: bool = False) -> dict:
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
        if walk_obs not in WALK_OBS_DIMS:
            return {"ok": False,
                    "error": (f"{Path(wpath).name} is not a walk policy "
                              f"(obs {walk_obs} not 72/74/93)")}
        # obs 74 = walk + phase clock (cw-arch-noslipphase1 no-slip
        # line): the runner appends [sin, cos] of a clock that advances
        # at meta["phase_hz"] while a velocity is commanded — the exact
        # contract of the sim's goal.walk_phase_obs=1. That line trains
        # ALL headings (no wedge) and has no rot-60/mirror machinery,
        # so it always runs naked. phase_hz MUST come from the export
        # meta: the sim default (1.0 Hz) is NOT this line's clock
        # (0.1666667 Hz) and a wrong clock is a silently broken gait.
        phase_hz = 0.0
        if walk_obs in WALK_PHASE_OBS_DIMS:
            if "phase_hz" not in policy.meta:
                return {"ok": False,
                        "error": (f"{Path(wpath).name} is obs-{walk_obs} "
                                  "but has "
                                  "no phase_hz in meta — re-export with "
                                  "--extra-meta phase_hz=<trained hz>")}
            phase_hz = float(policy.meta["phase_hz"])
            if turn is not None and walk_obs != 93:
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
        if turn is not None and walk_obs == 93 and turn == "hold":
            return {"ok": False,
                    "error": "turn=hold is not supported for obs-93 yet"}
        if turn is not None and walk_obs != 93:
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
    hz_err = policy_control_hz_error(policy.meta, Path(wpath).name)
    if hz_err:
        return {"ok": False, "error": hz_err}
    try:
        joint_frame = policy_joint_frame(policy, cfg)
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    timing = _policy_timing(policy)
    write_speed, write_acc = _policy_bus_profile(policy, cfg)
    inner_steps, inner_hz, inner_dt = _inner_stream_plan(
        policy, cfg, timing.policy_hz)
    debug = _RunDebug(mode, {
        "policy_path": str(wpath),
        "policy_name": policy.meta.get("name"),
        "obs_dim": policy.meta.get("obs_dim"),
        "joint_frame": joint_frame,
        "timing": {
            "policy_hz": timing.policy_hz,
            "trained_control_hz": timing.trained_control_hz,
            "runner_config_hz": timing.runner_config_hz,
            "adapted": timing.adapted,
            "inner_hz": inner_hz,
            "inner_steps": inner_steps,
        },
        "write_speed": write_speed,
        "write_acc": write_acc,
        "command": {"vx": vx, "vy": vy, "duration_s": duration_s,
                    "turn": turn, "rot60": rot60},
    })

    ok, reason, details = preflight(
        bus, mode,
        allow_step_stand=bool(allow_step_stand_start and mode == "walk"))
    debug.event("preflight", ok=ok, reason=reason, details=details)
    if not ok:
        return _finish_debug(
            debug, {"ok": False, "error": f"preflight: {reason}", **details})
    start_target_deg = None
    if mode == "walk":
        start_target_deg, start_err = _preflight_start_target_deg(
            mode, details,
            allow_step_stand=bool(allow_step_stand_start and mode == "walk"))
        if start_target_deg is None:
            return _finish_debug(
                debug, {"ok": False, "error": f"preflight: {start_err}",
                        **details})
        debug.event("start_target_selected",
                    start_pose=details.get("start_pose"),
                    target_deg=start_target_deg.tolist())

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
        # The cached DriveController.armed flag can survive a UI stop or
        # controller restart while the servo torque state is actually off.
        # Force a fresh torque-enable before RL takes ownership; otherwise
        # the runner can log "hold" while the body physically sags.
        _set_weight_bearing_torque(bus)
        drive._torque_all(True)
        drive.armed = True
        drive.status = "rl policy armed"

    est = RobotStateEstimator(bus, cfg)
    safety = SafetyLayer(cfg)
    max_dq_deg, max_dq_explicit = _apply_policy_safety_timing(
        safety, policy, cfg, timing)
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

    # Walk policies expect the simulator's normal walk-start frame. Do not let
    # a sag during arming become the new nominal pose.
    if mode == "walk" and start_target_deg is not None:
        state_robot, refresh, start_err = _refresh_verified_start_pose(
            bus, est, start_target_deg, timing=timing,
            write_speed=write_speed, write_acc=write_acc,
            abort_check=abort_check, label=str(details.get("start_pose")),
            debug=debug)
        details["start_refresh"] = refresh
        if start_err:
            return _finish_debug(
                debug, {"ok": False, "error": start_err, "held_pose": True,
                        "limped": False, "preflight": details})
        q_nom_robot = np.asarray(start_target_deg, dtype=float) * DEG2RAD
    else:
        # Stance/lower policies still use the exact pose they start from.
        state_robot = None
        for _ in range(5):
            state_robot = est.update(want_full_feedback=True)
            time.sleep(timing.policy_dt)
        if state_robot is None or not state_robot.bus_ok:
            limp()
            return {"ok": False,
                    "error": "feedback unavailable during settle"}
        q_nom_robot = state_robot.joint_position.copy()
    q_nom = robot_abs_rad_to_policy_rad(q_nom_robot, joint_frame)
    est.set_commanded(q_nom_robot)
    bus.write_all((q_nom_robot * RAD2DEG).tolist(), speed=write_speed,
                  acc=write_acc)
    last_q_robot_cmd = q_nom_robot.copy()
    est.reset_episode_filters()
    warmup_good = 0
    warmup_stale = 0
    for _ in range(3):
        try:
            sampled = est.update(want_full_feedback=False)
        except Exception:
            sampled = None
        if sampled is not None and sampled.bus_ok:
            state_robot = sampled
            warmup_good += 1
        else:
            warmup_stale += 1
        time.sleep(timing.policy_dt)
    if state_robot is None or not state_robot.bus_ok:
        warmup_refresh: dict = {}
        state_robot = _direct_start_state(bus, q_nom_robot, warmup_refresh)
        if state_robot is None:
            return _finish_debug(
                debug, {"ok": False,
                        "error": "feedback unavailable during start warmup",
                        "preflight": details,
                        "start_warmup": warmup_refresh})
        details["start_warmup_fallback"] = warmup_refresh
    if warmup_stale:
        details["start_warmup"] = {
            "snapshot_samples": warmup_good,
            "stale_samples": warmup_stale,
        }
    debug.event("start_warmup_done", state=_state_debug(state_robot),
                warmup=details.get("start_warmup"),
                fallback=details.get("start_warmup_fallback"))
    state = _state_for_policy_frame(state_robot, joint_frame)
    tilt_ref0 = (state.imu_roll, state.imu_pitch)
    safety.set_nominal(q_nom)
    safety.set_tilt_reference(*tilt_ref0)

    prev_action = np.zeros(N_JOINTS, dtype=float)
    vx_r = vy_r = 0.0
    n_ticks = int(round(total_s * timing.policy_hz))
    overruns = 0
    max_cur = 0.0
    tilt_rel_max = 0.0
    t_end = 0.0
    t_next = time.monotonic()
    result: dict = {"ok": True, "mode": mode,
                    "policy_joint_frame": joint_frame}
    last_good_stream_state = state_robot
    stale_stream_ticks = 0
    stale_stream_samples = 0
    max_stale_stream_ticks_seen = 0
    stale_stream_bursts = 0
    first_stale_at_s: float | None = None
    last_stale_at_s: float | None = None
    elog = _EpisodeLog(mode, obs_dim=int(policy.meta.get("obs_dim", 0)),
                       params={
        "mode": mode, "total_s": round(total_s, 1),
        "hz": timing.policy_hz,
        "policy_hz": timing.policy_hz,
        "trained_control_hz": timing.trained_control_hz,
        "trained_control_hz_explicit": timing.trained_control_hz_explicit,
        "runner_config_hz": timing.runner_config_hz,
        "policy_rate_adapted": timing.adapted,
        "inner_hz": inner_hz, "inner_steps": inner_steps,
        "max_delta_q_deg": round(max_dq_deg, 4),
        "max_delta_q_deg_explicit": max_dq_explicit,
        "write_speed": write_speed, "write_acc": write_acc,
        "policy": dict(policy.meta),
        "policy_joint_frame": joint_frame,
        "q_nom_deg": [round(float(q) * RAD2DEG, 2) for q in q_nom],
        "tilt_ref_deg": [round(tilt_ref0[0] * RAD2DEG, 2),
                         round(tilt_ref0[1] * RAD2DEG, 2)],
        "tilt_trip_deg": tilt_trip_deg,
        "debug_log": debug.name,
        "preflight": details,
        **({"vx": round(vx, 3), "vy": round(vy, 3),
            "rot60": canon is not None,
            **({"turn": turn} if turn else {})}
           if mode == "walk" else {}),
    }, debug=debug)

    phase = 0.0        # walk phase clock (obs-74/93 policies only)
    phase_run_on_yaw = bool(float(policy.meta.get("walk_phase_run_on_yaw",
                                                  0.0)))
    for i in range(n_ticks):
        if abort_check():
            # Operator stop: HOLD pose (torque stays on); X still limps.
            result.update(ok=False, error="aborted",
                          held_pose=True, ticks=i)
            break
        t = i * timing.policy_dt
        if mode == "walk":
            goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0,
                            height_ref=0.0, unload_leg=None)
            vx_r, vy_r = _walk_vel_ref(t, total_s, vx, vy)
            wz_r = 0.0
            if walk_obs == 93 and turn in ("left", "right"):
                turn_sign = 1.0 if turn == "left" else -1.0
                wz_r = turn_sign * float(policy.meta.get(
                    "walk_yaw_max_rad_s", WALK_YAW_SCALE))
            obs = build_obs(cfg, state, q_nom, prev_action, goal=goal,
                            tilt_ref=tilt_ref0)
            obs = np.concatenate(
                [obs, _walk_obs_tail(walk_obs, vx_r, vy_r, phase, wz_r)]
            ).astype(np.float32)
            if _walk_phase_runs(walk_obs, vx_r, vy_r, wz_r,
                                phase_run_on_yaw=phase_run_on_yaw):
                phase = (phase + 2.0 * math.pi * phase_hz
                         * timing.policy_dt) \
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
            chirality = selector.update(
                float(state.imu_gyro[2]), timing.policy_dt)
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
            debug.event("bad_action", tick=i, t_s=t, error=bad,
                        obs_len=len(obs), state=_state_debug(state))
            result.update(ok=False, error=f"bad action: {bad}", ticks=i)
            break
        q_prop = _CENTER_RAD + action * _HALF_RAD
        q_safe, status = safety.filter(q_prop, state, action=action)
        if status.terminate:
            limp()
            debug.event("safety_trip", tick=i, t_s=t,
                        reason=status.reason, detail=status.detail,
                        held=status.held, state=_state_debug(state),
                        q_prop_deg=[round(float(x) * RAD2DEG, 2)
                                    for x in q_prop],
                        q_safe_deg=[round(float(x) * RAD2DEG, 2)
                                    for x in q_safe])
            result.update(ok=False, error=f"safety trip: {status.reason}"
                          + (f" ({status.detail})" if status.detail else ""),
                          limped=True, ticks=i)
            break
        q_robot_cmd = policy_rad_to_robot_abs_rad(q_safe, joint_frame)
        prev_stale_ticks = stale_stream_ticks
        (state_robot, t_next, extra_overruns, stream_err,
         stale_stream_ticks, stale_added) = _stream_target(
            bus, est, last_q_robot_cmd, q_robot_cmd,
            t_next=t_next, inner_steps=inner_steps, inner_dt=inner_dt,
            write_speed=write_speed, write_acc=write_acc,
            abort_check=abort_check,
            last_good_state=last_good_stream_state,
            stale_ticks=stale_stream_ticks,
            max_stale_ticks=DRIVE_STREAM_STALE_TICKS)
        overruns += extra_overruns
        stale_stream_samples += stale_added
        if stale_added:
            burst_peak = max(prev_stale_ticks + stale_added,
                             stale_stream_ticks)
            max_stale_stream_ticks_seen = max(
                max_stale_stream_ticks_seen, burst_peak)
            last_stale_at_s = t
            if first_stale_at_s is None:
                first_stale_at_s = t
            if prev_stale_ticks == 0:
                stale_stream_bursts += 1
                debug.event("stream_feedback_stale_begin", tick=i, t_s=t,
                            stale_added=stale_added,
                            stale_ticks=stale_stream_ticks,
                            state=_state_debug(state_robot,
                                               q_cmd_rad=q_robot_cmd))
            if stale_stream_ticks == 0:
                debug.event("stream_feedback_recovered", tick=i, t_s=t,
                            previous_stale_ticks=burst_peak,
                            state=_state_debug(state_robot,
                                               q_cmd_rad=q_robot_cmd))
        elif prev_stale_ticks > 0 and stale_stream_ticks == 0:
            debug.event("stream_feedback_recovered", tick=i, t_s=t,
                        previous_stale_ticks=prev_stale_ticks,
                        state=_state_debug(state_robot,
                                           q_cmd_rad=q_robot_cmd))
        if stream_err:
            debug.event("stream_error", tick=i, t_s=t, error=stream_err,
                        stale_ticks=stale_stream_ticks,
                        stale_samples=stale_stream_samples,
                        state=_state_debug(state_robot,
                                           q_cmd_rad=q_robot_cmd))
            if stream_err == "aborted":
                result.update(ok=False, error="aborted",
                              held_pose=True, ticks=i)
            else:
                limp()
                result.update(ok=False, error=stream_err,
                              limped=True, ticks=i)
            break
        last_q_robot_cmd = q_robot_cmd.copy()
        prev_action = action.copy()
        if not _stream_state_is_stale(state_robot):
            last_good_stream_state = state_robot
        state = _state_for_policy_frame(state_robot, joint_frame)
        if state.servo_current is not None:
            max_cur = max(max_cur,
                          float(np.max(np.abs(state.servo_current))))
        tilt_rel_max = max(
            tilt_rel_max,
            abs(state.imu_roll - tilt_ref0[0]) * RAD2DEG,
            abs(state.imu_pitch - tilt_ref0[1]) * RAD2DEG)
        t_end = t + timing.policy_dt
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
                "stale_stream_ticks": stale_stream_ticks,
                "stale_stream_samples": stale_stream_samples,
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
            state_robot = est.update()
            state = _state_for_policy_frame(state_robot, joint_frame)
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
        policy_hz=timing.policy_hz,
        runner_config_hz=timing.runner_config_hz,
        policy_rate_adapted=timing.adapted,
        inner_hz=inner_hz, inner_steps=inner_steps,
        max_delta_q_deg=round(max_dq_deg, 4),
        max_delta_q_deg_explicit=max_dq_explicit,
        write_speed=write_speed, write_acc=write_acc,
        stale_stream_samples=stale_stream_samples,
        stale_stream_ticks=stale_stream_ticks,
        stale_stream_bursts=stale_stream_bursts,
        max_stale_stream_ticks_seen=max_stale_stream_ticks_seen,
        first_stale_stream_at_s=(round(first_stale_at_s, 3)
                                 if first_stale_at_s is not None else None),
        last_stale_stream_at_s=(round(last_stale_at_s, 3)
                                if last_stale_at_s is not None else None),
        max_stale_stream_ticks=DRIVE_STREAM_STALE_TICKS,
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
    debug.attach(result)
    result["log"] = elog.close(result)
    debug.event("episode_complete", result=result)
    debug.close(result)
    return result


def run_drive_session(drive, cmd: DriveCommand, *, on_progress=None,
                      abort_check=None, rot60: bool = True,
                      walk_weights: Path | None = None,
                      hold_weights: Path | None = None,
                      allow_step_stand_start: bool = False) -> dict:
    """Blocking persistent drive session (MuJoCo-viewer-style driving).

    Same conventions as run_policy_move mode="walk" — plant-stance
    start gate, 25 Hz, walk obs contract with meas := ref, rot-60
    canonicalizer, 25 deg tilt trip — but the command is LIVE: the
    browser streams body-frame (vx, vy) heartbeats into ``cmd`` while
    arrow keys are held. Refs slew toward the target at the trained
    ramp rate (0 -> full band in WALK_RAMP_S), so a key press feels
    like the training ramp and a release decays to the trained stop.

    Hold model: with ``hold_weights=None`` the session uses a built-in
    direct joint hold of the last safe commanded pose. This intentionally
    does NOT call the walk policy at zero refs: deployed walk champions
    can produce saturated gait actions at neutral joystick because zero
    speed was outside their useful hardware contract. A separate hold
    policy (obs 68 stance at height_ref 0, or another obs-72/74/93 file
    trained to stand still at zero refs) can take over instead. Every
    model switch re-anchors the episode frame (q_nom := present pose,
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
    if walk_obs not in WALK_OBS_DIMS:
        return {"ok": False,
                "error": (f"{Path(wpath).name} is not a walk policy "
                          f"(obs {walk_obs} not 72/74/93)")}
    # obs 74 = phase-clock walk (see run_policy_move): all-heading
    # training, no rot-60/mirror, phase_hz required in export meta.
    phase_hz = 0.0
    if walk_obs in WALK_PHASE_OBS_DIMS:
        if "phase_hz" not in walk_policy.meta:
            return {"ok": False,
                    "error": (f"{Path(wpath).name} is obs-{walk_obs} "
                              "but has no "
                              "phase_hz in meta — re-export with "
                              "--extra-meta phase_hz=<trained hz>")}
        phase_hz = float(walk_policy.meta["phase_hz"])
    hold_policy = None
    hold_obs = None
    if hold_weights is not None:
        hold_policy = NumpyPolicy(hold_weights)
        hold_obs = hold_policy.meta.get("obs_dim")
        if hold_obs not in (68, *WALK_OBS_DIMS):
            return {"ok": False,
                    "error": (f"{Path(hold_weights).name} fits no hold "
                              f"role (obs {hold_obs}, need 68/72/74/93)")}
        if hold_obs in WALK_PHASE_OBS_DIMS and "phase_hz" not in hold_policy.meta:
            return {"ok": False,
                    "error": (f"{Path(hold_weights).name} is "
                              f"obs-{hold_obs} but has no phase_hz in meta")}
    try:
        joint_frame = policy_joint_frame(walk_policy, cfg)
        hold_joint_frame = (policy_joint_frame(hold_policy, cfg)
                            if hold_policy is not None else joint_frame)
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    if hold_policy is not None and hold_joint_frame != joint_frame:
        return {"ok": False,
                "error": ("walk/hold policy joint_frame mismatch "
                          f"({joint_frame} vs {hold_joint_frame})")}
    timing = _policy_timing(walk_policy)
    if hold_policy is not None:
        hold_timing = _policy_timing(hold_policy)
        if abs(hold_timing.policy_hz - timing.policy_hz) > 1e-6:
            return {"ok": False,
                    "error": ("drive walk/hold policy_hz mismatch "
                              f"({timing.policy_hz:g} vs "
                              f"{hold_timing.policy_hz:g}); select a "
                              "hold policy trained at the same cadence")}
    write_speed, write_acc = _policy_bus_profile(walk_policy, cfg)
    inner_steps, inner_hz, inner_dt = _inner_stream_plan(
        walk_policy, cfg, timing.policy_hz)
    debug = _RunDebug("drive", {
        "walk_policy_path": str(wpath),
        "walk_policy_name": walk_policy.meta.get("name"),
        "walk_obs_dim": walk_obs,
        "hold_policy_path": str(hold_weights) if hold_weights else None,
        "hold_obs_dim": hold_obs,
        "joint_frame": joint_frame,
        "timing": {
            "policy_hz": timing.policy_hz,
            "trained_control_hz": timing.trained_control_hz,
            "runner_config_hz": timing.runner_config_hz,
            "adapted": timing.adapted,
            "inner_hz": inner_hz,
            "inner_steps": inner_steps,
        },
        "write_speed": write_speed,
        "write_acc": write_acc,
        "rot60": rot60,
    })

    canon = (make_walk_canonicalizer(walk_policy, cfg)
             if rot60 and walk_obs == 72 else None)
    hold_canon = (make_walk_canonicalizer(hold_policy, cfg)
                  if rot60 and hold_policy is not None and hold_obs == 72
                  else None)
    if canon is None and walk_obs == 72 and not _ROT60_OK:
        # Without the canonicalizer only the trained forward wedge is
        # safe; a live joystick can't be trusted to stay inside it.
        # (obs-74 phase policies trained all headings — naked is fine.)
        return _finish_debug(
            debug, {"ok": False,
                    "error": ("drive session needs the rot-60 canonicalizer "
                              "(rl_move/sim/rot60.py not deployed)")})

    ok, reason, details = preflight(
        bus, "walk", allow_step_stand=bool(allow_step_stand_start))
    debug.event("preflight", ok=ok, reason=reason, details=details)
    if not ok:
        return _finish_debug(
            debug, {"ok": False, "error": f"preflight: {reason}", **details})
    start_target_deg, start_err = _preflight_start_target_deg(
        "walk", details, allow_step_stand=bool(allow_step_stand_start))
    if start_target_deg is None:
        return _finish_debug(
            debug, {"ok": False, "error": f"preflight: {start_err}",
                    **details})
    debug.event("start_target_selected",
                start_pose=details.get("start_pose"),
                target_deg=start_target_deg.tolist())

    def limp():
        try:
            bus.enable_all_torque(False)
        except Exception:
            try:
                drive._torque_all(False)
            except Exception:
                pass

    def hold_current_pose_after_stream_loss(
            fallback_robot: np.ndarray) -> bool:
        """Keep a weight-bearing walk from turning one bus miss into a drop."""
        debug.event("hold_after_stream_loss_begin",
                    fallback_deg=[round(float(x) * RAD2DEG, 2)
                                  for x in fallback_robot])
        try:
            _set_weight_bearing_torque(bus)
            drive._torque_all(True)
            drive.armed = True
        except Exception:
            pass

        pose = None
        for _ in range(5):
            try:
                sampled = est.update(want_full_feedback=True)
            except Exception:
                sampled = None
            if sampled is not None and sampled.bus_ok:
                pose = sampled.joint_position.copy()
                debug.event("hold_after_stream_loss_sampled",
                            state=_state_debug(sampled))
                break
            time.sleep(min(0.05, timing.policy_dt))
        if pose is None:
            pose = np.asarray(fallback_robot, dtype=float).copy()

        try:
            est.set_commanded(pose)
            bus.write_all((pose * RAD2DEG).tolist(), speed=write_speed,
                          acc=write_acc)
            with drive._lock:
                drive.status = "rl drive holding after stream loss"
            debug.event("hold_after_stream_loss_ok",
                        pose_deg=[round(float(x) * RAD2DEG, 2)
                                  for x in pose])
            return True
        except Exception:
            # If the half-duplex bus is still recovering, the least bad
            # weight-bearing choice is to leave torque enabled instead of
            # limping the whole body onto the floor.
            try:
                _set_weight_bearing_torque(bus)
                drive._torque_all(True)
            except Exception:
                pass
            debug.event("hold_after_stream_loss_write_failed")
            return False

    with drive._lock:
        drive.mode = "demo"
        try:
            drive.gait.stop()
        except Exception:
            pass
        # Do not trust the cached armed flag here. A stale True with actual
        # torque disabled lets joint-hold command start pose at ~0A, which
        # looks like a slow/drop fall when Start Driving is pressed.
        _set_weight_bearing_torque(bus)
        drive._torque_all(True)
        drive.armed = True
        drive.status = "rl drive armed"

    est = RobotStateEstimator(bus, cfg)
    safety = SafetyLayer(cfg)
    max_dq_deg, max_dq_explicit = _apply_policy_safety_timing(
        safety, walk_policy, cfg, timing)
    safety.max_roll = math.radians(WALK_MAX_TILT_DEG)
    safety.max_pitch = math.radians(WALK_MAX_TILT_DEG)

    state_robot, refresh, start_err = _refresh_verified_start_pose(
        bus, est, start_target_deg, timing=timing,
        write_speed=write_speed, write_acc=write_acc,
        abort_check=abort_check, label=str(details.get("start_pose")),
        debug=debug)
    details["start_refresh"] = refresh
    if start_err:
        return _finish_debug(
            debug, {"ok": False, "error": start_err, "held_pose": True,
                    "limped": False, "preflight": details})
    q_nom_robot = np.asarray(start_target_deg, dtype=float) * DEG2RAD
    q_nom = robot_abs_rad_to_policy_rad(q_nom_robot, joint_frame)
    est.set_commanded(q_nom_robot)
    bus.write_all((q_nom_robot * RAD2DEG).tolist(), speed=write_speed,
                  acc=write_acc)
    last_q_robot_cmd = q_nom_robot.copy()
    last_q_policy_cmd = q_nom.copy()
    est.reset_episode_filters()
    warmup_good = 0
    warmup_stale = 0
    for _ in range(3):
        try:
            sampled = est.update(want_full_feedback=False)
        except Exception:
            sampled = None
        if sampled is not None and sampled.bus_ok:
            state_robot = sampled
            warmup_good += 1
        else:
            warmup_stale += 1
        time.sleep(timing.policy_dt)
    if state_robot is None or not state_robot.bus_ok:
        warmup_refresh: dict = {}
        state_robot = _direct_start_state(bus, q_nom_robot, warmup_refresh)
        if state_robot is None:
            return _finish_debug(
                debug, {"ok": False,
                        "error": "feedback unavailable during start warmup",
                        "held_pose": True,
                        "limped": False,
                        "preflight": details,
                        "start_warmup": warmup_refresh})
        details["start_warmup_fallback"] = warmup_refresh
    if warmup_stale:
        details["start_warmup"] = {
            "snapshot_samples": warmup_good,
            "stale_samples": warmup_stale,
        }
    debug.event("start_warmup_done", state=_state_debug(state_robot),
                warmup=details.get("start_warmup"),
                fallback=details.get("start_warmup_fallback"))
    state = _state_for_policy_frame(state_robot, joint_frame)
    tilt_ref0 = (state.imu_roll, state.imu_pitch)
    safety.set_nominal(q_nom)
    safety.set_tilt_reference(*tilt_ref0)

    prev_action = np.zeros(N_JOINTS, dtype=float)
    vx_r = vy_r = 0.0
    phase = 0.0     # walk phase clock (obs-74/93 policies only); like the
                    # sim it starts at 0 and freezes at zero command
    phase_run_on_yaw = bool(float(walk_policy.meta.get(
        "walk_phase_run_on_yaw", 0.0)))
    dv_max = WALK_SPEED_MAX * timing.policy_dt / WALK_RAMP_S
    active = "hold"
    walk_cmd_since: float | None = None
    walk_active_since: float | None = None
    zero_since = 0.0            # session time when refs+target went zero
    stopping = None             # reason string once winding down
    overruns = 0
    max_cur = 0.0
    tilt_rel_max = 0.0
    t = 0.0
    i = 0
    t_next = time.monotonic()
    last_hold_refresh_t = -DRIVE_HOLD_REFRESH_S
    result: dict = {"ok": True, "mode": "drive",
                    "policy_joint_frame": joint_frame}
    last_good_stream_state = state_robot
    stale_stream_ticks = 0
    stale_stream_samples = 0
    max_stale_stream_ticks_seen = 0
    stale_stream_bursts = 0
    first_stale_at_s: float | None = None
    last_stale_at_s: float | None = None
    elog = _EpisodeLog("drive", obs_dim=int(walk_obs), params={
        "mode": "drive", "hz": timing.policy_hz,
        "policy_hz": timing.policy_hz,
        "trained_control_hz": timing.trained_control_hz,
        "trained_control_hz_explicit": timing.trained_control_hz_explicit,
        "runner_config_hz": timing.runner_config_hz,
        "policy_rate_adapted": timing.adapted,
        "inner_hz": inner_hz, "inner_steps": inner_steps,
        "max_delta_q_deg": round(max_dq_deg, 4),
        "max_delta_q_deg_explicit": max_dq_explicit,
        "write_speed": write_speed, "write_acc": write_acc,
        "policy": dict(walk_policy.meta),
        "hold_policy": (dict(hold_policy.meta)
                        if hold_policy is not None else None),
        "hold_strategy": ("learned_policy"
                          if hold_policy is not None else "joint_hold"),
        "policy_joint_frame": joint_frame,
        "q_nom_deg": [round(float(q) * RAD2DEG, 2) for q in q_nom],
        "tilt_ref_deg": [round(tilt_ref0[0] * RAD2DEG, 2),
                         round(tilt_ref0[1] * RAD2DEG, 2)],
        "tilt_trip_deg": WALK_MAX_TILT_DEG,
        "debug_log": debug.name,
        "preflight": details, "rot60": canon is not None,
    }, debug=debug)

    def reanchor():
        """Episode re-anchor on model switch (q frame + prev_action)."""
        nonlocal q_nom, prev_action, last_q_robot_cmd, last_q_policy_cmd
        q_nom = state.joint_position.copy()
        safety.set_nominal(q_nom)
        prev_action = np.zeros(N_JOINTS, dtype=float)
        last_q_policy_cmd = q_nom.copy()
        if state_robot is not None:
            last_q_robot_cmd = state_robot.joint_position.copy()

    def sample_hold_tick() -> tuple[object | None, float, int, str]:
        """Wait one policy tick and read state without writing targets."""
        nonlocal t_next
        if abort_check():
            return state_robot, t_next, 0, "aborted"
        t_next += timing.policy_dt
        lag = time.monotonic() - t_next
        overruns_tick = 0
        if lag > 0:
            overruns_tick = 1
            t_next = time.monotonic()
        else:
            time.sleep(-lag)
        sampled = None
        for attempt in range(4):
            sampled = est.update()
            if sampled is not None and sampled.bus_ok:
                return sampled, t_next, overruns_tick, ""
            if abort_check():
                return state_robot, t_next, overruns_tick, "aborted"
            # Snapshot reads can miss a beat while the MCU is recovering
            # from a previous stream/glide. In joint-hold mode the servos
            # already have a safe target, so retry briefly instead of
            # converting one telemetry miss into an emergency limp.
            time.sleep(min(0.02, timing.policy_dt * 0.5))
        return sampled, t_next, overruns_tick, "feedback lost during hold"

    while True:
        if abort_check():
            result.update(ok=False, error="aborted", held_pose=True,
                          ticks=i)
            break
        t = i * timing.policy_dt
        vx_t, vy_t, wz_t, hb_age, stop_req = cmd.get()
        if stop_req and stopping is None:
            stopping = "stopped"
        if hb_age > DRIVE_IDLE_END_S and stopping is None:
            stopping = "no command from browser — session ended"
        if t > DRIVE_MAX_SESSION_S and stopping is None:
            stopping = f"session cap {DRIVE_MAX_SESSION_S:.0f}s reached"
        if stopping is not None or hb_age > DRIVE_CMD_TIMEOUT_S:
            vx_t = vy_t = wz_t = 0.0
        vx_t, vy_t = _drive_clamp_translation(vx_t, vy_t)
        moving_requested = _drive_command_is_moving(
            vx_t, vy_t, wz_t, int(walk_obs))
        if moving_requested:
            if walk_cmd_since is None:
                walk_cmd_since = t
            moving = (active == "walk"
                      or (t - walk_cmd_since) >= DRIVE_WALK_ENGAGE_S)
        else:
            walk_cmd_since = None
            moving = False
        if stopping is not None and not moving:
            # Graceful end: refs decayed to zero, robot HOLDS the pose.
            result.update(ticks=i, ended=stopping)
            break

        if moving:
            zero_since = t
            if active != "walk":
                prev_active = active
                active = "walk"
                walk_active_since = t
                reanchor()
                debug.event("drive_model_switch", tick=i, t_s=t,
                            from_model=prev_active, to_model=active,
                            vx_cmd=vx_t, vy_cmd=vy_t, wz_cmd=wz_t,
                            state=_state_debug(state_robot))
                # Engage inside the policy's trained command band on the
                # first walk tick. The safety layer still rate-limits joint
                # motion; this only prevents feeding the actor a 0.002 m/s
                # ramp value it was never meant to interpret.
                vx_r, vy_r = vx_t, vy_t
            else:
                # Slew refs toward the target at the trained ramp rate, then
                # keep the nonzero ref inside the trained band.
                vx_r += max(-dv_max, min(dv_max, vx_t - vx_r))
                vy_r += max(-dv_max, min(dv_max, vy_t - vy_r))
                vx_r, vy_r = _drive_clamp_translation(vx_r, vy_r)
        elif active == "walk":
            prev_active = active
            vx_r = vy_r = 0.0
            active = "hold"
            walk_active_since = None
            reanchor()
            last_hold_refresh_t = -DRIVE_HOLD_REFRESH_S
            debug.event("drive_model_switch", tick=i, t_s=t,
                        from_model=prev_active, to_model=active,
                        state=_state_debug(state_robot))
        else:
            vx_r = vy_r = 0.0

        goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0, height_ref=0.0,
                        unload_leg=None)
        base_obs = build_obs(cfg, state, q_nom, prev_action, goal=goal,
                             tilt_ref=tilt_ref0)
        need_obs = walk_obs if active == "walk" else hold_obs
        wz_r = wz_t if active == "walk" else 0.0
        obs = None
        action = None
        if _drive_uses_learned_policy(active, hold_policy):
            if need_obs in WALK_OBS_DIMS:
                obs = np.concatenate(
                    [base_obs, _walk_obs_tail(need_obs, vx_r, vy_r,
                                              phase, wz_r)]
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
            # Phase clock advance (obs-74/93 policies): after obs, gated on a
            # live command ref. obs-93 AMP policies may also advance on yaw.
            if _walk_phase_runs(walk_obs, vx_r, vy_r, wz_r,
                                phase_run_on_yaw=phase_run_on_yaw):
                phase = (phase + 2.0 * math.pi * phase_hz
                         * timing.policy_dt) \
                    % (2.0 * math.pi)
            action, bad = safety.validate_action(raw_act, n_act=N_JOINTS)
            if action is None:
                limp()
                debug.event("bad_action", tick=i, t_s=t, active=active,
                            error=bad, obs_len=len(obs),
                            state=_state_debug(state))
                result.update(ok=False, error=f"bad action: {bad}", ticks=i)
                break
            q_prop = _CENTER_RAD + action * _HALF_RAD
            if (active == "walk" and walk_active_since is not None
                    and DRIVE_WALK_ACTION_RAMP_S > 0.0):
                alpha = min(1.0, max(
                    0.0, (t - walk_active_since)
                    / DRIVE_WALK_ACTION_RAMP_S))
                q_prop = last_q_policy_cmd + alpha * (
                    q_prop - last_q_policy_cmd)
        else:
            q_prop = last_q_policy_cmd.copy()
        q_safe, status = safety.filter(q_prop, state, action=action)
        if status.terminate:
            limp()
            debug.event("safety_trip", tick=i, t_s=t, active=active,
                        reason=status.reason, detail=status.detail,
                        held=status.held, state=_state_debug(state),
                        q_prop_deg=[round(float(x) * RAD2DEG, 2)
                                    for x in q_prop],
                        q_safe_deg=[round(float(x) * RAD2DEG, 2)
                                    for x in q_safe])
            result.update(ok=False, error=f"safety trip: {status.reason}"
                          + (f" ({status.detail})" if status.detail else ""),
                          limped=True, ticks=i)
            break
        q_robot_cmd = policy_rad_to_robot_abs_rad(q_safe, joint_frame)
        prev_stale_ticks = stale_stream_ticks
        if _drive_uses_learned_policy(active, hold_policy):
            (state_robot, t_next, extra_overruns, stream_err,
             stale_stream_ticks, stale_added) = _stream_target(
                bus, est, last_q_robot_cmd, q_robot_cmd,
                t_next=t_next, inner_steps=inner_steps, inner_dt=inner_dt,
                write_speed=write_speed, write_acc=write_acc,
                abort_check=abort_check,
                last_good_state=last_good_stream_state,
                stale_ticks=stale_stream_ticks,
                max_stale_ticks=DRIVE_STREAM_STALE_TICKS)
        else:
            est.set_commanded(q_robot_cmd)
            stream_err = ""
            if t - last_hold_refresh_t >= DRIVE_HOLD_REFRESH_S:
                try:
                    bus.write_all((q_robot_cmd * RAD2DEG).tolist(),
                                  speed=write_speed, acc=write_acc)
                    last_hold_refresh_t = t
                except Exception as e:
                    stream_err = f"hold write failed: {e}"
            if stream_err:
                extra_overruns = 0
                stale_added = 0
            else:
                state_robot, t_next, extra_overruns, stream_err = (
                    sample_hold_tick())
                stale_added = 0
        overruns += extra_overruns
        stale_stream_samples += stale_added
        if stale_added:
            burst_peak = max(prev_stale_ticks + stale_added,
                             stale_stream_ticks)
            max_stale_stream_ticks_seen = max(
                max_stale_stream_ticks_seen, burst_peak)
            last_stale_at_s = t
            if first_stale_at_s is None:
                first_stale_at_s = t
            if prev_stale_ticks == 0:
                stale_stream_bursts += 1
                debug.event("stream_feedback_stale_begin", tick=i, t_s=t,
                            active=active, stale_added=stale_added,
                            stale_ticks=stale_stream_ticks,
                            state=_state_debug(state_robot,
                                               q_cmd_rad=q_robot_cmd))
            if stale_stream_ticks == 0:
                debug.event("stream_feedback_recovered", tick=i, t_s=t,
                            active=active,
                            previous_stale_ticks=burst_peak,
                            state=_state_debug(state_robot,
                                               q_cmd_rad=q_robot_cmd))
        elif prev_stale_ticks > 0 and stale_stream_ticks == 0:
            debug.event("stream_feedback_recovered", tick=i, t_s=t,
                        active=active,
                        previous_stale_ticks=prev_stale_ticks,
                        state=_state_debug(state_robot,
                                           q_cmd_rad=q_robot_cmd))
        if stream_err:
            debug.event("stream_error", tick=i, t_s=t, active=active,
                        error=stream_err,
                        stale_ticks=stale_stream_ticks,
                        stale_samples=stale_stream_samples,
                        state=_state_debug(state_robot,
                                           q_cmd_rad=q_robot_cmd))
            if stream_err == "aborted":
                result.update(ok=False, error="aborted",
                              held_pose=True, ticks=i)
            elif (stream_err == "feedback lost during hold"
                  and not _drive_uses_learned_policy(active, hold_policy)):
                result.update(ok=False, error=stream_err,
                              held_pose=True, ticks=i)
            elif (stream_err == "feedback stale during stream"
                  and active == "walk"):
                held = hold_current_pose_after_stream_loss(last_q_robot_cmd)
                result.update(
                    ok=False,
                    error=(stream_err + ("; held current pose" if held
                                         else "; torque left enabled")),
                    held_pose=held, limped=False, ticks=i)
            else:
                limp()
                result.update(ok=False, error=stream_err,
                              limped=True, ticks=i)
            break
        last_q_robot_cmd = q_robot_cmd.copy()
        last_q_policy_cmd = q_safe.copy()
        if action is not None:
            prev_action = action.copy()
        if not _stream_state_is_stale(state_robot):
            last_good_stream_state = state_robot
        state = _state_for_policy_frame(state_robot, joint_frame)
        if state.servo_current is not None:
            max_cur = max(max_cur,
                          float(np.max(np.abs(state.servo_current))))
        tilt_rel_max = max(
            tilt_rel_max,
            abs(state.imu_roll - tilt_ref0[0]) * RAD2DEG,
            abs(state.imu_pitch - tilt_ref0[1]) * RAD2DEG)
        # Hold-68 obs would misalign the fixed walk-wide obs columns —
        # blank them for those ticks (walk replay parity is what the
        # offline contract needs).
        obs_for_log = (obs if obs is not None and len(obs) == elog.obs_dim
                       else None)
        display_active = ("arming" if moving_requested and not moving
                          and stopping is None else active)
        elog.tick(t, state, action, q_safe, goal, vx_r, vy_r, max_cur,
                  obs=obs_for_log,
                  phase=("stopping" if stopping else display_active),
                  rot60_k=(canon.k if canon is not None
                           and active == "walk" else None))
        snap = {
            "t_s": round(t, 1), "model": display_active,
            "vx_ref": round(vx_r, 3), "vy_ref": round(vy_r, 3),
            "wz_ref": round(wz_r, 3),
            "vx_cmd": round(vx_t, 3), "vy_cmd": round(vy_t, 3),
            "wz_cmd": round(wz_t, 3),
            "walk_arming": bool(moving_requested and not moving),
            "roll_deg": round((state.imu_roll - tilt_ref0[0]) * RAD2DEG,
                              1),
            "pitch_deg": round((state.imu_pitch - tilt_ref0[1])
                               * RAD2DEG, 1),
            "max_current_a": round(max_cur, 2),
            "stale_stream_ticks": stale_stream_ticks,
            "stale_stream_samples": stale_stream_samples,
            "rot60_k": canon.k if canon is not None else None,
            "stopping": stopping, "overruns": overruns,
        }
        cmd.publish(snap)
        if i % 5 == 0:
            on_progress({
                "msg": (f"drive {display_active} t={t:5.1f}s "
                        f"v=({vx_r * 1000:+.0f},{vy_r * 1000:+.0f})mm/s "
                        f"wz={wz_r:+.2f}rad/s "
                        f"maxI={max_cur:.2f}A"
                        + (f" · {stopping}" if stopping else "")),
                **snap})
        i += 1
        t = i * timing.policy_dt

    # Same post-episode observation tail as run_policy_move: keep
    # reading (never commanding) so a fall during the end-of-session
    # hold is in the trace.
    TAIL_S = 3.0
    tail_tilt_max = 0.0
    for k in range(int(TAIL_S * 10)):
        time.sleep(0.1)
        try:
            state_robot = est.update()
            state = _state_for_policy_frame(state_robot, joint_frame)
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
        policy_hz=timing.policy_hz,
        runner_config_hz=timing.runner_config_hz,
        policy_rate_adapted=timing.adapted,
        inner_hz=inner_hz, inner_steps=inner_steps,
        max_delta_q_deg=round(max_dq_deg, 4),
        max_delta_q_deg_explicit=max_dq_explicit,
        write_speed=write_speed, write_acc=write_acc,
        stale_stream_samples=stale_stream_samples,
        stale_stream_ticks=stale_stream_ticks,
        stale_stream_bursts=stale_stream_bursts,
        max_stale_stream_ticks_seen=max_stale_stream_ticks_seen,
        first_stale_stream_at_s=(round(first_stale_at_s, 3)
                                 if first_stale_at_s is not None else None),
        last_stale_stream_at_s=(round(last_stale_at_s, 3)
                                if last_stale_at_s is not None else None),
        max_stale_stream_ticks=DRIVE_STREAM_STALE_TICKS,
        tilt_ref_deg=[round(tilt_ref0[0] * RAD2DEG, 2),
                      round(tilt_ref0[1] * RAD2DEG, 2)],
        tilt_rel_max_deg=round(tilt_rel_max, 1),
        tail_s=TAIL_S,
        tail_tilt_max_deg=round(tail_tilt_max, 1),
        fell=bool(max(tail_tilt_max, tilt_rel_max) > 35.0),
    )
    debug.attach(result)
    result["log"] = elog.close(result)
    debug.event("episode_complete", result=result)
    debug.close(result)
    return result
