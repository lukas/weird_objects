"""HexapodBalanceEnv — Gymnasium-shaped real-robot stationary balance."""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .body_ik import FixedFootBodyIK
from .config import cfg_get, load_config
from .control_loop import ControlLoop
from .logger import EpisodeLogger, flatten_state_row
from .robot_state import (
    DEG2RAD, N_JOINTS, RAD2DEG, RobotState, RobotStateEstimator,
)
from .safety import SafetyLayer, action_to_body_offset

_LINUX = Path(__file__).resolve().parents[1] / "linux_control"
if str(_LINUX) not in sys.path:
    sys.path.insert(0, str(_LINUX))
_URT2 = _LINUX / "urt2_setup"
if str(_URT2) not in sys.path:
    sys.path.insert(0, str(_URT2))


def _standing_q_rad() -> np.ndarray:
    from feetech_bus import standing_pose_degrees
    return np.array(standing_pose_degrees(), dtype=float) * DEG2RAD


@dataclass
class TaskGoal:
    """Commanded target for the goal-conditioned lean / rise / weight-shift
    task.

    ``roll_ref``/``pitch_ref`` are the desired body attitude (rad); the
    plain balance task is the special case where both are 0.
    ``height_ref`` is the desired body height offset from the nominal
    stance (m; rise episodes ramp it from a crouch to 0). ``unload_leg``
    (0-5 or None) asks the policy to drive that leg's servo currents to
    zero — i.e. shift the body until the leg carries no weight.
    """
    roll_ref: float = 0.0
    pitch_ref: float = 0.0
    height_ref: float = 0.0
    unload_leg: int | None = None

    def as_obs(self, cfg: dict) -> np.ndarray:
        """9-dim goal observation: scaled refs + unload one-hot."""
        ts = float(cfg_get(cfg, "obs", "tilt_scale", default=0.2))
        hs = float(cfg_get(cfg, "obs", "height_scale_m", default=0.05))
        onehot = np.zeros(6, dtype=float)
        if self.unload_leg is not None:
            onehot[int(self.unload_leg)] = 1.0
        return np.concatenate([
            [self.roll_ref / max(ts, 1e-6), self.pitch_ref / max(ts, 1e-6),
             self.height_ref / max(hs, 1e-6)],
            onehot])


GOAL_DIM = 9


def build_obs(cfg: dict, state: RobotState, q_nom: np.ndarray,
              prev_action: np.ndarray,
              goal: "TaskGoal | None" = None,
              tilt_ref: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """47-dim observation (56 with a goal appended).

    Shared by the hardware env and the sim twin. Tilt is reported
    RELATIVE to ``tilt_ref`` — the attitude measured at episode start —
    because the raw reading can carry a large constant bias (IMU mounted
    at an angle, sloped floor) that neither the policy nor the goal refs
    could ever null out. The goal refs use the same tilt scaling, so the
    policy sees reference and measurement in identical units.
    """
    qs = float(cfg_get(cfg, "obs", "q_scale", default=1.0))
    qds = float(cfg_get(cfg, "obs", "qd_scale", default=2.0))
    ts = float(cfg_get(cfg, "obs", "tilt_scale", default=0.2))
    gs = float(cfg_get(cfg, "obs", "gyro_scale", default=1.0))
    q_rel = (state.joint_position - q_nom) / max(qs, 1e-6)
    qd = state.joint_velocity / max(qds, 1e-6)
    tilt = np.array([state.imu_roll - tilt_ref[0],
                     state.imu_pitch - tilt_ref[1]],
                    dtype=float) / max(ts, 1e-6)
    gyro = state.imu_gyro / max(gs, 1e-6)
    parts = [q_rel, qd, tilt, gyro, prev_action]
    if goal is not None:
        parts.append(goal.as_obs(cfg))
    return np.concatenate(parts).astype(np.float32)


def compute_reward(cfg: dict, state: RobotState, action: np.ndarray,
                   prev_action: np.ndarray,
                   goal: "TaskGoal | None" = None,
                   tilt_ref: tuple[float, float] = (0.0, 0.0),
                   height_err: float | None = None,
                   unload_force_n: float | None = None,
                   ref_quiet: bool = False
                   ) -> tuple[float, dict]:
    """Task-tracking reward. Shared by hardware env and sim twin.

    Structure (post 2026-08-07 redesign): the ONLY substantial positive
    term is a Gaussian tracking kernel — near +1/tick for following the
    commanded roll/pitch (and height / leg-unload when the goal asks),
    near 0 for ignoring it. There is deliberately NO unconditional alive
    bonus: paying one taught PPO that freezing in place collects ~96% of
    the achievable reward. Penalties (gyro, action, current) are kept
    weak — learn the task first, optimize effort later. Small quadratic
    tilt/height terms supply gradient far from the reference where the
    kernel is flat.

    Tilt is measured relative to ``tilt_ref`` (episode-start attitude): a
    lean goal means "lean this far from where you settled", which stays
    achievable no matter how the IMU is mounted. ``height_err`` =
    measured body height minus (start + height_ref); the sim supplies it
    from privileged chassis state, hardware passes None.
    """
    kt = float(cfg_get(cfg, "reward", "k_track", default=1.0))
    kr = float(cfg_get(cfg, "reward", "k_roll", default=10.0))
    kp = float(cfg_get(cfg, "reward", "k_pitch", default=10.0))
    kh = float(cfg_get(cfg, "reward", "k_height", default=100.0))
    kg = float(cfg_get(cfg, "reward", "k_gyro", default=0.05))
    ka = float(cfg_get(cfg, "reward", "k_action", default=0.005))
    kad = float(cfg_get(cfg, "reward", "k_action_delta", default=0.01))
    kc = float(cfg_get(cfg, "reward", "k_current", default=0.005))
    # Peak-load penalty (default OFF): squared max |current| over all
    # servos. The sum-square term barely distinguishes "18 motors at
    # 0.6 A" from "3 motors at 2.4 A doing all the work"; this term does,
    # pushing the policy to recruit every leg instead of riding a few
    # motors near the 2.5 A breaker.
    kcm = float(cfg_get(cfg, "reward", "k_current_max", default=0.0))
    # Quiet-stance bonus (default OFF): pay for being MOTIONLESS while
    # AT the goal pose with a STATIONARY reference. Gaussian on mean
    # squared joint speed, MULTIPLIED by the task kernel so a frozen
    # belly-rest earns nothing (kernel ~0 off-target — no revival of
    # the freeze shortcut the alive-bonus removal killed). Targets the
    # "3-leg stand while the free legs thrash" artifact: thrashing
    # zeroes this bonus even though it never moves the body.
    ks = float(cfg_get(cfg, "reward", "k_still", default=0.0))
    sig_qd = float(cfg_get(cfg, "reward", "still_sigma_rad_s", default=0.3))
    ku = float(cfg_get(cfg, "reward", "k_unload", default=0.2))
    alive = float(cfg_get(cfg, "reward", "alive", default=0.0))
    sig_t = math.radians(float(cfg_get(
        cfg, "reward", "track_sigma_deg", default=1.5)))
    sig_h = float(cfg_get(
        cfg, "reward", "height_sigma_mm", default=10.0)) * 0.001
    sig_c = float(cfg_get(cfg, "reward", "unload_sigma_a", default=0.15))
    sig_f = float(cfg_get(cfg, "reward", "unload_sigma_n", default=1.0))

    roll_ref = goal.roll_ref if goal is not None else 0.0
    pitch_ref = goal.pitch_ref if goal is not None else 0.0
    e_roll = float(state.imu_roll - tilt_ref[0] - roll_ref)
    e_pitch = float(state.imu_pitch - tilt_ref[1] - pitch_ref)

    # Unload episodes NEED to lean: with position-controlled pinned feet
    # the only way to open a foot's contact is tipping the body so that
    # foot's target rises (measured: shift alone leaves ~2.4 N, shift +
    # 4° lean reaches ~1.3 N). Widen the tilt tolerance there so the
    # mechanism isn't punished away; the safety tilt trip still applies.
    if goal is not None and goal.unload_leg is not None:
        sig_t = math.radians(float(cfg_get(
            cfg, "reward", "unload_tilt_sigma_deg", default=4.0)))

    # --- the task kernel: product of per-objective Gaussians in [0, 1] ---
    kernel = math.exp(-(e_roll ** 2 + e_pitch ** 2) / (2.0 * sig_t ** 2))
    r_height = 0.0
    if height_err is not None:
        kernel *= math.exp(-height_err ** 2 / (2.0 * sig_h ** 2))
        r_height = -kh * height_err ** 2
    unload_cur = None
    if goal is not None and goal.unload_leg is not None:
        if unload_force_n is not None:
            # Preferred: the leg's ground-reaction force (sim touch
            # sensor). Quiet stance = ~3.5 N per foot → kernel ≈ 0, so
            # freezing earns nothing; a truly unloaded leg (<1 N) pays.
            # Servo current CANNOT see this — quiet-stand currents are
            # ~0.1 A on every leg, loaded or not.
            kernel *= math.exp(-unload_force_n ** 2 / (2.0 * sig_f ** 2))
        elif state.servo_current is not None:
            j0 = 3 * int(goal.unload_leg)
            unload_cur = float(
                np.mean(np.abs(state.servo_current[j0:j0 + 3])))
            kernel *= math.exp(-unload_cur ** 2 / (2.0 * sig_c ** 2))
    r_task = kt * kernel

    # --- weak shaping / regularization ---
    r_roll = -kr * e_roll ** 2
    r_pitch = -kp * e_pitch ** 2
    r_gyro = -kg * float(np.sum(state.imu_gyro ** 2))
    r_act = -ka * float(np.sum(action ** 2))
    r_dad = -kad * float(np.sum((action - prev_action) ** 2))
    # Effort: penalize fighting (sum of squared servo currents, A²) —
    # the "don't cook motors" signal (2026-08-06: ~7 A holds). Kept weak
    # so moving toward the goal is never more expensive than tracking
    # pays. Sim fills servo_current from actuator torque; hardware from
    # the ~10 Hz full-feedback read.
    r_cur = 0.0
    r_cur_max = 0.0
    if state.servo_current is not None:
        r_cur = -kc * float(np.sum(np.square(state.servo_current)))
        if kcm > 0.0:
            r_cur_max = -kcm * float(
                np.max(np.abs(state.servo_current))) ** 2
    # Weight-shift goal: weak linear gradient toward zero load (the
    # kernel above pays the actual success but is flat far from it).
    r_unload = 0.0
    if unload_force_n is not None:
        r_unload = -ku * 0.1 * unload_force_n
    elif unload_cur is not None:
        r_unload = -ku * unload_cur

    # still_factor is exported via parts so the sim's curl-window
    # repricing can rescale reward_still with the swapped-in kernel.
    still_factor = 0.0
    r_still = 0.0
    if ks > 0.0 and ref_quiet:
        qd2 = float(np.mean(np.square(state.joint_velocity)))
        still_factor = math.exp(-qd2 / (2.0 * sig_qd ** 2))
        r_still = ks * kernel * still_factor

    parts = {
        "reward_task": r_task,
        "reward_roll": r_roll,
        "reward_pitch": r_pitch,
        "reward_height": r_height,
        "reward_gyro": r_gyro,
        "reward_action": r_act,
        "reward_action_delta": r_dad,
        "reward_current": r_cur,
        "reward_current_max": r_cur_max,
        "reward_unload": r_unload,
        "reward_still": r_still,
        "still_factor": still_factor,
        "reward_alive": alive,
        "reward_termination": 0.0,
    }
    return (r_task + alive + r_roll + r_pitch + r_height + r_gyro + r_act
            + r_dad + r_cur + r_cur_max + r_unload + r_still, parts)


class HexapodBalanceEnv:
    """50 Hz balance env. Observation dim 47, action dim 6."""

    OBS_DIM = 47
    ACT_DIM = 6

    def __init__(self, bus: Any, cfg: dict | None = None, *,
                 log: bool = True):
        self.cfg = cfg or load_config()
        self.bus = bus
        self.hz = float(cfg_get(self.cfg, "control", "hz", default=50))
        self.dt = 1.0 / self.hz
        self.episode_steps = int(round(
            float(cfg_get(self.cfg, "episode", "seconds", default=5)) * self.hz))
        self.settle_s = float(
            cfg_get(self.cfg, "episode", "settle_seconds", default=1.0))
        self.estimator = RobotStateEstimator(bus, self.cfg)
        self.ik = FixedFootBodyIK()
        self.safety = SafetyLayer(self.cfg)
        self._q_nom = _standing_q_rad()
        self._prev_action = np.zeros(self.ACT_DIM, dtype=float)
        self._tilt_ref0 = (0.0, 0.0)
        self._step = 0
        self._episode = 0
        self._state: RobotState | None = None
        self._logger: EpisodeLogger | None = None
        if log:
            log_dir = cfg_get(self.cfg, "logging", "dir",
                              default="logs/rl_move")
            # Resolve relative to linux_control or cwd.
            p = Path(log_dir)
            if not p.is_absolute():
                p = _LINUX / p
            self._logger = EpisodeLogger(
                p, fmt=str(cfg_get(self.cfg, "logging", "format", default="csv")))
        self.write_speed = int(cfg_get(self.cfg, "bus", "write_speed", default=400))
        self.write_acc = int(cfg_get(self.cfg, "bus", "write_acc", default=20))
        self.stand_speed = int(cfg_get(self.cfg, "bus", "stand_speed", default=200))
        self.enable_motion = bool(
            cfg_get(self.cfg, "bus", "enable_motion", default=False))
        self.preflight_tilt = math.radians(float(
            cfg_get(self.cfg, "episode", "preflight_max_tilt_deg", default=12)))
        self.hold_current = bool(
            cfg_get(self.cfg, "episode", "hold_current_pose", default=True))
        # Phase-1 policy: never auto-blend to default +20/+80.
        self.allow_stand_blend = bool(
            cfg_get(self.cfg, "episode", "allow_stand_blend", default=False))
        self.require_captured_plant = bool(
            cfg_get(self.cfg, "episode", "require_captured_plant", default=True))
        self.max_plant_delta_deg = float(
            cfg_get(self.cfg, "episode", "max_plant_delta_deg", default=15.0))
        self.limp_on_terminate = bool(
            cfg_get(self.cfg, "safety", "limp_on_terminate", default=True))
        if not self.enable_motion:
            print("[env] MOTION DISABLED (bus.enable_motion=false) — "
                  "sense/IK/safety/log only")

    def _load_plant_q_deg(self) -> tuple[np.ndarray | None, dict]:
        from feetech_bus import load_plant_pose, standing_pose_degrees
        plant = load_plant_pose()
        joints = plant.get("joints_deg")
        if joints is not None and len(joints) == 18:
            return np.asarray(joints, dtype=float), plant
        if plant.get("learned"):
            return np.asarray(standing_pose_degrees(), dtype=float), plant
        return None, plant

    def _verify_near_plant(self, q_rad: np.ndarray) -> None:
        q_plant, plant = self._load_plant_q_deg()
        if self.require_captured_plant:
            if q_plant is None or plant.get("joints_deg") is None:
                raise RuntimeError(
                    "no captured plant (joints_deg). Physically set a stable "
                    "stance, then: python3 -m rl_move.scripts.capture_plant")
        if q_plant is None:
            return
        err = np.max(np.abs(q_rad * RAD2DEG - q_plant))
        if err > self.max_plant_delta_deg:
            raise RuntimeError(
                f"current pose far from captured plant "
                f"(max |Δq|={err:.1f}° > {self.max_plant_delta_deg:.1f}°). "
                f"Hand-place near plant or re-run capture_plant; "
                f"will not auto-stand.")

    def _obs(self, state: RobotState) -> np.ndarray:
        return build_obs(self.cfg, state, self._q_nom, self._prev_action,
                         tilt_ref=self._tilt_ref0)

    def _reward(self, state: RobotState, action: np.ndarray
                ) -> tuple[float, dict]:
        return compute_reward(self.cfg, state, action, self._prev_action,
                              tilt_ref=self._tilt_ref0)

    def _command_deg(self, q_rad: np.ndarray, *, speed: int | None = None) -> None:
        self.estimator.set_commanded(q_rad)
        if not self.enable_motion:
            return
        deg = (np.asarray(q_rad) * RAD2DEG).tolist()
        sp = self.write_speed if speed is None else int(speed)
        self.bus.write_all(deg, speed=sp, acc=self.write_acc)

    def _limp(self) -> None:
        if not self.enable_motion:
            return
        try:
            if hasattr(self.bus, "enable_all_torque"):
                self.bus.enable_all_torque(False)
            elif hasattr(self.bus, "safe_stop"):
                self.bus.safe_stop(limp=True)
        except Exception as e:
            print(f"[env] limp failed: {e}")

    def _preflight_or_raise(self, state: RobotState) -> None:
        if abs(state.imu_roll) > self.preflight_tilt or \
                abs(state.imu_pitch) > self.preflight_tilt:
            raise RuntimeError(
                f"preflight tilt too high: roll={state.imu_roll*RAD2DEG:+.1f}° "
                f"pitch={state.imu_pitch*RAD2DEG:+.1f}° "
                f"(limit ±{self.preflight_tilt*RAD2DEG:.0f}°). "
                "Stand the robot up before commanding motion — a tipped "
                "chassis + SyncWrite can brown-out the board.")
        if not state.bus_ok or not state.imu_ok:
            raise RuntimeError(
                f"preflight sensors bad bus_ok={state.bus_ok} imu_ok={state.imu_ok}")

    def _smooth_to_stand(self) -> None:
        # Slow blend — tip risk if starting far from plant.
        state = self.estimator.update(want_full_feedback=True)
        self._preflight_or_raise(state)
        q0 = state.joint_position.copy()
        q1 = self._q_nom
        n = max(1, int(self.settle_s / 0.05))
        for i in range(n):
            a = (i + 1) / n
            q = (1 - a) * q0 + a * q1
            self._command_deg(q, speed=self.stand_speed)
            time.sleep(0.05)
            st = self.estimator.update()
            if abs(st.imu_roll) > self.preflight_tilt or \
                    abs(st.imu_pitch) > self.preflight_tilt:
                self._limp()
                raise RuntimeError(
                    f"tilt during stand blend — limped "
                    f"(roll={st.imu_roll*RAD2DEG:+.1f}° "
                    f"pitch={st.imu_pitch*RAD2DEG:+.1f}°)")
        self._command_deg(q1, speed=self.stand_speed)
        time.sleep(0.2)

    def reset(self, *, seed=None, options=None):
        del seed, options
        self._episode += 1
        self._step = 0
        self._prev_action[:] = 0.0
        self.safety.clear_estop()

        # Sensor-only first — refuse to move if already tipped.
        for _ in range(5):
            self._state = self.estimator.update()
            time.sleep(self.dt)
        assert self._state is not None
        self._preflight_or_raise(self._state)

        # Always verify vs captured plant before any torque (even hold).
        self._verify_near_plant(self._state.joint_position)

        if self.hold_current or not self.allow_stand_blend:
            # Freeze at *current* pose — command exactly what we read.
            self._q_nom = self._state.joint_position.copy()
            self.ik.reset(self._q_nom, plant_q_rad=_standing_q_rad())
            self.safety.set_nominal(self._q_nom)
            if self.enable_motion:
                try:
                    if hasattr(self.bus, "enable_all_torque"):
                        self.bus.enable_all_torque(True)
                except Exception:
                    pass
                self._command_deg(self._q_nom, speed=self.write_speed)
            else:
                self.estimator.set_commanded(self._q_nom)
        else:
            # Opt-in only; requires captured plant (checked above).
            self._q_nom = _standing_q_rad()
            self._smooth_to_stand()
            self.ik.reset(self._q_nom, plant_q_rad=self._q_nom)
            self.safety.set_nominal(self._q_nom)
            self._command_deg(self._q_nom, speed=self.stand_speed)

        self.estimator.reset_episode_filters()
        for _ in range(5):
            self._state = self.estimator.update()
            time.sleep(self.dt)
        # Tipping is a CHANGE in tilt: anchor the trip, the obs, and the
        # reward to the attitude the episode actually started with, so IMU
        # mount bias / a sloped floor doesn't silently consume the safety
        # budget or make goals unsatisfiable.
        self._tilt_ref0 = (self._state.imu_roll, self._state.imu_pitch)
        self.safety.set_tilt_reference(*self._tilt_ref0)
        if self._logger:
            self._logger.start_episode(self._episode)
        info = {
            "episode": self._episode,
            "hold_current": self.hold_current,
            "q_nominal_deg": (self._q_nom * RAD2DEG).tolist(),
            "roll_deg": self._state.imu_roll * RAD2DEG,
            "pitch_deg": self._state.imu_pitch * RAD2DEG,
        }
        return self._obs(self._state), info

    def step(self, action):
        assert self._state is not None
        t0 = time.monotonic()
        clipped, bad = self.safety.validate_action(action)
        if clipped is None:
            q_safe = self.safety._last_safe.copy()
            self._command_deg(q_safe)
            self._state = self.estimator.update()
            pen = float(cfg_get(self.cfg, "reward",
                                "safety_termination_penalty", default=10))
            parts = {"reward_termination": -pen}
            if self._logger:
                self._logger.log(flatten_state_row(
                    episode_id=self._episode, step_id=self._step,
                    state=self._state, action_raw=None, action_clip=None,
                    q_ik=None, q_safe=q_safe, reward=-pen,
                    reward_parts=parts, terminated=True, truncated=False,
                    reason=bad, tick_overrun=False))
            self._step += 1
            return self._obs(self._state), -pen, True, False, {
                "termination_reason": bad, **parts}

        offset = action_to_body_offset(clipped, self.cfg)
        ik = self.ik.solve(offset)
        q_safe, status = self.safety.filter(
            ik.q_rad, self._state, ik_ok=ik.ok, ik_reason=ik.reason,
            action=clipped)

        t_cmd0 = time.monotonic()
        terminated = bool(status.terminate)
        if terminated:
            # Do not keep SyncWriting into a tip — that brown-outs the pack.
            if self.limp_on_terminate:
                self._limp()
        else:
            self._command_deg(q_safe)
        t_cmd = time.monotonic() - t_cmd0

        self._state = self.estimator.update()
        reward, parts = self._reward(self._state, clipped)
        if terminated:
            pen = float(cfg_get(self.cfg, "reward",
                                "safety_termination_penalty", default=10))
            parts["reward_termination"] = -pen
            reward -= pen
        self._step += 1
        truncated = self._step >= self.episode_steps
        if self._logger:
            self._logger.log(flatten_state_row(
                episode_id=self._episode, step_id=self._step,
                state=self._state, action_raw=np.asarray(action, dtype=float),
                action_clip=clipped, q_ik=ik.q_rad, q_safe=q_safe,
                reward=reward, reward_parts=parts, terminated=terminated,
                truncated=truncated, reason=status.reason,
                tick_overrun=False,
                extra={"t_cmd": t_cmd, "t_step": time.monotonic() - t0}))
        self._prev_action = clipped.copy()
        info = {"termination_reason": status.reason, **parts,
                "safety_ok": status.ok}
        return self._obs(self._state), float(reward), terminated, truncated, info

    def close(self) -> None:
        if self._logger:
            self._logger.close()


def open_bus(cfg: dict | None = None):
    """Open preferred MCU bus; stop competing owners first if needed."""
    from mcu_feetech_bus import open_feetech_bus
    port = cfg_get(cfg or {}, "bus", "port", default=None)
    bus, name = open_feetech_bus(port)
    return bus, name


def run_scripted_episodes(
    bus,
    cfg: dict,
    action_fn: Callable[[RobotState, int], np.ndarray],
    *,
    episodes: int = 3,
    log: bool = True,
    label: str = "scripted",
) -> dict:
    """Run timed 50 Hz episodes with a scripted action function."""
    env = HexapodBalanceEnv(bus, cfg, log=log)
    loop = ControlLoop(hz=float(cfg_get(cfg, "control", "hz", default=50)))
    summary = {"episodes": [], "label": label}

    for ep in range(episodes):
        obs, info = env.reset()
        del obs
        terminated = truncated = False
        step_i = 0
        rewards = []
        overruns = [0]

        def work(tick):
            nonlocal step_i, terminated, truncated
            if terminated or truncated:
                return
            state = env._state
            assert state is not None
            action = action_fn(state, step_i)
            _obs, rew, terminated, truncated, _info = env.step(action)
            rewards.append(rew)
            if tick.overrun:
                overruns[0] += 1
            step_i += 1
            if step_i >= env.episode_steps:
                truncated = True

        # Drive steps via env.step self-timing; still use loop for overrun stats
        # around a thin wrapper.
        t_end = time.monotonic() + env.episode_steps * env.dt + 2.0
        while (not terminated and not truncated
               and time.monotonic() < t_end
               and step_i < env.episode_steps):
            t0 = time.monotonic()
            state = env._state
            assert state is not None
            action = action_fn(state, step_i)
            _obs, rew, terminated, truncated, _info = env.step(action)
            rewards.append(rew)
            step_i += 1
            # Absolute-ish pacing
            remain = env.dt - (time.monotonic() - t0)
            if remain > 0:
                time.sleep(remain)
            else:
                overruns[0] += 1

        summary["episodes"].append({
            "episode": info.get("episode"),
            "steps": step_i,
            "reward_sum": float(sum(rewards)),
            "overruns": overruns[0],
            "terminated": terminated,
            "truncated": truncated,
        })
    env.close()
    del loop
    return summary
