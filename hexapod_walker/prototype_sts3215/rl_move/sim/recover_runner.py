"""recover_runner.py — DEPLOYMENT-GRADE runner for the recover mode.

Operator order 2026-08-20 (MCP operator lane, GPT-5 relay for Lukas):
make the predictive-context recovery model usable/production-ready,
sim/deploy readiness only. This module is the runner/API observation
contract for recover mode; the sim gate that exercises it end-to-end
is ``eval_recover_runner.py`` (23-rung ladder through THIS code path,
not the training env's obs builder).

Champion: ``ppo_goal_cw_recover_predictive1b_pop3_s13`` — the only
predictive1b cohort member that clears ALL 23 rungs including flip on
own-DR det (s11/s12 both fail flip; ledger verdicts 08-18/19).

THE OBSERVATION CONTRACT (what the policy trained on, and therefore
what any runner must feed it — sim_env/walk_task are the source of
truth, parity is test-locked by tests/test_recover_runner.py):

  * 16 frames x 90 dims, NEWEST FIRST, flattened to 1440.
  * frame = [59 proprio | 11 goal (all zeros in recover) |
             2 vel (:=ref copy = zeros) | 18 plant-relative q]
    - proprio: (q - q_nom)/q_scale, qd/qd_scale,
      (roll,pitch - tilt_ref)/tilt_scale, gyro/gyro_scale,
      prev_action (the last CLIPPED [-1,1] action; zeros before the
      first policy tick).
    - q_nom = the joint pose captured at mode ENTRY (hold-current
      semantics: encoders read at the settled fallen pose).
    - tilt_ref is LEVEL (gravity truth), NOT the fallen attitude:
      recover episodes train with ``_tipped_applied`` semantics —
      only IMU mount/bias offset belongs in the reference. On
      hardware that means "calibrated level-IMU bias", default (0,0).
    - plant-relative q = (q - q_plant)/q_scale, the task-stable pose
      frame (obs.recover_plant_q=1).
  * RESET-HISTORY PROBE (obs.reset_history_probe=1): the policy's
    first decision sees 15 REAL sensor frames + the entry frame, not
    16 copies of one frame. The runner reproduces this as an ENTRY
    HOLD: command the captured pose for 15 ticks (0.6 s @ 25 Hz)
    while pushing real frames; prev_action stays zeros throughout.
  * Actions: 18 raw joint targets in [-1,1] via joint_task
    ``action_to_q_rad`` through the SafetyLayer. Safety tilt envelope
    must be widened in recover mode (trained contract:
    safety.max_roll_deg=185, max_pitch_deg=185 — a fall is a
    recoverable state, not a termination). Current limits stay.

MODE GATING / MANUAL COMMAND SEMANTICS (mirrors web_session
_do_recover; operator ask "safe mode gating"):
  * recover NEVER auto-triggers: ``start()`` is an explicit manual
    command from the session controller / operator.
  * entry is REFUSED while the body is still tumbling (gyro not quiet
    for ``quiet_ticks`` consecutive ticks) — "recover after it lands".
  * active budget defaults to the trained 16 s episode; on timeout
    the runner reports TIMEOUT and freezes on the last safe command
    (the caller decides what to do next; nothing auto-retries).
  * SUCCESS is a deploy-side detector (attitude level + joints quiet
    + pose near plant, held 1.0 s — 2x the trained rec_hold_s, after
    a transiently-level flip partial recovery false-fired at 0.5 s
    in the 08-20 DR-0 gate); the
    recommended handoff is the STANCE HOLD policy, exactly like the
    session controller's STOP route.
"""
from __future__ import annotations

import hashlib
from collections import deque
from pathlib import Path

import numpy as np

from rl_move.config import cfg_get, load_config
from rl_move.robot_state import DEG2RAD, N_JOINTS, RobotState

_PROTO = Path(__file__).resolve().parents[2]

# ---------------------------------------------------------------------------
# Package manifest (the "download answer" for recover mode)
# ---------------------------------------------------------------------------
CHAMPION_ZIP = (_PROTO / "rl_move" / "sim" / "policies"
                / "ppo_goal_cw_recover_predictive1b_pop3_s13.zip")
CHAMPION_MD5 = "cba811e6a1e63b4b043d65f755356fb4"
ENCODER_PT = (_PROTO / "rl_move" / "dynamics" / "models"
              / "cw-dynrep-tf-state2-recovered1.pt")
ENCODER_MD5 = "9df48f687967c25085ee50171e4110ff"

HISTORY = 16
FRAME_WIDTH = 90
N_OBS_RECOVER = HISTORY * FRAME_WIDTH   # 1440

# The cfg the champion trained with, restricted to keys that change
# runner/env semantics (obs width, spawn safety envelope, vel source).
CONTRACT_CFG = {
    "obs.history_frames": 16.0,
    "obs.reset_history_probe": 1.0,
    "obs.recover_plant_q": 1.0,
    "goal.walk_obs_body_vel": 2.0,
    "safety.max_roll_deg": 185.0,
    "safety.max_pitch_deg": 185.0,
}


def contract_cfg(base_cfg: dict | None = None) -> dict:
    """A config dict carrying the champion's recover contract keys."""
    import copy
    cfg = copy.deepcopy(base_cfg) if base_cfg is not None else load_config()
    for key, val in CONTRACT_CFG.items():
        sect, name = key.split(".", 1)
        cfg.setdefault(sect, {})[name] = val
    return cfg


def _md5(path: Path) -> str:
    return hashlib.md5(Path(path).read_bytes()).hexdigest()


def load_recover_policy(zip_path: Path | str = CHAMPION_ZIP,
                        encoder_path: Path | str = ENCODER_PT,
                        verify_md5: bool = True, device: str = "cpu"):
    """Load the recover champion with its frozen encoder, relocatable.

    The SB3 zip pickles the ABSOLUTE train-pod path of the frozen
    dynamics transformer inside ``policy_kwargs['predictor_ckpt']``
    (PredictiveCriticPolicy._build re-reads that file to construct the
    snapshot modules before the saved state_dict overwrites them), so
    a plain ``PPO.load`` only works on the machine that trained it.
    This loader rewrites the path to a LOCAL encoder copy and verifies
    both artifacts' md5s against the run's live-integration gate.
    """
    from stable_baselines3 import PPO
    from stable_baselines3.common.save_util import load_from_zip_file

    zip_path, encoder_path = Path(zip_path), Path(encoder_path)
    if not zip_path.is_file():
        raise FileNotFoundError(f"policy zip missing: {zip_path}")
    if not encoder_path.is_file():
        raise FileNotFoundError(
            f"frozen encoder missing: {encoder_path} (the SB3 zip does "
            "NOT carry the raw encoder checkpoint; ship both files)")
    if verify_md5:
        got = _md5(encoder_path)
        if got != ENCODER_MD5:
            raise ValueError(f"encoder md5 {got} != gate {ENCODER_MD5}")
    data, _params, _vars = load_from_zip_file(zip_path, device=device)
    pk = dict(data.get("policy_kwargs") or {})
    if "predictor_ckpt" in pk:
        pk["predictor_ckpt"] = str(encoder_path)
    model = PPO.load(zip_path, device=device,
                     custom_objects={"policy_kwargs": pk})
    n_obs = int(np.prod(model.observation_space.shape))
    n_act = int(np.prod(model.action_space.shape))
    if n_obs != N_OBS_RECOVER or n_act != N_JOINTS:
        raise ValueError(
            f"checkpoint contract mismatch: obs {n_obs} (want "
            f"{N_OBS_RECOVER}), act {n_act} (want {N_JOINTS})")
    return model


# ---------------------------------------------------------------------------
# Observation builder
# ---------------------------------------------------------------------------
class RecoverObsBuilder:
    """Assemble the 16x90 recover observation from raw sensor reads.

    Deploy-honest: consumes only quantities a real robot has (encoder
    positions/velocities, IMU attitude + gyro, its own previous
    action) plus two constants (plant pose, level-IMU tilt bias).
    """

    def __init__(self, cfg: dict | None = None,
                 plant_rad: np.ndarray | None = None):
        self.cfg = contract_cfg(cfg) if cfg is None else cfg
        if plant_rad is None:
            from rl_move.sim.sim_env import _default_plant_deg
            plant_rad = _default_plant_deg() * DEG2RAD
        self.plant_rad = np.asarray(plant_rad, dtype=float).copy()
        self.q_scale = float(cfg_get(self.cfg, "obs", "q_scale",
                                     default=1.0))
        self.q_nom: np.ndarray | None = None
        self.tilt_ref = (0.0, 0.0)
        self.prev_action = np.zeros(N_JOINTS, dtype=float)
        self._hist: deque | None = None

    def begin(self, state: RobotState,
              tilt_bias: tuple[float, float] = (0.0, 0.0),
              q_nom: np.ndarray | None = None) -> None:
        """Mode entry: capture q_nom (hold-current), reset history.

        ``tilt_bias`` is the calibrated level-IMU offset (rad) — the
        LEVEL reference of the training contract. Never pass the
        measured fallen attitude here.

        ``q_nom`` defaults to the entry encoder read (deployment
        semantics). The training env captures its nominal at the
        passive-equilibrium tick ~0.3 s before the first obs read;
        sim parity harnesses pass ``env._q_nom`` explicitly.
        """
        self.q_nom = (np.asarray(state.joint_position, dtype=float).copy()
                      if q_nom is None
                      else np.asarray(q_nom, dtype=float).copy())
        self.tilt_ref = (float(tilt_bias[0]), float(tilt_bias[1]))
        self.prev_action = np.zeros(N_JOINTS, dtype=float)
        frame = self._frame(state)
        self._hist = deque([frame.copy() for _ in range(HISTORY)],
                           maxlen=HISTORY)

    def push(self, state: RobotState) -> None:
        """One sensor tick (entry-hold probe ticks AND policy ticks)."""
        if self._hist is None:
            raise RuntimeError("push() before begin()")
        self._hist.appendleft(self._frame(state))   # newest first

    def note_action(self, clipped_action: np.ndarray) -> None:
        """Record the CLIPPED [-1,1] action just sent to the servos."""
        self.prev_action = np.clip(
            np.asarray(clipped_action, dtype=float), -1.0, 1.0)

    def obs(self) -> np.ndarray:
        if self._hist is None:
            raise RuntimeError("obs() before begin()")
        return np.concatenate(list(self._hist)).astype(np.float32)

    def _frame(self, state: RobotState) -> np.ndarray:
        from rl_move.env import build_obs
        from rl_move.sim.walk_task import WalkGoal
        core = build_obs(self.cfg, state, self.q_nom, self.prev_action,
                         goal=WalkGoal(), tilt_ref=self.tilt_ref)
        vel = np.zeros(2)          # walk_obs_body_vel=2: meas := ref = 0
        plant_q = ((np.asarray(state.joint_position, dtype=float)
                    - self.plant_rad) / max(self.q_scale, 1e-6))
        frame = np.concatenate([core, vel, plant_q]).astype(np.float32)
        if frame.shape[0] != FRAME_WIDTH:
            raise ValueError(f"frame width {frame.shape[0]} != "
                             f"{FRAME_WIDTH} — cfg drift?")
        return frame


# ---------------------------------------------------------------------------
# Mode-gated runner
# ---------------------------------------------------------------------------
IDLE, ENTRY, ACTIVE, SUCCESS, TIMEOUT, STOPPED = (
    "idle", "entry", "active", "success", "timeout", "stopped")


class RecoverRunner:
    """Manual-command recover mode with safe gating and handoff.

    Usage per 25 Hz tick (contract mirrors the training reset probe:
    the first POLICY action fires on exactly [f15..f1, f0]):
        ok, why = runner.start(state_0)    # manual command, may refuse
        action = runner.entry_action()     # apply ONCE after start()
        loop:  action = runner.tick(state_k)   # holds for the first 14
               runner.note_action(applied)     # AFTER safety clipping
        if runner.state in (SUCCESS, TIMEOUT, STOPPED): hand off
        (recommended handoff: STANCE HOLD, the session STOP route).

    ``shadow_detector=True`` (gate harnesses): the deploy-side success
    detector records when it WOULD hand off (``detector_fire_tick``)
    but never interrupts the policy, so the sim gate can measure
    ground truth and detector agreement independently.
    """

    def __init__(self, model, cfg: dict | None = None,
                 plant_rad: np.ndarray | None = None, hz: float = 25.0,
                 active_budget_s: float = 16.0,
                 quiet_gyro_rad_s: float = 1.0, quiet_ticks: int = 5,
                 hold_s: float = 1.0, level_deg: float = 6.0,
                 qd_rms_max: float = 0.7, plant_close_deg: float = 15.0,
                 shadow_detector: bool = False):
        self.model = model
        self.builder = RecoverObsBuilder(cfg, plant_rad)
        self.hz = float(hz)
        self.active_budget = int(round(active_budget_s * hz))
        self.quiet_gyro = float(quiet_gyro_rad_s)
        self.quiet_ticks = int(quiet_ticks)
        self.hold_need = max(int(round(hold_s * hz)), 1)
        self.level_deg = float(level_deg)
        self.qd_rms_max = float(qd_rms_max)
        self.plant_close = float(plant_close_deg) * DEG2RAD
        self.shadow_detector = bool(shadow_detector)
        self.detector_fire_tick: int | None = None
        self.state = IDLE
        self._quiet_n = 0
        self._entry_left = 0
        self._active_n = 0
        self._hold_n = 0
        self._hold_action: np.ndarray | None = None

    # -- gating ---------------------------------------------------------
    def observe_idle(self, state: RobotState) -> None:
        """Feed ticks while idle so the quiet gate has history."""
        if float(np.max(np.abs(state.imu_gyro))) <= self.quiet_gyro:
            self._quiet_n += 1
        else:
            self._quiet_n = 0

    def start(self, state: RobotState,
              tilt_bias: tuple[float, float] = (0.0, 0.0),
              force: bool = False,
              q_nom_override: np.ndarray | None = None
              ) -> tuple[bool, str]:
        """Explicit manual command. Refuses while tumbling.

        ``q_nom_override`` is for sim parity harnesses only (see
        RecoverObsBuilder.begin); deployment always captures q_nom
        from the entry encoder read.
        """
        if self.state in (ENTRY, ACTIVE):
            return False, "recover already running"
        self.observe_idle(state)
        if not force and self._quiet_n < self.quiet_ticks:
            return False, "still tumbling - recover after it lands"
        from rl_move.sim.joint_task import q_rad_to_action
        self.builder.begin(state, tilt_bias=tilt_bias,
                           q_nom=q_nom_override)
        self._hold_action = q_rad_to_action(self.builder.q_nom)
        self._entry_left = HISTORY - 1          # 15 real probe frames
        self._active_n = 0
        self._hold_n = 0
        self.detector_fire_tick = None
        self.state = ENTRY
        return True, "recover started (entry hold)"

    def entry_action(self) -> np.ndarray:
        """The hold action to apply for the first tick after start()."""
        if self._hold_action is None:
            raise RuntimeError("entry_action() before start()")
        return self._hold_action.copy()

    def stop(self) -> None:
        """Manual abort; freezes on the hold action."""
        if self.state in (ENTRY, ACTIVE):
            self.state = STOPPED

    # -- per-tick -------------------------------------------------------
    def tick(self, state: RobotState) -> np.ndarray:
        if self.state == ENTRY:
            self.builder.push(state)            # real frame, action 0
            self._entry_left -= 1
            if self._entry_left > 0:
                self._last_was_entry = True
                return self._hold_action.copy()
            # 15th real frame just landed: history is now exactly the
            # training reset obs [f15..f1, f0] — first policy action
            # fires HERE, no extra hold tick.
            self.state = ACTIVE
            self._last_was_entry = False
            return self._predict(state, push=False)
        self._last_was_entry = False
        if self.state == ACTIVE:
            return self._predict(state, push=True)
        # idle / terminal: hold whatever we last committed to.
        if self._hold_action is not None:
            return self._hold_action.copy()
        from rl_move.sim.joint_task import q_rad_to_action
        return q_rad_to_action(np.asarray(state.joint_position))

    def note_action(self, clipped_action: np.ndarray) -> None:
        # Entry-hold ticks keep prev_action = 0 (training probe pays no
        # action); only ACTIVE policy actions enter the obs.
        if not getattr(self, "_last_was_entry", False):
            self.builder.note_action(clipped_action)

    def _predict(self, state: RobotState, push: bool) -> np.ndarray:
        if push:
            self.builder.push(state)
        self._active_n += 1
        action, _ = self.model.predict(self.builder.obs(),
                                       deterministic=True)
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self._update_success(state)
        if self._hold_n >= self.hold_need:
            if self.detector_fire_tick is None:
                self.detector_fire_tick = self._active_n
            if not self.shadow_detector:
                self.state = SUCCESS
        if self.state == ACTIVE and self._active_n >= self.active_budget:
            self.state = TIMEOUT
        return action

    # -- deploy-side success detector ------------------------------------
    def _update_success(self, state: RobotState) -> None:
        level = (abs(float(state.imu_roll)) <= self.level_deg * DEG2RAD
                 and abs(float(state.imu_pitch))
                 <= self.level_deg * DEG2RAD)
        qd_rms = float(np.sqrt(np.mean(
            np.square(np.asarray(state.joint_velocity, dtype=float)))))
        near_plant = bool(np.mean(np.abs(
            np.asarray(state.joint_position, dtype=float)
            - self.builder.plant_rad)) <= self.plant_close)
        quiet = (float(np.max(np.abs(state.imu_gyro))) <= self.quiet_gyro)
        ok = level and quiet and near_plant and qd_rms <= self.qd_rms_max
        self._hold_n = self._hold_n + 1 if ok else 0
