"""Stall detection and safe-stop for STS3215 joints.

A servo is considered stalling when it is near-stopped *and* drawing
near-stall load/current for a sustained window — the same signature
you'd see on the bus when a leg is jammed against an obstacle or a
gearbox is locked.

On trip the guard enters SAFE_STOP:
  1. freeze joint targets at the last measured pose (no more gait push)
  2. after ``hold_pose_seconds``, request torque-off (relax) so the
     motor is not cooking itself against the jam

The RL env treats a trip as a truncated episode with a stall penalty;
the hardware controller calls ``FeetechBus.torque(..., False)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

from sts_sensors import I_STALL_A, LOAD_PCT_FULL


class GuardState(Enum):
    NORMAL = auto()
    STALL_HOLD = auto()   # targets frozen at current pose
    RELAXED = auto()      # torque requested off


@dataclass
class StallConfig:
    # Trip thresholds (AND'd together, must persist ``persist_s``).
    load_pct_thresh: float = 80.0
    current_a_thresh: float = 2.0
    speed_deg_s_thresh: float = 8.0     # ~stopped while commanded
    persist_s: float = 0.12             # must hold for this long
    # After trip.
    hold_pose_seconds: float = 0.25
    then_relax: bool = True
    # Optional: also trip on over-temp.
    temp_c_thresh: float = 70.0
    # How many joints must trip before whole-robot safe-stop.
    # 1 = any joint; raise if you only want multi-joint jam events.
    min_joints: int = 1


@dataclass
class StallGuard:
    cfg: StallConfig = field(default_factory=StallConfig)
    state: GuardState = GuardState.NORMAL
    tripped_joints: np.ndarray = field(
        default_factory=lambda: np.zeros(18, dtype=bool)
    )
    freeze_deg: np.ndarray = field(
        default_factory=lambda: np.zeros(18, dtype=np.float64)
    )
    _accum_s: np.ndarray = field(
        default_factory=lambda: np.zeros(18, dtype=np.float64)
    )
    _hold_timer_s: float = 0.0
    relax_requested: bool = False

    def reset(self):
        self.state = GuardState.NORMAL
        self.tripped_joints[:] = False
        self.freeze_deg[:] = 0.0
        self._accum_s[:] = 0.0
        self._hold_timer_s = 0.0
        self.relax_requested = False

    def _candidate_mask(self, fb: np.ndarray) -> np.ndarray:
        """fb columns: deg, load%, V, °C, A, deg/s."""
        load = fb[:, 1]
        temp = fb[:, 3]
        current = fb[:, 4]
        speed = np.abs(fb[:, 5])
        near_stall = (
            (load >= self.cfg.load_pct_thresh)
            & (current >= self.cfg.current_a_thresh)
            & (speed <= self.cfg.speed_deg_s_thresh)
        )
        over_temp = temp >= self.cfg.temp_c_thresh
        return near_stall | over_temp

    def update(self, fb: np.ndarray, dt: float) -> GuardState:
        """Advance the FSM with one STS feedback sample (18, 6)."""
        fb = np.asarray(fb, dtype=np.float64)
        if self.state == GuardState.RELAXED:
            return self.state

        if self.state == GuardState.STALL_HOLD:
            self._hold_timer_s += dt
            if self.cfg.then_relax and self._hold_timer_s >= self.cfg.hold_pose_seconds:
                self.state = GuardState.RELAXED
                self.relax_requested = True
            return self.state

        # NORMAL — accumulate near-stall time per joint.
        cand = self._candidate_mask(fb)
        self._accum_s[cand] += dt
        self._accum_s[~cand] = 0.0
        tripped = self._accum_s >= self.cfg.persist_s
        if int(tripped.sum()) >= self.cfg.min_joints:
            self.tripped_joints = tripped
            self.freeze_deg = fb[:, 0].copy()
            self.state = GuardState.STALL_HOLD
            self._hold_timer_s = 0.0
        return self.state

    def is_safe_stopped(self) -> bool:
        return self.state in (GuardState.STALL_HOLD, GuardState.RELAXED)

    def freeze_targets_rad(self) -> np.ndarray:
        return np.radians(self.freeze_deg)


def stall_severity(fb: np.ndarray) -> float:
    """Scalar in [0, 1] for reward shaping (max joint near-stall fraction)."""
    load_frac = fb[:, 1] / LOAD_PCT_FULL
    cur_frac = fb[:, 4] / I_STALL_A
    speed = np.abs(fb[:, 5])
    # High when loaded AND slow.
    slow = np.clip(1.0 - speed / 40.0, 0.0, 1.0)
    return float(np.max(0.5 * (load_frac + cur_frac) * slow))
