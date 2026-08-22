"""SIM-side knee-convention adapters for the scripted gait stack.

Since commit 30660b51 ("Use measured stand geometry for calibration and
gaits") the hardware control stack speaks the measured robot's
ABSOLUTE-tibia convention (``tripod_gait.KNEE_ANGLE_CONVENTION ==
"absolute_tibia"``): stand homes (``feetech_bus.standing_pose_degrees``,
default +19/+28), learned plant poses, ``_leg_ik`` and every gait's
``desired_deg`` all carry the tibia angle measured in the leg plane.

The MuJoCo twin's knee hinge is RELATIVE to the femur
(tibia_abs = hip + knee_rel; see
``mujoco_prototype.stance_foot_z_relative_to_hip``), and every trained
checkpoint, reward term and eval harness speaks that joint space.
Feeding absolute-tibia angles into the sim silently mis-poses every
scripted rollout (semantics-bank regression, 2026-08-22 bisect: onset
exactly at 30660b51; probes green at a4beb8af).

This module is the ONE conversion boundary. rl_move / sim consumers
import the gait classes and the stand home from here instead of from the
hardware modules:

    * ``sync_plant_stance(hip_deg, knee_deg)`` takes the SIM-RELATIVE
      knee (the pre-30660b51 contract, e.g. the canonical +20/+80 walk
      stance),
    * ``desired_deg()`` / ``neutral_pose_deg()`` return SIM-RELATIVE
      knees ready to feed to the MuJoCo joint targets,
    * ``standing_pose_degrees()`` re-exports the hardware stand home
      converted to sim-relative knees.

Hardware behaviour is untouched: nothing in linux_control imports this
file, and the hardware classes are subclassed, never modified.
"""
from __future__ import annotations

import tripod_gait as _tg
import noslip_gait as _ng
from feetech_bus import standing_pose_degrees as _standing_abs_deg

__all__ = [
    "TripodGait", "NoSlipGait",
    "knee_rel_18", "knee_abs_18", "standing_pose_degrees",
]


def knee_rel_18(deg18) -> list[float]:
    """18 joint angles absolute-tibia -> sim-relative knee (degrees or
    radians — the conversion is a plain subtraction per leg)."""
    out = [float(v) for v in deg18]
    for i in range(6):
        out[3 * i + 2] -= out[3 * i + 1]
    return out


def knee_abs_18(deg18) -> list[float]:
    """Inverse of :func:`knee_rel_18`."""
    out = [float(v) for v in deg18]
    for i in range(6):
        out[3 * i + 2] += out[3 * i + 1]
    return out


def standing_pose_degrees() -> list[float]:
    """Hardware stand home (learned plant or default +19/+28 absolute)
    converted to sim-relative knees."""
    return knee_rel_18(_standing_abs_deg())


class _SimRelKneeMixin:
    """Convert the gait class's absolute-tibia surface to sim-relative."""

    def sync_plant_stance(self, hip_deg: float | None = None,
                          knee_deg: float | None = None):
        if hip_deg is None and knee_deg is None:
            # Load the hardware plant (absolute convention) unchanged —
            # the base class interprets it correctly.
            return super().sync_plant_stance()
        if hip_deg is None or knee_deg is None:
            raise ValueError(
                "sim_gait_compat: pass BOTH hip_deg and knee_deg "
                "(sim-relative) or neither")
        return super().sync_plant_stance(
            float(hip_deg), float(hip_deg) + float(knee_deg))

    def desired_deg(self, t: float) -> list[float]:
        return knee_rel_18(super().desired_deg(t))


class TripodGait(_SimRelKneeMixin, _tg.TripodGait):
    """tripod_gait.TripodGait with the sim-relative knee contract."""

    def neutral_pose_deg(self) -> list[float]:
        return knee_rel_18(super().neutral_pose_deg())


class NoSlipGait(_SimRelKneeMixin, _ng.NoSlipGait):
    """noslip_gait.NoSlipGait with the sim-relative knee contract."""
