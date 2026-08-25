"""Canonical RL walk start pose.

RL walk policies should start from the pose family used by the simulator's
normal walk resets, not from the mutable ``plant_pose.json`` calibration file.
Calibration can still measure/contact-fit a plant for diagnostics, but it must
not silently redefine the policy's episode frame.
"""
from __future__ import annotations

N_JOINTS = 18

# SimHexapod* walk resets use the canonical plant [yaw=0, hip=20, knee=80]
# when no explicit training override is provided. The baked STEP stand-up ends
# almost exactly here, so the hardware handoff should hold/verify this pose
# rather than driving down into a captured geometry/contact plant.
SIM_WALK_START_HIP_DEG = 20.0
SIM_WALK_START_KNEE_DEG = 80.0


def walk_start_pose_degrees() -> list[float]:
    return [0.0, SIM_WALK_START_HIP_DEG, SIM_WALK_START_KNEE_DEG] * 6

