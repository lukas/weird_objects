"""Sit / stand postures for the STS3215 hexapod.

Sit = legs sprawled out wide, chassis low (the pose you put the robot
down in, or that it settles to when you ask it to sit).
Stand = the normal tripod stance used for walking.

Nominal transitions are a cosine blend over a few seconds so a zero-
residual policy already stands up / sits down without slamming.  The RL
policy's job is to keep currents/loads gentle and recover if a foot
catches.
"""

from __future__ import annotations

import math

import numpy as np

import mujoco_prototype as MP

# Standing stance (matches mujoco_prototype / feetech_bus.standing_pose).
STAND_YAW = 0.0
STAND_PITCH = float(MP.STANCE_FEMUR)     # ≈ -25 deg
STAND_KNEE = float(MP.STANCE_TIBIA)      # ≈ +60 deg

# Legs-out-wide sit: femurs flatter than stance, knees more open so the
# feet plant farther from the chassis and the belly sits low (~12 mm).
SIT_YAW = 0.0
SIT_PITCH = -0.15                        # ≈ -8.6 deg (flatter femur)
SIT_KNEE = 0.35                          # ≈ +20 deg (more open than stance)

# Soft electrical ceilings used during posture episodes.  Tuned so a
# slow cosine stand-up on the realistic-mass robot stays near the soft
# band (~1.1–1.3 A/motor, ~12–15 A bus); slamming past hard truncates.
POSTURE_TORQUE_CAP_NM = 2.10             # ~71% of stall
I_SOFT_A = 1.05                          # per-servo soft (reward knee)
I_HARD_A = 1.80                          # per-servo hard (truncate)
I_TRUNK_SOFT_A = 12.0                    # whole-bus soft
I_TRUNK_HARD_A = 18.0                    # whole-bus hard (truncate)


def stand_joints() -> np.ndarray:
    """(18,) yaw/pitch/knee per leg at the standing stance."""
    out = np.empty(18, dtype=np.float64)
    for i in range(6):
        out[3 * i + 0] = STAND_YAW
        out[3 * i + 1] = STAND_PITCH
        out[3 * i + 2] = STAND_KNEE
    return out


def sit_joints() -> np.ndarray:
    """(18,) legs-out-wide sit pose."""
    out = np.empty(18, dtype=np.float64)
    for i in range(6):
        out[3 * i + 0] = SIT_YAW
        out[3 * i + 1] = SIT_PITCH
        out[3 * i + 2] = SIT_KNEE
    return out


def blend_joints(a: np.ndarray, b: np.ndarray, u: float) -> np.ndarray:
    """Cosine-eased blend, u in [0, 1]."""
    u = float(np.clip(u, 0.0, 1.0))
    s = 0.5 - 0.5 * math.cos(math.pi * u)
    return (1.0 - s) * a + s * b


def posture_target_z(cmd: float) -> float:
    """Approximate chassis height for a posture command in [0, 1].

    0 = sit (low), 1 = stand.  Used for reward shaping; the true height
    comes from the sim.  Stand target is slightly below the kinematic
    ideal to account for soft-PD sag under the posture torque cap.
    """
    # Empirically settled heights under the soft PD + posture torque cap.
    z_sit = 0.012
    z_stand = 0.038
    return z_sit + float(np.clip(cmd, 0.0, 1.0)) * (z_stand - z_sit)


def set_joint_qpos(model, data, joints: np.ndarray, *, chassis_z: float | None = None):
    """Write an 18-vector of joint angles into mjData and drop the chassis."""
    import mujoco
    for i in range(6):
        for k, kind in enumerate(("yaw", "pitch", "knee")):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{kind}")
            data.qpos[model.jnt_qposadr[j]] = float(joints[3 * i + k])
    if chassis_z is not None:
        data.qpos[2] = float(chassis_z)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
