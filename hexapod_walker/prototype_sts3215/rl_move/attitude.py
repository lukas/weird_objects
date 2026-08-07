"""Chassis roll/pitch from MPU-6050 (accel + gyro). No magnetometer.

Persistent physical level comes from calibrated accel (``imu_calib.json``
applied by ``McuFeetechBus.read_imu``). Episode reset clears filter
transients only — it must NOT redefine a tilted start as zero.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


G0 = 9.80665  # m/s²


@dataclass
class AttitudeEstimate:
    roll: float = 0.0   # rad, +roll = right side down (right-handed, X fwd)
    pitch: float = 0.0  # rad, +pitch = nose up
    yaw: float = 0.0    # integrated only; unused by Phase-1 policy


class ComplementaryAttitude:
    """Simple complementary filter.

    Accel axes: sensor frame after bias removal, units g.
    Gyro: rad/s in the same frame (converted from dps by caller).

    Convention (robot chassis, Z up, X forward-ish along +X flat):
      roll  = atan2(ay, az)
      pitch = atan2(-ax, hypot(ay, az))
    """

    def __init__(self, *, alpha: float = 0.98):
        """``alpha`` close to 1 trusts gyro more; accel corrects slowly."""
        self.alpha = float(alpha)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._inited = False

    def reset_transients(self) -> None:
        """Clear integrator bookkeeping; keep last angles (physical level)."""
        # Re-seed from next accel sample without zeroing to episode pose.
        self._inited = False

    def update(self, accel_g: tuple[float, float, float],
               gyro_rps: tuple[float, float, float],
               dt: float) -> AttitudeEstimate:
        ax, ay, az = (float(accel_g[0]), float(accel_g[1]), float(accel_g[2]))
        gx, gy, gz = (float(gyro_rps[0]), float(gyro_rps[1]), float(gyro_rps[2]))
        dt = max(0.0, float(dt))

        # Accel tilt (valid when mostly gravity).
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 1e-6:
            accel_roll = self.roll
            accel_pitch = self.pitch
        else:
            axn, ayn, azn = ax / a_norm, ay / a_norm, az / a_norm
            accel_roll = math.atan2(ayn, azn)
            accel_pitch = math.atan2(-axn, math.hypot(ayn, azn))

        if not self._inited or dt <= 0.0:
            self.roll = accel_roll
            self.pitch = accel_pitch
            self._inited = True
        else:
            # Gyro integrate (small-angle; adequate for balance rates).
            roll_g = self.roll + gx * dt
            pitch_g = self.pitch + gy * dt
            self.yaw = self.yaw + gz * dt
            a = self.alpha
            self.roll = a * roll_g + (1.0 - a) * accel_roll
            self.pitch = a * pitch_g + (1.0 - a) * accel_pitch

        return AttitudeEstimate(self.roll, self.pitch, self.yaw)
