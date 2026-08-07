"""STS3215 sensor model for MuJoCo ↔ real-bus parity.

Mirrors the feedback packet returned by
``prototype_sts3215/motor_setup/feetech_bus.FeetechBus.read_feedback``:

    deg, load_pct, volt, temp_c, current_a

so an RL policy trained here can consume the same channels the Uno Q
reads over the half-duplex TTL bus.  Torque → current / load mapping
matches ``prototype_sts3215/standup_current_sim.py`` (FEETECH ST-3215-C018
@ 12 V: stall 30 kg·cm ≈ 2.94 N·m, stall current ≈ 2.7 A).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Electrical / mechanical nameplate (12 V rail).
TAU_STALL_NM = 2.94
I_IDLE_A = 0.20
I_STALL_A = 2.70
V_NOMINAL = 12.0
V_SAG_PER_AMP = 0.035          # crude bus sag under load
TEMP_AMBIENT_C = 28.0
TEMP_RISE_PER_W = 4.5          # crude thermal gain (°C per electrical watt)
SERVO_R_OHM = V_NOMINAL / I_STALL_A   # ~4.4 Ω equivalent for I²R heating

# Encoder: 12-bit over 360°, centre = 2048 (matches feetech_bus.py).
STS_COUNTS_PER_REV = 4096
STS_CENTRE_COUNT = 2048
COUNTS_PER_RAD = STS_COUNTS_PER_REV / (2.0 * np.pi)

# Present-load register: bit10 = direction, lower 10 bits = magnitude in
# 0.1% units → load_pct ∈ [0, 100].
LOAD_PCT_FULL = 100.0


@dataclass
class SensorNoiseConfig:
    """Per-episode (or fixed) measurement noise / bias for domain rand."""
    pos_std_rad: float = 0.008          # ~0.5 deg encoder + mount play
    vel_std_rad_s: float = 0.05
    load_std_pct: float = 3.0
    current_std_a: float = 0.04
    volt_std_v: float = 0.08
    temp_std_c: float = 0.5
    load_bias_pct: float = 0.0          # constant offset (miscalibrated load)
    current_bias_a: float = 0.0
    dropout_prob: float = 0.0           # chance a joint reports NaN→hold-last


@dataclass
class JointFeedback:
    """One joint's STS-style feedback sample."""
    deg: float
    load_pct: float
    volt: float
    temp_c: float
    current_a: float
    speed_deg_s: float


def torque_to_current_a(tau_nm: float) -> float:
    """Linear |tau| → current model, capped at stall."""
    frac = min(1.0, abs(float(tau_nm)) / TAU_STALL_NM)
    return min(I_IDLE_A + (I_STALL_A - I_IDLE_A) * frac, I_STALL_A)


def torque_to_load_pct(tau_nm: float) -> float:
    """Present-load magnitude as percent of stall (matches STS load register)."""
    return min(LOAD_PCT_FULL, abs(float(tau_nm)) / TAU_STALL_NM * LOAD_PCT_FULL)


def quantize_position_rad(q_rad: float) -> float:
    """Round a joint angle through the 12-bit STS encoder and back."""
    count = STS_CENTRE_COUNT + q_rad * COUNTS_PER_RAD
    count = int(np.clip(np.rint(count), 0, STS_COUNTS_PER_REV - 1))
    return (count - STS_CENTRE_COUNT) / COUNTS_PER_RAD


class StsSensorBank:
    """Track thermal state + emit noisy STS feedback for all 18 joints."""

    N_JOINTS = 18

    def __init__(self, noise: SensorNoiseConfig | None = None):
        self.noise = noise or SensorNoiseConfig()
        self._temp_c = np.full(self.N_JOINTS, TEMP_AMBIENT_C, dtype=np.float64)
        self._last_fb = np.zeros((self.N_JOINTS, 6), dtype=np.float64)
        # Columns: deg, load_pct, volt, temp_c, current_a, speed_deg_s
        self._rng = np.random.default_rng()

    def reset(self, rng: np.random.Generator | None = None):
        if rng is not None:
            self._rng = rng
        self._temp_c[:] = TEMP_AMBIENT_C + self._rng.normal(
            0.0, 1.5, size=self.N_JOINTS
        )
        self._last_fb[:] = 0.0

    def sample(
        self,
        q_rad: np.ndarray,
        qd_rad_s: np.ndarray,
        tau_nm: np.ndarray,
        *,
        dt: float,
    ) -> np.ndarray:
        """Return (18, 6) feedback array: deg, load%, V, °C, A, deg/s.

        ``tau_nm`` should be the net actuator torque at each hinge
        (``data.qfrc_actuator[dof]``), matching standup_current_sim.
        """
        q_rad = np.asarray(q_rad, dtype=np.float64).reshape(self.N_JOINTS)
        qd_rad_s = np.asarray(qd_rad_s, dtype=np.float64).reshape(self.N_JOINTS)
        tau_nm = np.asarray(tau_nm, dtype=np.float64).reshape(self.N_JOINTS)
        n = self.noise
        rng = self._rng

        # Ideal (physics → STS register) values.
        q_enc = np.array([quantize_position_rad(q) for q in q_rad])
        deg = np.degrees(q_enc)
        speed_deg_s = np.degrees(qd_rad_s)
        load = np.array([torque_to_load_pct(t) for t in tau_nm])
        current = np.array([torque_to_current_a(t) for t in tau_nm])
        trunk_a = float(current.sum())
        volt = V_NOMINAL - V_SAG_PER_AMP * trunk_a
        volt = max(9.5, volt)  # undervoltage floor (empty pack / brownout)

        # Simple thermal: I²R heating + Newton's cooling toward ambient.
        power_w = (current ** 2) * SERVO_R_OHM * 0.15  # geared motor waste fraction
        self._temp_c += dt * (
            TEMP_RISE_PER_W * power_w - 0.35 * (self._temp_c - TEMP_AMBIENT_C)
        )
        temp = self._temp_c.copy()

        # Measurement noise / bias (what the policy actually sees).
        deg = deg + np.degrees(rng.normal(0.0, n.pos_std_rad, self.N_JOINTS))
        speed_deg_s = speed_deg_s + np.degrees(
            rng.normal(0.0, n.vel_std_rad_s, self.N_JOINTS)
        )
        load = np.clip(
            load + n.load_bias_pct + rng.normal(0.0, n.load_std_pct, self.N_JOINTS),
            0.0, LOAD_PCT_FULL,
        )
        current = np.clip(
            current + n.current_bias_a
            + rng.normal(0.0, n.current_std_a, self.N_JOINTS),
            0.0, I_STALL_A * 1.15,
        )
        volts = np.full(self.N_JOINTS, volt) + rng.normal(
            0.0, n.volt_std_v, self.N_JOINTS
        )
        temp = temp + rng.normal(0.0, n.temp_std_c, self.N_JOINTS)

        fb = np.stack([deg, load, volts, temp, current, speed_deg_s], axis=1)

        if n.dropout_prob > 0.0:
            drop = rng.random(self.N_JOINTS) < n.dropout_prob
            fb[drop] = self._last_fb[drop]

        self._last_fb = fb
        return fb

    def as_dicts(self, fb: np.ndarray) -> list[dict]:
        """Match ``FeetechBus.read_feedback`` field names (plus speed)."""
        out = []
        for j in range(self.N_JOINTS):
            out.append({
                "joint": j,
                "id": j + 2,  # SERVO_ID_OFFSET
                "deg": float(fb[j, 0]),
                "load_pct": float(fb[j, 1]),
                "volt": float(fb[j, 2]),
                "temp_c": float(fb[j, 3]),
                "current_a": float(fb[j, 4]),
                "speed_deg_s": float(fb[j, 5]),
            })
        return out
