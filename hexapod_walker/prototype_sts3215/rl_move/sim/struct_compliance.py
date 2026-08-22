"""Structural compliance: load-dependent leg deflection.

The physical robot shows visible load-dependent deformation of printed
brackets/boots/links while standing, so:

    actual leg pose = servo-reported pose + load-dependent deflection

while a rigid MuJoCo joint reports only one angle. This module models
that gap with a deliberately low-dimensional approximation: each joint's
structural path is a torsional spring in series with the servo. The
nominal stiffnesses in ``rl_move/config.yaml`` are first-pass, measured-
geometry-informed estimates; refine them with the protocol in
``rl_docs/COMPLIANCE.md`` as better data arrives.

Model (v1, deliberately simple — the directive: "a randomized
rotational spring/damper or equivalent compliant mount is sufficient
initially"): each joint's structural path (bracket + printed part +
bearing) is a torsional spring of stiffness k [N·m/rad] in SERIES with
the servo. Quasi-static:

    deflection = tau / k            (per joint, tau = actuator torque)

Two integration surfaces, both 2-3 lines at the call site (kept out of
sim_env.py on purpose while the mode-seq work is in flight there;
wire-up is a follow-up commit):

1. OBS SIDE (encoder-vs-reality gap): the servo encoder measures the
   servo shaft, the structure deflects downstream, so the REPORTED
   position understates the true pose under load. In
   `sim_env._read_state`, after reading physical joint positions:

       q_reported = comp.reported_q(q_phys, tau)     # q_phys - tau/k

   where tau = data.qfrc_actuator at the joint DOF addresses.

2. COMMAND/PHYSICS SIDE ("equivalent compliant mount"): a series
   spring makes the effective position-servo stiffness

       kp_eff = kp * k / (kp + k)

   `apply_effective_kp(model, ...)` rescales actuator gains in place
   (same surface as domain_rand's kp_scale, applied after it).

Domain randomization: `sample(rng, scale=dr_scale)` draws per-axis
stiffness scales from cfg-declared ranges so PPO trains across the
plausible band. The randomization band shrinks toward nominal when
``--dr-scale`` is below 1, matching the rest of the model DR.
Config keys (rl_move/config.yaml section `struct_comp`, ALL required
once enabled=1):

    struct_comp.enabled            0/1 (default 0 = rigid, bit-exact)
    struct_comp.k_yaw_nm_rad       measured yaw-axis stiffness
    struct_comp.k_hip_nm_rad      measured hip-axis stiffness
    struct_comp.k_knee_nm_rad     measured knee-axis stiffness
    struct_comp.dr_scale_lo/hi    randomization band (e.g. 0.5 / 2.0)

Unit tests: rl_move/tests/test_struct_compliance.py.
"""
from __future__ import annotations

import numpy as np

try:
    from rl_move.config import cfg_get
except ImportError:  # direct-script import path
    from config import cfg_get

N_JOINTS = 18
_AXES = ("yaw", "hip", "knee")


class StructCompliance:
    """Quasi-static series-elastic structural deflection for 18 joints.

    Construct via :meth:`from_cfg`; returns None when disabled so call
    sites stay one-line (``if comp is not None: ...``).
    """

    def __init__(self, k_axis_nm_rad: dict[str, float],
                 dr_lo: float = 1.0, dr_hi: float = 1.0):
        for ax in _AXES:
            k = float(k_axis_nm_rad[ax])
            if not (k > 0.0):
                raise ValueError(
                    f"struct_comp: k_{ax}_nm_rad must be > 0 "
                    f"(got {k!r}) — enter the MEASURED stiffness, see "
                    f"rl_docs/COMPLIANCE.md")
        self._k_axis = {ax: float(k_axis_nm_rad[ax]) for ax in _AXES}
        self._dr_lo = float(dr_lo)
        self._dr_hi = float(dr_hi)
        if not (self._dr_lo > 0.0 and self._dr_hi > 0.0
                and self._dr_lo <= self._dr_hi):
            raise ValueError(
                "struct_comp dr_scale_lo/hi must satisfy "
                f"0 < lo <= hi (got {self._dr_lo}, {self._dr_hi})")
        # Per-joint stiffness vector, axis pattern [yaw, hip, knee] x 6.
        self._k_base = np.array(
            [self._k_axis[_AXES[j % 3]] for j in range(N_JOINTS)],
            dtype=float)
        self.k = self._k_base.copy()

    # ---- construction ---------------------------------------------------
    @classmethod
    def from_cfg(cls, cfg: dict) -> "StructCompliance | None":
        if float(cfg_get(cfg, "struct_comp", "enabled",
                         default=0.0)) != 1.0:
            return None
        ks = {}
        for ax in _AXES:
            v = cfg_get(cfg, "struct_comp", f"k_{ax}_nm_rad",
                        default=None)
            if v is None:
                raise ValueError(
                    "struct_comp.enabled=1 but struct_comp."
                    f"k_{ax}_nm_rad is missing. The directive forbids "
                    "silent stiffness values: run the COMPLIANCE.md "
                    "measurement protocol or enter deliberate estimates "
                    "(or set struct_comp.enabled=0).")
            ks[ax] = float(v)
        return cls(
            ks,
            dr_lo=float(cfg_get(cfg, "struct_comp", "dr_scale_lo",
                                default=1.0)),
            dr_hi=float(cfg_get(cfg, "struct_comp", "dr_scale_hi",
                                default=1.0)))

    # ---- domain randomization -------------------------------------------
    @property
    def nominal_k(self) -> np.ndarray:
        return self._k_base.copy()

    def sample(self, rng: np.random.Generator, *,
               scale: float = 1.0) -> np.ndarray:
        """Draw this episode's per-axis stiffness scales (one scale per
        axis class, shared by all six legs: the printed parts came off
        the same printer; per-leg asymmetry waits for measurements).

        ``scale`` follows the env's DR curriculum: 0 returns nominal
        stiffness, 1 uses the full cfg band.
        """
        s = max(0.0, min(1.0, float(scale)))
        if s <= 0.0:
            self.k = self._k_base.copy()
            return self.k.copy()
        lo = 1.0 + (self._dr_lo - 1.0) * s
        hi = 1.0 + (self._dr_hi - 1.0) * s
        scales = {ax: float(rng.uniform(lo, hi))
                  for ax in _AXES}
        self.k = self._k_base * np.array(
            [scales[_AXES[j % 3]] for j in range(N_JOINTS)])
        return self.k.copy()

    # ---- the two integration surfaces -------------------------------
    def deflection(self, tau_nm: np.ndarray,
                   k: np.ndarray | None = None) -> np.ndarray:
        """Quasi-static structural deflection [rad] under joint torque
        tau [N·m] (sign: deflection is in the direction the load
        pushes, i.e. the structure yields WITH the torque)."""
        kk = self.k if k is None else np.asarray(k, dtype=float)
        return np.asarray(tau_nm, dtype=float) / kk

    def reported_q(self, q_phys: np.ndarray,
                   tau_nm: np.ndarray,
                   k: np.ndarray | None = None) -> np.ndarray:
        """Servo-encoder view of a physically-deflected pose.

        actual = reported + deflection  =>  reported = actual − tau/k.
        Feed the PHYSICAL joint positions and the actuator torques;
        returns what the encoder (and therefore obs + the hardware log
        comparison) would read."""
        return np.asarray(q_phys, dtype=float) - self.deflection(tau_nm, k)

    def effective_kp(self, kp: np.ndarray,
                     k: np.ndarray | None = None) -> np.ndarray:
        """Series combination of servo position stiffness and the
        structural spring: kp_eff = kp·k / (kp + k)."""
        kp = np.asarray(kp, dtype=float)
        kk = self.k if k is None else np.asarray(k, dtype=float)
        return kp * kk / (kp + kk)

    def apply_effective_kp(self, model, actuator_kp_rows,
                           k: np.ndarray | None = None) -> None:
        """Rescale MuJoCo position-actuator gains in place (the
        "equivalent compliant mount"). ``actuator_kp_rows`` = actuator
        indices in joint order (the same rows apply_params_to_model
        writes). Call AFTER domain_rand's kp_scale so DR composes."""
        kp = model.actuator_gainprm[actuator_kp_rows, 0]
        model.actuator_gainprm[actuator_kp_rows, 0] = \
            self.effective_kp(kp, k)
        # biasprm[1] mirrors -kp for MuJoCo position actuators.
        model.actuator_biasprm[actuator_kp_rows, 1] = \
            -model.actuator_gainprm[actuator_kp_rows, 0]

    def summary(self, k: np.ndarray | None = None) -> dict:
        kk = self.k if k is None else np.asarray(k, dtype=float)
        out = {}
        for ax_i, ax in enumerate(_AXES):
            vals = kk[ax_i::3]
            out[ax] = {
                "nominal_nm_rad": round(float(self._k_axis[ax]), 1),
                "episode_nm_rad": round(float(np.mean(vals)), 1),
            }
        return out
