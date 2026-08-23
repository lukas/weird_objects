"""Closed-loop yaw-rate trim for open-loop SE(2) gait controllers.

`paper_cpg_search`/`SE2FootGait` command a body twist (vx, vy, wz) and
integrate it OPEN LOOP: the swing/stance schedule assumes the commanded
angular rate is exactly what the body achieves. `eval_cpg_gate.py`'s
robustness panel found this breaks at lower ground friction (mu 0.8):
turn segments overshoot their target yaw by ~33% (yaw_along_frac
1.33/1.35 vs the gate's [0.70, 1.30] band) while mu 1.2 and mu 0 (the
search's own friction) stay inside band -- an open-loop parameter
search (120-iteration GP, cpg track 08-23) could not find any
(period, swing_frac, lift, cmd_tau, workspace_margin) point that fixes
this, because it is a closed-loop problem: the actual body-to-command
gain depends on friction, which the open-loop schedule cannot sense.

This module is the minimal fix named in `rl_docs/tracks/cpg/STATUS.md`
Next item 2: periodically compare the MEASURED yaw rate (from body yaw
samples -- a gyro z-rate on hardware) against the commanded rate and
trim a multiplicative SCALE on the commanded omega fed into
`SE2FootGait.set_velocity`. It is deliberately just the scale update
law (`update_trim`) -- callers own the control loop (when to sample,
how to call `set_velocity`) so it can be reused by any gait harness
without depending on MuJoCo, `SE2FootGait`, or `eval_cpg_gate`'s
segment bookkeeping. Default caller behavior (no calls into this
module) is unchanged; this file adds zero side effects on import.
"""
from __future__ import annotations


def update_trim(trim_scale: float, measured_wz: float, cmd_wz: float, *,
                 kp: float = 0.6, lo: float = 0.4, hi: float = 1.6) -> float:
    """One proportional-control update of a multiplicative omega trim.

    `trim_scale` is applied by the CALLER as
    ``gait.set_velocity(..., omega=cmd_wz * trim_scale)``. This function
    only updates the scale from one fresh measurement window.

    - `measured_wz`: actual body yaw rate (rad/s) achieved over the
      last sample window, sign-consistent with `cmd_wz`.
    - `cmd_wz`: the segment's target yaw rate (rad/s); if it is ~0
      (no turn commanded) the scale is returned unchanged -- trim only
      acts while a turn is actually commanded, never on straight/stop
      segments.
    - `kp`: proportional gain on the FRACTIONAL rate error
      ``(measured - cmd) / |cmd|``. 0.6 converges an 08-23-scale gain
      error (~33% overshoot) within a handful of update periods without
      overshooting the correction itself (see
      `test_yaw_trim.py::test_converges_from_synthetic_gain_error`).
    - `lo`/`hi`: hard clamp so a noisy single window can never command
      a runaway scale (0.4-1.6x covers everything measured on this
      track to date with margin).

    The update is MULTIPLICATIVE on the scale (``ratio = measured /
    cmd_wz`` is sign-invariant: a CW and a CCW overshoot of the same
    fractional size trim identically) rather than additive on omega,
    because the observed defect is a GAIN error (measured ~= gain *
    commanded), not an offset -- a multiplicative correction converges
    geometrically regardless of the commanded turn's sign or magnitude
    (see `test_yaw_trim.py`'s synthetic-gain-error convergence tests).
    """
    if abs(cmd_wz) < 1e-6:
        return trim_scale
    ratio = measured_wz / cmd_wz
    new_scale = trim_scale * (1.0 - kp * (ratio - 1.0))
    if new_scale < lo:
        return lo
    if new_scale > hi:
        return hi
    return new_scale
