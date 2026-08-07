"""Fit MuJoCo actuator params from the hardware motor-dynamics battery.

Reads ``linux_control/logs/motor_model.json`` (output of
``motor_dynamics.run_motor_dynamics``) and, per axis (yaw / hip / knee):

- copies the directly-measured quantities: latency (delay_ms), velocity
  ceiling (vel_max_deg_s), deadband (deadband_deg);
- fits (kp, kv, frictionloss) by *simulate-and-match*: replays the same
  ±step test on the same joint in a fixed-base MuJoCo model (legs
  straight out, bench pose) and coordinate-descends until the simulated
  rise / settle / overshoot metrics match the measured ones. Metrics are
  computed by ``motor_dynamics._fit_step`` — the identical code that
  scored the hardware — so there is no metric-definition mismatch.

Writes ``rl_move/sim/sim_model.json`` (consumed by ``SimServoParams``)
including the per-axis joint-to-joint spread used for DR ranges.

Run (from prototype_sts3215/):
    ../../.venv/bin/python -m rl_move.sim.fit_motor_model
    ../../.venv/bin/python -m rl_move.sim.fit_motor_model --defaults
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .servo_model import (  # noqa: E402
    AXES, SIM_MODEL_PATH, AxisParams, ServoProfile, SimServoParams,
    apply_params_to_model, build_model, joint_qpos_addrs, joint_qvel_addrs,
    position_actuator_ids,
)

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
DEFAULT_MODEL_JSON = _LINUX / "logs" / "motor_model.json"

# Metrics the fit tries to match. tracking_pct (steady-state % of the
# commanded step actually reached, gravity sag included — the bench pose
# holds the leg horizontal) is what anchors kp: rise/settle are mostly
# profile-speed-limited at 30 °/s and barely constrain stiffness.
# delay_ms anchors latency_ms, which is FITTED (not copied): the measured
# delay is time-to-10%-of-step and already contains profile travel time,
# so copying it as pure latency double-counts (replay gate caught this).
FIT_KEYS = ("delay_ms", "rise_ms", "settle_ms", "overshoot_deg",
            "tracking_pct")
FIT_WEIGHTS = {"delay_ms": 1.0, "rise_ms": 1.0, "settle_ms": 1.0,
               "overshoot_deg": 0.5, "tracking_pct": 2.0}


class _StepSim:
    """Fixed-base bench sim that replays one single-joint step test."""

    def __init__(self):
        import mujoco
        self._mujoco = mujoco
        self.model = build_model(fixed_base=True, flat_terrain=True)
        self.data = mujoco.MjData(self.model)
        self._qadr = joint_qpos_addrs(self.model)
        self._vadr = joint_qvel_addrs(self.model)
        self._pos_act = position_actuator_ids(self.model)

    def run_step(self, params: SimServoParams, joint: int, amp_deg: float,
                 *, speed_deg_s: float, acc_units: float = 15.0,
                 sample_dt: float = 0.04,
                 duration_s: float = 2.2) -> list[dict]:
        """Return motor_dynamics-style sample rows for ``_fit_step``."""
        mujoco = self._mujoco
        apply_params_to_model(self.model, params)
        mujoco.mj_resetData(self.model, self.data)
        # Bench pose: sit zero, legs straight out (all joints 0).
        self.data.qpos[self._qadr] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        # Let gravity load the pose under position hold.
        for _ in range(int(0.4 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)

        # Drive through the SAME ServoProfile the training env uses, so the
        # fitted params transfer exactly.
        q0 = self.data.qpos[self._qadr].copy()
        profile = ServoProfile(params, q0)
        q_goal = q0.copy()
        q_goal[joint] += amp_deg * DEG2RAD
        profile.command(q_goal, speed_deg_s=speed_deg_s,
                        acc_units=acc_units)

        goal_deg = float(q_goal[joint]) * RAD2DEG
        dt = self.model.opt.timestep
        n = int(duration_s / dt)
        next_sample = 0.0
        t = 0.0
        rows: list[dict] = []
        for _ in range(n):
            self.data.ctrl[self._pos_act] = profile.tick(dt)
            mujoco.mj_step(self.model, self.data)
            t += dt
            if t >= next_sample:
                rows.append({
                    "t_s": t,
                    "present_deg": float(
                        self.data.qpos[self._qadr[joint]]) * RAD2DEG,
                    "cmd_deg": goal_deg,
                    "speed_deg_s": float(
                        self.data.qvel[self._vadr[joint]]) * RAD2DEG,
                })
                next_sample += sample_dt
        return rows


def _sim_metrics(sim: _StepSim, params: SimServoParams, joint: int,
                 amp_deg: float, speed_deg_s: float,
                 acc_units: float = 15.0) -> dict:
    from motor_dynamics import _fit_step
    fits = []
    for amp in (+amp_deg, -amp_deg):
        rows = sim.run_step(params, joint, amp, speed_deg_s=speed_deg_s,
                            acc_units=acc_units)
        fits.append(_fit_step(rows, amp=amp, t_cmd=0.0))
    out = {}
    for k in FIT_KEYS:
        vals = [f.get(k) for f in fits if f.get(k) is not None]
        out[k] = statistics.mean(vals) if vals else None
    return out


def _loss(sim_m: dict, target: dict) -> float:
    total = 0.0
    for k in FIT_KEYS:
        t, s = target.get(k), sim_m.get(k)
        if t is None:
            continue
        if s is None:
            total += 4.0  # sim never reached the threshold — heavily wrong
            continue
        # Relative error with a floor so tiny targets (overshoot≈0) don't
        # explode the denominator.
        if k == "tracking_pct":
            # Compare the *sag* (100 − tracking), not the raw percentage;
            # 99% vs 90% tracking is a 10× stiffness difference.
            total += FIT_WEIGHTS[k] * (
                ((100.0 - s) - (100.0 - t)) / max(100.0 - t, 2.0)) ** 2
            continue
        floor = 20.0 if k.endswith("_ms") else 0.3
        total += FIT_WEIGHTS[k] * ((s - t) / max(abs(t), floor)) ** 2
    return total


def fit_axis(sim: _StepSim, params: SimServoParams, axis: str, joint: int,
             target: dict, *, amp_deg: float, speed_deg_s: float,
             acc_units: float = 15.0,
             rounds: int = 3, verbose: bool = True) -> AxisParams:
    """Coordinate descent over kp, kv, frictionloss, latency for one axis."""
    ax = params.axes[axis]
    meas_delay = float(target.get("delay_ms") or ax.latency_ms)
    # Start latency below the measured delay — the profile's own travel
    # time to 10% of the step makes up the difference.
    ax.latency_ms = 0.5 * meas_delay
    grids = {
        "kp": np.geomspace(4.0, 120.0, 9),
        "kv": np.geomspace(0.05, 2.0, 9),
        "frictionloss": np.array([0.0, 0.01, 0.02, 0.04, 0.08, 0.15]),
        "latency_ms": np.linspace(max(5.0, 0.15 * meas_delay),
                                  1.1 * meas_delay, 9),
    }

    def metrics() -> dict:
        return _sim_metrics(sim, params, joint, amp_deg, speed_deg_s,
                            acc_units)

    best = _loss(metrics(), target)
    for rnd in range(rounds):
        improved = False
        for key, grid in grids.items():
            cur = getattr(ax, key)
            for v in grid:
                if abs(v - cur) < 1e-9:
                    continue
                setattr(ax, key, float(v))
                loss = _loss(metrics(), target)
                if loss < best - 1e-6:
                    best, cur, improved = loss, float(v), True
                else:
                    setattr(ax, key, cur)
        # Refine grids around the current best.
        for key in ("kp", "kv"):
            c = getattr(ax, key)
            grids[key] = np.geomspace(c / 1.6, c * 1.6, 7)
        c = ax.frictionloss
        grids["frictionloss"] = np.linspace(max(0.0, c - 0.02), c + 0.02, 5)
        c = ax.latency_ms
        grids["latency_ms"] = np.linspace(max(5.0, c - 30.0), c + 30.0, 7)
        if verbose:
            print(f"  [{axis}] round {rnd + 1}: loss={best:.4f} "
                  f"kp={ax.kp:.1f} kv={ax.kv:.3f} "
                  f"fric={ax.frictionloss:.3f} lat={ax.latency_ms:.0f}ms")
        if not improved:
            break
    return ax


def _axis_spread(joints: list[dict], axis: str) -> dict:
    """Relative std of delay/rise across all OK joints of one axis."""
    rows = [j for j in joints if j.get("axis") == axis and j.get("ok")]
    out = {}
    for key in ("delay_ms", "rise_ms"):
        vals = [j[key] for j in rows if j.get(key)]
        if len(vals) >= 2 and statistics.mean(vals) > 0:
            out[f"{key}_pct"] = round(
                statistics.pstdev(vals) / statistics.mean(vals), 3)
    return out


def _targets_from_csv(csv_path: Path, joint: int,
                      amp_deg: float) -> dict | None:
    """Recompute fit targets from the raw battery CSV with _fit_step.

    The aggregates in motor_model.json were scored on-robot at battery
    time; when the metric definition changes afterwards (e.g. the
    2026-08-07 fix that derives speed from position instead of the glitchy
    STS speed register), the raw samples are the truth.
    """
    from motor_dynamics import _fit_step
    from .replay_compare import _STEP_RE, load_step_phases

    per_key: dict[str, list[float]] = {k: [] for k in FIT_KEYS}
    for (j, phase), rows in load_step_phases(csv_path).items():
        if j != joint:
            continue
        m = _STEP_RE.match(phase)
        amp = float(m.group(2)) * (1 if m.group(1) == "+" else -1)
        if abs(abs(amp) - amp_deg) > 1e-6:
            continue
        t0 = rows[0]["t_s"]
        fit = _fit_step([{**r, "t_s": r["t_s"] - t0} for r in rows],
                        amp=amp, t_cmd=0.0)
        for k in FIT_KEYS:
            if fit.get(k) is not None:
                per_key[k].append(float(fit[k]))
    if not any(per_key.values()):
        return None
    return {k: (round(statistics.mean(v), 2) if v else None)
            for k, v in per_key.items()}


def fit_from_hardware(model_json: Path, *, rounds: int = 3,
                      verbose: bool = True) -> SimServoParams:
    hw = json.loads(Path(model_json).read_text())
    axis_model = hw.get("axis_model") or {}
    if not axis_model:
        raise SystemExit(
            f"{model_json} has no axis_model — run the dynamics battery "
            "first (POST /api/rl/probe_dynamics)")

    amp_deg = float(hw.get("amp_deg", 12.0))
    speed_deg_s = float(hw.get("speed", 350)) * 360.0 / 4096.0
    acc_units = float(hw.get("acc", 15))
    params = SimServoParams.defaults()
    params.source = str(model_json)
    params.timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
    params.speed_counts_s = float(hw.get("speed", 350))

    # Axis name mapping: hardware "hip" == sim joint index 1 per leg.
    full_joints = {int(j) % 3: int(j) for j in hw.get("full_joints", [])}
    sim = _StepSim()
    for ai, axis in enumerate(AXES):
        meas = axis_model.get(axis)
        if not meas:
            if verbose:
                print(f"  [{axis}] no axis_model entry — keeping defaults")
            continue
        ax = params.axes[axis]
        # latency_ms is FITTED against delay_ms inside fit_axis — the
        # measured delay includes profile travel time, so no direct copy.
        if meas.get("vel_max_deg_s"):
            v = float(meas["vel_max_deg_s"])
            # Logs recorded before 2026-08-07 carry 50×-inflated vel_max
            # (speed register decoded as 0.732 rpm/unit instead of
            # counts/s). Anything far above the commanded profile speed
            # is that decode bug — fall back to the profile ceiling.
            if v > 2.0 * speed_deg_s:
                if verbose:
                    print(f"  [{axis}] measured vel_max {v:.0f} °/s is the "
                          f"pre-2026-08-07 50× decode bug — using profile "
                          f"speed {speed_deg_s:.1f} °/s")
                v = speed_deg_s
            ax.vel_max_deg_s = v
        if meas.get("deadband_deg") is not None:
            ax.deadband_deg = float(meas["deadband_deg"])

        joint = full_joints.get(ai, 6 + ai)  # default: L2's joint
        # Prefer recomputing targets from the raw CSV (local copy next to
        # motor_model.json) so the current metric definition applies.
        target = None
        csv_name = Path(str(hw.get("csv") or "")).name
        if csv_name:
            csv_local = Path(model_json).parent / csv_name
            if csv_local.is_file():
                target = _targets_from_csv(csv_local, joint, amp_deg)
        if target is None:
            if verbose:
                print(f"  [{axis}] no local battery CSV — using on-robot "
                      "aggregates from motor_model.json")
            target = {k: meas.get(k) for k in FIT_KEYS}
            # tracking_pct / overshoot live on the per-joint records, not
            # in axis_model — pull them from the full-battery joint.
            jrec = next((j for j in hw.get("joints", [])
                         if j.get("joint") == joint), {})
            for k in ("tracking_pct", "overshoot_deg"):
                if target.get(k) is None and jrec.get(k) is not None:
                    target[k] = jrec[k]
        if verbose:
            print(f"[{axis}] joint {joint} targets: "
                  + " ".join(f"{k}={target.get(k)}" for k in FIT_KEYS))
        fit_axis(sim, params, axis, joint, target,
                 amp_deg=amp_deg, speed_deg_s=speed_deg_s,
                 acc_units=acc_units, rounds=rounds, verbose=verbose)
        achieved = _sim_metrics(sim, params, joint, amp_deg, speed_deg_s,
                                acc_units)
        if verbose:
            print(f"  [{axis}] fitted kp={params.axes[axis].kp:.1f} "
                  f"kv={params.axes[axis].kv:.3f} "
                  f"fric={params.axes[axis].frictionloss:.3f} "
                  f"lat={params.axes[axis].latency_ms:.0f}ms | achieved "
                  + " ".join(f"{k}={achieved.get(k)}" for k in FIT_KEYS))
        params.spread[axis] = _axis_spread(hw.get("joints", []), axis)
    return params


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model-json", type=Path, default=DEFAULT_MODEL_JSON,
                    help="hardware motor_model.json "
                         f"(default {DEFAULT_MODEL_JSON})")
    ap.add_argument("--out", type=Path, default=SIM_MODEL_PATH)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--defaults", action="store_true",
                    help="skip fitting; write default params so the sim "
                         "pipeline runs before hardware data exists")
    args = ap.parse_args(argv)

    if args.defaults:
        params = SimServoParams.defaults()
    elif not args.model_json.is_file():
        print(f"[fit] {args.model_json} not found — writing defaults. "
              "Run the hardware battery, then re-run this script.")
        params = SimServoParams.defaults()
    else:
        params = fit_from_hardware(args.model_json, rounds=args.rounds)

    path = params.save(args.out)
    print(f"[fit] wrote {path} (source: {params.source})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
