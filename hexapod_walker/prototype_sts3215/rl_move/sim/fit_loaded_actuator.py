"""Fit LOADED actuator params from the 08-10 bench session traces.

The air battery (``fit_motor_model.py`` -> ``sim_model.json``) fitted the
servo model with the legs UNLOADED (bench pose, straight out). Hardware
session 3 measured the loaded truth (RL_LOG "hardware session 3" item 4;
GPT handoff ruling 5): at stance, command->first-motion is ~100-200 ms and
t90 is 260-430 ms versus tens of ms in sim, and the air fit recorded the
commanded profile speed (350 counts/s = 30.8 deg/s) as the velocity
ceiling while the loaded joint demonstrably reaches 48-67 deg/s when
commanded faster. Small corrections during weight transfer are ~5x slower
in reality than in sim — the strongest quantified suspect for the
deterministic liftoff +roll collapse.

Data (rl_move/hardware_traces/):
- ``step_ladder_20260810.csv`` — L0 knee (joint 2), robot standing at
  plant, +-2/5/10 deg steps, ~30-50 Hz reads with robot-side bus_ts.
  Driven over HTTP -> on-board drive loop (mcu bus defaults speed=1500
  counts/s, acc=30), so its profile ceiling (131.8 deg/s) was NOT
  binding: measured peak velocity IS the loaded servo capability.
  Its cmd->motion time includes the HTTP hop, so latency from this
  trace is an UPPER bound.
- ``rl_stand_20260810_*.csv`` — 25 Hz deployed-path episodes (on-board
  runner -> SyncWrite, speed=400, acc=20) logging cmd AND measured q
  per tick: the honest deployed-latency source (cross-correlation),
  and the held-out multi-step validation stream (the liftoff-collapse
  episodes themselves).

Fit (per GPT handoff ruling: simple identified model first): the existing
sim structure — pure delay + ServoProfile trapezoid (vel/acc ceilings) +
deadband + MuJoCo kp/kv tracking — refitted for the knee axis in a
sim-in-the-loop STANCE replay (free base, on ground, plant pose), by
coordinate descent on (kp, kv, frictionloss, latency_ms, vel_max_deg_s)
against the measured step metrics on the +-2 and +-10 deg steps; the
+-5 deg steps are HELD OUT for validation, plus a multi-step cmd-stream
replay of the rl_stand traces (fitted vs air params).

Output: ``rl_move/sim/sim_model_loaded.json`` (same schema as
sim_model.json). Envs select it with ``--cfg-set bus.servo_params=loaded``
(default cfg keeps the air model — legacy byte-exact).

Run (from prototype_sts3215/, repo .venv):
    ../../.venv/bin/python -m rl_move.sim.fit_loaded_actuator --measure
    ../../.venv/bin/python -m rl_move.sim.fit_loaded_actuator          # fit
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (str(_PROTO),):
    if p not in sys.path:
        sys.path.insert(0, p)

from .servo_model import (  # noqa: E402
    AXES, SIM_MODEL_PATH, AxisParams, ServoProfile, SimServoParams,
    apply_params_to_model, build_model, joint_qpos_addrs, joint_qvel_addrs,
    position_actuator_ids,
)

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

TRACES = _RL / "hardware_traces"
LADDER_CSV = TRACES / "step_ladder_20260810.csv"
STAND_CSVS = sorted(TRACES.glob("rl_stand_20260810_*.csv"))
LOADED_MODEL_PATH = Path(__file__).resolve().parent / "sim_model_loaded.json"

LADDER_JOINT = 2          # L0 knee (CSV column q2_deg; RL_LOG session 3)
# The ladder went over HTTP -> drive loop -> mcu bus write defaults
# (mcu_feetech_bus.write_all: speed=1500 counts/s, acc=30).
LADDER_SPEED_DEG_S = 1500.0 * 360.0 / 4096.0     # 131.8 — not binding
LADDER_ACC_UNITS = 30.0
# Deployed RL path (rl_policy.py + sim envs): bus.write_speed/acc cfg.
RL_SPEED_DEG_S = 400.0 * 360.0 / 4096.0          # 35.2
RL_ACC_UNITS = 20.0

MOTION_THRESH_DEG = 0.3   # first-motion threshold (encoder LSB 0.088)

FIT_AMPS = (2.0, 10.0)    # fitted steps
HOLDOUT_AMPS = (5.0,)     # held out for validation


# ---------------------------------------------------------------------------
# Trace parsing + step metrics
# ---------------------------------------------------------------------------

def load_ladder(path: Path = LADDER_CSV) -> list[dict]:
    """Split the ladder CSV into per-step phases with robot-side times."""
    rows = list(csv.DictReader(open(path)))
    phases: list[dict] = []
    cur: dict | None = None
    for r in rows:
        tag = r["tag"]
        if tag.startswith("CMD_"):
            if cur:
                phases.append(cur)
            name = tag[4:]
            amp = float(name[2:]) * (1.0 if name.startswith("up") else -1.0)
            cur = {"name": name, "amp": amp,
                   "t_cmd": float(r["t_unix"]), "t": [], "q": []}
        elif cur is not None and tag == cur["name"]:
            t = r["bus_ts"]
            cur["t"].append(float(t) if t not in (None, "None") else
                            float(r["t_unix"]))
            cur["q"].append(float(r["q2_deg"]))
    if cur:
        phases.append(cur)
    return phases


def step_metrics(t: np.ndarray, q: np.ndarray, t_cmd: float,
                 amp: float) -> dict:
    """Latency / t90 / peak vel / overshoot / tracking for one step."""
    q0 = float(q[0])
    qf = float(np.mean(q[-8:]))
    dq = q - q0
    s = math.copysign(1.0, amp)
    moved = np.abs(dq) > MOTION_THRESH_DEG
    i_move = int(np.argmax(moved)) if moved.any() else None
    reach = s * dq >= 0.9 * abs(qf - q0)
    i_90 = int(np.argmax(reach)) if reach.any() else None
    vel = np.diff(q) / np.maximum(np.diff(t), 1e-4)
    out = {
        "amp_deg": amp,
        "latency_ms": None if i_move is None
        else (t[i_move] - t_cmd) * 1e3,
        "t90_ms": None if i_90 is None else (t[i_90] - t_cmd) * 1e3,
        "t90_from_motion_ms": None if (i_90 is None or i_move is None)
        else (t[i_90] - t[i_move]) * 1e3,
        "peak_vel_deg_s": float(np.max(np.abs(vel))) if len(vel) else None,
        "overshoot_deg": float(max(0.0, np.max(s * dq) - abs(amp))),
        "tracking_pct": 100.0 * (qf - q0) / amp,
        "settle_err_deg": abs(q0 + amp - qf),
    }
    return out


def measure_ladder(verbose: bool = True) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for ph in load_ladder():
        m = step_metrics(np.asarray(ph["t"]), np.asarray(ph["q"]),
                         ph["t_cmd"], ph["amp"])
        out[ph["name"]] = m
        if verbose:
            print(f"  [{ph['name']:>4}] lat {m['latency_ms']:6.0f} ms  "
                  f"t90 {m['t90_ms']:6.0f} ms "
                  f"(from motion {m['t90_from_motion_ms']:5.0f})  "
                  f"peak {m['peak_vel_deg_s']:5.1f} deg/s  "
                  f"track {m['tracking_pct']:6.1f}%  "
                  f"ovsh {m['overshoot_deg']:.2f}")
    return out


def measure_stand_latency(verbose: bool = True) -> dict:
    """Deployed-path cmd->measured RESPONSE LAG via derivative xcorr.

    NOTE this is delay + tracking-lag combined (the xcorr peak of a
    lagged first-order response sits near its time constant), NOT pure
    transport latency — use it as a deployed-path consistency check
    against the ladder's t90, not as the latency param. 25 Hz ticks
    (40 ms) with parabolic sub-tick refinement; only joints that
    actually travel (>4 deg range) vote.
    """
    kmax = 15
    per_axis: dict[str, list[float]] = {ax: [] for ax in AXES}
    for path in STAND_CSVS:
        rows = list(csv.DictReader(open(path)))
        if len(rows) < 50:
            continue
        for j in range(18):
            cmd = np.array([float(r[f"cmd{j}_deg"]) for r in rows])
            q = np.array([float(r[f"q{j}_deg"]) for r in rows])
            if np.ptp(cmd) < 4.0:
                continue
            dc, dq = np.diff(cmd), np.diff(q)
            dc = dc - dc.mean()
            dq = dq - dq.mean()
            # Normalized by overlap so large k isn't penalized.
            xc = [float(np.dot(dc[:len(dc) - k], dq[k:]))
                  / max(len(dc) - k, 1) for k in range(kmax + 1)]
            k = int(np.argmax(xc))
            if 0 < k < kmax:
                a, b, c = xc[k - 1], xc[k], xc[k + 1]
                denom = a - 2 * b + c
                k = k + (0.5 * (a - c) / denom if abs(denom) > 1e-12 else 0.0)
            per_axis[AXES[j % 3]].append(k * 40.0)
    out = {}
    for ax, vals in per_axis.items():
        if vals:
            out[ax] = {"n": len(vals),
                       "median_ms": round(float(np.median(vals)), 1),
                       "p25_ms": round(float(np.percentile(vals, 25)), 1),
                       "p75_ms": round(float(np.percentile(vals, 75)), 1)}
            if verbose:
                print(f"  [{ax:>4}] deployed cmd->q response lag: "
                      f"median {out[ax]['median_ms']:.0f} ms  "
                      f"(p25 {out[ax]['p25_ms']:.0f} / "
                      f"p75 {out[ax]['p75_ms']:.0f}, n={out[ax]['n']})")
    return out


# ---------------------------------------------------------------------------
# Sim-in-the-loop stance replay
# ---------------------------------------------------------------------------

class _StanceStepSim:
    """Free-base sim standing at plant; replays the loaded step ladder.

    Placement + contact softening mirror ``SimHexapodBalanceEnv``
    (``_place_at_plant`` / ``soften_contacts``) so the load the servo
    fights is the same load training policies see.
    """

    def __init__(self):
        import mujoco
        from .sim_env import _default_plant_deg, soften_contacts
        self._mujoco = mujoco
        self.model = build_model(fixed_base=False, flat_terrain=True)
        soften_contacts(self.model)
        self.data = mujoco.MjData(self.model)
        self._qadr = joint_qpos_addrs(self.model)
        self._vadr = joint_qvel_addrs(self.model)
        self._pos_act = position_actuator_ids(self.model)
        self._plant_rad = _default_plant_deg() * DEG2RAD

    def place(self, q_rad: np.ndarray) -> None:
        """Same recipe as SimHexapodBalanceEnv._place_at_plant."""
        import mujoco_prototype as MP
        from rl_move.body_ik import fk_all_feet
        mujoco = self._mujoco
        feet = fk_all_feet(q_rad)
        foot_drop = float(np.min(feet[:, 2]))
        base_z = MP.YAW_OUTPUT_HEIGHT - foot_drop + MP.FOOT_R + 0.002
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0.0, 0.0, base_z)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self._qadr] = q_rad
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.ctrl[self._pos_act] = q_rad
        mujoco.mj_forward(self.model, self.data)
        for _ in range(40):
            worst = 0.0
            for ci in range(self.data.ncon):
                worst = min(worst, float(self.data.contact[ci].dist))
            if worst > -1e-4:
                break
            self.data.qpos[2] += -worst + 0.001
            mujoco.mj_forward(self.model, self.data)

    def _settle_at_plant(self, params: SimServoParams) -> np.ndarray:
        apply_params_to_model(self.model, params)
        self.place(self._plant_rad)
        q = self._plant_rad
        for _ in range(int(0.6 / self.model.opt.timestep)):
            self.data.ctrl[self._pos_act] = q
            self._mujoco.mj_step(self.model, self.data)
        return self.data.qpos[self._qadr].copy()

    def run_step(self, params: SimServoParams, amp_deg: float, *,
                 joint: int = LADDER_JOINT,
                 speed_deg_s: float = LADDER_SPEED_DEG_S,
                 acc_units: float = LADDER_ACC_UNITS,
                 sample_dt: float = 0.025,
                 duration_s: float = 2.5) -> tuple[np.ndarray, np.ndarray]:
        mujoco = self._mujoco
        q0 = self._settle_at_plant(params)
        profile = ServoProfile(params, q0)
        q_goal = q0.copy()
        q_goal[joint] += amp_deg * DEG2RAD
        profile.command(q_goal, speed_deg_s=speed_deg_s, acc_units=acc_units)
        dt = self.model.opt.timestep
        t, ts, qs = 0.0, [], []
        next_s = 0.0
        for _ in range(int(duration_s / dt)):
            self.data.ctrl[self._pos_act] = profile.tick(dt)
            mujoco.mj_step(self.model, self.data)
            t += dt
            if t >= next_s:
                ts.append(t)
                qs.append(float(self.data.qpos[self._qadr[joint]]) * RAD2DEG)
                next_s += sample_dt
        return np.asarray(ts), np.asarray(qs)


def sim_step_metrics(sim: _StanceStepSim, params: SimServoParams,
                     amp: float) -> dict:
    t, q = sim.run_step(params, amp)
    return step_metrics(t, q, 0.0, amp)


# ---------------------------------------------------------------------------
# Fit
# ---------------------------------------------------------------------------

# What the fit matches, and how much each counts. latency/t90 are the
# headline loaded gap; tracking (gravity sag) anchors kp; peak_vel
# anchors the true loaded ceiling; overshoot keeps kv honest.
LOSS_KEYS = ("latency_ms", "t90_ms", "peak_vel_deg_s", "tracking_pct",
             "overshoot_deg")
LOSS_W = {"latency_ms": 1.0, "t90_ms": 1.5, "peak_vel_deg_s": 1.0,
          "tracking_pct": 2.0, "overshoot_deg": 0.5}


def _loss_one(sim_m: dict, target: dict) -> float:
    total = 0.0
    amp = abs(target.get("amp_deg", 10.0))
    for k in LOSS_KEYS:
        tv, sv = target.get(k), sim_m.get(k)
        if tv is None:
            continue
        if sv is None:
            total += 4.0
            continue
        if k == "tracking_pct":
            # One encoder LSB (0.088 deg) is ~4% of a 2 deg step: below
            # that the measured tracking (101% on the 2 deg steps) is
            # quantization, not servo behaviour — don't chase it.
            lsb_pct = 100.0 * 0.088 / amp
            sag_t, sag_s = 100.0 - tv, 100.0 - sv
            err = max(0.0, abs(sag_s - sag_t) - lsb_pct)
            total += LOSS_W[k] * (err / max(sag_t, 2.0)) ** 2
        else:
            floor = 20.0 if k.endswith("_ms") else 2.0
            total += LOSS_W[k] * ((sv - tv) / max(abs(tv), floor)) ** 2
    return total


def fit_knee(measured: dict[str, dict], *, latency_cap_ms: float,
             rounds: int = 4, verbose: bool = True) -> SimServoParams:
    """Coordinate descent on the knee axis against the +-2/+-10 steps."""
    params = SimServoParams.load()          # start from the air fit
    params.source = f"loaded fit from {LADDER_CSV.name} (base: air fit)"
    params.timestamp = time.strftime("%Y-%m-%dT%H:%M:%S")
    ax = params.axes["knee"]
    sim = _StanceStepSim()

    targets = {name: m for name, m in measured.items()
               if abs(m["amp_deg"]) in FIT_AMPS}

    def total_loss() -> float:
        return sum(_loss_one(sim_step_metrics(sim, params, m["amp_deg"]), m)
                   for m in targets.values())

    grids = {
        "kp": np.geomspace(max(ax.kp / 8, 2.0), 800.0, 11),
        "kv": np.geomspace(0.05, 4.0, 9),
        "frictionloss": np.array([0.0, 0.02, 0.05, 0.1, 0.2, 0.4]),
        "latency_ms": np.linspace(10.0, latency_cap_ms, 9),
        "vel_max_deg_s": np.linspace(30.0, 90.0, 9),
        # The air-battery deadband (0.49 deg) came from the unloaded rig;
        # loaded tracking (96.6% at 5 deg) implies ~0.17 deg. The profile
        # stops inside the deadband, so it IS the tracking floor.
        "deadband_deg": np.linspace(0.05, 0.6, 12),
    }
    best = total_loss()
    if verbose:
        print(f"  start loss {best:.3f} (air params)")
    for rnd in range(rounds):
        improved = False
        for key, grid in grids.items():
            cur = getattr(ax, key)
            for v in grid:
                if abs(v - cur) < 1e-9:
                    continue
                setattr(ax, key, float(v))
                loss = total_loss()
                if loss < best - 1e-6:
                    best, cur, improved = loss, float(v), True
                else:
                    setattr(ax, key, cur)
        for key in ("kp", "kv"):
            c = getattr(ax, key)
            grids[key] = np.geomspace(c / 1.5, c * 1.5, 7)
        c = ax.latency_ms
        grids["latency_ms"] = np.linspace(
            max(5.0, c - 25.0), min(latency_cap_ms, c + 25.0), 7)
        c = ax.vel_max_deg_s
        grids["vel_max_deg_s"] = np.linspace(max(20.0, c - 12.0), c + 12.0, 7)
        c = ax.frictionloss
        grids["frictionloss"] = np.linspace(max(0.0, c - 0.04), c + 0.04, 5)
        c = ax.deadband_deg
        grids["deadband_deg"] = np.linspace(max(0.03, c - 0.1), c + 0.1, 5)
        if verbose:
            print(f"  round {rnd + 1}: loss={best:.3f} kp={ax.kp:.1f} "
                  f"kv={ax.kv:.3f} fric={ax.frictionloss:.3f} "
                  f"lat={ax.latency_ms:.0f}ms vel={ax.vel_max_deg_s:.1f} "
                  f"dbd={ax.deadband_deg:.2f}")
        if not improved:
            break
    if verbose:
        print("  final per-step (fit set):")
        for name, m in targets.items():
            sm = sim_step_metrics(sim, params, m["amp_deg"])
            print(f"    [{name:>4}] " + "  ".join(
                f"{k} meas {m[k]:.1f}/sim "
                f"{(sm[k] if sm[k] is not None else float('nan')):.1f}"
                for k in LOSS_KEYS if m.get(k) is not None)
                + f"  -> loss {_loss_one(sm, m):.2f}")
    return params


def propagate_axes(params: SimServoParams, deployed_delay: dict,
                   verbose: bool = True) -> None:
    """Carry the knee's loaded corrections to hip/yaw.

    ASSUMPTION (operator to review): only the knee has a loaded step
    ladder. All 18 servos are the same STS3215, so the fitted VELOCITY
    ceiling transfers as-is. Latency: hip/yaw get the same loaded-minus-
    air latency DELTA as the knee (bus/loop pickup + load-response lag
    are common-mode), cross-checked against the deployed-path xcorr
    medians where joints moved. kp/kv/frictionloss stay air-fitted for
    hip/yaw — no loaded response data; wishlist item 4 covers a proper
    per-axis loaded ladder.
    """
    air = SimServoParams.load()
    d_lat = params.axes["knee"].latency_ms - air.axes["knee"].latency_ms
    for axname in ("hip", "yaw"):
        ax = params.axes[axname]
        ax.latency_ms = air.axes[axname].latency_ms + d_lat
        ax.vel_max_deg_s = params.axes["knee"].vel_max_deg_s
        if verbose:
            print(f"  [{axname}] latency {air.axes[axname].latency_ms:.0f}"
                  f" -> {ax.latency_ms:.0f} ms (+{d_lat:.0f} knee delta), "
                  f"vel_max -> {ax.vel_max_deg_s:.1f} deg/s "
                  f"(deployed xcorr median: "
                  f"{(deployed_delay.get(axname) or {}).get('median_ms')} ms)")


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_holdout(params: SimServoParams, measured: dict[str, dict],
                     verbose: bool = True) -> dict:
    """Held-out +-5 deg steps: fitted vs air params, loss per step."""
    sim = _StanceStepSim()
    air = SimServoParams.load()
    out = {}
    for name, m in measured.items():
        if abs(m["amp_deg"]) not in HOLDOUT_AMPS:
            continue
        fit_m = sim_step_metrics(sim, params, m["amp_deg"])
        air_m = sim_step_metrics(sim, air, m["amp_deg"])
        out[name] = {"loss_fitted": round(_loss_one(fit_m, m), 3),
                     "loss_air": round(_loss_one(air_m, m), 3),
                     "sim_fitted": fit_m, "sim_air": air_m, "measured": m}
        if verbose:
            print(f"  [{name:>4}] holdout loss fitted "
                  f"{out[name]['loss_fitted']:.3f} vs air "
                  f"{out[name]['loss_air']:.3f} | t90 meas "
                  f"{m['t90_ms']:.0f} / fit {fit_m['t90_ms']:.0f} / air "
                  f"{air_m['t90_ms']:.0f} ms | track meas "
                  f"{m['tracking_pct']:.1f} / fit {fit_m['tracking_pct']:.1f}"
                  f" / air {air_m['tracking_pct']:.1f}%")
    return out


def validate_stand_replay(params: SimServoParams,
                          verbose: bool = True) -> dict:
    """Multi-step deployed-contract replay of the rl_stand traces.

    Feeds each trace's recorded 25 Hz cmd stream through ServoProfile +
    MuJoCo from the recorded start pose (free base, on ground — the
    stand attempt starts from the sit/belly pose) and reports measured-
    vs-sim q RMSE over the moving joints, fitted vs air params. This is
    the ruling's held-out multi-step prediction test AND the first
    piece of the liftoff-reproduction fixture.
    """
    import mujoco
    out = {}
    air = SimServoParams.load()
    for path in STAND_CSVS:
        rows = list(csv.DictReader(open(path)))
        if len(rows) < 50:
            continue
        cmd = np.array([[float(r[f"cmd{j}_deg"]) for j in range(18)]
                        for r in rows]) * DEG2RAD
        meas = np.array([[float(r[f"q{j}_deg"]) for j in range(18)]
                         for r in rows]) * DEG2RAD
        moving = np.ptp(meas, axis=0) * RAD2DEG > 4.0
        res = {}
        for tag, prm in (("fitted", params), ("air", air)):
            sim = _StanceStepSim()
            mujoco_m, data = sim.model, sim.data
            apply_params_to_model(mujoco_m, prm)
            q0 = meas[0]
            sim.place(q0)
            for _ in range(int(0.4 / mujoco_m.opt.timestep)):
                data.ctrl[sim._pos_act] = q0
                mujoco.mj_step(mujoco_m, data)
            profile = ServoProfile(prm, data.qpos[sim._qadr].copy())
            dt = mujoco_m.opt.timestep
            per = int(round(0.04 / dt))
            sim_q = []
            for k in range(len(cmd)):
                profile.command(cmd[k], speed_deg_s=RL_SPEED_DEG_S,
                                acc_units=RL_ACC_UNITS)
                for _ in range(per):
                    data.ctrl[sim._pos_act] = profile.tick(dt)
                    mujoco.mj_step(mujoco_m, data)
                sim_q.append(data.qpos[sim._qadr].copy())
            sim_q = np.asarray(sim_q)
            err = (sim_q - meas)[:, moving] * RAD2DEG
            res[tag] = round(float(np.sqrt(np.mean(err ** 2))), 2)
        out[path.name] = res
        if verbose:
            print(f"  [{path.name}] moving-joint RMSE: "
                  f"fitted {res['fitted']:.2f} deg vs air {res['air']:.2f} "
                  f"deg ({int(moving.sum())} joints, {len(cmd)} ticks)")
    return out


# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--measure", action="store_true",
                    help="print trace measurements only, no fit")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--out", type=Path, default=LOADED_MODEL_PATH)
    ap.add_argument("--latency-cap-ms", type=float, default=85.0,
                    help="fit ceiling for pure latency: the ladder's "
                         "cmd->motion includes the HTTP hop, and the MJX "
                         "pending ring caps latency*DR1.8 < 160 ms")
    args = ap.parse_args(argv)

    print(f"[ladder] {LADDER_CSV.name} (L0 knee, loaded at plant):")
    measured = measure_ladder()
    print("[deployed] rl_stand cmd->q delay (25 Hz xcorr):")
    deployed = measure_stand_latency()
    if args.measure:
        return 0

    print("[fit] knee axis, sim-in-the-loop stance replay "
          f"(fit amps {FIT_AMPS}, holdout {HOLDOUT_AMPS}):")
    params = fit_knee(measured, latency_cap_ms=args.latency_cap_ms,
                      rounds=args.rounds)
    print("[propagate] hip/yaw from knee deltas:")
    propagate_axes(params, deployed)
    print("[validate] held-out +-5 deg steps:")
    holdout = validate_holdout(params, measured)
    print("[validate] rl_stand multi-step replay:")
    replay = validate_stand_replay(params)

    # DR POLICY (operator, 08-10): parameters we are NOT sure about go
    # into domain randomization rather than a pretended-exact nominal.
    # Latency is the least-identified number here — the ladder's
    # cmd->motion includes the HTTP hop (upper bound ~160 ms) while the
    # direct-bus path could be far quicker (air knee measured 8.6 ms),
    # and the fit sat on its cap. Writing delay_ms_pct=0.45 makes
    # DomainRandomizer.from_params widen latency DR to x0.3-1.9
    # (~25-160 ms about the knee nominal) on every run that trains with
    # this file. rise_ms_pct keeps the air joint-to-joint spread
    # (drives kp/kv spread — the loaded fit has no per-joint data).
    air_spread = SimServoParams.load().spread
    params.spread = {ax: {**air_spread.get(ax, {}), "delay_ms_pct": 0.45}
                     for ax in ("yaw", "hip", "knee")}
    path = params.save(args.out)
    # Append the evidence trail to the JSON for provenance.
    blob = json.loads(path.read_text())
    blob["fit_evidence"] = {
        "ladder_measured": measured, "deployed_delay_xcorr": deployed,
        "holdout": {k: {kk: v[kk] for kk in ("loss_fitted", "loss_air")}
                    for k, v in holdout.items()},
        "stand_replay_rmse_deg": replay,
        "ladder_profile": {"speed_deg_s": LADDER_SPEED_DEG_S,
                           "acc_units": LADDER_ACC_UNITS,
                           "note": "HTTP->drive-loop path; mcu bus "
                                   "defaults speed=1500 acc=30 assumed"},
    }
    path.write_text(json.dumps(blob, indent=2))
    print(f"[fit] wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
