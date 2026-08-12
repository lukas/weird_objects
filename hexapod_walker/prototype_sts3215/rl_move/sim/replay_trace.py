"""Open-loop hardware-trace replay: sim-to-real divergence finder.

Feeds a logged on-robot episode CSV's recorded 25 Hz command stream
(``cmd*_deg`` — the post-safety targets the servos actually received)
through ServoProfile + free-base MuJoCo, starting from the trace's own
measured initial pose, with NO policy in the loop. Then overlays
hardware vs sim roll(t) / roll-rate(t) / q(t) and reports the first
tick where the trajectories materially diverge, plus per-foot contact
/ slip diagnostics around that tick.

Why (RL_REVIEW_BUNDLE 08-11, "we now have the data to model the
transient"): the real robot develops lateral roll during load
transitions (belly-curl rise: 5/5 deterministic tilt_roll trips at
tick ~227; walk takeoff: 13-27 deg first-second transients) that the
sim does not produce. This experiment discriminates the candidate
causes:

- joint positions diverge first        -> actuator/load model
- joints agree, contacts/slip diverge  -> contact/pinning
- both plausible, roll accel differs   -> inertia/CoM/mass distribution

Run (from prototype_sts3215/, repo .venv):
    ../../.venv/bin/python -m rl_move.sim.replay_trace \
        --csv rl_move/hardware_traces/stand_fail_20260811/*.csv \
        --servo-params both --plot
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (str(_PROTO),):
    if p not in sys.path:
        sys.path.insert(0, p)

from .servo_model import (  # noqa: E402
    ServoProfile, SimServoParams, apply_params_to_model, build_model,
    joint_qpos_addrs, joint_qvel_addrs, position_actuator_ids,
)

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# Deployed RL runner write profile (rl_policy.py bus defaults).
RL_SPEED_DEG_S = 400.0 * 360.0 / 4096.0          # 35.2
RL_ACC_UNITS = 20.0

LOADED_MODEL_PATH = Path(__file__).resolve().parent / "sim_model_loaded.json"

# Divergence thresholds: "material" = sustained, not one noisy tick.
ROLL_DIV_DEG = 2.0        # |sim_roll_rel - hw_roll_rel|
Q_DIV_DEG = 3.0           # per-joint |sim_q - hw_q| (moving joints)
SUSTAIN_TICKS = 5         # must hold this many consecutive ticks
MOVING_PTP_DEG = 4.0      # a joint "moves" if its hw range exceeds this
CONTACT_N = 0.5           # touch-sensor force threshold


# ---------------------------------------------------------------------------
# Trace loading
# ---------------------------------------------------------------------------

def load_trace(csv_path: Path) -> dict:
    """Run-phase rows of an on-robot rl_*.csv + its summary sidecar."""
    rows = [r for r in csv.DictReader(open(csv_path))
            if r.get("phase") == "run"]
    if len(rows) < 25:
        raise SystemExit(f"{csv_path.name}: only {len(rows)} run ticks")
    tr = {
        "name": csv_path.name,
        "t": np.array([float(r["t_s"]) for r in rows]),
        "roll": np.array([float(r["roll_deg"]) for r in rows]),
        "pitch": np.array([float(r["pitch_deg"]) for r in rows]),
        "gyro_x": np.array([float(r["gyro_x_dps"]) for r in rows]),
        "gyro_y": np.array([float(r["gyro_y_dps"]) for r in rows]),
        "q": np.array([[float(r[f"q{j}_deg"]) for j in range(18)]
                       for r in rows]),
        "cmd": np.array([[float(r[f"cmd{j}_deg"]) for j in range(18)]
                         for r in rows]),
    }
    summary = {}
    sp = csv_path.with_name(csv_path.stem + "_summary.json")
    if sp.exists():
        summary = json.loads(sp.read_text())
    tr["summary"] = summary
    ref = (summary.get("params", {}).get("tilt_ref_deg")
           or [tr["roll"][0], tr["pitch"][0]])
    tr["ref_roll"], tr["ref_pitch"] = float(ref[0]), float(ref[1])
    return tr


# ---------------------------------------------------------------------------
# Free-base replay
# ---------------------------------------------------------------------------

class _ReplaySim:
    """Free-base sim placed at a recorded pose; mirrors the training
    env's model prep (soften_contacts) and fit_loaded_actuator's
    placement recipe so the replay sees the same plant."""

    def __init__(self, params: SimServoParams, *, mu: float = 0.0,
                 com_shift_mm: tuple[float, float, float] = (0., 0., 0.)):
        import mujoco
        from .sim_env import set_foot_ground_friction, soften_contacts
        self._mujoco = mujoco
        self.model = build_model(fixed_base=False, flat_terrain=True)
        soften_contacts(self.model)
        if mu > 0.0:
            set_foot_ground_friction(self.model, mu)
        apply_params_to_model(self.model, params)
        self.params = params
        self.data = mujoco.MjData(self.model)
        self._qadr = joint_qpos_addrs(self.model)
        self._vadr = joint_qvel_addrs(self.model)
        self._pos_act = position_actuator_ids(self.model)
        self._chassis_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        if any(abs(c) > 1e-9 for c in com_shift_mm):
            self.model.body_ipos[self._chassis_bid] += (
                np.asarray(com_shift_mm) * 1e-3)
        gid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
        self._gyro_adr = self.model.sensor_adr[gid] if gid >= 0 else -1
        self._pad_bids = [mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_pad")
            for i in range(6)]
        self._touch_adr = []
        for i in range(6):
            sid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, f"L{i}_foot_t")
            self._touch_adr.append(
                self.model.sensor_adr[sid] if sid >= 0 else -1)

    def place(self, q_rad: np.ndarray) -> None:
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

    def roll_pitch(self) -> tuple[float, float]:
        """Chassis attitude in the IMU convention (roll=atan2(ay,az))."""
        R = np.asarray(self.data.xmat[self._chassis_bid],
                       dtype=float).reshape(3, 3)
        f = R.T @ np.array([0.0, 0.0, 9.80665])
        return (math.atan2(f[1], f[2]),
                math.atan2(-f[0], math.hypot(f[1], f[2])))

    def gyro_dps(self) -> np.ndarray:
        if self._gyro_adr >= 0:
            return (np.asarray(
                self.data.sensordata[self._gyro_adr:self._gyro_adr + 3])
                * RAD2DEG)
        return np.asarray(self.data.qvel[3:6]) * RAD2DEG

    def foot_state(self) -> tuple[np.ndarray, np.ndarray]:
        """(contact force per foot, pad world xyz per foot)."""
        f = np.array([
            float(self.data.sensordata[a]) if a >= 0 else 0.0
            for a in self._touch_adr])
        xyz = np.array([self.data.xpos[b] if b >= 0 else np.zeros(3)
                        for b in self._pad_bids])
        return f, xyz

    def replay(self, tr: dict, *, settle_s: float = 0.4,
               speed_deg_s: float = RL_SPEED_DEG_S,
               acc_units: float = RL_ACC_UNITS,
               fold_roll_deg: float = 0.0) -> dict:
        """Feed the recorded cmd stream; sample sim state per tick.

        ``fold_roll_deg``: persistent one-side fold bias added to the
        physical command, the exact ``sim_env._rise_rock_offset``
        mapping (target body roll / TIP_ROLL_PER_FOLD -> hip/knee
        fold on the side the roll leans toward). Used to CALIBRATE
        the rise-rock DR dose against the recorded failure.
        """
        mujoco = self._mujoco
        fold_dq = np.zeros(18)
        if abs(fold_roll_deg) > 1e-9:
            from .sim_env import SimHexapodBalanceEnv
            fold = (abs(fold_roll_deg)
                    / SimHexapodBalanceEnv.TIP_ROLL_PER_FOLD * DEG2RAD)
            legs = (3, 4, 5) if fold_roll_deg > 0 else (0, 1, 2)
            for leg in legs:
                fold_dq[3 * leg + 1] -= fold
                fold_dq[3 * leg + 2] += 0.5 * fold
        q0 = tr["q"][0] * DEG2RAD
        self.place(q0)
        dt = self.model.opt.timestep
        for _ in range(int(settle_s / dt)):
            self.data.ctrl[self._pos_act] = q0
            mujoco.mj_step(self.model, self.data)
        roll0, pitch0 = self.roll_pitch()

        profile = ServoProfile(self.params,
                               self.data.qpos[self._qadr].copy())
        cmd = tr["cmd"] * DEG2RAD + fold_dq
        t_hw = tr["t"]
        n = len(cmd)
        out = {k: [] for k in ("roll", "pitch", "gyro_x", "gyro_y", "q",
                               "foot_f", "foot_xyz")}
        for k in range(n):
            profile.command(cmd[k], speed_deg_s=speed_deg_s,
                            acc_units=acc_units)
            # Honor the recorded inter-tick duration (loop overruns).
            tick_s = (t_hw[k + 1] - t_hw[k]) if k + 1 < n else 0.04
            per = max(1, int(round(min(max(tick_s, 0.02), 0.12) / dt)))
            for _ in range(per):
                self.data.ctrl[self._pos_act] = profile.tick(dt)
                mujoco.mj_step(self.model, self.data)
            r, p = self.roll_pitch()
            g = self.gyro_dps()
            f, xyz = self.foot_state()
            out["roll"].append(r * RAD2DEG)
            out["pitch"].append(p * RAD2DEG)
            out["gyro_x"].append(float(g[0]))
            out["gyro_y"].append(float(g[1]))
            out["q"].append(self.data.qpos[self._qadr] * RAD2DEG)
            out["foot_f"].append(f)
            out["foot_xyz"].append(xyz)
        res = {k: np.asarray(v) for k, v in out.items()}
        res["ref_roll"] = roll0 * RAD2DEG
        res["ref_pitch"] = pitch0 * RAD2DEG
        return res


# ---------------------------------------------------------------------------
# Divergence analysis
# ---------------------------------------------------------------------------

def _first_sustained(mask: np.ndarray, sustain: int = SUSTAIN_TICKS) -> int:
    """First index where ``mask`` stays True for ``sustain`` ticks; -1."""
    run = 0
    for i, m in enumerate(mask):
        run = run + 1 if m else 0
        if run >= sustain:
            return i - sustain + 1
    return -1


def analyze(tr: dict, sim: dict) -> dict:
    hw_roll_rel = tr["roll"] - tr["ref_roll"]
    sim_roll_rel = sim["roll"] - sim["ref_roll"]
    d_roll = np.abs(sim_roll_rel - hw_roll_rel)
    roll_tick = _first_sustained(d_roll > ROLL_DIV_DEG)

    moving = np.ptp(tr["q"], axis=0) > MOVING_PTP_DEG
    dq = np.abs(sim["q"] - tr["q"])            # (n, 18) deg
    q_tick, q_joint = -1, -1
    for j in np.flatnonzero(moving):
        t_j = _first_sustained(dq[:, j] > Q_DIV_DEG)
        if t_j >= 0 and (q_tick < 0 or t_j < q_tick):
            q_tick, q_joint = t_j, int(j)
    q_rmse = float(np.sqrt(np.mean(dq[:, moving] ** 2))) if moving.any() \
        else float("nan")

    # Contact events: per-foot liftoff / touchdown ticks + loaded slip.
    f = sim["foot_f"]
    on = f > CONTACT_N
    xy = sim["foot_xyz"][:, :, :2]
    slip = np.zeros_like(f)
    slip[1:] = np.linalg.norm(np.diff(xy, axis=0), axis=2) \
        * (on[1:] & on[:-1])
    events = []
    for foot in range(6):
        ch = np.flatnonzero(np.diff(on[:, foot].astype(int)) != 0)
        for k in ch:
            events.append((int(k) + 1, foot,
                           "touchdown" if on[k + 1, foot] else "liftoff"))
    events.sort()

    # Peak rel-roll comparison — did the sim reproduce the excursion?
    hw_peak = float(np.max(np.abs(hw_roll_rel)))
    sim_peak = float(np.max(np.abs(sim_roll_rel)))

    return {
        "roll_div_tick": int(roll_tick),
        "q_div_tick": int(q_tick), "q_div_joint": q_joint,
        "q_rmse_moving_deg": round(q_rmse, 2),
        "hw_peak_roll_rel_deg": round(hw_peak, 2),
        "sim_peak_roll_rel_deg": round(sim_peak, 2),
        "hw_roll_rel": hw_roll_rel, "sim_roll_rel": sim_roll_rel,
        "d_roll": d_roll, "dq": dq, "moving": moving,
        "slip_mm": slip * 1000.0, "contact_on": on,
        "contact_events": events,
    }


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_overlay(tr: dict, runs: dict[str, tuple[dict, dict]],
                 out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ticks = np.arange(len(tr["t"]))
    moving = np.ptp(tr["q"], axis=0) > MOVING_PTP_DEG
    top_j = np.argsort(np.ptp(tr["q"], axis=0))[::-1][:6]
    n_rows = 3 + 1
    fig, axs = plt.subplots(n_rows, 1, figsize=(13, 3.0 * n_rows),
                            sharex=True)

    ax = axs[0]
    ax.plot(ticks, tr["roll"] - tr["ref_roll"], "k-", lw=2,
            label="hardware roll_rel")
    for tag, (sim, an) in runs.items():
        ax.plot(ticks, an["sim_roll_rel"], lw=1.4,
                label=f"sim {tag} (div@{an['roll_div_tick']})")
        if an["roll_div_tick"] >= 0:
            ax.axvline(an["roll_div_tick"], ls=":", alpha=0.5)
    ax.axhline(10, color="r", ls="--", alpha=0.4, label="tilt trip 10°")
    ax.axhline(-10, color="r", ls="--", alpha=0.4)
    ax.set_ylabel("roll rel (deg)")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title(tr["name"])

    ax = axs[1]
    ax.plot(ticks, tr["gyro_x"], "k-", lw=2, label="hardware gyro_x")
    for tag, (sim, an) in runs.items():
        ax.plot(ticks, sim["gyro_x"], lw=1.2, label=f"sim {tag}")
    ax.set_ylabel("roll rate (deg/s)")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)

    ax = axs[2]
    cmap = plt.cm.tab10
    for i, j in enumerate(top_j):
        c = cmap(i % 10)
        ax.plot(ticks, tr["q"][:, j], color=c, lw=2, alpha=0.8,
                label=f"q{j} hw")
        for tag, (sim, an) in runs.items():
            ls = "--" if tag == "air" else ":"
            ax.plot(ticks, sim["q"][:, j], color=c, lw=1.2, ls=ls)
    ax.set_ylabel("q (deg) — hw solid / sim dashed(air) dotted(loaded)")
    ax.legend(loc="upper left", fontsize=7, ncol=6)
    ax.grid(alpha=0.3)

    ax = axs[3]
    tag0 = next(iter(runs))
    sim, an = runs[tag0]
    for foot in range(6):
        base = foot * 1.2
        ax.fill_between(ticks, base, base + an["contact_on"][:, foot],
                        step="mid", alpha=0.35)
        ax.plot(ticks, base + np.minimum(an["slip_mm"][:, foot], 1.0),
                lw=0.8)
        ax.text(-6, base + 0.4, f"L{foot}", fontsize=8)
    ax.set_ylabel(f"sim contact+slip ({tag0})")
    ax.set_xlabel("tick (25 Hz)")
    ax.grid(alpha=0.2)

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {out}")


# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", type=Path, nargs="+", required=True)
    ap.add_argument("--servo-params", default="both",
                    choices=("air", "loaded", "both"))
    ap.add_argument("--mu", type=float, default=0.0,
                    help="foot-ground slide friction override "
                         "(0 = XML default; large ~50 = pinned feet)")
    ap.add_argument("--com-shift-mm", type=str, default="0,0,0",
                    help="chassis CoM shift in mm, 'x,y,z' "
                         "(+y = left side; -y shifts CoM right)")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="plot/JSON output dir (default: alongside csv)")
    ap.add_argument("--json-out", type=Path, default=None)
    args = ap.parse_args(argv)

    param_sets = {}
    if args.servo_params in ("air", "both"):
        param_sets["air"] = SimServoParams.load()
    if args.servo_params in ("loaded", "both"):
        param_sets["loaded"] = SimServoParams.load(LOADED_MODEL_PATH)

    all_out = {}
    for csv_path in args.csv:
        tr = load_trace(csv_path)
        n = len(tr["t"])
        print(f"\n[{tr['name']}] {n} run ticks, "
              f"hw peak roll_rel "
              f"{np.max(np.abs(tr['roll'] - tr['ref_roll'])):.1f} deg")
        runs = {}
        rec = {}
        com_shift = tuple(float(x) for x in args.com_shift_mm.split(","))
        for tag, params in param_sets.items():
            sim = _ReplaySim(params, mu=args.mu, com_shift_mm=com_shift)
            res = sim.replay(tr)
            an = analyze(tr, res)
            runs[tag] = (res, an)
            rec[tag] = {k: an[k] for k in (
                "roll_div_tick", "q_div_tick", "q_div_joint",
                "q_rmse_moving_deg", "hw_peak_roll_rel_deg",
                "sim_peak_roll_rel_deg")}
            first_events = ", ".join(
                f"t{t}:L{f}:{e}" for t, f, e in an["contact_events"][:8])
            print(f"  [{tag:>6}] roll div tick "
                  f"{an['roll_div_tick']:>4}  q div tick "
                  f"{an['q_div_tick']:>4} (j{an['q_div_joint']})  "
                  f"q RMSE {an['q_rmse_moving_deg']:.2f} deg  "
                  f"sim peak roll {an['sim_peak_roll_rel_deg']:.1f} vs hw "
                  f"{an['hw_peak_roll_rel_deg']:.1f}")
            if first_events:
                print(f"           contact events: {first_events}")
        all_out[tr["name"]] = rec
        if args.plot:
            out_dir = args.out_dir or csv_path.parent
            out_dir.mkdir(parents=True, exist_ok=True)
            plot_overlay(tr, runs,
                         out_dir / (csv_path.stem + ".replay.png"))

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(all_out, indent=2))
        print(f"\nwrote {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
