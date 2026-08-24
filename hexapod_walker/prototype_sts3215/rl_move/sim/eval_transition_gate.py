"""Transition/takeoff evaluation gate: stance -> locomotion handoff.

Operator RISE_WALK_NEXT_48H directive (08-13), "P2 — Add a dedicated
transition evaluation gate": do NOT evaluate transition quality only
through total return or fall rate; measure the first ~1 s after
locomotion control becomes active SEPARATELY, and compare candidates on
IDENTICAL initial states. The hardware failure being represented
(TRANSITIONS_DIRECTIVE + bench 08-13):

    stable upright pose -> walking policy activates at zero velocity
    -> many/all 18 joints request large changes -> slew saturation
    -> abrupt support-force redistribution -> roll transient
    -> velocity command has not even started yet

MECHANISM. Start states are minted once per (start kind, seed) as full
physics snapshots (qpos/qvel/ctrl/act + ServoProfile internals + the
safety layer's slew memory) and re-installed for EVERY candidate, so
all candidates take over from bit-identical robots:

  - start "plant": the settled canonical plant reset (the convenient
    steady-state the directive warns not to test exclusively);
  - start "rise": a stance specialist (--stand) rises from belly-flat
    and holds — the takeover state deployment actually produces.

Each candidate then runs the takeover schedule on a fresh plant-frame
walk episode (eval_handoff reanchor semantics — reset at walk, snapshot
physics restored on top, nothing teleported):

    zero-command hold (locomotion ACTIVE)  args.hold-s   <- window W0
    velocity ramp 0 -> v over args.ramp-s                <- window W1
    walk at v                              args.walk-s
    ramp v -> 0 + hold                     args.stop-s   <- window W2

Windows W0 (activation), W1 (ramp start) and W2 (command back to zero)
each report, over their first ~1 s (--win-s):

    peak |roll| / |pitch| (deg, relative to the pre-takeover attitude)
    peak roll rate (deg/s, gyro x)
    max fraction of joints simultaneously slew-saturated (profile
      velocity pinned at the per-joint clamp)
    max joint-target discontinuity (deg per tick, post-SafetyLayer)
    peak servo current (A, max single servo) + peak summed current
    loaded-foot slip (mm, XY displacement of feet in contact)
    contact transitions (per-foot contact flips)
    time-to-stable (s; W0 only: gyro quiet + attitude within
      --stable-tilt-deg of pre-takeover for --stable-hold-s, all
      before the ramp starts; None = never stabilized)
    falls

A candidate is NOT promoted for merely surviving a violent handoff:
the printed verdict flags any candidate whose W0 peak roll or
simultaneous-saturation fraction exceeds the (evidence-only, override-
able) bars, independent of fall rate. Post-takeoff walking quality
(tracking error, progress ratio) is also reported so a graceful-but-
parked candidate cannot win by doing nothing.

    uv run python -m rl_move.sim.eval_transition_gate \
        --stand rl_move/sim/policies/ppo_goal_cw_stand_footlow2_hard1.zip \
        --candidates rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --starts plant,rise --seeds 0,1,2 \
        --out logs/ckpt_eval/transition_gate.json
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
from pathlib import Path

import numpy as np

RISE_PHASE_S = 12.5      # eval_handoff worst-case rise budget
CONTACT_N = 0.5          # eval_checkpoint touch-force threshold


def _set_mix(gen, **p) -> None:
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_walk = 0.0
    for k, v in p.items():
        setattr(gen, f"p_{k}", v)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stand", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_stand_footlow2_hard1.zip"),
                    help="stance specialist minting the 'rise' starts")
    ap.add_argument("--candidates", required=True,
                    help="comma list of walk/unified checkpoints to "
                         "compare on identical initial states")
    ap.add_argument("--starts", default="plant,rise",
                    help="comma list of start kinds: plant (settled "
                         "canonical stance) and/or rise (post-rise "
                         "state the --stand specialist reaches)")
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--hold-s", type=float, default=2.0,
                    help="zero-command hold after activation (W0)")
    ap.add_argument("--ramp-s", type=float, default=1.0)
    ap.add_argument("--walk-s", type=float, default=3.0)
    ap.add_argument("--stop-s", type=float, default=2.0,
                    help="ramp-to-zero + hold at the end (W2)")
    ap.add_argument("--win-s", type=float, default=1.0,
                    help="measurement window after each event")
    ap.add_argument("--stable-tilt-deg", type=float, default=3.0)
    ap.add_argument("--stable-gyro-deg-s", type=float, default=30.0)
    ap.add_argument("--stable-hold-s", type=float, default=0.3)
    ap.add_argument("--bar-roll-deg", type=float, default=6.0,
                    help="violence flag: W0 peak |roll| bar (evidence-"
                         "only default from the 08-13 bench roll "
                         "transient; override once baselined)")
    ap.add_argument("--bar-sat-frac", type=float, default=0.5,
                    help="violence flag: W0 max simultaneous slew-"
                         "saturation fraction bar")
    ap.add_argument("--stochastic", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    import mujoco
    from stable_baselines3 import PPO

    from rl_move.config import load_config
    from rl_move.env import build_obs
    from rl_move.robot_state import RAD2DEG
    from .gru_policy import is_recurrent_checkpoint, load_checkpoint_auto
    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for spec in (args.cfg_set or []):
        key, val = spec.split("=", 1)
        sect, name = key.split(".", 1)
        try:
            parsed: float | str = float(val)
        except ValueError:
            parsed = val.strip()
        cfg.setdefault(sect, {})[name] = parsed
    det = not args.stochastic

    episode_s = (args.hold_s + args.ramp_s + args.walk_s + args.stop_s
                 + RISE_PHASE_S + 5.0)

    def make_env(seed: int):
        return SimHexapodJointWalkEnv(
            params=SimServoParams.from_cfg(cfg), cfg=cfg,
            randomize=False, episode_seconds=episode_s, seed=seed)

    # ---- snapshot / restore: bit-identical takeover states -------------
    def snapshot(env) -> dict:
        d, p = env.data, env._profile
        return {
            "qpos": d.qpos.copy(), "qvel": d.qvel.copy(),
            "ctrl": d.ctrl.copy(),
            "act": d.act.copy() if d.act.size else None,
            "safe": env.safety._last_safe.copy(),
            "cmd": env._cmd.copy(),
            "prof": {
                "goal": p.goal.copy(), "target": p.target.copy(),
                "_v": p._v.copy(), "_vel_now": p._vel_now.copy(),
                "_acc_now": p._acc_now.copy(),
                "_queue": [(t, q.copy(), v.copy(), a.copy())
                           for t, q, v, a in p._queue],
                "_t": p._t,
            },
        }

    def restore(env, snap: dict):
        """Fresh plant-frame walk episode with the snapshot's physics
        installed on top (eval_handoff reanchor semantics: references
        from a real reset at the target mode, robot state untouched)."""
        gen = env._goal_gen
        _set_mix(gen, walk=1.0)
        env.reset()
        d, p = env.data, env._profile
        d.qpos[:] = snap["qpos"]
        d.qvel[:] = snap["qvel"]
        d.ctrl[:] = snap["ctrl"]
        if snap["act"] is not None:
            d.act[:] = snap["act"]
        env.safety._last_safe = snap["safe"].copy()
        # The last written servo goal carries across the takeover (on
        # hardware a policy switch rewrites nothing until the new
        # policy's first tick); without this the first Δcmd would be
        # measured against the reset nominal — a restore artifact, not
        # a command discontinuity.
        env._cmd = snap["cmd"].copy()
        pr = snap["prof"]
        p.goal[:] = pr["goal"]
        p.target[:] = pr["target"]
        p._v[:] = pr["_v"]
        p._vel_now[:] = pr["_vel_now"]
        p._acc_now[:] = pr["_acc_now"]
        p._queue = [(t, q.copy(), v.copy(), a.copy())
                    for t, q, v, a in pr["_queue"]]
        p._t = pr["_t"]
        mujoco.mj_forward(env.model, env.data)
        env._state = env._read_state()
        return env._final_obs(
            build_obs(env.cfg, env._state, env._q_nom,
                      env._prev_action, goal=env._current_goal(),
                      tilt_ref=env._tilt_ref0), reset=True)

    # ---- mint start snapshots (once; shared by every candidate) --------
    print(f"[gate] minting start snapshots "
          f"(starts={args.starts} seeds={args.seeds})")
    stand = PPO.load(args.stand, device="cpu")
    n_stand = int(stand.observation_space.shape[0])
    starts = [s.strip() for s in args.starts.split(",") if s.strip()]
    seeds = [int(s) for s in args.seeds.split(",") if s != ""]
    snaps: dict[tuple[str, int], dict] = {}
    for seed in seeds:
        env = make_env(args.seed + seed)
        gen = env._goal_gen
        dt = env.dt
        for kind in starts:
            if kind == "plant":
                _set_mix(gen, walk=1.0)
                env.reset()
                snaps[(kind, seed)] = snapshot(env)
                continue
            if kind != "rise":
                raise SystemExit(f"unknown start kind {kind!r}")
            _set_mix(gen, rise=1.0)
            gen.force_rise_start = "flat"
            obs, _ = env.reset()
            gen.force_rise_start = None
            fell = None
            for _ in range(int(round(RISE_PHASE_S / dt))):
                a, _st = stand.predict(obs[:n_stand], deterministic=True)
                obs, _rw, term, trunc, info = env.step(a)
                if term or trunc:
                    fell = str(info.get("termination_reason"))
                    break
            if fell is not None:
                raise SystemExit(
                    f"[gate] stance prelude fell ({fell}) at seed "
                    f"{seed} — cannot mint a 'rise' start; check the "
                    f"--stand checkpoint")
            h_err = (float(env.data.xpos[env._chassis_bid, 2])
                     - (env._z0 + env._h_target))
            ok, detail = env.plant_report(height_err_m=h_err)
            if not ok:
                bad = [k for k, v in detail.items()
                       if k.endswith("_ok") and not v]
                print(f"[gate] WARN seed {seed}: rise start minted from "
                      f"an imperfect plant ({bad}) — kept on purpose "
                      f"(imperfect handoffs are in scope)")
            snaps[(kind, seed)] = snapshot(env)
        env.close()

    # ---- takeover schedule ---------------------------------------------
    def schedule(dt: float):
        """Yields (t_idx, vx, event) with event set on window starts."""
        idx = 0

        def phase(seconds, fn, event=None):
            nonlocal idx
            n = max(1, int(round(seconds / dt)))
            for i in range(n):
                yield idx, fn(i / max(n - 1, 1)), (event if i == 0
                                                   else None)
                idx += 1

        yield from phase(args.hold_s, lambda f: 0.0, "activate")
        yield from phase(args.ramp_s, lambda f: args.speed * f,
                         "ramp_start")
        yield from phase(args.walk_s, lambda f: args.speed)
        yield from phase(args.stop_s,
                         lambda f: args.speed * max(1.0 - 2.0 * f, 0.0),
                         "stop_cmd")

    class WinMeter:
        """Directive metric set over the first win_n ticks after an
        event. Peaks are relative to the pre-takeover attitude."""

        def __init__(self, win_n: int, roll0: float, pitch0: float):
            self.win_n, self.roll0, self.pitch0 = win_n, roll0, pitch0
            self.n = 0
            self.roll = self.pitch = self.rollrate = 0.0
            self.sat_frac = self.dq = 0.0
            self.amp_one = self.amp_sum = 0.0
            self.slip_mm = 0.0
            self.flips = 0
            self.track_se = 0.0
            self.track_n = 0
            self.fall = None

        def tick(self, env, dq_deg: float, prev_contact, contact,
                 slip_mm: float, fall):
            if self.n >= self.win_n:
                return
            st = env._state
            self.roll = max(self.roll, abs(
                (st.imu_roll - self.roll0) * RAD2DEG))
            self.pitch = max(self.pitch, abs(
                (st.imu_pitch - self.pitch0) * RAD2DEG))
            gyro = getattr(st, "imu_gyro", None)
            if gyro is not None:
                self.rollrate = max(self.rollrate,
                                    abs(float(gyro[0]) * RAD2DEG))
            p = env._profile
            self.sat_frac = max(self.sat_frac, float(np.mean(
                np.abs(p._v) >= 0.999 * np.maximum(p._vel_now, 1e-9))))
            self.dq = max(self.dq, dq_deg)
            cur = getattr(st, "servo_current", None)
            if cur is not None and len(cur):
                self.amp_one = max(self.amp_one, float(np.max(cur)))
                self.amp_sum = max(self.amp_sum, float(np.sum(cur)))
            self.slip_mm += slip_mm
            if prev_contact is not None:
                self.flips += int(np.sum(prev_contact != contact))
            err = (st.joint_position - env._cmd) * RAD2DEG
            self.track_se += float(np.sum(err ** 2))
            self.track_n += err.size
            if fall and self.fall is None:
                self.fall = fall
            self.n += 1

        def report(self) -> dict:
            return {"peak_roll_deg": round(self.roll, 2),
                    "peak_pitch_deg": round(self.pitch, 2),
                    "peak_roll_rate_deg_s": round(self.rollrate, 1),
                    "max_sat_frac": round(self.sat_frac, 3),
                    "max_target_disc_deg": round(self.dq, 2),
                    "peak_servo_a": round(self.amp_one, 2),
                    "peak_sum_a": round(self.amp_sum, 2),
                    "loaded_slip_mm": round(self.slip_mm, 1),
                    "contact_flips": self.flips,
                    "track_rmse_deg": round(math.sqrt(
                        self.track_se / max(self.track_n, 1)), 2),
                    "fall": self.fall}

    def run_candidate(path: Path, kind: str, seed: int) -> dict:
        env = make_env(args.seed + seed)
        dt = env.dt
        model = load_checkpoint_auto(path, device="cpu")
        recurrent = is_recurrent_checkpoint(path)
        n_model = int(model.observation_space.shape[0])
        obs = restore(env, snaps[(kind, seed)])
        n_env = int(env.observation_space.shape[0])
        if n_model != n_env:
            env.close()
            raise SystemExit(
                f"{path.name}: checkpoint obs {n_model} != env obs "
                f"{n_env} (pass matching --cfg-set, e.g. "
                f"obs.mode_onehot=1 for dual-core checkpoints)")
        st0 = env._state
        roll0, pitch0 = st0.imu_roll, st0.imu_pitch
        win_n = max(1, int(round(args.win_s / dt)))
        meters = {ev: WinMeter(win_n, roll0, pitch0)
                  for ev in ("activate", "ramp_start", "stop_cmd")}
        active: list[WinMeter] = []
        rec_state, rec_start = None, np.ones((1,), dtype=bool)

        traj = env._goal_traj
        pads = env._pad_bids
        prev_contact = None
        prev_xy = None
        prev_cmd = env._cmd.copy()
        stable_at = None
        stable_run = 0
        stable_need = max(1, int(round(args.stable_hold_s / dt)))
        hold_n = max(1, int(round(args.hold_s / dt)))
        n_err, err_sum = 0, 0
        cmd_dist = along_dist = 0.0
        fall = None
        for t_idx, vx, event in schedule(dt):
            if hasattr(traj, "vx"):
                traj.vx[:] = vx
                traj.vy[:] = 0.0
            if getattr(traj, "wz", None) is not None:
                traj.wz[:] = 0.0
            if event is not None:
                m = meters[event]
                m.roll0, m.pitch0 = roll0, pitch0
                active.append(m)
            if recurrent:
                a, rec_state = model.policy.predict(
                    obs, state=rec_state, episode_start=rec_start,
                    deterministic=det)
                rec_start = np.zeros((1,), dtype=bool)
            else:
                a, _ = model.predict(obs, deterministic=det)
            obs, _rw, term, trunc, info = env.step(a)
            if (term or trunc) and fall is None:
                fall = str(info.get("termination_reason")
                           or "episode_end")
            cmd_now = env._cmd.copy()
            dq_deg = float(np.max(np.abs(cmd_now - prev_cmd))) * RAD2DEG
            prev_cmd = cmd_now
            contact = np.array([
                float(env.data.sensordata[adr]) > CONTACT_N
                for adr in env._touch_adr])
            xy = np.array([env.data.xpos[b, :2] for b in pads])
            slip_mm = 0.0
            if prev_xy is not None and prev_contact is not None:
                moved = np.linalg.norm(xy - prev_xy, axis=1)
                slip_mm = float(np.sum(moved[prev_contact & contact])
                                * 1000.0)
            for m in active:
                m.tick(env, dq_deg, prev_contact, contact, slip_mm,
                       fall)
            # time-to-stable: inside the zero-command hold only
            if stable_at is None and t_idx < hold_n:
                st = env._state
                gyro = getattr(st, "imu_gyro", None)
                quiet = (abs((st.imu_roll - roll0) * RAD2DEG)
                         <= args.stable_tilt_deg
                         and abs((st.imu_pitch - pitch0) * RAD2DEG)
                         <= args.stable_tilt_deg
                         and (gyro is None or float(np.max(np.abs(
                             gyro))) * RAD2DEG <= args.stable_gyro_deg_s))
                stable_run = stable_run + 1 if quiet else 0
                if stable_run >= stable_need:
                    stable_at = (t_idx + 1) * dt - args.stable_hold_s
            prev_contact, prev_xy = contact, xy
            if vx > 1e-9:
                v = env._body_vel_xy()
                err_sum += abs(float(v[0]) - vx) + abs(float(v[1]))
                n_err += 1
                cmd_dist += vx * dt
                along_dist += float(v[0]) * dt
            if term or trunc:
                break
        env.close()
        rec = {"candidate": str(path), "start": kind, "seed": seed,
               "fall": fall,
               "time_to_stable_s": (round(stable_at, 2)
                                    if stable_at is not None else None),
               "trk_err": round(err_sum / max(n_err, 1), 4),
               "prog_ratio": (round(along_dist / cmd_dist, 3)
                              if cmd_dist > 1e-6 else None),
               "windows": {ev: m.report() for ev, m in meters.items()}}
        return rec

    candidates = [Path(c) for c in args.candidates.split(",") if c]
    results = {"starts": starts, "seeds": seeds, "speed": args.speed,
               "win_s": args.win_s, "stand": str(args.stand),
               "bars": {"roll_deg": args.bar_roll_deg,
                        "sat_frac": args.bar_sat_frac},
               "records": []}
    for cand in candidates:
        for kind in starts:
            for seed in seeds:
                rec = run_candidate(cand, kind, seed)
                results["records"].append(rec)
                w0 = rec["windows"]["activate"]
                print(f"[{cand.name} {kind} s{seed}] "
                      f"W0 roll {w0['peak_roll_deg']:.1f}deg "
                      f"rate {w0['peak_roll_rate_deg_s']:.0f}deg/s "
                      f"sat {w0['max_sat_frac']:.2f} "
                      f"disc {w0['max_target_disc_deg']:.1f}deg "
                      f"slip {w0['loaded_slip_mm']:.0f}mm "
                      f"stable@{rec['time_to_stable_s']} "
                      f"fall={rec['fall']} "
                      f"prog={rec['prog_ratio']}")

    # ---- verdict: per-candidate summary + violence flags ----------------
    print(f"\n=== transition gate (win {args.win_s}s, "
          f"{len(starts)}x{len(seeds)} identical starts/candidate) ===")
    summary = {}
    for cand in candidates:
        rs = [r for r in results["records"]
              if r["candidate"] == str(cand)]
        w0s = [r["windows"]["activate"] for r in rs]
        falls = sum(1 for r in rs if r["fall"])
        peak_roll = max(w["peak_roll_deg"] for w in w0s)
        peak_sat = max(w["max_sat_frac"] for w in w0s)
        stables = [r["time_to_stable_s"] for r in rs
                   if r["time_to_stable_s"] is not None]
        progs = [r["prog_ratio"] for r in rs
                 if r["prog_ratio"] is not None]
        violent = (peak_roll > args.bar_roll_deg
                   or peak_sat > args.bar_sat_frac)
        summary[cand.name] = {
            "falls": falls, "n": len(rs),
            "w0_peak_roll_deg": peak_roll,
            "w0_peak_sat_frac": peak_sat,
            "stable_frac": len(stables) / max(len(rs), 1),
            "prog_ratio_med": (float(np.median(progs))
                               if progs else None),
            "violent_takeover": violent,
        }
        flag = ("VIOLENT (survives, but do not promote on survival "
                "alone)" if violent and falls == 0 else
                "VIOLENT+FALLS" if violent else "graceful")
        print(f"{cand.name}: falls {falls}/{len(rs)}, W0 peak roll "
              f"{peak_roll:.1f}deg, peak sat {peak_sat:.2f}, stable "
              f"{len(stables)}/{len(rs)}, prog_med "
              f"{summary[cand.name]['prog_ratio_med']} -> {flag}")
    results["summary"] = summary
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(results, indent=1))
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
