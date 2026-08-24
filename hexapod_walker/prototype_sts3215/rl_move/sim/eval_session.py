"""Session gate: the interactive play.py protocol as a scripted eval.

Why this exists (operator model tour, 2026-08-11 — see
rl_docs/MODEL_TOUR_2026-08-11.md): the deployed stance checkpoint
passes its training-profile gates yet, under the exact protocol a human
drives in play.py, (a) its belly rise stalls at 55 mm forever under the
interactive ramp and (b) commanding sit from the 142 mm walk plant
frame tips it over at ~2.5 s, deterministically. No existing eval
visits either state: eval_checkpoint samples the TRAINING goal
profiles, eval_drive never sits. This gate drives a stance+walk
checkpoint pair through the same session a human would:

  belly -> auto stand (stance rise under the _InteractiveTraj ramp,
  1.5 s blend to the walk plant, episode re-anchor)  ->  walk fwd
  ->  strafe left / right  ->  stop+hold  ->  sit  ->  stand in place
  ->  walk back  ->  hold

HARD gates (exit 1 on failure):
  - no episode termination in any phase (the sit tip-over catch),
  - rise reaches 60 mm within the 9.5 s window (play.py's own bar),
  - sit actually descends (dz <= -40 mm from the plant stand).

SOFT gates (reported always; gate only with --strict — every current
checkpoint fails them, they define the bar for future candidates):
  - |yaw drift| < 10 deg over the 12 s fwd cruise (measured today:
    +25..+52 deg, all models, always CCW),
  - per-axis achieved velocity >= 70 % of command for fwd/left/right/
    back (today: back 12-34 %, left 24-44 %),
  - body height >= 110 mm while driving (today: 65-83 mm paddle),
  - post-walk hold: peak tilt < 6 deg and >= 5 feet with load duty
    >= 0.5 (tripod hover-park catch).

    cd prototype_sts3215 && uv run python -m rl_move.sim.eval_session \
        --stance rl_move/sim/policies/<stance68>.zip \
        --walk rl_move/sim/policies/<walk72>.zip \
        [--strict] [--out report.json] [--strips dir] [--cfg-set k=v]

Exit code 0 = gate passed, 1 = failed.
"""
from __future__ import annotations

import os

# Cap math threads before numpy import (same reason as eval_checkpoint:
# unbounded per-process pools thrashed the controller's node, 08-09).
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
from pathlib import Path

import numpy as np

RISE_S = 9.5          # play.py's auto-stand window (curl 5 s + rise)
BLEND_S = 1.5         # scripted crouch->plant blend, same as play.py
RISE_Z_M = 0.06       # play.py's "stood up" bar
SIT_DZ_MM = -40.0     # hard gate: the sit must actually descend
CRUISE = 0.05         # inside every deployed candidate's trained band
YAW_DRIFT_DEG = 10.0  # soft gate bounds (measured tour baselines in
TRACK_FRAC = 0.70     # the module docstring)
DRIVE_Z_MM = 110.0
HOLD_TILT_DEG = 6.0
HOLD_DUTY_FEET = 5
HOLD_DUTY_MIN = 0.5


def _yaw_deg(env, bid: int) -> float:
    r = np.asarray(env.data.xmat[bid], float).reshape(3, 3)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _tilt_deg(env, bid: int) -> tuple[float, float]:
    r = np.asarray(env.data.xmat[bid], float).reshape(3, 3)
    return (math.degrees(math.atan2(r[2, 1], r[2, 2])),
            math.degrees(math.atan2(-r[2, 0],
                                    math.hypot(r[2, 1], r[2, 2]))))


def run_session(stance_p: Path, walk_p: Path, cfg=None,
                strips: Path | None = None) -> dict:
    """Drive one full interactive-protocol session; return the report."""
    import mujoco
    from stable_baselines3 import PPO

    from rl_move.env import TaskGoal
    from .joint_task import q_rad_to_action
    from .play import _PlayEnv
    from .servo_model import SimServoParams

    cfg_kw = {"cfg": cfg} if cfg is not None else {}
    env = _PlayEnv(params=SimServoParams.from_cfg(cfg),
                   randomize=False, episode_seconds=3600.0,
                   render_mode="rgb_array" if strips else None, **cfg_kw)
    traj = env.traj
    cbid = env.model.body("chassis").id

    stance = PPO.load(stance_p, device="cpu")
    walk = PPO.load(walk_p, device="cpu")
    n_st = int(stance.observation_space.shape[0])
    if walk.observation_space.shape != env.observation_space.shape:
        raise SystemExit(
            f"walk policy obs {walk.observation_space.shape} != env "
            f"{env.observation_space.shape}")
    if n_st >= int(env.observation_space.shape[0]):
        raise SystemExit("stance obs must be a prefix of the walk env obs")

    def z() -> float:
        return float(env.data.xpos[cbid, 2])

    def qnow() -> np.ndarray:
        return env.data.qpos[7:25].copy()

    # Plant reset captures the blend target, exactly like play.py.
    traj.start_at = "plant"
    traj.goal = TaskGoal()
    traj.vx = traj.vy = 0.0
    traj.reset_published()
    obs, _ = env.reset()
    q_plant = qnow()
    stand_z = z()

    state = {"obs": obs, "frames": []}

    def do_reset(start: str) -> None:
        traj.start_at = start
        traj.goal = TaskGoal()
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        state["obs"], _ = env.reset()

    def re_anchor_plant() -> None:
        keep = env.data.qpos[:7].copy()
        traj.start_at = "plant"
        traj.goal = TaskGoal()
        traj.vx = traj.vy = 0.0
        traj.reset_published()
        state["obs"], _ = env.reset()
        env.data.qpos[:2] = keep[:2]
        mujoco.mj_forward(env.model, env.data)

    def snap_refs_for_walk() -> None:
        # engage_walk's ref snap: the walk line trained at refs = 0.
        traj.goal.roll_ref = traj.goal.pitch_ref = 0.0
        traj.goal.height_ref = 0.0
        traj._pub.roll_ref = traj._pub.pitch_ref = 0.0
        traj._pub.height_ref = 0.0

    blend_from = {"q": None}

    def controller(phase: str, i: int) -> np.ndarray:
        if phase == "blend":
            s = min((i * env.dt) / BLEND_S, 1.0)
            return q_rad_to_action((1.0 - s) * blend_from["q"]
                                   + s * q_plant)
        if phase in ("fwd", "left", "right", "back"):
            a, _ = walk.predict(state["obs"], deterministic=True)
            return a
        a, _ = stance.predict(state["obs"][:n_st], deterministic=True)
        return a

    report: dict = {"stance": stance_p.stem, "walk": walk_p.stem,
                    "stand_z_mm": round(stand_z * 1000, 1),
                    "phases": {}, "falls": []}

    def run_phase(phase: str, seconds: float, setup) -> dict:
        setup()
        n = int(round(seconds / env.dt))
        z0, yaw0 = z(), _yaw_deg(env, cbid)
        peak_tilt, sum_v, samples = 0.0, np.zeros(2), 0
        min_z = z0
        duty_num, duty_den = np.zeros(6), 0
        fell = None
        for i in range(n):
            act = controller(phase, i)
            state["obs"], _r, term, trunc, info = env.step(act)
            r, p = _tilt_deg(env, cbid)
            peak_tilt = max(peak_tilt, abs(r), abs(p))
            min_z = min(min_z, z())
            sum_v += np.asarray(env._body_vel_xy())
            samples += 1
            if phase in ("hold", "hold2", "sit"):
                for leg in range(6):
                    adr = env._touch_adr[leg]
                    if adr >= 0 and float(env.data.sensordata[adr]) > 0.5:
                        duty_num[leg] += 1
                duty_den += 1
            if strips and i % 20 == 0:
                # PIL, not cv2: the train pods (where the watcher's
                # pre-staged session gate runs, 08-13) carry
                # PIL/imageio but no cv2. Frames stay RGB end-to-end.
                from PIL import Image, ImageDraw
                f = env.render()
                im = Image.fromarray(f).resize(
                    (220, int(220 * f.shape[0] / f.shape[1])))
                ImageDraw.Draw(im).text(
                    (4, 6), f"{phase} {i * env.dt:.0f}s",
                    fill=(240, 240, 40))
                state["frames"].append(np.asarray(im))
            if term or trunc:
                fell = info.get("termination_reason") or "episode_end"
                report["falls"].append(
                    {"phase": phase, "t_s": round(i * env.dt, 1),
                     "reason": fell})
                do_reset("plant")   # stop-and-wait rule: operator resets
                break
        out = {
            "peak_tilt_deg": round(peak_tilt, 1),
            "z_end_mm": round(z() * 1000, 1),
            "dz_mm": round((z() - z0) * 1000, 1),
            "min_z_mm": round(min_z * 1000, 1),
            "yaw_drift_deg": round(
                ((_yaw_deg(env, cbid) - yaw0 + 180) % 360) - 180, 1),
            "mean_vx": round(float(sum_v[0] / max(samples, 1)), 3),
            "mean_vy": round(float(sum_v[1] / max(samples, 1)), 3),
            "fell": fell,
        }
        if duty_den:
            out["duty"] = [round(float(d) / duty_den, 2)
                           for d in duty_num]
        return out

    def setup_rise():
        do_reset("zero")
        traj.goal.height_ref = 0.045

    def setup_blend():
        blend_from["q"] = qnow()

    def setup_vel(vx, vy):
        def f():
            snap_refs_for_walk()
            traj.vx, traj.vy = vx, vy
        return f

    def setup_stop():
        traj.vx = traj.vy = 0.0

    def setup_sit():
        traj.vx = traj.vy = 0.0
        traj.goal.height_ref = -0.06

    def setup_stand_again():
        traj.goal.height_ref = 0.0

    ph = report["phases"]
    ph["rise"] = run_phase("rise", RISE_S, setup_rise)
    rise_ok = z() > RISE_Z_M and not ph["rise"]["fell"]
    report["rise_ok"] = bool(rise_ok)
    if rise_ok:
        ph["blend"] = run_phase("blend", BLEND_S, setup_blend)
        re_anchor_plant()
    else:
        do_reset("plant")   # a human gives up and resets standing

    ph["fwd"] = run_phase("fwd", 12, setup_vel(CRUISE, 0.0))
    ph["left"] = run_phase("left", 6, setup_vel(0.0, CRUISE))
    ph["right"] = run_phase("right", 6, setup_vel(0.0, -CRUISE))
    ph["hold"] = run_phase("hold", 4, setup_stop)
    ph["sit"] = run_phase("sit", 6, setup_sit)
    ph["stand2"] = run_phase("stand2", 6, setup_stand_again)
    ph["back"] = run_phase("back", 8, setup_vel(-CRUISE, 0.0))
    ph["hold2"] = run_phase("hold2", 4, setup_stop)

    # --- gates ----------------------------------------------------------
    hard, soft = {}, {}
    hard["no_falls"] = len(report["falls"]) == 0
    hard["rise"] = rise_ok
    hard["sit_descends"] = (ph["sit"]["fell"] is None
                            and ph["sit"]["dz_mm"] <= SIT_DZ_MM)
    soft["fwd_heading"] = abs(ph["fwd"]["yaw_drift_deg"]) < YAW_DRIFT_DEG
    soft["track_fwd"] = ph["fwd"]["mean_vx"] >= TRACK_FRAC * CRUISE
    soft["track_left"] = ph["left"]["mean_vy"] >= TRACK_FRAC * CRUISE
    soft["track_right"] = ph["right"]["mean_vy"] <= -TRACK_FRAC * CRUISE
    soft["track_back"] = ph["back"]["mean_vx"] <= -TRACK_FRAC * CRUISE
    soft["drive_height"] = min(
        ph[p]["min_z_mm"] for p in ("fwd", "left", "right",
                                    "back")) >= DRIVE_Z_MM
    hold_d = ph["hold2"].get("duty", [0.0] * 6)
    soft["hold_quiet"] = (
        ph["hold2"]["peak_tilt_deg"] < HOLD_TILT_DEG
        and sum(1 for d in hold_d if d >= HOLD_DUTY_MIN) >= HOLD_DUTY_FEET)
    report["gates"] = {"hard": hard, "soft": soft}

    if strips and state["frames"]:
        from PIL import Image
        strips.mkdir(parents=True, exist_ok=True)
        frames = state["frames"]
        cols = 10
        h, w = frames[0].shape[:2]
        rows = (len(frames) + cols - 1) // cols
        sheet = np.full((rows * h, cols * w, 3), 20, np.uint8)
        for k, f in enumerate(frames):
            rr, cc = divmod(k, cols)
            sheet[rr * h:rr * h + f.shape[0],
                  cc * w:cc * w + f.shape[1]] = f
        out_png = strips / f"session_{stance_p.stem}__{walk_p.stem}.png"
        Image.fromarray(sheet).save(out_png)
        report["strip"] = str(out_png)
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stance", type=Path, required=True,
                    help="stance checkpoint (obs 68)")
    ap.add_argument("--walk", type=Path, required=True,
                    help="walk checkpoint (obs 72)")
    ap.add_argument("--strict", action="store_true",
                    help="soft gates also gate the exit code")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--strips", type=Path, default=None,
                    help="dir for the labeled session contact sheet")
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V", help="env cfg overrides (own-cfg eval)")
    args = ap.parse_args()

    cfg = None
    if args.cfg_set:
        from rl_move.config import load_config
        cfg = load_config()
        for spec in args.cfg_set:
            key, val = spec.split("=", 1)
            sect, name = key.split(".", 1)
            try:
                parsed: float | str = float(val)
            except ValueError:
                parsed = val.strip()
            cfg.setdefault(sect, {})[name] = parsed

    rep = run_session(args.stance, args.walk, cfg=cfg, strips=args.strips)

    hard, soft = rep["gates"]["hard"], rep["gates"]["soft"]
    print(f"session {rep['stance']} + {rep['walk']}  "
          f"(stand {rep['stand_z_mm']} mm)")
    for phase, m in rep["phases"].items():
        extra = f"  duty {m['duty']}" if "duty" in m else ""
        print(f"  {phase:6s} tilt {m['peak_tilt_deg']:4.1f}  "
              f"z {m['z_end_mm']:5.1f}  dz {m['dz_mm']:+6.1f}  "
              f"yaw {m['yaw_drift_deg']:+5.1f}  "
              f"v ({m['mean_vx']:+.3f},{m['mean_vy']:+.3f})"
              f"{'  FELL: ' + m['fell'] if m['fell'] else ''}{extra}")
    for name, ok in hard.items():
        print(f"  HARD {name:14s} {'PASS' if ok else 'FAIL'}")
    for name, ok in soft.items():
        print(f"  soft {name:14s} {'pass' if ok else 'fail'}")

    ok = all(hard.values()) and (all(soft.values()) if args.strict
                                 else True)
    rep["pass"] = bool(ok)
    print(f"SESSION GATE: {'PASS' if ok else 'FAIL'}"
          f"{' (strict)' if args.strict else ''}")
    if args.out:
        args.out.write_text(json.dumps(rep, indent=1))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
