"""Held-out 60 s session gate for CPG gait-search winners (cpg track).

`paper_cpg_search` optimizes a low-dimensional SE2FootGait parameter
vector against short fixed-command rollouts. This harness is the
held-out DONE-gate check for a winner: ONE continuous 60 s MuJoCo
session per panel driving a seeded command SCRIPT of headings (angles
the search never trained on), turns in both directions, and
stops/restarts, scored segment-by-segment with the same behavioral
metrics the search uses (commanded progress, cross-track error,
loaded-foot slip, falls, tilt) plus the eval_checkpoint gait-validity
rule (no persistently sacrificed leg). It emits one JSON verdict, an
mp4 per panel, and a contact-sheet PNG.

Panels: `dr0` replicates the search conditions (XML friction, air
servo fit, write 1500/acc 80). `--robust` adds a modest robustness
panel: a second reset seed, friction 1.2 / 0.8 (XML default is 2.0),
and the loaded-bench servo fit. Overall PASS requires every panel to
pass.

Thresholds (assume-and-go, recorded 2026-08-23; tune only with a
logged reason):
  * zero terminations (falls) anywhere;
  * gait valid: no leg with session duty <0.10, or >0.95 with 0 swings;
  * heading segments: progress_frac >= 0.55 each, mean >= 0.70,
    cross_frac <= 0.35 each;
  * turn segments: yaw_along_frac in [0.70, 1.30] each;
  * stop segments: XY drift <= 0.06 m and |yaw drift| <= 0.25 rad each
    (the SE2 scheduler marches in place on a zero command by design —
    drift, not foot motion, is what we gate);
  * session slip: slip / (progress + 0.17*|yaw|) <= 2.9 (the teacher
    band cap the joystick gate uses).

Usage:
    python -m rl_move.sim.eval_cpg_gate \
        --params-from logs/paper_cpg_search/paper-cpg-contextual250-20260822T21Z.json \
        --name contextual250-winner --robust \
        --export rl_move/sim/policies/cpg_controller_contextual250.json
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.paper_cpg_search import GaitParams, _clip_params  # noqa: E402
from rl_move.sim.verify_noslip import (  # noqa: E402
    HOLD_S, PLANT_HIP_DEG, PLANT_KNEE_DEG, _make_env,
)

# Search-time training angles (deg) for the contextual suite; held-out
# headings are drawn away from these.
_TRAIN_ANGLES_DEG = (0.0, 35.0, -35.0, 70.0, -70.0)


@dataclass(frozen=True)
class Segment:
    name: str
    kind: str          # "heading" | "turn" | "stop"
    dur_s: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def build_script(seed: int, session_s: float, speed: float,
                 wz: float) -> list[Segment]:
    """Seeded held-out 60 s command script: stops, headings, both turns.

    Skeleton is fixed (so every winner faces the same segment types);
    heading ANGLES are drawn >=12 deg away from every search-time
    training angle and >=25 deg from each other.
    """
    rng = np.random.default_rng(seed)
    angles: list[float] = []
    while len(angles) < 5:
        a = float(rng.uniform(-95.0, 95.0))
        if any(abs(a - t) < 12.0 for t in _TRAIN_ANGLES_DEG):
            continue
        if any(abs(a - b) < 25.0 for b in angles):
            continue
        angles.append(a)

    def head(i: float, dur: float) -> Segment:
        a = math.radians(angles[int(i)])
        return Segment(f"head{angles[int(i)]:+.0f}", "heading", dur,
                       vx=speed * math.cos(a), vy=speed * math.sin(a))

    left_first = bool(rng.integers(0, 2))
    w1 = abs(wz) if left_first else -abs(wz)
    skeleton = [
        Segment("stop_start", "stop", 2.0),
        head(0, 8.0),
        head(1, 8.0),
        Segment("stop_mid1", "stop", 3.0),          # stop after motion
        Segment("turn_a", "turn", 7.0, wz=w1),      # restart into a turn
        head(2, 8.0),
        Segment("turn_b", "turn", 7.0, wz=-w1),
        Segment("stop_mid2", "stop", 3.0),
        head(3, 8.0),
        head(4, 6.0),
    ]
    total = sum(s.dur_s for s in skeleton)
    if abs(total - session_s) > 1e-6:
        # Rescale to the requested session length.
        k = session_s / total
        skeleton = [Segment(s.name, s.kind, s.dur_s * k, s.vx, s.vy, s.wz)
                    for s in skeleton]
    return skeleton


def _body_yaw(env) -> float:
    r = env.data.xmat[env._chassis_bid].reshape(3, 3)
    return math.atan2(float(r[1, 0]), float(r[0, 0]))


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _body_dir_world(env, vx: float, vy: float) -> np.ndarray:
    r = env.data.xmat[env._chassis_bid].reshape(3, 3)
    v = r @ np.asarray([vx, vy, 0.0], dtype=float)
    xy = np.asarray(v[:2], dtype=float)
    return xy / max(float(np.linalg.norm(xy)), 1e-9)


def run_session(env, params: GaitParams, script: list[Segment], *,
                seed: int, video_path: Path | None) -> dict:
    """One continuous plant-hold + scripted 60 s walk; per-segment metrics."""
    from sim_gait_compat import SE2FootGait

    try:
        env.reset(seed=seed)
    except TypeError:
        env.reset()

    gait = SE2FootGait(
        gait=params.gait, period=params.period,
        swing_frac=params.swing_frac, lift=params.lift_m,
        cmd_tau=params.cmd_tau, workspace_margin=params.workspace_margin,
    )
    gait.sync_plant_stance(PLANT_HIP_DEG, PLANT_KNEE_DEG)
    gait.reset_phase(t=0.0)
    gait.stop()

    writer = None
    if video_path is not None:
        import cv2
        writer = cv2.VideoWriter(
            str(video_path), cv2.VideoWriter_fourcc(*"mp4v"),
            int(round(1.0 / env.dt)), (640, 480))

    plant_rad = np.asarray(
        [0.0, PLANT_HIP_DEG, PLANT_KNEE_DEG] * 6, dtype=float) * DEG2RAD
    hold_steps = int(round(HOLD_S / env.dt))
    seg_steps = [max(1, int(round(s.dur_s / env.dt))) for s in script]
    boundaries = np.cumsum([0] + seg_steps)

    segments_out: list[dict] = []
    seg_i = -1
    seg: Segment | None = None
    seg_state: dict = {}
    terminated = ""
    term_segment = None
    contact_hist: list[np.ndarray] = []
    pad_prev = None
    touch_prev = None
    slip_session = 0.0
    progress_session = 0.0
    yaw_abs_session = 0.0
    rolls: list[float] = []
    pitches: list[float] = []
    heights: list[float] = []

    def _seg_begin(i: int) -> None:
        nonlocal seg_i, seg, seg_state
        seg_i, seg = i, script[i]
        gait.set_velocity(vx=seg.vx, vy=seg.vy, omega=seg.wz)
        seg_state = {
            "xy0": env.data.xpos[env._chassis_bid, :2].copy(),
            "yaw0": _body_yaw(env),
            "yaw_prev": _body_yaw(env),
            "yaw_cum": 0.0,
            "u_world": (_body_dir_world(env, seg.vx, seg.vy)
                        if seg.kind == "heading" else None),
            "slip_m": 0.0,
        }

    def _seg_end() -> None:
        nonlocal progress_session, yaw_abs_session
        if seg is None:
            return
        delta = env.data.xpos[env._chassis_bid, :2] - seg_state["xy0"]
        cmd_dist = math.hypot(seg.vx, seg.vy) * seg.dur_s
        yaw_d = seg_state["yaw_cum"]
        row: dict = {
            "name": seg.name, "kind": seg.kind,
            "dur_s": round(seg.dur_s, 2),
            "command": {"vx": seg.vx, "vy": seg.vy, "wz": seg.wz},
            "slip_m": round(seg_state["slip_m"], 4),
            "yaw_delta_rad": round(yaw_d, 4),
        }
        if seg.kind == "heading":
            u = seg_state["u_world"]
            prog = float(np.dot(delta, u))
            cross = abs(float(u[0] * delta[1] - u[1] * delta[0]))
            row |= {
                "progress_m": round(prog, 4),
                "cmd_dist_m": round(cmd_dist, 4),
                "progress_frac": round(prog / max(cmd_dist, 1e-9), 4),
                "cross_m": round(cross, 4),
                "cross_frac": round(cross / max(cmd_dist, 0.05), 4),
            }
            progress_session += max(prog, 0.0)
        elif seg.kind == "turn":
            tgt = seg.wz * seg.dur_s
            row |= {
                "yaw_target_rad": round(tgt, 4),
                "yaw_along_frac": round(
                    yaw_d * np.sign(seg.wz) / abs(tgt), 4),
                "drift_m": round(float(np.linalg.norm(delta)), 4),
            }
        else:  # stop
            row |= {
                "drift_m": round(float(np.linalg.norm(delta)), 4),
                "yaw_drift_rad": round(abs(yaw_d), 4),
            }
        yaw_abs_session += abs(yaw_d)
        segments_out.append(row)

    total_steps = hold_steps + int(boundaries[-1])
    for step in range(total_steps):
        walking = step >= hold_steps
        if walking:
            w = step - hold_steps
            if seg_i + 1 < len(script) and w >= boundaries[seg_i + 1]:
                _seg_end()
                _seg_begin(seg_i + 1)
            elif seg is None:
                _seg_begin(0)
            t = w * env.dt
            q_rad = np.asarray(gait.desired_deg(t), dtype=float) * DEG2RAD
        else:
            q_rad = plant_rad
        _obs, _r, term, trunc, info = env.step(q_rad_to_action(q_rad))
        if walking:
            yaw_now = _body_yaw(env)
            seg_state["yaw_cum"] += _wrap_angle(
                yaw_now - seg_state["yaw_prev"])
            seg_state["yaw_prev"] = yaw_now
            touch = np.asarray([
                float(env.data.sensordata[a]) > 0.5
                for a in env._touch_adr], dtype=bool)
            contact_hist.append(touch)
            pad_now = env.data.xpos[env._pad_bids, :2].copy()
            if pad_prev is not None and touch_prev is not None:
                moved = np.linalg.norm(pad_now - pad_prev, axis=1)
                s = float(moved[touch_prev].sum())
                seg_state["slip_m"] += s
                slip_session += s
            pad_prev, touch_prev = pad_now, touch
            st = env._state
            heights.append(float(env.data.xpos[env._chassis_bid, 2]))
            rolls.append(abs(float(st.imu_roll)) * 180.0 / math.pi)
            pitches.append(abs(float(st.imu_pitch)) * 180.0 / math.pi)
            if writer is not None:
                import cv2
                frame = env.render()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.putText(frame, f"{seg.name} t={w * env.dt:5.1f}s",
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
                writer.write(frame)
        if term or trunc:
            done_early = step < total_steps - 1
            if done_early:
                terminated = str(info.get("termination_reason") or "trunc")
                term_segment = seg.name if seg else "hold"
            break
    _seg_end()
    if writer is not None:
        writer.release()

    contact = (np.asarray(contact_hist, dtype=bool)
               if contact_hist else np.zeros((1, 6), dtype=bool))
    duty = contact.mean(axis=0)
    swings = []
    for f in range(6):
        d = np.diff(contact[:, f].astype(int))
        swings.append(int((d == -1).sum()))
    sacrificed = [f for f in range(6)
                  if duty[f] < 0.10 or (duty[f] > 0.95 and swings[f] == 0)]

    slip_den = max(progress_session + 0.17 * yaw_abs_session, 0.05)
    return {
        "terminated": terminated,
        "term_segment": term_segment,
        "segments": segments_out,
        "duty_cycle": [round(float(x), 3) for x in duty],
        "swing_count": swings,
        "sacrificed_legs": sacrificed,
        "gait_valid": not sacrificed,
        "slip_m_total": round(slip_session, 4),
        "progress_m_total": round(progress_session, 4),
        "yaw_abs_rad_total": round(yaw_abs_session, 4),
        "slip_per_m": round(slip_session / slip_den, 4),
        "height_mean_m": (round(float(np.mean(heights)), 4)
                          if heights else None),
        "roll_peak_deg": round(float(max(rolls)), 2) if rolls else 0.0,
        "pitch_peak_deg": round(float(max(pitches)), 2) if pitches else 0.0,
        "ik_failures": int(gait.ik_failures),
        "limit_clips": int(gait.limit_clips),
        "command_scale_last": float(gait.last_command_scale),
    }


# ---- gate thresholds (see module docstring) --------------------------------
TH = {
    "heading_progress_frac_min": 0.55,
    "heading_progress_frac_mean": 0.70,
    "heading_cross_frac_max": 0.35,
    "turn_yaw_along_lo": 0.70,
    "turn_yaw_along_hi": 1.30,
    "stop_drift_m_max": 0.06,
    "stop_yaw_drift_rad_max": 0.25,
    "slip_per_m_max": 2.9,   # teacher-band cap (joystick gate)
}


def judge_panel(sess: dict) -> dict:
    heads = [s for s in sess["segments"] if s["kind"] == "heading"]
    turns = [s for s in sess["segments"] if s["kind"] == "turn"]
    stops = [s for s in sess["segments"] if s["kind"] == "stop"]
    pf = [s["progress_frac"] for s in heads]
    checks = {
        "zero_falls": not sess["terminated"],
        "gait_valid": bool(sess["gait_valid"]),
        "headings_each": bool(pf) and all(
            p >= TH["heading_progress_frac_min"] for p in pf),
        "headings_mean": bool(pf) and (
            float(np.mean(pf)) >= TH["heading_progress_frac_mean"]),
        "headings_cross": all(
            s["cross_frac"] <= TH["heading_cross_frac_max"] for s in heads),
        "turns": bool(turns) and all(
            TH["turn_yaw_along_lo"] <= s["yaw_along_frac"]
            <= TH["turn_yaw_along_hi"] for s in turns),
        "stops": bool(stops) and all(
            s["drift_m"] <= TH["stop_drift_m_max"]
            and s["yaw_drift_rad"] <= TH["stop_yaw_drift_rad_max"]
            for s in stops),
        "slip_ok": sess["slip_per_m"] <= TH["slip_per_m_max"],
    }
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "heading_progress_frac_mean": (round(float(np.mean(pf)), 4)
                                       if pf else None),
        "heading_progress_frac_min": (round(float(np.min(pf)), 4)
                                      if pf else None),
        "turn_yaw_along": [s["yaw_along_frac"] for s in turns],
        "stop_drift_m": [s["drift_m"] for s in stops],
        "slip_per_m": sess["slip_per_m"],
    }


def _sheet(video: Path, n: int = 8) -> Path | None:
    import cv2
    cap = cv2.VideoCapture(str(video))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        return None
    frames = []
    for i in np.linspace(0, total - 1, n).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, f = cap.read()
        if ok:
            frames.append(cv2.resize(f, (f.shape[1] // 2, f.shape[0] // 2)))
    cap.release()
    if not frames:
        return None
    out = video.with_name(video.stem + "_sheet.png")
    cv2.imwrite(str(out), np.hstack(frames))
    return out


def _git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
            text=True).strip()
    except Exception:
        return "unknown"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params-json", default="",
                    help="inline JSON GaitParams dict")
    ap.add_argument("--params-from", default="",
                    help="paper_cpg_search report JSON; uses best.params")
    ap.add_argument("--name", required=True,
                    help="artifact/output name, e.g. contextual250-winner")
    ap.add_argument("--script-seed", type=int, default=20260823)
    ap.add_argument("--session-s", type=float, default=60.0)
    ap.add_argument("--speed", type=float, default=0.04)
    ap.add_argument("--wz", type=float, default=0.20)
    ap.add_argument("--seed", type=int, default=20260823)
    ap.add_argument("--robust", action="store_true",
                    help="add seed2/mu1.2/mu0.8/loaded panels after dr0")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--export", default="",
                    help="on overall PASS, write the named controller "
                         "artifact JSON here")
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args(argv)

    if bool(args.params_json) == bool(args.params_from):
        ap.error("pass exactly one of --params-json / --params-from")
    if args.params_json:
        raw = json.loads(args.params_json)
        source = "inline"
    else:
        rep = json.loads(Path(args.params_from).read_text())
        raw = rep["best"]["params"]
        source = str(args.params_from)
    params = _clip_params(GaitParams(**raw))

    out_dir = args.out_dir or (ROOT / "logs" / "cpg_gate" / args.name)
    out_dir.mkdir(parents=True, exist_ok=True)
    script = build_script(args.script_seed, args.session_s, args.speed,
                          args.wz)
    print("[eval_cpg_gate] script: " + " -> ".join(
        f"{s.name}({s.dur_s:.0f}s)" for s in script))

    # NOTE: with randomize=False the env is deterministic — varying the
    # reset seed reproduces the identical rollout (verified 08-23:
    # seed+100003 was bit-identical to dr0). The honest second held-out
    # axis is a DIFFERENT COMMAND SCRIPT, not a different env seed.
    script2 = build_script(args.script_seed + 1, args.session_s,
                           args.speed, args.wz)
    panels = [("dr0", dict(mu=0.0, servo_params="", script=script))]
    if args.robust:
        panels += [
            ("dr0_script2", dict(mu=0.0, servo_params="", script=script2)),
            ("mu12", dict(mu=1.2, servo_params="", script=script)),
            ("mu08", dict(mu=0.8, servo_params="", script=script)),
            ("loaded", dict(mu=0.0, servo_params="loaded", script=script)),
        ]

    episode_s = HOLD_S + args.session_s + 2.0
    results: dict[str, dict] = {}
    for pname, pcfg in panels:
        t0 = time.time()
        env = _make_env(
            pcfg["mu"], pcfg["servo_params"], args.seed,
            episode_s=episode_s, render=not args.no_video,
            write_speed=1500, write_acc=80, vel_max_deg_s=None)
        video = None if args.no_video else out_dir / f"session_{pname}.mp4"
        try:
            sess = run_session(env, params, pcfg["script"], seed=args.seed,
                               video_path=video)
        finally:
            env.close()
        verdict = judge_panel(sess)
        sheet = _sheet(video) if video is not None else None
        pcfg_json = {"mu": pcfg["mu"], "servo_params": pcfg["servo_params"],
                     "script": [asdict(s) for s in pcfg["script"]]}
        results[pname] = {
            "panel_cfg": pcfg_json, "session": sess, "verdict": verdict,
            "video": str(video) if video else None,
            "sheet": str(sheet) if sheet else None,
        }
        print(f"[panel {pname}] pass={verdict['pass']} "
              f"prog_mean={verdict['heading_progress_frac_mean']} "
              f"turns={verdict['turn_yaw_along']} "
              f"slip/m={verdict['slip_per_m']} "
              f"term={sess['terminated'] or 'none'} "
              f"sac={sess['sacrificed_legs']} "
              f"({time.time() - t0:.0f}s)", flush=True)

    overall = all(r["verdict"]["pass"] for r in results.values())
    gate = {
        "pass": overall,
        "name": args.name,
        "params": asdict(params),
        "params_source": source,
        "thresholds": TH,
        "script": [asdict(s) for s in script],
        "script_seed": args.script_seed,
        "session_s": args.session_s,
        "panels": {k: {"pass": v["verdict"]["pass"],
                       "checks": v["verdict"]["checks"],
                       "slip_per_m": v["verdict"]["slip_per_m"],
                       "heading_progress_frac_mean":
                           v["verdict"]["heading_progress_frac_mean"],
                       "turn_yaw_along": v["verdict"]["turn_yaw_along"]}
                   for k, v in results.items()},
        "git_sha": _git_sha(),
        "created_unix": time.time(),
    }
    (out_dir / "gate_verdict.json").write_text(
        json.dumps(gate, indent=2, sort_keys=True) + "\n")
    (out_dir / "gate_detail.json").write_text(
        json.dumps({"gate": gate, "results": results}, indent=2,
                   sort_keys=True) + "\n")
    print(f"[eval_cpg_gate] overall pass={overall} -> "
          f"{out_dir / 'gate_verdict.json'}")

    if args.export and overall:
        art = {
            "kind": "cpg_se2_controller",
            "name": args.name,
            "params": asdict(params),
            "plant_stance_deg": {"hip": PLANT_HIP_DEG,
                                 "knee": PLANT_KNEE_DEG},
            "knee_convention": "sim_relative (load via "
                               "linux_control/sim_gait_compat.SE2FootGait)",
            "provenance": {
                "params_source": source,
                "gate_verdict": str(out_dir / "gate_verdict.json"),
                "git_sha": _git_sha(),
                "created_unix": time.time(),
            },
            "gate_summary": gate["panels"],
        }
        p = Path(args.export)
        if p.exists():
            print(f"[eval_cpg_gate] REFUSING to overwrite artifact {p}")
            return 1
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(art, indent=2, sort_keys=True) + "\n")
        print(f"[eval_cpg_gate] exported controller artifact {p}")
    elif args.export:
        print("[eval_cpg_gate] gate FAILED; artifact NOT exported")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
