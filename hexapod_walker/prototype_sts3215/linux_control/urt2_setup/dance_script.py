"""dance_script — dances as portable data: bake, validate, upload, replay.

A dance script is a JSON document of guarded primitives — the same few
moves every dance runner in inplace_demos is built from.  Because it is
data, a robot can receive a whole dance over HTTP (POST /api/dances)
and replay it with all the usual safety machinery, no code deploy.  Two
robots given the same file perform the same dance.

Format ``hexapod-dance-script/1``::

    {"format": "hexapod-dance-script/1",
     "name": "fever_dream",            # [a-z0-9_]{1,32}, unique
     "title": "shown in the web UI",
     "stands": true,                    # needs the bench stand-up plumbing
     "acts": [
       {"kind": "note",    "msg": "act I — flatline …"},
       {"kind": "torque",  "limit": 450},
       {"kind": "stream",  "label": "resurrection", "hz": 20.0,
        "max_speed": 3000, "max_acc": 200,          # optional clamps
        "tilt_guard_deg": 45.0, "speed_cap": 1.5,   # optional (quad acts)
        "frames": [[18 joint degrees], ...]},        # sampled at hz
       {"kind": "ease",    "label": "set stance", "seconds": 0.7,
        "pose": [18 joint degrees]},
       {"kind": "standup", "mode": "step"},          # baked stand-up lab mode
       {"kind": "sit_zero", "seconds": 1.5},         # ease back to sit zero
       {"kind": "limp"}]}

Replay guarantees (run_dance_script):
  - streams go through inplace_demos.stream_pose_fn — carrot-lookahead
    pursuit, stall-fight current guard, 4 A hard cap, optional IMU tilt
    guard; a FRESH tracker per stream so one hot moment can't
    insta-guard later acts;
  - every frame/pose is clamped to the hardware joint limits, torque to
    150..1000, max_speed to 3000, max_acc to 254;
  - stand-ups only run through the bench's validated stand-up modes
    (standup_fn); a standing script without that plumbing refuses to
    start instead of improvising.

Baking (bake_demo): replays an existing demo runner against recording
stubs — the runner's own stream segments, eases, torque changes, notes
and stand-ups are captured in order and its pose functions sampled at
the stream tick.  Contact-/feedback-driven demos (rise, breathe, walks)
are not bakeable: their motion depends on live servo readings.

CLI (runs on the Mac; needs the repo venv for bake):
    python dance_script.py bake dance_wild -o fever.json
    python dance_script.py bake-all -d ../dances        # every bakeable demo
    python dance_script.py push fever.json --host http://hexapod.local:8080
    python dance_script.py push-all -d ../dances --host http://robot-b.local:8080
    python dance_script.py pull fever_dream --host http://robot-b.local:8080
"""
from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

FORMAT = "hexapod-dance-script/1"
NAME_RE = re.compile(r"^[a-z0-9_]{1,32}$")
AXIS_LIMITS_DEG = ((-35.0, 35.0), (-80.0, 30.0), (-20.0, 150.0))
ALLOWED_KINDS = ("note", "torque", "stream", "ease", "standup",
                 "sit_zero", "limp")
MAX_SCRIPT_BYTES = 6_000_000
MAX_SECONDS = 600.0
MAX_FRAMES = 20_000

# Demos whose motion depends on live feedback (contact stops, servo-side
# glides, velocity mode, RL) — a baked frame table cannot reproduce them.
UNBAKEABLE_PREFIXES = ("breathe", "shimmy_v", "rise", "walk", "quad",
                       "stand_hands", "plant_", "dance_walk")
# dance: the classic runner does raw bus ops the recorder can't stub
# (superseded by dance_wild).  dance_swarm_stand: its wall-clock pacing
# records a distorted timeline against instant stubs (572 s vs the
# 252 s song) — run the built-in instead.
UNBAKEABLE_NAMES = ("dance", "dance_swarm_stand")


def _clamp_pose(pose) -> list[float]:
    out = []
    for j, v in enumerate(pose[:18]):
        lo, hi = AXIS_LIMITS_DEG[j % 3]
        out.append(max(lo, min(hi, float(v))))
    while len(out) < 18:
        out.append(0.0)
    return out


# ---------------------------------------------------------------------------
# validation

def validate_script(script) -> tuple[list[str], dict]:
    """Return (errors, stats).  Empty errors == acceptable script."""
    errs: list[str] = []
    stats = {"seconds": 0.0, "acts": 0, "frames": 0,
             "peak_rate_deg_s": 0.0, "stands": False}
    if not isinstance(script, dict):
        return (["script must be a JSON object"], stats)
    if script.get("format") != FORMAT:
        errs.append(f"format must be {FORMAT!r}")
    name = script.get("name", "")
    if not isinstance(name, str) or not NAME_RE.match(name):
        errs.append("name must match [a-z0-9_]{1,32}")
    acts = script.get("acts")
    if not isinstance(acts, list) or not acts:
        return (errs + ["acts must be a non-empty list"], stats)
    stats["acts"] = len(acts)
    total_frames = 0
    for i, act in enumerate(acts):
        where = f"acts[{i}]"
        if not isinstance(act, dict):
            errs.append(f"{where}: not an object")
            continue
        kind = act.get("kind")
        if kind not in ALLOWED_KINDS:
            errs.append(f"{where}: unknown kind {kind!r}")
            continue
        if kind == "stream":
            hz = act.get("hz")
            frames = act.get("frames")
            if not isinstance(hz, (int, float)) or not 1.0 <= hz <= 100.0:
                errs.append(f"{where}: hz must be 1..100")
                continue
            if not isinstance(frames, list) or len(frames) < 2:
                errs.append(f"{where}: frames must have >= 2 rows")
                continue
            total_frames += len(frames)
            prev = None
            for k, row in enumerate(frames):
                if not isinstance(row, list) or len(row) != 18:
                    errs.append(f"{where}.frames[{k}]: needs 18 numbers")
                    break
                bad = [v for j, v in enumerate(row)
                       if not isinstance(v, (int, float))
                       or not (AXIS_LIMITS_DEG[j % 3][0] - 0.51 <= v
                               <= AXIS_LIMITS_DEG[j % 3][1] + 0.51)]
                if bad:
                    errs.append(f"{where}.frames[{k}]: outside joint "
                                f"limits: {bad[:3]}")
                    break
                if prev is not None:
                    r = max(abs(a - b) for a, b in zip(row, prev)) * hz
                    stats["peak_rate_deg_s"] = max(
                        stats["peak_rate_deg_s"], r)
                prev = row
            stats["seconds"] += (len(frames) - 1) / float(hz)
        elif kind == "ease":
            pose = act.get("pose")
            secs = act.get("seconds", 1.0)
            if not isinstance(pose, list) or len(pose) != 18:
                errs.append(f"{where}: pose needs 18 numbers")
            if not isinstance(secs, (int, float)) or not 0.2 <= secs <= 20:
                errs.append(f"{where}: seconds must be 0.2..20")
            else:
                stats["seconds"] += float(secs)
        elif kind == "standup":
            stats["stands"] = True
            mode = act.get("mode", "")
            if not isinstance(mode, str) or not NAME_RE.match(mode):
                errs.append(f"{where}: bad standup mode")
            stats["seconds"] += 10.0     # nominal
        elif kind == "torque":
            lim = act.get("limit")
            if not isinstance(lim, (int, float)) or not 150 <= lim <= 1000:
                errs.append(f"{where}: torque limit must be 150..1000")
        elif kind == "sit_zero":
            stats["seconds"] += float(act.get("seconds", 1.5) or 1.5)
        elif kind == "note":
            if not isinstance(act.get("msg", ""), str):
                errs.append(f"{where}: msg must be a string")
    stats["frames"] = total_frames
    if total_frames > MAX_FRAMES:
        errs.append(f"too many frames ({total_frames} > {MAX_FRAMES})")
    if stats["seconds"] > MAX_SECONDS:
        errs.append(f"too long ({stats['seconds']:.0f}s > {MAX_SECONDS:.0f}s)")
    if bool(script.get("stands")) != stats["stands"]:
        errs.append("'stands' flag must match presence of standup acts")
    return (errs, stats)


# ---------------------------------------------------------------------------
# replay

def _frames_fn(frames: list[list[float]], hz: float):
    n = len(frames)

    def fn(t: float) -> list[float]:
        x = max(0.0, t * hz)
        i = int(x)
        if i >= n - 1:
            return _clamp_pose(frames[-1])
        u = x - i
        a, b = frames[i], frames[i + 1]
        return _clamp_pose([p + (q - p) * u for p, q in zip(a, b)])

    return fn


def run_dance_script(bus, script: dict, *, abort_check=None, status_cb=None,
                     speed: float = 1.0, speed_fn=None, standup_fn=None,
                     log_path=None) -> str:
    """Replay a validated dance script with full guard machinery."""
    import inplace_demos as ID

    errs, stats = validate_script(script)
    if errs:
        return "error: bad script: " + "; ".join(errs[:3])
    if stats["stands"] and standup_fn is None:
        return ("error: script stands up — run it from the web bench "
                "(stand-up plumbing required)")

    live = ID._live_robot_ids(bus)
    if len(live) < 3:
        print(f"  Only {len(live)} robot servo(s) — need more.")
        return "skipped"
    check = abort_check or (lambda: False)
    base_spd = speed_fn or (lambda: ID._clamp_demo_speed(speed))

    def note(msg: str) -> None:
        print(f"  {msg}")
        if status_cb is not None:
            try:
                status_cb(str(msg))
            except Exception:
                pass

    ID._enable_torque(bus, live)
    ID._set_torque_limit(bus, live, ID.DEMO_TORQUE_LIMIT)
    peaks = ID.CurrentPeakTracker()

    def bail(label: str) -> str:
        ID._hold_here(bus, live)
        ID._set_torque_limit(bus, live, 1000)
        return f"error: {label}"

    log_cm = (ID.MotionLog(log_path, live)
              if log_path is not None else None)
    if log_cm is not None:
        log_cm.__enter__()
    ended_limp = False
    try:
        for act in script["acts"]:
            if check():
                ID._set_torque_limit(bus, live, 1000)
                return "aborted"
            kind = act["kind"]
            if kind == "note":
                note(act.get("msg", ""))
            elif kind == "torque":
                ID._set_torque_limit(
                    bus, live, max(150, min(1000, int(act["limit"]))))
            elif kind == "stream":
                label = str(act.get("label") or "stream")
                frames = act["frames"]
                hz = float(act["hz"])
                secs = (len(frames) - 1) / hz
                spd = base_spd
                cap = act.get("speed_cap")
                if cap:
                    spd = (lambda c=float(cap), f=base_spd:
                           min(c, ID._clamp_live_speed(f())))
                trk = ID.CurrentPeakTracker()
                st = ID.stream_pose_fn(
                    bus, live, _frames_fn(frames, hz), seconds=secs,
                    abort_check=check, speed_fn=spd, status_cb=status_cb,
                    label=label, tracker=trk, log=log_cm,
                    tick_s=min(0.25, max(0.02, 1.0 / hz)),
                    max_speed=min(3000, int(act.get("max_speed", 3000))),
                    max_acc=min(254, int(act.get("max_acc", 200))),
                    tilt_guard_deg=(float(act["tilt_guard_deg"])
                                    if act.get("tilt_guard_deg") else None))
                if trk.peak_a > peaks.peak_a:
                    peaks.peak_a = trk.peak_a
                    peaks.peak_joint = trk.peak_joint
                peaks.samples += trk.samples
                if st == "aborted":
                    ID._set_torque_limit(bus, live, 1000)
                    return "aborted"
                if st == "guard":
                    return bail(f"current guard ({label})")
                if st.startswith("tilt:"):
                    ID._set_torque_limit(bus, live, 1000)
                    return (f"error: tilt {st[5:]} deg during {label} — "
                            f"went limp")
            elif kind == "ease":
                if not ID.ease_to_pose(
                        bus, _clamp_pose(act["pose"]), abort_check=check,
                        seconds=float(act.get("seconds", 1.0)),
                        label=str(act.get("label") or "ease"),
                        current_tracker=peaks):
                    return bail(str(act.get("label") or "ease"))
            elif kind == "standup":
                try:
                    ok, err = standup_fn(mode=str(act.get("mode", "step")))
                except TypeError:      # older bench fn without mode kwarg
                    ok, err = standup_fn()
                if check():
                    return bail("stand-up aborted")
                if not ok:
                    note(f"stand-up stopped: {err}")
                    return bail(f"stand-up: {err}")
            elif kind == "sit_zero":
                if not ID.go_to_zero_pose(
                        bus, abort_check=check,
                        seconds=float(act.get("seconds", 1.5) or 1.5)):
                    ID._set_torque_limit(bus, live, 1000)
                    return "aborted"
            elif kind == "limp":
                ID._limp_all(bus, live)
                ended_limp = True
    finally:
        if log_cm is not None:
            log_cm.__exit__(None, None, None)

    if not ended_limp:
        if not ID.go_to_zero_pose(bus, abort_check=check, seconds=1.5):
            ID._set_torque_limit(bus, live, 1000)
            return "aborted"
        ID._limp_all(bus, live)
    ID._set_torque_limit(bus, live, 1000)
    return "done"


# ---------------------------------------------------------------------------
# baking

def bake_demo(name: str, *, title: str | None = None,
              out_name: str | None = None) -> dict:
    """Record an existing demo runner into a dance script.

    Replays the runner against stubs: stream segments are sampled at
    their tick into frame tables; eases, torque changes, notes and
    stand-ups are captured in order.
    """
    import inplace_demos as ID

    if name not in ID.DEMOS:
        raise SystemExit(f"unknown demo {name!r}")
    if (any(name.startswith(p) for p in UNBAKEABLE_PREFIXES)
            or name in UNBAKEABLE_NAMES):
        raise SystemExit(
            f"{name!r} depends on live feedback (contact / velocity / "
            f"walking / raw bus ops) — not bakeable into a frame script")
    # stand_* stream shows assume the bench already stood the robot up
    # and leave it holding the plant; a self-contained script wraps them
    # in the STEP stand-up and a gentle knee-fold descend.
    stand_stream = name in getattr(ID, "STAND_STREAM_DEMOS", ())

    acts: list[dict] = []
    stands = [False]

    def note_cb(msg: str) -> None:
        acts.append({"kind": "note", "msg": str(msg)})

    def rec_stream(bus, live, pose_fn, *, seconds, label="", tick_s=None,
                   max_speed=3000, max_acc=200, tilt_guard_deg=None,
                   speed_fn=None, **kw) -> str:
        tick = float(tick_s or ID.STREAM_TICK_S)
        hz = 1.0 / tick
        n = int(round(seconds * hz))
        frames = [[round(v, 2) for v in _clamp_pose(
            pose_fn(min(i * tick, seconds)))] for i in range(n + 1)]
        act = {"kind": "stream", "label": str(label or "stream"),
               "hz": round(hz, 3), "max_speed": int(max_speed),
               "max_acc": int(max_acc), "frames": frames}
        if tilt_guard_deg:
            act["tilt_guard_deg"] = float(tilt_guard_deg)
            act["speed_cap"] = 1.5      # quad prudence travels with it
        acts.append(act)
        return "done"

    def rec_frames(bus, live, frames, abort_check, *, label="", log=None,
                   dt=None, max_speed=None, max_acc=80, **kw) -> bool:
        tick = float(dt or ID.DT)
        rows = [[round(v, 2) for v in _clamp_pose(p)] for p in frames]
        if len(rows) >= 2:
            acts.append({"kind": "stream", "label": str(label or "frames"),
                         "hz": round(1.0 / tick, 3),
                         "max_speed": int(max_speed or 3000),
                         "max_acc": int(max_acc), "frames": rows})
        return True

    def rec_ease(bus, goal, *, seconds=1.0, label="", **kw) -> bool:
        acts.append({"kind": "ease", "label": str(label or "ease"),
                     "seconds": round(max(0.2, float(seconds)), 2),
                     "pose": [round(v, 2) for v in _clamp_pose(goal)]})
        return True

    def rec_glide(bus, goal, live, seconds, check, *, max_speed=None,
                  max_acc=None, **kw) -> bool:
        return rec_ease(bus, goal, seconds=seconds, label="glide")

    def rec_torque(bus, live, lim) -> None:
        lim = max(150, min(1000, int(lim)))
        if acts and acts[-1].get("kind") == "torque":
            acts[-1]["limit"] = lim
        else:
            acts.append({"kind": "torque", "limit": lim})

    def rec_zero(bus, *, abort_check=None, seconds=1.5, **kw) -> bool:
        acts.append({"kind": "sit_zero", "seconds": round(float(seconds), 2)})
        return True

    def rec_limp(bus, live) -> None:
        acts.append({"kind": "limp"})

    def rec_standup(**kw):
        stands[0] = True
        acts.append({"kind": "standup", "mode": "step"})
        return (True, "")

    class _DummyTracker:
        peak_a = 0.0
        peak_joint = None
        max_a: dict = {}
        samples = 0
        last_fb: list = []

        def sample(self, *a, **k):
            pass

        def print_report(self, **k):
            pass

    patched = {
        "stream_pose_fn": rec_stream,
        "_run_frames": rec_frames,
        "ease_to_pose": rec_ease,
        "_soft_glide": rec_glide,
        "_set_torque_limit": rec_torque,
        "go_to_zero_pose": rec_zero,
        "_limp_all": rec_limp,
        "_enable_torque": lambda *a, **k: None,
        "_hold_here": lambda *a, **k: None,
        "_live_robot_ids": lambda bus: set(range(2, 20)),
        "configure_stream_profile": lambda bus: False,
        "CurrentPeakTracker": _DummyTracker,
    }
    saved = {k: getattr(ID, k) for k in patched}
    for k, v in patched.items():
        setattr(ID, k, v)
    try:
        status = ID.run_demo(None, name, status_cb=note_cb,
                             standup_fn=rec_standup)
    finally:
        for k, v in saved.items():
            setattr(ID, k, v)
    if status != "done":
        raise SystemExit(f"bake replay ended {status!r} — not saving")
    if stand_stream:
        acts = ([{"kind": "standup", "mode": "step"}] + acts
                + [{"kind": "sit_zero", "seconds": 4.5}, {"kind": "limp"}])
        stands[0] = True

    script = {
        "format": FORMAT,
        "name": out_name or f"{name}_baked",
        "title": title or f"[uploaded] {ID.DEMOS[name][0]}",
        "stands": stands[0],
        "baked_from": name,
        "acts": acts,
    }
    errs, stats = validate_script(script)
    if errs:
        raise SystemExit("baked script failed validation: "
                         + "; ".join(errs[:5]))
    script["seconds"] = round(stats["seconds"], 1)
    return script


# ---------------------------------------------------------------------------
# CLI

def _cli() -> None:
    import argparse
    import urllib.request

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("bake", help="bake a built-in demo into a script")
    b.add_argument("demo")
    b.add_argument("-o", "--out", default=None)
    b.add_argument("--name", default=None, help="script name on the robot")
    ba = sub.add_parser("bake-all",
                        help="bake every bakeable demo into a directory")
    ba.add_argument("-d", "--dir", default="dances")
    p = sub.add_parser("push", help="upload a script to a robot")
    p.add_argument("file")
    p.add_argument("--host", default="http://hexapod.local:8080")
    pa = sub.add_parser("push-all",
                        help="upload every *.json in a directory")
    pa.add_argument("-d", "--dir", default="dances")
    pa.add_argument("--host", default="http://hexapod.local:8080")
    g = sub.add_parser("pull", help="download a script from a robot")
    g.add_argument("name")
    g.add_argument("--host", default="http://hexapod.local:8080")
    g.add_argument("-o", "--out", default=None)
    args = ap.parse_args()

    def _push_one(path: Path, host: str) -> None:
        body = path.read_bytes()
        if len(body) > MAX_SCRIPT_BYTES:
            raise SystemExit(f"{path} too big")
        req = urllib.request.Request(
            host.rstrip("/") + "/api/dances", data=body,
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as r:
            print(f"{path.name}: {r.read().decode()}")

    if args.cmd == "bake":
        script = bake_demo(args.demo, out_name=args.name)
        out = Path(args.out or f"{script['name']}.json")
        out.write_text(json.dumps(script))
        errs, stats = validate_script(script)
        print(f"baked {args.demo} -> {out}  ({out.stat().st_size/1024:.0f} KB, "
              f"{stats['acts']} acts, {stats['frames']} frames, "
              f"~{stats['seconds']:.0f}s, stands={script['stands']})")
    elif args.cmd == "bake-all":
        import inplace_demos as ID
        outdir = Path(args.dir)
        outdir.mkdir(parents=True, exist_ok=True)
        n_ok = 0
        for demo in ID.DEMOS:
            if demo == "breathe+":
                continue
            if (any(demo.startswith(p) for p in UNBAKEABLE_PREFIXES)
                    or demo in UNBAKEABLE_NAMES):
                continue
            script = bake_demo(demo)
            out = outdir / f"{script['name']}.json"
            out.write_text(json.dumps(script))
            _, stats = validate_script(script)
            print(f"  {demo:20s} -> {out.name:28s} "
                  f"{out.stat().st_size/1024:5.0f} KB  "
                  f"~{stats['seconds']:4.0f}s  stands={script['stands']}")
            n_ok += 1
        print(f"baked {n_ok} demos into {outdir}/")
    elif args.cmd == "push-all":
        files = sorted(Path(args.dir).glob("*.json"))
        if not files:
            raise SystemExit(f"no *.json in {args.dir}")
        for f in files:
            _push_one(f, args.host)
    elif args.cmd == "push":
        _push_one(Path(args.file), args.host)
    elif args.cmd == "pull":
        url = args.host.rstrip("/") + "/api/dances/" + args.name
        with urllib.request.urlopen(url, timeout=30) as r:
            data = r.read()
        out = Path(args.out or f"{args.name}.json")
        out.write_bytes(data)
        print(f"pulled {args.name} -> {out} ({len(data)/1024:.0f} KB)")


if __name__ == "__main__":
    _here = Path(__file__).resolve().parent
    sys.path.insert(0, str(_here))
    # tripod_gait & friends live in linux_control (quad_walk imports them)
    sys.path.insert(0, str(_here.parent / "linux_control"))
    _cli()
