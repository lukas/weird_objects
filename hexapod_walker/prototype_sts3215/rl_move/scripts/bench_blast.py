"""Bench blast — the whole next hardware session as one guided checklist.

Companion to ``rl_move/sim/sim_blast.py`` (the sim half). Runs the
open bench experiments from RL_PLAN queue -1 / HARDWARE.md in
decision-value order, over HTTP only (never SSH), one confirmed step
at a time. THE OPERATOR IS THE SAFETY SYSTEM: every motion step waits
for an explicit ``go`` while you watch the robot, and anything else
skips it. Ctrl-C anywhere aborts the current step via /api/rl/stop.

Per hardware-safety rules this script NEVER arms, zeros, or limps the
robot — do ``set_zero → ARM → P`` from the web UI first, exactly like
every working session. Motion is only sent with ``--go``; the default
is a dry run that prints the full plan and sends nothing.

Session plan (steps run in order; --only / --skip to cherry-pick):

  info       Read-only preflight: policy slots + md5-active flags,
             stand-weights goal-profile check (refuses to offer STAND
             on a profile-less copy — the stale-copy trap), tilt trip,
             live preflight for walk/stand.
  ab         THE headline experiment: vref1-r1 vs tip1 same-floor A/B,
             --rounds x (walk 6 s @ 0.05 m/s each, alternating).
             VERIFIES the policy switch actually landed before every
             walk (the 08-10 A/B silently never switched), downloads
             each episode CSV over /api/logs, and computes the
             roll-ramp rate per run. Output: runaways per N + ramp
             slope per policy — the discriminating data the runaway
             story needs.
  tape       Tape-measure one RL walk (prompts for the reading,
             records via /api/measure/note kind=rl_walk_tape).
  rot60      First off-wedge heading on hardware: one 6 s BACKWARD
             walk. Requires the post-08-11 deploy re-push; if the
             runner lacks the rot60 port it refuses the command,
             which is itself the result.
  stand      First learned stand-up on hardware (riskiest last):
             gated on the goal-profile check from `info`, preflight,
             then /api/rl/stand with your hands ready; offers
             /api/rl/lower afterwards.
  turnsign   wz sign audit: scripted turn-in-place +0.3 then -0.3
             rad/s, you report observed direction (cw/ccw) —
             closes the TURN sign question.
  hold       Holding-current cards: planted 30 s, then hover 30 s
             (you unload one leg between them). No commanded motion.

Everything lands in rl_move/hardware_traces/bench_blast_<stamp>/
(episode CSVs + session summary.json) and in the robot's own
measurements.jsonl.

Usage (bench, robot ARMed and standing at plant, web UI handy):

    python -m rl_move.scripts.bench_blast              # dry run (plan)
    python -m rl_move.scripts.bench_blast --go
    python -m rl_move.scripts.bench_blast --go --video  # film it instead
    python -m rl_move.scripts.bench_blast --go --only ab tape
    python -m rl_move.scripts.bench_blast --go --rounds 4

VIDEO MODE (--video): the zero-typing session. Lay the tape measure on
the floor along the walk direction, start filming (whole runway in
frame, Mac speaker audible), run with --go --video, and only type the
per-step ``go``s. Every step is SPOKEN onto the video's audio track
and timestamped in summary.json; the tape reading and the turn
direction are read off the footage afterwards by
``rl_move/scripts/video_review.py`` + the analysis agent — you never
report a number.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[2]))  # prototype_sts3215/

from rl_move.remote import HexapodClient  # noqa: E402

TRACES_DIR = _HERE.parents[1] / "hardware_traces"
WALK_SPEED = 0.05          # m/s — inside the trained 0.05-0.06 band
WALK_SECONDS = 6.0
RUNAWAY_END_DEG = 10.0     # |roll| at episode end that counts as runaway
FALL_TAIL_DEG = 10.0       # |roll| still there AFTER the episode = down
MAX_RECOVERIES = 2         # unattended falls tolerated before aborting
DEFAULT_STEPS = ("info", "ab", "tape", "rot60", "stand", "turnsign", "hold")
# rise/sit: single transitions for composed --only sessions (e.g. a
# belly-start unattended run: info rise ab turnsign sit)
STEPS = DEFAULT_STEPS + ("rise", "sit")


def ask(prompt: str) -> str:
    try:
        return input(f"{prompt} ").strip().lower()
    except EOFError:
        return "quit"


class SessionAbort(Exception):
    """Motion abort: robot judged down/unsafe with nobody at the bench."""


class CameraRecorder:
    """Record the Mac's own camera for the whole session (--camera).

    Kills the last manual step of video mode: no phone, no spoken sync
    mark. Frames are written on a wall-clock schedule at a fixed fps, so
    video time == (t_unix - t0_unix) exactly; video_review.py reads the
    ``camera`` block from summary.json and syncs with zero guesswork.
    (cv2 capture has no audio track — sync never depends on `say`.)
    """

    def __init__(self, out_path: Path, index: int = 0, fps: float = 15.0):
        self.path = out_path
        self.index = index
        self.fps = fps
        self.t0: float | None = None
        self.frames = 0
        self._stop = None
        self._thread = None
        self._cap = None
        self._writer = None

    def start(self) -> bool:
        import threading
        import cv2
        self._cap = cv2.VideoCapture(self.index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        ok, frame = self._cap.read()
        if not ok:
            print("    !! camera: no frame (permission? index?) — "
                  "recording DISABLED, session continues")
            self._cap.release()
            self._cap = None
            return False
        # auto-exposure warmup: the first ~2 s of frames are near-black
        warm_until = time.time() + 2.0
        while time.time() < warm_until:
            ok, f = self._cap.read()
            if ok:
                frame = f
        h, w = frame.shape[:2]
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._writer = cv2.VideoWriter(
            str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
        self.t0 = time.time()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(frame,),
                                        daemon=True)
        self._thread.start()
        print(f"    camera recording {w}x{h} @ {self.fps:.0f}fps "
              f"-> {self.path.name}")
        return True

    def _run(self, first_frame) -> None:
        latest = first_frame
        next_t = self.t0
        while not self._stop.is_set():
            ok, f = self._cap.read()      # blocks at the camera's rate
            if ok:
                latest = f
            now = time.time()
            # constant-rate output: repeat the newest frame as needed
            while next_t <= now:
                self._writer.write(latest)
                self.frames += 1
                next_t += 1.0 / self.fps

    def stop(self) -> dict | None:
        if self._thread is None:
            return None
        self._stop.set()
        self._thread.join(timeout=5.0)
        self._cap.release()
        self._writer.release()
        dur = self.frames / self.fps
        print(f"    camera: {self.frames} frames ({dur:.0f}s) "
              f"-> {self.path.name}")
        return {"video": self.path.name, "t0_unix": round(self.t0, 3),
                "fps": self.fps, "frames": self.frames}


class Session:
    def __init__(self, client: HexapodClient, go: bool, rounds: int,
                 video: bool = False, auto: bool = False,
                 camera: int | None = None):
        self.c = client
        self.go = go
        self.rounds = rounds
        self.video = video or camera is not None
        self.auto = auto
        self.camera = camera
        self.recorder: CameraRecorder | None = None
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.out = TRACES_DIR / f"bench_blast_{self.stamp}"
        self.summary: dict = {"stamp": self.stamp, "video_mode": self.video,
                              "walks": [], "notes": [], "events": []}
        self.stand_profile_ok = False
        self.policy_files: dict[str, str] = {}   # "vref1"/"tip1" -> file
        self.recoveries = 0
        self._dir = 1                             # A/B walk direction flip

    # -- plumbing ---------------------------------------------------------

    def req(self, method: str, path: str, body: dict | None = None) -> dict:
        r = self.c._req(method, path, body)
        if not r.get("ok", True):
            print(f"    !! {path}: {r.get('error')}")
        return r

    def announce(self, text: str) -> None:
        """Timestamped event; in --video mode also SPOKEN aloud (macOS
        `say`) so the label lands on the video's audio track and
        video_review.py / the analysis agent can line clips up with
        the log without the operator writing anything down."""
        self.summary["events"].append(
            {"t_unix": round(time.time(), 2), "text": text})
        print(f"    [{time.strftime('%H:%M:%S')}] {text}")
        if self.video and self.camera is None:
            # --camera syncs by unix time (cv2 has no audio track), so
            # speech would only slow the session down; phone-video mode
            # still needs the label on the audio track.
            import subprocess
            try:
                # Blocking on purpose: motion must not start before the
                # label is fully on the audio track.
                subprocess.run(["say", "-r", "200", text], timeout=15)
            except Exception:
                pass  # no `say` (not macOS) — timestamps still work

    def confirm(self, what: str, countdown: int = 5) -> bool:
        """Motion gate: explicit per-step operator go — or, in --auto,
        a SPOKEN countdown with Ctrl-C as the abort (the operator is
        watching the robot, not the keyboard)."""
        if not self.go:
            print(f"    [dry-run] would: {what}")
            return False
        if self.auto:
            self.announce(f"next: {what}. starting in {countdown} "
                          "seconds. control C to abort.")
            for i in range(countdown, 0, -1):
                print(f"    ... {i}", flush=True)
                time.sleep(1.0)
            return True
        a = ask(f"    >> {what} — type 'go' (anything else skips):")
        if a == "quit":
            raise KeyboardInterrupt
        return a == "go"

    def newest_walk_csv(self, after_unix: float = 0.0,
                        wait_s: float = 12.0) -> str | None:
        """Newest rl_walk CSV WRITTEN AFTER after_unix, and only once its
        size has stopped growing. Two races bit on 08-11: the tape walk
        pulled the PREVIOUS csv (no after_unix), and vref1-r1 pulled a
        csv mid-flush and got 'too few rows'."""
        deadline = time.time() + wait_s
        last: tuple[str, int] | None = None
        while True:
            logs = self.req("GET", "/api/logs")
            found: tuple[str, int] | None = None
            for f in logs.get("files", []):
                if (f.get("name", "").startswith("rl_walk_")
                        and f.get("name", "").endswith(".csv")
                        and f.get("mtime_unix", 0) > after_unix):
                    found = (f["name"], int(f.get("bytes", 0)))
                    break
            if found is not None and found == last:
                return found[0]        # same name+size twice = flushed
            last = found
            if time.time() >= deadline:
                return found[0] if found else None
            time.sleep(1.5)

    def pull_csv(self, name: str) -> Path | None:
        import urllib.request
        url = f"{self.c.base}/api/logs/{name}"
        dst = self.out / name
        try:
            self.out.mkdir(parents=True, exist_ok=True)
            with urllib.request.urlopen(url, timeout=30) as resp:
                dst.write_bytes(resp.read())
            return dst
        except Exception as e:
            print(f"    !! trace pull failed ({name}): {e}")
            return None

    # -- analysis ---------------------------------------------------------

    @staticmethod
    def roll_stats(path: Path) -> dict:
        """Roll-ramp fingerprint from an episode CSV (t_s, phase,
        roll_deg columns; tail rows keep logging after the walk)."""
        t, roll, tail_roll = [], [], []
        with io.open(path, newline="") as f:
            for row in csv.DictReader(f):
                try:
                    tt = float(row["t_s"])
                    rr = float(row["roll_deg"])
                except (KeyError, ValueError):
                    continue
                if str(row.get("phase", "")).startswith("tail"):
                    tail_roll.append(rr)
                else:
                    t.append(tt)
                    roll.append(rr)
        if len(roll) < 10:
            return {"error": f"too few rows in {path.name}"}
        n4 = max(1, len(roll) // 4)
        q1 = sum(roll[:n4]) / n4
        q4 = sum(roll[-n4:]) / n4
        dur = t[-1] - t[0] or 1.0
        end = sum(roll[-n4:]) / n4
        peak = max(abs(r) for r in roll)
        tail_peak = max((abs(r) for r in tail_roll), default=0.0)
        ramp = (q4 - q1) / (0.75 * dur)     # deg/s, Q1->Q4 mean drift
        runaway = abs(end) >= RUNAWAY_END_DEG or abs(ramp) >= 1.5
        return {"csv": path.name, "roll_q1_deg": round(q1, 1),
                "roll_q4_deg": round(q4, 1), "roll_peak_deg": round(peak, 1),
                "tail_peak_deg": round(tail_peak, 1),
                "ramp_deg_per_s": round(ramp, 2), "runaway": runaway}

    # -- steps ------------------------------------------------------------

    def step_info(self) -> None:
        ping = self.req("GET", "/api/ping")
        if not ping.get("ok", True):
            if self.go:
                raise SystemExit("robot unreachable — check "
                                 "hexapod.local:8080")
            print("    (robot offline — dry run continues with the plan)")
            return
        pols = self.req("GET", "/api/rl/policies")
        print("    policies on robot:")
        for p in pols.get("policies", []):
            mark = " <ACTIVE>" if p.get("active") else ""
            print(f"      {p.get('slot', '?'):6s} {p.get('file'):40s}"
                  f" src={p.get('source', '')}{mark}")
            src = (p.get("source") or "") + (p.get("file") or "")
            if p.get("slot") == "walk":
                if "vref1" in src:
                    self.policy_files["vref1"] = p["file"]
                if "tip1" in src:
                    self.policy_files["tip1"] = p["file"]
        info = self.req("GET", "/api/rl/policy")
        # Stale-copy trap (HARDWARE.md 08-11): a stance export without
        # its goal profile gets the legacy +50mm/4s ramp — never STAND.
        prof_keys = [k for k in info if "profile" in k.lower()
                     or "ramp" in k.lower() or "goal" in k.lower()]
        self.stand_profile_ok = bool(prof_keys)
        print(f"    stance weights meta keys: {sorted(info.keys())}")
        print(f"    goal-profile present: {self.stand_profile_ok}"
              + ("" if self.stand_profile_ok else
                 "  -> STAND step will be REFUSED (re-push via "
                 "deploy_adb.sh first)"))
        for mode in ("walk", "stand"):
            pf = self.req("GET", f"/api/rl/preflight?mode={mode}")
            print(f"    preflight {mode}: ok={pf.get('ok')} "
                  f"{pf.get('reason', '')}")
        self.summary["policies"] = pols.get("policies")
        self.summary["stand_profile_ok"] = self.stand_profile_ok

    def _one_walk(self, tag: str, vx: float) -> None:
        self.announce(f"walk {tag} starting")
        t0 = time.time()
        r = self.req("POST", "/api/rl/walk",
                     {"vx": vx, "vy": 0.0, "duration_s": WALK_SECONDS})
        if not r.get("ok", True):
            self.announce(f"walk {tag} refused")
            self.summary["walks"].append({"tag": tag, "error": r.get("error")})
            return
        # walk + post-episode tail + margin
        time.sleep(WALK_SECONDS + 6.0)
        end_state = self.c.wait_idle(timeout_s=30.0)
        self.announce(f"walk {tag} done")
        stats: dict = {"tag": tag, "vx": vx,
                       "t_start_unix": round(t0, 2),
                       "t_end_unix": round(time.time(), 2)}
        # 08-11 lesson: the kickoff response only proves the runner
        # STARTED. tip1's tilt_roll safety trips were invisible until the
        # robot's own event log was read. Record the terminal result.
        res = self._runner_result(end_state)
        if res is not None:
            stats["result"] = res
        name = self.newest_walk_csv(after_unix=t0)
        if name:
            p = self.pull_csv(name)
            if p:
                stats.update(self.roll_stats(p))
        self.summary["walks"].append(stats)
        print(f"    {tag}: ramp={stats.get('ramp_deg_per_s')} deg/s "
              f"peak={stats.get('roll_peak_deg')} "
              f"tail={stats.get('tail_peak_deg')} "
              f"runaway={stats.get('runaway')}")
        self._maybe_recover(stats)

    @staticmethod
    def _runner_result(state: dict) -> dict | None:
        """Terminal result of the calibrate worker from a wait_idle()
        return (None when the state carries no result, e.g. timeout)."""
        res = state.get("result")
        if res is None:
            res = (state.get("calibrate") or {}).get("result")
        return res

    def _transition(self, mode: str) -> bool:
        """Learned stand ('stand') or lower ('lower') with the TERMINAL
        result recorded — returns True only on a clean finish."""
        self.announce(f"learned "
                      f"{'stand up' if mode == 'stand' else mode} starting")
        kick = self.req("POST", f"/api/rl/{mode}", {})
        entry: dict = {"kick": kick}
        if kick.get("ok", True):
            end_state = self.c.wait_idle(timeout_s=90.0)
            entry["result"] = self._runner_result(end_state)
        res = entry.get("result")
        ok = bool(isinstance(res, dict) and res.get("ok"))
        entry["ok"] = ok
        self.summary.setdefault("transitions", []).append(
            {"mode": mode, "t_unix": round(time.time(), 2), **entry})
        self.summary[mode] = entry
        self.announce(f"{mode} {'done' if ok else 'FAILED'}")
        if not ok:
            print(f"    !! {mode} did not finish clean: "
                  f"{json.dumps(res)[:200] if res else 'no result'}")
        return ok

    def _maybe_recover(self, stats: dict) -> None:
        """08-11 --camera lesson: with nobody at the bench, one fall
        poisons every later step (fallen walks, grinding turns, board
        brownout). In --auto, a walk that ends tilted triggers ONE
        recovery: safe_zero to belly, learned stand back up. Past
        MAX_RECOVERIES (or if recovery fails) all motion aborts."""
        if not self.auto:
            return                     # operator present — their call
        res = stats.get("result")
        tripped = isinstance(res, dict) and not res.get("ok", True)
        fell = tripped or stats.get("tail_peak_deg", 0.0) >= FALL_TAIL_DEG
        if not fell:
            return
        self.recoveries += 1
        if self.recoveries > MAX_RECOVERIES:
            self.announce("fall limit reached. aborting all motion.")
            raise SessionAbort("too many falls")
        self.announce("robot looks down. recovering: safe zero, "
                      "then stand back up.")
        r = self.req("POST", "/api/safe_zero", {})
        self.c.wait_idle(timeout_s=60.0)
        if not r.get("ok", True):
            raise SessionAbort("safe_zero refused")
        if not self._transition("stand"):
            raise SessionAbort("recovery stand-up failed")
        self.summary.setdefault("recoveries", []).append(
            {"after": stats.get("tag"), "t_unix": round(time.time(), 2)})

    def _select_and_verify(self, key: str) -> bool:
        file = self.policy_files.get(key)
        if not file:
            print(f"    !! no {key} file found in the robot's policies/")
            return False
        r = self.req("POST", "/api/rl/policy_select", {"file": file})
        if not r.get("ok"):
            return False
        # THE 08-10 lesson: verify the switch actually landed.
        pols = self.req("GET", "/api/rl/policies")
        for p in pols.get("policies", []):
            if p.get("file") == file and p.get("active"):
                print(f"    verified active: {file}")
                return True
        print(f"    !! select of {file} did NOT land (active flag "
              f"unset) — NOT walking")
        return False

    def step_ab(self) -> None:
        print(f"    plan: {self.rounds} rounds x (vref1 walk, tip1 walk), "
              f"{WALK_SECONDS:.0f}s @ {WALK_SPEED} m/s each, ALTERNATING "
              "direction (keeps the robot in camera frame instead of "
              "marching off one way), policy switch VERIFIED before "
              "every walk.")
        for i in range(self.rounds):
            for key in ("vref1", "tip1"):
                vx = self._dir * WALK_SPEED
                word = "fwd" if vx > 0 else "BACK"
                if not self.confirm(f"round {i + 1}: switch to {key} and "
                                    f"walk {word} {WALK_SECONDS:.0f}s"):
                    continue
                if not self._select_and_verify(key):
                    continue
                self._dir *= -1
                self._one_walk(f"{key}-r{i + 1}", vx)
            # odd rounds start BACK so each policy sees both directions
            self._dir *= -1
        tally: dict[str, list] = {"vref1": [], "tip1": []}
        for w in self.summary["walks"]:
            k = w.get("tag", "").split("-r")[0]
            if k in tally and "runaway" in w:
                tally[k].append(w["runaway"])
        for k, v in tally.items():
            if v:
                print(f"    {k}: {sum(v)}/{len(v)} runaway")
        self.summary["ab_tally"] = {
            k: {"runs": len(v), "runaways": sum(v)}
            for k, v in tally.items() if v}

    def step_tape(self) -> None:
        if not self.confirm("one more RL walk for the tape (mark the "
                            "start point first)"):
            return
        key = "tip1" if "tip1" in self.policy_files else "vref1"
        if not self._select_and_verify(key):
            return
        self._one_walk(f"tape-{key}", WALK_SPEED)
        commanded_mm = WALK_SPEED * WALK_SECONDS * 1000.0
        if self.video:
            # No typing at the bench: film the tape instead. The
            # analysis pass reads the number off the frames and files
            # the measure/note record afterwards.
            self.announce("tape walk done. point the camera straight "
                          "down the tape for five seconds")
            time.sleep(6.0)
            self.summary["tape"] = {"policy": key,
                                    "commanded_mm": commanded_mm,
                                    "measured_mm": None,
                                    "from_video": True}
            return
        val = ask(f"    tape reading in mm (commanded {commanded_mm:.0f}):")
        try:
            measured = float(val)
        except ValueError:
            print("    skipped (no number)")
            return
        self.req("POST", "/api/measure/note",
                 {"kind": "rl_walk_tape",
                  "fields": {"measured_mm": measured,
                             "commanded_mm": commanded_mm,
                             "notes": f"bench_blast {self.stamp} {key}"}})
        self.summary["tape"] = {"policy": key, "commanded_mm": commanded_mm,
                                "measured_mm": measured,
                                "ratio": round(measured / commanded_mm, 3)}
        print(f"    slip ratio {measured / commanded_mm:.2f} recorded")

    def step_rot60(self) -> None:
        print("    first off-wedge heading on hardware: 6 s BACKWARD "
              "walk. Needs the 08-11 re-push (rot60 runner port); an "
              "un-pushed runner refuses the command — also a result.")
        if not self.confirm("backward walk 6s @ 0.05 m/s (rot60 wedge "
                            "canonicalization)"):
            return
        self._one_walk("rot60-back", -WALK_SPEED)

    def step_stand(self) -> None:
        if not self.stand_profile_ok:
            print("    REFUSED: stance weights carry no goal profile "
                  "(stale copy) — run deploy_adb.sh, re-select, rerun "
                  "the info step.")
            return
        # 08-11 lesson: commanding STAND while already standing made the
        # runner drag the loaded robot toward belly zero (L0 hip stall ->
        # limp). Order the pair from the robot's ACTUAL pose: stand
        # preflight passes only belly-down.
        pf = self.req("GET", "/api/rl/preflight?mode=stand")
        belly = bool(pf.get("ok"))
        plan = ("stand", "lower") if belly else ("lower", "stand")
        print(f"    robot is {'belly-down' if belly else 'standing'} -> "
              f"order: {' then '.join(plan)}. Scripted tuck is the "
              "known-good fallback.")
        for mode in plan:
            if not self.confirm(f"RL {mode.upper()} (preflight-gated)",
                                countdown=8):
                continue
            if not self._transition(mode):
                # unknown pose after a failed transition — do not chain
                # the second blend on top of it (safety rule: no retries)
                break

    def step_rise(self) -> None:
        """Learned stand-up alone — the opener for a belly-start
        unattended session. A failed rise aborts all motion in --auto
        (nothing downstream makes sense off an unknown pose)."""
        if not self.stand_profile_ok:
            print("    REFUSED: stance weights carry no goal profile "
                  "(stale copy) — see the stand step notes.")
            return
        if not self.confirm("RL STAND UP (rise only)", countdown=8):
            return
        if not self._transition("stand") and self.auto:
            raise SessionAbort("opening stand-up failed")

    def step_sit(self) -> None:
        """Learned lower alone — the session closer (park on the belly)."""
        if self.confirm("RL LOWER (sit only)", countdown=8):
            self._transition("lower")

    def step_turnsign(self) -> None:
        for omega in (0.3, -0.3):
            if not self.confirm(f"scripted turn-in-place omega={omega:+} "
                                f"rad/s, 6s"):
                continue
            self.announce(f"turn sign omega {omega:+} starting")
            t0 = time.time()
            self.req("POST", "/api/measure/walk",
                     {"vx_mm": 0.0, "omega": omega, "duration_s": 6.0})
            time.sleep(8.0)
            self.announce(f"turn sign omega {omega:+} done")
            if self.video:
                # Rotation direction is read off the footage afterwards;
                # the pending measure record stays un-annotated on the
                # robot and gets its observed_turn from the video pass.
                self.summary.setdefault("turnsign", []).append(
                    {"omega": omega, "observed": "video",
                     "t_start_unix": round(t0, 2),
                     "t_end_unix": round(time.time(), 2)})
                continue
            seen = ask("    observed rotation (cw/ccw/none):")
            if seen in ("cw", "ccw", "none"):
                self.req("POST", "/api/measure/annotate",
                         {"fields": {"observed_turn": seen,
                                     "notes": f"omega {omega:+}"}})
            self.summary.setdefault("turnsign", []).append(
                {"omega": omega, "observed": seen})

    def step_hold(self) -> None:
        for label in ("planted", "hover"):
            if label == "hover":
                print("    now unload/raise ONE leg by hand "
                      "(torque stays on), then confirm.")
            if not self.confirm(f"log {label} holding currents 30s "
                                f"(no commanded motion)"):
                continue
            self.req("POST", "/api/measure/hold",
                     {"label": label, "duration_s": 30.0})
            time.sleep(32.0)

    # -- driver -----------------------------------------------------------

    def run(self, only: list[str], skip: list[str]) -> None:
        if only:
            # --only order is honored (e.g. turns BEFORE the risky
            # stand/lower pair when the robot starts the session up)
            chosen = [s for s in only if s not in skip]
        else:
            chosen = [s for s in DEFAULT_STEPS if s not in skip]
        print(f"session {self.stamp} -> {self.out}")
        print(f"steps: {' '.join(chosen)}"
              + ("" if self.go else "   [DRY RUN — add --go]"))
        if self.camera is not None and self.go:
            self.recorder = CameraRecorder(self.out / "camera.mp4",
                                           index=self.camera)
            if not self.recorder.start():
                self.recorder = None
        elif self.video and self.go:
            print("\nVIDEO MODE — camera setup (then just film, no "
                  "typing beyond the go's):\n"
                  "  * tape measure flat on the floor along the walk "
                  "direction, robot start at 0\n"
                  "  * frame the whole runway; keep the Mac's speaker "
                  "audible (steps are spoken onto the audio track)")
            if self.auto:
                # No Enter either: 20 spoken seconds to grab the phone
                # and hit record before the sync mark lands.
                self.announce("bench blast auto session. start "
                              "recording now. twenty seconds to the "
                              "sync mark.")
                time.sleep(20.0)
            else:
                print("  * START RECORDING NOW, then press Enter")
                ask("    recording?")
        if self.video and self.go:
            # Sync anchor: video_review.py maps this announcement's
            # t_unix to a video timestamp and everything else follows.
            # (--camera sessions sync exactly by t0_unix; the mark is
            # kept as a redundant cross-check.)
            self.announce(f"bench blast session {self.stamp} sync mark")
        try:
            for s in chosen:
                print(f"\n== {s} ==")
                getattr(self, f"step_{s}")()
        except KeyboardInterrupt:
            print("\nABORT: stopping any RL worker")
            self.req("POST", "/api/rl/stop", {})
        except SessionAbort as e:
            print(f"\nMOTION ABORTED: {e} — stopping any RL worker; "
                  "remaining steps skipped")
            self.summary["aborted"] = str(e)
            self.req("POST", "/api/rl/stop", {})
        finally:
            if self.recorder is not None:
                meta = self.recorder.stop()
                if meta:
                    self.summary["camera"] = meta
            self.out.mkdir(parents=True, exist_ok=True)
            f = self.out / "summary.json"
            f.write_text(json.dumps(self.summary, indent=1))
            print(f"\nwrote {f}")
            if self.summary.get("camera"):
                print("next: python -m rl_move.scripts.video_review "
                      f"--session {self.out}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=None,
                    help="robot base URL (default http://hexapod.local:8080)")
    ap.add_argument("--go", action="store_true",
                    help="actually send motion (default: dry-run plan)")
    ap.add_argument("--rounds", type=int, default=3,
                    help="A/B rounds (each = 1 vref1 walk + 1 tip1 walk)")
    ap.add_argument("--only", nargs="*", default=[], choices=STEPS)
    ap.add_argument("--skip", nargs="*", default=[], choices=STEPS)
    ap.add_argument("--video", action="store_true",
                    help="videotaped session: no tape/turn typing — "
                         "steps are spoken aloud (macOS say) onto the "
                         "video's audio track + timestamped in "
                         "summary.json; afterwards run "
                         "rl_move/scripts/video_review.py on the "
                         "footage (or hand it to the agent)")
    ap.add_argument("--auto", action="store_true",
                    help="no typed go's: each motion step runs after a "
                         "SPOKEN 5s countdown (8s for STAND); Ctrl-C "
                         "aborts and stops the robot. The operator "
                         "must be watching the robot the whole time.")
    ap.add_argument("--camera", type=int, default=None, metavar="N",
                    help="record the Mac's camera (device index N, 0 = "
                         "built-in) for the whole session — implies "
                         "--video minus the phone: exact unix-time sync, "
                         "footage lands in the session dir as camera.mp4")
    args = ap.parse_args()

    s = Session(HexapodClient(args.base), args.go, args.rounds,
                video=args.video, auto=args.auto, camera=args.camera)
    s.run(args.only, args.skip)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
