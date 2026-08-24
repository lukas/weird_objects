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
  zero       Laptop-driven safe zero (operator ruling 08-12: the slow
             tilt-gated safe_zero glide MAY be commanded from the
             laptop). Glides to belly zero, then verifies the stance
             TWO ways before anything riskier runs: all 18 encoders
             near 0 deg, and (with --camera) a still frame saved as
             zero_check.jpg for a visual look at the pose. Fails the
             session if the pose does not verify.
  turnsign   wz sign audit: scripted turn-in-place +0.3 then -0.3
             rad/s, you report observed direction (cw/ccw) —
             closes the TURN sign question.
  hold       Holding-current cards: planted 30 s, then hover 30 s
             (you unload one leg between them). No commanded motion.

Everything lands in rl_move/hardware_traces/bench_blast_<stamp>/
(episode CSVs + session summary.json) and in the robot's own
measurements.jsonl.

Usage (bench, robot ARMed and standing at plant, web UI handy):

    uv run python -m rl_move.scripts.bench_blast              # dry run (plan)
    uv run python -m rl_move.scripts.bench_blast --go
    uv run python -m rl_move.scripts.bench_blast --go --video  # film it instead
    uv run python -m rl_move.scripts.bench_blast --go --only ab tape
    uv run python -m rl_move.scripts.bench_blast --go --rounds 4

INTERACTION MODES — how much typing each combination needs:

  (default)      Per-step gate: each motion step waits for a typed
                 ``go``; anything else skips that one step. Safest;
                 you are at the keyboard AND watching the robot.
  --auto         NO keyboard at all: every gate becomes a spoken
                 3-second countdown (watch the robot, not the
                 terminal; Ctrl-C or the web STOP aborts). Preflight
                 failures ABORT the session instead of asking. A walk
                 that ends tilted gets one recovery, then aborts after
                 MAX_RECOVERIES.
  --one-go       One typed ``go`` at the very start (get hands
                 ready), then the rest runs --auto style.
  --video        Film-instead-of-type for the MEASUREMENTS: steps are
                 spoken onto the audio track, and the tape reading +
                 turn direction are read OFF THE FOOTAGE afterwards
                 (``video_review.py``) — without --video, ``tape``
                 and ``turnsign`` prompt for typed readings and MUST
                 have a human at the keyboard. With --auto (phone
                 filming) you get a spoken 20 s lead-in to hit record.
  --camera N     Record camera.mp4 from Mac/USB camera index N
                 automatically (no phone, no lead-in, exact t0 sync;
                 a recorder that fails to open is non-fatal). Also
                 saves zero_check.jpg during the ``zero`` step.

  Fully unattended (agent-driven / zero stdin) invocation:

      uv run python -m rl_move.scripts.bench_blast --go --auto --video \
          --camera 0 [--skip ...]

  Do NOT run without --video in a shell that has no stdin: the
  ``tape``/``turnsign`` prompts would hit EOF and abort the session.
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
MAX_RECOVERIES = 4         # unattended falls tolerated before aborting
                           # (raised 2->4 08-11 eve: fall-rate per policy IS
                           # the measurement; each recovery limps on trip so
                           # falls stay low-energy)
THERMAL_WARN_C = 55        # hold before motion if any servo is at/above
THERMAL_RESUME_C = 45      # ...and resume only once back under this
THERMAL_COOL_WAIT_S = 360  # max cooldown hold before aborting the session
DEFAULT_STEPS = ("info", "ab", "tape", "rot60", "stand", "turnsign", "hold")
# rise/sit/zero: single transitions for composed --only sessions (e.g.
# a belly-start unattended run: info zero rise ab turnsign sit)
STEPS = DEFAULT_STEPS + ("rise", "sit", "zero")
ZERO_MAX_JOINT_DEG = 6.0   # post-safe_zero encoder bar (loaded sag;
                           # the 08-11 19:07 stalled leg read 78 deg)
ONE_GO_LEAD_S = 8.0        # single spoken lead-in before the session's
                           # first motion (operator 08-12: one short
                           # wait, no ARM dance, no long countdowns)
VIDEO_STATE_DIR = _HERE.parents[2] / "video_state"
IN_VIEW_STREAK = 3         # consecutive 'full' verdicts to open the gate
IN_VIEW_TIMEOUT_S = 300.0  # give up if the robot is never fully framed


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
        self.latest = None            # newest frame (for snapshots)
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
        self.latest = first_frame
        next_t = self.t0
        while not self._stop.is_set():
            ok, f = self._cap.read()      # blocks at the camera's rate
            if ok:
                latest = f
                self.latest = f
            now = time.time()
            # constant-rate output: repeat the newest frame as needed
            while next_t <= now:
                self._writer.write(latest)
                self.frames += 1
                next_t += 1.0 / self.fps

    def snapshot(self, path: Path) -> bool:
        """Save the newest frame as a still (zero_check etc.)."""
        if self.latest is None:
            return False
        import cv2
        path.parent.mkdir(parents=True, exist_ok=True)
        return bool(cv2.imwrite(str(path), self.latest))

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
                 camera: int | None = None, one_go: bool = False,
                 tilt_trip: float | None = None,
                 extra_hold: float = 0.0):
        self.c = client
        self.go = go
        self.rounds = rounds
        self.video = video or camera is not None
        self.auto = auto
        self.one_go = one_go
        self.tilt_trip = tilt_trip     # stand/lower tip envelope (deg)
        self.extra_hold = extra_hold   # stand/lower episode extension (s)
        self._model_line_said = False  # spoken model intro, once
        self._started = False          # one-go gate already passed?
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

    def _thermal_gate(self) -> None:
        """08-11 19:18 lesson: falls + recoveries stacked heat until the
        L2 hip hit 72 C (shutoff 65) mid-glide with nothing watching.
        Before every motion step, read bus temps; at/above THERMAL_WARN_C
        hold (limp cools servos fast) until back under THERMAL_RESUME_C,
        abort if it will not cool."""
        st = self.req("GET", "/api/status")
        temps = [(m.get("temp_c") or m.get("temp") or 0, m.get("name", "?"))
                 for m in st.get("motors", [])]
        if not temps:
            return
        t, name = max(temps)
        if t < THERMAL_WARN_C:
            return
        self.announce(f"thermal hold: {name} at {t} degrees. waiting for "
                      f"cooldown under {THERMAL_RESUME_C}.")
        deadline = time.time() + THERMAL_COOL_WAIT_S
        while time.time() < deadline:
            time.sleep(20.0)
            st = self.req("GET", "/api/status")
            temps = [(m.get("temp_c") or m.get("temp") or 0,
                      m.get("name", "?")) for m in st.get("motors", [])]
            t, name = max(temps) if temps else (0, "?")
            print(f"    thermal: hottest {name} {t}C")
            if t <= THERMAL_RESUME_C:
                self.announce("cooldown done. resuming.")
                return
        raise SessionAbort(f"thermal: {name} still {t}C after "
                           f"{THERMAL_COOL_WAIT_S:.0f}s cooldown")

    def _say(self, text: str) -> None:
        """announce() + ALWAYS speak (even in --camera mode, which
        normally skips speech) — speech is fire-and-forget (Popen),
        never a stall: operator 08-12 asked the script not to block
        on narration."""
        self.announce(text)
        if self.camera is not None:
            import subprocess
            try:
                subprocess.Popen(["say", "-r", "200", text])
            except Exception:
                pass

    def _wait_robot_in_view(self) -> None:
        """--one-go start gate (operator 08-12): hold the session until
        the hexapod is FULLY inside the camera frame, so a run can't be
        filmed half out of shot — and placing the robot in view is what
        actually starts the session. Uses the classical-CV red-and-blue
        classifier (video_state/detect.py) on the recorder's live feed;
        opens after IN_VIEW_STREAK consecutive 'full' verdicts ~0.5 s
        apart. Skipped with a spoken note if there is no live camera or
        the detector will not load — never a silent hang."""
        if self.recorder is None or self.recorder.t0 is None:
            self._say("no live camera. skipping the in-view check.")
            return
        try:
            if str(VIDEO_STATE_DIR) not in sys.path:
                sys.path.insert(0, str(VIDEO_STATE_DIR))
            from detect import RobotDetector
        except Exception as e:
            self._say("robot detector unavailable. skipping the "
                      "in-view check.")
            print(f"    !! robot_in_frame import failed: {e}")
            return
        det = RobotDetector()
        self._say("waiting for the whole robot to be in camera view.")
        deadline = time.time() + IN_VIEW_TIMEOUT_S
        streak = 0
        last_note = time.time()
        last_verdict = "?"
        while time.time() < deadline:
            frame = self.recorder.latest
            v = "no_frame"
            if frame is not None:
                v = det.detect(frame).verdict
            streak = streak + 1 if v == "full" else 0
            if streak >= IN_VIEW_STREAK:
                self._say("robot is fully in view.")
                return
            if v != last_verdict:
                print(f"    in-view: {v}")
                last_verdict = v
            if time.time() - last_note > 20.0:
                last_note = time.time()
                self._say(f"still waiting: robot is {v.replace('_', ' ')}.")
            time.sleep(0.5)
        raise SessionAbort(
            f"robot never fully in camera view for {IN_VIEW_STREAK} "
            f"consecutive checks within {IN_VIEW_TIMEOUT_S:.0f}s "
            f"(last verdict: {last_verdict})")

    def _one_go_gate(self, what: str) -> None:
        """The single operator wait (--one-go): hold until the robot is
        fully in camera view, then one spoken lead-in and the whole
        session runs --auto style (spoken step countdowns, Ctrl-C or
        the web STOP aborts). No ARM dance — the robot's motion workers
        arm the bus themselves and the operator asked for hands-off
        starts (08-12)."""
        self._wait_robot_in_view()
        self._say(f"bench session starting in {ONE_GO_LEAD_S:.0f} "
                  f"seconds. first motion: {what}. "
                  "control C or web stop to abort.")
        time.sleep(ONE_GO_LEAD_S)
        self._started = True
        self.auto = True               # spoken countdowns from here on

    def confirm(self, what: str, countdown: int = 3) -> bool:
        """Motion gate: explicit per-step operator go — or, in --auto,
        a SPOKEN countdown with Ctrl-C as the abort (the operator is
        watching the robot, not the keyboard). With --one-go the first
        motion step adds one short spoken lead-in, then everything
        runs auto-style."""
        if not self.go:
            print(f"    [dry-run] would: {what}")
            return False
        self._thermal_gate()
        if self.one_go and not self._started:
            self._one_go_gate(what)
        if self.auto:
            self._say(f"next: {what}. starting in {countdown} "
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
                        wait_s: float = 12.0,
                        prefix: str = "rl_walk_") -> str | None:
        """Newest episode CSV (default rl_walk_, prefix for
        rl_stand_/rl_lower_) WRITTEN AFTER after_unix, and only once its
        size has stopped growing. Two races bit on 08-11: the tape walk
        pulled the PREVIOUS csv (no after_unix), and vref1-r1 pulled a
        csv mid-flush and got 'too few rows'."""
        deadline = time.time() + wait_s
        last: tuple[str, int] | None = None
        while True:
            logs = self.req("GET", "/api/logs")
            found: tuple[str, int] | None = None
            for f in logs.get("files", []):
                if (f.get("name", "").startswith(prefix)
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

    def _model_line(self, mode: str) -> str | None:
        """'model <name>: <first words of its notes>' for the policy
        that will actually run ``mode`` — the ROLE assignment when one
        is set (the runner resolves roles first), else the live slot.
        Read-only; a fetch error just skips the line — the model intro
        must never block or kill the session."""
        try:
            name = ""
            roles = self.c._req("GET", "/api/rl/roles")
            if roles.get("ok"):
                r = (roles.get("roles") or {}).get(mode) or {}
                if r.get("file"):
                    name = str(r.get("resolved") or "").strip()
            info = self.c._req("GET", "/api/rl/policy")
            if not name:
                name = str(info.get("name") or "").strip()
            if not name:
                return None
            short = ""
            if name == str(info.get("name") or "").strip():
                short = " ".join(str(info.get("notes") or "").split()[:16])
            return f"model {name}" + (f": {short}" if short else "")
        except Exception:
            return None

    def _transition(self, mode: str) -> bool:
        """Learned stand ('stand') or lower ('lower') with the TERMINAL
        result recorded — returns True only on a clean finish. Pulls
        the episode CSV + its _summary.json into the session dir (same
        treatment walks always got)."""
        self.announce(f"learned "
                      f"{'stand up' if mode == 'stand' else mode} starting")
        line = self._model_line(mode)
        if line and not self._model_line_said:
            self._say(line)            # spoken async — never blocks
            self._model_line_said = True
        elif line:
            self.announce(line)
        t0 = time.time()
        body: dict = {}
        if self.tilt_trip:
            body["tilt_trip_deg"] = float(self.tilt_trip)
        if self.extra_hold:
            body["extra_hold_s"] = float(self.extra_hold)
        if body:
            print(f"    aggressive-test params: {body}")
        kick = self.req("POST", f"/api/rl/{mode}", body)
        entry: dict = {"kick": kick}
        if kick.get("ok", True):
            end_state = self.c.wait_idle(timeout_s=90.0)
            entry["result"] = self._runner_result(end_state)
            name = self.newest_walk_csv(after_unix=t0,
                                        prefix=f"rl_{mode}_")
            if name and self.pull_csv(name):
                entry["csv"] = name
                sname = name[:-4] + "_summary.json"
                if self.pull_csv(sname):
                    entry["summary_json"] = sname
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

    def _wait_demo(self, timeout_s: float = 90.0) -> dict:
        """Poll /api/pose until the current demo (safe_zero, zero_stand…)
        stops running; returns the final demo block. Demos are invisible
        to HexapodClient.wait_idle, which only watches the calibrate/RL
        runner."""
        t_end = time.time() + timeout_s
        demo: dict = {}
        while time.time() < t_end:
            d = self.req("GET", "/api/pose")
            demo = d.get("demo") or {}
            if not demo.get("running"):
                return demo
            time.sleep(1.0)
        return demo

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
        # force=true: a fall by definition leaves the body past the IMU
        # tilt gate, so the un-forced call ALWAYS refuses right when
        # recovery is needed (19:13 session died to exactly this).
        # force bypasses ONLY the tilt gate — the stall / unexpected-force
        # limp protection stays armed throughout the motion.
        r = self.req("POST", "/api/safe_zero", {"force": True})
        demo = self._wait_demo(timeout_s=60.0)
        if not r.get("ok", True) or "error" in str(demo.get("status", "")).lower():
            raise SessionAbort(f"safe_zero refused/errored: "
                               f"{demo.get('status', r)}")
        if not self._transition("stand"):
            # 08-11 evening: the learned rise trips tilt_roll ~10 deg at
            # the same curl tick 3/3 from clean zero — a deterministic
            # sim-to-real gap (cw-stand-riserock1 queued for it). Fall
            # back to the SCRIPTED validated plant stand so one fall
            # doesn't end the whole session.
            self.announce("learned stand tripped. using the scripted "
                          "stand glide instead.")
            z = self.req("POST", "/api/zero", {"pose": "stand"})
            # /api/zero runs as a DEMO (zero_stand), which wait_idle does
            # NOT cover — 19:16 session read a mid-glide pose, failed the
            # preflight, and aborted while the robot was still moving.
            # Poll the demo itself, and surface ITS error (that session's
            # real failure was 'LIMP — L2 hip at 72 C', not a bad pose).
            demo = self._wait_demo(timeout_s=90.0)
            status = str(demo.get("status", ""))
            if "error" in status.lower():
                raise SessionAbort(f"scripted stand errored: {status}")
            pf = self.req("GET", "/api/rl/preflight?mode=walk")
            if not z.get("ok", True) or not pf.get("ok", True):
                raise SessionAbort("recovery stand-up failed (learned "
                                   "AND scripted)")
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
                                countdown=4):
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
        # 08-11 19:07 lesson: the session started with L4 knee 78 deg off
        # zero (a stalled safe_zero leg, quietly hold-hunting) and the
        # rise curled asymmetrically straight into a tilt trip. In --auto
        # nobody is there to eyeball the pose, so when the stand
        # preflight rejects it, run safe_zero OURSELVES and re-check
        # before committing to the rise.
        pf = self.req("GET", "/api/rl/preflight?mode=stand")
        if self.auto and not pf.get("ok", True):
            self.announce("pose is not belly zero. running safe zero "
                          "before the stand-up.")
            r = self.req("POST", "/api/safe_zero", {})
            self._wait_demo(timeout_s=60.0)
            pf = self.req("GET", "/api/rl/preflight?mode=stand")
            if not r.get("ok", True) or not pf.get("ok", True):
                raise SessionAbort(
                    "start pose is not belly zero even after safe_zero: "
                    + json.dumps(pf)[:160])
        if not self.confirm("RL STAND UP (rise only)", countdown=4):
            return
        if not self._transition("stand") and self.auto:
            raise SessionAbort("opening stand-up failed")

    def step_sit(self) -> None:
        """Learned lower alone — the session closer (park on the belly)."""
        if self.confirm("RL LOWER (sit only)", countdown=4):
            self._transition("lower")

    def step_zero(self) -> None:
        """Laptop-driven safe zero + verification (operator ruling
        08-12: safe_zero may run from the laptop — it is the slow
        tilt-gated glide; stall/unexpected-force limp protection stays
        armed throughout). After the glide, verify the zero stance two
        ways BEFORE anything riskier is offered: every encoder near
        0 deg (catches the 08-11 stalled-leg-78-deg-off trap), and a
        camera still for the visual check. NOT a set_zero remap — it
        never rewrites the zero frame, only glides to it."""
        if not self.confirm("SAFE ZERO glide to belly zero (slow, "
                            "tilt-gated)"):
            return
        self.announce("safe zero starting")
        r = self.req("POST", "/api/safe_zero", {})
        if not r.get("ok", True) and "tilt" in str(r.get("error", "")).lower():
            # The rest pose sits on the wedge past the IMU tilt gate, so
            # the unforced call refuses right at session start (08-11
            # 19:13 recovery lesson). force bypasses ONLY the tilt gate;
            # stall/unexpected-force limp protection stays armed.
            self.announce("tilt gate refused safe zero. retrying with "
                          "force, tilt gate only.")
            r = self.req("POST", "/api/safe_zero", {"force": True})
        demo = self._wait_demo(timeout_s=90.0)
        status = str(demo.get("status", ""))
        if not r.get("ok", True) or "error" in status.lower():
            raise SessionAbort(f"safe_zero refused/errored: "
                               f"{status or r}")
        pose = self.req("GET", "/api/pose")
        raw = pose.get("degrees") or []
        missing = sum(1 for d in raw if d is None)
        degs = [(abs(d), i) for i, d in enumerate(raw) if d is not None]
        worst, wi = max(degs) if degs else (None, -1)
        names = ("yaw", "hip", "knee")
        worst_name = f"L{wi // 3} {names[wi % 3]}" if wi >= 0 else "?"
        entry: dict = {"t_unix": round(time.time(), 2),
                       "worst_joint_deg": worst,
                       "worst_joint": worst_name,
                       "missing_joints": missing}
        ok = (worst is not None and missing == 0
              and worst <= ZERO_MAX_JOINT_DEG)
        entry["encoders_ok"] = ok
        print(f"    encoders: worst |joint| = {worst} deg on "
              f"{worst_name} (bar {ZERO_MAX_JOINT_DEG}), "
              f"missing={missing} -> {'OK' if ok else 'NOT ZERO'}")
        if self.recorder is not None:
            still = self.out / "zero_check.jpg"
            if self.recorder.snapshot(still):
                entry["still"] = still.name
                self.announce("zero check still saved")
                print(f"    visual check: {still}\n"
                      "    (legs straight out, belly down, level — "
                      "review before any stand)")
        self.summary["zero_check"] = entry
        if not ok:
            # Encoders that disagree with the commanded zero mean a
            # stalled/jammed leg or a wrong logical zero — per the
            # safety rules never push a stand on top of it.
            raise SessionAbort(
                f"zero stance NOT verified after safe_zero (worst "
                f"{worst} deg on {worst_name}, {missing} missing) — "
                "eyeball the robot / zero_check.jpg; if encoders and "
                "the photo disagree, re-do set_zero from the web UI")

    def step_turnsign(self) -> None:
        for omega in (0.3, -0.3):
            if not self.confirm(f"scripted turn-in-place omega={omega:+} "
                                f"rad/s, 6s"):
                continue
            self.announce(f"turn sign omega {omega:+} starting")
            t0 = time.time()
            if self.video:
                # Video mode never annotates on the spot, so the previous
                # turn's pending measure record BLOCKS the next one (08-11
                # 19:33: the -0.3 turn was silently refused this way and
                # the robot just stood there for its window). The turn
                # sign comes from the footage; the record is disposable.
                self.req("POST", "/api/measure/discard", {})
            r = self.req("POST", "/api/measure/walk",
                         {"vx_mm": 0.0, "omega": omega, "duration_s": 6.0})
            if not r.get("ok", True):
                self.summary.setdefault("turnsign", []).append(
                    {"omega": omega, "observed": "REFUSED",
                     "error": r.get("error")})
                continue
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
                print("next: uv run python -m rl_move.scripts.video_review "
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
                         "SPOKEN 3s countdown (4s for STAND); Ctrl-C "
                         "aborts and stops the robot. The operator "
                         "must be watching the robot the whole time.")
    ap.add_argument("--camera", type=int, default=None, metavar="N",
                    help="record the Mac's camera (device index N, 0 = "
                         "built-in) for the whole session — implies "
                         "--video minus the phone: exact unix-time sync, "
                         "footage lands in the session dir as camera.mp4")
    ap.add_argument("--one-go", action="store_true",
                    help="hands-off start: ONE spoken lead-in "
                         f"({ONE_GO_LEAD_S:.0f}s) before the first "
                         "motion, then the whole session runs --auto "
                         "style with short spoken countdowns; no ARM "
                         "dance, no typed go's; Ctrl-C / web STOP abort")
    ap.add_argument("--tilt-trip", type=float, default=None,
                    metavar="DEG",
                    help="stand/lower tip envelope override, deg "
                         "(robot clamps 5..30; default = robot config "
                         "10). Operator 08-12: be more aggressive — "
                         "the 10 deg trip kills every rise mid-curl.")
    ap.add_argument("--extra-hold", type=float, default=0.0,
                    metavar="S",
                    help="extend stand/lower episodes past the "
                         "profile's total_s (robot clamps 0..15 s); "
                         "the height ref just holds the target longer")
    args = ap.parse_args()

    s = Session(HexapodClient(args.base), args.go, args.rounds,
                video=args.video, auto=args.auto, camera=args.camera,
                one_go=args.one_go, tilt_trip=args.tilt_trip,
                extra_hold=args.extra_hold)
    s.run(args.only, args.skip)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
