#!/usr/bin/env python3
"""
Web control panel for the STS3215 hexapod.

Runs on the Uno Q Linux side (or a laptop) and serves a drive UI over
HTTP/HTTPS.  Open from your laptop — on-screen sticks, keyboard, or an Xbox
controller plugged into the laptop (browser Gamepad API needs HTTPS).

Commands go to ``DriveController`` (TripodGait → Feetech bus), not the old
v1 STM32 ``J`` bridge.

  python3 web_drive.py                 # :8080 + https :8443
  python3 web_drive.py --dry-run       # UI only (no bus)
  python3 web_drive.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import json
import os
import ssl
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from bench_api import BenchAPI  # noqa: E402
from drive_controller import DriveController  # noqa: E402

# Self-signed TLS material for the HTTPS listener (generated on first run).
CERT_FILE = os.path.expanduser("~/.hexapod_sts_cert.pem")
KEY_FILE = os.path.expanduser("~/.hexapod_sts_key.pem")


def ensure_cert():
    """Make a long-lived self-signed cert if we don't have one. Returns True
    when both files are present afterwards."""
    if os.path.exists(CERT_FILE) and os.path.exists(KEY_FILE):
        return True
    try:
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
             "-keyout", KEY_FILE, "-out", CERT_FILE, "-days", "3650",
             "-subj", "/CN=hexapod.local",
             "-addext", "subjectAltName=DNS:hexapod.local,DNS:localhost"],
            check=True, capture_output=True)
        print(f"[https] generated self-signed cert at {CERT_FILE}")
        return True
    except (OSError, subprocess.CalledProcessError) as e:
        print(f"[https] cert generation failed ({e}); HTTPS disabled")
        return False

# Persisted calibration (the UNO Q has no EEPROM, so the standing foot height
# the user dials in is remembered HERE, on the board's Linux side, and pushed
# to the firmware with `Z <mm>` on startup / page load).
CAL_FILE = os.path.expanduser("~/.hexapod_cal.json")


def load_cal():
    try:
        with open(CAL_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_cal(d):
    try:
        tmp = CAL_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, CAL_FILE)
        return True
    except OSError as e:
        print(f"[cal] save failed: {e}")
        return False


CAL = load_cal()


class Link:
    """Thread-safe bridge from HTTP /cmd into DriveController.handle()."""

    def __init__(self, drive: DriveController):
        self.drive = drive
        self.lock = threading.Lock()

    def send(self, line: str) -> bool:
        with self.lock:
            try:
                try:
                    from event_log import emit
                    emit("cmd", line.strip(), src="drive",
                         data={"line": line.strip()})
                except Exception:
                    pass
                self.drive.handle(line)
                return True
            except Exception as e:
                print(f"[link] handle failed: {e}")
                return False


LINK = None   # set in main()
DRIVE = None
BENCH = None
HTTPS_PORT = None   # actual HTTPS port that bound (443 if privileged, else 8443)


# Browser UI lives in webui/ next to this file (index.html + style.css +
# app.js + favicon.svg -- see webui/README.md). Resolved relative to
# __file__, never the CWD, because systemd starts us from an arbitrary
# directory. Files are read fresh per request so robot-side edits show up
# on a browser reload without restarting the server.
WEBUI_DIR = HERE / "webui"
PAGE_PATHS = ("/", "/index.html", "/debug", "/motors", "/demos",
              "/dance", "/quad", "/rl", "/experiments", "/measure",
              "/calibrate")
# Exact whitelisted names only -- no generic static-dir handler, so nothing
# else on disk is reachable (path-traversal safety).
STATIC_FILES = {
    "/style.css": ("style.css", "text/css; charset=utf-8",
                   "no-store, max-age=0, must-revalidate"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8",
                "no-store, max-age=0, must-revalidate"),
    "/favicon.svg": ("favicon.svg", "image/svg+xml", "max-age=86400"),
}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass  # access noise; meaningful traffic goes through emit_http

    def _send(self, code, body, ctype="text/plain; charset=utf-8",
              cache=None):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        if cache:
            self.send_header("Cache-Control", cache)
        elif "text/html" in ctype:
            self.send_header(
                "Cache-Control", "no-store, max-age=0, must-revalidate")
        self.end_headers()
        try:
            self.wfile.write(data)
        except OSError:
            pass

    def _json(self, code, obj):
        # Any error response the website shows also lands in the event
        # log + logs/errors.jsonl (refusals, ok:false, 4xx/5xx).
        try:
            err = None
            if isinstance(obj, dict):
                if obj.get("error"):
                    err = str(obj["error"])
                elif obj.get("ok") is False:
                    err = "request failed (no error message)"
            if err is not None or code >= 400:
                from event_log import emit_api_error
                emit_api_error(self.command, self.path, code=code,
                               error=err, peer=self._peer())
        except Exception:
            pass
        self._send(code, json.dumps(obj), "application/json")

    def _peer(self) -> str:
        try:
            return self.client_address[0]
        except Exception:
            return ""

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        try:
            from event_log import emit_http
            emit_http("GET", self.path, peer=self._peer())
        except Exception:
            pass
        if path in PAGE_PATHS:
            index = WEBUI_DIR / "index.html"
            try:
                page = index.read_text(encoding="utf-8")
            except OSError as e:
                self._send(500, f"webui file missing: expected {index} ({e})")
                return
            page = page.replace("__HTTPS_PORT__", str(HTTPS_PORT or 8443))
            self._send(
                200, page, "text/html; charset=utf-8",
                cache="no-store, max-age=0, must-revalidate")
        elif path in STATIC_FILES:
            name, ctype, cache = STATIC_FILES[path]
            fpath = WEBUI_DIR / name
            try:
                data = fpath.read_bytes()
            except OSError as e:
                self._send(500, f"webui file missing: expected {fpath} ({e})")
                return
            self._send(200, data, ctype, cache=cache)
        elif path == "/cal":
            self._json(200, {"stand_z": CAL.get("stand_z"),
                             "tuck_r": CAL.get("tuck_r")})
        elif path == "/api/ping":
            # Lightweight heartbeat — no bus traffic.
            self._json(200, {"ok": True, "service": "hexapod-web"})
        elif path == "/api/rl/preflight":
            mode = "stand"
            qs = self.path.split("?", 1)
            if len(qs) == 2 and "mode=" in qs[1]:
                mode = qs[1].split("mode=")[1].split("&")[0]
            self._json(200, BENCH.rl_preflight(mode=mode) if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/policy":
            self._json(200, BENCH.rl_policy_info() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/policies":
            self._json(200, BENCH.rl_policies() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path.startswith("/api/rl/policies/"):
            # Export one policy JSON (push it to another robot / sim).
            name = Path(path[len("/api/rl/policies/"):]).name
            text = BENCH.get_rl_policy(name) if BENCH else None
            if text is None:
                self._json(404, {"ok": False,
                                 "error": f"no policy {name!r}"})
            else:
                self._send(200, text, "application/json")
        elif path == "/api/rl/roles":
            self._json(200, BENCH.rl_roles() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/drive":
            # Live drive-session snapshot (no bus traffic).
            self._json(200, BENCH.rl_drive_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/measure/list":
            self._json(200, BENCH.measure_list() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/logs":
            # List log files (episode traces, measurements, events) so
            # agents can pull data over HTTP instead of SSH (08-10).
            try:
                from event_log import log_dir
                d = log_dir()
                files = []
                for f in sorted(d.iterdir()):
                    if f.is_file():
                        st = f.stat()
                        files.append({"name": f.name,
                                      "bytes": st.st_size,
                                      "mtime_unix": round(st.st_mtime, 1)})
                files.sort(key=lambda x: -x["mtime_unix"])
                self._json(200, {
                    "ok": True, "dir": str(d), "files": files,
                    "hint": ("GET /api/logs/<name> downloads a file; "
                             "?tail=N returns only the last N lines")})
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
        elif path.startswith("/api/logs/"):
            try:
                from event_log import log_dir
                # Path().name forbids traversal / subdirs.
                name = Path(path[len("/api/logs/"):]).name
                fpath = log_dir() / name
                if not name or not fpath.is_file():
                    self._json(404, {"ok": False,
                                     "error": f"no such log: {name!r}"})
                else:
                    tail = 0
                    qs = self.path.split("?", 1)
                    if len(qs) == 2 and "tail=" in qs[1]:
                        try:
                            tail = int(qs[1].split("tail=")[1]
                                       .split("&")[0])
                        except ValueError:
                            tail = 0
                    data = fpath.read_bytes()
                    if tail > 0:
                        lines = data.splitlines(keepends=True)
                        data = b"".join(lines[-tail:])
                    ctype = {
                        ".csv": "text/csv; charset=utf-8",
                        ".json": "application/json",
                        ".jsonl": "application/x-ndjson",
                    }.get(fpath.suffix, "text/plain; charset=utf-8")
                    self._send(200, data, ctype=ctype)
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
        elif path == "/api/events":
            try:
                from event_log import recent, events_path, stats
                n = 100
                qs = self.path.split("?", 1)
                if len(qs) == 2 and "n=" in qs[1]:
                    try:
                        n = int(qs[1].split("n=")[1].split("&")[0])
                    except ValueError:
                        n = 100
                self._json(200, {
                    "ok": True,
                    "path": str(events_path()),
                    "stats": stats(),
                    "events": recent(n),
                })
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
        elif path == "/api/errors":
            try:
                from event_log import errors_path, recent
                n = 100
                qs = self.path.split("?", 1)
                if len(qs) == 2 and "n=" in qs[1]:
                    try:
                        n = int(qs[1].split("n=")[1].split("&")[0])
                    except ValueError:
                        n = 100
                n = max(1, min(1000, n))
                # Read the persisted errors.jsonl so history survives
                # service restarts (the in-memory ring starts empty).
                errs = []
                ep = errors_path()
                if ep.is_file():
                    tail = ep.read_bytes()[-512 * 1024:]
                    for ln in tail.decode("utf-8", "replace").splitlines():
                        try:
                            errs.append(json.loads(ln))
                        except ValueError:
                            continue   # partial first line of the tail
                    errs = errs[-n:]
                else:
                    errs = [e for e in recent(500)
                            if e.get("level") == "error"][-n:]
                self._json(200, {
                    "ok": True,
                    "path": str(ep),
                    "errors": errs,
                })
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
        elif path == "/api/demo/status":
            # Back-compat; prefer /api/robot.
            if BENCH:
                self._json(200, BENCH.robot_state(check_zero=False))
            else:
                self._json(200, {
                    "activity": "idle", "demo": {
                        "name": None, "status": "idle", "running": False,
                    }})
        elif path == "/api/robot":
            qs = self.path.split("?", 1)
            want_zero = False
            if len(qs) == 2:
                want_zero = "zero=1" in qs[1] or "zero=true" in qs[1]
            if BENCH:
                self._json(200, BENCH.robot_state(check_zero=want_zero))
            else:
                self._json(200, {"activity": "idle", "error": "no bench"})
        elif path == "/api/status":
            self._json(200, BENCH.status() if BENCH else {"error": "no bench"})
        elif path == "/api/pose":
            self._json(200, BENCH.pose() if BENCH else {"ok": False, "error": "no bench"})
        elif path == "/api/demos":
            self._json(200, {"demos": BENCH.list_demos() if BENCH else []})
        elif path == "/api/dances":
            self._json(200, {"dances": BENCH.list_dance_scripts()
                             if BENCH else []})
        elif path.startswith("/api/dances/"):
            # Export one uploaded dance script (push it to another robot).
            name = Path(path[len("/api/dances/"):]).name
            script = BENCH.get_dance_script(name) if BENCH else None
            if script is None:
                self._json(404, {"ok": False,
                                 "error": f"no uploaded dance {name!r}"})
            else:
                self._json(200, script)
        elif path == "/api/calibrate":
            self._json(200, BENCH.calibrate_state() if BENCH
                       else {"running": False, "error": "no bench"})
        elif path == "/api/plant":
            self._json(200, BENCH.plant_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/imu":
            self._json(200, BENCH.imu_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/calibration/report":
            self._json(200, BENCH.calibration_report() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/state" or path == "/api/rl":
            self._json(200, BENCH.rl_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/standup/modes":
            self._json(200, BENCH.standup_modes() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/feedback":
            # Fast read-only bulk telemetry (one MCU round-trip + IMU) for
            # external loggers — /api/status's full scan takes seconds.
            self._json(200, BENCH.rl_feedback() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/pinned_tip":
            # Read-only pinned-leg-tip verdict (tipped over a folded
            # knee?). Never moves the robot.
            self._json(200, BENCH.pinned_tip_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        else:
            self._send(404, "not found")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(n) if n else b""
        body = raw.decode("utf-8", "ignore") if raw else ""
        path = self.path.split("?", 1)[0]
        body_obj = None
        if body:
            try:
                body_obj = json.loads(body)
            except ValueError:
                body_obj = body.strip()[:500]
        if path != "/api/rl/drive/cmd":
            # Drive heartbeats stream at ~5 Hz — logging each would
            # drown events.jsonl; the session's 25 Hz CSV has every ref.
            try:
                from event_log import emit_http
                emit_http("POST", self.path, body=body_obj,
                          peer=self._peer())
            except Exception:
                pass
        if path == "/cmd":
            line = body.strip()
            # E-STOP / limp MUST go through the bench so the demo worker is
            # aborted first — a bare drive-level X limps the bus but the
            # still-running demo re-enables torque on its next write and
            # keeps going (observed with the dance, 2026-08-18).
            if line.upper() in ("X", "DISARM", "RELAX") and BENCH is not None:
                BENCH.estop()
                self._send(200, "limp")
            else:
                if BENCH is not None and line.upper() == "SETTLE":
                    # Graceful power-off also preempts any demo so the
                    # settle doesn't fight a running routine.
                    BENCH._preempt_demo_thread(reason="settle", timeout=3.0)
                ok = LINK.send(line)
                self._send(200 if ok else 502, "ok" if ok else "link down")
        elif path == "/api/wiggle":
            try:
                data = json.loads(body or "{}")
                self._json(200, BENCH.wiggle(int(data.get("joint", -1)),
                                            float(data.get("amp", 6))))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/demo":
            try:
                data = json.loads(body or "{}")
                kw = dict(
                    speed=float(data.get("speed", 1.0)),
                    size=float(data.get("size", 1.0)),
                    softness=float(data.get("softness", 1.0)),
                )
                if "rate" in data and data.get("rate") is not None:
                    kw["rate"] = float(data["rate"])
                if "torque" in data and data.get("torque") is not None:
                    kw["torque"] = int(float(data["torque"]))
                if "seconds" in data and data.get("seconds") is not None:
                    kw["seconds"] = float(data["seconds"])
                if "motion_log" in data:
                    v = data.get("motion_log")
                    kw["motion_log"] = (
                        v.strip().lower() in ("1", "true", "yes", "on")
                        if isinstance(v, str) else bool(v))
                self._json(200, BENCH.run_demo(str(data.get("name", "")), **kw))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/demo/speed":
            # LIVE tempo: adjusts the running demo (streamed demos every
            # tick; breathe at the next half-breath).
            try:
                data = json.loads(body or "{}")
                self._json(200, BENCH.set_demo_speed(data.get("speed", 1.0)))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/demo/stop":
            self._json(200, BENCH.stop_demo())
        elif path == "/api/rl/policies":
            # Upload an RL policy (policies as data — rl_move/np_policy).
            try:
                from rl_move.np_policy import MAX_POLICY_BYTES
                if n > MAX_POLICY_BYTES:
                    self._json(413, {"ok": False, "error": "policy too big"})
                    return
                qs = self.path.split("?", 1)
                name = ""
                if len(qs) == 2 and "name=" in qs[1]:
                    name = qs[1].split("name=", 1)[1].split("&", 1)[0]
                self._json(200, BENCH.save_rl_policy(body_obj, name=name))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/rl/policies/delete":
            try:
                self._json(200, BENCH.delete_rl_policy(
                    str((body_obj or {}).get("file", ""))))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/dances":
            # Upload a dance script (dances as data — dance_script.py).
            try:
                import dance_script as DS
                if n > DS.MAX_SCRIPT_BYTES:
                    self._json(413, {"ok": False, "error": "script too big"})
                    return
                script = body_obj
                if isinstance(script, dict) and "script" in script:
                    script = script["script"]
                self._json(200, BENCH.save_dance_script(script))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/dances/delete":
            try:
                self._json(200, BENCH.delete_dance_script(
                    str((body_obj or {}).get("name", ""))))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/zero":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            pose = "sit"
            force = False
            if isinstance(data, dict):
                if data.get("pose"):
                    pose = str(data.get("pose"))
                force = bool(data.get("force", False))
            elif body and body.strip() in ("sit", "stand"):
                pose = body.strip()
            self._json(200, BENCH.go_zero(pose=pose, force=force))
        elif path == "/api/set_zero":
            self._json(200, BENCH.set_zero_here() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/safe_zero":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not isinstance(data, dict):
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.safe_zero(
                    dry_run=bool(data.get("dry_run", False)),
                    force=bool(data.get("force", False))))
        elif path == "/api/untrap":
            # Low-torque untrap fold; refuses unless the detector
            # confirms a pinned-leg tip (force=true for bench tests).
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not isinstance(data, dict):
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.untrap(
                    force=bool(data.get("force", False))))
        elif path == "/api/calibrate":
            try:
                data = json.loads(body or "{}")
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.run_calibrate(
                    mode=str(data.get("mode", "step")),
                    step_deg=float(data.get("step_deg", 10)),
                    nudge_deg=float(data.get("nudge_deg", 2)),
                    axis=str(data.get("axis", "all")),
                    clearance_mm=float(data.get("clearance_mm", 40)),
                    quad_body_frame=bool(data.get("quad_body_frame", True)),
                    force=bool(data.get("force", False)),
                ))
        elif path == "/api/calibrate/stop":
            self._json(200, BENCH.stop_calibrate() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/plant/reset":
            self._json(200, BENCH.reset_plant() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/imu/reset":
            self._json(200, BENCH.reset_imu() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/find_plant":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_find_plant(
                    clearance_mm=float(data.get("clearance_mm", 40)),
                    force=bool(data.get("force", False))))
        elif path == "/api/rl/capture_plant":
            self._json(200, BENCH.rl_capture_plant() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/stop":
            self._json(200, BENCH.rl_stop() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path in ("/api/rl/stand", "/api/rl/lower"):
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                kw = {}
                if data.get("tilt_trip_deg"):
                    kw["tilt_trip_deg"] = float(data["tilt_trip_deg"])
                if data.get("extra_hold_s"):
                    kw["extra_hold_s"] = float(data["extra_hold_s"])
                self._json(200, BENCH.rl_policy_move(
                    mode=path.rsplit("/", 1)[-1], **kw))
        elif path == "/api/rl/walk":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_policy_move(
                    mode="walk",
                    vx=float(data.get("vx", 0.03)),
                    vy=float(data.get("vy", 0.0)),
                    duration_s=float(data.get("duration_s", 6.0)),
                    # rot-60 canonicalizer: default ON (exact no-op for
                    # forward-wedge commands); false = naked A/B baseline.
                    rot60=bool(data.get("rot60", True)),
                    # mirror chirality selection (TURN.md): "left" /
                    # "right" = arc turn, "hold" = heading hold;
                    # absent = today's naked path, bit-identical.
                    turn=(str(data["turn"]).strip().lower()
                          if data.get("turn") else None)))
        elif path == "/api/rl/set_stance":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_set_stance(
                    hip_deg=float(data.get("hip_deg", -20)),
                    knee_deg=float(data.get("knee_deg", 55)),
                    seconds=float(data.get("seconds", 10)),
                    yaw_deg=float(data.get("yaw_deg", 0)),
                    force=bool(data.get("force", False)),
                ))
        elif path == "/api/rl/roles":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_role_set(
                    role=str(data.get("role", "")),
                    file=str(data.get("file", ""))))
        elif path == "/api/rl/drive/start":
            self._json(200, BENCH.rl_drive_start() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/drive/cmd":
            # High-rate heartbeat (~5 Hz while driving): body-frame
            # vx/vy m/s. Kept out of the event log's HTTP stream by
            # volume; the session's own 25 Hz CSV logs every ref.
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_drive_cmd(
                    vx=float(data.get("vx", 0.0)),
                    vy=float(data.get("vy", 0.0))))
        elif path == "/api/rl/drive/stop":
            self._json(200, BENCH.rl_drive_stop() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/policy_select":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_policy_select(
                    file=str(data.get("file", ""))))
        elif path.startswith("/api/measure/"):
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            elif path == "/api/measure/walk":
                self._json(200, BENCH.measure_walk(
                    vx_mm=float(data.get("vx_mm", 30.0)),
                    vy_mm=float(data.get("vy_mm", 0.0)),
                    omega=float(data.get("omega", 0.0)),
                    duration_s=float(data.get("duration_s", 20.0))))
            elif path == "/api/measure/hold":
                self._json(200, BENCH.measure_hold(
                    label=str(data.get("label", "planted")),
                    duration_s=float(data.get("duration_s", 30.0))))
            elif path == "/api/measure/slip":
                self._json(200, BENCH.measure_slip())
            elif path == "/api/measure/annotate":
                self._json(200, BENCH.measure_annotate(
                    fields=data if isinstance(data, dict) else {}))
            elif path == "/api/measure/discard":
                self._json(200, BENCH.measure_discard())
            elif path == "/api/measure/note":
                self._json(200, BENCH.measure_note(
                    kind=str(data.get("kind", "note")),
                    fields=(data.get("fields")
                            if isinstance(data.get("fields"), dict)
                            else data)))
            else:
                self._json(404, {"ok": False, "error": "bad measure path"})
        elif path == "/api/standup":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.standup(
                    mode=str(data.get("mode", "tuck")),
                    speed=float(data.get("speed", 1.0)),
                    direction=str(data.get("direction", "up")),
                    force=bool(data.get("force", False)),
                    torque=int(data.get("torque", 700))))
        elif path == "/api/standup/stop":
            self._json(200, BENCH.stop_demo() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/sysid/run":
            # Deterministic sysid command stream (sysid_runner.py).
            # Body: {"protocol": {...}, "force": false}.
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            elif not isinstance(data, dict) or "protocol" not in data:
                self._json(400, {"ok": False,
                                 "error": "body needs {'protocol': {...}}"})
            else:
                self._json(200, BENCH.sysid_run(
                    data["protocol"],
                    force=bool(data.get("force", False))))
        elif path == "/api/rl/probe_dynamics":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_probe_dynamics(
                    amp_deg=float(data.get("amp_deg", 10)),
                    axis=str(data.get("axis", "all")),
                    soft_torque=int(data.get("soft_torque", 450)),
                ))
        elif path == "/cal":
            try:
                z = float(body.strip())
            except ValueError:
                self._send(400, "bad value")
                return
            CAL["stand_z"] = z
            saved = save_cal(CAL)
            LINK.send(f"Z {z:.1f}")
            self._send(200 if saved else 500, "saved" if saved else "save failed")
        elif path == "/cal_tuck":
            try:
                r = float(body.strip())
            except ValueError:
                self._send(400, "bad value")
                return
            CAL["tuck_r"] = r
            saved = save_cal(CAL)
            LINK.send(f"$ {r:.1f}")
            self._send(200 if saved else 500, "saved" if saved else "save failed")
        else:
            self._send(404, "not found")


def main():
    global LINK, DRIVE, BENCH, HTTPS_PORT
    ap = argparse.ArgumentParser(description="STS3215 hexapod web control panel")
    ap.add_argument("--bind", default="0.0.0.0", help="HTTP bind address")
    ap.add_argument("--http-port", type=int, default=8080, dest="http_port")
    ap.add_argument("--https-port", type=int, default=8443, dest="https_port",
                    help="HTTPS port for browser Xbox/Gamepad API (default 8443)")
    ap.add_argument("--port", default=None,
                    help="Feetech bus serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--dry-run", action="store_true",
                    help="serve UI without opening the servo bus")
    ap.add_argument("--log-host", default=None,
                    help="UDP host for timestamped event stream "
                         "(default: HEXAPOD_LOG_HOST env)")
    ap.add_argument("--log-port", type=int, default=None,
                    help="UDP port for event stream (default 9377)")
    args = ap.parse_args()

    try:
        from event_log import configure, emit, install_print_hook
        cfg = configure(host=args.log_host, port=args.log_port)
        install_print_hook()
        emit("boot", "web_drive start", src="web", data=cfg)
        sinks = cfg.get("udp") or []
        print(f"[log] events → {cfg['path']}  udp→{sinks or 'auto'}  "
              f"beacon:{cfg.get('beacon_port')}")
    except Exception as e:
        print(f"[log] event_log unavailable: {e}")

    DRIVE = DriveController(port=args.port, baud=args.baud, dry_run=args.dry_run)
    DRIVE.start()
    try:
        _main_after_bus(args)
    except Exception as e:
        # Bus is up but the web stack died — put the error on the TFT so a
        # headless robot isn't just silently stuck (service will restart).
        _show_fatal_on_tft(e)
        raise


def _show_fatal_on_tft(exc: BaseException) -> None:
    try:
        from status_display import _wrap
        bus = DRIVE.bus if DRIVE else None
        if bus is None or not hasattr(bus, "display_job"):
            return
        body = _wrap(f"{type(exc).__name__}: {exc}", 26, 4)
        while len(body) < 4:
            body.append("")
        bus.display_job(["WEB ERROR"] + body + ["restarting..."],
                        pct=-1, timeout=6.0)
    except Exception:
        pass


def _main_after_bus(args) -> None:
    global LINK, BENCH, HTTPS_PORT
    BENCH = BenchAPI(DRIVE)
    DRIVE.bench = BENCH
    LINK = Link(DRIVE)
    if not args.dry_run:
        # The ST7789 shares the MCU serial path with motion/test commands.
        # Keep it opt-in so cosmetic screen repaints cannot delay robot work.
        tft_status = os.environ.get("HEXAPOD_TFT_STATUS", "").strip().lower()
        if tft_status in ("1", "true", "yes", "on"):
            BENCH.start_status_display()
            print("[web] TFT status display started (MCU ST7789)")
        else:
            print("[web] TFT status display disabled "
                  "(set HEXAPOD_TFT_STATUS=1 to enable)")
        BENCH.start_servo_watch()
        print("[web] servo watch started (liveness + 65C cutoff)")

    if ensure_cert():
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(CERT_FILE, KEY_FILE)
        for hp in (args.https_port, 8443, 9443):
            try:
                httpsd = ThreadingHTTPServer((args.bind, hp), Handler)
                httpsd.socket = ctx.wrap_socket(httpsd.socket, server_side=True)
                threading.Thread(target=httpsd.serve_forever, daemon=True).start()
                HTTPS_PORT = hp
                print(f"[web] gamepad-ready HTTPS on https://{args.bind}:{hp}")
                break
            except PermissionError:
                print(f"[https] no privilege to bind {hp}; trying fallback")
            except OSError as e:
                print(f"[https] could not bind {hp} ({e})")

    srv = ThreadingHTTPServer((args.bind, args.http_port), Handler)
    print(f"[web] serving on http://{args.bind}:{args.http_port}")
    print("[web] open via adb:  adb forward tcp:8080 tcp:8080")
    print("[web]              then http://127.0.0.1:8080")
    if HTTPS_PORT:
        print(f"[web] Xbox/gamepad: adb forward tcp:{HTTPS_PORT} tcp:{HTTPS_PORT}")
        print(f"[web]              then https://127.0.0.1:{HTTPS_PORT}  (accept cert)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if BENCH:
            BENCH.stop_status_display()
        srv.server_close()
        DRIVE.close()


if __name__ == "__main__":
    main()
