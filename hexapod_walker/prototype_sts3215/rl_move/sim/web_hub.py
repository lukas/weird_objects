"""Target router for one laptop-hosted hexapod web UI.

The hub serves the same browser files as the robot, but routes API calls
to a local MuJoCo session, a robot-side web server, or both. The robot-side
server remains the hardware safety boundary for the Feetech bus.
"""
from __future__ import annotations

import json
import ssl
import threading
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import Any


@dataclass
class RouteResponse:
    code: int
    body: bytes
    ctype: str = "application/json"
    cache: str | None = None

    @classmethod
    def json(cls, obj: Any, code: int = 200) -> "RouteResponse":
        return cls(code, json.dumps(obj).encode("utf-8"), "application/json")

    @classmethod
    def text(cls, text: str, code: int = 200) -> "RouteResponse":
        return cls(code, text.encode("utf-8"), "text/plain; charset=utf-8")

    def json_obj(self) -> dict[str, Any]:
        try:
            obj = json.loads(self.body.decode("utf-8", "replace"))
        except ValueError:
            obj = {"ok": self.code < 400,
                   "body": self.body.decode("utf-8", "replace")}
        return obj if isinstance(obj, dict) else {"ok": self.code < 400,
                                                  "value": obj}

    def ok(self) -> bool:
        obj = self.json_obj()
        return self.code < 400 and obj.get("ok") is not False


def _json_body(raw: bytes) -> dict[str, Any]:
    if not raw:
        return {}
    try:
        obj = json.loads(raw.decode("utf-8", "ignore"))
    except ValueError:
        return {"_text": raw.decode("utf-8", "ignore")}
    return obj if isinstance(obj, dict) else {"_value": obj}


def _query_value(full_path: str, key: str, default: str = "") -> str:
    qs = urllib.parse.urlsplit(full_path).query
    vals = urllib.parse.parse_qs(qs)
    return vals.get(key, [default])[0]


class SimTarget:
    """Route adapter around ``SimWebSession``."""

    name = "sim"

    def __init__(self, session: Any):
        self.session = session

    def available(self) -> bool:
        return self.session is not None

    def ping_meta(self) -> dict[str, Any]:
        if not self.available():
            return {"available": False}
        try:
            meta = self.session.ping()
        except Exception as e:
            return {"available": True, "ok": False, "error": str(e)}
        return {"available": True, **meta}

    def request(self, method: str, full_path: str,
                body: bytes = b"", headers: dict[str, str] | None = None
                ) -> RouteResponse:
        path = full_path.split("?", 1)[0]
        try:
            if method == "GET":
                return self._get(path, full_path)
            if method == "POST":
                return self._post(path, body)
        except FileNotFoundError as e:
            return RouteResponse.json({"ok": False, "error": str(e)}, 404)
        except Exception as e:
            return RouteResponse.json({"ok": False, "error": str(e)}, 500)
        return RouteResponse.json({"ok": False,
                                   "error": f"bad method {method}"}, 405)

    def _get(self, path: str, full_path: str) -> RouteResponse:
        s = self.session
        if path == "/api/ping":
            return RouteResponse.json(s.ping())
        if path == "/api/robot":
            return RouteResponse.json(s.robot_state())
        if path == "/api/status":
            return RouteResponse.json(s.status())
        if path == "/api/calibrate":
            return RouteResponse.json(s.operation_state())
        if path in ("/api/rl", "/api/rl/state"):
            return RouteResponse.json(s.operation_state())
        if path == "/api/rl/preflight":
            return RouteResponse.json(s.rl_preflight(
                mode=_query_value(full_path, "mode", "stand")))
        if path == "/api/rl/policy":
            return RouteResponse.json(s.rl_policy_info())
        if path == "/api/rl/policies":
            return RouteResponse.json(s.rl_policies())
        if path == "/api/rl/roles":
            return RouteResponse.json(s.rl_roles())
        if path == "/api/rl/drive":
            return RouteResponse.json(s.rl_drive_state())
        if path == "/api/sim/state":
            return RouteResponse.json(s.sim_state())
        if path == "/api/sim/frame.jpg":
            return RouteResponse(200, s.frame_jpeg(), "image/jpeg",
                                 cache="no-cache")
        if path == "/api/logs":
            return RouteResponse.json(s.logs())
        if path.startswith("/api/logs/"):
            name = Path(path[len("/api/logs/"):]).name
            data, ctype = s.log_file(name, full_path)
            return RouteResponse(200, data, ctype)
        return RouteResponse.json({"ok": False,
                                   "error": f"no sim route: {path}"}, 404)

    def _post(self, path: str, raw: bytes) -> RouteResponse:
        s = self.session
        if path == "/cmd":
            line = raw.decode("utf-8", "ignore").strip()
            ok = s.cmd(line).get("ok")
            return RouteResponse.text("ok" if ok else "failed",
                                      200 if ok else 502)
        data = _json_body(raw)
        if path == "/api/standup":
            return RouteResponse.json(s.sim_reset(start="plant"))
        if path == "/api/rl/capture_plant":
            return RouteResponse.json(s.rl_capture_plant())
        if path == "/api/rl/stop":
            return RouteResponse.json(s.rl_stop())
        if path in ("/api/rl/stand", "/api/rl/lower"):
            return RouteResponse.json(s.rl_policy_move(
                mode=path.rsplit("/", 1)[-1]))
        if path == "/api/rl/walk":
            return RouteResponse.json(s.rl_policy_move(
                mode="walk",
                vx=float(data.get("vx", 0.03)),
                vy=float(data.get("vy", 0.0)),
                duration_s=float(data.get("duration_s", 6.0))))
        if path == "/api/rl/roles":
            return RouteResponse.json(s.rl_role_set(
                role=str(data.get("role", "")),
                file=str(data.get("file", ""))))
        if path == "/api/rl/drive/start":
            return RouteResponse.json(s.rl_drive_start())
        if path == "/api/rl/drive/cmd":
            return RouteResponse.json(s.rl_drive_cmd(
                vx=float(data.get("vx", 0.0)),
                vy=float(data.get("vy", 0.0))))
        if path == "/api/rl/drive/stop":
            return RouteResponse.json(s.rl_drive_stop())
        if path == "/api/rl/policy_select":
            return RouteResponse.json(s.rl_policy_select(
                file=str(data.get("file", ""))))
        if path == "/api/sim/reset":
            return RouteResponse.json(s.sim_reset(
                start=str(data.get("start", "plant"))))
        if path == "/api/sim/fall":
            return RouteResponse.json(s.sim_fall())
        if path == "/api/sim/recover":
            return RouteResponse.json(s.sim_recover())
        if path == "/api/sim/push":
            return RouteResponse.json(s.sim_push(
                x=float(data.get("x", 4.0)),
                y=float(data.get("y", 0.0))))
        return RouteResponse.json({"ok": False,
                                   "error": f"no sim route: {path}"}, 404)

    def close(self) -> None:
        close = getattr(self.session, "close", None)
        if close:
            close()

    def run_native_viewer(self, web_url: str = "") -> None:
        self.session.run_native_viewer(web_url)


class RobotProxyTarget:
    """HTTP proxy target for the robot-side ``linux_control/web_drive.py``."""

    name = "robot"

    def __init__(self, base_url: str, *, timeout: float = 5.0,
                 ping_timeout: float = 0.7, insecure_tls: bool = False):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.ping_timeout = ping_timeout
        self.ssl_context = None
        if insecure_tls:
            self.ssl_context = ssl._create_unverified_context()

    def available(self) -> bool:
        return bool(self.base_url)

    def ping_meta(self) -> dict[str, Any]:
        if not self.available():
            return {"available": False}
        resp = self.request("GET", "/api/ping", timeout=self.ping_timeout)
        obj = resp.json_obj()
        return {"available": True, "url": self.base_url,
                **obj, "ok": resp.ok()}

    def request(self, method: str, full_path: str,
                body: bytes = b"", headers: dict[str, str] | None = None,
                timeout: float | None = None) -> RouteResponse:
        if not self.available():
            return RouteResponse.json({"ok": False,
                                       "error": "robot target not configured"},
                                      503)
        url = self.base_url + full_path
        req_headers = {}
        if headers:
            for k, v in headers.items():
                if k.lower() in {"content-type", "accept"}:
                    req_headers[k] = v
        data = body if method == "POST" else None
        if data and "Content-Type" not in req_headers:
            req_headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, method=method,
                                     headers=req_headers)
        try:
            with urllib.request.urlopen(
                    req, timeout=self.timeout if timeout is None else timeout,
                    context=self.ssl_context) as r:
                ctype = r.headers.get("Content-Type",
                                      "application/octet-stream")
                return RouteResponse(r.status, r.read(), ctype)
        except urllib.error.HTTPError as e:
            ctype = e.headers.get("Content-Type", "text/plain; charset=utf-8")
            return RouteResponse(e.code, e.read(), ctype)
        except Exception as e:
            return RouteResponse.json({"ok": False,
                                       "error": f"robot proxy failed: {e}"},
                                      502)


class HubController:
    """Switches and broadcasts between configured robot/sim targets."""

    BROADCAST_POSTS = {
        "/cmd",
        "/api/standup",
        "/api/rl/stop",
        "/api/rl/stand",
        "/api/rl/lower",
        "/api/rl/walk",
        "/api/rl/roles",
        "/api/rl/drive/start",
        "/api/rl/drive/cmd",
        "/api/rl/drive/stop",
        "/api/rl/policy_select",
    }

    def __init__(self, sim: Any | None, robot: Any | None,
                 target: str = "sim"):
        self.sim = sim
        self.robot = robot
        self.lock = threading.RLock()
        self.target = "sim"
        self.set_target(target)

    def _has_sim(self) -> bool:
        return self.sim is not None and self.sim.available()

    def _has_robot(self) -> bool:
        return self.robot is not None and self.robot.available()

    def set_target(self, target: str) -> None:
        target = target.strip().lower()
        if target not in {"sim", "robot", "both"}:
            raise ValueError("target must be sim, robot, or both")
        if target in {"sim", "both"} and not self._has_sim():
            raise ValueError("sim target is not configured")
        if target in {"robot", "both"} and not self._has_robot():
            raise ValueError("robot target is not configured")
        with self.lock:
            self.target = target

    def ping(self) -> dict[str, Any]:
        with self.lock:
            target = self.target
        sim_meta = self.sim.ping_meta() if self._has_sim() else {
            "available": False}
        robot_meta = self.robot.ping_meta() if self._has_robot() else {
            "available": False}
        active_robot = target in {"robot", "both"}
        active_sim = target in {"sim", "both"}
        ok = True
        if active_robot:
            ok = ok and robot_meta.get("ok") is not False
        if active_sim:
            ok = ok and sim_meta.get("ok") is not False
        kind = "robot" if active_robot else "sim"
        return {
            "ok": ok,
            "service": "hexapod-hub",
            "kind": kind,
            "hub": True,
            "target": target,
            "viewer": bool(sim_meta.get("viewer", False)),
            "frames": bool(sim_meta.get("frames", True)),
            "active": {"robot": active_robot, "sim": active_sim},
            "targets": {"robot": robot_meta, "sim": sim_meta},
        }

    def handle_get(self, full_path: str,
                   headers: dict[str, str] | None = None) -> RouteResponse:
        path = full_path.split("?", 1)[0]
        if path == "/api/ping":
            return RouteResponse.json(self.ping(), 200)
        if path == "/api/hub":
            return RouteResponse.json(self.ping(), 200)
        if path.startswith("/api/sim/"):
            return self._send("sim", "GET", full_path, b"", headers)
        with self.lock:
            target = self.target
        if target == "sim":
            return self._send("sim", "GET", full_path, b"", headers)
        return self._send("robot", "GET", full_path, b"", headers)

    def handle_post(self, full_path: str, body: bytes,
                    headers: dict[str, str] | None = None) -> RouteResponse:
        path = full_path.split("?", 1)[0]
        if path == "/api/hub":
            data = _json_body(body)
            try:
                self.set_target(str(data.get("target", "")))
            except Exception as e:
                return RouteResponse.json({"ok": False, "error": str(e)}, 400)
            return RouteResponse.json({"ok": True, **self.ping()})
        if path.startswith("/api/sim/"):
            return self._send("sim", "POST", full_path, body, headers)
        with self.lock:
            target = self.target
        if target == "both" and path in self.BROADCAST_POSTS:
            return self._broadcast_post(full_path, body, headers)
        return self._send(target, "POST", full_path, body, headers)

    def _send(self, target: str, method: str, full_path: str, body: bytes,
              headers: dict[str, str] | None) -> RouteResponse:
        if target == "sim":
            if not self._has_sim():
                return RouteResponse.json({"ok": False,
                                           "error": "sim target unavailable"},
                                          503)
            return self.sim.request(method, full_path, body, headers)
        if not self._has_robot():
            return RouteResponse.json({"ok": False,
                                       "error": "robot target unavailable"},
                                      503)
        return self.robot.request(method, full_path, body, headers)

    def _broadcast_post(self, full_path: str, body: bytes,
                        headers: dict[str, str] | None) -> RouteResponse:
        robot_resp = self._send("robot", "POST", full_path, body, headers)
        sim_resp = self._send("sim", "POST", full_path, body, headers)
        path = full_path.split("?", 1)[0]
        if path == "/cmd":
            ok = robot_resp.ok() and sim_resp.ok()
            return RouteResponse.text("ok" if ok else "failed",
                                      200 if ok else 502)
        primary = robot_resp if self._has_robot() else sim_resp
        obj = primary.json_obj()
        out = dict(obj)
        out["ok"] = primary.ok() and robot_resp.ok() and sim_resp.ok()
        out["hub"] = {
            "target": "both",
            "robot": robot_resp.json_obj(),
            "sim": sim_resp.json_obj(),
        }
        return RouteResponse.json(out, 200 if out["ok"] else 502)

    def close(self) -> None:
        if self.sim is not None:
            self.sim.close()

    def run_native_viewer(self, web_url: str = "") -> None:
        if self.sim is not None:
            self.sim.run_native_viewer(web_url)


def make_hub_handler(hub: HubController, webui_dir: Path,
                     https_port: int, page_paths: set[str] | tuple[str, ...],
                     static_files: dict[str, tuple[str, str, str]]
                     ) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def _send(self, resp: RouteResponse) -> None:
            self.send_response(resp.code)
            self.send_header("Content-Type", resp.ctype)
            self.send_header("Content-Length", str(len(resp.body)))
            if resp.cache:
                self.send_header("Cache-Control", resp.cache)
            elif "text/html" in resp.ctype:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(resp.body)
            except OSError:
                pass

        def _static(self, path: str) -> bool:
            if path in page_paths:
                index = webui_dir / "index.html"
                try:
                    page = index.read_text(encoding="utf-8")
                except OSError as e:
                    self._send(RouteResponse.text(
                        f"webui file missing: expected {index} ({e})", 500))
                    return True
                page = page.replace("__HTTPS_PORT__", str(https_port))
                self._send(RouteResponse(
                    200, page.encode("utf-8"), "text/html; charset=utf-8",
                    cache="no-cache"))
                return True
            if path in static_files:
                name, ctype, cache = static_files[path]
                fpath = webui_dir / name
                try:
                    data = fpath.read_bytes()
                except OSError as e:
                    self._send(RouteResponse.text(
                        f"webui file missing: expected {fpath} ({e})", 500))
                    return True
                self._send(RouteResponse(200, data, ctype, cache=cache))
                return True
            return False

        def _request_headers(self) -> dict[str, str]:
            return {k: v for k, v in self.headers.items()}

        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if self._static(path):
                return
            self._send(hub.handle_get(self.path, self._request_headers()))

        def do_POST(self) -> None:
            n = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(n) if n else b""
            self._send(hub.handle_post(self.path, body,
                                       self._request_headers()))

    return Handler
