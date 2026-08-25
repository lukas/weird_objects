"""Target router for one laptop-hosted hexapod web UI.

The hub serves the same browser files as the robot, but routes API calls
to a local MuJoCo session, a robot-side web server, or both. The robot-side
server remains the hardware safety boundary for the Feetech bus.
"""
from __future__ import annotations

import importlib.util
import json
import queue
import socket
import ssl
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import warnings
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import Any

ROBOT_PING_TIMEOUT_S = 2.0
ROBOT_CATALOG_TIMEOUT_S = 1.5
ROBOT_RESOLVE_TTL_S = 30.0
ROBOT_DEFAULT_TIMEOUT_S = 5.0
ROBOT_SET_ZERO_TIMEOUT_S = 20.0
ROBOT_MOTION_START_TIMEOUT_S = 12.0

ROBOT_ROUTE_TIMEOUTS_S = {
    # Feetech middle-calibrate touches every live servo. It is non-motion, but
    # routinely takes longer than a generic proxy request; timing out here is
    # especially confusing because the robot may still finish and redefine
    # zero after the hub has already reported failure.
    "/api/set_zero": ROBOT_SET_ZERO_TIMEOUT_S,
    # These can spend a few seconds preempting or starting a guarded motion
    # before returning the accepted/failed JSON receipt.
    "/api/safe_zero": ROBOT_MOTION_START_TIMEOUT_S,
    "/api/zero": ROBOT_MOTION_START_TIMEOUT_S,
}


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


def _cmd_line(raw: bytes) -> str:
    return raw.decode("utf-8", "ignore").strip().upper()


def _header_value(headers: dict[str, str] | None, name: str) -> str:
    if not headers:
        return ""
    want = name.lower()
    for k, v in headers.items():
        if k.lower() == want:
            return str(v)
    return ""


def _query_value(full_path: str, key: str, default: str = "") -> str:
    qs = urllib.parse.urlsplit(full_path).query
    vals = urllib.parse.parse_qs(qs)
    return vals.get(key, [default])[0]


def _normalize_base_url(base_url: str) -> str:
    raw = (base_url or "").strip()
    if not raw:
        return ""
    if "://" not in raw:
        raw = "http://" + raw
    p = urllib.parse.urlsplit(raw)
    if p.scheme not in {"http", "https"}:
        raise ValueError("robot URL must start with http:// or https://")
    if not p.netloc:
        raise ValueError("robot URL needs a host")
    return urllib.parse.urlunsplit((p.scheme, p.netloc, "", "", ""))


_LOCAL_ROBOT_DEMOS: list[dict[str, Any]] | None = None
_LOCAL_ROBOT_DEMOS_ERROR: str | None = None
_LOCAL_ROBOT_DEMOS_LOCK = threading.Lock()


def _local_robot_demos() -> tuple[list[dict[str, Any]], str | None]:
    """Return the checked-in robot demo catalog without contacting hardware."""
    global _LOCAL_ROBOT_DEMOS, _LOCAL_ROBOT_DEMOS_ERROR
    with _LOCAL_ROBOT_DEMOS_LOCK:
        if _LOCAL_ROBOT_DEMOS is not None:
            return [dict(x) for x in _LOCAL_ROBOT_DEMOS], None
        if _LOCAL_ROBOT_DEMOS_ERROR is not None:
            return [], _LOCAL_ROBOT_DEMOS_ERROR

        root = Path(__file__).resolve().parents[2]
        web_control = root / "linux_control"
        urt2 = web_control / "urt2_setup"
        bench_path = web_control / "bench_api.py"
        old_path = list(sys.path)
        try:
            for p in (str(web_control), str(urt2)):
                if p not in sys.path:
                    sys.path.insert(0, p)
            spec = importlib.util.spec_from_file_location(
                "_hexapod_local_bench_api", bench_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"cannot load {bench_path}")
            mod = importlib.util.module_from_spec(spec)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", SyntaxWarning)
                spec.loader.exec_module(mod)

                class _DemoCatalogOnly:
                    def list_dance_scripts(self) -> list[dict[str, Any]]:
                        return []

                demos = mod.BenchAPI.list_demos(_DemoCatalogOnly())
            _LOCAL_ROBOT_DEMOS = [dict(x) for x in demos]
            return [dict(x) for x in _LOCAL_ROBOT_DEMOS], None
        except Exception as e:
            _LOCAL_ROBOT_DEMOS_ERROR = str(e)
            return [], _LOCAL_ROBOT_DEMOS_ERROR
        finally:
            sys.path[:] = old_path


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
                return self._post(path, body, full_path)
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
        if path == "/api/demo/status":
            return RouteResponse.json(s.robot_state())
        if path == "/api/demos":
            return RouteResponse.json({"demos": s.list_demos()})
        if path == "/api/pose":
            return RouteResponse.json(s.pose())
        if path == "/api/calibrate":
            return RouteResponse.json(s.operation_state())
        if path == "/api/calibration/report":
            return RouteResponse.json(s.calibration_report())
        if path in ("/api/rl", "/api/rl/state"):
            return RouteResponse.json(s.operation_state())
        if path == "/api/rl/preflight":
            return RouteResponse.json(s.rl_preflight(
                mode=_query_value(full_path, "mode", "stand")))
        if path == "/api/rl/policy":
            return RouteResponse.json(s.rl_policy_info())
        if path == "/api/rl/policies":
            return RouteResponse.json(s.rl_policies())
        if path.startswith("/api/rl/policies/"):
            name = Path(path[len("/api/rl/policies/"):]).name
            text = s.get_rl_policy(name)
            if text is None:
                return RouteResponse.json(
                    {"ok": False, "error": f"no policy {name!r}"}, 404)
            return RouteResponse(200, text.encode("utf-8"),
                                 "application/json")
        if path == "/api/rl/roles":
            return RouteResponse.json(s.rl_roles())
        if path == "/api/rl/drive":
            return RouteResponse.json(s.rl_drive_state())
        if path == "/api/standup/modes":
            return RouteResponse.json(s.standup_modes())
        if path == "/api/sim/state":
            return RouteResponse.json(s.sim_state())
        if path == "/api/sim/frame.jpg":
            return RouteResponse(200, s.frame_jpeg(), "image/jpeg",
                                 cache="no-cache")
        if path == "/api/dances":
            return RouteResponse.json({"dances": s.list_dance_scripts()})
        if path.startswith("/api/dances/"):
            name = Path(path[len("/api/dances/"):]).name
            script = s.get_dance_script(name)
            if script is None:
                return RouteResponse.json(
                    {"ok": False, "error": f"no dance {name!r}"}, 404)
            return RouteResponse.json(script)
        if path == "/api/logs":
            return RouteResponse.json(s.logs())
        if path.startswith("/api/logs/"):
            name = Path(path[len("/api/logs/"):]).name
            data, ctype = s.log_file(name, full_path)
            return RouteResponse(200, data, ctype)
        return RouteResponse.json({"ok": False,
                                   "error": f"no sim route: {path}"}, 404)

    def _post(self, path: str, raw: bytes,
              full_path: str = "") -> RouteResponse:
        s = self.session
        if path == "/cmd":
            line = raw.decode("utf-8", "ignore").strip()
            ok = s.cmd(line).get("ok")
            return RouteResponse.text("ok" if ok else "failed",
                                      200 if ok else 502)
        data = _json_body(raw)
        if path == "/api/standup":
            return RouteResponse.json(s.sim_standup(
                mode=str(data.get("mode", "tuck")),
                speed=float(data.get("speed", 1.0)),
                direction=str(data.get("direction", "up"))))
        if path == "/api/standup/stop":
            return RouteResponse.json(s.stop_demo())
        if path == "/api/demo":
            return RouteResponse.json(s.run_demo(
                str(data.get("name", "")),
                speed=float(data.get("speed", 1.0)),
                size=float(data.get("size", 1.0)),
                rate=(float(data["rate"]) if data.get("rate") is not None
                      else None),
                torque=(int(float(data["torque"]))
                        if data.get("torque") is not None else None),
                softness=float(data.get("softness", 1.0)),
                seconds=(float(data["seconds"])
                         if data.get("seconds") is not None else None)))
        if path == "/api/calibrate":
            return RouteResponse.json(s.run_calibrate(
                mode=str(data.get("mode", "checkup")),
                step_deg=float(data.get("step_deg", 10)),
                nudge_deg=float(data.get("nudge_deg", 2)),
                axis=str(data.get("axis", "all")),
                clearance_mm=float(data.get("clearance_mm", 40)),
                quad_body_frame=bool(data.get("quad_body_frame", False))))
        if path == "/api/dances":
            script = data
            if isinstance(script, dict) and "script" in script:
                script = script["script"]
            return RouteResponse.json(s.save_dance_script(script))
        if path == "/api/dances/delete":
            return RouteResponse.json(s.delete_dance_script(
                str((data or {}).get("name", ""))))
        if path == "/api/demo/speed":
            return RouteResponse.json(s.set_demo_speed(data.get("speed", 1.0)))
        if path == "/api/demo/stop":
            return RouteResponse.json(s.stop_demo())
        if path == "/api/zero":
            return RouteResponse.json(s.go_zero(
                pose=str(data.get("pose", "sit")),
                force=bool(data.get("force", False))))
        if path == "/api/safe_zero":
            return RouteResponse.json(s.safe_zero())
        if path == "/api/set_zero":
            return RouteResponse.json(s.set_zero_here())
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
                vy=float(data.get("vy", 0.0)),
                wz=float(data.get("wz", 0.0)),
                dh=float(data.get("dh", 0.0))))
        if path == "/api/rl/drive/stop":
            return RouteResponse.json(s.rl_drive_stop())
        if path == "/api/rl/policies":
            return RouteResponse.json(s.save_rl_policy(
                data, name=_query_value(full_path, "name", "")))
        if path == "/api/rl/policies/delete":
            return RouteResponse.json(s.delete_rl_policy(
                str((data or {}).get("file", ""))))
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
        if path == "/api/sim/pose":
            return RouteResponse.json(s.sim_pose(
                degrees=data.get("degrees"),
                source=str(data.get("source", "api"))))
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

    def __init__(self, base_url: str, *, timeout: float = ROBOT_DEFAULT_TIMEOUT_S,
                 ping_timeout: float = ROBOT_PING_TIMEOUT_S,
                 insecure_tls: bool = False):
        self.base_url = _normalize_base_url(base_url)
        self.timeout = timeout
        self.ping_timeout = ping_timeout
        self.ssl_context = None
        self._resolved_base_url = ""
        self._resolved_host_header = ""
        self._resolved_ip = ""
        self._resolved_until = 0.0
        self.configure(self.base_url, insecure_tls=insecure_tls)

    def configure(self, base_url: str,
                  insecure_tls: bool | None = None) -> None:
        self.base_url = _normalize_base_url(base_url)
        self._resolved_base_url = ""
        self._resolved_host_header = ""
        self._resolved_ip = ""
        self._resolved_until = 0.0
        if insecure_tls is not None:
            self.ssl_context = (
                ssl._create_unverified_context() if insecure_tls else None)

    def available(self) -> bool:
        return bool(self.base_url)

    def config_meta(self) -> dict[str, Any]:
        if not self.available():
            return {"available": False}
        out = {"available": True, "url": self.base_url}
        if self._resolved_ip:
            out["resolved_ip"] = self._resolved_ip
            out["resolve_ttl_s"] = max(
                0.0, round(self._resolved_until - time.monotonic(), 1))
        return out

    def ping_meta(self) -> dict[str, Any]:
        if not self.available():
            return {"available": False}
        resp = self.request("GET", "/api/ping", timeout=self.ping_timeout)
        obj = resp.json_obj()
        return {"available": True, "url": self.base_url,
                **obj, "ok": resp.ok()}

    def _connect_base(self) -> tuple[str, str]:
        p = urllib.parse.urlsplit(self.base_url)
        host = p.hostname or ""
        if p.scheme != "http" or not host.endswith(".local"):
            return self.base_url, ""
        now = time.monotonic()
        if self._resolved_base_url and now < self._resolved_until:
            return self._resolved_base_url, self._resolved_host_header
        port = p.port or 80
        infos = socket.getaddrinfo(
            host, port, socket.AF_INET, socket.SOCK_STREAM)
        ip = infos[0][4][0]
        netloc = ip if p.port is None and port == 80 else f"{ip}:{port}"
        self._resolved_base_url = urllib.parse.urlunsplit(
            (p.scheme, netloc, "", "", ""))
        self._resolved_host_header = p.netloc
        self._resolved_ip = ip
        self._resolved_until = now + ROBOT_RESOLVE_TTL_S
        return self._resolved_base_url, self._resolved_host_header

    def _clear_resolved(self) -> None:
        self._resolved_base_url = ""
        self._resolved_host_header = ""
        self._resolved_ip = ""
        self._resolved_until = 0.0

    def request(self, method: str, full_path: str,
                body: bytes = b"", headers: dict[str, str] | None = None,
                timeout: float | None = None) -> RouteResponse:
        if not self.available():
            return RouteResponse.json({"ok": False,
                                       "error": "robot target not configured"},
                                      503)
        req_headers = {}
        if headers:
            for k, v in headers.items():
                if k.lower() in {"content-type", "accept"}:
                    req_headers[k] = v
        data = body if method == "POST" else None
        if data and "Content-Type" not in req_headers:
            req_headers["Content-Type"] = "application/json"
        budget = self.timeout if timeout is None else timeout
        deadline = time.monotonic() + max(0.1, float(budget))
        last_err: Exception | None = None
        for attempt in range(2):
            try:
                connect_base, host_header = self._connect_base()
            except Exception as e:
                last_err = e
                connect_base, host_header = self.base_url, ""
            attempt_headers = dict(req_headers)
            if host_header and "Host" not in attempt_headers:
                attempt_headers["Host"] = host_header
            url = connect_base + full_path
            req = urllib.request.Request(url, data=data, method=method,
                                         headers=attempt_headers)
            try:
                remaining = max(0.1, deadline - time.monotonic())
                with urllib.request.urlopen(
                        req, timeout=remaining,
                        context=self.ssl_context) as r:
                    ctype = r.headers.get("Content-Type",
                                          "application/octet-stream")
                    return RouteResponse(r.status, r.read(), ctype)
            except urllib.error.HTTPError as e:
                ctype = e.headers.get(
                    "Content-Type", "text/plain; charset=utf-8")
                return RouteResponse(e.code, e.read(), ctype)
            except Exception as e:
                last_err = e
                if attempt == 0 and self._resolved_ip:
                    # DHCP can change the robot address. If a cached IPv4
                    # target fails, re-resolve once instead of sitting on a
                    # stale address until the TTL expires.
                    self._clear_resolved()
                    continue
                break
        return RouteResponse.json(
            {"ok": False, "error": f"robot proxy failed: {last_err}"},
            502)


class HubController:
    """Switches and broadcasts between configured robot/sim targets."""

    BROADCAST_POSTS = {
        "/cmd",
        "/api/calibrate",
        "/api/demo",
        "/api/demo/speed",
        "/api/demo/stop",
        "/api/standup",
        "/api/standup/stop",
        "/api/zero",
        "/api/safe_zero",
        "/api/rl/stop",
        "/api/rl/stand",
        "/api/rl/lower",
        "/api/rl/walk",
        "/api/rl/roles",
        "/api/rl/drive/start",
        "/api/rl/drive/cmd",
        "/api/rl/drive/stop",
        "/api/rl/policy_select",
        # Uploads are data, not motion — in "both" mode one POST seeds
        # the same policy/dance onto the sim AND the robot.
        "/api/rl/policies",
        "/api/rl/policies/delete",
        "/api/dances",
        "/api/dances/delete",
    }
    SAFETY_POSTS = {
        "/api/demo/stop",
        "/api/standup/stop",
        "/api/rl/stop",
        "/api/rl/drive/stop",
    }
    ESTOP_CMDS = {"X", "DISARM", "RELAX"}

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

    def configure_robot(self, base_url: str,
                        insecure_tls: bool | None = None) -> None:
        if self.robot is None or not hasattr(self.robot, "configure"):
            self.robot = RobotProxyTarget(
                "", insecure_tls=bool(insecure_tls))
        self.robot.configure(base_url, insecure_tls=insecure_tls)
        with self.lock:
            if not self._has_robot() and self.target in {"robot", "both"}:
                self.target = "sim"

    def _robot_config_meta(self) -> dict[str, Any]:
        if not self._has_robot():
            return {"available": False}
        config_meta = getattr(self.robot, "config_meta", None)
        if config_meta:
            return config_meta()
        return {"available": True}

    def ping(self) -> dict[str, Any]:
        with self.lock:
            target = self.target
        sim_meta = self.sim.ping_meta() if self._has_sim() else {
            "available": False}
        active_robot = target in {"robot", "both"}
        active_sim = target in {"sim", "both"}
        robot_meta = self._robot_config_meta()
        if active_robot and self._has_robot():
            robot_resp = self._request_with_timeout(
                "robot", "GET", "/api/ping", b"", None,
                timeout=ROBOT_PING_TIMEOUT_S)
            robot_obj = robot_resp.json_obj()
            robot_meta = {
                **robot_meta,
                **robot_obj,
                "ok": robot_resp.ok() and robot_obj.get("ok") is not False,
            }
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
        if path == "/api/demos":
            return self._demo_catalog(headers)
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
                if "robot_url" in data:
                    insecure = (
                        bool(data["robot_insecure_tls"])
                        if "robot_insecure_tls" in data else None)
                    self.configure_robot(str(data.get("robot_url", "")),
                                         insecure_tls=insecure)
                target = str(data.get("target", "")).strip()
                if target:
                    self.set_target(target)
            except Exception as e:
                return RouteResponse.json({"ok": False, "error": str(e)}, 400)
            return RouteResponse.json({"ok": True, **self.ping()})
        if path == "/api/sim/sync_robot_pose":
            return self._sync_sim_from_robot_pose(headers)
        if path.startswith("/api/sim/"):
            return self._send("sim", "POST", full_path, body, headers)
        with self.lock:
            target = self.target
        if self._must_broadcast_safety(path, body, headers):
            return self._broadcast_post(full_path, body, headers)
        if target == "both" and path in self.BROADCAST_POSTS:
            return self._broadcast_post(full_path, body, headers)
        return self._send(target, "POST", full_path, body, headers)

    def _must_broadcast_safety(
            self, path: str, body: bytes,
            headers: dict[str, str] | None) -> bool:
        if path in self.SAFETY_POSTS:
            return self._has_sim() and self._has_robot()
        global_stop = _header_value(
            headers, "X-Hexapod-Global-Stop").lower() in {
                "1", "true", "yes"}
        if path == "/cmd" and global_stop and _cmd_line(body) in self.ESTOP_CMDS:
            return self._has_sim() and self._has_robot()
        return False

    @staticmethod
    def _robot_route_timeout(method: str, full_path: str) -> float:
        path = full_path.split("?", 1)[0]
        if method == "POST":
            return ROBOT_ROUTE_TIMEOUTS_S.get(path, ROBOT_DEFAULT_TIMEOUT_S)
        return ROBOT_DEFAULT_TIMEOUT_S

    def _request_with_timeout(self, target: str, method: str, full_path: str,
                              body: bytes,
                              headers: dict[str, str] | None,
                              timeout: float | None = None) -> RouteResponse:
        if target != "robot" or timeout is None:
            return self._send(target, method, full_path, body, headers)
        if not self._has_robot():
            return RouteResponse.json({"ok": False,
                                       "error": "robot target unavailable"},
                                      503)
        q: queue.Queue[RouteResponse | BaseException] = queue.Queue(maxsize=1)

        def work() -> None:
            try:
                try:
                    resp = self.robot.request(method, full_path, body, headers,
                                              timeout=timeout)
                except TypeError:
                    resp = self.robot.request(method, full_path, body,
                                              headers)
                q.put(resp)
            except BaseException as e:
                q.put(e)

        threading.Thread(target=work, daemon=True).start()
        try:
            out = q.get(timeout=timeout)
        except queue.Empty:
            return RouteResponse.json({
                "ok": False,
                "error": f"robot proxy timed out after {timeout:.1f}s",
            }, 504)
        if isinstance(out, BaseException):
            return RouteResponse.json({"ok": False, "error": str(out)}, 502)
        return out

    def _demo_catalog(self, headers: dict[str, str] | None) -> RouteResponse:
        rows: dict[str, dict[str, Any]] = {}
        sources: dict[str, dict[str, Any]] = {}

        def merge_items(target: str, demos: list[Any]) -> None:
            for item in demos:
                if not isinstance(item, dict) or not item.get("name"):
                    continue
                row = dict(item)
                row.setdefault("target", target)
                row["available_on"] = sorted({
                    *rows.get(row["name"], {}).get("available_on", []),
                    target,
                })
                # Prefer robot metadata for duplicate names so the full
                # hardware catalog wins over the sim's narrow subset.
                if target == "robot" or row["name"] not in rows:
                    rows[row["name"]] = row
                else:
                    rows[row["name"]]["available_on"] = row["available_on"]

        def add_from(target: str, timeout: float | None = None,
                     optional: bool = False) -> None:
            if target == "sim" and not self._has_sim():
                sources[target] = {"ok": False, "error": "sim unavailable"}
                return
            if target == "robot" and not self._has_robot():
                sources[target] = {"ok": False, "error": "robot unavailable"}
                return
            resp = self._request_with_timeout(
                target, "GET", "/api/demos", b"", headers, timeout=timeout)
            obj = resp.json_obj()
            prev_source = sources.get(target)
            live_source = {
                "ok": resp.ok(),
                "status": resp.code,
                "error": obj.get("error"),
            }
            if optional and prev_source:
                # Keep an already-useful local catalog source green; expose the
                # live failure as diagnostic detail instead of hiding the demos.
                if prev_source.get("ok") and not resp.ok():
                    prev_source["live_status"] = resp.code
                    prev_source["live_error"] = obj.get("error")
                    return
            sources[target] = live_source
            if resp.ok():
                merge_items(target, obj.get("demos", []))

        def add_local_robot() -> None:
            demos, err = _local_robot_demos()
            if demos:
                sources["robot"] = {
                    "ok": True,
                    "status": "local",
                    "local": True,
                    "configured": self._has_robot(),
                }
                merge_items("robot", demos)
                return
            sources["robot"] = {
                "ok": False,
                "status": "local",
                "error": err or "local robot catalog unavailable",
            }

        add_from("sim")
        add_local_robot()
        if self._has_robot():
            add_from("robot", timeout=ROBOT_CATALOG_TIMEOUT_S, optional=True)
        return RouteResponse.json({
            "ok": any(s.get("ok") for s in sources.values()),
            "hub": True,
            "target": self.target,
            "sources": sources,
            "demos": list(rows.values()),
        })

    def _sync_sim_from_robot_pose(
            self, headers: dict[str, str] | None) -> RouteResponse:
        if not self._has_robot():
            return RouteResponse.json({"ok": False,
                                       "error": "robot target unavailable"},
                                      503)
        if not self._has_sim():
            return RouteResponse.json({"ok": False,
                                       "error": "sim target unavailable"},
                                      503)
        robot_resp = self._request_with_timeout(
            "robot", "GET", "/api/pose", b"", headers, timeout=5.0)
        pose = robot_resp.json_obj()
        if not robot_resp.ok() or pose.get("ok") is False:
            return RouteResponse.json({
                "ok": False,
                "error": pose.get("error") or "robot pose unavailable",
                "robot": pose,
            }, 502)
        degrees = pose.get("degrees")
        if not isinstance(degrees, list) or len(degrees) != 18:
            return RouteResponse.json({
                "ok": False,
                "error": "robot pose did not include 18 joint degrees",
                "robot": pose,
            }, 502)
        missing = [i for i, v in enumerate(degrees) if v is None]
        if missing:
            return RouteResponse.json({
                "ok": False,
                "error": ("robot pose missing joints "
                          + ",".join(str(i) for i in missing)),
                "robot": pose,
            }, 502)
        sim_body = json.dumps({
            "degrees": degrees,
            "source": "robot",
        }).encode("utf-8")
        sim_resp = self._send("sim", "POST", "/api/sim/pose", sim_body,
                              {"Content-Type": "application/json"})
        sim_obj = sim_resp.json_obj()
        out = dict(sim_obj)
        out["hub"] = True
        out["robot_pose"] = {
            "live": pose.get("live"),
            "ts": pose.get("ts"),
            "armed": pose.get("armed"),
            "mode": pose.get("mode"),
        }
        return RouteResponse.json(out, 200 if sim_resp.ok() else 502)

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
        return self.robot.request(
            method, full_path, body, headers,
            timeout=self._robot_route_timeout(method, full_path))

    def _broadcast_post(self, full_path: str, body: bytes,
                        headers: dict[str, str] | None) -> RouteResponse:
        q: queue.Queue[tuple[str, RouteResponse]] = queue.Queue(maxsize=2)

        def send_robot() -> None:
            try:
                resp = self._request_with_timeout(
                    "robot", "POST", full_path, body, headers,
                    timeout=self._robot_route_timeout("POST", full_path))
            except BaseException as e:
                resp = RouteResponse.json({"ok": False, "error": str(e)},
                                          502)
            q.put(("robot", resp))

        def send_sim() -> None:
            try:
                resp = self._send("sim", "POST", full_path, body, headers)
            except BaseException as e:
                resp = RouteResponse.json({"ok": False, "error": str(e)},
                                          502)
            q.put(("sim", resp))

        threading.Thread(target=send_robot, daemon=True).start()
        threading.Thread(target=send_sim, daemon=True).start()
        results = {name: resp for name, resp in (q.get(), q.get())}
        robot_resp = results["robot"]
        sim_resp = results["sim"]
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
