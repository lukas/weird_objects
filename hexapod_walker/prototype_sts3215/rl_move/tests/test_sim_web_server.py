from __future__ import annotations

import json
from io import BytesIO
import urllib.error

from rl_move.sim.web_hub import HubController, RouteResponse, make_hub_handler
from rl_move.sim.web_server import PAGE_PATHS, STATIC_FILES, WEBUI_DIR, make_handler


class FakeSession:
    def __init__(self):
        self.calls = []

    def ping(self):
        return {"ok": True, "service": "hexapod-sim", "kind": "sim"}

    def robot_state(self):
        return {"ok": True, "activity": "idle", "sim": True}

    def status(self):
        return {"ok": True, "sim": True, "motors": []}

    def operation_state(self):
        return {"ok": True, "running": False, "result": {"ok": True}}

    def list_demos(self):
        return [{"name": "quad_walk", "group": "quad"}]

    def demo_state(self):
        return {"name": "quad_walk", "status": "idle", "running": False}

    def rl_preflight(self, mode="stand"):
        return {"ok": True, "mode": mode, "sim": True}

    def rl_policy_info(self):
        return {"ok": True, "source": "stance.zip", "obs_dim": 68,
                "hidden": [256, 256], "act_dim": 18, "activation": "Tanh",
                "walk": {"source": "walk.zip", "obs_dim": 72,
                         "hidden": [256, 256], "act_dim": 18,
                         "activation": "Tanh"}}

    def rl_policies(self):
        return {"ok": True, "policies": []}

    def rl_roles(self):
        return {"ok": True, "roles": {}, "allowed_obs": {}}

    def rl_drive_state(self):
        return {"ok": True, "active": False}

    def sim_state(self):
        return {"ok": True, "live": {"mode": "hold"}}

    def frame_jpeg(self):
        return b"\xff\xd8\xff\xd9"

    def logs(self):
        return {"ok": True, "files": []}

    def log_file(self, name, request_path=""):
        return b"t_s,mode\n", "text/csv; charset=utf-8"

    def cmd(self, line):
        self.calls.append(("cmd", line))
        return {"ok": True}

    def sim_reset(self, start="plant"):
        self.calls.append(("sim_reset", start))
        return {"ok": True, "status": start}

    def run_demo(self, name, **kw):
        self.calls.append(("run_demo", name, kw))
        return {"ok": True, "params": kw, "home": "stand",
                "demo": self.demo_state(), "robot": self.robot_state()}

    def set_demo_speed(self, speed):
        self.calls.append(("set_demo_speed", speed))
        return {"ok": True, "speed": speed, "demo": self.demo_state()}

    def stop_demo(self):
        self.calls.append(("stop_demo",))
        return {"ok": True, "demo": self.demo_state(),
                "robot": self.robot_state()}

    def go_zero(self, pose="sit", force=False):
        self.calls.append(("go_zero", pose, force))
        return {"ok": True, "pose": pose, "demo": self.demo_state(),
                "robot": self.robot_state()}

    def safe_zero(self):
        self.calls.append(("safe_zero",))
        return {"ok": True}

    def set_zero_here(self):
        self.calls.append(("set_zero_here",))
        return {"ok": True}

    def sim_fall(self):
        self.calls.append(("sim_fall",))
        return {"ok": True}

    def sim_recover(self):
        self.calls.append(("sim_recover",))
        return {"ok": True}

    def sim_push(self, x=4.0, y=0.0):
        self.calls.append(("sim_push", x, y))
        return {"ok": True}

    def rl_capture_plant(self):
        return {"ok": True}

    def rl_stop(self):
        return {"ok": True}

    def rl_policy_move(self, **kw):
        self.calls.append(("rl_policy_move", kw))
        return {"ok": True, **kw}

    def rl_role_set(self, role, file):
        self.calls.append(("rl_role_set", role, file))
        return {"ok": True}

    def rl_drive_start(self):
        self.calls.append(("rl_drive_start",))
        return {"ok": True, "active": True}

    def rl_drive_cmd(self, vx, vy):
        self.calls.append(("rl_drive_cmd", vx, vy))
        return {"ok": True, "active": True}

    def rl_drive_stop(self):
        self.calls.append(("rl_drive_stop",))
        return {"ok": True, "active": False}

    def rl_policy_select(self, file):
        self.calls.append(("rl_policy_select", file))
        return {"ok": True, "name": file}


def _request(fake, path, method="GET", body=None):
    handler_cls = make_handler(fake, webui_dir=WEBUI_DIR)
    h = handler_cls.__new__(handler_cls)
    data = json.dumps(body).encode() if body is not None else b""
    h.path = path
    h.command = method
    h.headers = {"Content-Length": str(len(data))}
    h.rfile = BytesIO(data)
    h.wfile = BytesIO()
    h._headers = {}
    h.send_response = lambda code: setattr(h, "_code", code)
    h.send_header = lambda k, v: h._headers.__setitem__(k, v)
    h.end_headers = lambda: None
    if method == "POST":
        handler_cls.do_POST(h)
    else:
        handler_cls.do_GET(h)
    return h._code, h._headers, h.wfile.getvalue()


def _json(fake, path, method="GET", body=None):
    code, _headers, payload = _request(fake, path, method=method, body=body)
    if code >= 400:
        raise urllib.error.HTTPError(path, code, "error", {}, BytesIO(payload))
    return json.loads(payload.decode())


def test_serves_shared_webui_and_sim_ping():
    fake = FakeSession()
    code, headers, payload = _request(fake, "/rl")
    assert code == 200
    assert "text/html" in headers["Content-Type"]
    html = payload.decode()
    assert "Hexapod STS3215" in html
    assert "__HTTPS_PORT__" not in html
    assert _json(fake, "/api/ping")["service"] == "hexapod-sim"


def test_dispatches_rl_drive_and_sim_routes():
    fake = FakeSession()
    assert _json(fake, "/api/rl/drive/start", method="POST")["active"]
    assert _json(fake, "/api/rl/drive/cmd", method="POST",
                 body={"vx": 0.05, "vy": -0.02})["active"]
    assert _json(fake, "/api/sim/reset", method="POST",
                 body={"start": "belly"})["status"] == "belly"
    assert ("rl_drive_cmd", 0.05, -0.02) in fake.calls
    assert ("sim_reset", "belly") in fake.calls


def test_dispatches_robot_compatible_demo_routes():
    fake = FakeSession()
    demos = _json(fake, "/api/demos")
    assert demos["demos"][0]["name"] == "quad_walk"

    started = _json(fake, "/api/demo", method="POST",
                    body={"name": "quad_walk", "speed": 1.25,
                          "seconds": 40})
    assert started["ok"] is True
    assert started["home"] == "stand"
    assert ("run_demo", "quad_walk",
            {"speed": 1.25, "size": 1.0, "softness": 1.0,
             "seconds": 40.0}) in fake.calls

    assert _json(fake, "/api/demo/speed", method="POST",
                 body={"speed": 0.75})["speed"] == 0.75
    assert _json(fake, "/api/demo/stop", method="POST")["ok"] is True
    assert _json(fake, "/api/zero", method="POST",
                 body={"pose": "stand"})["pose"] == "stand"


def test_unknown_route_returns_json_404():
    fake = FakeSession()
    try:
        _json(fake, "/nope")
    except urllib.error.HTTPError as e:
        assert e.code == 404
        body = json.loads(e.read().decode())
        assert body["ok"] is False


class FakeTarget:
    def __init__(self, name):
        self.name = name
        self.calls = []

    def available(self):
        return True

    def ping_meta(self):
        return {"available": True, "ok": True, "name": self.name}

    def request(self, method, full_path, body=b"", headers=None):
        self.calls.append((method, full_path, body))
        path = full_path.split("?", 1)[0]
        if path == "/cmd":
            return RouteResponse.text("ok")
        try:
            data = json.loads(body.decode()) if body else {}
        except ValueError:
            data = body.decode("utf-8", "replace")
        return RouteResponse.json({
            "ok": True,
            "target": self.name,
            "method": method,
            "path": path,
            "body": data,
        })

    def close(self):
        pass


class ConfigurableFakeTarget(FakeTarget):
    def __init__(self, name):
        super().__init__(name)
        self.base_url = ""

    def available(self):
        return bool(self.base_url)

    def configure(self, base_url, insecure_tls=None):
        self.base_url = (
            base_url if "://" in base_url else "http://" + base_url
        ).rstrip("/")

    def config_meta(self):
        if not self.available():
            return {"available": False}
        return {"available": True, "url": self.base_url}

    def ping_meta(self):
        return {**self.config_meta(), "ok": self.available(),
                "name": self.name}


def _hub_request(hub, path, method="GET", body=None):
    handler_cls = make_hub_handler(
        hub, WEBUI_DIR, 8443, PAGE_PATHS, STATIC_FILES)
    h = handler_cls.__new__(handler_cls)
    data = json.dumps(body).encode() if body is not None else b""
    h.path = path
    h.command = method
    h.headers = {"Content-Length": str(len(data))}
    h.rfile = BytesIO(data)
    h.wfile = BytesIO()
    h._headers = {}
    h.send_response = lambda code: setattr(h, "_code", code)
    h.send_header = lambda k, v: h._headers.__setitem__(k, v)
    h.end_headers = lambda: None
    if method == "POST":
        handler_cls.do_POST(h)
    else:
        handler_cls.do_GET(h)
    return h._code, h._headers, h.wfile.getvalue()


def _hub_json(hub, path, method="GET", body=None):
    code, _headers, payload = _hub_request(
        hub, path, method=method, body=body)
    if code >= 400:
        raise urllib.error.HTTPError(path, code, "error", {}, BytesIO(payload))
    return json.loads(payload.decode())


def test_hub_ping_and_target_switch():
    sim = FakeTarget("sim")
    robot = FakeTarget("robot")
    hub = HubController(sim=sim, robot=robot, target="both")
    ping = _hub_json(hub, "/api/ping")
    assert ping["service"] == "hexapod-hub"
    assert ping["target"] == "both"
    assert ping["active"] == {"robot": True, "sim": True}

    switched = _hub_json(hub, "/api/hub", method="POST",
                         body={"target": "sim"})
    assert switched["target"] == "sim"
    assert switched["active"] == {"robot": False, "sim": True}


def test_hub_can_configure_robot_target_at_runtime():
    sim = FakeTarget("sim")
    robot = ConfigurableFakeTarget("robot")
    hub = HubController(sim=sim, robot=robot, target="sim")
    ping = _hub_json(hub, "/api/ping")
    assert ping["target"] == "sim"
    assert ping["targets"]["robot"] == {"available": False}

    connected = _hub_json(hub, "/api/hub", method="POST",
                          body={"robot_url": "hexapod.local:8080",
                                "target": "robot"})
    assert connected["ok"] is True
    assert connected["target"] == "robot"
    assert connected["active"] == {"robot": True, "sim": False}
    assert connected["targets"]["robot"]["url"] == "http://hexapod.local:8080"

    r = _hub_json(hub, "/api/status")
    assert r["target"] == "robot"
    assert robot.calls[-1][1] == "/api/status"


def test_hub_broadcasts_drive_commands_only_in_both_mode():
    sim = FakeTarget("sim")
    robot = FakeTarget("robot")
    hub = HubController(sim=sim, robot=robot, target="both")
    d = _hub_json(hub, "/api/rl/drive/cmd", method="POST",
                  body={"vx": 0.04, "vy": 0.0})
    assert d["ok"] is True
    assert d["hub"]["robot"]["body"]["vx"] == 0.04
    assert d["hub"]["sim"]["body"]["vx"] == 0.04
    assert ("POST", "/api/rl/drive/cmd", b'{"vx": 0.04, "vy": 0.0}') in robot.calls
    assert ("POST", "/api/rl/drive/cmd", b'{"vx": 0.04, "vy": 0.0}') in sim.calls

    _hub_json(hub, "/api/hub", method="POST", body={"target": "robot"})
    _hub_json(hub, "/api/wiggle", method="POST",
              body={"joint": 1, "amp": 4})
    assert robot.calls[-1][1] == "/api/wiggle"
    assert sim.calls[-1][1] == "/api/rl/drive/cmd"


def test_hub_broadcasts_demo_commands_in_both_mode():
    sim = FakeTarget("sim")
    robot = FakeTarget("robot")
    hub = HubController(sim=sim, robot=robot, target="both")
    d = _hub_json(hub, "/api/demo", method="POST",
                  body={"name": "quad_walk", "speed": 1.0})
    assert d["ok"] is True
    assert d["hub"]["robot"]["body"]["name"] == "quad_walk"
    assert d["hub"]["sim"]["body"]["name"] == "quad_walk"
    assert ("POST", "/api/demo", b'{"name": "quad_walk", "speed": 1.0}') in robot.calls
    assert ("POST", "/api/demo", b'{"name": "quad_walk", "speed": 1.0}') in sim.calls
