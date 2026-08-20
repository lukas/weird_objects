from __future__ import annotations

import json
from io import BytesIO
import urllib.error

from rl_move.sim.web_server import WEBUI_DIR, make_handler


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


def test_unknown_route_returns_json_404():
    fake = FakeSession()
    try:
        _json(fake, "/nope")
    except urllib.error.HTTPError as e:
        assert e.code == 404
        body = json.loads(e.read().decode())
        assert body["ok"] is False
