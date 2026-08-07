"""HTTP client for hexapod-web — prefer this over SSH.

Default base: ``http://192.168.4.44:8080`` (override with ``HEXAPOD_URL``).

Examples
--------
::

    from rl_move.remote import HexapodClient
    c = HexapodClient()
    print(c.state())
    c.find_plant(clearance_mm=40)   # async — poll until done
    c.wait_idle()
    print(c.plant())
"""
from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from typing import Any


DEFAULT_BASE = os.environ.get("HEXAPOD_URL", "http://192.168.4.44:8080")


class HexapodClient:
    def __init__(self, base: str | None = None, *, timeout: float = 10.0):
        self.base = (base or DEFAULT_BASE).rstrip("/")
        self.timeout = float(timeout)

    def _req(self, method: str, path: str, body: dict | None = None) -> dict:
        url = f"{self.base}{path}"
        data = None
        headers = {"Accept": "application/json"}
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read().decode("utf-8")
        except urllib.error.HTTPError as e:
            raw = e.read().decode("utf-8", "ignore")
            try:
                return json.loads(raw)
            except Exception:
                return {"ok": False, "error": f"HTTP {e.code}: {raw[:200]}"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
        try:
            return json.loads(raw) if raw else {"ok": True}
        except json.JSONDecodeError:
            return {"ok": False, "error": f"non-JSON: {raw[:200]}"}

    def ping(self) -> dict:
        return self._req("GET", "/api/ping")

    def state(self) -> dict:
        return self._req("GET", "/api/rl/state")

    def pose(self) -> dict:
        return self._req("GET", "/api/pose")

    def plant(self) -> dict:
        return self._req("GET", "/api/plant")

    def find_plant(self, *, clearance_mm: float = 40.0) -> dict:
        return self._req("POST", "/api/rl/find_plant",
                         {"clearance_mm": clearance_mm})

    def capture_plant(self) -> dict:
        return self._req("POST", "/api/rl/capture_plant", {})

    def stop(self) -> dict:
        return self._req("POST", "/api/rl/stop", {})

    def set_stance(self, *, hip_deg: float = -20.0, knee_deg: float = 55.0,
                   seconds: float = 10.0, yaw_deg: float = 0.0,
                   force: bool = False) -> dict:
        return self._req("POST", "/api/rl/set_stance", {
            "hip_deg": hip_deg, "knee_deg": knee_deg,
            "seconds": seconds, "yaw_deg": yaw_deg, "force": force,
        })

    def probe_dynamics(self, *, amp_deg: float = 10.0, axis: str = "all",
                       soft_torque: int = 450) -> dict:
        return self._req("POST", "/api/rl/probe_dynamics", {
            "amp_deg": amp_deg, "axis": axis, "soft_torque": soft_torque,
        })

    def wait_idle(self, *, timeout_s: float = 120.0,
                  poll_s: float = 0.5) -> dict:
        """Poll ``/api/rl/state`` until calibrate/demo worker is idle."""
        t0 = time.monotonic()
        last: dict[str, Any] = {}
        while time.monotonic() - t0 < timeout_s:
            last = self.state()
            cal = last.get("calibrate") or {}
            robot = last.get("robot") or {}
            demo = robot.get("demo") or cal.get("demo") or {}
            running = bool(cal.get("running") or demo.get("running"))
            if not running:
                # Prefer calibrate result when present (dynamics stores there).
                if cal.get("result") is not None:
                    last["result"] = cal["result"]
                return last
            time.sleep(poll_s)
        last = dict(last)
        last["ok"] = False
        last["error"] = f"timeout after {timeout_s:.0f}s"
        return last


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default=None)
    ap.add_argument("cmd", nargs="?", default="state",
                    choices=("ping", "state", "pose", "plant",
                             "find_plant", "capture_plant", "set_stance",
                             "probe_dynamics", "stop", "wait"))
    ap.add_argument("--clearance-mm", type=float, default=40.0)
    ap.add_argument("--hip", type=float, default=-20.0)
    ap.add_argument("--knee", type=float, default=55.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--amp", type=float, default=10.0)
    ap.add_argument("--axis", default="all")
    ap.add_argument("--soft-torque", type=int, default=450)
    args = ap.parse_args()
    c = HexapodClient(args.url)
    if args.cmd == "ping":
        out = c.ping()
    elif args.cmd == "state":
        out = c.state()
    elif args.cmd == "pose":
        out = c.pose()
    elif args.cmd == "plant":
        out = c.plant()
    elif args.cmd == "find_plant":
        out = c.find_plant(clearance_mm=args.clearance_mm)
        print(json.dumps(out, indent=2))
        out = c.wait_idle(timeout_s=180)
    elif args.cmd == "capture_plant":
        out = c.capture_plant()
    elif args.cmd == "set_stance":
        out = c.set_stance(hip_deg=args.hip, knee_deg=args.knee,
                           seconds=args.seconds)
        print(json.dumps(out, indent=2))
        out = c.wait_idle(timeout_s=90)
    elif args.cmd == "probe_dynamics":
        out = c.probe_dynamics(amp_deg=args.amp, axis=args.axis,
                               soft_torque=args.soft_torque)
        print(json.dumps(out, indent=2))
        out = c.wait_idle(timeout_s=600)
    elif args.cmd == "stop":
        out = c.stop()
    else:
        out = c.wait_idle()
    print(json.dumps(out, indent=2))
    return 0 if out.get("ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
