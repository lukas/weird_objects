#!/usr/bin/env python3
"""Receive timestamped hexapod events on this laptop (auto-discovery).

Listens for UDP JSONL on :9377 and **beacons** this machine's address to the
robot on :9378 so you never have to set ``HEXAPOD_LOG_HOST``.

    cd hexapod_walker/prototype_sts3215/linux_control
    uv run python receive_robot_logs.py

    # backup if UDP is firewalled:
    uv run python receive_robot_logs.py --ssh arduino@hexapod.local

Writes ``logs/robot_events.jsonl`` next to this script (or --out).
"""
from __future__ import annotations

import argparse
import json
import os
import select
import socket
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_PORT = int(os.environ.get("HEXAPOD_LOG_PORT", "9377"))
DEFAULT_BEACON_PORT = int(os.environ.get("HEXAPOD_LOG_BEACON_PORT", "9378"))
DEFAULT_OUT = HERE / "logs" / "robot_events.jsonl"
DEFAULT_BOARD = os.environ.get("HEXAPOD_BOARD", "hexapod.local")


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def _local_ipv4s() -> list[str]:
    found: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                found.add(ip)
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((DEFAULT_BOARD, 80))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith("127."):
            found.add(ip)
    except OSError:
        pass
    # macOS: en0 / en1
    for iface in ("en0", "en1", "eth0", "wlan0"):
        try:
            out = subprocess.check_output(
                ["ipconfig", "getifaddr", iface],
                text=True, stderr=subprocess.DEVNULL, timeout=1,
            ).strip()
            if out and not out.startswith("127."):
                found.add(out)
        except Exception:
            pass
    return sorted(found)


def _fmt(ev: dict) -> str:
    ts = ev.get("ts") or _utc()
    kind = ev.get("kind", "?")
    src = ev.get("src", "")
    msg = ev.get("msg", "")
    data = ev.get("data")
    extra = ""
    if isinstance(data, dict) and data:
        bits = []
        for k in ("mode", "cmd", "reply", "name", "path", "grade", "saved",
                  "ax_g", "az_g", "current_a", "method", "host"):
            if k in data:
                bits.append(f"{k}={data[k]}")
        if not bits:
            keys = list(data.keys())[:4]
            bits = [f"{k}={data[k]}" for k in keys]
        extra = "  " + " ".join(str(b) for b in bits)
        if len(extra) > 160:
            extra = extra[:157] + "…"
    return f"{ts}  {kind:<7} {src:<6} {msg}{extra}"


def _write(fh, line: str, raw: dict | None) -> None:
    fh.write(line if line.endswith("\n") else line + "\n")
    fh.flush()
    try:
        ev = raw if raw is not None else json.loads(line)
    except Exception:
        ev = {"ts": _utc(), "kind": "raw", "msg": line.strip()}
    print(_fmt(ev), flush=True)


def _beacon_loop(event_port: int, beacon_port: int, board: str,
                 stop: threading.Event) -> None:
    """Tell the robot where to unicast events (and refresh periodically)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while not stop.is_set():
        ips = _local_ipv4s()
        payload = json.dumps({
            "kind": "log_sink",
            "port": event_port,
            "ips": ips,
            "ts": _utc(),
        }).encode("utf-8")
        dests = {
            (board, beacon_port),
            ("255.255.255.255", beacon_port),
        }
        for ip in ips:
            parts = ip.split(".")
            if len(parts) == 4:
                dests.add((".".join(parts[:3] + ["255"]), beacon_port))
        for dest in dests:
            try:
                sock.sendto(payload, dest)
            except OSError:
                pass
        stop.wait(2.0)
    sock.close()


def listen_udp(port: int, out: Path, *, board: str,
               beacon_port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.setblocking(False)
    out.parent.mkdir(parents=True, exist_ok=True)

    stop = threading.Event()
    beacons = threading.Thread(
        target=_beacon_loop, args=(port, beacon_port, board, stop),
        name="log-beacon-tx", daemon=True,
    )
    beacons.start()

    ips = _local_ipv4s()
    print(f"[recv] UDP :{port} → {out}", flush=True)
    print(f"[recv] beacons → {board}:{beacon_port} + broadcast "
          f"(this host ips={ips or '?'})", flush=True)
    print("[recv] no HEXAPOD_LOG_HOST needed — robot auto-learns this laptop",
          flush=True)

    with out.open("a", encoding="utf-8", buffering=1) as fh:
        buf = b""
        try:
            while True:
                r, _, _ = select.select([sock], [], [], 1.0)
                if not r:
                    continue
                data, _addr = sock.recvfrom(65535)
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    text = line.decode("utf-8", errors="replace")
                    try:
                        ev = json.loads(text)
                    except Exception:
                        ev = None
                    # Ignore our own beacons if they ever loop back.
                    if isinstance(ev, dict) and ev.get("kind") == "log_sink":
                        continue
                    _write(fh, text, ev)
        except KeyboardInterrupt:
            print("\n[recv] stop", flush=True)
        finally:
            stop.set()


def ssh_tail(board: str, out: Path, password: str = "arduino") -> None:
    remote = "~/hexapod_sts/linux_control/logs/events.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"[recv] SSH-tail {board}:{remote} → {out}", flush=True)
    cmd = [
        "sshpass", "-p", password,
        "ssh", "-o", "PreferredAuthentications=password",
        "-o", "PubkeyAuthentication=no",
        "-o", "StrictHostKeyChecking=accept-new",
        board,
        f"mkdir -p $(dirname {remote}); touch {remote}; tail -n 50 -F {remote}",
    ]
    if subprocess.call(["which", "sshpass"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL) != 0:
        cmd = ["ssh", "-o", "StrictHostKeyChecking=accept-new", board,
               f"mkdir -p $(dirname {remote}); touch {remote}; "
               f"tail -n 50 -F {remote}"]
    with out.open("a", encoding="utf-8", buffering=1) as fh:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1)
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                if not line.strip():
                    continue
                try:
                    ev = json.loads(line)
                except Exception:
                    ev = None
                _write(fh, line, ev)
        finally:
            proc.kill()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--beacon-port", type=int, default=DEFAULT_BEACON_PORT)
    ap.add_argument("--board", default=DEFAULT_BOARD,
                    help="robot IP/hostname for unicast beacons")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--ssh", nargs="?", const=DEFAULT_BOARD,
                    default=None,
                    help="also/instead SSH-tail events.jsonl on the board")
    ap.add_argument("--ssh-only", action="store_true",
                    help="skip UDP; only SSH-tail")
    args = ap.parse_args()

    if args.ssh_only:
        ssh_tail(args.ssh or args.board, args.out)
        return 0

    if args.ssh:
        t = threading.Thread(
            target=ssh_tail,
            args=(args.ssh, args.out.with_name(args.out.stem + "_ssh.jsonl")),
            daemon=True,
        )
        t.start()

    listen_udp(args.port, args.out, board=args.board,
               beacon_port=args.beacon_port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
