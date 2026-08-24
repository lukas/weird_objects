#!/usr/bin/env python3
"""Timestamped hexapod event log + auto UDP stream to laptop(s).

Every notable command / log line becomes one JSON object:

    {"ts":"2026-08-06T17:50:01.123Z","mono":1234.567,"kind":"cmd",
     "src":"http","msg":"POST /api/demo","data":{...}}

Streaming (no manual laptop IP)
-------------------------------
* Always writes ``logs/events.jsonl`` on the robot.
* UDP-fanouts each event to:
  1. subnet broadcast(s) ``x.x.x.255:9377`` (auto from the robot's NICs)
  2. any laptop that beacons ``kind=log_sink`` on port **9378**
  3. optional ``HEXAPOD_LOG_HOST`` override if you still want one

Laptop: ``uv run python receive_robot_logs.py`` — listens on 9377 and beacons
its presence so the robot learns the unicast address automatically.
"""
from __future__ import annotations

import json
import os
import queue
import socket
import struct
import threading
import time
import atexit
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_HERE = Path(__file__).resolve().parent
DEFAULT_LOG_DIR = _HERE / "logs"
DEFAULT_UDP_PORT = 9377
DEFAULT_BEACON_PORT = 9378


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, "") or default)
    except ValueError:
        return default

# MCU line prefixes that are too chatty for the command stream (still
# available if HEXAPOD_LOG_MCU=all).
MCU_SKIP_PREFIXES = (
    "R1 ", "R2 ", "RP ", "IMUR", "PWR", "DX ", "PING ",
)

_lock = threading.Lock()
_fh = None
_err_fh = None
_path: Path | None = None
_sock: socket.socket | None = None
_targets: set[tuple[str, int]] = set()   # unicast + broadcast destinations
_seq = 0
_print_hooked = False
_orig_print = print
_mcu_mode = "cmds"  # cmds | all | off
_ring: list[dict] = []
_RING_MAX = 500
_QUEUE_MAX = max(100, _env_int("HEXAPOD_LOG_QUEUE", 1000))
_q: queue.Queue[tuple[str, bytes, bool]] = queue.Queue(maxsize=_QUEUE_MAX)
_worker_thread: threading.Thread | None = None
_worker_stop = threading.Event()
_dropped_events = 0
_last_worker_refresh = 0.0
_beacon_thread: threading.Thread | None = None
_beacon_stop = threading.Event()
_configured = False


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def log_dir() -> Path:
    env = os.environ.get("HEXAPOD_LOG_DIR")
    if env:
        return Path(env)
    return DEFAULT_LOG_DIR


def events_path() -> Path:
    return log_dir() / "events.jsonl"


def errors_path() -> Path:
    """Errors-only sidecar log (every level="error" event also lands here)."""
    return log_dir() / "errors.jsonl"


def _local_ipv4s() -> list[str]:
    """Best-effort list of non-loopback IPv4 addresses on this host."""
    found: set[str] = set()
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                found.add(ip)
    except OSError:
        pass
    # Also poke a UDP connect to learn the default-route address.
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith("127."):
            found.add(ip)
    except OSError:
        pass
    # Linux: parse ``ip -4 -o addr``
    try:
        import subprocess
        out = subprocess.check_output(
            ["ip", "-4", "-o", "addr", "show", "up"],
            text=True, timeout=2,
        )
        for line in out.splitlines():
            parts = line.split()
            if "inet" in parts:
                i = parts.index("inet")
                cidr = parts[i + 1]
                ip = cidr.split("/")[0]
                if not ip.startswith("127."):
                    found.add(ip)
    except Exception:
        pass
    return sorted(found)


def _broadcast_for(ip: str) -> str | None:
    """Assume /24 for LAN discovery (Uno Q / home Wi-Fi)."""
    parts = ip.split(".")
    if len(parts) != 4:
        return None
    if ip.startswith("127.") or ip.startswith("169.254."):
        return None
    return ".".join(parts[:3] + ["255"])


def _ensure_sock() -> socket.socket:
    global _sock
    if _sock is None:
        _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Don't block emit on network hiccups.
        _sock.setblocking(False)
    return _sock


def add_target(host: str, port: int) -> bool:
    """Add a UDP destination. Returns True if newly added."""
    if not host or host in ("0.0.0.0", "::"):
        return False
    key = (host, int(port))
    with _lock:
        if key in _targets:
            return False
        _targets.add(key)
    return True


def targets() -> list[str]:
    with _lock:
        return [f"{h}:{p}" for h, p in sorted(_targets)]


def _refresh_broadcast_targets(port: int) -> None:
    for ip in _local_ipv4s():
        bcast = _broadcast_for(ip)
        if bcast:
            add_target(bcast, port)
    # Global broadcast as a last-resort LAN spray.
    add_target("255.255.255.255", port)


def _write_line(line: str, payload: bytes, is_error: bool) -> None:
    """Worker-side durable write + UDP fanout. Never called by motion code."""
    global _err_fh
    with _lock:
        try:
            if _fh is not None:
                _fh.write(line + "\n")
                if is_error:
                    _fh.flush()
        except Exception:
            pass
        if is_error:
            try:
                if _err_fh is None:
                    out = errors_path()
                    out.parent.mkdir(parents=True, exist_ok=True)
                    _err_fh = out.open("a", encoding="utf-8", buffering=1)
                _err_fh.write(line + "\n")
                _err_fh.flush()
            except Exception:
                pass
        dests = list(_targets)
        sock = _sock
    if sock is not None:
        for dest in dests:
            try:
                sock.sendto(payload, dest)
            except OSError:
                pass


def _worker_emit_drop_notice(n: int) -> None:
    ev = {
        "ts": _utc_iso(),
        "mono": round(time.monotonic(), 6),
        "kind": "log",
        "level": "warn",
        "src": "event_log",
        "msg": f"log queue dropped {n} low-priority event(s)",
        "data": {"dropped": n, "queue_max": _QUEUE_MAX},
    }
    line = json.dumps(ev, default=str, separators=(",", ":"))
    payload = (line + "\n").encode("utf-8")
    with _lock:
        _ring.append(ev)
        if len(_ring) > _RING_MAX:
            del _ring[: len(_ring) - _RING_MAX]
    _write_line(line, payload, False)


def _log_worker() -> None:
    global _dropped_events, _last_worker_refresh
    while not _worker_stop.is_set() or not _q.empty():
        try:
            line, payload, is_error = _q.get(timeout=0.25)
        except queue.Empty:
            continue

        with _lock:
            dropped = _dropped_events
            _dropped_events = 0
        if dropped:
            _worker_emit_drop_notice(dropped)

        now = time.monotonic()
        if now - _last_worker_refresh > 30.0:
            try:
                port = int(os.environ.get("HEXAPOD_LOG_PORT", DEFAULT_UDP_PORT))
            except ValueError:
                port = DEFAULT_UDP_PORT
            try:
                _refresh_broadcast_targets(port)
            except Exception:
                pass
            _last_worker_refresh = now

        _write_line(line, payload, is_error)
        _q.task_done()


def _ensure_worker() -> None:
    global _worker_thread
    if _worker_thread is not None and _worker_thread.is_alive():
        return
    _worker_stop.clear()
    _worker_thread = threading.Thread(
        target=_log_worker, name="event-log-writer", daemon=True)
    _worker_thread.start()


def _enqueue(line: str, payload: bytes, is_error: bool) -> None:
    """Queue a log write without allowing logging to block motion."""
    global _dropped_events
    try:
        _q.put_nowait((line, payload, is_error))
        return
    except queue.Full:
        pass
    if is_error:
        # Preserve safety/error breadcrumbs by evicting one older event.
        try:
            _q.get_nowait()
            _q.task_done()
        except queue.Empty:
            pass
        try:
            _q.put_nowait((line, payload, is_error))
            return
        except queue.Full:
            pass
    with _lock:
        _dropped_events += 1


def _beacon_loop(beacon_port: int, event_port: int) -> None:
    """Listen for laptop ``log_sink`` beacons and learn their unicast IP."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", beacon_port))
    except OSError:
        return
    sock.settimeout(1.0)
    while not _beacon_stop.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            msg = json.loads(data.decode("utf-8", errors="replace"))
        except Exception:
            continue
        if not isinstance(msg, dict) or msg.get("kind") != "log_sink":
            continue
        try:
            port = int(msg.get("port") or event_port)
        except (TypeError, ValueError):
            port = event_port
        host = addr[0]
        # Prefer an IP the laptop claims on our subnet, else the packet source.
        claimed = msg.get("ips") or []
        local = _local_ipv4s()
        chosen = host
        for cand in claimed:
            if not isinstance(cand, str):
                continue
            for lip in local:
                if _broadcast_for(cand) == _broadcast_for(lip):
                    chosen = cand
                    break
        if add_target(chosen, port):
            try:
                emit("log", f"log sink {chosen}:{port}", src="event_log",
                     data={"host": chosen, "port": port, "via": "beacon"})
            except Exception:
                pass
    try:
        sock.close()
    except OSError:
        pass


def configure(*, host: str | None = None, port: int | None = None,
              path: Path | None = None, mcu_mode: str | None = None,
              beacon_port: int | None = None) -> dict:
    """Open JSONL + start auto UDP fanout (broadcast + beacon learning)."""
    global _fh, _path, _mcu_mode, _beacon_thread, _configured

    env_host = os.environ.get("HEXAPOD_LOG_HOST", "").strip()
    if host is None:
        host = env_host
    # "auto" / empty / "broadcast" → no fixed unicast; use discovery.
    if host.lower() in ("", "auto", "broadcast", "discover", "*"):
        host = ""

    try:
        port = int(port if port is not None
                   else os.environ.get("HEXAPOD_LOG_PORT", DEFAULT_UDP_PORT))
    except ValueError:
        port = DEFAULT_UDP_PORT
    try:
        beacon_port = int(
            beacon_port if beacon_port is not None
            else os.environ.get("HEXAPOD_LOG_BEACON_PORT", DEFAULT_BEACON_PORT)
        )
    except ValueError:
        beacon_port = DEFAULT_BEACON_PORT

    if mcu_mode is not None:
        _mcu_mode = mcu_mode
    else:
        _mcu_mode = os.environ.get("HEXAPOD_LOG_MCU", "cmds").strip().lower() or "cmds"

    out = path or events_path()
    out.parent.mkdir(parents=True, exist_ok=True)
    with _lock:
        if _fh is not None and _path != out:
            try:
                _fh.close()
            except Exception:
                pass
            _fh = None
        _path = out
        if _fh is None:
            _fh = out.open("a", encoding="utf-8", buffering=1)
        _ensure_sock()
    _ensure_worker()

    _refresh_broadcast_targets(port)
    if host:
        add_target(host, port)

    # Beacon listener (once).
    if _beacon_thread is None or not _beacon_thread.is_alive():
        _beacon_stop.clear()
        _beacon_thread = threading.Thread(
            target=_beacon_loop, args=(beacon_port, port),
            name="log-beacon", daemon=True,
        )
        _beacon_thread.start()

    _configured = True
    return {
        "path": str(out),
        "udp": targets(),
        "beacon_port": beacon_port,
        "event_port": port,
        "mcu_mode": _mcu_mode,
        "queue_max": _QUEUE_MAX,
        "auto": True,
    }


def _ensure() -> None:
    if not _configured or _fh is None:
        configure()


def emit(kind: str, msg: str = "", *, src: str = "robot",
         data: dict | None = None, level: str = "info") -> dict:
    """Record one event.

    The hot path updates the in-memory ring and queues disk/UDP work for a
    bounded background writer. If the writer falls behind, low-priority log
    traffic is dropped instead of delaying motion.
    """
    global _seq
    _ensure()

    with _lock:
        _seq += 1
        seq = _seq
    ev = {
        "ts": _utc_iso(),
        "mono": round(time.monotonic(), 6),
        "seq": seq,
        "kind": str(kind),
        "level": str(level),
        "src": str(src),
        "msg": str(msg) if msg is not None else "",
    }
    if data:
        try:
            blob = json.dumps(data, default=str)
            if len(blob) > 3500:
                data = {"_truncated": True, "keys": list(data.keys())[:40]}
        except Exception:
            data = {"_error": "unserializable"}
        ev["data"] = data

    line = json.dumps(ev, default=str, separators=(",", ":"))
    payload = (line + "\n").encode("utf-8")
    with _lock:
        _ring.append(ev)
        if len(_ring) > _RING_MAX:
            del _ring[: len(_ring) - _RING_MAX]
    _enqueue(line, payload, ev["level"] == "error")
    return ev


def recent(n: int = 100) -> list[dict]:
    with _lock:
        return list(_ring[-max(1, n):])


def stats() -> dict:
    with _lock:
        dropped = _dropped_events
    return {
        "queue": _q.qsize(),
        "queue_max": _QUEUE_MAX,
        "dropped_pending_notice": dropped,
        "writer_alive": bool(
            _worker_thread is not None and _worker_thread.is_alive()),
        "udp_targets": targets(),
    }


def flush(timeout: float = 1.0) -> bool:
    """Best-effort drain for shutdown/tests; never used in motion loops."""
    end = time.monotonic() + max(0.0, float(timeout))
    while time.monotonic() < end:
        if _q.empty():
            return True
        time.sleep(0.01)
    return _q.empty()


def should_log_mcu(cmd: str) -> bool:
    mode = _mcu_mode
    if mode in ("off", "0", "none", "false"):
        return False
    if mode in ("all", "raw", "full"):
        return True
    c = (cmd or "").lstrip()
    for p in MCU_SKIP_PREFIXES:
        if c.startswith(p) or c == p.strip():
            return False
    return True


def emit_mcu(cmd: str, reply: str | None, *, ms: float | None = None) -> None:
    if not should_log_mcu(cmd):
        return
    data: dict[str, Any] = {"cmd": cmd, "reply": (reply or "")[:240]}
    if ms is not None:
        data["ms"] = round(ms, 2)
    emit("mcu", cmd.split()[0] if cmd else "mcu", src="mcu", data=data)


# Identical repeated errors (e.g. a poll loop hitting the same refusal)
# only re-log every this many seconds.
_ERR_DEDUPE_S = 10.0
_err_last: dict[tuple, float] = {}


def emit_api_error(method: str, path: str, *, code: int | None = None,
                   error: str | None = None, peer: str | None = None,
                   body: Any = None) -> None:
    """Log one website/API error response (refusals, 4xx/5xx, ok:false).

    Lands in events.jsonl AND logs/errors.jsonl. Deduped so a chatty
    poll repeating the same failure logs at most every 10 s.
    """
    base = path.split("?", 1)[0]
    msg = str(error) if error else f"HTTP {code}"
    key = (method, base, msg[:200])
    now = time.monotonic()
    with _lock:
        last = _err_last.get(key)
        if last is not None and now - last < _ERR_DEDUPE_S:
            return
        if len(_err_last) > 200:
            _err_last.clear()
        _err_last[key] = now
    data: dict[str, Any] = {"method": method, "path": path}
    if code is not None:
        data["code"] = code
    if error:
        data["error"] = str(error)[:500]
    if peer:
        data["peer"] = peer
    if body not in (None, "", {}, []):
        if isinstance(body, (dict, list)):
            data["body"] = body
        else:
            data["body"] = str(body)[:300]
    emit("error", f"{method} {base}: {msg}", src="http",
         level="error", data=data)


def emit_http(method: str, path: str, *, body: Any = None,
              code: int | None = None, peer: str | None = None) -> None:
    base = path.split("?", 1)[0]
    if method == "GET" and base in (
        "/api/ping", "/api/demo/status", "/api/robot", "/api/status",
        "/api/pose", "/api/calibrate", "/api/plant", "/api/imu",
        "/api/events", "/api/errors", "/", "/index.html",
    ):
        return
    data: dict[str, Any] = {"method": method, "path": path}
    if peer:
        data["peer"] = peer
    if code is not None:
        data["code"] = code
    if body not in (None, "", {}, []):
        if isinstance(body, (dict, list)):
            data["body"] = body
        else:
            text = str(body)
            data["body"] = text[:500]
    emit("cmd", f"{method} {base}", src="http", data=data)


def install_print_hook() -> None:
    """Mirror ``print(...)`` into the event stream (kind=log)."""
    global _print_hooked, _orig_print
    if _print_hooked:
        return
    _orig_print = print

    def _hooked(*args, **kwargs):
        _orig_print(*args, **kwargs)
        try:
            sep = kwargs.get("sep", " ")
            text = sep.join(str(a) for a in args)
            if text.strip():
                emit("log", text[:1000], src="print")
        except Exception:
            pass

    import builtins
    builtins.print = _hooked
    _print_hooked = True


# Auto-configure from env on import so early prints still land somewhere.
try:
    configure()
except Exception:
    pass


atexit.register(flush, 1.5)
