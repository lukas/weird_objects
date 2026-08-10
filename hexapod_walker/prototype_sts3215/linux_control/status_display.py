#!/usr/bin/env python3
"""Push web-like status onto the MCU ST7789 panel.

``DX`` updates activity text; the MCU reads each motor's current, lights
the schematic (brighter = more amps), and appends live/I/V footer lines.

Startup always hard-reinits the panel (``DI``): reseating the ribbon leaves
the ST7789 blank while MCU RAM still thinks it is initialized. If a later
paint fails, we reinit again.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Any, Callable


def _has_display(bus: Any) -> bool:
    return hasattr(bus, "display_push") and callable(bus.display_push)


def net_status() -> str:
    """WiFi/IP for an 11-char edge slot: 'ip .4.44' or 'NO WIFI'.

    The UDP connect only performs a local routing lookup (no packets), so
    this is instant whether or not the network is up.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.settimeout(0.2)
            s.connect(("8.8.8.8", 53))
            ip = s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        return "NO WIFI"
    parts = ip.split(".")
    return f"ip .{parts[-2]}.{parts[-1]}"


# Friendly titles for the job panel (26 chars max).
_JOB_TITLES = {
    "rl_probe_dynamics": "MOTOR DYNAMICS",
    "rl_find_plant": "FIND PLANT",
    "calibrate": "CALIBRATE",
}


def _wrap(text: str, width: int, max_lines: int) -> list[str]:
    words = str(text).split()
    lines: list[str] = []
    cur = ""
    for w in words:
        if not cur:
            cur = w
        elif len(cur) + 1 + len(w) <= width:
            cur += " " + w
        else:
            lines.append(cur)
            if len(lines) == max_lines:
                return lines
            cur = w
    if cur:
        lines.append(cur)
    return lines[:max_lines]


def format_job_screen(robot: dict) -> tuple[int, list[str]] | None:
    """Job panel content (pct, [title, 4 body, footer]) or None when idle.

    Used while a calibration/demo job is running: the full-screen ``DJ``
    mode fits 26-char rows and a progress bar, versus the 11-char edge
    slots of the normal schematic panel.
    """
    activity = str(robot.get("activity") or "")
    demo = robot.get("demo") or {}
    running = bool(demo.get("running"))
    if not (running or activity in ("calibrating", "zeroing", "stopping")):
        return None

    name = str(demo.get("name") or activity or "job")
    title = _JOB_TITLES.get(
        name, name.replace("rl_", "").replace("_", " ").upper())
    if activity == "stopping":
        title = "STOPPING"

    progress = demo.get("progress") or {}
    msg = str(progress.get("msg") or demo.get("status")
              or robot.get("detail") or "")
    body = _wrap(msg, 26, 3)

    idx, total = progress.get("index"), progress.get("total")
    pct = -1
    if isinstance(idx, int) and isinstance(total, int) and total > 0:
        # Mid-item estimate: item idx of total is in flight, not done.
        pct = int(round(100.0 * (idx + 0.5) / total))
        body.append(f"{idx + 1}/{total}")
    while len(body) < 4:
        body.append("")

    footer = "ARMED" if robot.get("armed") else "limp"
    return pct, [title] + body[:4] + [footer]


def format_error_screen(robot: dict) -> list[str] | None:
    """Full-screen (``DJ``) rows for bus-level faults, or None.

    When the motor controller link is down, the normal schematic panel —
    robot logo plus edge slots — is misleading: nothing it shows is real.
    Take over the whole screen instead. Single-servo and over-temperature
    problems keep the schematic (still useful) via ``servo_alert``.
    """
    sv = robot.get("servo") or {}
    footer = "ARMED" if robot.get("armed") else "limp"
    if not sv.get("ok"):
        # Watchdog can't complete a bus transaction; ts == 0 is startup
        # (no tick yet) — stay quiet for that.
        if not sv.get("ts"):
            return None
        return ["NOT CONNECTED", "",
                "motor controller bus",
                "is not answering",
                "check MCU/ctrl link", footer]
    missing = sv.get("missing") or []
    exp = int(sv.get("expected") or 0)
    if missing and exp and len(missing) >= exp:
        return ["NOT CONNECTED", "",
                "no motor controller",
                "or 12V power is off",
                f"0/{exp} servos answer", footer]
    return None


def servo_alert(robot: dict) -> list[str] | None:
    """Two panel lines for bus-link / missing / over-hot servos, or None.

    Fed by the ``servo`` block ServoWatch publishes into robot_state().
    A dead motor-controller link outranks everything (zero servos or a
    failing bus is the controller/12 V, not 18 simultaneous failures);
    then missing servos (a hot servo still answers; a missing one is
    power/bus/failure); then heat; then a dead IMU. All outrank
    ordinary activity detail.
    """
    sv = robot.get("servo") or {}
    if not sv.get("ok"):
        # Watchdog can't complete a bus transaction. ts == 0 means it
        # simply hasn't ticked yet (startup) — stay quiet for that.
        if not sv.get("ts"):
            return None
        return ["NO MOTOR", "CONTROLLER"]
    missing = sv.get("missing") or []
    exp = int(sv.get("expected") or 0)
    if missing and exp and len(missing) >= exp:
        # Bus answers but zero servos do: controller unplugged from the
        # servo bus, or the 12 V rail is off.
        return ["NO MOTOR", "CTRL / 12V?"]
    if missing:
        n = len(missing)
        names = sv.get("missing_names") or []
        first = str(names[0] if names else f"j{missing[0]}")
        more = f" +{n - 1}" if n > 1 else ""
        return [f"{n} SVO DOWN" if n > 1 else "SERVO DOWN",
                (first + more)[:20]]
    hot = sv.get("hot") or []
    if hot:
        worst = max(hot, key=lambda h: int(h.get("temp_c") or 0))
        off = " OFF" if worst.get("tripped") else ""
        return [f"HOT {worst.get('temp_c')}C{off}"[:20],
                str(worst.get("name") or "")[:20]]
    if sv.get("imu_ok") is False:
        # None = watchdog can't probe the IMU (USB adapter mode) — only
        # an explicit failed probe is an error.
        return ["NO IMU", "GY-521 I2C?"]
    return None


def format_status_lines(robot: dict, net: str | None = None) -> list[str]:
    """Activity lines only (≤20 chars). Power footer is MCU-side.

    The MCU shows the first 3 lines in 11-char edge slots. Servo alerts
    (missing / over-temperature) take the two free slots first; then a
    finished job that died with ``status = "error: ..."`` (it used
    to vanish from the panel the moment the worker stopped); otherwise
    the last free slot carries the WiFi/IP line, so a headless robot
    still tells you how to reach it — or that it has NO WIFI.
    """
    activity = str(robot.get("activity") or "idle")
    detail = str(robot.get("detail") or "")
    armed = bool(robot.get("armed"))
    demo = robot.get("demo") or {}
    demo_name = str(demo.get("name") or "")
    demo_status = str(demo.get("status") or "")
    running = bool(demo.get("running"))

    lines = ["ARMED" if armed else "limp"]

    alert = servo_alert(robot)
    if alert is not None:
        lines += alert
    elif not running and demo_status.startswith("error"):
        msg = demo_status.split(":", 1)[-1].strip() or "job failed"
        short = demo_name.replace("rl_", "").replace("_", " ") or "job"
        # ERR first — the MCU edge slot truncates to 11 chars.
        lines.append(f"ERR {short}"[:20])
        lines.append(msg[:20])
    elif activity == "demo" or running:
        name = demo_name or detail or "demo"
        lines.append(f"demo {name}"[:20])
        if demo_status and demo_status not in ("running", "idle", ""):
            lines.append(demo_status[:20])
    elif activity in ("calibrating", "zeroing", "stopping"):
        label = "STOPPING" if activity == "stopping" else activity
        lines.append(label[:20])
        if detail:
            lines.append(detail[:20])
    else:
        lines.append(activity[:20])
        if detail:
            lines.append(detail[:20])

    if net == "NO WIFI":
        # Being unreachable outranks activity detail — take slot 3.
        lines = lines[:2] + [net]
    elif net and len(lines) < 3:
        lines.append(net)

    return lines[:5]


def recover_display(bus: Any, *, attempts: int = 3) -> bool:
    """Force TFT hard-reinit. Returns True if MCU acknowledged."""
    if bus is None or not _has_display(bus):
        return False
    recover = getattr(bus, "display_recover", None)
    if callable(recover):
        return bool(recover(attempts=attempts))
    init = getattr(bus, "display_init", None)
    if not callable(init):
        return False
    for i in range(max(1, int(attempts))):
        try:
            if init(timeout=6.0):
                return True
        except Exception:
            pass
        time.sleep(0.15 * (i + 1))
    return False


class StatusDisplay:
    """Background refresher for the MCU TFT."""

    def __init__(self, get_bus: Callable[[], Any],
                 get_robot: Callable[[], dict],
                 *, period_s: float = 1.6):
        self._get_bus = get_bus
        self._get_robot = get_robot
        self._period = float(period_s)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_err = ""
        self._ok_count = 0
        self._fail_streak = 0
        self._recovered = False
        self._net = ""
        self._net_t = 0.0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        # Synchronous recover before the refresh loop — covers ribbon reseat
        # while Linux was down / web was restarting.
        try:
            bus = self._get_bus()
            self._recovered = recover_display(bus, attempts=3)
            if not self._recovered:
                self._last_err = "TFT DI recover failed at start"
        except Exception as e:
            self._recovered = False
            self._last_err = str(e)
        self._thread = threading.Thread(
            target=self._run, name="status-display", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        inited = self._recovered
        while not self._stop.is_set():
            t0 = time.monotonic()
            job = None
            try:
                bus = self._get_bus()
                if bus is None or not _has_display(bus):
                    self._stop.wait(self._period)
                    continue
                if not inited:
                    inited = recover_display(bus, attempts=3)
                    if not inited:
                        self._last_err = "TFT DI recover failed"
                        self._stop.wait(self._period)
                        continue
                    self._recovered = True
                robot = self._get_robot()
                now = time.monotonic()
                if now - self._net_t > 10.0:
                    self._net = net_status()
                    self._net_t = now
                job = format_job_screen(robot)
                err = None if job is not None else format_error_screen(robot)
                if job is not None and hasattr(bus, "display_job"):
                    pct, lines = job
                    painted = (
                        {"job": True}
                        if bus.display_job(lines, pct=pct, timeout=10.0)
                        else None)
                elif err is not None and hasattr(bus, "display_job"):
                    # Bus-level fault: full-screen error, not the happy
                    # schematic. DJ does no servo reads, so keep it on
                    # the job (no-throttle) path below.
                    job = ("error", err)
                    painted = (
                        {"error": True}
                        if bus.display_job(err, pct=-1, timeout=10.0)
                        else None)
                else:
                    job = None
                    painted = bus.display_push(
                        format_status_lines(robot, net=self._net),
                        timeout=10.0)
                if painted is None:
                    self._fail_streak += 1
                    self._last_err = "TFT DX failed"
                    # Panel may have been unplugged/replugged mid-run.
                    if self._fail_streak >= 2:
                        inited = False
                        self._recovered = False
                        recover_display(bus, attempts=2)
                        inited = True
                        self._fail_streak = 0
                else:
                    self._fail_streak = 0
                    self._ok_count += 1
                    self._last_err = ""
            except Exception as e:
                self._last_err = str(e)
                self._fail_streak += 1
                # The fault may be in get_robot()/formatting while the bus
                # is fine — show it instead of leaving a stale panel.
                painted_err = False
                try:
                    bus = self._get_bus()
                    if bus is not None and _has_display(bus):
                        painted_err = bus.display_push(
                            ["ERR", "web error", str(e)[:20]],
                            timeout=6.0) is not None
                except Exception:
                    pass
                if not painted_err:
                    inited = False
                    self._recovered = False
            elapsed = time.monotonic() - t0
            period = self._period
            try:
                demo = (self._get_robot().get("demo") or {})
                # DX reads all servo currents on the MCU — throttle it while
                # a job owns the bus. DJ is pure display, so the job panel
                # can refresh at the normal rate.
                if demo.get("running") and job is None:
                    period = max(period, 2.4)
            except Exception:
                pass
            self._stop.wait(max(0.3, period - elapsed))
