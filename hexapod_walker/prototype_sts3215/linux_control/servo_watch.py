"""Background servo health watchdog: liveness + over-temperature.

Every few seconds it does ONE MCU round-trip (``read_all_feedback`` — a
single bulk transaction covering all 18 expected servos) and publishes:

- which expected servos are MISSING (drives the TFT error panel and the
  ``servo`` block in ``/api/robot``), and
- each servo's temperature. Any servo at/above ``SHUTOFF_C`` gets its
  torque cut immediately and is latched ``tripped`` until it cools below
  ``CLEAR_C``. Torque is never re-enabled automatically — that is the
  operator's call (ARM again once it has cooled).

The STS3215 also has its own EEPROM max-temp limit (~70 C default) that
unloads torque in firmware; this watchdog trips earlier (65 C, matching
``rl_move`` safety), logs the event, and puts it on the screen instead of
failing silently (2026-08-06: a knee cooked with no warning anywhere).

Bus contention: the MCU bus serializes transactions internally, so a
watch read only adds a brief latency blip to a running job's loop. While
a bench job owns the bus we still watch, just less often (``BUSY_PERIOD``)
— overheat risk is highest exactly when something is driving the motors.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable

from feetech_bus import N_JOINTS, joint_to_servo_id

WATCH_PERIOD_S = 3.0    # idle cadence
BUSY_PERIOD_S = 10.0    # while a demo/job owns the bus
WARN_C = 55             # show on screen / web
SHUTOFF_C = 65          # cut that servo's torque (rl_move safety uses 65)
CLEAR_C = 50            # un-latch "tripped" once cooled below this
STALE_S = 30.0          # snapshot older than this counts as unknown


class ServoWatch:
    """Liveness + temperature monitor with a per-servo thermal cutoff."""

    def __init__(self, get_bus: Callable[[], Any],
                 is_busy: Callable[[], bool],
                 label: Callable[[int], str]):
        self._get_bus = get_bus
        self._is_busy = is_busy
        self._label = label  # joint index -> human name ("L5 knee")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._snap: dict = {"ok": False, "ts": 0.0}
        self._tripped: set[int] = set()   # joints we torqued off for heat
        # Joints that read >= SHUTOFF_C on the LAST tick. A single hot read
        # is not trusted: corrupted bytes on the shared bus produced four
        # phantom 70-90 C trips on 2026-08-09 (each "cooled" to ~33 C within
        # seconds — thermally impossible). Real heat survives two ticks.
        self._hot_pending: set[int] = set()

    # -- public ---------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="servo-watch", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def state(self) -> dict:
        """Latest snapshot (display-ready; no bus traffic)."""
        with self._lock:
            snap = dict(self._snap)
        if snap.get("ts") and time.time() - snap["ts"] > STALE_S:
            snap["ok"] = False
            snap["stale"] = True
        return snap

    # -- internals ------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            busy = False
            try:
                busy = bool(self._is_busy())
                self._tick()
            except Exception as e:
                with self._lock:
                    self._snap = {"ok": False, "ts": time.time(),
                                  "error": str(e)}
            self._stop.wait(BUSY_PERIOD_S if busy else WATCH_PERIOD_S)

    def _tick(self) -> None:
        bus = self._get_bus()
        if bus is None or not hasattr(bus, "read_all_feedback"):
            with self._lock:
                self._snap = {"ok": False, "ts": time.time(),
                              "error": "no bus"}
            return

        fb = bus.read_all_feedback()  # {joint: {temp_c, current_a, ...}}
        now = time.time()

        # IMU liveness piggybacks on the same tick: one cheap MCU round
        # trip. None = bus can't probe it (USB adapter mode), not a fault.
        imu_ok = None
        read_imu = getattr(bus, "read_imu", None)
        if callable(read_imu):
            try:
                imu_ok = read_imu(timeout=0.6, apply_calib=False) is not None
            except Exception:
                imu_ok = False

        missing = sorted(j for j in range(N_JOINTS) if j not in fb)
        hot: list[dict] = []
        max_t, max_j = -1, None
        for j, f in sorted(fb.items()):
            t = int(f.get("temp_c") or 0)
            if t > max_t:
                max_t, max_j = t, j
            if t >= SHUTOFF_C and j not in self._tripped:
                if j in self._hot_pending:      # 2nd consecutive hot read
                    self._trip(bus, j, t)
                else:
                    self._hot_pending.add(j)
            elif t < SHUTOFF_C:
                self._hot_pending.discard(j)
            if j in self._tripped and t <= CLEAR_C:
                self._tripped.discard(j)
                self._emit("servo_cooled",
                           f"{self._label(j)} cooled to {t}C "
                           "(still torque-off; ARM to re-enable)",
                           {"joint": j, "temp_c": t})
            if t >= WARN_C:
                hot.append({"joint": j, "name": self._label(j), "temp_c": t,
                            "tripped": j in self._tripped})

        with self._lock:
            self._snap = {
                "ok": True,
                "ts": now,
                "expected": N_JOINTS,
                "live": len(fb),
                "missing": missing,
                "missing_names": [self._label(j) for j in missing],
                "max_temp_c": max_t if max_t >= 0 else None,
                "hottest": self._label(max_j) if max_j is not None else None,
                "hot": hot,
                "tripped": sorted(self._tripped),
                "tripped_names": [self._label(j)
                                  for j in sorted(self._tripped)],
                "warn_c": WARN_C,
                "shutoff_c": SHUTOFF_C,
                "imu_ok": imu_ok,
            }

    def _trip(self, bus: Any, joint: int, temp_c: int) -> None:
        """Cut torque on one over-temperature servo (never re-enables)."""
        sid = joint_to_servo_id(joint)
        try:
            bus.torque(sid, False)
            ok = True
        except Exception:
            ok = False
        self._tripped.add(joint)
        self._emit("servo_overtemp",
                   f"OVERTEMP {self._label(joint)} {temp_c}C >= {SHUTOFF_C}C"
                   f" — torque {'CUT' if ok else 'cut FAILED'}",
                   {"joint": joint, "servo_id": sid, "temp_c": temp_c,
                    "torque_cut_ok": ok}, level="warn")

    @staticmethod
    def _emit(kind: str, msg: str, data: dict, *, level: str = "info"):
        try:
            from event_log import emit
            emit(kind, msg, src="servo_watch", data=data, level=level)
        except Exception:
            print(f"[servo_watch] {msg}")
