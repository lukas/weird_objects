"""JSON bench helpers for the web UI: status, wiggle, demos.

Uses the same Feetech bus as ``DriveController`` (shared lock).
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

from feetech_bus import N_JOINTS, joint_to_servo_id

if TYPE_CHECKING:
    from drive_controller import DriveController

AXIS = ("yaw", "hip", "knee")
REGISTRY_CANDIDATES = (
    Path(__file__).resolve().parent / "urt2_setup" / "motor_setup_registry.json",
    Path(__file__).resolve().parent.parent / "motor_setup" / "motor_setup_registry.json",
    Path(__file__).resolve().parent / "motor_setup_registry.json",
    Path.home() / "hexapod_sts" / "urt2_setup" / "motor_setup_registry.json",
)

# Air demos must start near logical 0°. Planted/rise demos start from a stand.
AIR_DEMO_NAMES = frozenset({
    "breathe", "breathe_v", "heartbeat", "twinkle", "shimmy", "ripple",
    "conductor", "arms_up",
    "air_meet", "air_pendulum", "air_orbits",
    # dance goes planted mid-routine but starts AND ends at sit zero
    # (limp), so it homes like an air demo and must not stand-hold after.
    "dance",
    "dance_walk",
})
ZERO_TOL_DEG = 6.0

# Sit-from-stand exemption to the MAX_SAFE_DELTA_DEG guard: the present
# pose counts as "at stand" when every live joint is within this many
# degrees of the captured stand pose. Must tolerate a gait stopped
# mid-stride: the loaded tripod freezes in its push posture, measured
# up to 17.4 deg from the captured plant (tape session 08-10, leg2b —
# 15 deg refused the Sit button after EVERY walk). A wrong-zero pose
# reads knees ~160 deg off the plant, so 30 deg is still unambiguous.

def _emit_servo_fb(tag: str, tracker, target: list[float] | None = None,
                   ) -> None:
    """Log the tracker's last full feedback sweep to the event stream.

    One ``servo_fb`` event per sweep: per-joint present deg, current,
    speed, load, temp — "what all the servos are saying" — plus the
    commanded target at that moment. Lands in logs/events.jsonl and
    the /api/events ring.
    """
    try:
        from event_log import emit
    except ImportError:
        return
    try:
        data: dict = {"joints": [
            {"j": fb["joint"], "id": fb["id"],
             "deg": round(fb["deg"], 1),
             "a": round(fb["current_a"], 2),
             "dps": round(fb["speed_deg_s"]),
             "load_pct": fb["load_pct"],
             "temp_c": fb["temp_c"]} for fb in tracker.last_fb]}
        if target is not None:
            data["target_deg"] = [round(x, 1) for x in target]
        emit("servo_fb", tag, data=data)
    except Exception:
        pass


def _load_names() -> dict[int, str]:
    for path in REGISTRY_CANDIDATES:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        out: dict[int, str] = {}
        for entry in (data.get("servos") or {}).values():
            try:
                out[int(entry["id"])] = str(entry.get("name") or f"ID{entry['id']}")
            except (KeyError, TypeError, ValueError):
                continue
        if out:
            return out
    return {}


def joint_label(joint: int, names: dict[int, str]) -> str:
    sid = joint_to_servo_id(joint)
    if sid in names:
        return names[sid]
    leg, axis = divmod(joint, 3)
    return f"L{leg} {AXIS[axis]}"


class BenchAPI:
    def __init__(self, drive: "DriveController"):
        self.drive = drive
        self.names = _load_names()
        self._demo_thread: threading.Thread | None = None
        self._demo_abort = threading.Event()
        self._demo_gen = 0  # bumps on each new demo/zero so stale workers exit
        self._demo_name: str | None = None
        self._demo_status = "idle"
        self._demo_params: dict = {}
        # LIVE tempo multiplier (web slider while a demo runs). Streamed
        # demos read it every tick; breathe at each half-breath.
        self._demo_speed_live = 1.0
        # What the robot is *trying* to do (UI-facing intent).
        # idle | limp | armed | demo | zeroing | stopping | calibrating
        self._activity = "idle"
        self._activity_detail = ""
        self._lock = threading.Lock()
        # Last / in-progress step-calibrate report (web Calibrate tab).
        self._cal_result: dict | None = None
        self._cal_progress: dict = {}
        # Last demo cmd-vs-actual telemetry (auto-logged from web demos).
        self._demo_telemetry: dict | None = None
        # True while a motion job is actively streaming pose writes —
        # the TFT display thread must NOT transact on the MCU link
        # (a job-panel text redraw holds it ~1.5 s; measured 08-10,
        # the "big pause in the middle of standing").
        self._bus_hot = False
        # Measure tab: finished run awaiting the operator's tape reading.
        self._meas_pending: dict | None = None
        self._status_display = None
        self._servo_watch = None
        # Live drive session (rl_policy.DriveCommand) — set while an
        # rl_drive worker owns the demo slot, None otherwise.
        self._drive_cmd = None

    def start_status_display(self) -> None:
        """Mirror web status + Σ motor current onto the MCU ST7789."""
        if self._status_display is not None:
            return
        try:
            from status_display import StatusDisplay
        except ImportError:
            return

        def _bus():
            return self.drive.bus

        self._status_display = StatusDisplay(
            _bus, lambda: self.robot_state(), period_s=1.8)
        self._status_display.start()
        if self._status_display._recovered:
            print("[web] TFT recovered (hard reinit OK)")
        else:
            err = self._status_display._last_err or "unknown"
            print(f"[web] TFT recover FAILED: {err}")

    def stop_status_display(self) -> None:
        if self._status_display is not None:
            self._status_display.stop()
            self._status_display = None

    def start_servo_watch(self) -> None:
        """Liveness + over-temp watchdog (TFT error panel, 65C cutoff)."""
        if self._servo_watch is not None or self.drive.dry_run:
            return
        try:
            from servo_watch import ServoWatch
        except ImportError:
            return
        self._servo_watch = ServoWatch(
            lambda: self.drive.bus,
            lambda: bool(self._demo_thread and self._demo_thread.is_alive()),
            lambda j: joint_label(j, self.names),
            on_trip=self.thermal_panic)
        self._servo_watch.start()

    def thermal_panic(self, reason: str) -> None:
        """Kill ALL motion and limp the robot — the watchdog's overtemp
        response (08-11: two hips crossed shutoff mid-glide; the old
        single-servo cut would have left the job driving the other 17
        joints on a robot with one dead leg). Runs on the watchdog
        thread; never raises."""
        try:
            from event_log import emit
            emit("thermal_panic",
                 f"THERMAL PANIC: {reason} — stopping all motion, "
                 "torque off all", src="servo_watch", level="error")
        except Exception:
            print(f"[thermal_panic] {reason}")
        freed = self._preempt_demo_thread(reason=f"thermal: {reason}",
                                          timeout=4.0)
        d = self.drive
        with d._lock:
            d.mode = "idle"
            try:
                d.gait.stop()
            except Exception:
                pass
            d.armed = False
            if freed:
                # Bus rule (stop_demo): no writes while a stuck worker
                # might still be mid-SyncWrite — that hang cooked the MCU
                # bridge before. If the join timed out, the per-servo cut
                # + the servo's own EEPROM limit (~70C) stay the backstop.
                try:
                    d._torque_all(False)
                except Exception:
                    pass
        with self._lock:
            self._activity = "limp"
            self._activity_detail = f"thermal panic: {reason}"

    def stop_servo_watch(self) -> None:
        if self._servo_watch is not None:
            self._servo_watch.stop()
            self._servo_watch = None

    # -- robot / demo state --------------------------------------------------
    def _set_activity(self, activity: str, detail: str = "") -> None:
        with self._lock:
            self._activity = activity
            self._activity_detail = detail
        # Errors/refusals that only surface via status polling still get
        # a line in the event log + logs/errors.jsonl.
        low = (detail or "").lower()
        if "error" in low or "refused" in low:
            try:
                from event_log import emit
                emit("error", f"{activity}: {detail}", src="bench",
                     level="error", data={"activity": activity})
            except Exception:
                pass

    def demo_state(self) -> dict:
        with self._lock:
            return {
                "name": self._demo_name,
                "status": self._demo_status,
                "running": bool(self._demo_thread and self._demo_thread.is_alive()),
                "speed_live": self._demo_speed_live,
                "params": dict(self._demo_params),
                # Live worker progress (msg + optional joint/index/total) —
                # the TFT job panel renders counts/percent from this.
                "progress": dict(self._cal_progress)
                if self._cal_progress else None,
                "telemetry": dict(self._demo_telemetry)
                if self._demo_telemetry else None,
                "bus_hot": bool(
                    self._bus_hot and self._demo_thread
                    and self._demo_thread.is_alive()),
            }

    def robot_state(self, *, check_zero: bool = False) -> dict:
        """Global intent + drive mode (+ optional near-zero probe)."""
        d = self.drive
        with d._lock:
            armed = d.armed
            mode = d.mode
            dry = d.dry_run
            drive_status = d.status
        demo = self.demo_state()
        with self._lock:
            activity = self._activity
            detail = self._activity_detail
        # Derive a clearer activity if the worker hasn't set one yet.
        if demo["running"] and activity not in ("demo", "zeroing", "stopping"):
            activity = "demo"
        elif not demo["running"] and activity in ("demo", "zeroing", "stopping"):
            # Stale — worker finished without clearing (shouldn't happen).
            if demo["status"] in ("done", "aborted", "idle", "skipped"):
                activity = "armed" if armed else "limp"
                detail = demo["status"]
        if not armed and activity in ("idle", "armed"):
            activity = "limp"

        out = {
            "activity": activity,
            "detail": detail,
            "armed": armed,
            "mode": mode,
            "dry_run": dry,
            "drive_status": drive_status,
            "demo": demo,
            "air_demos_need_zero": True,
            "zero_tol_deg": ZERO_TOL_DEG,
        }
        if self._servo_watch is not None:
            out["servo"] = self._servo_watch.state()
        if check_zero:
            out["zero"] = self.check_near_zero()
        return out

    def check_near_zero(self, tol_deg: float = ZERO_TOL_DEG) -> dict:
        """Read present pose; ``at_zero`` if all live joints within tol of 0°."""
        d = self.drive
        if d.dry_run:
            return {"at_zero": True, "max_err_deg": 0.0, "live": 0, "tol_deg": tol_deg}
        if not d.bus:
            return {"at_zero": False, "max_err_deg": None, "live": 0,
                    "tol_deg": tol_deg, "error": "no bus"}
        # Avoid fighting an active demo thread on the serial bus.
        if self._demo_thread and self._demo_thread.is_alive():
            return {"at_zero": False, "max_err_deg": None, "live": 0,
                    "tol_deg": tol_deg, "error": "busy", "busy": True}
        max_err = 0.0
        n = 0
        try:
            with d._lock:
                bus = d.bus
                for joint in range(N_JOINTS):
                    deg = bus.read_position_deg(joint)
                    if deg is None:
                        continue
                    n += 1
                    max_err = max(max_err, abs(float(deg)))
        except Exception as e:
            return {"at_zero": False, "max_err_deg": None, "live": n,
                    "tol_deg": tol_deg, "error": str(e)}
        if n == 0:
            return {"at_zero": False, "max_err_deg": None, "live": 0,
                    "tol_deg": tol_deg, "error": "no feedback"}
        return {
            "at_zero": max_err <= float(tol_deg),
            "max_err_deg": round(max_err, 2),
            "live": n,
            "tol_deg": float(tol_deg),
        }

    # -- status (motors table) -----------------------------------------------
    def status(self) -> dict:
        d = self.drive
        with d._lock:
            port = d.port
            dry = d.dry_run
            armed = d.armed
            mode = d.mode
            bus = d.bus
            drive_status = d.status
        motors = []
        live: list[int] = []
        if bus is not None and not (self._demo_thread and self._demo_thread.is_alive()):
            try:
                from urt2_bench import read_servo_health
                live = sorted(bus.scan(range(1, 31)))
                for sid in live:
                    try:
                        h = read_servo_health(bus, sid)
                    except Exception as e:
                        motors.append({
                            "id": sid, "ok": False, "error": str(e),
                        })
                        continue
                    joint = sid - 2 if 2 <= sid <= 19 else None
                    motors.append({
                        "id": sid,
                        "ok": True,
                        "joint": joint,
                        "name": (joint_label(joint, self.names)
                                 if joint is not None
                                 else self.names.get(sid, f"ID{sid}")),
                        "deg": round(float(h.get("deg", 0.0)), 2),
                        "load_pct": round(float(h.get("load_pct", 0.0)), 1),
                        "current_a": round(float(h.get("current_a", 0.0)), 3),
                        "volt": round(float(h.get("volt", 0.0)), 2),
                        "temp_c": int(h.get("temp_c") or 0),
                        "moving": int(h.get("moving") or 0),
                        "torque": int(h.get("torque_enable") or 0),
                        "alarm": bool(h.get("alarm")),
                        "status_bits": [n for n, _ in (h.get("status_bits") or [])],
                        "volt_limit_max": h.get("volt_limit_max"),
                    })
            except Exception as e:
                return {
                    "port": port, "dry_run": dry, "armed": armed, "mode": mode,
                    "status": drive_status, "error": str(e),
                    "live_ids": [], "motors": [],
                    "demo": self.demo_state(),
                    "robot": self.robot_state(),
                }
        return {
            "port": port,
            "dry_run": dry,
            "armed": armed,
            "mode": mode,
            "status": drive_status,
            "live_ids": live,
            "motors": motors,
            "demo": self.demo_state(),
            "robot": self.robot_state(),
        }

    def pose(self) -> dict:
        """Fast present-angle snapshot for the live schematic (no health scan).

        Returns 18 logical joint degrees (yaw/hip/knee × 6).  Missing servos
        are ``null``.  Does **not** hold the drive lock across the whole scan
        (that starved stand/rise SyncWrites when Live was open).
        """
        d = self.drive
        with d._lock:
            dry = d.dry_run
            armed = d.armed
            mode = d.mode
            bus = d.bus
        geom = {
            "coxa_mm": 12.5,
            "femur_mm": 90.0,
            "tibia_mm": 128.0,
            "body_r_mm": 55.0,
        }
        demo = self.demo_state()
        if dry:
            # Sit-ish default so the page still draws something offline.
            deg = []
            for _ in range(6):
                deg.extend([0.0, 0.0, 0.0])
            return {
                "ok": True, "dry_run": True, "armed": armed, "mode": mode,
                "degrees": deg, "live": 0, "ts": time.time(), "geom": geom,
                "demo": demo,
            }
        if not bus:
            return {
                "ok": False, "error": "no bus", "degrees": [None] * N_JOINTS,
                "live": 0, "ts": time.time(), "geom": geom,
                "demo": demo,
            }
        degrees: list[float | None] = [None] * N_JOINTS
        live = 0
        try:
            # Bus has its own lock; avoid holding drive._lock here.
            for joint in range(N_JOINTS):
                try:
                    v = bus.read_position_deg(joint)
                except Exception:
                    v = None
                if v is None:
                    continue
                degrees[joint] = round(float(v), 2)
                live += 1
        except Exception as e:
            return {
                "ok": False, "error": str(e), "degrees": degrees,
                "live": live, "ts": time.time(), "geom": geom,
                "demo": demo,
            }
        return {
            "ok": True, "dry_run": False, "armed": armed, "mode": mode,
            "degrees": degrees, "live": live, "ts": time.time(), "geom": geom,
            "demo": demo,
        }

    def _enter_stand_hold(self) -> None:
        """Keep re-holding walk-plant after stand zero / planted demos."""
        d = self.drive
        try:
            from feetech_bus import standing_pose_degrees
            stand = standing_pose_degrees()
        except Exception:
            stand = None
        # Full torque for weight-bearing hold (demos leave soft limits on).
        try:
            from inplace_demos import (STAND_TORQUE_LIMIT, _live_robot_ids,
                                       _set_torque_limit)
            if d.bus is not None:
                _set_torque_limit(d.bus, _live_robot_ids(d.bus),
                                  STAND_TORQUE_LIMIT)
        except Exception:
            pass
        with d._lock:
            d.armed = True
            d.mode = "stand"
            d.gait.stop()
            if stand is not None:
                d._last_pose = list(stand)
            d.status = "standing"

    def list_demos(self) -> list[dict]:
        try:
            from inplace_demos import DEMOS
        except ImportError:
            return []
        try:
            from inplace_demos import (STAND_STREAM_DEMOS,
                                       STREAM_POSE_FACTORIES)
            stand_stream = set(STAND_STREAM_DEMOS)
            live_names = set(STREAM_POSE_FACTORIES) | {"breathe"}
        except ImportError:
            stand_stream, live_names = set(), set()
        out = []
        for n, (t, _) in DEMOS.items():
            # breathe+ kept as alias; UI uses size slider on breathe.
            if n == "breathe+":
                continue
            if n in AIR_DEMO_NAMES:
                group = "air"
            elif n.startswith("quad"):
                group = "quad"      # own web tab (tip-back 4-leg walk)
            elif n in stand_stream:
                group = "stand"
            elif n.startswith("walk"):
                group = "walk"
            else:
                group = "plant"
            out.append({
                "name": n,
                "title": t,
                "air": n in AIR_DEMO_NAMES,
                "group": group,
                "live_speed": n in live_names,
                "has_size": n in ("breathe", "breathe_v", "dance",
                              "dance_walk"),
            })
        return out

    def set_demo_speed(self, speed) -> dict:
        """LIVE tempo (web slider): takes effect on the running demo."""
        try:
            v = float(speed)
        except (TypeError, ValueError):
            return {"ok": False, "error": "bad speed"}
        v = max(0.25, min(3.0, v))
        with self._lock:
            self._demo_speed_live = v
            running = bool(self._demo_thread and self._demo_thread.is_alive())
            if running:
                self._demo_params = {**self._demo_params, "speed_live": v}
        return {"ok": True, "speed": v, "running": running,
                "demo": self.demo_state()}

    # -- actions -------------------------------------------------------------
    def wiggle(self, joint: int, amp: float = 8.0) -> dict:
        if not (0 <= joint < N_JOINTS):
            return {"ok": False, "error": "bad joint"}
        if self.drive.dry_run:
            return {"ok": True, "dry_run": True}
        if not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            return {"ok": False, "error": "stop the demo first"}
        msg = self.drive.handle(f"Q {joint} {amp}")
        if msg == "need ARM":
            return {"ok": False, "error": "need ARM"}
        return {"ok": True, "result": msg, "joint": joint, "amp": amp}

    def _preempt_demo_thread(self, *, reason: str = "stop",
                             timeout: float = 5.0) -> bool:
        """Abort any running demo/zero worker. True if bus is free afterward."""
        t = self._demo_thread
        if t is None or not t.is_alive():
            return True
        self._demo_abort.set()
        with self._lock:
            self._demo_status = "stopping"
        self._set_activity("stopping", reason)
        t.join(timeout=float(timeout))
        still = bool(self._demo_thread and self._demo_thread.is_alive())
        if still:
            with self._lock:
                self._demo_status = "aborted"
                self._activity = "idle"
                self._activity_detail = "stop timed out — use E-STOP if stuck"
            return False
        with self.drive._lock:
            if self.drive.mode == "demo":
                self.drive.mode = "idle"
        return True

    def stop_demo(self) -> dict:
        """Abort the demo thread. Do NOT touch the bus here — concurrent
        SyncWrite + HOLD was hanging the MCU bridge and leaving status at
        ``stopping`` forever."""
        prev = self._demo_name or ""
        ok = self._preempt_demo_thread(reason=prev or "stop", timeout=4.0)
        with self._lock:
            if self._demo_status in ("stopping",):
                self._demo_status = "aborted"
            if ok and self._activity == "stopping":
                self._activity = "armed" if self.drive.armed else "limp"
                self._activity_detail = "aborted"
        return {"ok": True, "demo": self.demo_state(), "robot": self.robot_state()}

    def estop(self) -> dict:
        """TRUE emergency stop: kill the demo/RL worker AND limp, in order.

        Root cause of the 2026-08-18 scare: the web E-STOP sent a bare
        ``X`` to the DriveController, which limps the bus but never tells
        the demo thread — and every demo primitive re-enables torque at
        its next write, so the dance shrugged the limp off and kept going.
        This method is what ``/cmd X`` now routes through:

        1. abort event + gen bump — the worker exits at its next
           checkpoint (≤ ~0.1 s) without running outro glides;
        2. limp NOW (torque off) so the robot stops moving immediately;
        3. wait briefly for the worker to die (its bail path may write
           one last hold);
        4. limp AGAIN so the guaranteed final state is torque-off,
           no matter what the dying worker wrote in between.
        """
        self._demo_abort.set()
        with self._lock:
            self._demo_gen += 1
            if self._demo_thread and self._demo_thread.is_alive():
                self._demo_status = "estopped"

        def _limp() -> None:
            try:
                self.drive.handle("X")
            except Exception:
                pass

        _limp()
        t = self._demo_thread
        joined = True
        if t is not None and t.is_alive():
            t.join(timeout=3.0)
            joined = not t.is_alive()
            _limp()
            if not joined:
                # Worker outlived the join window — keep a watcher that
                # limps once more the moment it finally dies, so a late
                # bail write can never leave torque on.
                def _watch(th: threading.Thread = t) -> None:
                    th.join()
                    _limp()
                threading.Thread(target=_watch, daemon=True).start()
        try:
            from event_log import emit
            emit("cmd", "EMERGENCY STOP (estop)", src="bench",
                 data={"worker_exited": joined})
        except Exception:
            pass
        self._set_activity(
            "limp",
            "EMERGENCY STOP" if joined else
            "EMERGENCY STOP — worker still exiting (bus limp; a watcher "
            "re-limps the instant it dies)")
        return {"ok": True, "worker_exited": joined,
                "demo": self.demo_state(), "robot": self.robot_state()}

    def run_demo(self, name: str, *, speed: float = 1.0,
                 size: float = 1.0, rate: float | None = None,
                 torque: int | None = None, softness: float = 1.0,
                 seconds: float | None = None) -> dict:
        try:
            from inplace_demos import DEMOS, run_demo
        except ImportError as e:
            return {"ok": False, "error": f"inplace_demos missing: {e}"}
        try:
            from inplace_demos import STREAM_POSE_FACTORIES
            streamable = set(STREAM_POSE_FACTORIES)
        except ImportError:
            streamable = set()
        # Alias: breathe+ → breathe at size 2.
        if name == "breathe+":
            name = "breathe"
            size = max(float(size), 2.0)
        if name not in DEMOS:
            return {"ok": False, "error": f"unknown demo {name!r}",
                    "demos": [n for n in DEMOS if n != "breathe+"]}
        if self.drive.dry_run:
            return {"ok": False, "error": "dry-run — no bus"}
        if not self.drive.bus:
            return {"ok": False, "error": "no bus"}

        def _f(val, default, lo, hi):
            try:
                x = float(val)
            except (TypeError, ValueError):
                x = default
            return max(lo, min(hi, x))

        speed = _f(speed, 1.0, 0.25, 3.0)
        size = _f(size, 1.0, 0.5, 3.0)
        softness = _f(softness, 1.0, 0.5, 3.0)
        if rate is not None:
            rate = _f(rate, 0.28, 0.08, 0.60)
        # ``seconds`` is a duration only for demos that take one (air +
        # streamed); planted shows/glides keep their choreographed times.
        duration_ok = name in AIR_DEMO_NAMES or name in streamable
        if seconds is not None:
            seconds = _f(seconds, 60.0, 5.0, 300.0) if duration_ok else None
        if torque is not None:
            try:
                torque = int(round(float(torque)))
            except (TypeError, ValueError):
                torque = None
            if torque is not None:
                torque = max(150, min(1000, torque))

        home = "sit" if name in AIR_DEMO_NAMES else "stand"
        switched_from = None
        if self._demo_thread and self._demo_thread.is_alive():
            switched_from = self._demo_name
            if not self._preempt_demo_thread(
                    reason=f"{switched_from or '?'} → {name}", timeout=5.0):
                return {"ok": False,
                        "error": "previous demo did not stop — try Stop / E-STOP",
                        "demo": self.demo_state(), "robot": self.robot_state()}

        params = {"speed": speed, "home": home}
        if seconds is not None:
            params["seconds"] = seconds
        if name in ("breathe", "breathe_v", "dance", "dance_walk"):
            params.update({"size": size, "softness": softness})
            if rate is not None:
                params["rate"] = rate
        if torque is not None and name in AIR_DEMO_NAMES:
            params["torque"] = torque
        if switched_from:
            params["switched_from"] = switched_from

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = name
            self._demo_status = "starting"
            self._demo_params = dict(params)
            # Live tempo starts at the requested speed; /api/demo/speed
            # can change it while the demo runs.
            self._demo_speed_live = speed
        bits = [f"{name} @ {speed:.2f}×"]
        if switched_from:
            bits.insert(0, f"switch←{switched_from}")
        if name in ("breathe", "breathe_v", "dance", "dance_walk"):
            bits.append(f"size {size:.2f}×")
            if rate is not None:
                bits.append(f"{rate:.2f} Hz")
            bits.append(f"soft {softness:.2f}×")
        if torque is not None:
            bits.append(f"τ {torque}")
        detail = " ".join(bits)
        self._set_activity("demo", detail)

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True
            try:
                # Always home first (sit for air, stand for planted) so a
                # mid-demo switch — or starting from the wrong pose — just
                # works. 08-11 directive: acquire the start SAFELY
                # (collision-aware zero, validated plant stand-up); if
                # that fails the robot is already stopped/limped and the
                # demo must NOT run.
                with self._lock:
                    self._demo_status = f"homing {home}"
                self._set_activity("zeroing", f"{home} zero → {name}")

                def _home_prog(p: dict) -> None:
                    with self._lock:
                        self._demo_status = str(p.get("msg")
                                                or f"homing {home}")

                res_home = self._acquire_start(
                    "zero" if home == "sit" else "stand",
                    gen=gen, on_progress=_home_prog)
                if gen != self._demo_gen:
                    return
                if self._demo_abort.is_set():
                    with self._lock:
                        self._demo_status = "aborted"
                    return
                if not res_home.get("ok"):
                    with self._lock:
                        self._demo_status = (
                            "error: start pose not reached — "
                            + str(res_home.get("error") or "aborted"))
                    return

                with self._lock:
                    self._demo_status = f"running @ {speed:.2f}×"
                self._set_activity("demo", detail)
                # Own the MCU link for the whole motion. Without this the
                # TFT job-panel repaint (~1.5 s serial hold every 1.6 s)
                # starved the 20 Hz demo stream to ~2 Hz — measured
                # 08-17: stand_wave turned into rare giant steps and
                # tipped the robot. Same bug rl_policy_move fixed 08-10.
                self._bus_hot = True
                # Every web demo is also a calibrate run: cmd vs encoder CSV.
                log_dir = Path(__file__).resolve().parent / "logs"
                log_dir.mkdir(parents=True, exist_ok=True)
                stamp = time.strftime("%Y%m%d_%H%M%S")
                log_path = log_dir / f"demo_{name}_{stamp}.csv"
                with self._lock:
                    self._demo_params = {
                        **dict(self._demo_params),
                        "log": log_path.name,
                    }
                def _live_status(msg: str) -> None:
                    if gen != self._demo_gen:
                        return
                    with self._lock:
                        self._demo_status = str(msg)

                if name == "dance_walk":
                    status = self._run_dance_walk(
                        gen=gen, speed=speed, size=size,
                        softness=softness, torque=torque,
                        status_cb=_live_status, log_path=log_path)
                else:
                    extra = {}
                    if name == "dance":
                        extra["standup_fn"] = self._step_standup_fn(
                            gen=gen, speed=speed)
                    status = run_demo(
                        d.bus, name,
                        speed=speed,
                        seconds=seconds,
                        size=size,
                        rate=rate,
                        torque=torque,
                        softness=softness,
                        abort_check=self._demo_abort.is_set,
                        speed_fn=lambda: self._demo_speed_live,
                        status_cb=_live_status,
                        log_path=log_path,
                        **extra,
                    )
                telem = None
                summary_path = log_path.with_name(
                    log_path.stem + "_summary.json")
                if summary_path.is_file():
                    try:
                        telem = json.loads(summary_path.read_text())
                    except (OSError, ValueError):
                        telem = None
                if telem is None and log_path.is_file():
                    telem = {
                        "ok": True,
                        "log": str(log_path),
                        "log_name": log_path.name,
                        "hint": "CSV written; summary pending",
                    }
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._demo_telemetry = telem
                    if self._demo_abort.is_set():
                        self._demo_status = "aborted"
                    else:
                        self._demo_status = status or "done"
                        if telem and telem.get("counts"):
                            c = telem["counts"]
                            self._demo_status = (
                                f"{status or 'done'} · "
                                f"{c.get('green', 0)}g/"
                                f"{c.get('yellow', 0)}y/"
                                f"{c.get('red', 0)}r"
                            )
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._demo_status = f"error: {e}"
            finally:
                self._bus_hot = False
                if gen != self._demo_gen:
                    return
                with self._lock:
                    st = self._demo_status
                if st == "stopping":
                    st = "aborted"
                    with self._lock:
                        self._demo_status = st
                # Planted / rise demos finish at stand zero — keep re-holding.
                if st == "done" and name not in AIR_DEMO_NAMES:
                    self._enter_stand_hold()
                else:
                    with d._lock:
                        if d.mode == "demo":
                            d.mode = "idle"
                self._set_activity(
                    "armed" if d.armed else "limp",
                    st if st in ("done", "aborted", "skipped") else st)

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "params": params, "home": home,
                "switched": bool(switched_from),
                "switched_from": switched_from,
                "demo": self.demo_state(), "robot": self.robot_state()}

    # Victory lap: horse-prance out (open-loop tripod — quick cadence,
    # high knees), RL MOONWALK straight home (silky contrast, heading
    # untouched so the return line is true), then a full 360° pirouette
    # finale in place, where open-loop heading slip can't hurt anything.
    DANCE_LAP_V = 0.055          # m/s, just under the trained 0.06 band
    # Sized to the sim-predicted realized prance-out distance (~0.17 m
    # at 0.038 m/s realized). If hardware pranced further, the moonwalk
    # UNDERSHOOTS home — safe; a long moonwalk overshooting backward is
    # the dangerous direction.
    DANCE_MOONWALK_S = 3.5

    def _step_standup_fn(self, *, gen: int, speed: float):
        """Bound STEP stand-up for the dance's act IV (inline, same gen).

        The dance's own tempo never SLOWS the stand-up below 1x (the
        experiments-tab pacing the operator liked, 08-18); speeding
        the dance up speeds the stand-up too.
        """
        def fn() -> tuple[bool, str]:
            res = self.standup(mode="step",
                               speed=max(1.0, float(speed)),
                               direction="up", torque=700,
                               sync_gen=gen)
            return bool(res.get("ok")), str(res.get("error") or "")
        return fn

    def _run_dance_walk(self, *, gen: int, speed: float, size: float,
                        softness: float, torque: int | None,
                        status_cb, log_path: Path) -> str:
        """dance acts I–V → RL walk victory lap → dance act VI.

        Runs inside the demo worker thread (slot already claimed, sit
        homing already done). A refused lap (rl_policy missing, walk
        preflight fails) is never fatal — the outro still plays so the
        robot always ends asleep at sit zero. A SAFETY-TRIPPED lap is
        fatal: the robot is limped and stays limped (no blind outro
        from an unknown pose — 2026-08-06 lesson).
        """
        from inplace_demos import run_dance_demo

        d = self.drive
        st = run_dance_demo(
            d.bus, part="show", speed=speed, size=size, softness=softness,
            torque=torque, abort_check=self._demo_abort.is_set,
            status_cb=status_cb, log_path=log_path,
            standup_fn=self._step_standup_fn(gen=gen, speed=speed))
        if st != "planted":
            return st

        lap_err, limped = self._victory_lap(gen=gen, status_cb=status_cb)
        if self._demo_abort.is_set() or lap_err == "aborted":
            return "aborted"
        if limped:
            return f"error: lap safety-tripped ({lap_err}) — robot limp"
        if lap_err:
            status_cb(f"lap skipped ({lap_err}) — descending anyway")

        outro_log = log_path.with_name(log_path.stem + "_outro.csv")
        return run_dance_demo(
            d.bus, part="outro", speed=speed, size=size, softness=softness,
            torque=torque, abort_check=self._demo_abort.is_set,
            status_cb=status_cb, log_path=outro_log)

    def _victory_lap(self, *, gen: int,
                     status_cb) -> tuple[str | None, bool]:
        """Prance out → RL moonwalk home → pirouette.

        Returns ``(error, limped)`` — ``(None, False)`` on success.
        A missing gait or refused moonwalk degrades gracefully (the
        remaining phases still play); an abort or safety limp is fatal.
        """
        d = self.drive
        try:
            from inplace_demos import run_dance_prance
        except ImportError as e:
            return f"inplace_demos missing: {e}", False

        st = run_dance_prance(d.bus, "out",
                              abort_check=self._demo_abort.is_set,
                              status_cb=status_cb)
        if st == "aborted" or self._demo_abort.is_set():
            return "aborted", False
        if st != "done":
            # Never moved — skip the whole lap rather than moonwalk
            # from an unknown spot.
            return "prance gait unavailable", False

        moon_err, limped = self._rl_moonwalk(gen=gen, status_cb=status_cb)
        if limped:
            return moon_err, True
        if self._demo_abort.is_set():
            return "aborted", False
        if moon_err:
            status_cb(f"moonwalk skipped ({moon_err}) — pirouette")

        st = run_dance_prance(d.bus, "spin",
                              abort_check=self._demo_abort.is_set,
                              status_cb=status_cb)
        if st == "aborted" or self._demo_abort.is_set():
            return "aborted", False
        return None, False

    def _rl_moonwalk(self, *, gen: int,
                     status_cb) -> tuple[str | None, bool]:
        """RL drive session walking straight backward to the start.

        Returns ``(error, limped)``. Same start contract as
        rl_drive_start: read-only walk preflight with the moderate-tilt
        auto-acquire fallback.
        """
        try:
            from rl_policy import DriveCommand, preflight, run_drive_session
        except ImportError as e:
            return f"rl_policy missing: {e}", False
        d = self.drive
        status_cb("victory lap — walk preflight")
        ok, reason, details = preflight(d.bus, "walk")
        if not ok and (reason.startswith("pose is not")
                       or reason.startswith("tilt too high")):
            r = abs(float(details.get("roll_deg", 90.0)))
            p = abs(float(details.get("pitch_deg", 90.0)))
            if r <= 35.0 and p <= 35.0:
                status_cb("victory lap — acquiring the walk stance")
                res_a = self._acquire_start(
                    "stand", gen=gen,
                    on_progress=lambda pr: status_cb(
                        str(pr.get("msg") or "acquiring stance…")))
                if not res_a.get("ok"):
                    return (str(res_a.get("error") or "stance not acquired"),
                            False)
                ok, reason, details = preflight(d.bus, "walk")
        if not ok:
            return f"preflight: {reason}", False

        # Full torque for the weight-bearing walk (RL never touches the
        # limit register; the dance show leaves it at rise torque).
        try:
            from inplace_demos import _live_robot_ids, _set_torque_limit
            _set_torque_limit(d.bus, _live_robot_ids(d.bus), 1000)
        except Exception:
            pass

        cmd = DriveCommand()
        session_over = threading.Event()

        def _sched():
            status_cb("victory lap — MOONWALK home (silky RL glide)")
            t_end = time.time() + float(self.DANCE_MOONWALK_S)
            while time.time() < t_end:
                if session_over.is_set() or self._demo_abort.is_set():
                    break
                cmd.set(-self.DANCE_LAP_V, 0.0)
                time.sleep(0.15)
            cmd.request_stop()

        sched = threading.Thread(target=_sched, daemon=True)
        sched.start()
        try:
            res = run_drive_session(
                d, cmd,
                on_progress=lambda p: None,
                abort_check=self._demo_abort.is_set,
                walk_weights=self._role_weights("walk"),
                hold_weights=self._role_weights("hold"))
        finally:
            session_over.set()
        sched.join(timeout=2.0)
        if not res.get("ok"):
            return (str(res.get("error") or "drive session failed"),
                    bool(res.get("limped")))
        return None, bool(res.get("limped"))

    def _delta_vs_present(self, goal: list[float]
                          ) -> tuple[float | None, int | None]:
        """Like drive._max_delta_vs_present, but None-aware.

        The first MCU round-trip after a service restart can time out
        WHOLESALE (TFT reinit holds the link); drive's helper then
        compares against nothing and reports worst=0.0, which made
        pose gates pick wrong paths and silently defeated the delta
        guard (08-11: a standing robot 'passed' the belly-down
        guard). Retries once; returns (None, None) when the bus
        really has no readings — callers must treat that as UNKNOWN,
        not 'at the goal'."""
        pairs: list = []
        for attempt in range(2):
            with self.drive._lock:
                present = self.drive._read_present_pose()
            pairs = [(j, p) for j, p in enumerate(present)
                     if p is not None]
            if len(pairs) >= max(1, len(goal) - 4):
                worst, wj = 0.0, None
                for j, p in pairs:
                    dd = abs(float(goal[j]) - float(p))
                    if dd > worst:
                        worst, wj = dd, j
                return worst, wj
            time.sleep(0.4)
        return None, None

    def go_zero(self, pose: str = "sit", *, force: bool = False) -> dict:
        """Go to sit zero (legs out) or stand zero (walk plant) — SAFELY.

        ``pose``: ``sit`` | ``stand``.  Stand keeps torque on (no limp).

        Neither direction refuses on a big delta any more (operator
        directives 08-10 sit / 08-11 stand): the robot ACQUIRES the
        pose instead. SIT = the collision-aware safe-zero plan (stages
        around ground and leg-vs-leg contact; LIMPS on stall or
        unexpected force). STAND = safe zero first when needed, then
        the validated keyframe stand-up onto the plant. If acquisition
        fails the robot stops (hold or limp) and the job errors out.
        Near-pose fast paths still delegate straight to the tuck
        keyframes at 10x.
        """
        pose = (pose or "sit").strip().lower()
        if pose in ("stand", "standing", "plant"):
            pose = "stand"
        else:
            pose = "sit"
        if self.drive.dry_run:
            return {"ok": True, "dry_run": True, "pose": pose}
        if not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            if not self._preempt_demo_thread(
                    reason=f"→ {pose} zero", timeout=5.0):
                return {"ok": False,
                        "error": "previous demo did not stop — try Stop / E-STOP",
                        "robot": self.robot_state()}

        # Standard stand/sit = the validated tuck keyframes at 10x
        # (operator 08-10). When the robot sits at the tuck start
        # (see _delta_vs_present for why the gates must be None-aware)
        # (belly-down, legs out) a STAND delegates to the fast tuck
        # rise; when it is in the tuck stance a SIT delegates to the
        # reversed keyframes. Anything else acquires the pose safely
        # below (safe-zero plan / plant stand-up).
        try:
            kfs = self._load_standup()["modes"]["tuck"]["keyframes"]
            tuck_zero = [float(x) for x in kfs[0]["q_deg"]]
            tuck_stand = [float(x) for x in kfs[-1]["q_deg"]]
            d_zero, _ = self._delta_vs_present(tuck_zero)
            d_stand, _ = self._delta_vs_present(tuck_stand)
            if (pose == "stand" and d_zero is not None
                    and d_zero <= 25.0):
                return self.standup(mode="tuck", speed=10.0,
                                    direction="up")
            # 35 deg: knees sag ~15-20 deg holding the stance at idle
            # torque; the standup align phase eases that out safely.
            if (pose == "sit" and d_stand is not None
                    and d_stand <= 35.0):
                return self.standup(mode="tuck", speed=10.0,
                                    direction="down")
        except Exception:
            pass

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = f"zero_{pose}"
            self._demo_status = "zeroing"
            self._demo_params = {"pose": pose, "force": bool(force)}
            self._cal_progress = {"msg": f"go to {pose} zero"}
        self._set_activity("zeroing", f"go to {pose} zero")

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True
            result: dict = {}
            try:
                self._bus_hot = True

                def _prog(p: dict) -> None:
                    with self._lock:
                        self._cal_progress = dict(p)

                if pose == "stand":
                    result = self._acquire_start("stand", gen=gen,
                                                 on_progress=_prog)
                else:
                    result = self._safe_zero_sync(
                        abort_check=self._demo_abort.is_set,
                        on_progress=_prog)
                if gen != self._demo_gen:
                    return
                with self._lock:
                    if result.get("ok"):
                        self._demo_status = "done"
                    elif result.get("aborted"):
                        self._demo_status = "aborted"
                    else:
                        self._demo_status = (
                            f"error: {result.get('error') or 'failed'}")
                    self._cal_progress = {"msg": self._demo_status}
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._demo_status = f"error: {e}"
            finally:
                self._bus_hot = False
                if gen != self._demo_gen:
                    return
                if result.get("limp"):
                    with d._lock:
                        d.armed = False
                with self._lock:
                    st = self._demo_status
                # Stand home must keep mode=stand so the drive loop re-holds
                # plant (otherwise stance droops after the one-shot glide).
                if st == "done" and pose == "stand":
                    self._enter_stand_hold()
                    self._set_activity("armed", "at stand zero")
                else:
                    with d._lock:
                        if d.mode == "demo":
                            d.mode = "idle"
                    if str(st).startswith("error"):
                        self._set_activity("armed" if d.armed else "limp", st)
                    else:
                        self._set_activity(
                            "armed" if d.armed else "limp",
                            f"at {pose} zero" if st == "done" else st)

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "pose": pose, "demo": self.demo_state(),
                "robot": self.robot_state()}

    def set_zero_here(self) -> dict:
        """Feetech middle-calibrate: current pose becomes logical 0° (no move)."""
        try:
            from urt2_bench import redefine_zero_here
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        if self.drive.dry_run:
            return {"ok": True, "dry_run": True}
        if self._demo_thread and self._demo_thread.is_alive():
            return {"ok": False, "error": "stop the demo first"}
        if not self.drive.bus:
            return {"ok": False, "error": "no bus"}

        d = self.drive
        with d._lock:
            d.mode = "idle"
            d.gait.stop()
            d.armed = False
            try:
                d._torque_all(False)
            except Exception:
                pass
            try:
                result = redefine_zero_here(d.bus)
            except Exception as e:
                return {"ok": False, "error": str(e)}
            d.status = (f"zero-here {result.get('ok_n', 0)}/"
                        f"{result.get('count', 0)} (limp)")
        self._set_activity("limp", "zero redefined here")
        return result

    def _present_pose18(self) -> tuple[list, list[int]]:
        """All 18 present joint degrees (bulk read + per-joint retry).

        Returns ``(values, missing_joint_indices)`` — values contain
        None at the missing slots.
        """
        bus = self.drive.bus
        vals: list = [None] * N_JOINTS
        if bus is None:
            return vals, list(range(N_JOINTS))
        try:
            if hasattr(bus, "read_all_positions"):
                for j, v in (bus.read_all_positions() or {}).items():
                    if 0 <= j < N_JOINTS:
                        vals[j] = float(v)
        except Exception:
            pass
        for j in range(N_JOINTS):
            if vals[j] is None:
                try:
                    v = bus.read_position_deg(j)
                except Exception:
                    v = None
                vals[j] = None if v is None else float(v)
        return vals, [j for j, v in enumerate(vals) if v is None]

    def _safe_zero_sync(self, *, abort_check, on_progress=None) -> dict:
        """Plan + execute the collision-aware go-to-zero SYNCHRONOUSLY.

        Runs in the caller's worker thread; claims no job slot and
        paints no status — the caller owns those. No-ops when already
        at zero. Any anomaly during motion limps the robot
        (``run_safe_zero``) and returns ``ok=False`` — the caller must
        stop its own routine in that case.

        PINNED-TIP GATE (08-11 overheat lesson): when the read-only
        detector says the body is tipped over a folded knee, the
        low-torque untrap fold runs FIRST — driving 18 joints toward
        zero at working torque against a pinned leg is exactly the
        loop that stacked hips to 71 °C. Every motion path that
        acquires its start through here inherits the gate.
        """
        try:
            from safe_zero import (belly_ground_z_mm, plan_safe_zero,
                                   run_safe_zero)
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        untrap = None
        try:
            from pinned_tip import check_pinned_tip, run_untrap_tuck
            verdict = check_pinned_tip(self.drive.bus)
        except Exception:
            verdict = {"pinned": False}
        if verdict.get("pinned"):
            try:
                from event_log import emit
                emit("pinned_tip", verdict.get("why", "pinned-leg tip"),
                     data=verdict, level="warn")
            except Exception:
                pass
            if on_progress:
                on_progress({"msg": "tipped on a trapped leg — "
                                    "low-torque untrap fold first"})
            untrap = run_untrap_tuck(self.drive.bus,
                                     abort_check=abort_check,
                                     on_progress=on_progress)
            if not untrap.get("ok"):
                return {"ok": False, "limp": bool(untrap.get("limp")),
                        "untrap": untrap, "pinned_tip": verdict,
                        "error": ("untrap failed: "
                                  + str(untrap.get("error") or "?"))}
        present, missing = self._present_pose18()
        if missing:
            return {"ok": False,
                    "error": ("no encoder reading from " + ", ".join(
                        joint_label(j, self.names) for j in missing))}
        plan = plan_safe_zero(present, ground_z_mm=belly_ground_z_mm())
        if not plan.get("ok"):
            if untrap is not None:
                plan["untrap"] = untrap
            return plan
        if not plan["stages"]:
            return {"ok": True, "already_at_zero": True,
                    **({"untrap": untrap} if untrap else {})}
        result = run_safe_zero(self.drive.bus, plan["stages"],
                               abort_check=abort_check,
                               on_progress=on_progress)
        if untrap is not None:
            result["untrap"] = untrap
        return result

    def _acquire_start(self, kind: str, *, gen: int,
                       on_progress=None) -> dict:
        """Safely bring the robot to a routine's required start pose.

        ``kind``: ``zero`` (belly down, legs out) or ``stand`` (the
        captured plant stance). Runs INSIDE the caller's worker thread
        — the caller must own the job slot (``gen``).

        Operator directive 08-11: stance-gated routines ACQUIRE their
        start instead of refusing. Strategy: collision-aware safe zero
        first, then the validated keyframe stand-up onto the plant
        when a stand is needed (a near-plant start shortcuts to the
        single-frame re-seat). On ANY failure the failing engine has
        already stopped the robot (hold or limp); this returns
        ``ok=False`` and the caller MUST NOT run its routine.
        """
        def _prog(p: dict) -> None:
            if on_progress:
                try:
                    on_progress(dict(p))
                except Exception:
                    pass
            else:
                with self._lock:
                    self._cal_progress = dict(p)

        kind = "stand" if str(kind).startswith("stand") else "zero"
        acquired: list[str] = []
        if kind == "stand":
            wp = None
            try:
                from feetech_bus import standing_pose_degrees
                wp, _ = self._delta_vs_present(standing_pose_degrees())
            except Exception:
                wp = None
            if wp is not None and wp <= 8.0:
                return {"ok": True, "acquired": acquired}
            if wp is not None and wp <= 25.0:
                # Already standing near the plant: the synthetic
                # "plant" mode shortcuts to align + tripod re-seat —
                # no pointless sit/stand cycle.
                _prog({"msg": "acquiring start: re-seat onto plant…"})
                rs = self.standup(mode="plant", speed=10.0,
                                  direction="up", sync_gen=gen)
                if not rs.get("ok"):
                    return {"ok": False, "acquired": acquired,
                            "error": ("could not reach stand start: "
                                      + str(rs.get("error")
                                            or "aborted"))}
                return {"ok": True, "acquired": ["plant_reseat"]}
        # Everything else goes through a safe zero first (no-op when
        # already there; plans around ground / leg collisions; limps
        # on stall or unexpected force).
        _prog({"msg": "acquiring start: safe zero…"})
        rz = self._safe_zero_sync(abort_check=self._demo_abort.is_set,
                                  on_progress=_prog)
        if not rz.get("ok"):
            why = (rz.get("error")
                   or ("aborted" if rz.get("aborted") else "failed"))
            return {"ok": False, "acquired": acquired,
                    "limp": bool(rz.get("limp")),
                    "error": f"could not reach zero start: {why}"}
        if not rz.get("already_at_zero"):
            acquired.append("safe_zero")
        if kind == "zero":
            return {"ok": True, "acquired": acquired}
        _prog({"msg": "acquiring start: stand-up onto plant…"})
        rs = self.standup(mode="plant", speed=10.0, direction="up",
                          sync_gen=gen)
        if not rs.get("ok"):
            return {"ok": False, "acquired": acquired,
                    "error": ("could not reach stand start: "
                              + str(rs.get("error") or "aborted"))}
        acquired.append("standup_plant")
        return {"ok": True, "acquired": acquired}

    def pinned_tip_state(self) -> dict:
        """READ-ONLY pinned-leg-tip verdict (see pinned_tip.py).

        One IMU read on a level robot; when tipped it settles ~1.2 s
        and reads again before classifying. Never commands motion —
        safe to poll from the web UI.
        """
        try:
            from pinned_tip import check_pinned_tip
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        v = check_pinned_tip(self.drive.bus)
        return {"ok": "error" not in v, **v}

    def untrap(self, *, force: bool = False) -> dict:
        """Low-torque untrap fold (pinned_tip.run_untrap_tuck) as a job.

        Refuses unless the read-only detector confirms a pinned-leg
        tip (``force=true`` overrides for bench testing while the
        operator watches — the move is torque-bounded either way).
        Success leaves the robot level + folded, holding at the LOW
        limit; run safe_zero next. Failure/abort leaves it LIMP.
        """
        try:
            from pinned_tip import TUCK_TORQUE, check_pinned_tip, \
                run_untrap_tuck
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}

        verdict = check_pinned_tip(self.drive.bus)
        if not verdict.get("pinned") and not force:
            return {"ok": False, "pinned_tip": verdict,
                    "error": ("not a pinned-leg tip ("
                              + str(verdict.get("why")
                                    or verdict.get("error") or "?")
                              + ") — nothing to untrap. force=true "
                              "runs the fold anyway (watching!).")}

        if self._demo_thread and self._demo_thread.is_alive():
            if not self._preempt_demo_thread(reason="→ untrap",
                                             timeout=5.0):
                return {"ok": False,
                        "error": ("previous job did not stop — "
                                  "try Stop / E-STOP"),
                        "robot": self.robot_state()}

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = "untrap"
            self._demo_status = "untrap: low-torque fold"
            self._demo_params = {"force": bool(force),
                                 "torque_limit": TUCK_TORQUE}
            self._cal_result = None
            self._cal_progress = {"msg": "untrap: starting"}
        self._set_activity("zeroing", "untrap (low-torque fold)")

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
            result: dict = {}
            try:
                from event_log import emit
                emit("untrap", "start", data=verdict, level="warn")
            except Exception:
                pass
            try:
                self._bus_hot = True

                def _prog(dct: dict) -> None:
                    with self._lock:
                        self._cal_progress = dict(dct)

                result = run_untrap_tuck(
                    d.bus, abort_check=self._demo_abort.is_set,
                    on_progress=_prog)
                result["pinned_tip"] = verdict
            except Exception as e:
                result = {"ok": False, "error": str(e)}
            finally:
                self._bus_hot = False
                if gen != self._demo_gen:
                    return
                limp = bool(result.get("limp"))
                with self._lock:
                    self._cal_result = result
                    if result.get("ok"):
                        self._demo_status = ("done · level + folded "
                                             "(low torque) — safe zero "
                                             "next")
                    else:
                        self._demo_status = str(
                            result.get("error") or "error")
                    self._cal_progress = {"msg": self._demo_status}
                    st = self._demo_status
                with d._lock:
                    if d.mode == "demo":
                        d.mode = "idle"
                    if limp:
                        d.armed = False
                    else:
                        d.armed = True
                    d.status = st
                try:
                    from event_log import emit
                    emit("untrap", "done" if result.get("ok") else st,
                         data={k: result.get(k)
                               for k in ("ok", "limp", "tilt_deg",
                                         "trapped_names", "peak_a")
                               if k in result})
                except Exception:
                    pass
                self._set_activity(
                    "limp" if limp else "armed", st)

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "started": True, "pinned_tip": verdict,
                "demo": self.demo_state(), "robot": self.robot_state()}

    def safe_zero(self, *, dry_run: bool = False,
                  force: bool = False) -> dict:
        """Collision-aware go-to-zero with limp-on-anomaly (ask 08-10).

        Plans staged waypoints from the present encoders to logical 0°
        (``safe_zero.plan_safe_zero``: straighten → center yaws with
        feet lifted clear of the ground → extend flat) and REFUSES with
        an error when no ground/self-collision-free path exists. During
        motion, any servo reporting stall-fight current, sustained
        load, or "commanded but not turning" LIMPS the whole robot
        immediately (``run_safe_zero``).

        ``dry_run=True`` returns the plan without any motion.
        ``force`` bypasses only the IMU tilt gate — never the
        geometric feasibility or wrong-zero refusals.
        """
        try:
            import math as _math
            from safe_zero import belly_ground_z_mm, plan_safe_zero
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        if self.drive.dry_run:
            return {"ok": True, "dry_run": True}
        bus = self.drive.bus
        if not bus:
            return {"ok": False, "error": "no bus"}

        present, missing = self._present_pose18()
        if missing:
            return {"ok": False,
                    "error": ("no encoder reading from " + ", ".join(
                        joint_label(j, self.names) for j in missing)
                        + " — safe zero needs all 18 joints")}

        # Tilt gate: the planner's ground model assumes a roughly
        # level body (belly-down or standing on its feet).
        tilt = None
        pinned = None
        try:
            if hasattr(bus, "read_imu"):
                imu = bus.read_imu()
                if imu:
                    ax = float(imu.get("ax_g", 0.0))
                    ay = float(imu.get("ay_g", 0.0))
                    az = float(imu.get("az_g", 0.0))
                    roll = _math.degrees(_math.atan2(ay, az))
                    pitch = _math.degrees(
                        _math.atan2(-ax, _math.hypot(ay, az)))
                    tilt = max(abs(roll), abs(pitch))
        except Exception:
            tilt = None
        if tilt is not None:
            # Tipped over a folded knee (THE post-fall state, 08-11)?
            # Then safe zero knows how to proceed: the worker runs the
            # low-torque untrap fold before any planned stage, so a
            # bare refusal here would just push the caller to retry
            # stand/walk against the pin instead. Classify on EVERY
            # call (pure math on data already in hand) — the first
            # live pinned test rested at only 13° tilt, well under
            # this endpoint's 20° hard gate, and a pinned pose can
            # also defeat the preview planner below.
            try:
                from pinned_tip import classify_pinned_tip
                pinned = classify_pinned_tip(present, roll, pitch)
            except Exception:
                pinned = None
            if (tilt > 20.0 and not (pinned and pinned.get("pinned"))
                    and not force):
                return {"ok": False, "tilt_deg": round(tilt, 1),
                        **({"pinned_tip": pinned} if pinned else {}),
                        "error": (f"body tilted {tilt:.0f}° with legs "
                                  "near straight — on a slope or "
                                  "hand-placed? Safe zero assumes "
                                  "roughly level. Right the robot (or "
                                  "force=true while watching).")}

        plan = plan_safe_zero(present, ground_z_mm=belly_ground_z_mm())
        plan["present_deg"] = [round(v, 2) for v in present]
        if tilt is not None:
            plan["tilt_deg"] = round(tilt, 1)
        if pinned and pinned.get("pinned"):
            plan["pinned_tip"] = pinned
            if not plan.get("ok"):
                # A tipped pose can defeat the preview planner (crossed
                # legs); the worker re-plans on fresh encoders AFTER the
                # untrap fold, so this preview must not block motion.
                plan = {"ok": True, "stages": [], "pinned_tip": pinned,
                        "present_deg": plan["present_deg"],
                        "tilt_deg": plan.get("tilt_deg"),
                        "notes": ["pinned-leg tip: low-torque untrap "
                                  "fold first, then re-plan"]}
        if dry_run or not plan.get("ok"):
            plan["dry_run"] = bool(dry_run)
            return plan
        if not plan["stages"] and not (pinned and pinned.get("pinned")):
            return {**plan, "msg": "already at zero"}

        if self._demo_thread and self._demo_thread.is_alive():
            if not self._preempt_demo_thread(reason="→ safe zero",
                                             timeout=5.0):
                return {"ok": False,
                        "error": ("previous job did not stop — "
                                  "try Stop / E-STOP"),
                        "robot": self.robot_state()}

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = "safe_zero"
            self._demo_status = "safe zero"
            self._demo_params = {"stages": len(plan["stages"]),
                                 "force": bool(force)}
            self._cal_result = None
            self._cal_progress = {"msg": "safe zero: starting"}
        self._set_activity("zeroing", "safe zero")

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True
            result: dict = {}
            try:
                from event_log import emit
                emit("safe_zero",
                     f"start ({len(plan['stages'])} stages, "
                     f"{plan.get('total_s')}s)",
                     data={"stages": [s["label"]
                                      for s in plan["stages"]]})
            except Exception:
                pass
            try:
                self._bus_hot = True

                def _prog(dct: dict) -> None:
                    with self._lock:
                        self._cal_progress = dct

                # _safe_zero_sync re-plans on fresh encoders: a limp
                # robot may have sagged between the HTTP call and
                # torque-on.
                result = self._safe_zero_sync(
                    abort_check=self._demo_abort.is_set,
                    on_progress=_prog)
                if result.get("already_at_zero"):
                    result.setdefault("msg", "already at zero")
            except Exception as e:
                result = {"ok": False, "error": str(e)}
            finally:
                self._bus_hot = False
                if gen != self._demo_gen:
                    return
                limp = bool(result.get("limp"))
                with self._lock:
                    self._cal_result = result
                    if result.get("ok"):
                        self._demo_status = "done · at zero (safe)"
                    elif result.get("aborted"):
                        self._demo_status = "aborted (holding)"
                    else:
                        self._demo_status = str(
                            result.get("error") or "error")
                    self._cal_progress = {"msg": self._demo_status}
                    st = self._demo_status
                with d._lock:
                    if d.mode == "demo":
                        d.mode = "idle"
                    if limp:
                        d.armed = False
                        d.status = st
                try:
                    from event_log import emit
                    emit("safe_zero",
                         "done" if result.get("ok") else st,
                         data={k: result.get(k)
                               for k in ("ok", "limp", "stage", "peak_a",
                                         "peak_joint")
                               if k in result})
                except Exception:
                    pass
                self._set_activity(
                    "limp" if (limp or not d.armed) else "armed", st)

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {
            "ok": True, "started": True,
            "plan": {"stages": [{"label": s["label"],
                                 "seconds": s["seconds"]}
                                for s in plan["stages"]],
                     "total_s": plan.get("total_s"),
                     "notes": plan.get("notes") or []},
            "demo": self.demo_state(),
            "robot": self.robot_state(),
        }

    # -- step calibrate (cmd vs encoder) -------------------------------------
    def calibrate_state(self) -> dict:
        with self._lock:
            result = dict(self._cal_result) if self._cal_result else None
            progress = dict(self._cal_progress)
        # rl_policy_* and rl_probe_* jobs share the same worker slot and
        # progress/result plumbing — report them as running too, or their
        # pollers see running=false mid-job and give up.
        running = bool(self._demo_thread and self._demo_thread.is_alive()
                       and (self._demo_name or "").startswith(
                           ("calibrate", "rl_", "standup_", "measure_")))
        plant = self.plant_state()
        imu = self.imu_state()
        return {
            "running": running,
            "name": self._demo_name,
            "progress": progress,
            "result": result,
            "plant": plant,
            "imu": imu,
            "demo": self.demo_state(),
            "robot": self.robot_state(),
        }

    def plant_state(self) -> dict:
        try:
            from plant_calibrate import plant_state
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        return plant_state()

    def imu_state(self) -> dict:
        try:
            from imu_calibrate import imu_state
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        return imu_state()

    def reset_plant(self) -> dict:
        try:
            from plant_calibrate import reset_plant_pose
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        out = reset_plant_pose()
        out["ok"] = True
        return out

    def reset_imu(self) -> dict:
        try:
            from imu_calibrate import reset_imu_calib
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        out = reset_imu_calib()
        out["ok"] = True
        bus = getattr(self.drive, "bus", None)
        reload = getattr(bus, "reload_imu_calib", None) if bus else None
        if callable(reload):
            try:
                reload()
            except Exception:
                pass
        return out

    def run_calibrate(self, *, mode: str = "step",
                      step_deg: float = 10.0,
                      nudge_deg: float = 2.0,
                      axis: str = "all",
                      clearance_mm: float = 40.0,
                      force: bool = False) -> dict:
        """Background step, shake/hold, plant-height, geometry plant, or IMU."""
        mode = (mode or "step").strip().lower()
        if mode in ("hold", "hunt"):
            mode = "shake"
        if mode in ("plant", "plant_height", "height", "stand_height"):
            mode = "plant"
        if mode in ("geometry", "geometry_plant", "geo_plant", "rl_plant"):
            mode = "geometry"
        if mode in ("imu", "mpu", "gyro", "accel"):
            mode = "imu"

        if mode == "plant":
            try:
                from plant_calibrate import run_plant_calibrate
            except ImportError as e:
                return {"ok": False, "error": f"plant_calibrate missing: {e}"}
        elif mode == "geometry":
            if not force:
                return {
                    "ok": False,
                    "error": (
                        "geometry plant disabled without force=true "
                        "(2026-08-06 incident). Prefer capture_plant."
                    ),
                }
            try:
                from geometry_plant import run_geometry_plant
            except ImportError as e:
                return {"ok": False, "error": f"geometry_plant missing: {e}"}
        elif mode == "imu":
            try:
                from imu_calibrate import run_imu_calibrate
            except ImportError as e:
                return {"ok": False, "error": f"imu_calibrate missing: {e}"}
        else:
            try:
                from joint_calibrate import run_calibrate
            except ImportError as e:
                return {"ok": False, "error": f"joint_calibrate missing: {e}"}

        if self.drive.dry_run:
            return {"ok": False, "error": "dry-run — no bus"}
        if not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            if not self._preempt_demo_thread(
                    reason="→ calibrate", timeout=5.0):
                return {"ok": False,
                        "error": "previous demo did not stop — try Stop",
                        "calibrate": self.calibrate_state()}

        try:
            step_deg = float(step_deg)
        except (TypeError, ValueError):
            step_deg = 10.0
        try:
            nudge_deg = float(nudge_deg)
        except (TypeError, ValueError):
            nudge_deg = 2.0
        axis = (axis or "all").strip().lower()
        if mode not in ("step", "shake", "plant", "geometry", "imu"):
            mode = "step"
        try:
            clearance_mm = float(clearance_mm)
        except (TypeError, ValueError):
            clearance_mm = 40.0

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        if mode == "plant":
            label = "plant height (contact reach)"
        elif mode == "geometry":
            label = f"geometry plant (hip≈0 / knee≈90, +{clearance_mm:.0f}mm)"
        elif mode == "imu":
            label = "IMU rest (hold still)"
        elif mode == "shake":
            label = f"shake +{nudge_deg:.1f}° hold ({axis})"
        else:
            label = f"step +{step_deg:.0f}° ({axis})"
        with self._lock:
            self._demo_name = f"calibrate:{mode}:{axis}"
            self._demo_status = "calibrating"
            self._demo_params = {
                "mode": mode, "step_deg": step_deg,
                "nudge_deg": nudge_deg, "axis": axis,
                "clearance_mm": clearance_mm,
            }
            self._cal_result = None
            self._cal_progress = {"msg": "starting…"}
        self._set_activity("calibrating", label)

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
                # IMU rest calib does not need torque; leave limp alone.
                if mode != "imu" and not d.armed:
                    d._torque_all(True)
                    d.armed = True

            def _on_progress(p: dict) -> None:
                with self._lock:
                    self._cal_progress = dict(p)
                    self._demo_status = str(p.get("msg") or "calibrating")

            try:
                if mode == "plant":
                    result = run_plant_calibrate(
                        d.bus,
                        names=self.names,
                        abort_check=self._demo_abort.is_set,
                        on_progress=_on_progress,
                    )
                elif mode == "geometry":
                    result = run_geometry_plant(
                        d.bus,
                        abort_check=self._demo_abort.is_set,
                        on_progress=_on_progress,
                        clearance_mm=clearance_mm,
                    )
                elif mode == "imu":
                    result = run_imu_calibrate(
                        d.bus,
                        abort_check=self._demo_abort.is_set,
                        on_progress=_on_progress,
                    )
                else:
                    result = run_calibrate(
                        d.bus,
                        mode=mode,
                        step_deg=step_deg,
                        nudge_deg=nudge_deg,
                        axis=axis,
                        names=self.names,
                        abort_check=self._demo_abort.is_set,
                        on_progress=_on_progress,
                    )
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = result
                    if self._demo_abort.is_set() or result.get("aborted"):
                        self._demo_status = "aborted"
                    elif result.get("ok") and mode == "plant":
                        if result.get("saved"):
                            self._demo_status = (
                                f"done · plant hip {result.get('hip_deg')}° / "
                                f"knee {result.get('knee_deg')}°"
                            )
                        else:
                            self._demo_status = (
                                "done · no contact (plant not saved)"
                            )
                    elif result.get("ok") and mode == "geometry":
                        self._demo_status = (
                            f"done · geo plant hip {result.get('hip_deg')}° / "
                            f"knee {result.get('knee_deg')}°"
                        )
                    elif result.get("ok") and mode == "imu":
                        g = result.get("grade") or "?"
                        if result.get("saved"):
                            self._demo_status = f"done · IMU {g} (saved)"
                        else:
                            self._demo_status = f"done · IMU {g} (not saved)"
                    elif result.get("ok"):
                        c = result.get("counts") or {}
                        self._demo_status = (
                            f"done · {c.get('green', 0)}g/"
                            f"{c.get('yellow', 0)}y/{c.get('red', 0)}r"
                        )
                    else:
                        self._demo_status = (
                            f"error: {result.get('error') or 'failed'}"
                        )
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = {"ok": False, "error": str(e)}
                    self._demo_status = f"error: {e}"
            finally:
                if gen != self._demo_gen:
                    return
                with d._lock:
                    if d.mode == "demo":
                        d.mode = "idle"
                with self._lock:
                    st = self._demo_status
                self._set_activity(
                    "armed" if d.armed else "limp",
                    st if st else "calibrate done")

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "calibrate": self.calibrate_state()}

    def stop_calibrate(self) -> dict:
        """Alias for stop_demo (same worker slot)."""
        out = self.stop_demo()
        out["calibrate"] = self.calibrate_state()
        return out

    # -- RL / agent HTTP surface (prefer this over SSH) ---------------------
    def rl_state(self) -> dict:
        """Pose + plant + activity in one JSON blob for agents / UI."""
        return {
            "ok": True,
            "service": "hexapod-web",
            "pose": self.pose(),
            "plant": self.plant_state(),
            "imu": self.imu_state(),
            "calibrate": self.calibrate_state(),
            "robot": self.robot_state(),
            "drive": self.rl_drive_state(),
        }

    def rl_find_plant(self, *, clearance_mm: float = 40.0,
                      force: bool = False) -> dict:
        """Start geometry+contact plant finder (async). Poll ``rl_state``.

        Disabled unless ``force=true`` — 2026-08-06 unsupervised plant
        blends tipped/browned-out and cooked a knee servo.
        """
        if not force:
            return {
                "ok": False,
                "error": (
                    "find_plant disabled without force=true. "
                    "Hand-set a low stance, set-zero-here if needed, "
                    "capture_plant — do not auto-stand."
                ),
            }
        return self.run_calibrate(mode="geometry", clearance_mm=clearance_mm)

    def rl_capture_plant(self) -> dict:
        """Save current 18-joint pose as plant (no motion)."""
        from feetech_bus import save_plant_pose
        import statistics as _stats

        if self._demo_thread and self._demo_thread.is_alive():
            return {"ok": False, "error": "stop the running job first"}
        d = self.drive
        if d.dry_run or not d.bus:
            return {"ok": False, "error": "no bus"}
        samples: list[list[float]] = []
        for _ in range(6):
            row: list[float] = []
            ok = True
            with d._lock:
                bus = d.bus
            for j in range(18):
                try:
                    v = bus.read_position_deg(j)
                except Exception:
                    v = None
                if v is None:
                    ok = False
                    break
                row.append(float(v))
            if ok:
                samples.append(row)
            time.sleep(0.04)
        if not samples:
            return {"ok": False, "error": "could not read joints"}
        q = []
        for j in range(18):
            q.append(float(_stats.median([s[j] for s in samples])))
        hips = [q[i] for i in range(1, 18, 3)]
        knees = [q[i] for i in range(2, 18, 3)]
        path = save_plant_pose(
            float(_stats.median(hips)), float(_stats.median(knees)),
            extra={
                "joints_deg": [round(x, 3) for x in q],
                "source": "api_capture",
                "contact_found": True,
            },
        )
        return {
            "ok": True,
            "path": str(path),
            "hip_deg": round(float(_stats.median(hips)), 3),
            "knee_deg": round(float(_stats.median(knees)), 3),
            "joints_deg": [round(x, 3) for x in q],
            "plant": self.plant_state(),
        }

    def rl_stop(self) -> dict:
        """Abort calibrate / find_plant / demo worker."""
        return self.stop_calibrate()

    def rl_probe_dynamics(self, *, amp_deg: float = 10.0,
                          axis: str = "all",
                          soft_torque: int = 350) -> dict:
        """Air-only per-joint step probe → motor_model.json (async)."""
        try:
            from motor_dynamics import run_motor_dynamics
        except ImportError as e:
            return {"ok": False, "error": f"motor_dynamics missing: {e}"}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            if not self._preempt_demo_thread(reason="→ dynamics", timeout=5.0):
                return {"ok": False, "error": "previous job still running"}

        try:
            amp_deg = float(amp_deg)
        except (TypeError, ValueError):
            amp_deg = 10.0
        try:
            soft_torque = int(soft_torque)
        except (TypeError, ValueError):
            soft_torque = 450
        axis = (axis or "all").strip().lower()

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = "rl_probe_dynamics"
            self._demo_status = f"dynamics ±{amp_deg:.0f}° ({axis})"
            self._demo_params = {
                "amp_deg": amp_deg, "axis": axis,
                "soft_torque": soft_torque,
            }
            self._cal_result = None
            self._cal_progress = {"msg": self._demo_status}
        self._set_activity("calibrating", self._demo_status)

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True

            def _on_progress(p: dict) -> None:
                with self._lock:
                    self._cal_progress = dict(p)
                    self._demo_status = str(p.get("msg") or "dynamics")

            try:
                result = run_motor_dynamics(
                    d.bus,
                    amp_deg=amp_deg,
                    axis=axis,
                    soft_torque=soft_torque,
                    names=self.names,
                    abort_check=self._demo_abort.is_set,
                    on_progress=_on_progress,
                )
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = result
                    if self._demo_abort.is_set() or result.get("aborted"):
                        self._demo_status = "aborted"
                    elif result.get("ok"):
                        self._demo_status = (
                            f"done · {result.get('joints_ok')}/"
                            f"{result.get('joints_tested')} ok"
                        )
                    else:
                        self._demo_status = (
                            f"error: {result.get('error') or result.get('msg')}"
                        )
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = {"ok": False, "error": str(e),
                                       "mode": "dynamics"}
                    self._demo_status = f"error: {e}"
            finally:
                if gen != self._demo_gen:
                    return
                # Probe limp's the bus; keep drive disarmed.
                with d._lock:
                    d.armed = False
                    if d.mode == "demo":
                        d.mode = "idle"
                with self._lock:
                    st = self._demo_status
                self._set_activity("limp", st or "dynamics done")

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "calibrate": self.calibrate_state()}

    def sysid_run(self, protocol: dict, *, force: bool = False) -> dict:
        """Run a sysid protocol (deterministic command stream, async).

        Body-supplied protocol JSON (see ``sysid_protocol.py``); logged
        to ``logs/sysid_<name>_<stamp>.csv`` with per-tick send/recv
        timestamps. Whole-body ``traj`` segments additionally require
        ``force=true`` (and the runner's own start-pose gate).
        """
        try:
            from sysid_protocol import duration_s, validate
            from sysid_runner import run_sysid_protocol
        except ImportError as e:
            return {"ok": False, "error": f"sysid modules missing: {e}"}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if not isinstance(protocol, dict):
            return {"ok": False, "error": "protocol must be a JSON object"}
        errs = validate(protocol)
        if errs:
            return {"ok": False,
                    "error": "invalid protocol: " + "; ".join(errs)}
        if self._demo_thread and self._demo_thread.is_alive():
            if not self._preempt_demo_thread(reason="→ sysid", timeout=5.0):
                return {"ok": False, "error": "previous job still running"}

        name = str(protocol.get("name", "unnamed"))
        secs = duration_s(protocol)
        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = "sysid_run"
            self._demo_status = f"sysid '{name}' ({secs:.0f}s)"
            self._demo_params = {"name": name, "force": bool(force),
                                 "duration_s": round(secs, 1),
                                 "segments": len(protocol.get("segments",
                                                              []))}
            self._cal_result = None
            self._cal_progress = {"msg": self._demo_status}
        self._set_activity("calibrating", self._demo_status)

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True

            def _on_progress(p: dict) -> None:
                with self._lock:
                    self._cal_progress = dict(p)
                    self._demo_status = str(p.get("msg") or "sysid")

            try:
                result = run_sysid_protocol(
                    d.bus,
                    protocol,
                    force=bool(force),
                    abort_check=self._demo_abort.is_set,
                    on_progress=_on_progress,
                )
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = result
                    if self._demo_abort.is_set() or result.get("aborted"):
                        self._demo_status = "aborted"
                    elif result.get("ok"):
                        self._demo_status = (
                            f"done · {result.get('ticks_done')}/"
                            f"{result.get('ticks_planned')} ticks")
                    else:
                        self._demo_status = f"error: {result.get('error')}"
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = {"ok": False, "error": str(e),
                                       "mode": "sysid"}
                    self._demo_status = f"error: {e}"
            finally:
                if gen != self._demo_gen:
                    return
                # Runner limps the bus; keep drive disarmed.
                with d._lock:
                    d.armed = False
                    if d.mode == "demo":
                        d.mode = "idle"
                with self._lock:
                    st = self._demo_status
                self._set_activity("limp", st or "sysid done")

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "name": name, "duration_s": round(secs, 1),
                "calibrate": self.calibrate_state()}

    def rl_preflight(self, *, mode: str = "stand") -> dict:
        """Read-only readiness check for the RL stand/lower/walk buttons."""
        mode = (mode or "stand").strip().lower()
        if mode not in ("stand", "lower", "walk"):
            return {"ok": False, "error": f"bad mode {mode!r}"}
        try:
            from rl_policy import preflight
        except ImportError as e:
            return {"ok": False, "error": f"rl_policy missing: {e}"}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        ok, reason, details = preflight(self.drive.bus, mode)
        out = {"ok": ok, "mode": mode, **details}
        if not ok:
            out["error"] = reason
        return out

    def rl_feedback(self) -> dict:
        """One-shot telemetry: 18-joint bulk feedback + IMU tilt. Read-only.

        ONE MCU bulk transaction (``read_all_feedback``) + one IMU read —
        a few Hz sustainable even while the drive loop walks, unlike
        ``/api/status`` whose 1..31 scan takes seconds. Built for external
        telemetry loggers (``rl_move/scripts/tape_measure_walk.py``).
        ``joints`` is indexed 0..17; missing servos are null.
        """
        import math as _math

        d = self.drive
        if d.dry_run or not d.bus:
            return {"ok": False, "error": "no bus"}
        bus = d.bus
        try:
            fb = bus.read_all_feedback()
        except Exception as e:
            return {"ok": False, "error": f"feedback: {e}"}
        joints: list[dict | None] = []
        for j in range(N_JOINTS):
            f = fb.get(j)
            joints.append(None if f is None else {
                "deg": round(float(f.get("deg", 0.0)), 2),
                "cur_a": round(float(f.get("current_a", 0.0)), 3),
                "temp_c": int(f.get("temp_c") or 0),
                "load_pct": round(float(f.get("load_pct", 0.0)), 1),
                "volt": round(float(f.get("volt", 0.0)), 2),
            })
        out: dict = {"ok": True, "t_unix": round(time.time(), 3),
                     "live": len(fb), "joints": joints}
        try:
            imu = bus.read_imu(apply_calib=True)
        except Exception:
            imu = None
        if isinstance(imu, dict) and "ax_g" in imu:
            roll = _math.degrees(_math.atan2(imu["ay_g"], imu["az_g"]))
            pitch = _math.degrees(_math.atan2(
                -imu["ax_g"], _math.hypot(imu["ay_g"], imu["az_g"])))
            out["roll_deg"] = round(roll, 2)
            out["pitch_deg"] = round(pitch, 2)
            out["gyro_dps"] = [round(float(imu.get(k, 0.0)), 2)
                               for k in ("gx_dps", "gy_dps", "gz_dps")]
        return out

    def rl_policy_info(self) -> dict:
        """Metadata of the deployed policy weights (no bus traffic)."""
        try:
            from rl_policy import WEIGHTS_PATH, WALK_WEIGHTS_PATH
            meta = json.loads(Path(WEIGHTS_PATH).read_text())["meta"]
            out = {"ok": True, **meta}
            try:
                walk = json.loads(
                    Path(WALK_WEIGHTS_PATH).read_text())["meta"]
                out["walk"] = walk
            except Exception as e:
                out["walk"] = {"error": str(e)}
            return out
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # Swappable policy registry (operator request 08-10): exported
    # weight JSONs live in linux_control/policies/; selecting one
    # atomically copies it over the live rl_policy_weights.json /
    # rl_walk_weights.json. No restart needed — run_policy_move loads
    # weights fresh at every episode start. Slot is inferred from the
    # obs dim (68 = stance stand/lower, 72 = walk).
    POLICIES_DIR = Path(__file__).resolve().parent / "policies"
    # obs 74 = walk + phase clock (sin/cos appended by the runner; the
    # cw-arch-noslipphase1 no-slip line). Same walk slot.
    _SLOT_OBS = {68: "stance", 72: "walk", 74: "walk"}

    def _policy_slot_targets(self) -> dict:
        from rl_policy import WALK_WEIGHTS_PATH, WEIGHTS_PATH
        return {"stance": Path(WEIGHTS_PATH), "walk": Path(WALK_WEIGHTS_PATH)}

    def rl_policies(self) -> dict:
        """List swappable policies + which one is live in each slot."""
        import hashlib

        def _md5(p: Path):
            try:
                return hashlib.md5(p.read_bytes()).hexdigest()
            except Exception:
                return None

        active = {slot: _md5(p)
                  for slot, p in self._policy_slot_targets().items()}
        out = []
        for f in sorted(self.POLICIES_DIR.glob("*.json")):
            try:
                meta = json.loads(f.read_text())["meta"]
            except Exception as e:
                out.append({"file": f.name, "error": str(e)})
                continue
            slot = self._SLOT_OBS.get(meta.get("obs_dim"))
            out.append({
                "file": f.name,
                "name": meta.get("name") or f.stem,
                "slot": slot,
                "obs_dim": meta.get("obs_dim"),
                "source": (meta.get("source") or "").rsplit("/", 1)[-1],
                "notes": meta.get("notes", ""),
                "active": slot is not None and _md5(f) == active.get(slot),
            })
        return {"ok": True, "dir": str(self.POLICIES_DIR), "policies": out}

    def rl_policy_select(self, *, file: str = "") -> dict:
        """Make policies/<file> the live policy for its slot (no motion)."""
        import os

        if self._demo_thread and self._demo_thread.is_alive():
            return {"ok": False, "error": "stop the running job first"}
        name = Path(str(file)).name          # forbid path traversal
        src = self.POLICIES_DIR / name
        if not src.is_file():
            return {"ok": False, "error": f"no such policy file: {name}"}
        try:
            payload = src.read_text()
            meta = json.loads(payload)["meta"]
        except Exception as e:
            return {"ok": False, "error": f"unreadable policy: {e}"}
        slot = self._SLOT_OBS.get(meta.get("obs_dim"))
        if slot is None:
            return {"ok": False,
                    "error": (f"obs_dim {meta.get('obs_dim')} fits no slot "
                              f"(68 = stance, 72/74 = walk)")}
        dst = self._policy_slot_targets()[slot]
        tmp = dst.with_name(dst.name + ".tmp")
        tmp.write_text(payload)
        os.replace(tmp, dst)
        try:
            from event_log import emit
            emit("rl_policy_select", f"{slot} <- {name}", src="bench",
                 data={"slot": slot, "file": name,
                       "source": meta.get("source", "")})
        except Exception:
            pass
        return {"ok": True, "slot": slot, "file": name,
                "name": meta.get("name") or src.stem,
                "source": (meta.get("source") or "").rsplit("/", 1)[-1]}

    # Role registry (operator 08-11, MuJoCo-style driving): which
    # policies/<file> serves each FUNCTION. One file can hold several
    # roles — a walk champion that stops cleanly can be both "walk" and
    # "hold"; the rise specialist and the stance champion can split
    # "stand" and "lower". Stored on the board's home dir (like
    # ~/.hexapod_cal.json), NOT in the repo. A role of None keeps the
    # pre-roles behavior: the live slot file (rl_policy_select). The
    # special hold value "walk" (the default) means "the walk policy at
    # zero command" — its trained stop, no model switch at all.
    ROLES_FILE = Path.home() / ".hexapod_rl_roles.json"
    _ROLE_OBS = {"walk": (72, 74), "hold": (68, 72, 74),
                 "stand": (68,), "lower": (68,)}

    def _roles(self) -> dict:
        roles = {"walk": None, "hold": "walk", "stand": None,
                 "lower": None}
        try:
            d = json.loads(self.ROLES_FILE.read_text())
            for k in roles:
                if k in d:
                    roles[k] = d[k]
        except Exception:
            pass
        return roles

    def _role_weights(self, role: str) -> Path | None:
        """Weights-file override for a role; None = default behavior
        (the live slot file, or walk@zero for the hold role)."""
        v = self._roles().get(role)
        if not v or v == "walk":
            return None
        p = self.POLICIES_DIR / Path(str(v)).name
        return p if p.is_file() else None

    def rl_roles(self) -> dict:
        """Current role assignments + what each resolves to (no bus)."""
        roles = self._roles()
        out = {}
        for role, v in roles.items():
            p = self._role_weights(role)
            if p is not None:
                try:
                    meta = json.loads(p.read_text())["meta"]
                    resolved = meta.get("name") or p.stem
                except Exception:
                    resolved = p.name
            elif role == "hold":
                resolved = "walk policy @ zero command"
            else:
                slot = "walk" if role == "walk" else "stance"
                resolved = f"live {slot} slot"
            out[role] = {"file": v, "resolved": resolved}
        return {"ok": True, "roles": out,
                "allowed_obs": {k: list(v)
                                for k, v in self._ROLE_OBS.items()}}

    def rl_role_set(self, *, role: str = "", file: str = "") -> dict:
        """Assign policies/<file> to a role (no motion; takes effect at
        the next episode / session start). file="" resets to default;
        file="walk" (hold role only) = walk policy at zero command."""
        role = (role or "").strip().lower()
        if role not in self._ROLE_OBS:
            return {"ok": False,
                    "error": f"bad role {role!r} (walk/hold/stand/lower)"}
        val: str | None
        if not file:
            val = "walk" if role == "hold" else None
        elif file == "walk":
            if role != "hold":
                return {"ok": False,
                        "error": "'walk' shorthand is hold-role only"}
            val = "walk"
        else:
            name = Path(str(file)).name        # forbid path traversal
            p = self.POLICIES_DIR / name
            if not p.is_file():
                return {"ok": False, "error": f"no such policy: {name}"}
            try:
                meta = json.loads(p.read_text())["meta"]
            except Exception as e:
                return {"ok": False, "error": f"unreadable policy: {e}"}
            if meta.get("obs_dim") not in self._ROLE_OBS[role]:
                return {"ok": False,
                        "error": (f"{name} (obs {meta.get('obs_dim')}) "
                                  f"does not fit role {role} "
                                  f"(needs obs {self._ROLE_OBS[role]})")}
            val = name
        roles = self._roles()
        roles[role] = val
        try:
            tmp = self.ROLES_FILE.with_name(self.ROLES_FILE.name + ".tmp")
            tmp.write_text(json.dumps(roles, indent=1))
            tmp.replace(self.ROLES_FILE)
        except OSError as e:
            return {"ok": False, "error": f"could not save roles: {e}"}
        try:
            from event_log import emit
            emit("rl_role_set", f"{role} <- {val or 'default'}",
                 src="bench", data={"role": role, "file": val})
        except Exception:
            pass
        return {"ok": True, **self.rl_roles()}

    # -- Live drive session (held-arrow-key driving, operator 08-11) ----
    def _drive_active(self) -> bool:
        with self._lock:
            name = self._demo_name
        return (self._drive_cmd is not None
                and self._demo_thread is not None
                and self._demo_thread.is_alive()
                and name == "rl_drive")

    def rl_drive_state(self) -> dict:
        """Session snapshot for the UI (no bus traffic)."""
        active = self._drive_active()
        out: dict = {"ok": True, "active": active}
        cmd = self._drive_cmd
        if cmd is not None:
            out["live"] = cmd.live
        with self._lock:
            if active:
                out["status"] = self._demo_status
            elif self._demo_name == "rl_drive":
                out["result"] = self._cal_result
        return out

    def rl_drive_cmd(self, *, vx: float = 0.0, vy: float = 0.0) -> dict:
        """Heartbeat from the browser: body-frame (vx, vy) m/s while
        keys are held, (0, 0) when released. Never touches the bus —
        the 25 Hz session loop reads it. Stale heartbeats (> 0.6 s)
        decay to zero server-side, so this must keep streaming."""
        if not self._drive_active():
            return {"ok": False, "error": "no drive session", "active": False}
        self._drive_cmd.set(float(vx), float(vy))
        with self._lock:
            status = self._demo_status
        return {"ok": True, "active": True, "status": status,
                "live": self._drive_cmd.live}

    def rl_drive_stop(self) -> dict:
        """Graceful end: refs ramp to zero, robot HOLDS the pose."""
        if self._drive_cmd is not None:
            self._drive_cmd.request_stop()
        return self.rl_drive_state()

    def rl_drive_start(self) -> dict:
        """Start a persistent RL drive session (async, demo slot).

        Same start contract as the episodic walk: read-only preflight
        from the captured plant stance, with the moderate-tilt/pose
        auto-acquire (scripted stand) fallback. Then the loop runs
        until stop / heartbeat silence / cap / safety trip, driven
        live by rl_drive_cmd. THE OPERATOR MUST BE WATCHING.
        """
        try:
            from rl_policy import DriveCommand, preflight, run_drive_session
        except ImportError as e:
            return {"ok": False, "error": f"rl_policy missing: {e}"}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            if self._drive_active():
                return {"ok": True, "already": True,
                        **self.rl_drive_state()}
            return {"ok": False, "error": "stop the running job first"}

        walk_w = self._role_weights("walk")
        hold_w = self._role_weights("hold")

        ok, reason, details = preflight(self.drive.bus, "walk")
        acquire_first = False
        if not ok and (reason.startswith("pose is not")
                       or reason.startswith("tilt too high")):
            # Same moderate-tilt descent rule as rl_policy_move: a
            # sprawled-but-level robot may acquire the stand first; a
            # truly tipped or half-dead robot always refuses.
            r = abs(float(details.get("roll_deg", 90.0)))
            p = abs(float(details.get("pitch_deg", 90.0)))
            if r <= 35.0 and p <= 35.0:
                acquire_first = True
            else:
                return {"ok": False,
                        "error": f"preflight: {reason}", **details}
        elif not ok:
            return {"ok": False, "error": f"preflight: {reason}", **details}

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        cmd = DriveCommand()
        self._drive_cmd = cmd
        with self._lock:
            self._demo_name = "rl_drive"
            self._demo_status = "drive session starting"
            self._demo_params = {
                "walk": walk_w.name if walk_w else "slot",
                "hold": hold_w.name if hold_w else "walk@zero"}
            self._cal_result = None
            self._cal_progress = {"msg": self._demo_status}
        self._set_activity("rl_policy", "RL drive")

        def _worker():
            d = self.drive

            def _on_progress(p: dict) -> None:
                with self._lock:
                    self._cal_progress = dict(p)
                    self._demo_status = str(p.get("msg") or "RL drive")

            try:
                if acquire_first:
                    with d._lock:
                        d.mode = "demo"
                        if not d.armed:
                            d._torque_all(True)
                            d.armed = True
                    _on_progress({"msg": "acquiring stand start pose…"})
                    res_a = self._acquire_start(
                        "stand", gen=gen, on_progress=_on_progress)
                    if not res_a.get("ok"):
                        raise RuntimeError(
                            "could not reach the start pose — "
                            + str(res_a.get("error") or "aborted"))
                    ok2, reason2, _d2 = preflight(d.bus, "walk")
                    if not ok2:
                        raise RuntimeError(
                            "still failing preflight after acquiring "
                            f"stand: {reason2}")
                result = run_drive_session(
                    d, cmd, on_progress=_on_progress,
                    abort_check=self._demo_abort.is_set,
                    walk_weights=walk_w, hold_weights=hold_w)
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = result
                    if result.get("ok"):
                        self._demo_status = (
                            "drive session ended"
                            + (f" — {result['ended']}"
                               if result.get("ended") else "")
                            + f" · maxI {result.get('max_current_a', 0):.2f}A")
                    else:
                        self._demo_status = (
                            f"RL drive: {result.get('error')}")
            except Exception as e:
                if gen != self._demo_gen:
                    return
                try:
                    d.bus.enable_all_torque(False)
                except Exception:
                    pass
                with self._lock:
                    self._cal_result = {"ok": False, "error": str(e),
                                        "mode": "drive"}
                    self._demo_status = f"error: {e}"
            finally:
                if gen != self._demo_gen:
                    return
                res = self._cal_result or {}
                limped = bool(res.get("limped"))
                with d._lock:
                    d.armed = not limped
                    if d.mode == "demo":
                        d.mode = "idle"
                with self._lock:
                    st = self._demo_status
                self._set_activity("limp" if limped else "holding",
                                   st or "drive session done")

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "mode": "drive",
                "walk": walk_w.name if walk_w else "live slot",
                "hold": hold_w.name if hold_w else "walk@zero"}

    def rl_policy_move(self, *, mode: str = "stand", vx: float = 0.03,
                       vy: float = 0.0, duration_s: float = 6.0,
                       rot60: bool = True, turn: str | None = None,
                       tilt_trip_deg: float | None = None,
                       extra_hold_s: float = 0.0) -> dict:
        """Run a trained RL policy: stand up / lower / walk.

        Async (demo-thread slot, poll ``rl_state``, abort via ``rl_stop``).
        Read-only preflight refuses to move unless all 18 servos answer,
        the IMU is alive, and the present pose matches the expected start
        (belly/zero for stand, captured plant for lower AND walk).
        Safety layer trips (tilt / sustained over-current / temp) limp
        immediately. Walk extras: body-frame vx/vy (m/s, clamped to the
        trained 0.06 band) and duration_s (clamped 3..20 s).
        ``turn`` (walk only): "left"/"right" = mirror-selection arc
        turn, "hold" = heading hold; None = the unchanged naked path.
        The OPERATOR MUST BE WATCHING — this is the explicit order.
        """
        mode = (mode or "stand").strip().lower()
        if mode not in ("stand", "lower", "walk"):
            return {"ok": False, "error": f"bad mode {mode!r}"}
        try:
            from rl_policy import preflight, run_policy_move
        except ImportError as e:
            return {"ok": False, "error": f"rl_policy missing: {e}"}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            if self._drive_active():
                # Stand/Sit during a drive session flips models
                # (operator 08-11): end the session — the robot holds
                # its pose — then run the episode from there.
                if not self._preempt_demo_thread(
                        reason=f"drive → {mode}", timeout=8.0):
                    return {"ok": False,
                            "error": "drive session did not stop"}
            else:
                return {"ok": False, "error": "stop the running job first"}
        weights_path = self._role_weights(mode)

        # Preflight before claiming the worker slot so refusals are
        # instant and motion-free.
        ok, reason, details = preflight(self.drive.bus, mode)
        acquire_first: str | None = None
        if not ok and (reason.startswith("pose is not")
                       or reason.startswith("tilt too high")):
            # RL moves from any pose (operator 08-11): when the pose
            # or a MODERATE tilt is all that fails (servos + IMU
            # healthy), the worker ACQUIRES the expected start first —
            # safe zero for RL stand; safe zero + the validated plant
            # stand-up for lower / walk — then re-preflights (strict
            # gates) and runs. A sprawled robot resting crooked on
            # folded legs reads 15-25 deg of tilt — the descent
            # flattens it. A truly tipped robot (roll toward 90)
            # still refuses, as do servo / IMU failures — never
            # auto-move a tipped or half-dead robot.
            r = abs(float(details.get("roll_deg", 90.0)))
            p = abs(float(details.get("pitch_deg", 90.0)))
            if r <= 35.0 and p <= 35.0:
                acquire_first = "zero" if mode == "stand" else "stand"
            else:
                return {"ok": False,
                        "error": f"preflight: {reason}", **details}
        elif not ok:
            return {"ok": False, "error": f"preflight: {reason}", **details}

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        label = {"stand": "RL stand up", "lower": "RL lower",
                 "walk": "RL walk"}[mode]
        with self._lock:
            self._demo_name = f"rl_policy_{mode}"
            self._demo_status = f"{label} starting"
            self._demo_params = {"mode": mode}
            if mode == "walk":
                self._demo_params.update(
                    vx=float(vx), vy=float(vy),
                    duration_s=float(duration_s), rot60=bool(rot60))
                if turn:
                    self._demo_params["turn"] = str(turn)
            else:
                if tilt_trip_deg:
                    self._demo_params["tilt_trip_deg"] = float(tilt_trip_deg)
                if extra_hold_s:
                    self._demo_params["extra_hold_s"] = float(extra_hold_s)
            self._cal_result = None
            self._cal_progress = {"msg": self._demo_status}
        self._set_activity("rl_policy", label)

        def _worker():
            d = self.drive

            def _on_progress(p: dict) -> None:
                with self._lock:
                    self._cal_progress = dict(p)
                    self._demo_status = str(p.get("msg") or label)

            try:
                # 08-12 operator-observed freezes during the RL stand:
                # this worker was the ONLY motion path that never set
                # _bus_hot, so the TFT panel kept repainting mid-episode
                # (a DJ redraw holds the MCU link ~1.5 s — the measured
                # "big pause in the middle of standing"). Same guard as
                # every other motion worker.
                self._bus_hot = True
                if acquire_first:
                    with d._lock:
                        d.mode = "demo"
                        if not d.armed:
                            d._torque_all(True)
                            d.armed = True
                    _on_progress({"msg": (f"acquiring {acquire_first} "
                                          "start pose first…")})
                    res_a = self._acquire_start(
                        acquire_first, gen=gen,
                        on_progress=_on_progress)
                    if not res_a.get("ok"):
                        raise RuntimeError(
                            "could not reach the start pose — "
                            + str(res_a.get("error") or "aborted"))
                    ok2, reason2, _d2 = preflight(d.bus, mode)
                    if not ok2:
                        raise RuntimeError(
                            f"still failing preflight after acquiring "
                            f"{acquire_first}: {reason2}")
                result = run_policy_move(
                    d, mode, on_progress=_on_progress,
                    abort_check=self._demo_abort.is_set,
                    vx=float(vx), vy=float(vy),
                    duration_s=float(duration_s), rot60=bool(rot60),
                    turn=(str(turn) if turn else None),
                    weights_path=weights_path,
                    tilt_trip_deg=(float(tilt_trip_deg)
                                   if tilt_trip_deg else None),
                    extra_hold_s=float(extra_hold_s or 0.0))
                if gen != self._demo_gen:
                    return
                if result.get("ok") and mode == "stand":
                    # Honesty check: the policy 'finishing' its
                    # episode is not the same as standing (08-11: a
                    # nominally-ok run ended legs-flailed at 0.16 A).
                    try:
                        kfs = (self._load_standup()["modes"]["tuck"]
                               ["keyframes"])
                        stance = [float(x) for x in
                                  kfs[-1]["q_deg"]]
                        w_st, _ = self._delta_vs_present(stance)
                        if w_st is not None and w_st > 30.0:
                            result["stood"] = False
                            result["stance_err_deg"] = round(w_st, 1)
                        else:
                            result["stood"] = True
                    except Exception:
                        pass
                with self._lock:
                    self._cal_result = result
                    if result.get("ok") and result.get("stood") is False:
                        self._demo_status = (
                            f"{label} finished but NOT standing "
                            f"(pose {result['stance_err_deg']:.0f} deg "
                            "from stance) — use the scripted stand")
                    elif result.get("ok"):
                        self._demo_status = (
                            f"{label} done · maxI "
                            f"{result.get('max_current_a', 0):.2f}A")
                    else:
                        self._demo_status = (
                            f"{label}: {result.get('error')}")
            except Exception as e:
                if gen != self._demo_gen:
                    return
                try:
                    d.bus.enable_all_torque(False)
                except Exception:
                    pass
                with self._lock:
                    self._cal_result = {"ok": False, "error": str(e),
                                        "mode": mode}
                    self._demo_status = f"error: {e}"
            finally:
                self._bus_hot = False
                if gen != self._demo_gen:
                    return
                res = self._cal_result or {}
                limped = bool(res.get("limped"))
                with d._lock:
                    d.armed = not limped
                    if d.mode == "demo":
                        d.mode = "idle"
                with self._lock:
                    st = self._demo_status
                self._set_activity("limp" if limped else "holding",
                                   st or f"{label} done")

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "mode": mode, "calibrate": self.calibrate_state()}

    def rl_set_stance(self, *, hip_deg: float = -20.0, knee_deg: float = 55.0,
                      seconds: float = 10.0, yaw_deg: float = 0.0,
                      force: bool = False) -> dict:
        """Slow ease to a shared hip/knee stance (async).

        A large Δq from present no longer refuses (08-11 directive):
        the worker first ACQUIRES the zero start safely (collision-
        aware, limps on stall) and eases to the stance from there. If
        acquisition fails the job errors out and the ease never runs.
        Hip≈0°+knee≈80° is stilts — not a low plant.
        """
        try:
            from inplace_demos import (
                _enable_torque, _live_robot_ids, _set_torque_limit, ease_to_pose,
            )
            from drive_controller import MAX_SAFE_DELTA_DEG
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            if not self._preempt_demo_thread(reason="→ set_stance", timeout=5.0):
                return {"ok": False, "error": "previous job still running"}

        hip_deg = max(-80.0, min(30.0, float(hip_deg)))
        knee_deg = max(-20.0, min(80.0, float(knee_deg)))
        yaw_deg = max(-35.0, min(35.0, float(yaw_deg)))
        seconds = max(2.0, min(30.0, float(seconds)))
        goal: list[float] = []
        for _ in range(6):
            goal.extend([yaw_deg, hip_deg, knee_deg])
        acquire_zero_first = False
        if not force:
            worst, j = self._delta_vs_present(goal)
            if worst is None:
                return {"ok": False,
                        "error": ("no encoder readings — cannot check "
                                  "the start pose; retry in a few "
                                  "seconds")}
            if worst > MAX_SAFE_DELTA_DEG:
                # 08-11 directive: acquire the start instead of
                # refusing — safe zero first, then the ease.
                acquire_zero_first = True

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = "rl_set_stance"
            self._demo_status = (
                f"stance hip {hip_deg:+.0f}° / knee {knee_deg:+.0f}°")
            self._demo_params = {
                "hip_deg": hip_deg, "knee_deg": knee_deg,
                "yaw_deg": yaw_deg, "seconds": seconds,
            }
            self._cal_result = None
            self._cal_progress = {"msg": self._demo_status}
        self._set_activity("calibrating", self._demo_status)

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True
            live = _live_robot_ids(d.bus)
            try:
                if acquire_zero_first:
                    res_a = self._acquire_start("zero", gen=gen)
                    if gen != self._demo_gen:
                        return
                    if not res_a.get("ok"):
                        with self._lock:
                            self._demo_status = (
                                "error: start pose not reached — "
                                + str(res_a.get("error") or "aborted"))
                            self._cal_result = {
                                "ok": False, "mode": "set_stance",
                                "error": self._demo_status}
                            self._cal_progress = {
                                "msg": self._demo_status}
                        return
                _set_torque_limit(d.bus, live, 550)
                _enable_torque(d.bus, live)
                ok = ease_to_pose(
                    d.bus, goal, abort_check=self._demo_abort.is_set,
                    seconds=seconds, label="rl_stance")
                if gen != self._demo_gen:
                    return
                with self._lock:
                    if self._demo_abort.is_set() or not ok:
                        self._demo_status = "aborted"
                        self._cal_result = {
                            "ok": False, "aborted": True,
                            "mode": "set_stance",
                        }
                    else:
                        self._demo_status = (
                            f"done · hip {hip_deg:+.0f}° / knee {knee_deg:+.0f}°")
                        self._cal_result = {
                            "ok": True, "mode": "set_stance",
                            "hip_deg": hip_deg, "knee_deg": knee_deg,
                            "yaw_deg": yaw_deg, "goal": goal,
                        }
                        self._cal_progress = {"msg": self._demo_status}
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._demo_status = f"error: {e}"
                    self._cal_result = {"ok": False, "error": str(e),
                                       "mode": "set_stance"}
            finally:
                if gen != self._demo_gen:
                    return
                try:
                    _set_torque_limit(d.bus, live, 1000)
                except Exception:
                    pass
                with d._lock:
                    if d.mode == "demo":
                        d.mode = "idle"
                with self._lock:
                    st = self._demo_status
                self._set_activity(
                    "armed" if d.armed else "limp", st or "stance done")

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "calibrate": self.calibrate_state(),
                "target": {"hip_deg": hip_deg, "knee_deg": knee_deg,
                           "seconds": seconds}}

    # -- stand-up lab ---------------------------------------------------------
    # Sim-validated stand-up strategies (rl_move/sim/compare_standup.py):
    # keyframes baked into standup_modes.json, played back as chained
    # ease_to_pose glides. See the JSON's per-mode descriptions.
    STANDUP_FILE = Path(__file__).resolve().parent / "standup_modes.json"

    def _load_standup(self) -> dict:
        # Read fresh each call (small file) so a re-deployed bake is
        # picked up without restarting the service.
        return json.loads(self.STANDUP_FILE.read_text())

    def standup_modes(self) -> dict:
        """List the available stand-up strategies (web UI selector)."""
        try:
            data = self._load_standup()
        except (OSError, ValueError) as e:
            return {"ok": False, "error": f"standup_modes.json: {e}"}
        return {
            "ok": True,
            "frame": data.get("frame", ""),
            "modes": [
                {"name": name,
                 "description": m.get("description", ""),
                 "keyframes": len(m.get("keyframes", [])),
                 "total_s": m.get("total_s")}
                for name, m in (data.get("modes") or {}).items()],
        }

    def standup(self, *, mode: str = "tuck", speed: float = 1.0,
                direction: str = "up", force: bool = False,
                torque: int = 700, abort_current_a: float = 3.0,
                sync_gen: int | None = None) -> dict:
        """Play one baked stand-up strategy (async).

        ``direction="up"`` starts from the ZERO pose (belly down, legs
        straight out); ``"down"`` plays the same keyframes in reverse —
        from the mode's standing stance back to the belly. When the
        present pose is far from the required start it ACQUIRES it
        first (08-11 directive): up → safe zero first, then the
        keyframes; down + not at stance → the safe descent to zero IS
        the down path. Acquisition failure stops everything with an
        error and the keyframes never play.
        Aborts between keyframes if any servo peaked above
        ``abort_current_a`` (stall-fight = the pinned-feet failure this
        lab exists to fix; do not grind on it).

        ``sync_gen`` (internal): run the whole job INLINE in the
        calling worker thread under the caller's demo generation —
        used by ``_acquire_start`` to chain safe-zero → stand-up. No
        job-slot claim, no preempt, returns the final result dict.

        Hardware truth 08-10: tuck stood at 2.48 A peak, step at
        2.97 A; blend stalled short at only 0.57 A (the servos give up
        quietly under the torque limit — matches the sim's low-torque
        rows). Faster tempos raise push currents toward the guard.
        """
        try:
            from inplace_demos import (
                CurrentPeakTracker, PoseStreamer, _enable_torque,
                _live_robot_ids, _set_torque_limit, _write_pose,
                ease_to_pose,
            )
            from drive_controller import MAX_SAFE_DELTA_DEG
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        try:
            data = self._load_standup()
            if str(mode) == "plant":
                # Synthetic mode: the tuck sequence ending exactly at
                # the LIVE captured plant pose (what RL walk expects)
                # instead of the tibia-vertical display stance. From
                # a standing start it shortcuts to a single-frame
                # path — the worker's align + tripod re-seat carry
                # the robot onto the plant without a full sit/stand.
                from feetech_bus import standing_pose_degrees
                plant = [float(x) for x in standing_pose_degrees()]
                kfs = data["modes"]["tuck"]["keyframes"]
                q_zero = [float(v) for v in kfs[0]["q_deg"]]
                w_plant, _ = self._delta_vs_present(plant)
                w_zero, _ = self._delta_vs_present(q_zero)
                if (w_plant is not None and w_zero is not None
                        and w_plant <= 25.0 and w_plant < w_zero):
                    keyframes = [{"q_deg": plant, "s": 0.4}]
                else:
                    keyframes = (kfs[:-1]
                                 + [{"q_deg": plant, "s": 0.5}])
                direction = "up"
            else:
                m = data["modes"][str(mode)]
                keyframes = m["keyframes"]
        except (OSError, ValueError, KeyError, ImportError) as e:
            return {"ok": False, "error": f"unknown stand-up mode: {e}"}
        if sync_gen is None and (self._demo_thread
                                 and self._demo_thread.is_alive()):
            if not self._preempt_demo_thread(reason="→ standup",
                                             timeout=5.0):
                return {"ok": False, "error": "previous job still running"}

        speed = max(0.25, min(10.0, float(speed)))
        torque = max(300, min(1000, int(torque)))
        down = str(direction) == "down"
        # frames: (18-joint target deg, glide seconds). Reversed playback
        # keeps each segment's duration with its segment: the glide from
        # keyframe i to i-1 takes what i-1 -> i took, plus a short
        # align glide onto the last keyframe first.
        frames = [([float(v) for v in kf["q_deg"]], float(kf["s"]))
                  for kf in keyframes]
        if down:
            qs = [q for q, _ in frames]
            ss = [s for _, s in frames]
            frames = [(qs[-1], 0.8)] + [
                (qs[i], ss[i + 1]) for i in range(len(qs) - 2, -1, -1)]
        first = frames[0][0]
        acquire_zero_first = False
        safe_down_instead = False
        if not force:
            worst, j = self._delta_vs_present(first)
            if worst is None:
                return {"ok": False,
                        "error": ("no encoder readings (bus warming "
                                  "up after restart?) — cannot check "
                                  "the start pose; retry in a few "
                                  "seconds")}
            if worst > MAX_SAFE_DELTA_DEG:
                # 08-11 directive: acquire the start pose instead of
                # refusing. Up → safe zero first. Down from a pose the
                # keyframes weren't baked for → the safe descent IS
                # the down path (never play reversed keyframes from an
                # unknown stance).
                if down:
                    safe_down_instead = True
                else:
                    acquire_zero_first = True

        verb = "sit-down" if down else "stand-up"
        if sync_gen is None:
            self._demo_gen += 1
            gen = self._demo_gen
            self._demo_abort.clear()
            with self._lock:
                self._demo_name = (f"standup_{mode}"
                                   + ("_down" if down else ""))
                self._demo_status = f"{verb} · {mode} (x{speed:.2f})"
                self._demo_params = {"mode": mode, "speed": speed,
                                     "direction": direction,
                                     "torque": torque}
                self._cal_result = None
                self._cal_progress = {"msg": self._demo_status}
            self._set_activity("demo", self._demo_status)
        else:
            gen = int(sync_gen)
            with self._lock:
                self._cal_progress = {
                    "msg": f"{verb} · {mode} (x{speed:.2f})"}

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                d.gait.stop()
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True
            live = _live_robot_ids(d.bus)
            tracker = CurrentPeakTracker()
            result: dict = {"ok": False, "mode": mode,
                            "direction": direction}
            # Worker-local copy: the down path drops the wide frame
            # (kf_path = kf_path[1:]); assigning to the closure name
            # `frames` would make it worker-local and blow up every
            # earlier read (UnboundLocalError — bit us 08-11).
            kf_path = list(frames)
            try:
                from event_log import emit
                emit("standup", f"{mode} {verb} x{speed:g} start",
                     data={"mode": mode, "direction": direction,
                           "speed": speed, "keyframes": len(kf_path),
                           "live_ids": sorted(live)})
            except Exception:
                pass
            try:
                self._bus_hot = True

                def _acq_prog(p: dict) -> None:
                    with self._lock:
                        self._cal_progress = dict(p)

                if safe_down_instead:
                    # Not at this mode's stance — the collision-aware
                    # descent is the whole down job.
                    res = self._safe_zero_sync(
                        abort_check=self._demo_abort.is_set,
                        on_progress=_acq_prog)
                    result.update(res)
                    result["via"] = "safe_zero"
                    result.setdefault("ok", False)
                    if gen != self._demo_gen:
                        return
                    with self._lock:
                        self._cal_result = result
                        self._demo_status = (
                            "done · safe descent to zero (was not at "
                            "stance)" if result.get("ok") else
                            str(result.get("error") or "aborted"))
                        self._cal_progress = {"msg": self._demo_status}
                    return result
                if acquire_zero_first:
                    res_a = self._acquire_start("zero", gen=gen,
                                                on_progress=_acq_prog)
                    if gen != self._demo_gen:
                        return
                    if not res_a.get("ok"):
                        result["error"] = (
                            "start pose not reached — "
                            + str(res_a.get("error") or "aborted"))
                        with self._lock:
                            self._cal_result = result
                            self._demo_status = result["error"]
                            self._cal_progress = {
                                "msg": self._demo_status}
                        return result

                _set_torque_limit(d.bus, live, torque)
                _enable_torque(d.bus, live)
                n = len(kf_path)

                def guard_msg() -> str:
                    return (f"stopped: {tracker.peak_a:.2f} A peak on "
                            f"joint {tracker.peak_joint} (> "
                            f"{abort_current_a:.1f} A) — stall-fight, "
                            "not grinding on it")

                # Guard semantics (08-10, after a 3.04 A spike aborted
                # a healthy 10x stand at 60%): trip on STALL-FIGHT —
                # a joint over the limit while NOT MOVING, two sweeps
                # in a row — not on an instantaneous reading. A moving
                # joint briefly over 3 A is honest acceleration work.
                # Hard cap 4.0 A trips regardless.
                HARD_CAP_A = 4.0
                stall_prev: set = set()

                def stall_trip() -> bool:
                    nonlocal stall_prev
                    if tracker.peak_a > HARD_CAP_A:
                        return True
                    now = {fb["joint"] for fb in tracker.last_fb
                           if abs(fb["current_a"]) > abort_current_a
                           and abs(fb["speed_deg_s"]) < 8.0}
                    hit = bool(now & stall_prev)
                    stall_prev = now
                    return hit

                def _replant(target_q: list[float]) -> bool:
                    """Re-seat all six feet at target_q, one tripod
                    at a time. Loaded feet cannot slide sideways
                    against ground friction (sim-validated) — lifting
                    is the only way to move them. Lift via hip -6 /
                    knee +6: near tibia-vertical a knee-only lift
                    barely clears the floor (cos ~ 0), the hip raise
                    gives ~9 mm."""
                    for legs in ((0, 2, 4), (1, 3, 5)):
                        if self._demo_abort.is_set():
                            return False
                        q_lift = list(target_q)
                        for lg in legs:
                            q_lift[3 * lg + 1] -= 6.0
                            q_lift[3 * lg + 2] += 6.0
                        _write_pose(d.bus, q_lift, live,
                                    speed=400, acc=50)
                        time.sleep(0.4)
                        _write_pose(d.bus, target_q, live,
                                    speed=300, acc=40)
                        time.sleep(0.45)
                    return True

                # PURSUE — stream the interpolated keyframe path at
                # ~20 Hz for ALL tempos. Per-keyframe glides settle-
                # polled to within 2.5 deg at EVERY waypoint; loaded
                # joints never converge, so each waypoint burned its
                # full timeout (measured: 24 s for the 8.8 s tuck at
                # 1x — operator, 08-10: "ridiculously slow"). The old
                # "careful" per-keyframe path is gone; pursuit has the
                # same current guard and abort checks. Sizing gotchas
                # (operator, 08-10, "starts slow then pops up"): the
                # module MAX_STREAM_SPEED=450 (~40 deg/s) let targets
                # outrun the servos and the final settle covered the
                # backlog in one pop — pass an explicit cap; size each
                # write by the ACTUAL elapsed tick (the current sweep
                # steals 0.1-0.3 s).
                q0 = kf_path[0][0]
                aborted = False
                t_run0 = time.monotonic()
                if down and len(kf_path) >= 2:
                    # Sit starts at the wide (tibia-vertical) stance;
                    # the feet must come back UNDER the body before
                    # the fold, and loaded feet can't slide inward —
                    # re-seat them on the narrow stance by tripods.
                    # If the robot already stands narrow (old-stance
                    # or RL-plant start) skip the wide frame instead
                    # of easing outward pointlessly.
                    with self.drive._lock:
                        w_wide, _ = (self.drive
                                     ._max_delta_vs_present(
                                         kf_path[0][0]))
                        w_narrow, _ = (self.drive
                                       ._max_delta_vs_present(
                                           kf_path[1][0]))
                    if w_narrow <= w_wide:
                        kf_path = kf_path[1:]
                        q0 = kf_path[0][0]
                    else:
                        with self._lock:
                            self._cal_progress = {
                                "msg": f"{mode} {verb}: re-seating "
                                       "feet under body"}
                        if not _replant(kf_path[1][0]):
                            aborted = True
                        else:
                            kf_path = kf_path[1:]
                            q0 = kf_path[0][0]
                with self.drive._lock:
                    worst0, _ = self.drive._max_delta_vs_present(q0)
                if worst0 > 5.0 and not aborted:
                    with self._lock:
                        self._cal_progress = {
                            "msg": f"{mode} {verb}: aligning"}
                    ok = ease_to_pose(
                        d.bus, q0,
                        abort_check=self._demo_abort.is_set,
                        seconds=max(0.6, kf_path[0][1] / speed),
                        label=f"{mode} align",
                        current_tracker=tracker)
                    aborted = not ok
                # Schedule: each segment gets max(authored/tempo,
                # travel-at-90deg/s). PER-SEGMENT, not a uniform
                # rescale: servo_fb traces (08-10, sit x10) showed
                # authored-duration pacing idles the servos through
                # slow phases (13 deg/s measured) then demands
                # 130 deg/s+ through fast ones — loaded servos fell
                # 76 deg behind and the settle glide did the last
                # quarter of the motion in one yank ("pauses a bunch
                # of times"). Travel-based pacing keeps the target
                # moving at a rate the hardware actually tracks.
                RATE_DPS = 90.0
                ts, qs = [0.0], [q0]
                for q_deg, kf_s in kf_path[1:]:
                    d_seg = max(abs(b - a) for a, b in
                                zip(qs[-1], q_deg))
                    ts.append(ts[-1] + max(0.02, kf_s / speed,
                                           d_seg / RATE_DPS))
                    qs.append(q_deg)
                streamer = PoseStreamer()
                # Prime: we are already at q0 (aligned / guarded), so
                # skip the streamer's gentle first-write ease (speed
                # 120 ≈ 10 deg/s — measured as a near-stalled first
                # 0.3 s of every run).
                streamer.last = list(q0)
                tripped = False
                seg, last_sample = 1, -1.0
                t0 = time.monotonic()
                align_s = t0 - t_run0
                t_prev = 0.0
                nticks, write_s, sample_s = 0, 0.0, 0.0
                # Carrot lookahead: command the pose ~2 ticks AHEAD of
                # the schedule. Sizing speeds to finish a step exactly
                # within one tick means any jitter makes the servo
                # arrive EARLY and park until the next write — an
                # ~18 Hz stutter (operator 08-10: "still glitchy").
                # With the target held ahead, the servo never runs out
                # of goal and moves continuously; the settle absorbs
                # the small trailing gap.
                LOOKAHEAD_S = 0.12

                def _q_at(tq: float) -> list[float]:
                    tq = min(tq, ts[-1])
                    s = seg
                    while s < len(qs) and tq > ts[s]:
                        s += 1
                    s = min(s, len(qs) - 1)
                    f = ((tq - ts[s - 1])
                         / max(ts[s] - ts[s - 1], 1e-6))
                    f = min(max(f, 0.0), 1.0)
                    return [a + (b - a) * f for a, b in
                            zip(qs[s - 1], qs[s])]

                # Adaptive tempo: the 90 deg/s base is the LOADED
                # tracking rate; unloaded the STS3215 does 270 deg/s
                # at 12 V (spec: 0.222 s/60deg no-load). Advance the
                # schedule through a rate multiplier steered by the
                # measured tracking error each feedback sweep — air
                # phases speed up toward the no-load ceiling, loaded
                # phases back off instead of piling up backlog.
                rate = 1.0
                t = 0.0
                wall_prev = t0
                while not aborted and not tripped:
                    if self._demo_abort.is_set():
                        aborted = True
                        break
                    wall = time.monotonic()
                    t += (wall - wall_prev) * rate
                    wall_prev = wall
                    while seg < len(qs) and t > ts[seg]:
                        seg += 1
                    if seg >= len(qs):
                        break
                    q = _q_at(t + LOOKAHEAD_S * rate)
                    w0 = time.monotonic()
                    # dt*0.75: cancels _speed_for_delta's 0.9
                    # undershoot AND commands ~1.2x the carrot rate so
                    # accumulated lag drains — at exactly 1.0x a lag
                    # persists forever and pins the adaptive rate.
                    streamer.write(
                        d.bus, q, live,
                        dt=min(max(wall - t0 - t_prev, 0.03), 0.25)
                        * 0.75,
                        deadband=0.3, max_speed=3000, max_acc=200)
                    write_s += time.monotonic() - w0
                    nticks += 1
                    t_prev = wall - t0
                    if wall - t0 - last_sample > 0.25:
                        # feedback sweep costs real bus time —
                        # sample sparsely, mid-motion
                        s0 = time.monotonic()
                        tracker.sample(d.bus, live)
                        sample_s += time.monotonic() - s0
                        _emit_servo_fb(
                            f"{mode} {verb} t={t:.1f}s r={rate:.1f}",
                            tracker, target=q)
                        last_sample = wall - t0
                        # lag vs the SCHEDULE pose (not the carrot,
                        # which is deliberately ahead by rate*look)
                        q_sched = _q_at(t)
                        err = max(
                            (abs(q_sched[fb["joint"]] - fb["deg"])
                             for fb in tracker.last_fb), default=0.0)
                        if err < 16.0:
                            rate = min(rate * 1.35, 2.8)
                        elif err > 28.0:
                            rate = max(rate * 0.6, 0.6)
                        with self._lock:
                            self._cal_progress = {
                                "msg": (f"{mode} {verb}: "
                                        f"{t:.1f}/{ts[-1]:.1f}s "
                                        f"x{rate:.1f} peak "
                                        f"{tracker.peak_a:.2f}A"),
                                    "keyframe": seg, "of": n}
                        tripped = stall_trip()
                    time.sleep(0.05)
                stream_s = time.monotonic() - t0
                settle_s, worst = 0.0, -1.0
                if not aborted and not tripped:
                    # settle: direct command to the final pose, then
                    # up to 3 slow corrective RE-commands. Loaded
                    # joints stop a few degrees short on the first
                    # write (foot friction / stiction), and at high
                    # tempo the feet land with more scatter — the
                    # operator sees a "weird stance". Re-commanding
                    # the same absolute target at low speed nudges
                    # each joint the rest of the way; converge to
                    # 1.2 deg, not 2.5.
                    st0 = time.monotonic()
                    _write_pose(d.bus, qs[-1], live,
                                speed=900, acc=80)
                    deadline = st0 + 1.2
                    while time.monotonic() < deadline:
                        if self._demo_abort.is_set():
                            aborted = True
                            break
                        with self.drive._lock:
                            worst, _ = (self.drive
                                        ._max_delta_vs_present(
                                            qs[-1]))
                        if worst < 2.5:
                            break
                        time.sleep(0.1)
                    with self.drive._lock:
                        worst, _ = (self.drive
                                    ._max_delta_vs_present(qs[-1]))
                    if (verb == "stand-up" and worst > 1.5
                            and not aborted
                            and not self._demo_abort.is_set()):
                        # Stiction re-plant: re-commanding the same
                        # target can't drag a LOADED foot sideways —
                        # the joint just stalls against ground
                        # friction (measured: 3 corrective passes
                        # left 2.4 deg / 2.3 A). Re-seat each foot
                        # friction-free where it belongs.
                        if not _replant(qs[-1]):
                            aborted = True
                        with self.drive._lock:
                            worst, _ = (self.drive
                                        ._max_delta_vs_present(
                                            qs[-1]))
                    if verb == "stand-up" and not aborted:
                        # Hold at FULL torque: the motion ran at
                        # τ700 for the guard, but holding a loaded
                        # stance at 700 lets knees buckle within
                        # seconds (measured 08-11: leg 5 knee +13 deg
                        # ~10 s after a clean 1.9 deg settle). The
                        # legacy glide always held at τ1000.
                        _set_torque_limit(d.bus, live, 1000)
                        _write_pose(d.bus, qs[-1], live,
                                    speed=300, acc=40)
                    tracker.sample(d.bus, live)
                    _emit_servo_fb(f"{mode} {verb} settle",
                                   tracker, target=qs[-1])
                    settle_s = time.monotonic() - st0
                    tripped = tracker.peak_a > HARD_CAP_A
                timing = (f"align {align_s:.2f}s (worst0 "
                          f"{worst0:.1f}deg) + stream "
                          f"{stream_s:.2f}s (sched {ts[-1]:.2f}s, "
                          f"{nticks} ticks, write {write_s:.2f}s, "
                          f"sample {sample_s:.2f}s) + settle "
                          f"{settle_s:.2f}s (end err "
                          f"{worst:.1f}deg) = "
                          f"{time.monotonic() - t_run0:.2f}s")
                print(f"[standup] {mode} {verb} x{speed:g}: "
                      f"{timing}, peak {tracker.peak_a:.2f}A",
                      flush=True)
                result["timing"] = timing
                if tripped:
                    result["error"] = guard_msg()
                elif aborted:
                    result["aborted"] = True
                else:
                    result["ok"] = True
                result["keyframes_done"] = min(seg, n)
                result["peak_a"] = round(tracker.peak_a, 2)
                result["peak_joint"] = tracker.peak_joint
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = result
                    self._demo_status = (
                        f"done · {mode} {verb} peak "
                        f"{tracker.peak_a:.2f} A"
                        if result["ok"] else
                        result.get("error", "aborted"))
                    self._cal_progress = {"msg": self._demo_status}
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._demo_status = f"error: {e}"
                    self._cal_result = {"ok": False, "error": str(e),
                                        "mode": mode}
            finally:
                self._bus_hot = False
                if gen != self._demo_gen:
                    return
                try:
                    _set_torque_limit(d.bus, live, 1000)
                except Exception:
                    pass
                with d._lock:
                    if d.mode == "demo":
                        d.mode = "idle"
                with self._lock:
                    st = self._demo_status
                self._set_activity(
                    "armed" if d.armed else "limp", st or "standup done")
            return result

        if sync_gen is not None:
            # Inline run inside the caller's worker (acquisition
            # chaining). Returns the final result, not a job handle.
            res = _worker()
            if res is None:
                res = {"ok": False, "mode": mode,
                       "error": "preempted by another job"}
            return res

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "mode": mode, "speed": speed,
                "direction": direction, "keyframes": len(frames),
                "calibrate": self.calibrate_state()}

    # ------------------------------------------------------------------
    # Measurement lab (web UI "Measure" tab, 2026-08-10).
    #
    # Operator-run data collection that settles open sim-calibration
    # decisions (rl_docs/HARDWARE.md "Experiment backlog"):
    #   walk  — tape-measured distance vs commanded (slip / contact
    #           pricing; scripted gait, same caps as
    #           rl_move/scripts/tape_measure_walk.py). Also does the
    #           commanded-turn sign check (omega-only runs).
    #   hold  — planted-vs-hover per-servo holding currents (the last
    #           effort-pricing gap in the sim reward).
    #   note  — standalone operator record (e.g. tape distance for an
    #           RL walk episode; auto-attaches the newest rl_walk CSV).
    #
    # Every run samples /api/rl/feedback-shaped telemetry at ~3 Hz into
    # logs/meas_<stamp>_{servo,imu}.csv (the same CSV shapes the
    # calibration tooling reads). Runs that need a physical reading
    # leave a PENDING record; measure_annotate() merges the operator's
    # numbers and appends the finished record to logs/measurements.jsonl.
    # Fetch: scp arduino@hexapod.local:hexapod_sts/linux_control/logs/
    # {measurements.jsonl,meas_*.csv} rl_move/hardware_traces/

    MEAS_MAX_VX_MM = 60.0
    MEAS_MAX_VY_MM = 40.0
    MEAS_MAX_OMEGA = 0.5
    MEAS_MAX_WALK_S = 60.0
    MEAS_MAX_HOLD_S = 120.0
    MEAS_TILT_STOP_DEG = 30.0   # working gait rocks ±10-20°; 30 = wrong
    MEAS_POLL_S = 0.3

    def _meas_dir(self) -> Path:
        from event_log import log_dir
        return log_dir()

    def _meas_file(self) -> Path:
        return self._meas_dir() / "measurements.jsonl"

    def _meas_finalize(self, rec: dict) -> dict:
        rec["saved_unix"] = round(time.time(), 3)
        with self._lock:
            self._meas_pending = None
        with self._meas_file().open("a") as f:
            f.write(json.dumps(rec) + "\n")
        try:
            from event_log import emit
            emit("measurement", rec.get("kind", "?"), src="bench",
                 data={k: rec[k] for k in ("kind", "stamp") if k in rec})
        except Exception:
            pass
        return {"ok": True, "record": rec}

    def measure_list(self, n: int = 20) -> dict:
        """Recent saved measurements + the pending (unannotated) one."""
        recs: list[dict] = []
        try:
            lines = self._meas_file().read_text().strip().splitlines()
            for ln in lines[-int(n):]:
                try:
                    recs.append(json.loads(ln))
                except ValueError:
                    pass
        except OSError:
            pass
        with self._lock:
            pending = dict(self._meas_pending) if getattr(
                self, "_meas_pending", None) else None
        return {"ok": True, "records": list(reversed(recs)),
                "pending": pending,
                "fetch": ("HTTP: GET /api/logs (list) + "
                          "/api/logs/<name> (download); or scp "
                          "arduino@hexapod.local:hexapod_sts/"
                          "linux_control/logs/{measurements.jsonl,"
                          "meas_*.csv} rl_move/hardware_traces/")}

    def _meas_telemetry(self, stamp: str, seconds: float,
                        stop_early=None) -> dict:
        """Sample rl_feedback at ~3 Hz for `seconds` into CSVs.

        Returns aggregates (per-joint mean current, bus totals, tilt
        peaks). `stop_early()` (optional) ends the loop; a tilt beyond
        MEAS_TILT_STOP_DEG sets aggregate["tilt_alert"] and ends it too.
        """
        import csv as _csv

        servo_csv = self._meas_dir() / f"meas_{stamp}_servo.csv"
        imu_csv = self._meas_dir() / f"meas_{stamp}_imu.csv"
        agg: dict = {"servo_csv": servo_csv.name, "imu_csv": imu_csv.name,
                     "samples": 0, "tilt_alert": False,
                     "per_joint_mean_a": None, "bus_a_mean": None,
                     "bus_a_max": None, "max_abs_roll_deg": 0.0,
                     "max_abs_pitch_deg": 0.0}
        sums = [0.0] * N_JOINTS
        counts = [0] * N_JOINTS
        totals: list[float] = []
        hdr = ["t_unix", "live"]
        for j in range(N_JOINTS):
            hdr += [f"q{j}_deg", f"cur{j}_a", f"temp{j}_c"]
        t_end = time.monotonic() + seconds
        with servo_csv.open("w", newline="") as fs, \
                imu_csv.open("w", newline="") as fi:
            ws = _csv.writer(fs)
            ws.writerow(hdr)
            wi = _csv.writer(fi)
            wi.writerow(["t_unix", "roll_deg", "pitch_deg",
                         "gx_dps", "gy_dps", "gz_dps"])
            while time.monotonic() < t_end:
                if self._demo_abort.is_set():
                    break
                if stop_early is not None and stop_early():
                    break
                t0 = time.monotonic()
                fb = self.rl_feedback()
                if fb.get("ok"):
                    t = fb.get("t_unix")
                    joints = fb.get("joints") or []
                    row: list = [t, fb.get("live", 0)]
                    total = 0.0
                    for j in range(N_JOINTS):
                        m = (joints[j]
                             if j < len(joints) and joints[j] else None)
                        if m is None:
                            row += ["", "", ""]
                            continue
                        cur = abs(float(m.get("cur_a", 0.0) or 0.0))
                        sums[j] += cur
                        counts[j] += 1
                        total += cur
                        row += [m.get("deg", ""), m.get("cur_a", ""),
                                m.get("temp_c", "")]
                    ws.writerow(row)
                    fs.flush()
                    totals.append(total)
                    agg["samples"] += 1
                    roll, pitch = fb.get("roll_deg"), fb.get("pitch_deg")
                    if roll is not None and pitch is not None:
                        g = fb.get("gyro_dps") or ["", "", ""]
                        wi.writerow([t, roll, pitch, *g])
                        fi.flush()
                        agg["max_abs_roll_deg"] = max(
                            agg["max_abs_roll_deg"], abs(float(roll)))
                        agg["max_abs_pitch_deg"] = max(
                            agg["max_abs_pitch_deg"], abs(float(pitch)))
                        if (abs(float(roll)) > self.MEAS_TILT_STOP_DEG or
                                abs(float(pitch)) > self.MEAS_TILT_STOP_DEG):
                            agg["tilt_alert"] = True
                            break
                time.sleep(max(0.0, self.MEAS_POLL_S
                               - (time.monotonic() - t0)))
        if totals:
            agg["bus_a_mean"] = round(sum(totals) / len(totals), 3)
            agg["bus_a_max"] = round(max(totals), 3)
        agg["per_joint_mean_a"] = [
            round(sums[j] / counts[j], 3) if counts[j] else None
            for j in range(N_JOINTS)]
        agg["max_abs_roll_deg"] = round(agg["max_abs_roll_deg"], 1)
        agg["max_abs_pitch_deg"] = round(agg["max_abs_pitch_deg"], 1)
        return agg

    def measure_walk(self, *, vx_mm: float = 30.0, vy_mm: float = 0.0,
                     omega: float = 0.0, duration_s: float = 20.0) -> dict:
        """Scripted-gait measured run (tape distance / turn sign).

        Drives the tripod gait (`J vx vy omega`) for duration_s while
        logging telemetry, then stops to a planted stand (torque stays
        on) and leaves a PENDING record — enter the tape reading via
        measure_annotate. Same caps as tape_measure_walk.py. If the
        robot is not already ARMED + STANDING the worker ACQUIRES the
        stand first (08-11 directive: safe zero → plant stand-up);
        acquisition failure stops everything and the walk never runs.
        MOTION: operator must be watching.
        """
        d = self.drive
        if d.dry_run or not d.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            return {"ok": False, "error": "stop the running job first"}
        with self._lock:
            if getattr(self, "_meas_pending", None):
                return {"ok": False,
                        "error": "pending measurement — save or discard "
                                 "it first"}
        vx = max(-self.MEAS_MAX_VX_MM, min(self.MEAS_MAX_VX_MM,
                                           float(vx_mm)))
        vy = max(-self.MEAS_MAX_VY_MM, min(self.MEAS_MAX_VY_MM,
                                           float(vy_mm)))
        om = max(-self.MEAS_MAX_OMEGA, min(self.MEAS_MAX_OMEGA,
                                           float(omega)))
        secs = min(max(float(duration_s), 3.0), self.MEAS_MAX_WALK_S)
        stamp = time.strftime("%Y%m%d_%H%M%S")

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        label = (f"measure walk vx={vx:.0f} vy={vy:.0f} "
                 f"w={om:+.2f} {secs:.0f}s")
        with self._lock:
            self._demo_name = "measure_walk"
            self._demo_status = label
            self._demo_params = {"vx_mm": vx, "vy_mm": vy, "omega": om,
                                 "duration_s": secs}
            self._cal_result = None
            self._cal_progress = {"msg": label}
        self._set_activity("measure", label)

        def _worker():
            import math as _math
            rec: dict = {
                "kind": "turn_sign" if (om and not vx and not vy)
                        else "walk_tape",
                "stamp": stamp,
                "vx_mm_s": vx, "vy_mm_s": vy, "omega_rad_s": om,
                "planned_s": secs,
                "cmd_smoothing_note": ("gait vel low-pass tau=0.15s; "
                                       "commanded path ~|v|*0.15s "
                                       "shorter than |v|*T"),
            }
            try:
                # Acquire ARM + planted stand when missing (08-11):
                # safe zero → validated plant stand-up → stand hold.
                need_stand = True
                try:
                    from feetech_bus import standing_pose_degrees
                    w_st, _ = self._delta_vs_present(
                        standing_pose_degrees())
                    need_stand = w_st is None or w_st > 30.0
                except Exception:
                    need_stand = True
                if need_stand or not d.armed:
                    if need_stand:
                        with self._lock:
                            self._demo_status = (
                                "acquiring stand before walk…")
                        res_a = self._acquire_start("stand", gen=gen)
                        if gen != self._demo_gen:
                            return
                        if not res_a.get("ok"):
                            with self._lock:
                                self._cal_result = {
                                    "ok": False,
                                    "error": (
                                        "start pose not reached — "
                                        + str(res_a.get("error")
                                              or "aborted"))}
                                self._demo_status = (
                                    self._cal_result["error"])
                            return
                    self._enter_stand_hold()
                    time.sleep(0.5)
                    with self._lock:
                        self._demo_status = label
                r = d.handle(f"J {vx:.1f} {vy:.1f} {om:.3f}")
                if r != "J":
                    with self._lock:
                        self._cal_result = {"ok": False,
                                            "error": f"J refused: {r}"}
                        self._demo_status = f"refused: {r}"
                    return
                t0 = time.monotonic()
                agg = self._meas_telemetry(stamp, secs)
                walked = time.monotonic() - t0
                d.handle("J 0 0 0")   # planted stand, torque stays on
                time.sleep(1.0)
                speed = _math.hypot(vx, vy)
                rec.update(
                    walked_s=round(walked, 2),
                    commanded_mm=round(speed * walked, 1),
                    commanded_rot_deg=round(
                        _math.degrees(om * walked), 1),
                    stopped=("tilt_alert" if agg["tilt_alert"] else
                             "abort" if self._demo_abort.is_set()
                             else "duration"),
                    **agg)
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._meas_pending = rec
                    self._cal_result = {"ok": True, "pending": True,
                                        **rec}
                    self._demo_status = (
                        f"walked {walked:.1f}s (commanded "
                        f"~{rec['commanded_mm']:.0f} mm) — read the "
                        "tape and save in the Measure tab")
                    self._cal_progress = {"msg": self._demo_status}
            except Exception as e:
                d.handle("J 0 0 0")
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = {"ok": False, "error": str(e)}
                    self._demo_status = f"error: {e}"
            finally:
                if gen == self._demo_gen:
                    with self._lock:
                        st = self._demo_status
                    self._set_activity(
                        "armed" if d.armed else "limp", st)

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "stamp": stamp, "duration_s": secs}

    def measure_hold(self, *, label: str = "planted",
                     duration_s: float = 30.0) -> dict:
        """Static holding-current measurement (NO commanded motion).

        Torques on and HOLDS the PRESENT pose (never yanks), then logs
        per-servo currents for duration_s and saves the record
        immediately. Run once with the feet planted ("planted") and
        once with the robot propped so the feet hang free ("hover") —
        the delta is the load-dependent current the sim's effort
        pricing is missing.
        """
        d = self.drive
        if d.dry_run or not d.bus:
            return {"ok": False, "error": "no bus"}
        if self._demo_thread and self._demo_thread.is_alive():
            return {"ok": False, "error": "stop the running job first"}
        label = str(label or "planted").strip().lower()
        if label not in ("planted", "hover"):
            return {"ok": False,
                    "error": "label must be 'planted' or 'hover'"}
        secs = min(max(float(duration_s), 5.0), self.MEAS_MAX_HOLD_S)
        stamp = time.strftime("%Y%m%d_%H%M%S")

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        title = f"measure hold ({label}) {secs:.0f}s"
        with self._lock:
            self._demo_name = "measure_hold"
            self._demo_status = title
            self._demo_params = {"label": label, "duration_s": secs}
            self._cal_result = None
            self._cal_progress = {"msg": title}
        self._set_activity("measure", title)

        def _worker():
            rec: dict = {"kind": "hold_current", "label": label,
                         "stamp": stamp, "planned_s": secs}
            try:
                # Hold the PRESENT pose: same never-yank arm sequence
                # the RL runner uses.
                with d._lock:
                    d.mode = "demo"
                    d.gait.stop()
                    if not d.armed:
                        d._torque_all(True)
                        d.armed = True
                    d._hold_here()
                time.sleep(0.5)   # let currents settle after torque-on
                agg = self._meas_telemetry(stamp, secs)
                rec.update(**agg)
                if gen != self._demo_gen:
                    return
                out = self._meas_finalize(rec)
                with self._lock:
                    self._cal_result = out
                    self._demo_status = (
                        f"{label} hold: bus mean "
                        f"{agg['bus_a_mean'] or '?'} A over "
                        f"{agg['samples']} samples — saved")
                    self._cal_progress = {"msg": self._demo_status}
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = {"ok": False, "error": str(e)}
                    self._demo_status = f"error: {e}"
            finally:
                if gen == self._demo_gen:
                    with d._lock:
                        if d.mode == "demo":
                            d.mode = "idle"
                    with self._lock:
                        st = self._demo_status
                    self._set_activity(
                        "armed" if d.armed else "limp", st)

        self._demo_thread = threading.Thread(target=_worker, daemon=True)
        self._demo_thread.start()
        return {"ok": True, "stamp": stamp, "label": label,
                "duration_s": secs}

    def measure_annotate(self, *, fields: dict | None = None) -> dict:
        """Merge operator readings into the pending record and save it.

        Accepted fields (all optional): measured_mm, lateral_drift_mm,
        measured_rot_deg (+ = CW), observed_turn ("cw"/"ccw"/"none"),
        notes. Computes slip ratio when measured_mm arrives.
        """
        with self._lock:
            rec = getattr(self, "_meas_pending", None)
        if not rec:
            return {"ok": False, "error": "no pending measurement"}
        fields = fields or {}
        for k in ("measured_mm", "lateral_drift_mm", "measured_rot_deg"):
            if fields.get(k) not in (None, ""):
                try:
                    rec[k] = float(fields[k])
                except (TypeError, ValueError):
                    return {"ok": False, "error": f"bad number for {k}"}
        if fields.get("observed_turn") in ("cw", "ccw", "none"):
            rec["observed_turn"] = fields["observed_turn"]
        if fields.get("notes"):
            rec["notes"] = str(fields["notes"])[:500]
        if (rec.get("measured_mm") is not None
                and rec.get("commanded_mm", 0) > 1e-6):
            rec["slip_ratio_measured_over_commanded"] = round(
                rec["measured_mm"] / rec["commanded_mm"], 3)
        return self._meas_finalize(rec)

    def measure_discard(self) -> dict:
        """Drop the pending measurement without saving."""
        with self._lock:
            had = getattr(self, "_meas_pending", None) is not None
            self._meas_pending = None
        return {"ok": True, "discarded": had}

    def measure_note(self, *, kind: str = "note",
                     fields: dict | None = None) -> dict:
        """Standalone operator record (no run). kind='rl_walk_tape'
        auto-attaches the newest rl_walk episode CSV so the tape
        reading lines up with its 25 Hz trace."""
        kind = str(kind or "note").strip()[:40]
        rec: dict = {"kind": kind,
                     "stamp": time.strftime("%Y%m%d_%H%M%S")}
        fields = fields or {}
        for k, v in fields.items():
            if k in ("measured_mm", "lateral_drift_mm",
                     "measured_rot_deg", "commanded_mm"):
                try:
                    rec[k] = float(v)
                except (TypeError, ValueError):
                    pass
            elif k == "notes":
                rec["notes"] = str(v)[:500]
        if kind == "rl_walk_tape":
            try:
                latest = max(self._meas_dir().glob("rl_walk_*.csv"),
                             key=lambda p: p.stat().st_mtime)
                rec["rl_episode_csv"] = latest.name
            except (ValueError, OSError):
                rec["rl_episode_csv"] = None
        if (rec.get("measured_mm") is not None
                and rec.get("commanded_mm", 0) > 1e-6):
            rec["slip_ratio_measured_over_commanded"] = round(
                rec["measured_mm"] / rec["commanded_mm"], 3)
        # Standalone records skip the pending slot entirely.
        rec["saved_unix"] = round(time.time(), 3)
        with self._meas_file().open("a") as f:
            f.write(json.dumps(rec) + "\n")
        return {"ok": True, "record": rec}
