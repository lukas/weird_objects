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
})
ZERO_TOL_DEG = 6.0

# Sit-from-stand exemption to the MAX_SAFE_DELTA_DEG guard: the present
# pose counts as "at stand" when every live joint is within this many
# degrees of the captured stand pose. Must tolerate a gait stopped
# mid-stride: the loaded tripod freezes in its push posture, measured
# up to 17.4 deg from the captured plant (tape session 08-10, leg2b —
# 15 deg refused the Sit button after EVERY walk). A wrong-zero pose
# reads knees ~160 deg off the plant, so 30 deg is still unambiguous.

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
        # Measure tab: finished run awaiting the operator's tape reading.
        self._meas_pending: dict | None = None
        self._status_display = None
        self._servo_watch = None

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
            lambda j: joint_label(j, self.names))
        self._servo_watch.start()

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
                "params": dict(self._demo_params),
                # Live worker progress (msg + optional joint/index/total) —
                # the TFT job panel renders counts/percent from this.
                "progress": dict(self._cal_progress)
                if self._cal_progress else None,
                "telemetry": dict(self._demo_telemetry)
                if self._demo_telemetry else None,
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
        out = []
        for n, (t, _) in DEMOS.items():
            # breathe+ kept as alias; UI uses size slider on breathe.
            if n == "breathe+":
                continue
            out.append({
                "name": n,
                "title": t,
                "air": n in AIR_DEMO_NAMES,
                "has_size": n in ("breathe", "breathe_v"),
            })
        return out

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

    def run_demo(self, name: str, *, speed: float = 1.0,
                 size: float = 1.0, rate: float | None = None,
                 torque: int | None = None, softness: float = 1.0) -> dict:
        try:
            from inplace_demos import (DEMOS, go_to_stand_pose, go_to_zero_pose,
                                       run_demo)
        except ImportError as e:
            return {"ok": False, "error": f"inplace_demos missing: {e}"}
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
        if name in ("breathe", "breathe_v"):
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
        bits = [f"{name} @ {speed:.2f}×"]
        if switched_from:
            bits.insert(0, f"switch←{switched_from}")
        if name in ("breathe", "breathe_v"):
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
                # mid-demo switch — or starting from the wrong pose — just works.
                with self._lock:
                    self._demo_status = f"homing {home}"
                self._set_activity("zeroing", f"{home} zero → {name}")
                if home == "sit":
                    ok_home = go_to_zero_pose(
                        d.bus, abort_check=self._demo_abort.is_set,
                        seconds=3.5)
                else:
                    ok_home = go_to_stand_pose(
                        d.bus, abort_check=self._demo_abort.is_set,
                        seconds=3.5)
                if gen != self._demo_gen:
                    return
                if self._demo_abort.is_set() or not ok_home:
                    with self._lock:
                        self._demo_status = "aborted"
                    return

                with self._lock:
                    self._demo_status = f"running @ {speed:.2f}×"
                self._set_activity("demo", detail)
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
                status = run_demo(
                    d.bus, name,
                    speed=speed,
                    size=size,
                    rate=rate,
                    torque=torque,
                    softness=softness,
                    abort_check=self._demo_abort.is_set,
                    log_path=log_path,
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

    def go_zero(self, pose: str = "sit", *, force: bool = False) -> dict:
        """Ease to sit zero (legs out) or stand zero (walk plant).

        ``pose``: ``sit`` | ``stand``.  Stand keeps torque on (no limp).

        STAND refuses if any live joint would move more than
        MAX_SAFE_DELTA_DEG unless ``force=True`` — gliding to the stand
        pose with wrong logical zeros is exactly the stilts incident.

        SIT never refuses on delta (operator ruling 2026-08-10, after
        repeated complaints: the Sit button kept refusing from RL
        crouches / stand-up-lab stances that don't match the captured
        stand pose, leaving the robot stuck standing). A big delta just
        makes the eased descent SLOWER (up to 10 s, abortable) — a slow
        supervised glide down is not the incident class; violent
        one-shot writes and unsupervised blends are. Do NOT re-add a
        sit refusal.
        """
        try:
            from inplace_demos import go_to_stand_pose, go_to_zero_pose
            from feetech_bus import N_JOINTS, standing_pose_degrees
            from drive_controller import MAX_SAFE_DELTA_DEG
        except ImportError as e:
            return {"ok": False, "error": str(e)}
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

        goal = (standing_pose_degrees() if pose == "stand"
                else [0.0] * N_JOINTS)
        sit_seconds = 4.0
        if not force:
            with self.drive._lock:
                worst, j = self.drive._max_delta_vs_present(goal)
            if worst > MAX_SAFE_DELTA_DEG:
                if pose == "stand":
                    return {
                        "ok": False,
                        "error": (
                            f"refused {pose} zero: j{j} would move "
                            f"{worst:.1f}° (>{MAX_SAFE_DELTA_DEG:.0f}°). "
                            "POST /api/set_zero at the real pose, or "
                            "pass force=true"
                        ),
                        "max_delta_deg": round(worst, 2),
                        "joint": j,
                        "hint": "set-zero-here — do not FORCE unless watching",
                    }
                # SIT with a big delta: never refuse — descend slower
                # instead (see docstring; operator ruling 08-10).
                sit_seconds = min(10.0, max(6.0, worst / 15.0))
                try:
                    from event_log import emit
                    emit("log",
                         f"sit with big delta (j{j} {worst:.0f}°): "
                         f"slow {sit_seconds:.0f}s eased descent",
                         src="bench")
                except Exception:
                    pass

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        with self._lock:
            self._demo_name = f"zero_{pose}"
            self._demo_status = "zeroing"
            self._demo_params = {"pose": pose, "force": bool(force),
                                 "sit_seconds": sit_seconds}
        self._set_activity("zeroing", f"go to {pose} zero")

        def _worker():
            d = self.drive
            with d._lock:
                d.mode = "demo"
                if not d.armed:
                    d._torque_all(True)
                    d.armed = True
            stand_info: dict = {}
            try:
                if pose == "stand":
                    ok = go_to_stand_pose(
                        d.bus, abort_check=self._demo_abort.is_set,
                        seconds=4.5, result=stand_info)
                else:
                    ok = go_to_zero_pose(
                        d.bus, abort_check=self._demo_abort.is_set,
                        seconds=sit_seconds)
                if gen != self._demo_gen:
                    return
                with self._lock:
                    if ok:
                        self._demo_status = "done"
                    elif stand_info.get("error"):
                        self._demo_status = f"error: {stand_info['error']}"
                    else:
                        self._demo_status = "aborted"
                    if stand_info:
                        self._demo_params = {
                            **dict(self._demo_params or {}),
                            "stand_check": stand_info,
                        }
            except Exception as e:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._demo_status = f"error: {e}"
            finally:
                if gen != self._demo_gen:
                    return
                with self._lock:
                    st = self._demo_status
                # Stand home must keep mode=stand so the drive loop re-holds
                # plant (otherwise stance droops after the one-shot glide).
                if st == "done" and pose == "stand":
                    self._enter_stand_hold()
                    detail = "at stand zero"
                    if stand_info.get("max_err_deg") is not None:
                        detail += f" (err {stand_info['max_err_deg']:.1f}°)"
                    self._set_activity("armed", detail)
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
    _SLOT_OBS = {68: "stance", 72: "walk"}

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
                              f"(68 = stance, 72 = walk)")}
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

    def rl_policy_move(self, *, mode: str = "stand", vx: float = 0.03,
                       vy: float = 0.0, duration_s: float = 6.0) -> dict:
        """Run a trained RL policy: stand up / lower / walk.

        Async (demo-thread slot, poll ``rl_state``, abort via ``rl_stop``).
        Read-only preflight refuses to move unless all 18 servos answer,
        the IMU is alive, and the present pose matches the expected start
        (belly/zero for stand, captured plant for lower AND walk).
        Safety layer trips (tilt / sustained over-current / temp) limp
        immediately. Walk extras: body-frame vx/vy (m/s, clamped to the
        trained 0.06 band) and duration_s (clamped 3..20 s).
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
            return {"ok": False, "error": "stop the running job first"}

        # Preflight before claiming the worker slot so refusals are
        # instant and motion-free.
        ok, reason, details = preflight(self.drive.bus, mode)
        if not ok:
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
                    duration_s=float(duration_s))
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
                result = run_policy_move(
                    d, mode, on_progress=_on_progress,
                    abort_check=self._demo_abort.is_set,
                    vx=float(vx), vy=float(vy),
                    duration_s=float(duration_s))
                if gen != self._demo_gen:
                    return
                with self._lock:
                    self._cal_result = result
                    if result.get("ok"):
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

        Refuses large Δq from present unless ``force``. Hip≈0°+knee≈80° is
        stilts — not a low plant.
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
        if not force:
            with self.drive._lock:
                worst, j = self.drive._max_delta_vs_present(goal)
            if worst > MAX_SAFE_DELTA_DEG:
                return {
                    "ok": False,
                    "error": (
                        f"refused set_stance: j{j} Δq={worst:.1f}° "
                        f"(>{MAX_SAFE_DELTA_DEG:.0f}°). Smaller step or force"
                    ),
                    "max_delta_deg": round(worst, 2),
                    "joint": j,
                }

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
                torque: int = 700, abort_current_a: float = 3.0) -> dict:
        """Play one baked stand-up strategy (async).

        ``direction="up"`` starts from the ZERO pose (belly down, legs
        straight out); ``"down"`` plays the same keyframes in reverse —
        from the mode's standing stance back to the belly. Refuses if
        the present pose is far from the first frame unless ``force``.
        Aborts between keyframes if any servo peaked above
        ``abort_current_a`` (stall-fight = the pinned-feet failure this
        lab exists to fix; do not grind on it).

        Hardware truth 08-10: tuck stood at 2.48 A peak, step at
        2.97 A; blend stalled short at only 0.57 A (the servos give up
        quietly under the torque limit — matches the sim's low-torque
        rows). Faster tempos raise push currents toward the guard.
        """
        try:
            from inplace_demos import (
                CurrentPeakTracker, PoseStreamer, _enable_torque,
                _live_robot_ids, _set_torque_limit, ease_to_pose,
            )
            from drive_controller import MAX_SAFE_DELTA_DEG
        except ImportError as e:
            return {"ok": False, "error": str(e)}
        if self.drive.dry_run or not self.drive.bus:
            return {"ok": False, "error": "no bus"}
        try:
            data = self._load_standup()
            m = data["modes"][str(mode)]
            keyframes = m["keyframes"]
        except (OSError, ValueError, KeyError) as e:
            return {"ok": False, "error": f"unknown stand-up mode: {e}"}
        if self._demo_thread and self._demo_thread.is_alive():
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
        if not force:
            with self.drive._lock:
                worst, j = self.drive._max_delta_vs_present(first)
            if worst > MAX_SAFE_DELTA_DEG:
                where = ("this mode's standing stance (stand up with the "
                         "same mode first)" if down else
                         "the zero start pose. Lay the robot belly-down "
                         "with legs straight out (or set_zero there)")
                return {
                    "ok": False,
                    "error": (
                        f"refused standup: j{j} Δq={worst:.1f}° "
                        f"(>{MAX_SAFE_DELTA_DEG:.0f}°) from {where}, "
                        "or force."),
                    "max_delta_deg": round(worst, 2),
                    "joint": j,
                }

        self._demo_gen += 1
        gen = self._demo_gen
        self._demo_abort.clear()
        verb = "sit-down" if down else "stand-up"
        with self._lock:
            self._demo_name = f"standup_{mode}" + ("_down" if down else "")
            self._demo_status = f"{verb} · {mode} (x{speed:.2f})"
            self._demo_params = {"mode": mode, "speed": speed,
                                 "direction": direction, "torque": torque}
            self._cal_result = None
            self._cal_progress = {"msg": self._demo_status}
        self._set_activity("demo", self._demo_status)

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
            try:
                _set_torque_limit(d.bus, live, torque)
                _enable_torque(d.bus, live)
                n = len(frames)

                def guard_msg() -> str:
                    return (f"stopped: {tracker.peak_a:.2f} A peak on "
                            f"joint {tracker.peak_joint} (> "
                            f"{abort_current_a:.1f} A) — stall-fight, "
                            "not grinding on it")

                if speed > 1.25:
                    # PURSUE — stream the interpolated keyframe path
                    # at ~20 Hz. Per-keyframe glides decelerate to
                    # ZERO at every waypoint (the servo trapezoid), so
                    # even schedule-paced playback stop-started
                    # (operator, 08-10: "still stops at the key
                    # frames"). PoseStreamer keeps the target moving
                    # ahead of the servos. Sizing gotchas (operator,
                    # 08-10 round 2, "starts slow then pops up"):
                    # the module MAX_STREAM_SPEED=450 (~40 deg/s) let
                    # targets outrun the servos, so the final settle
                    # covered the backlog in one pop — pass an
                    # explicit cap near the walk speed (1500). And
                    # size each write by the ACTUAL elapsed tick: the
                    # current sweep steals 0.1-0.3 s, which otherwise
                    # under-sizes the next write.
                    q0 = frames[0][0]
                    aborted = False
                    with self.drive._lock:
                        worst0, _ = self.drive._max_delta_vs_present(q0)
                    if worst0 > 5.0:
                        with self._lock:
                            self._cal_progress = {
                                "msg": f"{mode} {verb}: aligning"}
                        ok = ease_to_pose(
                            d.bus, q0,
                            abort_check=self._demo_abort.is_set,
                            seconds=max(0.6, frames[0][1] / speed),
                            label=f"{mode} align",
                            current_tracker=tracker)
                        aborted = not ok
                    ts, qs = [0.0], [q0]
                    for q_deg, kf_s in frames[1:]:
                        ts.append(ts[-1] + max(0.06, kf_s / speed))
                        qs.append(q_deg)
                    streamer = PoseStreamer()
                    tripped = False
                    seg, last_sample = 1, -1.0
                    t0 = time.monotonic()
                    t_prev = 0.0
                    while not aborted and not tripped:
                        if self._demo_abort.is_set():
                            aborted = True
                            break
                        t = time.monotonic() - t0
                        while seg < len(qs) and t > ts[seg]:
                            seg += 1
                        if seg >= len(qs):
                            break
                        f = ((t - ts[seg - 1])
                             / max(ts[seg] - ts[seg - 1], 1e-6))
                        q = [a + (b - a) * f for a, b in
                             zip(qs[seg - 1], qs[seg])]
                        streamer.write(
                            d.bus, q, live,
                            dt=min(max(t - t_prev, 0.03), 0.25),
                            deadband=0.3, max_speed=1500, max_acc=200)
                        t_prev = t
                        if t - last_sample > 0.3:
                            # feedback sweep costs real bus time —
                            # sample sparsely, mid-motion
                            tracker.sample(d.bus, live)
                            last_sample = t
                            with self._lock:
                                self._cal_progress = {
                                    "msg": (f"{mode} {verb}: "
                                            f"{t:.1f}/{ts[-1]:.1f}s "
                                            f"peak "
                                            f"{tracker.peak_a:.2f}A"),
                                    "keyframe": seg, "of": n}
                            tripped = tracker.peak_a > abort_current_a
                        time.sleep(0.05)
                    if not aborted and not tripped:
                        # settle + hold cleanly on the final pose
                        # (with correct pursuit speeds this is a
                        # residual nudge, not a pop)
                        ok = ease_to_pose(
                            d.bus, qs[-1],
                            abort_check=self._demo_abort.is_set,
                            seconds=0.5, label=f"{mode} settle",
                            current_tracker=tracker)
                        aborted = not ok
                        tripped = tracker.peak_a > abort_current_a
                    if tripped:
                        result["error"] = guard_msg()
                    elif aborted:
                        result["aborted"] = True
                    else:
                        result["ok"] = True
                    result["keyframes_done"] = min(seg, n)
                else:
                    # careful path: one glide + settle per keyframe
                    for i, (q_deg, kf_s) in enumerate(frames):
                        if self._demo_abort.is_set():
                            result["aborted"] = True
                            break
                        secs = max(0.35, kf_s / speed)
                        with self._lock:
                            self._cal_progress = {
                                "msg": (f"{mode} {verb}: keyframe "
                                        f"{i + 1}/{n} ({secs:.1f}s)"),
                                "keyframe": i + 1, "of": n}
                        ok = ease_to_pose(
                            d.bus, q_deg,
                            abort_check=self._demo_abort.is_set,
                            seconds=secs, label=f"{mode} {i + 1}/{n}",
                            current_tracker=tracker)
                        if not ok:
                            result["aborted"] = True
                            break
                        if tracker.peak_a > abort_current_a:
                            result["error"] = guard_msg()
                            break
                    else:
                        result["ok"] = True
                    result["keyframes_done"] = min(i + 1, n)
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
                "fetch": ("scp arduino@hexapod.local:hexapod_sts/"
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
        measure_annotate. Same caps as tape_measure_walk.py. Requires
        the robot ARMED and STANDING (P) — refused otherwise.
        MOTION: operator must be watching.
        """
        d = self.drive
        if d.dry_run or not d.bus:
            return {"ok": False, "error": "no bus"}
        if not d.armed:
            return {"ok": False,
                    "error": "need ARM + standing (P) before a walk run"}
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
