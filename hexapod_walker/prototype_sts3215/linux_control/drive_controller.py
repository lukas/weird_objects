"""STS3215 drive loop: web/xbox command lines → TripodGait → FeetechBus.

Runs on the Uno Q Linux side. Prefer the MCU UART bridge (FE-URT on D0/D1);
fall back to a USB URT-2 if present.  Command language mirrors the v1
firmware enough that the web UI can stay familiar:

  ARM / X / SETTLE     arm torque / emergency limp / sit then limp
  P                    stand / park (planted stance)
  C                    centre all joints to 0°
  J vx vy omega [gait] live drive (vx,vy in mm/s; omega rad/s)
  K <lift_mm>          swing foot lift
  GAIT <id> [alpha]    pick the walk gait: 0 = tripod (body-frame drag,
                       legacy), 1 = no-slip world-pinned (noslip_gait);
                       alpha 0..1 = body-motion overlap (0 = step-then-
                       shift, 1 = continuous). Swaps are refused while
                       walking — send J 0 0 0 first; alpha alone
                       retunes the live no-slip gait at the next phase
                       boundary.
  # <j> <deg>          set one joint
  Q <j> <amp>          wiggle one joint
  HOLD                 freeze at present pose
"""
from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
VENDOR = HERE / "vendor"
MOTOR_SETUP = HERE.parent / "motor_setup"
URT2_SETUP = HERE / "urt2_setup"
URT2_SETUP_HOME = Path.home() / "hexapod_sts" / "urt2_setup"

# Prefer vendored SDK (offline Uno Q), then urt2_setup bundle, then motor_setup.
# reversed(): each insert(0) puts the LAST-inserted first, so iterate the
# priority list back-to-front (the old forward loop silently REVERSED the
# priority — motor_setup shadowed urt2_setup, bug found 2026-08-18).
for p in reversed((str(VENDOR), str(URT2_SETUP), str(URT2_SETUP_HOME),
                   str(MOTOR_SETUP), str(HERE))):
    if Path(p).is_dir() and p not in sys.path:
        sys.path.insert(0, p)

from feetech_bus import (  # noqa: E402
    ADDR_TORQUE_ENABLE, BAUD_DEFAULT, N_JOINTS, WALK_ACC,
    WALK_SPEED, deg_to_count, joint_to_servo_id, normalize_acc,
    normalize_speed, standing_pose_degrees,
)
from mcu_feetech_bus import open_feetech_bus  # noqa: E402
from noslip_gait import NoSlipGait  # noqa: E402
from tripod_gait import TripodGait  # noqa: E402

DT = 0.05  # 20 Hz walk loop
SETTLE_SECONDS = 4.0
LIVE_SCAN_PERIOD_S = 2.0
# Refuse absolute centre/stand SyncWrites that yank any live joint farther
# than this from its *present* angle (2026-08-06 cooked-motor incident).
# Sit→default stand is ~80° on knees; operator OK'd 90° (2026-08-07).
# Override with trailing FORCE on C / P only when the operator means it.
MAX_SAFE_DELTA_DEG = 90.0


class DriveController:
    def __init__(self, port: str | None = None, *, baud: int = BAUD_DEFAULT,
                 dry_run: bool = False):
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.bus = None  # FeetechBus | McuFeetechBus
        self.gait = TripodGait()
        self.armed = False
        self.mode = "idle"  # idle | stand | walk | settle
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._vx = self._vy = self._omega = 0.0
        self._gait_id = 0            # 0 = tripod (drag), 1 = no-slip
        self._noslip_alpha = 0.0     # body-motion overlap (no-slip only)
        self._lift_mm: float | None = None   # last K value, re-applied on swap
        self._last_pose = standing_pose_degrees()
        self.status = "init"
        self._live_ids_cache: set[int] = set()
        self._live_ids_t = 0.0
        # Set by web_drive after construction (optional bench JSON API).
        self.bench = None

    def start(self) -> None:
        if not self.dry_run:
            self.bus, port = open_feetech_bus(self.port, baud=self.baud)
            self.port = port
            # Boot limp — nothing moves until ARM.
            self._torque_all(False)
            self.status = f"bus:{port} disarmed"
        else:
            self.status = "dry-run (no bus)"
            print("[drive] dry-run — no bus opened")
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            if self.bus and self.armed:
                self._torque_all(False)
        except Exception:
            pass
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass

    # -- bus helpers ---------------------------------------------------------
    def _live_ids(self, *, force: bool = False) -> set[int]:
        if not self.bus:
            return set()
        now = time.monotonic()
        if (not force and self._live_ids_cache
                and now - self._live_ids_t < LIVE_SCAN_PERIOD_S):
            return self._live_ids_cache
        try:
            self._live_ids_cache = {sid for sid in self.bus.scan(range(2, 20))}
            self._live_ids_t = now
        except Exception:
            pass
        return self._live_ids_cache

    def _torque_all(self, on: bool) -> None:
        if not self.bus:
            return
        if hasattr(self.bus, "enable_all_torque"):
            try:
                self.bus.enable_all_torque(on)
                return
            except Exception:
                pass
        for sid in sorted(self._live_ids(force=True) or
                          {joint_to_servo_id(j) for j in range(N_JOINTS)}):
            try:
                self.bus.pkt.write1ByteTxRx(
                    sid, ADDR_TORQUE_ENABLE, 1 if on else 0)
            except Exception:
                pass

    def _read_present_pose(self) -> list[float | None]:
        if not self.bus:
            return [None] * N_JOINTS
        out: list[float | None] = []
        for j in range(N_JOINTS):
            try:
                out.append(self.bus.read_position_deg(j))
            except Exception:
                out.append(None)
        return out

    def _max_delta_vs_present(self, goal: list[float]
                              ) -> tuple[float, int | None]:
        """Largest |goal − present| on live joints that have a reading."""
        present = self._read_present_pose()
        live = self._live_ids()
        worst = 0.0
        worst_j: int | None = None
        for j, g in enumerate(goal):
            sid = joint_to_servo_id(j)
            if live and sid not in live:
                continue
            p = present[j] if j < len(present) else None
            if p is None:
                continue
            d = abs(float(g) - float(p))
            if d > worst:
                worst = d
                worst_j = j
        return worst, worst_j

    def _refuse_large_delta(self, goal: list[float], *,
                            force: bool, label: str) -> str | None:
        if force:
            return None
        worst, j = self._max_delta_vs_present(goal)
        if worst <= MAX_SAFE_DELTA_DEG:
            return None
        self.status = (
            f"refused {label}: max Δq={worst:.0f}° on j{j} "
            f"(>{MAX_SAFE_DELTA_DEG:.0f}°). set-zero-here or FORCE")
        return (
            f"refused {label}: would move j{j} by {worst:.1f}° "
            f"(max {MAX_SAFE_DELTA_DEG:.0f}° without FORCE). "
            f"If encoders disagree with the pose, POST /api/set_zero first."
        )

    def _write_pose(self, degrees: list[float], *,
                    speed: int = WALK_SPEED, acc: int = WALK_ACC) -> None:
        self._last_pose = list(degrees)
        if not self.bus or not self.armed:
            return
        speed = normalize_speed(speed)
        acc = normalize_acc(acc)
        live = self._live_ids()
        for joint, deg in enumerate(degrees):
            sid = joint_to_servo_id(joint)
            if live and sid not in live:
                continue
            count = deg_to_count(joint, deg, self.bus.trims[joint])
            self.bus.pkt.SyncWritePosEx(sid, count, speed, acc)
        self.bus.pkt.groupSyncWrite.txPacket()
        self.bus.pkt.groupSyncWrite.clearParam()

    def _hold_here(self) -> None:
        if not self.bus or not self.armed:
            return
        pose = []
        for j in range(N_JOINTS):
            d = self.bus.read_position_deg(j)
            pose.append(0.0 if d is None else d)
        self._write_pose(pose, speed=250, acc=30)

    # -- gait selection --------------------------------------------------------
    def _gait_desc(self) -> str:
        if self._gait_id == 1:
            return f"noslip alpha={self._noslip_alpha:.2f}"
        return "tripod (drag)"

    def _set_gait(self, gait_id: int, alpha: float | None = None) -> str:
        """Swap / retune the walk gait (call with the lock held).

        Swaps only happen while NOT walking: a fresh gait re-pins its
        feet at neutral, which would yank mid-stride legs. Alpha alone
        retunes a live no-slip gait safely (phase-boundary semantics).
        """
        gait_id = 1 if int(gait_id) == 1 else 0
        if alpha is not None:
            self._noslip_alpha = max(0.0, min(1.0, float(alpha)))
            if gait_id == 1 and self._gait_id == 1:
                self.gait.set_alpha(self._noslip_alpha)
        if gait_id == self._gait_id:
            return f"gait {self._gait_desc()}"
        moving = abs(self._vx) + abs(self._vy) + abs(self._omega) > 1e-4
        if self.mode == "walk" or moving:
            self.status = "gait swap refused while walking (J 0 0 0 first)"
            return "refused gait swap while walking - send J 0 0 0 first"
        self.gait = (NoSlipGait(alpha=self._noslip_alpha) if gait_id == 1
                     else TripodGait())
        self.gait.sync_plant_stance()
        if self._lift_mm is not None:
            self.gait.set_lift_mm(self._lift_mm)
        self.gait.reset_phase(t=time.monotonic())
        self._gait_id = gait_id
        self.status = f"gait -> {self._gait_desc()}"
        return f"gait {self._gait_desc()}"

    # -- command API ---------------------------------------------------------
    def handle(self, line: str) -> str:
        line = (line or "").strip()
        if not line:
            return "empty"
        with self._lock:
            return self._handle_locked(line)

    def _handle_locked(self, line: str) -> str:
        parts = line.split()
        cmd = parts[0].upper()

        if cmd in ("ARM",):
            self._torque_all(True)
            self.armed = True
            self.mode = "idle"
            self.status = "armed"
            return "armed"

        if cmd in ("X", "DISARM", "RELAX"):
            self.mode = "idle"
            self.gait.stop()
            self._vx = self._vy = self._omega = 0.0
            self._torque_all(False)
            self.armed = False
            self.status = "disarmed (limp)"
            return "limp"

        if cmd in ("SETTLE",):
            if not self.armed:
                self._torque_all(True)
                self.armed = True
            self.mode = "settle"
            self.gait.stop()
            self._vx = self._vy = self._omega = 0.0
            self.status = "settling"
            return "settle"

        if cmd == "P":
            if not self.armed:
                return "need ARM"
            force = any(p.upper() == "FORCE" for p in parts[1:])
            self.gait.stop()
            self.gait.sync_plant_stance()
            self.gait.reset_phase(t=time.monotonic())
            self._vx = self._vy = self._omega = 0.0
            stand = standing_pose_degrees()
            refused = self._refuse_large_delta(stand, force=force, label="stand")
            if refused:
                self.mode = "idle"
                return refused
            self.mode = "stand"
            self._last_pose = list(stand)
            self._write_pose(stand, speed=400, acc=20)
            self.status = "standing"
            return "stand"

        if cmd == "C":
            if not self.armed:
                return "need ARM"
            force = any(p.upper() == "FORCE" for p in parts[1:])
            goal = [0.0] * N_JOINTS
            refused = self._refuse_large_delta(goal, force=force, label="centre")
            if refused:
                self.mode = "idle"
                return refused
            self.mode = "idle"
            self.gait.stop()
            self._write_pose(goal, speed=300, acc=15)
            self.status = "centred"
            return "centre"

        if cmd == "HOLD":
            if self.armed:
                self.mode = "idle"
                self.gait.stop()
                self._hold_here()
            return "hold"

        if cmd == "J" and len(parts) >= 4:
            if not self.armed:
                return "need ARM"
            try:
                vx_mm = float(parts[1])
                vy_mm = float(parts[2])
                omega = float(parts[3])
            except ValueError:
                return "bad J"
            gid = None
            if len(parts) >= 5:
                try:
                    gid = int(parts[4])
                except ValueError:
                    gid = None
            # UI uses mm/s; gait wants m/s. Cap gently for first teleop.
            self._vx = max(-0.20, min(0.20, vx_mm / 1000.0))
            self._vy = max(-0.15, min(0.15, vy_mm / 1000.0))
            self._omega = max(-0.9, min(0.9, omega))
            moving = abs(self._vx) + abs(self._vy) + abs(self._omega) > 1e-4
            was_walking = self.mode == "walk"
            self.mode = "walk" if moving else "stand"
            if gid is not None and gid != self._gait_id:
                # Picker swap carried on the J stream: lands on the first
                # stopped packet (refused while moving — see _set_gait).
                self._set_gait(gid)
            if not (was_walking and moving):
                # Pick up the latest learned plant when a walk engages /
                # while standing — but never mid-walk: NoSlipGait's sync
                # re-pins the world anchors, which would snap planted
                # feet back to neutral under load.
                self.gait.sync_plant_stance()
                if moving and isinstance(self.gait, NoSlipGait):
                    # Fresh cycle on engage: re-pin feet under the robot
                    # NOW and restart the startup-softened phase machine.
                    self.gait.reset_phase()
            self.gait.set_velocity(vx=self._vx, vy=self._vy, omega=self._omega)
            self.status = (f"walk[{self._gait_desc()}] vx={self._vx:.3f} "
                           f"vy={self._vy:.3f} w={self._omega:.2f}")
            return "J"

        if cmd == "K" and len(parts) >= 2:
            try:
                lift_mm = float(parts[1])
            except ValueError:
                return "bad K"
            self._lift_mm = lift_mm       # survives gait swaps
            self.gait.set_lift_mm(lift_mm)
            return "K"

        if cmd == "GAIT":
            if len(parts) < 2:
                return f"gait {self._gait_desc()}"
            try:
                gid = int(parts[1])
            except ValueError:
                return "bad GAIT"
            alpha = None
            if len(parts) >= 3:
                try:
                    alpha = float(parts[2])
                except ValueError:
                    return "bad GAIT"
            return self._set_gait(gid, alpha)

        if cmd == "#" and len(parts) >= 3:
            if not self.armed:
                return "need ARM"
            try:
                j = int(parts[1])
                deg = float(parts[2])
            except ValueError:
                return "bad #"
            if not (0 <= j < N_JOINTS):
                return "bad joint"
            force = any(p.upper() == "FORCE" for p in parts[3:])
            # Always seed from *present* encoders — never yank other joints
            # toward a stale stand/zero _last_pose (2026-08-06 incident).
            pose: list[float] = []
            for i in range(N_JOINTS):
                d = self.bus.read_position_deg(i) if self.bus else None
                if d is None:
                    d = self._last_pose[i] if i < len(self._last_pose) else 0.0
                pose.append(float(d))
            pose[j] = deg
            refused = self._refuse_large_delta(pose, force=force, label=f"#{j}")
            if refused:
                return refused
            self.mode = "idle"
            self._write_pose(pose, speed=400, acc=25)
            return f"#{j}"

        if cmd == "Q" and len(parts) >= 3:
            try:
                j = int(parts[1])
                amp = abs(float(parts[2]))
            except ValueError:
                return "bad Q"
            if not (0 <= j < N_JOINTS):
                return "bad joint"
            # Stop stand/walk re-hold so it can't overwrite the wiggle.
            if not self.armed:
                self._torque_all(True)
                self.armed = True
            self.mode = "idle"
            self.gait.stop()
            self._vx = self._vy = self._omega = 0.0
            threading.Thread(
                target=self._wiggle, args=(j, amp), daemon=True).start()
            self.status = f"wiggle j{j} ±{amp:.0f}°"
            return f"Q{j}"

        # Ignore v1-only config / dances politely.
        if cmd in ("E", "Z", "$", "U", "CROUCH", "SIT") or cmd.startswith("M"):
            return "ignored"
        if cmd in ("V", "B", "O", "T"):
            return "ignored"

        return f"unknown:{cmd}"

    def _wiggle(self, joint: int, amp: float) -> None:
        """Nudge one joint ±amp around its *present* angle, then return."""
        with self._lock:
            if not self.bus:
                return
            if not self.armed:
                self._torque_all(True)
                self.armed = True
            self.mode = "idle"
            # Prefer live encoder reading — _last_pose is often stale (e.g.
            # after set-zero-here or hand-posing while limp).
            base = self.bus.read_position_deg(joint)
            if base is None:
                base = self._last_pose[joint]
            pose = list(self._last_pose)
            # Seed other joints from present too when we can, so a sync-write
            # doesn't yank the rest of the body toward an old stand pose.
            for j in range(N_JOINTS):
                d = self.bus.read_position_deg(j)
                if d is not None:
                    pose[j] = d
            self._last_pose = list(pose)

        for sign in (+1, -1, 0):
            with self._lock:
                if not self.armed or not self.bus:
                    return
                self.mode = "idle"
                pose = list(self._last_pose)
                pose[joint] = base + sign * amp
                self._write_pose(pose, speed=500, acc=30)
            time.sleep(0.45)

    # -- background loop -----------------------------------------------------
    def _loop(self) -> None:
        t0 = time.monotonic()
        settle_t0 = None
        while not self._stop.is_set():
            tick = time.monotonic()
            with self._lock:
                mode = self.mode
                armed = self.armed
                if mode == "settle":
                    if settle_t0 is None:
                        settle_t0 = tick
                    u = min(1.0, (tick - settle_t0) / SETTLE_SECONDS)
                    s = u * u * (3 - 2 * u)
                    start = self._last_pose
                    # Ease toward stand plant then limp.
                    goal = list(standing_pose_degrees())
                    pose = [a + (b - a) * s for a, b in zip(start, goal)]
                    if armed:
                        self._write_pose(pose, speed=200, acc=12)
                    if u >= 1.0:
                        self._torque_all(False)
                        self.armed = False
                        self.mode = "idle"
                        self.status = "settled + limp"
                        settle_t0 = None
                else:
                    settle_t0 = None
                    if mode == "demo":
                        pass  # bench / inplace_demos owns the bus
                    elif armed and mode == "walk":
                        pose = self.gait.desired_deg(tick - t0)
                        self._write_pose(pose, speed=WALK_SPEED, acc=WALK_ACC)
                    elif armed and mode == "stand":
                        # Occasional re-hold so stance doesn't droop.
                        # Must match stand zero / learned plant — NOT the
                        # old gait neutral (−25/+60), which yanked hips up
                        # after feet had just planted at +20/+80.
                        if int(tick * 2) % 5 == 0:
                            hold = (list(self._last_pose)
                                    if self._last_pose
                                    else standing_pose_degrees())
                            self._write_pose(hold, speed=300, acc=20)
            time.sleep(DT)
