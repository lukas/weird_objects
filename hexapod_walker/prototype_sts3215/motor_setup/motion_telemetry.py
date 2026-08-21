"""Live STS3215 motion telemetry for shake / hunt diagnosis.

Writes a CSV while demos (or a hold test) run, then prints a short
ranking of which motors look like they're hunting.

Columns per sample
------------------
t_s, joint, id, name, cmd_deg, present_deg, err_deg, speed_deg_s,
load_pct, current_a, volt, wrote, cmd_speed, cmd_acc, moving,
ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c,
roll_deg, pitch_deg,
balance_axis, balance_axis_roll, balance_axis_pitch,
balance_target_pitch_deg, balance_pitch_deg, balance_err_deg,
balance_rate_deg_s, balance_pitch_trim_deg, balance_dx_trim_mm,
balance_speed_scale

IMU columns are chassis MPU-6050 (MCU Wire), repeated on every joint
row for that tick so motor amps and body accel/gyro stay aligned.
Blank when the bus has no IMU (USB URT) or the sensor misses.

``wrote=1`` means this tick issued a SyncWritePosEx for that joint.
High ``|err|`` + high ``|speed|`` while ``wrote=0`` and cmd is steady
usually means internal PID hunting (backlash / binding), not host spam.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path

from feetech_bus import (
    ADDR_TORQUE_ENABLE, FeetechBus, N_JOINTS, joint_to_servo_id,
)

# STS moving flag (1 = still executing a goal move).
ADDR_MOVING = 66

LOG_DIR = Path(__file__).resolve().parent / "logs"
AXIS = ("yaw", "hip", "knee")

REGISTRY_CANDIDATES = (
    Path(__file__).resolve().parent / "motor_setup_registry.json",
)


def joint_name(joint: int, registry: dict[int, str] | None = None) -> str:
    if registry and joint_to_servo_id(joint) in registry:
        return registry[joint_to_servo_id(joint)]
    leg, axis = divmod(joint, 3)
    return f"L{leg} {AXIS[axis]}"


def load_id_names() -> dict[int, str]:
    """Map servo bus ID → human name from motor_setup registry if present."""
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


def default_log_path(tag: str = "motion") -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return LOG_DIR / f"{tag}_{stamp}.csv"


@dataclass
class MotionLog:
    path: Path
    live: set[int]
    names: dict[int, str] = field(default_factory=load_id_names)
    sample_every: int = 1  # 1 = every control tick
    _fh: object | None = None
    _w: csv.DictWriter | None = None
    _tick: int = 0
    _t0: float = 0.0
    # Per-joint accumulators for the end-of-run summary.
    _err: dict[int, list[float]] = field(default_factory=dict)
    _spd: dict[int, list[float]] = field(default_factory=dict)
    _load: dict[int, list[float]] = field(default_factory=dict)
    _present: dict[int, list[float]] = field(default_factory=dict)
    _wrote_n: dict[int, int] = field(default_factory=dict)
    _samples: int = 0
    _miss: dict[int, int] = field(default_factory=dict)

    def __enter__(self) -> "MotionLog":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", newline="")
        self._w = csv.DictWriter(self._fh, fieldnames=[
            "t_s", "tick", "joint", "id", "name",
            "cmd_deg", "present_deg", "err_deg",
            "speed_deg_s", "load_pct", "current_a", "volt",
            "wrote", "cmd_speed", "cmd_acc", "moving",
            "ax_g", "ay_g", "az_g",
            "gx_dps", "gy_dps", "gz_dps", "temp_c",
            "roll_deg", "pitch_deg",
            "balance_axis", "balance_axis_roll", "balance_axis_pitch",
            "balance_target_pitch_deg", "balance_pitch_deg",
            "balance_err_deg", "balance_rate_deg_s",
            "balance_pitch_trim_deg", "balance_dx_trim_mm",
            "balance_speed_scale",
        ])
        self._w.writeheader()
        self._t0 = time.monotonic()
        print(f"  Telemetry → {self.path}")
        return self

    def __exit__(self, *exc) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        summary = self.summary()
        try:
            side = self.path.with_name(self.path.stem + "_summary.json")
            side.write_text(json.dumps(summary, indent=2))
            print(f"  Summary → {side.name}")
        except OSError as e:
            print(f"  Summary write failed: {e}")
        self.print_summary()

    def summary(self) -> dict:
        """Compact per-joint cmd-vs-actual stats for calibrate / UI."""
        rows = []
        for joint, errs in self._err.items():
            if len(errs) < 3:
                continue
            presents = self._present.get(joint, [])
            try:
                std_err = statistics.pstdev(errs) if len(errs) > 1 else 0.0
                std_pos = statistics.pstdev(presents) if len(presents) > 1 else 0.0
            except statistics.StatisticsError:
                std_err = 0.0
                std_pos = 0.0
            rms_err = (sum(e * e for e in errs) / len(errs)) ** 0.5
            mean_load = sum(self._load.get(joint, [0.0])) / max(
                1, len(self._load.get(joint, [0])))
            peak_load = max(self._load.get(joint, [0.0]) or [0.0])
            write_frac = self._wrote_n.get(joint, 0) / max(1, self._samples)
            shake = std_err * 2.0 + rms_err
            if write_frac < 0.15:
                shake += std_pos * 3.0
            if not errs or (rms_err < 0.35 and shake < 0.6):
                grade = "green"
            elif rms_err < 1.2 and shake < 2.0:
                grade = "yellow"
            else:
                grade = "red"
            sid = joint_to_servo_id(joint)
            rows.append({
                "joint": joint,
                "id": sid,
                "name": joint_name(joint, self.names),
                "axis": AXIS[joint % 3],
                "rms_err_deg": round(rms_err, 3),
                "std_err_deg": round(std_err, 3),
                "std_pos_deg": round(std_pos, 3),
                "mean_load_pct": round(mean_load, 1),
                "peak_load_pct": round(peak_load, 1),
                "write_frac": round(write_frac, 3),
                "shake": round(shake, 3),
                "grade": grade,
                "samples": len(errs),
            })
        rows.sort(key=lambda r: r["shake"], reverse=True)
        counts = {"green": 0, "yellow": 0, "red": 0}
        for r in rows:
            counts[r["grade"]] = counts.get(r["grade"], 0) + 1
        return {
            "ok": True,
            "log": str(self.path),
            "log_name": self.path.name,
            "samples": self._samples,
            "counts": counts,
            "rows": rows,
            "noisiest": [r["name"] for r in rows[:3]],
            "hint": (
                "Demo telemetry: cmd vs encoder while the motion ran. "
                "red = large tracking error / hunt; yellow = mild chatter; "
                "green = tracked cleanly"
            ),
        }

    def sample(self, bus: FeetechBus, cmd: list[float],
               *, wrote: dict[int, tuple[int, int]] | None = None,
               extra: dict | None = None) -> None:
        """Read live joints and append CSV rows.

        ``wrote`` maps joint index → (cmd_speed, cmd_acc) for joints that
        received a SyncWrite this tick (empty/None = none written).
        """
        self._tick += 1
        if (self._tick - 1) % max(1, self.sample_every) != 0:
            return
        wrote = wrote or {}
        t = time.monotonic() - self._t0
        self._samples += 1

        imu: dict | None = None
        read_imu = getattr(bus, "read_imu", None)
        if callable(read_imu):
            try:
                imu = read_imu()
            except Exception:
                imu = None
        if imu:
            roll = math.degrees(math.atan2(
                float(imu["ay_g"]), float(imu["az_g"])))
            pitch = math.degrees(math.atan2(
                -float(imu["ax_g"]),
                math.hypot(float(imu["ay_g"]), float(imu["az_g"]))))
            imu_cols = {
                "ax_g": f"{float(imu['ax_g']):.4f}",
                "ay_g": f"{float(imu['ay_g']):.4f}",
                "az_g": f"{float(imu['az_g']):.4f}",
                "gx_dps": f"{float(imu['gx_dps']):.2f}",
                "gy_dps": f"{float(imu['gy_dps']):.2f}",
                "gz_dps": f"{float(imu['gz_dps']):.2f}",
                "temp_c": f"{float(imu['temp_c']):.1f}",
                "roll_deg": f"{roll:.2f}",
                "pitch_deg": f"{pitch:.2f}",
            }
        else:
            imu_cols = {
                "ax_g": "", "ay_g": "", "az_g": "",
                "gx_dps": "", "gy_dps": "", "gz_dps": "", "temp_c": "",
                "roll_deg": "", "pitch_deg": "",
            }
        extra_cols = {
            "balance_axis": "",
            "balance_axis_roll": "",
            "balance_axis_pitch": "",
            "balance_target_pitch_deg": "",
            "balance_pitch_deg": "",
            "balance_err_deg": "",
            "balance_rate_deg_s": "",
            "balance_pitch_trim_deg": "",
            "balance_dx_trim_mm": "",
            "balance_speed_scale": "",
        }
        if extra:
            for k in extra_cols:
                if k in extra:
                    extra_cols[k] = extra[k]

        current_sum = 0.0
        n_fb = 0
        # Prefer one bulk MCU FeedBack for all live IDs (≫ faster than
        # per-joint R1/R2 spam on the host↔MCU link).
        bulk: dict[int, dict] = {}
        read_all = getattr(bus, "read_all_feedback", None)
        if callable(read_all):
            try:
                bulk = read_all(sorted(self.live)) or {}
            except Exception:
                bulk = {}

        for joint in range(N_JOINTS):
            sid = joint_to_servo_id(joint)
            if sid not in self.live:
                continue
            fb = bulk.get(joint)
            if fb is None:
                fb = bus.read_feedback(joint)
            if fb is None:
                self._miss[joint] = self._miss.get(joint, 0) + 1
                continue
            moving = int(fb["moving"]) if fb.get("moving") is not None else -1
            if moving < 0:
                try:
                    mv, r, _e = bus.pkt.read1ByteTxRx(sid, ADDR_MOVING)
                    if r == bus.scs.COMM_SUCCESS:
                        moving = int(mv)
                except Exception:
                    pass

            cmd_deg = float(cmd[joint]) if joint < len(cmd) else 0.0
            present = float(fb["deg"])
            err = present - cmd_deg
            w_speed, w_acc = wrote.get(joint, (0, 0))
            did_write = 1 if joint in wrote else 0
            cur_a = float(fb["current_a"])
            current_sum += abs(cur_a)
            n_fb += 1

            row = {
                "t_s": f"{t:.4f}",
                "tick": self._tick,
                "joint": joint,
                "id": sid,
                "name": joint_name(joint, self.names),
                "cmd_deg": f"{cmd_deg:.3f}",
                "present_deg": f"{present:.3f}",
                "err_deg": f"{err:.3f}",
                "speed_deg_s": f"{float(fb.get('speed_deg_s', 0.0)):.2f}",
                "load_pct": f"{float(fb['load_pct']):.1f}",
                "current_a": f"{cur_a:.3f}",
                "volt": f"{float(fb['volt']):.2f}",
                "wrote": did_write,
                "cmd_speed": w_speed,
                "cmd_acc": w_acc,
                "moving": moving,
            }
            row.update(imu_cols)
            row.update(extra_cols)
            self._w.writerow(row)

            self._err.setdefault(joint, []).append(err)
            self._spd.setdefault(joint, []).append(abs(float(fb.get("speed_deg_s", 0.0))))
            self._load.setdefault(joint, []).append(float(fb["load_pct"]))
            self._present.setdefault(joint, []).append(present)
            if did_write:
                self._wrote_n[joint] = self._wrote_n.get(joint, 0) + 1

        if self._fh is not None and self._samples % 10 == 0:
            self._fh.flush()

        # One streamed tick summary (amps + IMU) — not 18 joint rows.
        try:
            from event_log import emit
            data = {
                "tick": self._tick,
                "t_s": round(t, 4),
                "log": self.path.name,
                "joints": n_fb,
                "current_a": round(current_sum, 3),
            }
            if imu:
                roll = math.degrees(math.atan2(
                    float(imu["ay_g"]), float(imu["az_g"])))
                pitch = math.degrees(math.atan2(
                    -float(imu["ax_g"]),
                    math.hypot(float(imu["ay_g"]), float(imu["az_g"]))))
                data.update({
                    "ax_g": round(float(imu["ax_g"]), 4),
                    "ay_g": round(float(imu["ay_g"]), 4),
                    "az_g": round(float(imu["az_g"]), 4),
                    "gx_dps": round(float(imu["gx_dps"]), 2),
                    "gy_dps": round(float(imu["gy_dps"]), 2),
                    "gz_dps": round(float(imu["gz_dps"]), 2),
                    "roll_deg": round(roll, 2),
                    "pitch_deg": round(pitch, 2),
                    "temp_c": round(float(imu["temp_c"]), 1),
                    "imu_calibrated": bool(imu.get("calibrated")),
                })
            if extra:
                data.update(extra)
            emit("telemetry", f"tick {self._tick}", src="motion", data=data)
        except Exception:
            pass

    def print_summary(self) -> None:
        if not self._err:
            print("  Telemetry: no samples.")
            return

        # Prefer std of tracking *error* over std of present angle — during a
        # shimmy every yaw has large std_pos on purpose.
        rows = []
        for joint, errs in self._err.items():
            if len(errs) < 3:
                continue
            presents = self._present.get(joint, [])
            try:
                std_err = statistics.pstdev(errs) if len(errs) > 1 else 0.0
                std_pos = statistics.pstdev(presents) if len(presents) > 1 else 0.0
            except statistics.StatisticsError:
                std_err = 0.0
                std_pos = 0.0
            rms_err = (sum(e * e for e in errs) / len(errs)) ** 0.5
            mean_load = sum(self._load.get(joint, [0.0])) / max(
                1, len(self._load.get(joint, [0])))
            write_frac = self._wrote_n.get(joint, 0) / max(1, self._samples)
            # Shake ≈ chatter around the command (not "how much it moved").
            shake = std_err * 2.0 + rms_err
            if write_frac < 0.15:
                shake += std_pos * 3.0  # idle joint that still jitters
            rows.append((shake, joint, rms_err, std_err, std_pos, mean_load,
                         write_frac))

        rows.sort(reverse=True)
        active = [r for r in rows if r[0] > 0.05]
        print()
        print(f"  Telemetry summary  ({self._samples} ticks → {self.path.name})")
        print(f"  {'rank':>4}  {'name':<10}  {'id':>3}  {'rms_err':>7}  "
              f"{'std_err':>7}  {'std_pos':>7}  {'load%':>6}  {'write%':>6}  "
              f"shake")
        for i, (shake, joint, rms, std_err, std_pos, mload,
                wfrac) in enumerate((active or rows)[:8], 1):
            sid = joint_to_servo_id(joint)
            print(f"  {i:4d}  {joint_name(joint, self.names):<10}  {sid:3d}  "
                  f"{rms:7.2f}  {std_err:7.2f}  {std_pos:7.2f}  "
                  f"{mload:6.1f}  {100 * wfrac:5.1f}%  {shake:5.2f}")
        if not active:
            print("  → all quiet (no tracking chatter). Good for a hold test.")
        elif len(active) >= 2 and active[0][0] > 0.3:
            a, b = active[0], active[1]
            print(f"  → noisiest tracking: "
                  f"{joint_name(a[1], self.names)} (ID {joint_to_servo_id(a[1])}) "
                  f"and {joint_name(b[1], self.names)} (ID {joint_to_servo_id(b[1])})")
            print("  Tip: std_err = chatter around the command. Compare yaws "
                  "to each other — shakers reverse/jitter more.")
        miss_total = sum(self._miss.values())
        if miss_total:
            print(f"  (bus misses: {miss_total} read failures)")


def run_hold_log(bus: FeetechBus, live: set[int], *,
                 seconds: float = 5.0, recommand: bool = False,
                 path: Path | None = None,
                 abort_check=None) -> Path:
    """Hold present pose and log — isolates hold-hunt from motion tracking.

    ``recommand=False`` (default): command once, then only read.
    ``recommand=True``: re-send hold every tick (old footgun behaviour).
    """
    path = path or default_log_path("hold")
    check = abort_check or (lambda: False)
    # Snapshot pose.
    pose = [0.0] * N_JOINTS
    for j in range(N_JOINTS):
        sid = joint_to_servo_id(j)
        if sid not in live:
            continue
        deg = bus.read_position_deg(j)
        pose[j] = 0.0 if deg is None else deg
        bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)

    from feetech_bus import HOLD_ACC, HOLD_SPEED, deg_to_count
    # One gentle hold command.
    for j, deg in enumerate(pose):
        sid = joint_to_servo_id(j)
        if sid not in live:
            continue
        bus.pkt.SyncWritePosEx(sid, deg_to_count(j, deg, bus.trims[j]),
                               HOLD_SPEED, HOLD_ACC)
    if live:
        bus.pkt.groupSyncWrite.txPacket()
        bus.pkt.groupSyncWrite.clearParam()

    mode = "re-command every tick" if recommand else "command once, then only read"
    print(f"  Hold-log for {seconds:.1f}s ({mode})")
    dt = 0.05
    n = max(1, int(seconds / dt))
    with MotionLog(path, live) as log:
        for _ in range(n):
            if check():
                print("    hold-log aborted.")
                break
            wrote = {}
            if recommand:
                for j, deg in enumerate(pose):
                    sid = joint_to_servo_id(j)
                    if sid not in live:
                        continue
                    bus.pkt.SyncWritePosEx(
                        sid, deg_to_count(j, deg, bus.trims[j]),
                        HOLD_SPEED, HOLD_ACC)
                    wrote[j] = (HOLD_SPEED, HOLD_ACC)
                if wrote:
                    bus.pkt.groupSyncWrite.txPacket()
                    bus.pkt.groupSyncWrite.clearParam()
            log.sample(bus, pose, wrote=wrote)
            time.sleep(dt)
    return path
