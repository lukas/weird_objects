"""MPU-6050 rest calibration: gyro bias + accel sensor bias at a still pose.

Hold the chassis still (sit or stand; limp is fine). Collects samples via
``bus.read_imu()``, writes ``logs/imu_calib.json``, and the MCU bus applies
offsets on later reads / motion logs.

Accel bias assumes |g| ≈ 1 at rest: ``bias = mean − unit(mean)``. Gyro bias
is the mean rate while still.
"""
from __future__ import annotations

import json
import math
import statistics
import time
from pathlib import Path
from typing import Callable

_HERE = Path(__file__).resolve().parent
LOG_DIR = _HERE / "logs"

IMU_PATH_CANDIDATES = (
    LOG_DIR / "imu_calib.json",
    Path.home() / "hexapod_sts" / "linux_control" / "logs" / "imu_calib.json",
    Path.home() / "hexapod_sts" / "logs" / "imu_calib.json",
    _HERE / "imu_calib.json",
)

DEFAULT_DURATION_S = 2.5
DEFAULT_SAMPLE_HZ = 40.0
# Stillness gates (pre-bias gyro peak-to-peak / residual RMS).
GYRO_PTP_GREEN_DPS = 8.0
GYRO_PTP_YELLOW_DPS = 25.0
GYRO_RMS_GREEN_DPS = 1.5
ACCEL_MAG_LO = 0.75
ACCEL_MAG_HI = 1.35


def imu_calib_path() -> Path:
    for path in IMU_PATH_CANDIDATES:
        parent = path.parent
        if parent.is_dir() or path == IMU_PATH_CANDIDATES[0]:
            return path
    return IMU_PATH_CANDIDATES[0]


def load_imu_calib() -> dict | None:
    """Return calib dict or None if missing/invalid."""
    for path in IMU_PATH_CANDIDATES:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
            gb = data["gyro_bias_dps"]
            ab = data["accel_bias_g"]
            out = {
                "gyro_bias_dps": {
                    "x": float(gb["x"]),
                    "y": float(gb["y"]),
                    "z": float(gb["z"]),
                },
                "accel_bias_g": {
                    "x": float(ab["x"]),
                    "y": float(ab["y"]),
                    "z": float(ab["z"]),
                },
                "accel_rest_g": {
                    "x": float((data.get("accel_rest_g") or {}).get("x", 0)),
                    "y": float((data.get("accel_rest_g") or {}).get("y", 0)),
                    "z": float((data.get("accel_rest_g") or {}).get("z", 1)),
                },
                "path": str(path),
                "learned": True,
                "timestamp": data.get("timestamp"),
                "source": data.get("source") or "imu_calibrate",
                "n_samples": int(data.get("n_samples") or 0),
                "grade": data.get("grade"),
                "gyro_ptp_dps": data.get("gyro_ptp_dps"),
                "accel_mag_g": data.get("accel_mag_g"),
            }
            return out
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return None


def save_imu_calib(payload: dict) -> Path:
    path = imu_calib_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")
    return path


def clear_imu_calib() -> bool:
    cleared = False
    for path in IMU_PATH_CANDIDATES:
        if path.is_file():
            try:
                path.unlink()
                cleared = True
            except OSError:
                pass
    return cleared


def imu_state() -> dict:
    c = load_imu_calib()
    if not c:
        return {
            "ok": True,
            "learned": False,
            "path": str(imu_calib_path()),
            "gyro_bias_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            "accel_bias_g": {"x": 0.0, "y": 0.0, "z": 0.0},
            "msg": "no IMU calib yet",
        }
    return {
        "ok": True,
        "learned": True,
        "path": c["path"],
        "timestamp": c.get("timestamp"),
        "source": c.get("source"),
        "n_samples": c.get("n_samples"),
        "grade": c.get("grade"),
        "gyro_bias_dps": c["gyro_bias_dps"],
        "accel_bias_g": c["accel_bias_g"],
        "accel_rest_g": c.get("accel_rest_g"),
        "gyro_ptp_dps": c.get("gyro_ptp_dps"),
        "accel_mag_g": c.get("accel_mag_g"),
        "msg": f"learned · {c.get('grade') or '?'} · {c.get('timestamp') or ''}",
    }


def reset_imu_calib() -> dict:
    cleared = clear_imu_calib()
    st = imu_state()
    st["cleared"] = cleared
    st["msg"] = ("cleared — raw IMU until next calib"
                 if cleared or not st["learned"] else "already clear")
    return st


def apply_imu_calib(sample: dict, calib: dict | None) -> dict:
    """Return a copy with gyro/accel biases subtracted when calib is set."""
    out = dict(sample)
    if not calib:
        out["calibrated"] = False
        return out
    gb = calib["gyro_bias_dps"]
    ab = calib["accel_bias_g"]
    out["gx_dps"] = float(sample["gx_dps"]) - float(gb["x"])
    out["gy_dps"] = float(sample["gy_dps"]) - float(gb["y"])
    out["gz_dps"] = float(sample["gz_dps"]) - float(gb["z"])
    out["ax_g"] = float(sample["ax_g"]) - float(ab["x"])
    out["ay_g"] = float(sample["ay_g"]) - float(ab["y"])
    out["az_g"] = float(sample["az_g"]) - float(ab["z"])
    out["calibrated"] = True
    return out


def _mean(xs: list[float]) -> float:
    return float(statistics.fmean(xs)) if xs else 0.0


def _ptp(xs: list[float]) -> float:
    return (max(xs) - min(xs)) if xs else 0.0


def _rms(xs: list[float]) -> float:
    if not xs:
        return 0.0
    return math.sqrt(sum(v * v for v in xs) / len(xs))


def run_imu_calibrate(
    bus,
    *,
    duration_s: float = DEFAULT_DURATION_S,
    sample_hz: float = DEFAULT_SAMPLE_HZ,
    abort_check: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
) -> dict:
    """Still-chassis IMU calib. Does not move servos."""
    abort_check = abort_check or (lambda: False)
    on_progress = on_progress or (lambda _p: None)

    read_imu = getattr(bus, "read_imu", None)
    if not callable(read_imu):
        return {"ok": False, "error": "bus has no read_imu()", "mode": "imu"}

    # Prefer raw samples during calib (ignore any previous offsets).
    try:
        probe = read_imu(apply_calib=False)
    except TypeError:
        probe = read_imu()
    if probe is None:
        return {
            "ok": False,
            "mode": "imu",
            "error": "MPU not answering (need MCU Wire SDA/SCL + feetech_bridge)",
        }

    duration_s = max(0.8, float(duration_s))
    sample_hz = max(5.0, min(80.0, float(sample_hz)))
    dt = 1.0 / sample_hz
    n_target = max(20, int(duration_s * sample_hz))

    ax_s: list[float] = []
    ay_s: list[float] = []
    az_s: list[float] = []
    gx_s: list[float] = []
    gy_s: list[float] = []
    gz_s: list[float] = []
    temps: list[float] = []

    on_progress({"msg": "hold still — sampling IMU…", "index": 0,
                 "total": n_target})
    t0 = time.monotonic()
    misses = 0
    while len(ax_s) < n_target:
        if abort_check():
            return {"ok": False, "aborted": True, "mode": "imu",
                    "samples": len(ax_s)}
        try:
            s = read_imu(apply_calib=False)
        except TypeError:
            s = read_imu()
        if s is None:
            misses += 1
            if misses > 15:
                return {"ok": False, "mode": "imu",
                        "error": "IMU read failed mid-cal"}
            time.sleep(dt)
            continue
        misses = 0
        ax_s.append(float(s["ax_g"]))
        ay_s.append(float(s["ay_g"]))
        az_s.append(float(s["az_g"]))
        gx_s.append(float(s["gx_dps"]))
        gy_s.append(float(s["gy_dps"]))
        gz_s.append(float(s["gz_dps"]))
        temps.append(float(s.get("temp_c") or 0.0))
        if len(ax_s) % 10 == 0 or len(ax_s) == n_target:
            on_progress({
                "msg": f"hold still… {len(ax_s)}/{n_target}",
                "index": len(ax_s) - 1,
                "total": n_target,
            })
        # Pace samples (IMUR is slower than dt sometimes — just don't spin).
        elapsed = time.monotonic() - t0
        target_t = len(ax_s) * dt
        if target_t > elapsed:
            time.sleep(min(dt, target_t - elapsed))

    mx, my, mz = _mean(ax_s), _mean(ay_s), _mean(az_s)
    mag = math.sqrt(mx * mx + my * my + mz * mz) or 1.0
    ux, uy, uz = mx / mag, my / mag, mz / mag
    # Sensor bias = measured rest − 1 g along measured gravity.
    abx, aby, abz = mx - ux, my - uy, mz - uz
    gbx, gby, gbz = _mean(gx_s), _mean(gy_s), _mean(gz_s)

    gyro_ptp = max(_ptp(gx_s), _ptp(gy_s), _ptp(gz_s))
    gx_r = [v - gbx for v in gx_s]
    gy_r = [v - gby for v in gy_s]
    gz_r = [v - gbz for v in gz_s]
    gyro_rms = math.sqrt(
        (_rms(gx_r) ** 2 + _rms(gy_r) ** 2 + _rms(gz_r) ** 2) / 3.0)

    if (gyro_ptp <= GYRO_PTP_GREEN_DPS and gyro_rms <= GYRO_RMS_GREEN_DPS
            and ACCEL_MAG_LO <= mag <= ACCEL_MAG_HI):
        grade = "green"
    elif gyro_ptp <= GYRO_PTP_YELLOW_DPS and ACCEL_MAG_LO * 0.9 <= mag <= ACCEL_MAG_HI * 1.1:
        grade = "yellow"
    else:
        grade = "red"

    stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
    payload = {
        "timestamp": stamp,
        "source": "imu_calibrate",
        "n_samples": len(ax_s),
        "duration_s": round(time.monotonic() - t0, 3),
        "gyro_bias_dps": {"x": round(gbx, 4), "y": round(gby, 4),
                          "z": round(gbz, 4)},
        "accel_bias_g": {"x": round(abx, 5), "y": round(aby, 5),
                         "z": round(abz, 5)},
        "accel_rest_g": {"x": round(mx, 5), "y": round(my, 5),
                         "z": round(mz, 5)},
        "accel_mag_g": round(mag, 4),
        "gyro_ptp_dps": round(gyro_ptp, 3),
        "gyro_rms_dps": round(gyro_rms, 3),
        "temp_c": round(_mean(temps), 2),
        "grade": grade,
    }
    saved = False
    path = None
    if grade != "red":
        path = save_imu_calib(payload)
        saved = True
        reload = getattr(bus, "reload_imu_calib", None)
        if callable(reload):
            try:
                reload()
            except Exception:
                pass

    hint = {
        "green": "Still & |g|≈1 — biases saved; logs use calibrated IMU.",
        "yellow": "Mild motion during sample — saved anyway; re-run if shaky.",
        "red": "Too much motion or bad |g| — not saved. Hold still and retry.",
    }[grade]

    return {
        "ok": True,
        "mode": "imu",
        "saved": saved,
        "grade": grade,
        "hint": hint,
        "log": str(path) if path else None,
        "log_name": path.name if path else None,
        "samples": len(ax_s),
        "gyro_bias_dps": payload["gyro_bias_dps"],
        "accel_bias_g": payload["accel_bias_g"],
        "accel_rest_g": payload["accel_rest_g"],
        "accel_mag_g": payload["accel_mag_g"],
        "gyro_ptp_dps": payload["gyro_ptp_dps"],
        "gyro_rms_dps": payload["gyro_rms_dps"],
        "temp_c": payload["temp_c"],
        "imu": imu_state(),
        "rows": [
            {"axis": "gyro_bias", "x": gbx, "y": gby, "z": gbz,
             "unit": "dps"},
            {"axis": "accel_rest", "x": mx, "y": my, "z": mz, "unit": "g"},
            {"axis": "accel_bias", "x": abx, "y": aby, "z": abz, "unit": "g"},
        ],
    }
