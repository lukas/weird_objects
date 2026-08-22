"""Off-robot tests for calibration TFT progress hints."""
from __future__ import annotations

import sys
import threading
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from bench_api import BenchAPI  # noqa: E402
from mcu_feetech_bus import McuFeetechBus  # noqa: E402
from status_display import format_job_screen  # noqa: E402


class _Drive:
    dry_run = True
    bus = None
    armed = False

    def __init__(self) -> None:
        self._lock = threading.Lock()


def test_format_job_screen_names_calibration_phase() -> None:
    robot = {
        "activity": "calibrating",
        "armed": False,
        "demo": {
            "name": "calibrate:checkup:all",
            "running": True,
            "status": "calibrating",
            "progress": {
                "phase": "geometry_sweep",
                "mode": "checkup",
                "msg": "Geo sweep: L0 contact pose 3",
            },
        },
    }
    job = format_job_screen(robot)
    assert job is not None
    _pct, lines = job
    assert lines[0] == "CALIBRATING"
    assert lines[1] == "DIMENSION SWEEP"
    assert "Geo sweep" in " ".join(lines)


def test_bench_tft_lines_use_phase_and_footer() -> None:
    api = BenchAPI(_Drive())
    lines = api._calibration_tft_lines({
        "phase": "traction_probe",
        "msg": "Traction: shear +3.0 deg trial 1/2",
    })
    assert lines[0] == "CALIBRATING"
    assert lines[1] == "TRACTION / SLIP"
    assert lines[-1] == "watch robot"


def test_display_job_try_skips_when_lock_busy() -> None:
    bus = object.__new__(McuFeetechBus)
    bus._lock = threading.Lock()
    bus._lock.acquire()
    try:
        assert bus.display_job_try(["CALIBRATING"], timeout=0.01) is False
    finally:
        bus._lock.release()


if __name__ == "__main__":
    test_format_job_screen_names_calibration_phase()
    test_bench_tft_lines_use_phase_and_footer()
    test_display_job_try_skips_when_lock_busy()
    print("tft calibration progress tests passed")
