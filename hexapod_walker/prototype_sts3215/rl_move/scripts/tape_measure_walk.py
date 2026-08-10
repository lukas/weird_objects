"""Scripted-gait tape-measure session — ground-truth walk distance.

HARDWARE.md experiment backlog item 1, the last missing input for the
contact/current pricing calibration (the P0 blocker behind the RL
paddle/creep gait): drive the KNOWN-WORKING scripted tripod gait for a
timed leg, let the operator read the tape, and record
commanded-vs-measured distance next to per-servo currents and body tilt.

Everything goes over HTTP (never SSH). Per hardware-safety rules this
script NEVER arms, stands, zeros, or limps the robot — the operator does
`set_zero → ARM → P` from the web UI first, exactly like the 08-09
session that walked. The only commands this script sends are gait
velocity updates (`J vx vy omega`, and `J 0 0 0` to stop), and only:

- when started with ``--go`` (default is a dry run that prints the plan
  and sends NOTHING), and
- after the operator types ``go`` for each leg while watching the robot.

Ctrl-C or a tilt alert (>30 deg — a working gait rocks +-20) sends
``J 0 0 0`` (planted stand, torque stays on); limping is the operator's
call.

Outputs (in ``rl_move/hardware_traces/``):

- ``tape_<stamp>_<leg>_servo.csv`` — ~3 Hz (via the fast ``/api/feedback``
  bulk route), same columns as hw_session2_20260810.csv (t_unix, status,
  q/cur/temp per joint) so the existing calibration tooling reads it
  unchanged.
- ``tape_<stamp>_<leg>_imu.csv``  — same cadence, t_unix, roll_deg,
  pitch_deg (same shape as imu_walk_20260810.csv).
- ``tape_<stamp>_summary.json``   — per leg: command, duration,
  commanded distance, TAPE-MEASURED distance/drift, slip ratio, tilt
  envelope, current stats, trace filenames.

Usage (bench, operator watching, robot ARMed and standing at plant):

    python -m rl_move.scripts.tape_measure_walk            # dry run
    python -m rl_move.scripts.tape_measure_walk --go       # default plan
    python -m rl_move.scripts.tape_measure_walk --go \
        --leg fwd30:30,0,0,20 --leg crab30:0,30,0,20

Default plan mirrors the 08-09 working session: fwd 30 mm/s x 20 s and
fwd 50 mm/s x 20 s (commanded 600 / 1000 mm).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[2]))  # prototype_sts3215/

from rl_move.remote import HexapodClient  # noqa: E402

TRACES_DIR = _HERE.parents[1] / "hardware_traces"
N_JOINTS = 18

# Teleop caps (drive_controller clips harder; stay inside known-good).
MAX_VX_MM = 60.0
MAX_VY_MM = 40.0
MAX_OMEGA = 0.5
MAX_LEG_S = 60.0
# Alert threshold: a WORKING gait rocks +-10-20 deg (measured 08-09);
# beyond 30 deg something is wrong -> stop the gait and tell the operator.
TILT_ALERT_DEG = 30.0

# /api/feedback = one MCU bulk transaction + IMU read; ~3 Hz sustains
# fine alongside the 20 Hz walk loop (same read path servo_watch uses).
FEEDBACK_POLL_S = 0.3


@dataclass
class Leg:
    name: str
    vx_mm: float
    vy_mm: float
    omega: float
    seconds: float

    @property
    def speed_mm_s(self) -> float:
        return math.hypot(self.vx_mm, self.vy_mm)

    @property
    def commanded_mm(self) -> float:
        """|v| * T. The gait low-passes commands with tau=0.15 s, so the
        true commanded path is ~|v|*0.15 s shorter; noted in the summary,
        irrelevant next to tape-measure precision."""
        return self.speed_mm_s * self.seconds

    def describe(self) -> str:
        parts = []
        if self.vx_mm:
            parts.append(f"vx={self.vx_mm:+.0f}mm/s")
        if self.vy_mm:
            parts.append(f"vy={self.vy_mm:+.0f}mm/s")
        if self.omega:
            parts.append(f"omega={self.omega:+.2f}rad/s (+ = clockwise)")
        return f"{self.name}: {' '.join(parts) or 'zero'} for {self.seconds:.0f}s"


def parse_leg(spec: str) -> Leg:
    """``name:vx,vy,omega,seconds`` (mm/s, mm/s, rad/s, s)."""
    try:
        name, rest = spec.split(":", 1)
        vx, vy, om, secs = (float(x) for x in rest.split(","))
    except ValueError:
        raise SystemExit(f"bad --leg {spec!r} (want name:vx,vy,omega,seconds)")
    if abs(vx) > MAX_VX_MM or abs(vy) > MAX_VY_MM or abs(om) > MAX_OMEGA:
        raise SystemExit(f"--leg {spec!r} exceeds caps "
                         f"(|vx|<={MAX_VX_MM:.0f}mm/s |vy|<={MAX_VY_MM:.0f} "
                         f"|omega|<={MAX_OMEGA})")
    secs = min(max(secs, 3.0), MAX_LEG_S)
    return Leg(name.strip() or "leg", vx, vy, om, secs)


DEFAULT_PLAN = [
    Leg("fwd30", 30.0, 0.0, 0.0, 20.0),
    Leg("fwd50", 50.0, 0.0, 0.0, 20.0),
]


@dataclass
class Telemetry:
    """Background /api/feedback poller -> servo + imu CSVs + tilt alert.

    One sample = one MCU bulk feedback transaction + one IMU read, split
    into the two CSV shapes the calibration tooling already reads
    (hw_session2_*.csv and imu_walk_*.csv from the 08-09 session).
    """
    client: HexapodClient
    servo_csv: Path
    imu_csv: Path
    rows_servo: int = 0
    rows_imu: int = 0
    max_roll: float = 0.0
    max_pitch: float = 0.0
    cur_sums: list = field(default_factory=list)   # total bus A per sample
    tilt_alert: threading.Event = field(default_factory=threading.Event)
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=4.0)

    def _loop(self) -> None:
        hdr = ["t_unix", "status"]
        for j in range(N_JOINTS):
            hdr += [f"q{j}_deg", f"cur{j}_a", f"temp{j}_c"]
        with self.servo_csv.open("w", newline="") as fs, \
                self.imu_csv.open("w", newline="") as fi:
            ws = csv.writer(fs)
            ws.writerow(hdr)
            wi = csv.writer(fi)
            wi.writerow(["t_unix", "roll_deg", "pitch_deg"])
            while not self._stop.is_set():
                t0 = time.monotonic()
                fb = self.client.feedback()
                if fb.get("ok"):
                    t = fb.get("t_unix") or round(time.time(), 3)
                    joints = fb.get("joints") or []
                    if any(j for j in joints):
                        row = [t, f"live {fb.get('live', 0)}/18"]
                        total = 0.0
                        for j in range(N_JOINTS):
                            m = (joints[j] if j < len(joints)
                                 and joints[j] else {})
                            total += abs(float(m.get("cur_a", 0.0) or 0.0))
                            row += [m.get("deg", ""), m.get("cur_a", ""),
                                    m.get("temp_c", "")]
                        ws.writerow(row)
                        fs.flush()
                        self.rows_servo += 1
                        self.cur_sums.append(total)
                    roll, pitch = fb.get("roll_deg"), fb.get("pitch_deg")
                    if roll is not None and pitch is not None:
                        wi.writerow([t, roll, pitch])
                        fi.flush()
                        self.rows_imu += 1
                        self.max_roll = max(self.max_roll, abs(float(roll)))
                        self.max_pitch = max(self.max_pitch,
                                             abs(float(pitch)))
                        if (abs(float(roll)) > TILT_ALERT_DEG
                                or abs(float(pitch)) > TILT_ALERT_DEG):
                            self.tilt_alert.set()
                self._stop.wait(max(0.0, FEEDBACK_POLL_S
                                    - (time.monotonic() - t0)))


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip()
    except EOFError:
        return "q"


def ask_float(prompt: str) -> float | None:
    while True:
        s = ask(prompt)
        if s in ("", "-", "skip"):
            return None
        try:
            return float(s)
        except ValueError:
            print("  (number in mm, or Enter to skip)")


def run_leg(client: HexapodClient, leg: Leg, stamp: str) -> dict:
    servo_csv = TRACES_DIR / f"tape_{stamp}_{leg.name}_servo.csv"
    imu_csv = TRACES_DIR / f"tape_{stamp}_{leg.name}_imu.csv"
    tel = Telemetry(client, servo_csv, imu_csv)

    result: dict = {
        "leg": leg.name,
        "vx_mm_s": leg.vx_mm, "vy_mm_s": leg.vy_mm,
        "omega_rad_s": leg.omega, "seconds": leg.seconds,
        "commanded_mm": round(leg.commanded_mm, 1),
        "commanded_rot_deg": round(math.degrees(leg.omega * leg.seconds), 1),
        "cmd_smoothing_note": "gait vel low-pass tau=0.15s; commanded path "
                              "~|v|*0.15s shorter than |v|*T",
        "servo_csv": servo_csv.name, "imu_csv": imu_csv.name,
        "t_start_unix": round(time.time(), 3),
    }

    tel.start()
    stopped_reason = "duration"
    t0 = time.monotonic()
    r = client.cmd(f"J {leg.vx_mm:.1f} {leg.vy_mm:.1f} {leg.omega:.3f}")
    if r != "ok":
        tel.stop()
        result["error"] = f"J refused: {r}"
        return result
    print(f"  walking ({leg.describe()}) ...", flush=True)
    try:
        while time.monotonic() - t0 < leg.seconds:
            if tel.tilt_alert.is_set():
                stopped_reason = "tilt_alert"
                print(f"  !! tilt beyond {TILT_ALERT_DEG:.0f} deg — "
                      "stopping gait (robot HOLDS; X on the UI to limp)")
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        stopped_reason = "operator_interrupt"
        print("\n  Ctrl-C — stopping gait (robot HOLDS; X on the UI to limp)")
    walked_s = time.monotonic() - t0
    client.cmd("J 0 0 0")   # planted stand, torque stays on
    time.sleep(1.0)         # let it settle before the last samples
    tel.stop()

    result.update(
        stopped=stopped_reason,
        walked_s=round(walked_s, 2),
        commanded_mm=round(leg.speed_mm_s * walked_s, 1),
        commanded_rot_deg=round(math.degrees(leg.omega * walked_s), 1),
        max_abs_roll_deg=round(tel.max_roll, 1),
        max_abs_pitch_deg=round(tel.max_pitch, 1),
        servo_samples=tel.rows_servo, imu_samples=tel.rows_imu,
    )
    if tel.cur_sums:
        result["bus_current_a"] = {
            "mean": round(sum(tel.cur_sums) / len(tel.cur_sums), 3),
            "max": round(max(tel.cur_sums), 3),
        }

    print(f"  gait stopped after {walked_s:.1f}s "
          f"(commanded ~{result['commanded_mm']:.0f} mm). Read the tape.")
    meas = ask_float("  measured distance along travel direction (mm): ")
    drift = ask_float("  lateral drift (mm, Enter to skip): ")
    if leg.omega:
        rot = ask_float("  measured heading change (deg, + = CW): ")
        if rot is not None:
            result["measured_rot_deg"] = rot
    notes = ask("  notes (slip, rocking, snags — Enter for none): ")
    if meas is not None:
        result["measured_mm"] = meas
        if result["commanded_mm"] > 1e-6:
            result["slip_ratio_measured_over_commanded"] = round(
                meas / result["commanded_mm"], 3)
    if drift is not None:
        result["lateral_drift_mm"] = drift
    if notes:
        result["notes"] = notes
    return result


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=None, help="robot base URL")
    ap.add_argument("--go", action="store_true",
                    help="actually send gait commands (default: dry run)")
    ap.add_argument("--leg", action="append", default=[],
                    metavar="name:vx,vy,omega,seconds",
                    help="mm/s, mm/s, rad/s, s — repeatable; replaces the "
                         "default fwd30/fwd50 plan")
    ap.add_argument("--floor", default=None,
                    help="floor description for the summary (asked if absent)")
    args = ap.parse_args()

    plan = [parse_leg(s) for s in args.leg] or list(DEFAULT_PLAN)
    print("Plan:")
    for leg in plan:
        print(f"  {leg.describe()}  -> commanded "
              f"{leg.commanded_mm:.0f} mm"
              + (f" / {math.degrees(leg.omega * leg.seconds):.0f} deg"
                 if leg.omega else ""))
    if not args.go:
        print("\nDry run (no --go): nothing was sent to the robot.")
        return 0

    client = HexapodClient(args.url)
    ping = client.ping()
    if not ping.get("ok"):
        print(f"robot not reachable: {ping.get('error')}")
        return 1

    # This script never arms or stands the robot — verify the operator did.
    robot = client.robot()
    if not robot.get("armed"):
        print("Robot is not ARMed. Operator sequence first (web UI, per the "
              "08-09 working session): set_zero at a known pose -> ARM -> P "
              "(plant). Then rerun with --go.")
        return 1
    pf = client.preflight("lower")
    print(f"start attitude: roll {pf.get('roll_deg')} deg, "
          f"pitch {pf.get('pitch_deg')} deg; "
          f"pose delta vs plant {pf.get('max_pose_delta_deg')} deg "
          f"(tol {pf.get('pose_tol_deg')})")
    if not pf.get("ok"):
        print(f"NOTE: lower-preflight not green ({pf.get('error')}). The "
              "gait syncs stance to the PRESENT pose — stale/slumped "
              "stances caused the 08-09 falls. Fresh set_zero -> P "
              "recommended before walking.")

    floor = args.floor or ask("floor description (e.g. 'rough concrete, "
                              "rubber tips'): ")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    TRACES_DIR.mkdir(exist_ok=True)
    summary: dict = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "url": client.base,
        "floor": floor,
        "purpose": "scripted-gait ground-truth distance for contact/current "
                   "pricing calibration (HARDWARE.md backlog item 1)",
        "tilt_alert_deg": TILT_ALERT_DEG,
        "legs": [],
    }
    sum_path = TRACES_DIR / f"tape_{stamp}_summary.json"

    for leg in plan:
        print(f"\n=== {leg.describe()} ===")
        print("  Mark the start point (tape zero at the reference feature). "
              "Camera on the tape if you want video backup.")
        a = ask("  type 'go' to walk, 's' to skip, 'q' to quit: ").lower()
        if a == "q":
            break
        if a != "go":
            print("  skipped")
            continue
        res = run_leg(client, leg, stamp)
        summary["legs"].append(res)
        sum_path.write_text(json.dumps(summary, indent=1))  # survive aborts
        print(f"  saved -> {sum_path.name}")

    print(f"\nSession summary: {sum_path}")
    print("Robot is left STANDING with torque on — X (limp) or SETTLE from "
          "the web UI when you're done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
