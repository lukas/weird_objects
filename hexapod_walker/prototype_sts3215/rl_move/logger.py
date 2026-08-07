"""Async episode logger (CSV v1) — never flush on the control thread."""
from __future__ import annotations

import csv
import json
import queue
import threading
import time
from pathlib import Path
from typing import Any


class EpisodeLogger:
    def __init__(self, log_dir: str | Path, *, fmt: str = "csv",
                 prefix: str = "ep"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.fmt = fmt
        self.prefix = prefix
        self._q: queue.Queue = queue.Queue(maxsize=5000)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fh = None
        self._writer = None
        self._path: Path | None = None
        self._fields: list[str] | None = None
        self.dropped = 0

    def start_episode(self, episode_id: int | str) -> Path:
        self.close()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        ext = "csv" if self.fmt == "csv" else "jsonl"
        self._path = self.log_dir / f"{self.prefix}_{episode_id}_{stamp}.{ext}"
        self._fh = open(self._path, "w", encoding="utf-8", newline="")
        self._fields = None
        self._writer = None
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="rl-balance-log", daemon=True)
        self._thread.start()
        return self._path

    def log(self, row: dict[str, Any]) -> None:
        try:
            self._q.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def close(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass
        self._fh = None
        self._writer = None

    def _run(self) -> None:
        while not self._stop.is_set() or not self._q.empty():
            try:
                row = self._q.get(timeout=0.05)
            except queue.Empty:
                continue
            if self._fh is None:
                continue
            if self.fmt == "jsonl":
                self._fh.write(json.dumps(row, default=float) + "\n")
            else:
                if self._fields is None:
                    self._fields = list(row.keys())
                    self._writer = csv.DictWriter(self._fh, fieldnames=self._fields)
                    self._writer.writeheader()
                # Fill missing keys for ragged rows.
                assert self._writer is not None and self._fields is not None
                full = {k: row.get(k, "") for k in self._fields}
                for k, v in row.items():
                    if k not in full:
                        # Late new keys: ignore to keep CSV stable this episode.
                        continue
                    full[k] = v
                self._writer.writerow(full)
            if self._q.qsize() == 0:
                try:
                    self._fh.flush()
                except Exception:
                    pass


def flatten_state_row(*, episode_id, step_id, state, action_raw, action_clip,
                      q_ik, q_safe, reward, reward_parts, terminated,
                      truncated, reason, tick_overrun, extra=None) -> dict:
    row = {
        "episode_id": episode_id,
        "step_id": step_id,
        "timestamp": state.timestamp,
        "actual_dt": state.dt,
        "overrun": int(bool(tick_overrun)),
        "bus_ok": int(state.bus_ok),
        "imu_ok": int(state.imu_ok),
        "roll": state.imu_roll,
        "pitch": state.imu_pitch,
        "yaw": state.imu_yaw,
        "gyro_x": float(state.imu_gyro[0]),
        "gyro_y": float(state.imu_gyro[1]),
        "gyro_z": float(state.imu_gyro[2]),
        "accel_x": float(state.imu_accel[0]),
        "accel_y": float(state.imu_accel[1]),
        "accel_z": float(state.imu_accel[2]),
        "reward": reward,
        "terminated": int(bool(terminated)),
        "truncated": int(bool(truncated)),
        "termination_reason": reason or "",
        "t_pos": state.timing.get("t_pos", ""),
        "t_imu": state.timing.get("t_imu", ""),
        "t_fb": state.timing.get("t_fb", ""),
        "t_acq": state.timing.get("t_total", ""),
    }
    for i in range(5):
        row[f"a_raw_{i}"] = float(action_raw[i]) if action_raw is not None else ""
        row[f"a_clip_{i}"] = float(action_clip[i]) if action_clip is not None else ""
    for j in range(18):
        row[f"q_{j}"] = float(state.joint_position[j])
        row[f"qd_{j}"] = float(state.joint_velocity[j])
        row[f"q_ik_{j}"] = float(q_ik[j]) if q_ik is not None else ""
        row[f"q_safe_{j}"] = float(q_safe[j]) if q_safe is not None else ""
    if state.servo_current is not None:
        for j in range(18):
            row[f"i_{j}"] = float(state.servo_current[j])
            row[f"load_{j}"] = float(state.servo_load[j])
            row[f"temp_{j}"] = float(state.servo_temperature[j])
    if reward_parts:
        for k, v in reward_parts.items():
            row[k] = v
    if extra:
        row.update(extra)
    return row
