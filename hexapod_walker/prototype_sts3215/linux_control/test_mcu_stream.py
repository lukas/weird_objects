"""Off-robot tests for the MCU stream-bridge codec (2026-08-19 upgrade).

Run locally (repo venv):  python3 linux_control/test_mcu_stream.py
No hardware: a FakeSerial plays the firmware side of the 'S'/'s'
combined write+snapshot transaction, byte-exact against the framing in
firmware/feetech_bridge (sendSnapshot / feedHostByte).
"""
from __future__ import annotations

import struct
import sys
import threading
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parent / "motor_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from feetech_bus import (N_JOINTS, count_to_deg, deg_to_count,
                         joint_to_servo_id, speed_counts_to_deg_s)
from mcu_feetech_bus import (SNAP_AGE_INVALID, SNAP_HEAD_LEN, SNAP_REC_LEN,
                             McuFeetechBus, encode_sync_frame,
                             parse_snapshot_payload)


# ---------------------------------------------------------------------------
# Unit conversion — the "wrong speed units" fix
# ---------------------------------------------------------------------------

def test_speed_counts_to_deg_s():
    # STS3215: counts/s, 4096/rev. 400 counts/s (the rl write_speed cap)
    # is ~35 deg/s — NOT the 0.732 rpm/unit SCS decode (which said 1757).
    assert abs(speed_counts_to_deg_s(400) - 35.15625) < 1e-9
    assert abs(speed_counts_to_deg_s(4096) - 360.0) < 1e-9
    assert speed_counts_to_deg_s(-100) == -speed_counts_to_deg_s(100)
    # The old bug was a ~50x inflation (49.97).
    assert abs((0.732 * 6.0) / (360.0 / 4096.0) - 50.0) < 0.05


# ---------------------------------------------------------------------------
# Frame encode (host -> MCU), matches firmware feedHostByte's reader
# ---------------------------------------------------------------------------

def _xor(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def test_encode_sync_frame_layout():
    items = [(2, 2048, 400, 20), (3, -12, 65535, 255)]
    frame = encode_sync_frame(ord("S"), items)
    assert frame[:2] == b"\xa5\x5a"
    assert frame[2] == ord("S")
    assert frame[3] == 2
    assert len(frame) == 4 + 2 * 6 + 1
    # Checksum covers cmd, n and the records (firmware binXor).
    assert frame[-1] == _xor(frame[2:-1])
    sid, pos, spd, acc = struct.unpack_from("<BhHB", frame, 4)
    assert (sid, pos, spd, acc) == (2, 2048, 400, 20)
    sid, pos, spd, acc = struct.unpack_from("<BhHB", frame, 10)
    assert (sid, pos, spd, acc) == (3, -12, 65535, 255)


def test_encode_snapshot_query():
    # n=0 'S' = read-only snapshot (positions+speed+IMU, no write).
    frame = encode_sync_frame(ord("S"), [])
    assert frame == bytes([0xA5, 0x5A, ord("S"), 0, ord("S") ^ 0])


# ---------------------------------------------------------------------------
# Snapshot decode (MCU -> host), byte-exact vs firmware sendSnapshot
# ---------------------------------------------------------------------------

def _fw_snapshot_payload(seq, pos_age, imu_age, imu_raw, servos) -> bytes:
    payload = struct.pack("<HHH", seq, pos_age, imu_age)
    payload += struct.pack("<7h", *imu_raw)
    for sid, ok, pos, spd in servos:
        payload += struct.pack("<BBhh", sid, ok, pos, spd)
    return payload


def _fw_snapshot_frame(payload: bytes, n: int) -> bytes:
    body = bytes([ord("s"), n]) + payload
    return bytes([0xA5, 0x5A]) + body + bytes([_xor(body)])


def test_parse_snapshot_payload():
    imu_raw = (100, -200, 16384, -5, 6, -7, 1234)
    servos = [(2 + j, 1, 2048 + j, -30 + j) for j in range(18)]
    payload = _fw_snapshot_payload(7, 3, 2, imu_raw, servos)
    assert len(payload) == SNAP_HEAD_LEN + 18 * SNAP_REC_LEN
    snap = parse_snapshot_payload(18, payload)
    assert snap["seq"] == 7
    assert snap["pos_age_ms"] == 3
    assert snap["imu_age_ms"] == 2
    assert snap["imu_raw"] == imu_raw
    assert len(snap["servos"]) == 18
    assert snap["servos"][0] == {"id": 2, "ok": True,
                                 "pos_counts": 2048, "spd_counts_s": -30}
    assert snap["servos"][17]["spd_counts_s"] == -13


# ---------------------------------------------------------------------------
# Full step_all transaction against a fake firmware
# ---------------------------------------------------------------------------

class FakeSerial:
    def __init__(self, reply: bytes):
        self._rx = bytearray(reply)
        self.tx = bytearray()

    def read(self, n: int = 1) -> bytes:
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def write(self, data) -> None:
        self.tx += bytes(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass


def _mk_bus(reply: bytes) -> McuFeetechBus:
    bus = McuFeetechBus.__new__(McuFeetechBus)
    bus._ser = FakeSerial(reply)
    bus._lock = threading.Lock()
    bus.trims = [0.0] * N_JOINTS
    bus._pos_cache = {}
    bus._pos_cache_mono = 0.0
    bus._fb_cache = {}
    bus._fb_cache_mono = 0.0
    bus._imu_calib = None
    bus._imu_mount = "normal"
    bus.has_stream = True
    bus.streaming = True
    return bus


def test_step_all_round_trip():
    imu_raw = (0, 0, 16384, 131, -131, 0, 0)   # flat, 1 g, ±1 dps
    servos = [(2 + j, 1, 2048 + 10 * j, 40) for j in range(18)]
    reply = _fw_snapshot_frame(
        _fw_snapshot_payload(42, 4, 1, imu_raw, servos), 18)
    bus = _mk_bus(reply)

    degrees = [float(j) for j in range(N_JOINTS)]
    snap = bus.step_all(degrees, speed=400, acc=20)
    assert snap is not None

    # TX side: exactly the 'S' frame write_all would have sync-written.
    want_items = [(joint_to_servo_id(j), deg_to_count(j, degrees[j], 0.0),
                   400, 20) for j in range(N_JOINTS)]
    assert bytes(bus._ser.tx) == encode_sync_frame(ord("S"), want_items)

    # RX side: engineering units.
    assert snap["seq"] == 42 and snap["pos_age_ms"] == 4
    for j in range(N_JOINTS):
        assert abs(snap["pos_deg"][j]
                   - count_to_deg(j, 2048 + 10 * j)) < 1e-9
        assert abs(snap["speed_deg_s"][j]
                   - speed_counts_to_deg_s(40)) < 1e-9
    imu = snap["imu"]
    assert imu is not None
    assert abs(imu["az_g"] - 1.0) < 1e-6
    assert abs(imu["gx_dps"] - 1.0) < 1e-6
    assert abs(imu["gy_dps"] + 1.0) < 1e-6
    # Position cache refreshed for read_position_deg fast hits.
    assert abs(bus._pos_cache[0] - count_to_deg(0, 2048)) < 1e-9


def test_step_all_marks_dead_servo():
    imu_raw = (0, 0, 16384, 0, 0, 0, 0)
    servos = [(2 + j, 1 if j != 5 else 0, 2000, 0) for j in range(18)]
    reply = _fw_snapshot_frame(
        _fw_snapshot_payload(1, 2, 3, imu_raw, servos), 18)
    bus = _mk_bus(reply)
    snap = bus.step_all([0.0] * N_JOINTS)
    assert snap is not None
    assert 5 not in snap["pos_deg"]          # dead servo omitted
    assert len(snap["pos_deg"]) == 17


def test_step_all_bad_checksum_returns_none():
    imu_raw = (0, 0, 16384, 0, 0, 0, 0)
    servos = [(2 + j, 1, 2048, 0) for j in range(18)]
    frame = bytearray(_fw_snapshot_frame(
        _fw_snapshot_payload(1, 2, 3, imu_raw, servos), 18))
    frame[-1] ^= 0xFF                        # corrupt checksum
    bus = _mk_bus(bytes(frame))
    assert bus.step_all([0.0] * N_JOINTS) is None


def test_snapshot_imu_invalid_age():
    servos = [(2 + j, 1, 2048, 0) for j in range(18)]
    reply = _fw_snapshot_frame(
        _fw_snapshot_payload(1, 2, SNAP_AGE_INVALID,
                             (0, 0, 0, 0, 0, 0, 0), servos), 18)
    bus = _mk_bus(reply)
    snap = bus.read_snapshot()
    assert snap is not None
    assert snap["imu"] is None               # dead/asleep MPU -> no IMU
    # TX was the read-only n=0 query.
    assert bytes(bus._ser.tx) == encode_sync_frame(ord("S"), [])


def test_read_imu_prefers_stream_snapshot():
    imu_raw = (0, 0, 16384, 131, -131, 0, 0)
    servos = [(2 + j, 1, 2048, 0) for j in range(18)]
    reply = _fw_snapshot_frame(
        _fw_snapshot_payload(5, 2, 1, imu_raw, servos), 18)
    bus = _mk_bus(reply)

    imu = bus.read_imu()
    assert imu is not None
    assert abs(imu["az_g"] - 1.0) < 1e-6
    assert abs(imu["gx_dps"] - 1.0) < 1e-6
    assert abs(imu["gy_dps"] + 1.0) < 1e-6
    assert bytes(bus._ser.tx) == encode_sync_frame(ord("S"), [])


def _main() -> int:
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}: {e}")
    print("OK" if fails == 0 else f"{fails} FAILURES")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_main())
