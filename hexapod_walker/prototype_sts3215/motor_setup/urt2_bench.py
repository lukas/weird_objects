"""Shared Waveshare URT-2 bench helpers (raw servo ID, no joint model)."""
from __future__ import annotations

import glob
import os
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager
from pathlib import Path

from feetech_bus import (
    ADDR_TORQUE_ENABLE, COUNTS_PER_DEG, FeetechBus, STS_CENTRE_COUNT,
)

# STS3215 EEPROM: operating mode (0 = position/servo, 1 = wheel, 2 = PWM, 3 = step).
ADDR_MODE = 33
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMP = 63
ADDR_SERVO_STATUS = 65
ADDR_MOVING = 66
ADDR_PRESENT_CURRENT = 69
# EEPROM identity / limits / alarm config.
ADDR_FW_MAJOR = 0
ADDR_FW_MINOR = 1
ADDR_SERVO_MAJOR = 3
ADDR_SERVO_MINOR = 4
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
ADDR_MAX_TEMP_LIMIT = 13
ADDR_MAX_VOLTAGE_LIMIT = 14
ADDR_MIN_VOLTAGE_LIMIT = 15
ADDR_UNLOAD_CONDITION = 19
ADDR_LED_ALARM = 20
ADDR_PROTECTION_CURRENT = 28
ADDR_BAUD_RATE = 6   # EEPROM: 0=1M, 1=500k, 2=250k, 3=128k, 4=115200, …

# Host baud ↔ STS EEPROM reg 6
BAUD_TO_REG = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    76_800: 5,
    57_600: 6,
    38_400: 7,
}
REG_TO_BAUD = {v: k for k, v in BAUD_TO_REG.items()}

# Status reg 65 bits — same bits drive the red LED when LED-alarm is enabled.
STATUS_BITS = (
    (0, "voltage",
     "bus voltage outside servo min/max — if bus is ~12 V but limits "
     "show max≤8 V, EEPROM max-voltage was mis-set (restore to 4–14 V)"),
    (1, "sensor",
     "encoder / magnetic sensor fault"),
    (2, "temperature",
     "over-temp — let it cool; check binding / stalled load"),
    (3, "current",
     "over-current protection tripped — free the horn, then re-command"),
    (4, "angle",
     "angle / position limit fault"),
    (5, "overload",
     "overload (stalled >~80% for ~2 s) — limp, free path; "
     "a new position command clears the flag"),
)

# STS3215 factory defaults (register units: volts × 10).
STS_DEFAULT_VMAX = 140   # 14.0 V
STS_DEFAULT_VMIN = 40    # 4.0 V  (factory often 50; 40 is also common)
STS_DEFAULT_TMAX = 70    # °C



def default_port() -> str | None:
    """MCU bridge on Uno Q, else first USB URT serial device.

    On the board, ``mcu`` / ``/dev/ttyHS1`` is preferred (FE-URT on D0/D1).
    USB CH343 / ttyUSB still wins when explicitly present for laptop use.
    """
    env = os.environ.get("HEXAPOD_BUS_PORT")
    if env:
        return env
    # Uno Q MCU bridge (feetech_bridge sketch).
    if Path("/dev/ttyHS1").exists():
        return "mcu"
    for pattern in (
        "/dev/ttyCH343USB*",
        "/dev/ttyUSB*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.usbserial*",
        "/dev/cu.usbmodem*",
        "/dev/ttyACM*",
    ):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def decode_status(status: int) -> list[tuple[str, str]]:
    """Return [(name, hint), ...] for set bits in servo status byte."""
    return [(name, hint) for bit, name, hint in STATUS_BITS
            if status & (1 << bit)]


def read_servo_identity(bus: FeetechBus, sid: int) -> dict:
    """Firmware / series version + voltage limits (no uninstall needed)."""
    fw_m, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_FW_MAJOR)
    fw_s, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_FW_MINOR)
    sv_m, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_SERVO_MAJOR)
    sv_s, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_SERVO_MINOR)
    vmax, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_MAX_VOLTAGE_LIMIT)
    vmin, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_MIN_VOLTAGE_LIMIT)
    vnow, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_PRESENT_VOLTAGE)
    iprot, _, _ = bus.pkt.read2ByteTxRx(sid, ADDR_PROTECTION_CURRENT)
    status, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_SERVO_STATUS)
    vmax_v = vmax / 10.0
    vmin_v = vmin / 10.0
    # Heuristic only: Feetech 7.4 V SKUs often *ship* with max≈8.0 V;
    # 12 V / C018 SKUs ship with max≈14.0 V.  Max-voltage is EEPROM and
    # writable, so a low value can also mean a torn/odd write — not proof.
    if vmax_v <= 8.5:
        profile = "low-max (7.4V-SKU factory OR EEPROM glitch)"
    elif vmax_v >= 12.0:
        profile = "12V-safe EEPROM"
    else:
        profile = "odd mid max-voltage"
    return {
        "id": sid,
        "fw": f"{fw_m}.{fw_s}",
        "servo": f"{sv_m}.{sv_s}",
        "volt_limit_min": vmin_v,
        "volt_limit_max": vmax_v,
        "volt": vnow / 10.0,
        "iprot": iprot,
        "iprot_a": iprot * 0.0065,
        "status": status,
        "profile": profile,
    }


def print_identity_report(bus: FeetechBus, live: list[int] | None = None,
                          *, label_fn=None) -> list[dict]:
    """Dump identity + voltage profile for every answering servo."""
    if live is None:
        live = sorted(bus.scan(range(1, 31)))
    print()
    print("  Servo identity (queried over the bus — no uninstall)")
    print("  ------------------------------------------------------------")
    print("  Spec you want: FEETECH STS3215 ST-3215-C018 (12 V / 30 kg·cm).")
    print("  Same case exists as a 7.4 V / ~19 kg·cm SKU — sticker is the")
    print("  only hard physical proof; EEPROM max-voltage is a soft hint")
    print("  (writable, so '8 V max' ≠ guaranteed wrong motor).")
    print("  ------------------------------------------------------------")
    if not live:
        print("  (none answering)")
        return []
    print(f"  {'ID':>3}  {'name':<12}  {'fw':>5}  {'servo':>5}  "
          f"{'V limits':>11}  {'Vnow':>5}  {'Iprot':>6}  profile")
    rows = []
    low = []
    for sid in live:
        info = read_servo_identity(bus, sid)
        rows.append(info)
        name = ""
        if label_fn is not None:
            try:
                name = label_fn(sid) or ""
            except Exception:
                name = ""
        name = (name[:12] if name else "-")
        print(f"  {sid:3d}  {name:<12}  {info['fw']:>5}  {info['servo']:>5}  "
              f"{info['volt_limit_min']:.1f}-{info['volt_limit_max']:.1f}V  "
              f"{info['volt']:5.1f}  {info['iprot_a']:5.2f}A  "
              f"{info['profile']}")
        if info["volt_limit_max"] <= 8.5:
            low.append(sid)
    fws = sorted({r["fw"] for r in rows})
    svs = sorted({r["servo"] for r in rows})
    print()
    print(f"  firmware versions seen: {fws}")
    print(f"  servo-series versions:  {svs}")
    if len(fws) == 1 and len(svs) == 1:
        print("  All answering motors report the SAME fw/series — looks like "
              "one product family, not a mixed SKU batch.")
    if low:
        print(f"  IDs still with ≤8.5 V max EEPROM: {low}")
        print("  → heal with debug (d) / force-heal, or check case sticker "
              "for '7.4V' / '19kg' vs '12V' / '30kg' / 'C018'.")
    else:
        print("  No low max-voltage EEPROM right now (all look 12 V-safe).")
    return rows


# Feetech one-key middle calibrate: write 128 to torque-enable.
# Does NOT turn the horn — rewrites logical centre to the current pose.
TORQUE_CALIBRATE_MIDDLE = 128


def read_feedback_raw(bus: FeetechBus, sid: int) -> dict:
    """Position/load/volt/temp/current for a raw servo ID."""
    pos, result, _err = bus.pkt.ReadPos(sid)
    if result != bus.scs.COMM_SUCCESS:
        raise SystemExit(f"servo {sid}: position read failed")
    load, _r, _e = bus.pkt.read2ByteTxRx(sid, ADDR_PRESENT_LOAD)
    volt, _r, _e = bus.pkt.read1ByteTxRx(sid, ADDR_PRESENT_VOLTAGE)
    temp, _r, _e = bus.pkt.read1ByteTxRx(sid, ADDR_PRESENT_TEMP)
    cur, _r, _e = bus.pkt.read2ByteTxRx(sid, ADDR_PRESENT_CURRENT)
    return {
        "count": pos,
        "deg": (pos - STS_CENTRE_COUNT) / COUNTS_PER_DEG,
        "load_pct": (load & 0x3FF) / 10.0,
        "volt": volt / 10.0,
        "temp_c": temp,
        "current_a": cur * 0.0065,
    }


def apply_middle_calibrate(bus: FeetechBus, sid: int) -> dict:
    """Redefine *current* physical pose as logical 0° (count ~2048).

    Horn does not turn.  Returns ``{ok, before_deg, after_deg, before_count,
    after_count}`` or raises on hard failure.
    """
    before = read_feedback_raw(bus, sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.05)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, TORQUE_CALIBRATE_MIDDLE)
    time.sleep(0.15)
    after = read_feedback_raw(bus, sid)
    ok = (abs(after["count"] - STS_CENTRE_COUNT) <= 20
          or abs(after["deg"]) < 5.0)
    return {
        "ok": ok,
        "id": sid,
        "before_deg": before["deg"],
        "after_deg": after["deg"],
        "before_count": before["count"],
        "after_count": after["count"],
    }


def redefine_zero_here(bus: FeetechBus,
                       ids: list[int] | None = None) -> dict:
    """Middle-calibrate every live robot servo (IDs 2..19 by default).

    Limps first, calibrates each, leaves torque OFF.  Motors do not move.
    """
    live = sorted(bus.scan(range(2, 20)) if ids is None else ids)
    results = []
    ok_n = 0
    for sid in live:
        try:
            if not bus.ping(sid):
                results.append({"id": sid, "ok": False, "error": "missing"})
                continue
            r = apply_middle_calibrate(bus, sid)
            results.append(r)
            if r["ok"]:
                ok_n += 1
        except Exception as exc:
            results.append({"id": sid, "ok": False, "error": str(exc)})
    # Leave everything limp.
    for sid in live:
        try:
            bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
        except Exception:
            pass
    return {
        "ok": ok_n == len(live) and len(live) > 0,
        "count": len(live),
        "ok_n": ok_n,
        "results": results,
    }


def read_servo_health(bus: FeetechBus, sid: int) -> dict:
    """Feedback plus status/moving/torque + EEPROM limits / LED config."""
    fb = read_feedback_raw(bus, sid)
    status, r_st, err_st = bus.pkt.read1ByteTxRx(sid, ADDR_SERVO_STATUS)
    if r_st != bus.scs.COMM_SUCCESS:
        status = -1
        err_st = -1
    moving, r_mv, _e = bus.pkt.read1ByteTxRx(sid, ADDR_MOVING)
    if r_mv != bus.scs.COMM_SUCCESS:
        moving = -1
    torq, r_tq, _e = bus.pkt.read1ByteTxRx(sid, ADDR_TORQUE_ENABLE)
    if r_tq != bus.scs.COMM_SUCCESS:
        torq = -1
    mode, r_md, _e = bus.pkt.read1ByteTxRx(sid, ADDR_MODE)
    if r_md != bus.scs.COMM_SUCCESS:
        mode = -1
    vmax, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_MAX_VOLTAGE_LIMIT)
    vmin, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_MIN_VOLTAGE_LIMIT)
    tmax, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_MAX_TEMP_LIMIT)
    led_alarm, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_LED_ALARM)
    unload, _, _ = bus.pkt.read1ByteTxRx(sid, ADDR_UNLOAD_CONDITION)
    amin, _, _ = bus.pkt.read2ByteTxRx(sid, ADDR_MIN_ANGLE_LIMIT)
    amax, _, _ = bus.pkt.read2ByteTxRx(sid, ADDR_MAX_ANGLE_LIMIT)
    # Packet error byte from ping (protocol status flags, same bit meanings).
    _model, r_ping, err_ping = bus.pkt.ping(sid)
    if r_ping != bus.scs.COMM_SUCCESS:
        err_ping = -1

    faults = decode_status(status) if status >= 0 else []
    pkt_faults = decode_status(err_ping) if err_ping and err_ping > 0 else []
    fb.update({
        "status": status,
        "status_bits": faults,
        "packet_error": err_ping,
        "packet_error_bits": pkt_faults,
        "read_status_error": err_st,
        "alarm": bool(faults) or bool(pkt_faults),
        "moving": moving,
        "torque_enable": torq,
        "mode": mode,
        "volt_limit_max": vmax / 10.0,
        "volt_limit_min": vmin / 10.0,
        "temp_limit_max": tmax,
        "led_alarm": led_alarm,
        "unload_condition": unload,
        "angle_limit_min": amin,
        "angle_limit_max": amax,
    })
    return fb


def sample_status(bus: FeetechBus, sid: int, *, samples: int = 12,
                  interval: float = 0.15) -> list[int]:
    """Poll status reg repeatedly — catches intermittent voltage/overload blips."""
    out: list[int] = []
    for _ in range(samples):
        st, result, _err = bus.pkt.read1ByteTxRx(sid, ADDR_SERVO_STATUS)
        if result == bus.scs.COMM_SUCCESS:
            out.append(st)
        time.sleep(interval)
    return out


def format_health_lines(sid: int, h: dict, *, label: str = "") -> list[str]:
    """Human-readable debug lines for one servo (red LED = status bits)."""
    tag = f" {label}" if label else ""
    lines = [
        f"ID {sid}{tag}: count {h['count']} ({h['deg']:+.1f} deg)  "
        f"V={h['volt']:.1f}  T={h['temp_c']}°C  "
        f"load={h['load_pct']:.0f}%  I={h['current_a']:.2f} A",
    ]
    torq = h.get("torque_enable", -1)
    torq_s = {0: "OFF(limp)", 1: "ON"}.get(torq, f"raw={torq}")
    moving = h.get("moving", -1)
    mov_s = {0: "still", 1: "moving"}.get(moving, f"raw={moving}")
    mode = h.get("mode", -1)
    mode_s = {0: "position", 1: "speed", 2: "PWM", 3: "step"}.get(
        mode, f"raw={mode}")
    st = h.get("status", -1)
    lines.append(
        f"  torque={torq_s}  moving={mov_s}  mode={mode_s}  "
        f"status=0x{st:02X}" if st >= 0 else
        f"  torque={torq_s}  moving={mov_s}  mode={mode_s}  status=READ_FAIL"
    )
    pkt = h.get("packet_error", -1)
    if pkt is not None and pkt >= 0:
        lines.append(f"  packet error byte=0x{pkt:02X}"
                     + (f" ({', '.join(n for n, _ in h.get('packet_error_bits', []))})"
                        if h.get("packet_error_bits") else " (clean)"))
    lines.append(
        f"  limits: V {h.get('volt_limit_min', 0):.1f}"
        f"–{h.get('volt_limit_max', 0):.1f}  "
        f"Tmax={h.get('temp_limit_max', '?')}°C  "
        f"angle {h.get('angle_limit_min', '?')}–{h.get('angle_limit_max', '?')}  "
        f"LED_alarm=0x{h.get('led_alarm', 0):02X}  "
        f"unload=0x{h.get('unload_condition', 0):02X}"
    )
    faults = h.get("status_bits") or []
    if st == 0 and not h.get("packet_error_bits"):
        lines.append(
            "  faults: none in status/packet.  A slow blink can still be "
            "the normal power heartbeat — alarm blinks usually match "
            "status≠0.  If THIS servo's LED looks different from neighbors, "
            "run a status sample / force-heal."
        )
    elif faults:
        lines.append("  *** RED LED ALARM — active fault flags ***")
        for name, hint in faults:
            lines.append(f"    - {name}: {hint}")
    elif st > 0:
        lines.append(f"  faults: unknown bits set in status 0x{st:02X}")
    if h.get("packet_error_bits") and not faults:
        lines.append("  *** packet error flags (status reg was clear) ***")
        for name, hint in h["packet_error_bits"]:
            lines.append(f"    - {name}: {hint}")
    # Heuristics even when status is clear.
    vmax = h.get("volt_limit_max", 14.0)
    if h["volt"] >= 9.0 and vmax < 10.0:
        lines.append(
            f"  *** EEPROM max-voltage is only {vmax:.1f} V — too low for "
            f"a 12 V STS3215. Bus is fine ({h['volt']:.1f} V); restore "
            f"limits to {STS_DEFAULT_VMIN / 10:.0f}–{STS_DEFAULT_VMAX / 10:.0f} V."
        )
    if h["volt"] < 9.0:
        lines.append("  warn: bus < 9 V — USB alone won't run these; "
                     "check URT-2 screw-terminal 12 V")
    if h["temp_c"] >= 65:
        lines.append("  warn: hot — back off load / duty cycle")
    if h["load_pct"] >= 50:
        lines.append("  warn: high load — possible binding or stall")
    return lines


def ping_ok(bus: FeetechBus, sid: int, *, tries: int = 3) -> int:
    """Return how many of ``tries`` pings succeeded."""
    return sum(1 for _ in range(tries) if bus.ping(sid))


def _eeprom_write_log(sid: int, addr: int, value: int, note: str) -> None:
    """Append-only forensic log — how EEPROM got to 4–8 V, etc."""
    path = Path(__file__).resolve().parent / "eeprom_write_log.txt"
    ts = time.strftime("%Y-%m-%dT%H:%M:%S")
    line = f"{ts}  id={sid}  addr={addr}  value={value}  {note}\n"
    try:
        path.open("a").write(line)
    except Exception:
        pass
    print(f"  [eeprom] ID {sid} addr {addr} <- {value}  ({note})")


def restore_voltage_limits(bus: FeetechBus, sid: int,
                           *, vmin: int = STS_DEFAULT_VMIN,
                           vmax: int = STS_DEFAULT_VMAX,
                           require_solid_link: bool = True) -> dict | None:
    """Write STS3215 EEPROM min/max voltage (units 0.1 V). Torque off first.

    Refuses to unlock EEPROM if the link is flaky — a torn write can
    corrupt ID/baud and the servo "disappears" while still blinking.

    Note: we only ever write the 12 V profile (default 40/140 = 4–14 V).
    We never write max=80 (8 V); that value is Feetech's 7.4 V-sku
    factory default and means something else changed the EEPROM.
    """
    hits = ping_ok(bus, sid, tries=5)
    if require_solid_link and hits < 4:
        print(f"  REFUSING EEPROM write on ID {sid}: only {hits}/5 pings "
              f"OK. Power-cycle, plug it alone, then retry — writing on a "
              f"noisy link can brick ID/baud.")
        return None
    before = read_servo_health(bus, sid)
    print(f"  voltage limits before: "
          f"{before['volt_limit_min']:.1f}–{before['volt_limit_max']:.1f} V")
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.05)
    bus.pkt.unLockEprom(sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_MIN_VOLTAGE_LIMIT, int(vmin))
    _eeprom_write_log(sid, ADDR_MIN_VOLTAGE_LIMIT, int(vmin),
                      "restore_voltage_limits min")
    bus.pkt.write1ByteTxRx(sid, ADDR_MAX_VOLTAGE_LIMIT, int(vmax))
    _eeprom_write_log(sid, ADDR_MAX_VOLTAGE_LIMIT, int(vmax),
                      "restore_voltage_limits max (12V profile, not 8V)")
    bus.pkt.LockEprom(sid)
    time.sleep(0.15)
    if ping_ok(bus, sid, tries=3) == 0:
        print(f"  !!! ID {sid} went silent RIGHT AFTER EEPROM write. "
              f"Do NOT keep writing. Power-cycle, put it ALONE on the "
              f"URT-2, run baud/ID probe.")
        return None
    # Clear latched voltage flag with a gentle hold (SRAM only).
    try:
        fb = read_feedback_raw(bus, sid)
        bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)
        bus.pkt.WritePosEx(sid, fb["count"], 50, 5)
        time.sleep(0.15)
        bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    except Exception as exc:
        print(f"  (post-write clear move failed: {exc})")
    time.sleep(0.1)
    return read_servo_health(bus, sid)


def restore_bus_baud(bus: FeetechBus, sid: int,
                     *, host_baud: int,
                     target_baud: int = 1_000_000) -> bool:
    """Set servo EEPROM baud to ``target_baud``. Host must already match
    the servo's *current* baud (``host_baud``)."""
    if target_baud not in BAUD_TO_REG:
        print(f"  unsupported target baud {target_baud}")
        return False
    if not bus.port.setBaudRate(host_baud):
        return False
    time.sleep(0.05)
    if ping_ok(bus, sid, tries=3) < 2:
        print(f"  ID {sid} not solid at host baud {host_baud}")
        return False
    reg = BAUD_TO_REG[target_baud]
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    bus.pkt.unLockEprom(sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_BAUD_RATE, reg)
    bus.pkt.LockEprom(sid)
    time.sleep(0.1)
    # Servo now speaks target_baud; switch host and confirm.
    bus.port.setBaudRate(target_baud)
    time.sleep(0.05)
    ok = ping_ok(bus, sid, tries=3) >= 2
    print(f"  ID {sid} baud EEPROM -> {target_baud}: "
          f"{'OK' if ok else 'NO REPLY at new baud'}")
    return ok


def force_heal(bus: FeetechBus, sid: int) -> dict | None:
    """Restore 12 V-safe voltage limits + clear latched faults."""
    after = restore_voltage_limits(bus, sid)
    if after is None:
        return None
    return try_clear_alarm(bus, sid)


def print_servo_health(bus: FeetechBus, sid: int, *, label: str = "") -> dict:
    h = read_servo_health(bus, sid)
    for line in format_health_lines(sid, h, label=label):
        print(f"  {line}" if not line.startswith("ID") else f"  {line}")
    return h


def try_clear_alarm(bus: FeetechBus, sid: int) -> dict:
    """Limp, then hold-in-place position command (clears overload/current flags)."""
    before = read_servo_health(bus, sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.05)
    # Re-issue current pose as goal — Feetech clears overload on new pos cmd.
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)
    bus.pkt.WritePosEx(sid, before["count"], 50, 5)
    time.sleep(0.2)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.1)
    return read_servo_health(bus, sid)


def ensure_position_mode(bus: FeetechBus, sid: int) -> None:
    mode, result, _err = bus.pkt.read1ByteTxRx(sid, ADDR_MODE)
    if result != bus.scs.COMM_SUCCESS:
        print("  (mode register read failed; assuming position mode)")
        return
    if mode == 0:
        print("  mode: position/servo (0) -- OK")
        return
    print(f"  mode was {mode}; switching to position mode (0) ...")
    bus.pkt.unLockEprom(sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_MODE, 0)
    bus.pkt.LockEprom(sid)
    time.sleep(0.05)


@contextmanager
def keystroke_abort_watch():
    """During motion: any keystroke sets the returned checker True.

    Puts the TTY in cbreak so a single key (no Enter) aborts immediately.
    Restores terminal settings on exit and flushes leftover input.
    Non-TTY stdin → checker always False.
    """
    if not sys.stdin.isatty():
        yield lambda: False
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        termios.tcflush(fd, termios.TCIFLUSH)
        tty.setcbreak(fd)

        def key_hit() -> bool:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if not ready:
                return False
            while select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
            return True

        yield key_hit
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        termios.tcflush(fd, termios.TCIFLUSH)


def limp_now(bus: FeetechBus, sid: int) -> None:
    """Best-effort torque OFF (ignore bus errors during emergency stop)."""
    try:
        bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    except Exception:
        pass


def move_and_wait(bus: FeetechBus, sid: int, target: int,
                  speed: int, acc: int, timeout: float = 5.0,
                  *, abort_check=None) -> str:
    """Command a raw-count move and poll until the servo settles.

    Returns ``ok``, ``timeout``, or ``aborted`` (keystroke / Ctrl-C).
    If ``abort_check`` is None, installs a local keystroke watch for this
    move only.

    Note: Feetech ``speed=0`` means MAX — coerced to a gentle hold speed.
    """
    from feetech_bus import HOLD_SPEED, normalize_acc, normalize_speed
    speed = normalize_speed(speed)
    acc = normalize_acc(acc)
    if speed == 0:  # belt-and-suspenders if allow_max ever leaks in
        speed = HOLD_SPEED

    def _run(check) -> str:
        bus.pkt.WritePosEx(sid, target, speed, acc)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if check():
                limp_now(bus, sid)
                print("    STOP — keystroke abort.  Torque OFF.")
                return "aborted"
            time.sleep(0.05)
            fb = read_feedback_raw(bus, sid)
            print(f"    pos {fb['deg']:+7.1f} deg   load {fb['load_pct']:5.1f}%"
                  f"   {fb['current_a']:.2f} A   {fb['volt']:.1f} V")
            if abs(fb["count"] - target) < 8:      # ~0.7 deg
                return "ok"
        print("    (did not settle within timeout -- horn blocked or "
              "supply sagging?)")
        return "timeout"

    try:
        if abort_check is not None:
            return _run(abort_check)
        with keystroke_abort_watch() as check:
            return _run(check)
    except KeyboardInterrupt:
        limp_now(bus, sid)
        print("    STOP — Ctrl-C.  Torque OFF.")
        return "aborted"
