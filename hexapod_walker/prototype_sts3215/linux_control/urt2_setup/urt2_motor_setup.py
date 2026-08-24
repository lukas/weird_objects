#!/usr/bin/env python3
"""Name / test / rename STS3215 servos via a URT-2 — any order.

Plug in a factory (or unnamed) servo, watch it wiggle, then say what it
should be (``L2 knee``, …).  Or pick a motor already on the bus to
**test** (rotate sweep + diagnostics), **rename**, or **delete** (free a
slot to replace a dead/wrong motor).  Already-named motors may stay
daisy-chained; ID 1 is never a robot ID.

Robot IDs = ``joint + 2`` → **2..19** (see ``feetech_bus.py``).

Examples
--------
    uv run python urt2_motor_setup.py              # menu: n/t/r/x/d/i/z/g/l/f/p/s/?/q
    uv run python urt2_motor_setup.py --debug     # dump fault status for all on bus
    uv run python urt2_motor_setup.py --once      # one action then exit
    uv run python urt2_motor_setup.py --leg 2     # old guided per-leg flow
    uv run python urt2_motor_setup.py --status
    uv run python urt2_motor_setup.py --reset

Install once:  uv pip install ftservo-python-sdk
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from feetech_bus import (  # noqa: E402
    ADDR_TORQUE_ENABLE, BAUD_DEFAULT, COUNTS_PER_DEG, FACTORY_SERVO_ID,
    FeetechBus, SERVO_ID_OFFSET, STS_CENTRE_COUNT, STS_COUNTS_PER_REV,
    joint_to_servo_id,
)
from urt2_bench import (  # noqa: E402
    default_port, ensure_position_mode, force_heal, keystroke_abort_watch,
    limp_now, print_identity_report, print_servo_health, read_feedback_raw,
    read_servo_health, restore_bus_baud, restore_voltage_limits,
    sample_status, try_clear_alarm,
)

# Identify nudge: tiny + slow — motor may already be mounted near a stop.
# Optional post-name test (ask) can sweep farther once the user is ready.
SETUP_SPEED = 60          # steps/s  (DEFAULT_SPEED is 1500 — far too fast)
SETUP_ACC = 4             # *100 steps/s^2
WIGGLE_AMP_DEG = 3.0      # ±3° from current pose — just enough to see it
SETUP_TORQUE_LIMIT = 150  # SRAM addr 48: 15% stall (0..1000)
SETUP_LOAD_ABORT_PCT = 18.0
SETUP_CURRENT_ABORT_A = 0.25

# Post-naming optional rotate-to-test (still load-watched).
TEST_SPEED = 180
TEST_ACC = 8
TEST_AMP_DEFAULT_DEG = 15.0
TEST_AMP_MAX_DEG = 45.0
TEST_TORQUE_LIMIT = 350   # 35%
TEST_LOAD_ABORT_PCT = 30.0
TEST_CURRENT_ABORT_A = 0.6

MIN_BUS_VOLT = 9.0
ADDR_TORQUE_LIMIT = 48    # STS3215 SRAM; does not touch EEPROM max-torque
# Torque-enable magic: 128 = "define current pose as centre (2048)", no move.
TORQUE_CALIBRATE_MIDDLE = 128

AXIS_NAMES = ("yaw", "hip", "knee")
AXIS_ORDER_IN_LEG = ("yaw", "hip", "knee")
AXIS_ALIASES = {
    "yaw": "yaw", "y": "yaw", "0": "yaw",
    "hip": "hip", "h": "hip", "pitch": "hip", "p": "hip", "1": "hip",
    "knee": "knee", "k": "knee", "2": "knee",
}

REGISTRY_PATH = Path(__file__).resolve().parent / "motor_setup_registry.json"


def joint_of(leg: int, axis: str) -> int:
    return leg * 3 + AXIS_NAMES.index(axis)


def target_for(leg: int, axis: str) -> dict:
    j = joint_of(leg, axis)
    sid = joint_to_servo_id(j)
    return {
        "id": sid,
        "name": f"L{leg} {axis}",
        "leg": leg,
        "axis": axis,
        "joint": j,
    }


def leg_targets(leg: int) -> list[dict]:
    return [target_for(leg, axis) for axis in AXIS_ORDER_IN_LEG]


def all_targets() -> list[dict]:
    out = []
    for leg in range(6):
        for axis in AXIS_NAMES:
            out.append(target_for(leg, axis))
    return out


def remaining_targets(reg: dict) -> list[dict]:
    done = {int(k) for k in reg.get("servos", {})}
    return [t for t in all_targets() if t["id"] not in done]


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------


def load_registry() -> dict:
    if REGISTRY_PATH.exists():
        try:
            return json.loads(REGISTRY_PATH.read_text())
        except json.JSONDecodeError:
            pass
    return {"created": datetime.now(timezone.utc).isoformat(),
            "servos": {}, "scheme": "ids_2_to_19"}


def save_registry(reg: dict) -> None:
    reg["updated"] = datetime.now(timezone.utc).isoformat()
    reg["scheme"] = "ids_2_to_19"
    REGISTRY_PATH.write_text(json.dumps(reg, indent=2) + "\n")


def print_status(reg: dict) -> None:
    done = {int(k) for k in reg.get("servos", {})}
    print(f"Registry: {REGISTRY_PATH}")
    print(f"  configured: {len(done)} / 18   "
          f"(robot IDs 2..19; factory ID {FACTORY_SERVO_ID} never assigned)")
    for leg in range(6):
        cells = []
        for axis in AXIS_NAMES:
            t = target_for(leg, axis)
            mark = "OK" if t["id"] in done else "--"
            cells.append(f"{axis}:{t['id']}[{mark}]")
        print(f"  L{leg}  " + "  ".join(cells))
    left = remaining_targets(reg)
    if left:
        print("  still open: " + ", ".join(t["name"] for t in left))


def _label_for_id(sid: int, reg: dict) -> str:
    entry = reg.get("servos", {}).get(str(sid))
    if entry:
        return f"{entry['name']:<10}  joint {entry['joint']}"
    if sid == FACTORY_SERVO_ID:
        return "factory / unnamed"
    # Known robot ID that isn't in the registry yet.
    j = sid - SERVO_ID_OFFSET
    if 0 <= j <= 17:
        t = target_for(j // 3, AXIS_NAMES[j % 3])
        return f"{t['name']:<10}  (not in registry)"
    return "unknown ID"


def print_connected(bus: FeetechBus | None, reg: dict,
                    live: set[int] | None = None,
                    *, verbose: bool = False) -> set[int]:
    """Print named + live motors with connected/offline status."""
    if live is None:
        if bus is None:
            raise ValueError("bus or live set required")
        live = set(bus.scan())
    named = {int(k) for k in reg.get("servos", {})}
    # Inventory: every named ID, plus any live IDs not in the registry.
    inventory = sorted(named | live)
    n_conn = sum(1 for sid in inventory if sid in live)
    n_off = sum(1 for sid in inventory if sid not in live)
    print(f"Motors: {n_conn} connected, {n_off} offline")
    if not inventory:
        print("  (none named yet, none answering — plug a servo + 12 V)")
        return set()

    alarms: list[int] = []
    for sid in inventory:
        on = sid in live
        status = "connected" if on else "offline"
        entry = reg.get("servos", {}).get(str(sid))
        if entry:
            label = f"{entry['name']:<10}  joint {entry['joint']}"
            tag = ""
        elif sid == FACTORY_SERVO_ID:
            label = "factory / unnamed"
            tag = "  [UNNAMED]"
        else:
            label = _label_for_id(sid, reg)
            tag = "  [UNNAMED]" if on else ""
        alarm_note = ""
        if on and bus is not None:
            try:
                h = read_servo_health(bus, sid)
                if h.get("alarm"):
                    alarms.append(sid)
                    names = ",".join(n for n, _ in h["status_bits"])
                    alarm_note = f"  *** ALARM:{names} ***"
                elif verbose:
                    alarm_note = (f"  V={h['volt']:.1f} T={h['temp_c']}°C "
                                  f"load={h['load_pct']:.0f}%")
            except Exception as exc:
                alarm_note = f"  (health read failed: {exc})"
        print(f"  ID {sid:2d}  {label:<22}  {status}{tag}{alarm_note}")

    if FACTORY_SERVO_ID in live:
        print(f"  note: factory ID {FACTORY_SERVO_ID} is on the bus (use n to name)")
    if alarms:
        print(f"  !!! {len(alarms)} servo(s) with RED LED / status fault: "
              f"{alarms}")
        print("      Use action 'd' (debug) for full decode + clear attempt.")
    return set(live)


def first_incomplete_leg(reg: dict) -> int | None:
    done = {int(k) for k in reg.get("servos", {})}
    for leg in range(6):
        ids = {target_for(leg, a)["id"] for a in AXIS_NAMES}
        if not ids.issubset(done):
            return leg
    return None


# ---------------------------------------------------------------------------
# Interactive helpers
# ---------------------------------------------------------------------------


def ask(prompt: str, default: str | None = None) -> str:
    suffix = f" [{default}]" if default is not None else ""
    try:
        ans = input(f"{prompt}{suffix}: ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\nAborted.")
        raise SystemExit(1)
    if not ans and default is not None:
        return default
    return ans


def ask_yn(prompt: str, default: str = "y") -> bool:
    while True:
        ans = ask(prompt + " [y/n]", default).lower()
        if ans in ("y", "yes"):
            return True
        if ans in ("n", "no"):
            return False
        print("  please answer y or n")


def limp_ids(bus: FeetechBus, ids) -> None:
    for sid in ids:
        try:
            bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
        except Exception:
            pass


def wait_for_unnamed_servo(bus: FeetechBus, already: set[int], reg: dict,
                           *, prompt_hint: str | None = None) -> int:
    """Poll until exactly one not-yet-named ID is on the bus."""
    print()
    print("  Waiting for one unnamed servo on the bus ...")
    if prompt_hint:
        print(f"  {prompt_hint}")
    print(f"  (already named: {sorted(already) or 'none'}; "
          "scan every 1 s; Ctrl-C to abort)")
    last: set[int] | None = None
    while True:
        found = set(bus.scan())
        if found != last:
            print_connected(bus, reg, live=found)
            last = found
        unnamed = sorted(found - already)
        if len(unnamed) == 1:
            return unnamed[0]
        if len(unnamed) > 1:
            print(f"  {len(unnamed)} unnamed IDs ({unnamed}) -- leave only "
                  "the NEW servo unnamed (named ones may stay)")
        time.sleep(1.0)


def _set_torque_limit(bus: FeetechBus, sid: int, limit: int) -> None:
    """SRAM torque limit (0..1000 = 0..100%).  Volatile — no EEPROM write."""
    try:
        bus.pkt.write2ByteTxRx(sid, ADDR_TORQUE_LIMIT, int(limit))
    except Exception as exc:
        print(f"  (could not set torque limit: {exc})")


def _count_to_deg(count: int) -> float:
    return (count - STS_CENTRE_COUNT) / COUNTS_PER_DEG


def _clamp_count(count: int) -> int:
    return max(0, min(STS_COUNTS_PER_REV - 1, int(count)))


def print_servo_diagnostics(bus: FeetechBus, sid: int,
                            *, title: str = "diagnostics") -> dict:
    """Print live feedback + fault flags + room left to each encoder end."""
    print(f"  [{title}]")
    h = print_servo_health(bus, sid)
    room_plus = _count_to_deg((STS_COUNTS_PER_REV - 1) - h["count"])
    room_minus = _count_to_deg(h["count"] - 0)
    print(f"    room toward +end: {room_plus:.0f}°   "
          f"toward -end: {room_minus:.0f}°   "
          f"(encoder span ±180° around centre {STS_CENTRE_COUNT})")
    if abs(h["deg"]) > 150:
        print("    note: near an encoder end — a ±sweep may not fit; "
              "recenter or use a smaller amp.")
    h["room_plus_deg"] = room_plus
    h["room_minus_deg"] = room_minus
    return h


# Common STS baud rates (register 6 values map to these).
BAUD_PROBE_LIST = (
    1_000_000,
    500_000,
    250_000,
    128_000,
    115_200,
    57_600,
    38_400,
)


def probe_missing_motor(bus: FeetechBus, sid: int, *,
                        name: str = "",
                        working_baud: int = BAUD_DEFAULT) -> int | None:
    """Try to find a registry motor that isn't answering at the working baud.

    Returns the ID it answered as (may differ if it was re-IDed), or None.
    Restores ``working_baud`` before returning.
    """
    label = name or f"ID {sid}"
    print()
    print(f"  --- probe missing {label} (expected ID {sid}) ---")
    print("  LED blinking + no bus reply ⇒ POWER is present, DATA is not.")
    print("  As-built harness (see firmware/WIRING.md §6.2):")
    print("    DATA enters each leg at the YAW, then yaw→hip→knee;")
    print("    POWER injects at each HIP and tees to yaw + knee.")
    print("  So a blinking offline yaw usually means: hip power tee is OK,")
    print("  but the DATA feed into that yaw (or the yaw UART) is dead.")
    print("  Check chassis→yaw data seating; keep thick hip power leads")
    print("  off the data pair (parallel fat V+ can corrupt 1 Mbps TTL).")
    print("  Also power-cycle 12 V in case the servo MCU hung.")

    # Retries at current baud.
    hits = 0
    try:
        for _ in range(8):
            if bus.ping(sid):
                hits += 1
    except Exception as exc:
        print(f"  ping failed ({exc}) — replug USB and retry")
        return None
    if hits:
        print(f"  ID {sid} answered {hits}/8 pings at {working_baud} baud — "
              f"flaky link. Check the daisy-chain DATA crimp near this servo.")
        return sid

    print(f"  ID {sid}: 0/8 pings at {working_baud} baud.")
    print("  Scanning IDs 1..30 at current baud for strangers ...")
    found = bus.scan(range(1, 31))
    print(f"    live now: {found or '(none)'}")
    if found and sid not in found:
        print(f"    (bus is alive — {found} — but expected ID {sid} is absent)")

    print("  Trying other bauds (looking for any new ID, esp. "
          f"expected {sid} or factory {FACTORY_SERVO_ID}) ...")
    seen_elsewhere: dict[int, list[int]] = {}
    port_died = False
    try:
        for baud in BAUD_PROBE_LIST:
            if baud == working_baud:
                continue
            try:
                if not bus.port.setBaudRate(baud):
                    print(f"    baud {baud}: port rejected")
                    continue
                time.sleep(0.08)
                hits_at = [i for i in range(1, 31) if bus.ping(i)]
            except Exception as exc:
                print(f"    baud {baud}: port error ({exc}) — stopping sweep")
                port_died = True
                break
            if hits_at:
                seen_elsewhere[baud] = hits_at
                print(f"    baud {baud}: {hits_at}")
            else:
                print(f"    baud {baud}: (none)")
    finally:
        try:
            bus.port.setBaudRate(working_baud)
            time.sleep(0.08)
        except Exception as exc:
            print(f"  WARNING: could not restore baud {working_baud}: {exc}")
            print("  Unplug/replug USB and re-run — do not continue on a dead port.")
            port_died = True

    if port_died:
        print("  Serial port glitched mid-probe (unplug, multi-access, or")
        print("  adapter reset). Re-plug URT-2 USB and run --debug again.")
        return None

    if sid in found:
        print(f"  ID {sid} showed up on a rescan — try again.")
        return sid
    for baud, ids in seen_elsewhere.items():
        if sid in ids:
            print(f"  *** Found expected ID {sid} at baud {baud} "
                  f"(not {working_baud}).")
            print("  EEPROM baud was likely corrupted by a write on a "
                  "flaky bus (e.g. voltage-limit 'fix').")
            if ask_yn(f"  Rewrite ID {sid} baud EEPROM back to "
                      f"{working_baud}?", "y"):
                if restore_bus_baud(bus, sid, host_baud=baud,
                                    target_baud=working_baud):
                    bus.port.setBaudRate(working_baud)
                    return sid
            bus.port.setBaudRate(working_baud)
            return sid
        if FACTORY_SERVO_ID in ids:
            print(f"  Factory ID {FACTORY_SERVO_ID} lives at baud {baud} — "
                  f"possible reset of {label}.")
            if ask_yn(f"  Set that servo's baud to {working_baud} and "
                      f"you'll re-ID it as {sid} next?", "y"):
                restore_bus_baud(bus, FACTORY_SERVO_ID, host_baud=baud,
                                 target_baud=working_baud)
                bus.port.setBaudRate(working_baud)
                return FACTORY_SERVO_ID
    bus.port.setBaudRate(working_baud)
    time.sleep(0.05)
    print("  Not found on any baud 1..30.")
    print("  If it vanished right after a debug 'fix'/EEPROM write:")
    print("    1. Power-cycle 12 V.")
    print("    2. Unplug EVERYTHING else — only this servo on the URT-2.")
    print("    3. Re-run --debug and probe again (baud may have changed).")
    print("    4. If it shows up as ID 1, rename it back to this joint.")
    print("  Also check chassis→yaw data vs fat hip power (WIRING.md §6.2.1).")
    print()
    print("  LED blinks the instant USB is plugged (even before a scan)?")
    print("  That's usually a LOCAL fault loop, not 'I see bus traffic':")
    print("    A) 12 V ON, DATA unplugged from this servo only — still blink?")
    print("       yes → fault/LED alarm on the servo itself (or bad V+).")
    print("       no  → blink was from host traffic / data path.")
    print("    B) 12 V OFF, USB only — measure V+ at the servo plug.")
    print("       If V+ is ~0 but LED still blinks hard, treat as hung/faulty")
    print("       MCU; leave it off the chain until it answers alone @ some baud.")
    print("    C) Alone on URT-2 + 12 V, don't run any script for 10 s.")
    print("       Steady/periodic blink with no host → alarm/hardware.")
    print("       Quiet until you run scan → normal RX blink.")
    print()
    print("  Whole leg blinking together (e.g. all L0 yaw+hip+knee)?")
    print("  Treat it as the LEG BRANCH, not three separate failures:")
    print("    - DATA enters at yaw → hip → knee (one stub).")
    print("    - POWER injects at hip → tees to yaw + knee.")
    print("    1) Unplug leg DATA at the yaw — all three go quiet?")
    print("       yes → noise/traffic/short on that data stub.")
    print("    2) Unplug leg POWER at the hip — LEDs die?")
    print("       yes → shared V+ path (expected); check branch fuse/polarity.")
    print("    3) Leave other legs on; scan. If only IDs for that leg are")
    print("       missing, fix that leg alone on the URT-2 before rejoining.")
    print()
    print("  LED went SOLID red (not blinking) after isolating?")
    print("  That is usually a LATCHED protection/fault, not RX traffic.")
    print("    1) Kill 12 V for 10+ s (USB unplug too), then power 12 V only.")
    print("    2) One servo alone on URT-2 → run --debug / scan.")
    print("    3) If it answers: clear alarm (SRAM) before any EEPROM write.")
    print("    4) If solid red + never pings after power-cycle: likely hung")
    print("       or damaged — try other bauds, then swap with a known-good.")
    return None


def bringup_after_new_leg(bus: FeetechBus, reg: dict, *,
                          baud: int = BAUD_DEFAULT) -> None:
    """Guided checks when the bus broke after plugging another leg."""
    print()
    print("=" * 60)
    print("  BRING-UP: worked until a new leg was plugged in")
    print("=" * 60)
    print("  At 1 Mbps, the 4th leg is a common breakpoint: more cable")
    print("  capacitance, another data stub, or a bad leg-to-leg jumper")
    print("  can take down nodes that used to work (often an early yaw).")
    print()
    print("  Do these in order (Enter after each physical step):")
    print()

    ask("  1) UNPLUG the newest leg's DATA jumper only "
        "(leave its power if you want). Enter when unplugged", "")
    live = sorted(bus.scan())
    print(f"     scan without that data stub: {live or '(none)'}")
    named = {int(k) for k in reg.get("servos", {})}
    offline = sorted(named - set(live))
    if offline:
        print(f"     still missing: "
              f"{', '.join(reg['servos'][str(s)]['name'] for s in offline)}")
    else:
        print("     all named IDs answered — newest leg's DATA path is the "
              "suspect.")

    ask("  2) Also unplug that leg's POWER branch. Enter when done", "")
    live2 = sorted(bus.scan())
    print(f"     scan power+data off: {live2 or '(none)'}")
    if set(live2) > set(live):
        print("     more motors appeared with power removed — check for a "
              "V+ short on that leg's data jumper (leg-to-leg must be "
              "signal+GND only, no V+).")

    print()
    print("  3) Inspect the newest leg-to-leg DATA cable:")
    print("       - Only SIGNAL + GND (middle + ground) — V+ wire cut/omitted")
    print("       - No swapped pins / partial insertion at either yaw")
    print("       - Not zip-tied along a fat power lead")
    ask("     Enter when checked", "")

    print()
    print("  4) Plug ONLY the newest leg into the adapter (alone).")
    ask("     Enter when that leg alone is on the bus", "")
    alone = sorted(bus.scan())
    print(f"     alone scan: {alone or '(none)'}")
    if not alone:
        print("     That leg alone is deaf/shorted — fix it before "
              "rejoining the chain.")
    else:
        print("     Leg alone talks — rejoining the chain is a loading / "
              "jumper / layout issue, not dead servos.")

    print()
    print("  5) Rejoin: 3 known-good legs first, confirm scan, THEN add")
    print("     the 4th. If it fails only when the 4th data stub is on:")
    print("       - shorten / separate that stub from power")
    print(f"       - try temporarily:  --baud 500000  (now {baud})")
    print("       - add legs one at a time; don't hot-plug data under load")
    ask("     Enter to rescan as currently wired", "")
    final = sorted(bus.scan())
    print(f"     current scan: {final or '(none)'}")
    print_connected(bus, reg, live=set(final))


def debug_bus(bus: FeetechBus, reg: dict, *,
              baud: int = BAUD_DEFAULT) -> None:
    """Full health dump for every connected servo (live only)."""
    print()
    print("=" * 60)
    print("  DEBUG / FAULT DECODE")
    print("  Red LED blink = status reg 65 fault bit(s)  — when it answers")
    print("  LED + missing from scan = powered but deaf → menu action p")
    print("=" * 60)
    live = set(bus.scan())
    print_connected(bus, reg, live=live)

    if ask_yn("Did this break right after plugging in another leg?", "n"):
        if ask_yn("Run the 'new leg broke the bus' bring-up checklist?",
                  "y"):
            bringup_after_new_leg(bus, reg, baud=baud)
            live = set(bus.scan())

    if not live:
        print("  No servos answering — fix wiring/power before fault decode.")
        return

    alarmed: list[int] = []
    for sid in sorted(live):
        label = _label_for_id(sid, reg)
        print()
        h = print_servo_health(bus, sid, label=label)
        if h.get("alarm"):
            alarmed.append(sid)
    print()
    # Catch the common "max voltage EEPROM stuck at 8 V" misconfig.
    bad_vmax = []
    for sid in live:
        try:
            h = read_servo_health(bus, sid)
        except Exception:
            continue
        if h.get("volt_limit_max", 14.0) < 10.0 and h.get("volt", 0) >= 9.0:
            bad_vmax.append(sid)
    if bad_vmax:
        print(f"  IDs with EEPROM max-voltage too low for 12 V bus: {bad_vmax}")
        print("  (healthy STS3215 default is ~4–14 V; 4–8 V makes 12 V look "
              "like an over-voltage fault)")
        print("  Prefer CLEAR-ONLY first — EEPROM writes on a flaky bus can")
        print("  corrupt baud/ID and the servo vanishes while still blinking.")
        if ask_yn("Clear alarms only (no EEPROM write)?", "y"):
            for sid in bad_vmax:
                print(f"  clearing ID {sid} (SRAM only) ...")
                after = try_clear_alarm(bus, sid)
                still = (",".join(n for n, _ in after.get("status_bits", []))
                         or "none")
                print(f"    after: status=0x{after['status']:02X}  "
                      f"faults={still}  "
                      f"limits still V {after['volt_limit_min']:.1f}"
                      f"–{after['volt_limit_max']:.1f}")
            limp_ids(bus, bus.scan())
        if ask_yn("Also rewrite EEPROM voltage limits to 4.0–14.0 V? "
                  "(only if pings are solid)", "n"):
            for sid in bad_vmax:
                print(f"  fixing ID {sid} EEPROM ...")
                after = restore_voltage_limits(bus, sid)
                if after is None:
                    print(f"    skipped/failed — if ID {sid} is now missing, "
                          f"use menu action p to probe.")
                    continue
                still = (",".join(n for n, _ in after.get("status_bits", []))
                         or "none")
                print(f"    now limits V {after['volt_limit_min']:.1f}"
                      f"–{after['volt_limit_max']:.1f}  "
                      f"status=0x{after['status']:02X}  faults={still}")
            limp_ids(bus, bus.scan())
            gone = [s for s in bad_vmax if not bus.ping(s)]
            if gone:
                print(f"  !!! went silent after EEPROM write: {gone}")
                print("  Use menu action  p  to probe them (retries + baud "
                      "sweep).")

    if alarmed:
        print(f"  Alarmed IDs: {alarmed}")
        if ask_yn("Try to clear alarms (limp + re-hold current pose)?", "y"):
            for sid in alarmed:
                print(f"  clearing ID {sid} ...")
                after = try_clear_alarm(bus, sid)
                still = (",".join(n for n, _ in after.get("status_bits", []))
                         or "none")
                print(f"    after: status=0x{after['status']:02X}  "
                      f"faults={still}")
            limp_ids(bus, bus.scan())
    else:
        print("  No status faults reported right now.")
        print("  Tip: STS LEDs often pulse as a normal power heartbeat when "
              "status=0.  Compare blink rate to a known-good neighbor.")

    needs_attention = bool(alarmed or bad_vmax)
    if needs_attention:
        prompt = "Deep-dive / force-heal a live flagged servo?"
        default = "y"
    else:
        print("  Live bus reads look healthy — no servo is flagged as wrong.")
        prompt = "Deep-dive a live servo anyway (optional)?"
        default = "n"

    while ask_yn(prompt, default):
        sid = ask_existing_servo(bus, reg)
        if sid is None:
            break
        inspect_and_heal_servo(bus, reg, sid)
        prompt = "Deep-dive another live servo?"
        default = "n"

    print()
    print("  Debug session finished (normal exit — nothing crashed).")
    print_connected(bus, reg)


def inspect_and_heal_servo(bus: FeetechBus, reg: dict, sid: int) -> None:
    """Deep dive on one ID: sample intermittent faults, then optional heal."""
    label = _label_for_id(sid, reg)
    print()
    print(f"  --- deep dive ID {sid} ({label}) ---")
    print_servo_health(bus, sid, label=label)
    print("  Sampling status 12× over ~2 s (catches blips) ...")
    samples = sample_status(bus, sid)
    if not samples:
        print("  (no successful status reads)")
    else:
        nonzero = [s for s in samples if s]
        uniq = sorted(set(samples))
        print(f"    samples: {[f'0x{s:02X}' for s in samples]}")
        print(f"    unique:  {[f'0x{s:02X}' for s in uniq]}  "
              f"({len(nonzero)}/{len(samples)} nonzero)")
        if nonzero:
            from urt2_bench import decode_status
            bits = decode_status(nonzero[-1])
            names = ", ".join(n for n, _ in bits) or f"0x{nonzero[-1]:02X}"
            print(f"    intermittent fault seen: {names}")
        else:
            print("    status stayed 0x00 the whole time — LED blink with "
                  "clean status is usually the normal heartbeat, not an alarm.")
            print("    Power-cycle 12 V if you still want to reset the LED "
                  "state machine.")
    h = read_servo_health(bus, sid)
    already_ok = (h.get("volt_limit_max", 0) >= 12.0
                  and h.get("status", 1) == 0)
    if already_ok:
        print("  Limits already 12 V-safe and status clean — force-heal "
              "would only rewrite the same EEPROM values.")
    if ask_yn(f"Force-heal ID {sid} (restore 4–14 V limits + clear)?", "n"):
        print("  Note: refuses EEPROM write unless pings are solid; a torn")
        print("  write can make this servo vanish from the bus.")
        after = force_heal(bus, sid)
        if after is None:
            print("  heal aborted or link lost — try alone on URT-2, then "
                  "menu action p.")
            return
        still = (",".join(n for n, _ in after.get("status_bits", []))
                 or "none")
        print(f"    after heal: V limits {after['volt_limit_min']:.1f}"
              f"–{after['volt_limit_max']:.1f}  "
              f"status=0x{after['status']:02X}  faults={still}")
        limp_ids(bus, [sid])
        if not bus.ping(sid):
            print("  !!! silent after heal — use menu action p (baud sweep).")
            return
        print("  Heal OK.  If LED still blinks with status 0x00, that is the "
              "normal heartbeat — not a fault.")
    print("  (back to debug menu)")


def _safe_nudge(bus: FeetechBus, sid: int, target: int, *,
                speed: int, acc: int,
                load_abort: float, current_abort: float,
                timeout: float = 6.0,
                abort_check=None) -> tuple[str, dict]:
    """Creep toward ``target``; abort + limp on load/current/keystroke.

    Returns ``(status, last_feedback)`` where status is
    ``ok`` / ``blocked`` / ``timeout`` / ``aborted``.
    """
    check = abort_check or (lambda: False)
    bus.pkt.WritePosEx(sid, target, speed, acc)
    t0 = time.time()
    fb = read_feedback_raw(bus, sid)
    try:
        while time.time() - t0 < timeout:
            if check():
                limp_now(bus, sid)
                print("    STOP — keystroke abort.  Torque OFF.")
                return "aborted", fb
            time.sleep(0.05)
            fb = read_feedback_raw(bus, sid)
            if check():
                limp_now(bus, sid)
                print("    STOP — keystroke abort.  Torque OFF.")
                return "aborted", fb
            if (fb["load_pct"] >= load_abort
                    or fb["current_a"] >= current_abort):
                limp_now(bus, sid)
                print(f"    STOP — hit resistance "
                      f"(load {fb['load_pct']:.0f}%, {fb['current_a']:.2f} A). "
                      f"Torque OFF.")
                return "blocked", fb
            if abs(fb["count"] - target) < 12:   # ~1 deg
                return "ok", fb
        return "timeout", fb
    except KeyboardInterrupt:
        limp_now(bus, sid)
        print("    STOP — Ctrl-C.  Torque OFF.")
        return "aborted", fb


def _sweep_from_here(bus: FeetechBus, sid: int, amp_deg: float, *,
                     speed: int, acc: int, torque_limit: int,
                     load_abort: float, current_abort: float) -> str:
    """±amp around current pose, then back.

    Returns ``ok``, ``blocked``, ``range``, ``timeout``, or ``aborted``.
    Any keystroke (or Ctrl-C) immediately limps and stops the sweep.
    """
    fb = print_servo_diagnostics(bus, sid, title="before sweep")
    home = fb["count"]
    delta = int(round(amp_deg * COUNTS_PER_DEG))
    goals = [
        (f"+{amp_deg:.0f}", home + delta),
        (f"-{amp_deg:.0f}", home - delta),
        ("home", home),
    ]

    # Warn / clamp if a goal falls outside the 0..4095 encoder window.
    clipped = False
    planned = []
    for label, raw_t in goals:
        t = _clamp_count(raw_t)
        if t != raw_t:
            clipped = True
            print(f"  {label}: requested {_count_to_deg(raw_t):+.1f} deg "
                  f"(count {raw_t}) is past encoder end — "
                  f"clamping to {_count_to_deg(t):+.1f} deg (count {t})")
        else:
            print(f"  {label}: target {_count_to_deg(t):+.1f} deg "
                  f"(count {t})")
        planned.append((label, t, raw_t))

    if clipped:
        print("  Motor is likely fine — not enough encoder room for the "
              "full ±sweep from this pose.")

    _set_torque_limit(bus, sid, torque_limit)
    worst = "ok"
    try:
        with keystroke_abort_watch() as abort_check:
            print("  Press ANY KEY to abort immediately (torque OFF).")
            bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)
            for label, target, raw_t in planned:
                if abort_check():
                    limp_now(bus, sid)
                    print("    STOP — keystroke abort.  Torque OFF.")
                    return "aborted"
                # Skip a no-op move (already at clamp / home).
                if abs(read_feedback_raw(bus, sid)["count"] - target) < 12:
                    print(f"  -> {label} deg  (already there, skip)")
                    continue
                print(f"  -> {label} deg")
                status, fb = _safe_nudge(
                    bus, sid, target,
                    speed=speed, acc=acc,
                    load_abort=load_abort, current_abort=current_abort,
                    abort_check=abort_check,
                )
                err = fb["count"] - target
                print(f"    pos {fb['deg']:+.1f} deg  count {fb['count']}  "
                      f"err {err:+d} ct  load {fb['load_pct']:.0f}%  "
                      f"{fb['current_a']:.2f} A  [{status}]")
                if status == "aborted":
                    return "aborted"
                if status == "blocked":
                    return "blocked"
                if status == "timeout":
                    if raw_t != target:
                        worst = "range" if worst == "ok" else worst
                        print("    (timeout at encoder end — not necessarily "
                              "a bad motor)")
                    else:
                        if worst == "ok":
                            worst = "timeout"
                        print("    (didn't settle — path blocked, too slow, "
                              "or supply sagging)")
    except KeyboardInterrupt:
        limp_now(bus, sid)
        print("    STOP — Ctrl-C.  Torque OFF.")
        return "aborted"
    finally:
        _set_torque_limit(bus, sid, 1000)
        try:
            print_servo_diagnostics(bus, sid, title="after sweep")
        except Exception:
            pass
    return worst


def gentle_wiggle(bus: FeetechBus, sid: int) -> bool:
    """Tiny identify nudge from the *current* pose.  Never seeks centre."""
    fb = read_feedback_raw(bus, sid)
    print(f"  starting at {fb['deg']:+.1f} deg — tiny ±{WIGGLE_AMP_DEG:.0f}° "
          f"nudge (speed {SETUP_SPEED}, acc {SETUP_ACC}, "
          f"torque ≤{SETUP_TORQUE_LIMIT / 10:.0f}%)")
    print("  Only THIS servo should move.  KEEP CLEAR of its horn.")
    print(f"  Auto-stops if load ≥{SETUP_LOAD_ABORT_PCT:.0f}% or "
          f"current ≥{SETUP_CURRENT_ABORT_A:.2f} A.  Any key aborts.")
    status = _sweep_from_here(
        bus, sid, WIGGLE_AMP_DEG,
        speed=SETUP_SPEED, acc=SETUP_ACC,
        torque_limit=SETUP_TORQUE_LIMIT,
        load_abort=SETUP_LOAD_ABORT_PCT,
        current_abort=SETUP_CURRENT_ABORT_A,
    )
    if status == "aborted":
        print("  Identify nudge aborted.")
        return False
    return status in ("ok", "range")


def _apply_recenter(bus: FeetechBus, sid: int) -> tuple[bool, dict, dict]:
    """Feetech one-key calibrate: current pose → logical centre (2048).

    Does NOT turn the horn.  Torque must be off.  Returns
    ``(ok, before_fb, after_fb)``.
    """
    before = read_feedback_raw(bus, sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.05)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, TORQUE_CALIBRATE_MIDDLE)
    time.sleep(0.15)
    after = read_feedback_raw(bus, sid)
    ok = abs(after["count"] - STS_CENTRE_COUNT) <= 20 or abs(after["deg"]) < 5.0
    return ok, before, after


def _recenter_servo(bus: FeetechBus, sid: int) -> bool:
    """Define the *current* physical pose as logical centre (count 2048).

    Does NOT turn the horn — Feetech one-key calibrate (torque-enable=128).
    Hand-position the motor first (use limp), then call this.
    """
    before = read_feedback_raw(bus, sid)
    print(f"  Recenter = redefine THIS pose as 0 deg "
          f"(now count {before['count']}, {before['deg']:+.1f} deg).")
    print("  Horn will NOT turn — only the zero point changes.")
    if not ask_yn("  Set current position as centre?", "y"):
        return False
    ok, _before, after = _apply_recenter(bus, sid)
    print(f"    now reports count {after['count']}  "
          f"({after['deg']:+.1f} deg from new centre)")
    if ok:
        print("    recenter OK — current pose is logical 0 deg.")
        return True
    print("    warning: expected ~2048 after calibrate; power-cycle and "
          "retry if sweeps still look wrong.")
    return False


def zero_straight_out(bus: FeetechBus, reg: dict) -> None:
    """Pose all legs straight out from the body, then set that as 0° everywhere.

    Limps the bus, waits for you to hand-pose every joint, then runs the
    Feetech middle-calibrate on each answering robot servo (IDs 2..19).
    Horns do not turn — only the zero point is rewritten.
    """
    print()
    print("=" * 60)
    print("  ZERO POSE — legs straight out = 0°")
    print("=" * 60)
    print("  Pose the robot so EVERY leg sticks STRAIGHT OUT from the")
    print("  body (radial, flat, horns not fighting stops), then this")
    print("  redefines that physical pose as logical 0° on each servo.")
    print("  Motors will NOT move — they only re-zero their encoders.")
    print()
    print("  Note: walk plant uses standing_pose_degrees() (default hip +19° / "
          "knee +28°, or logs/plant_pose.json) — not the old CAD crouch −25°/+60°.")
    print("  as offsets from this mechanical zero.")
    print()

    all_live = set(bus.scan())
    live = sorted(i for i in all_live if i != FACTORY_SERVO_ID)
    if not live:
        print("  No robot servos answering (IDs 2..19).")
        return
    print_connected(bus, reg, live=all_live)
    print(f"  Will zero {len(live)} servo(s): {live}")
    offline_named = sorted(
        {int(k) for k in reg.get("servos", {})} - set(live))
    if offline_named:
        print(f"  Skipping offline named IDs (not on bus): {offline_named}")

    print()
    limp_ids(bus, live)
    print("  All listed servos are LIMP — hand-pose legs straight out now.")
    ask("  Enter when every joint is posed and clear of stops", "")

    if not ask_yn(f"  Write current pose as 0° on {len(live)} servo(s)?",
                  "y"):
        print("  cancelled.")
        return

    print()
    print(f"  {'ID':>3}  {'name':<10}  {'before':>10}  {'after':>10}  result")
    ok_n = 0
    fail: list[int] = []
    for sid in live:
        entry = reg.get("servos", {}).get(str(sid), {})
        name = entry.get("name", "unnamed")
        try:
            if not bus.ping(sid):
                print(f"  {sid:3d}  {name:<10}  {'—':>10}  {'—':>10}  "
                      f"MISSING")
                fail.append(sid)
                continue
            ok, before, after = _apply_recenter(bus, sid)
            tag = "OK" if ok else "WARN"
            if ok:
                ok_n += 1
            else:
                fail.append(sid)
            print(f"  {sid:3d}  {name:<10}  {before['deg']:+9.1f}°  "
                  f"{after['deg']:+9.1f}°  {tag}  "
                  f"(count {before['count']}→{after['count']})")
        except Exception as exc:
            print(f"  {sid:3d}  {name:<10}  failed: {exc}")
            fail.append(sid)

    limp_ids(bus, bus.scan())
    print()
    print(f"  Zeroed {ok_n}/{len(live)}.  Torque OFF.")
    if fail:
        print(f"  Needs attention: {fail}")
    else:
        print("  All answering servos now read ~0° at this straight-out pose.")
        print("  Tip: action t / a small sweep should move relative to this zero.")


def maybe_test_rotation(bus: FeetechBus, sid: int,
                        *, prompt: bool = True) -> None:
    """Sweep test with diagnostics + retry/reset.

    If ``prompt``, ask first (post-naming).  Otherwise go straight in
    (existing-motor test from the main menu).
    """
    if prompt and not ask_yn("Rotate farther to test this motor?", "y"):
        return

    amp = TEST_AMP_DEFAULT_DEG
    while True:
        print_servo_diagnostics(bus, sid, title="pre-test")
        raw = ask(f"  sweep ±deg (max {TEST_AMP_MAX_DEG:.0f})",
                  f"{amp:.0f}")
        try:
            amp = float(raw)
        except ValueError:
            print("  need a number (or 0 to skip)")
            continue
        if amp <= 0:
            print("  skipped.")
            return
        if amp > TEST_AMP_MAX_DEG:
            print(f"  capped at ±{TEST_AMP_MAX_DEG:.0f}°")
            amp = TEST_AMP_MAX_DEG

        fb = read_feedback_raw(bus, sid)
        room = min(fb["count"], (STS_COUNTS_PER_REV - 1) - fb["count"])
        room_deg = room / COUNTS_PER_DEG
        if amp > room_deg + 0.5:
            print(f"  Only ~{room_deg:.0f}° of room each side from here "
                  f"(pose {fb['deg']:+.1f} deg).  Full ±{amp:.0f}° will "
                  f"hit an encoder end — motor can still be fine.")
            print("  Tip: limp, hand-turn to a mid/clear pose, then "
                  "recenter (defines that pose as 0 — no motor turn).")
            if ask_yn("  Recenter HERE (make this pose 0 deg)?", "y"):
                if not _recenter_servo(bus, sid):
                    print("  recenter didn't take — see diagnostics.")
                continue

        print(f"  Test sweep ±{amp:.0f}° (speed {TEST_SPEED}, "
              f"torque ≤{TEST_TORQUE_LIMIT / 10:.0f}%; "
              f"auto-stop on load ≥{TEST_LOAD_ABORT_PCT:.0f}% / "
              f"{TEST_CURRENT_ABORT_A:.2f} A)")
        print("  KEEP CLEAR — path must be free for this range.")
        print("  Press ANY KEY (or Ctrl-C) to abort immediately.")
        status = _sweep_from_here(
            bus, sid, amp,
            speed=TEST_SPEED, acc=TEST_ACC,
            torque_limit=TEST_TORQUE_LIMIT,
            load_abort=TEST_LOAD_ABORT_PCT,
            current_abort=TEST_CURRENT_ABORT_A,
        )
        if status == "aborted":
            print("  Test aborted — motor limp.")
        elif status == "ok":
            print("  test sweep OK — motor looks healthy.")
        elif status == "range":
            print("  Partial sweep (encoder-range limited).  Low load "
                  "usually means the motor is OK — limp to a mid pose, "
                  "recenter (set as 0), then retry.")
        elif status == "blocked":
            print("  Stopped on high load/current — free the horn/path "
                  "before retrying.")
        else:
            print("  Didn't settle.  Check supply, cabling, or obstruction.")

        print("  next:  retry | recenter (set HERE as 0) | limp | amp | done")
        ans = ask("  ", "done").lower()
        if ans in ("done", "d", "q", "skip", "n", "no"):
            return
        if ans in ("retry", "r", "again", "y", "yes"):
            continue
        if ans in ("recenter", "centre", "center", "c", "0"):
            _recenter_servo(bus, sid)
            continue
        if ans in ("limp", "l", "free"):
            bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 0)
            print("  torque OFF — hand-turn to the pose you want as 0 deg,")
            ask("  then press Enter (use 'recenter' next to save it)", "")
            continue
        if ans in ("amp", "a", "deg"):
            continue
        # Treat a bare number as new amplitude.
        try:
            amp = float(ans)
            continue
        except ValueError:
            print("  unknown — use retry / recenter / limp / amp / done")
            continue


def parse_target(text: str, *, allow_taken: bool = False,
                 reg: dict | None = None) -> dict | None:
    """Parse 'L2 knee', '2 hip', 'knee L3', 'j7', menu '3' / '#3', etc."""
    raw = text.strip().lower()
    if not raw:
        return None

    # Explicit joint index: j7 / joint 7
    m = re.fullmatch(r"(?:j|joint)\s*(\d{1,2})", raw)
    if m:
        j = int(m.group(1))
        if 0 <= j <= 17:
            return target_for(j // 3, AXIS_NAMES[j % 3])

    # L<leg> <axis>  or  <leg> <axis>  (either order)
    m = re.fullmatch(
        r"l?\s*([0-5])\s*[/\-:,]?\s*(yaw|y|hip|h|pitch|p|knee|k)",
        raw,
    )
    if m:
        return target_for(int(m.group(1)), AXIS_ALIASES[m.group(2)])

    m = re.fullmatch(
        r"(yaw|y|hip|h|pitch|p|knee|k)\s*[/\-:,]?\s*l?\s*([0-5])",
        raw,
    )
    if m:
        return target_for(int(m.group(2)), AXIS_ALIASES[m.group(1)])

    # Bare / #N → 1-based menu into open slots (when registry given).
    if reg is not None and re.fullmatch(r"#?\d{1,2}", raw):
        idx = int(raw.lstrip("#")) - 1
        left = all_targets() if allow_taken else remaining_targets(reg)
        if not left and allow_taken is False:
            left = all_targets()
        if 0 <= idx < len(left):
            return left[idx]

    return None


def ask_target(reg: dict, *, default: dict | None = None) -> dict | None:
    """Ask the user which joint this motor should become."""
    left = remaining_targets(reg)
    done = {int(k) for k in reg.get("servos", {})}

    print()
    print("  What should this motor be?")
    if left:
        print("  Open slots:")
        for i, t in enumerate(left, 1):
            print(f"    {i:2d})  {t['name']:<10}  -> ID {t['id']}  "
                  f"(joint {t['joint']})")
    else:
        print("  (all 18 slots filled — pick an existing one to reassign)")

    print("  Type e.g.  L2 knee  |  2 hip  |  j7  |  menu number  |  skip")
    default_s = default["name"] if default is not None else None

    while True:
        ans = ask("  assign to", default_s)
        if ans.lower() in ("skip", "s", "none", "q", "quit"):
            return None
        target = parse_target(ans, reg=reg)
        if target is None:
            print("  couldn't parse that — try 'L1 hip' or a menu number")
            continue
        if target["id"] in done:
            print(f"  {target['name']} (ID {target['id']}) is ALREADY named.")
            if not ask_yn("  Reassign / overwrite that slot?", "n"):
                continue
        return target


def pick_unnamed(bus: FeetechBus, already: set[int],
                 reg: dict) -> int | None:
    """Return an unnamed ID already on the bus, or None to wait."""
    live = print_connected(bus, reg)
    unnamed = sorted(live - already)
    if not unnamed:
        return None

    if FACTORY_SERVO_ID in unnamed:
        primary = FACTORY_SERVO_ID
    elif len(unnamed) == 1:
        primary = unnamed[0]
    else:
        print(f"  several unnamed IDs: {unnamed}")
        print("  unplug extras so only the NEW servo is unnamed "
              f"(named IDs {sorted(already & live) or 'none'} may stay).")
        return None

    if ask_yn(f"Identify the servo at ID {primary} (gentle wiggle)?", "y"):
        return primary
    print("  OK -- waiting for a different servo instead.")
    return None


def name_servo(bus: FeetechBus, sid: int, target: dict, reg: dict) -> bool:
    """Wiggle already done; re-ID ``sid`` -> ``target`` and save registry."""
    label = f"{target['name']} / ID {target['id']}"
    assert target["id"] != FACTORY_SERVO_ID, "robot IDs must not use ID 1"

    if not ask_yn(f"Name this motor '{label}' and set ID "
                  f"{sid} -> {target['id']}?"):
        print("  skipped.")
        return False

    from_id = sid
    if sid != target["id"]:
        on_bus = set(bus.scan())
        # If the target ID is live but it's THIS servo after a previous
        # partial rename, fine; otherwise collision.
        if target["id"] in on_bus and target["id"] != sid:
            print(f"  ID {target['id']} is ALREADY on the bus -- "
                  "collision.  Unplug the duplicate.")
            return False
        print(f"  re-ID {sid} -> {target['id']} ...")
        bus.set_id(sid, target["id"])
        time.sleep(0.1)
        sid = target["id"]
        if not bus.ping(sid):
            print("  re-ID FAILED -- power-cycle the chain and retry.")
            return False
        print("  re-ID OK")
    else:
        print(f"  already at ID {sid} -- keeping it")

    # Drop any previous registry entry that claimed this slot, and any
    # stale entry keyed by the old from_id if it was a robot ID.
    servos = reg.setdefault("servos", {})
    for key in list(servos.keys()):
        entry = servos[key]
        if int(key) == target["id"] or (
            entry.get("joint") == target["joint"]
        ):
            del servos[key]

    servos[str(target["id"])] = {
        "id": target["id"],
        "name": target["name"],
        "leg": target["leg"],
        "axis": target["axis"],
        "joint": target["joint"],
        "named_at": datetime.now(timezone.utc).isoformat(),
        "from_id": from_id,
    }
    save_registry(reg)

    print()
    print(f"  >>>  LABEL THIS SERVO:   {label}")
    print(f"  >>>  leave it plugged in if you like.")
    print(f"  (progress: {len(reg['servos'])} / 18)")
    return True


def identify_and_name(bus: FeetechBus, reg: dict,
                      *, preset: dict | None = None) -> bool:
    """Detect one unnamed servo, wiggle, ask what it is, name it."""
    already = {int(k) for k in reg.get("servos", {})}

    print()
    print("=" * 60)
    print("  NEW MOTOR  — plug it in (factory ID 1 is fine)")
    print("=" * 60)
    print("  Named motors may stay on the daisy-chain.")
    print("  Horn empty on the newcomer.")

    sid = pick_unnamed(bus, already, reg)
    if sid is None:
        ask("Press Enter when the new servo is on the bus", "")
        sid = pick_unnamed(bus, already, reg)
        if sid is None:
            sid = wait_for_unnamed_servo(
                bus, already, reg,
                prompt_hint="Plug ANY unnamed STS3215; you'll choose "
                            "its joint after the wiggle.",
            )
    print(f"  using servo at ID {sid}")
    limp_ids(bus, sorted(set(bus.scan()) - {sid}))

    fb = read_feedback_raw(bus, sid)
    print(f"  {fb['volt']:.1f} V, {fb['temp_c']} degC, "
          f"at {fb['deg']:+.1f} deg")
    if fb["volt"] < MIN_BUS_VOLT:
        print(f"  STOP: bus is only {fb['volt']:.1f} V.  Connect 12 V to "
              "the URT-2 screw terminals.")
        return False

    # These STS3215 lots often ship with EEPROM max-voltage = 8.0 V
    # (7.4 V-SKU factory table) even when sold as 12 V / 30 kg.  Our
    # tools never write 8 V — only 4–14 V.  Fix once on first contact
    # so every new motor doesn't trip a voltage alarm on the 12 V bus.
    try:
        h0 = read_servo_health(bus, sid)
    except Exception:
        h0 = {}
    if h0.get("volt_limit_max", 14.0) < 10.0:
        print(f"  Newcomer EEPROM max-voltage is "
              f"{h0['volt_limit_min']:.1f}–{h0['volt_limit_max']:.1f} V "
              f"(factory default on this batch).")
        print("  Rewriting to 4.0–14.0 V for the 12 V bus "
              "(one-time EEPROM fix) ...")
        after = restore_voltage_limits(bus, sid)
        if after is None:
            print("  Could not rewrite limits (flaky link).  Continue "
                  "carefully, or heal later with action i / d.")
        else:
            print(f"  Limits now {after['volt_limit_min']:.1f}"
                  f"–{after['volt_limit_max']:.1f} V  "
                  f"status=0x{after['status']:02X}")

    ensure_position_mode(bus, sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)
    print("  torque: ON (newcomer only)")

    try:
        tracked = gentle_wiggle(bus, sid)
        if not tracked:
            print("  Identify move aborted or didn't settle.  Free the horn "
                  f"and keep the path clear (±{WIGGLE_AMP_DEG:.0f}° only).")
            if not ask_yn("Retry naming this servo anyway?", "n"):
                return False
        elif not ask_yn(
                f"Did THAT servo nudge (±{WIGGLE_AMP_DEG:.0f}°) cleanly "
                "(others still)?"):
            print("  OK -- leaving it alone.  Fix and retry.")
            return False

        target = preset or ask_target(reg)
        if target is None:
            print("  skipped.")
            return False

        named = name_servo(bus, sid, target, reg)
        if named:
            # After re-ID the servo answers at the new ID.
            maybe_test_rotation(bus, target["id"])
        return named
    finally:
        try:
            # After re-ID, limp the (possibly new) id.
            live = set(bus.scan())
            limp_ids(bus, sorted(live))
            print("  torque: OFF (limp)")
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Legacy per-leg guided flow (optional)
# ---------------------------------------------------------------------------


def configure_one(bus: FeetechBus, target: dict, reg: dict,
                  baseline: set[int]) -> tuple[bool, set[int]]:
    print()
    print("=" * 60)
    print(f"  LEG {target['leg']}  ·  {target['axis'].upper()}  "
          f"->  servo ID {target['id']}   ({target['name']})")
    print("=" * 60)
    ok = identify_and_name(bus, reg, preset=target)
    return ok, set(bus.scan())


def run_leg(bus: FeetechBus, leg: int, reg: dict) -> bool:
    """Configure the incomplete joints of one leg.  Returns True if leg done."""
    done = {int(k) for k in reg.get("servos", {})}
    targets = [t for t in leg_targets(leg) if t["id"] not in done]
    ids = [target_for(leg, a)["id"] for a in AXIS_NAMES]

    print()
    print("#" * 60)
    print(f"  LEG {leg}   joints -> IDs {ids[0]}/{ids[1]}/{ids[2]}  "
          f"(yaw/hip/knee)")
    print(f"  Order inside the leg: {' -> '.join(AXIS_ORDER_IN_LEG)}")
    print("#" * 60)

    if not targets:
        print(f"  Leg {leg} already fully named.")
        return True

    print(f"  Still need: {', '.join(t['name'] for t in targets)}")
    if not ask_yn(f"Start / continue leg {leg}?"):
        return False

    baseline = set(bus.scan())
    for t in targets:
        ok, baseline = configure_one(bus, t, reg, baseline)
        if not ok:
            if ask_yn("Retry this same joint?", "y"):
                ok, baseline = configure_one(bus, t, reg, baseline)
            if not ok:
                print(f"  stopping leg {leg} early; resume with --leg {leg}")
                return False
        if t is not targets[-1]:
            ask("Plug the NEXT joint of this leg onto the chain, "
                "then press Enter", "")
            baseline = set(bus.scan())

    remaining = [t for t in leg_targets(leg)
                 if t["id"] not in {int(k) for k in reg.get("servos", {})}]
    if remaining:
        return False
    print()
    print(f"  Leg {leg} complete.  Label check: "
          + ", ".join(f"{a}={target_for(leg, a)['id']}" for a in AXIS_NAMES))
    return True


def ask_existing_servo(bus: FeetechBus, reg: dict) -> int | None:
    """Pick a motor already on the bus by name, ID, or menu number."""
    live = print_connected(bus, reg)
    if not live:
        print("  Nothing on the bus.")
        return None

    entries = []
    for sid in sorted(live):
        entry = reg.get("servos", {}).get(str(sid))
        name = entry["name"] if entry else _label_for_id(sid, reg)
        entries.append((sid, name, entry is not None))

    print("  Pick a connected motor:")
    for i, (sid, name, named) in enumerate(entries, 1):
        tag = "named" if named else "unnamed"
        print(f"    {i:2d})  ID {sid:2d}  {name:<12}  [{tag}]")
    print("  Type name (L0 yaw), bus ID, or #menu  |  skip")

    while True:
        ans = ask("  motor", None)
        if not ans or ans.lower() in ("skip", "s", "q", "quit", "none"):
            return None
        # Explicit menu: #1
        if re.fullmatch(r"#\d{1,2}", ans):
            idx = int(ans[1:]) - 1
            if 0 <= idx < len(entries):
                return entries[idx][0]
            print("  bad menu #")
            continue
        # Bare number = bus ID if live, else menu index.
        if re.fullmatch(r"\d{1,2}", ans):
            n = int(ans)
            if n in live:
                return n
            if 1 <= n <= len(entries):
                return entries[n - 1][0]
            print("  not a live ID or menu #")
            continue
        target = parse_target(ans, allow_taken=True, reg=reg)
        if target is not None:
            if target["id"] in live:
                return target["id"]
            print(f"  {target['name']} (ID {target['id']}) is not on the bus")
            continue
        print("  couldn't parse — try 'L2 knee', a bus ID, or #1")


def ask_offline_servo(bus: FeetechBus, reg: dict) -> int | None:
    """Pick a named motor that is NOT answering the bus."""
    live = set(bus.scan())
    offline = []
    for sid_s, entry in sorted(reg.get("servos", {}).items(),
                               key=lambda kv: int(kv[0])):
        sid = int(sid_s)
        if sid not in live:
            offline.append((sid, entry["name"]))
    if not offline:
        print("  No offline named motors — everything in the registry answers.")
        return None

    print("  Offline named motors:")
    for i, (sid, name) in enumerate(offline, 1):
        print(f"    {i:2d})  ID {sid:2d}  {name}")
    print("  Type name, bus ID, or #menu  |  skip")
    offline_ids = {sid for sid, _ in offline}

    while True:
        ans = ask("  motor to probe", None)
        if not ans or ans.lower() in ("skip", "s", "q", "quit", "none"):
            return None
        if re.fullmatch(r"#\d{1,2}", ans):
            idx = int(ans[1:]) - 1
            if 0 <= idx < len(offline):
                return offline[idx][0]
            print("  bad menu #")
            continue
        if re.fullmatch(r"\d{1,2}", ans):
            n = int(ans)
            if n in offline_ids:
                return n
            if 1 <= n <= len(offline):
                return offline[n - 1][0]
            if n in live:
                print(f"  ID {n} is live — probe is for offline motors")
            else:
                print("  not an offline ID or menu #")
            continue
        target = parse_target(ans, allow_taken=True, reg=reg)
        if target is not None:
            if target["id"] in offline_ids:
                return target["id"]
            if target["id"] in live:
                print(f"  {target['name']} is live — nothing to probe")
            else:
                print(f"  {target['name']} is not in the offline list")
            continue
        print("  couldn't parse — try 'L0 yaw', a bus ID, or #1")


def probe_offline(bus: FeetechBus, reg: dict, *,
                  baud: int = BAUD_DEFAULT) -> None:
    """Retries + baud sweep for one named motor that isn't answering."""
    print()
    print("=" * 60)
    print("  PROBE OFFLINE MOTOR")
    print("=" * 60)
    print_connected(bus, reg)
    sid = ask_offline_servo(bus, reg)
    if sid is None:
        return
    entry = reg.get("servos", {}).get(str(sid), {})
    probe_missing_motor(
        bus, sid,
        name=entry.get("name", f"ID {sid}"),
        working_baud=baud,
    )
    print_connected(bus, reg)


def prepare_existing(bus: FeetechBus, sid: int) -> bool:
    """Limp others, check voltage, enable torque on ``sid``."""
    limp_ids(bus, sorted(set(bus.scan()) - {sid}))
    fb = read_feedback_raw(bus, sid)
    print(f"  ID {sid}: {fb['volt']:.1f} V, {fb['temp_c']} degC, "
          f"at {fb['deg']:+.1f} deg")
    if fb["volt"] < MIN_BUS_VOLT:
        print(f"  STOP: bus is only {fb['volt']:.1f} V.")
        return False
    ensure_position_mode(bus, sid)
    bus.pkt.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, 1)
    return True


def test_existing(bus: FeetechBus, reg: dict) -> None:
    """Run the rotate-to-test flow on a motor already on the bus."""
    print()
    print("=" * 60)
    print("  TEST EXISTING MOTOR")
    print("=" * 60)
    sid = ask_existing_servo(bus, reg)
    if sid is None:
        return
    try:
        if not prepare_existing(bus, sid):
            return
        maybe_test_rotation(bus, sid, prompt=False)
    finally:
        limp_ids(bus, bus.scan())
        print("  torque: OFF (limp)")


def rename_existing(bus: FeetechBus, reg: dict) -> bool:
    """Reassign a live motor to a different joint / ID."""
    print()
    print("=" * 60)
    print("  RENAME EXISTING MOTOR")
    print("=" * 60)
    sid = ask_existing_servo(bus, reg)
    if sid is None:
        return False

    old = reg.get("servos", {}).get(str(sid))
    if old:
        print(f"  currently: {old['name']}  (ID {sid}, joint {old['joint']})")
    else:
        print(f"  ID {sid} is on the bus but not in the registry.")

    # Temporarily treat this ID as free so ask_target's "already named"
    # prompt is about the *destination* slot, not the source.
    shadow = json.loads(json.dumps(reg))
    shadow.get("servos", {}).pop(str(sid), None)
    target = ask_target(shadow)
    if target is None:
        print("  skipped.")
        return False
    if target["id"] == sid and old and old.get("joint") == target["joint"]:
        print("  already that joint — nothing to do.")
        if ask_yn("Test it instead?", "y"):
            try:
                if prepare_existing(bus, sid):
                    maybe_test_rotation(bus, sid, prompt=False)
            finally:
                limp_ids(bus, bus.scan())
        return False

    try:
        if not prepare_existing(bus, sid):
            return False
        # Gentle identify so the user confirms which physical motor.
        if ask_yn(f"Gentle nudge ID {sid} to confirm it's the right one?",
                  "y"):
            if not gentle_wiggle(bus, sid):
                if not ask_yn("Nudge failed — rename anyway?", "n"):
                    return False
        ok = name_servo(bus, sid, target, reg)
        if ok and ask_yn("Test the renamed motor?", "y"):
            maybe_test_rotation(bus, target["id"], prompt=False)
        return ok
    finally:
        limp_ids(bus, bus.scan())
        print("  torque: OFF (limp)")


def ask_named_servo(bus: FeetechBus, reg: dict) -> int | None:
    """Pick any registry motor (connected or offline) by name / ID / menu #."""
    servos = reg.get("servos") or {}
    if not servos:
        print("  Registry is empty — nothing to delete.")
        return None

    live = set(bus.scan())
    entries: list[tuple[int, str, bool]] = []
    for sid_s, entry in sorted(servos.items(), key=lambda kv: int(kv[0])):
        sid = int(sid_s)
        name = str(entry.get("name") or f"ID{sid}")
        entries.append((sid, name, sid in live))

    print("  Named motors in the registry:")
    for i, (sid, name, on) in enumerate(entries, 1):
        status = "connected" if on else "offline"
        print(f"    {i:2d})  ID {sid:2d}  {name:<12}  [{status}]")
    print("  Type name (L0 yaw), bus ID, or #menu  |  skip")
    named_ids = {sid for sid, _, _ in entries}

    while True:
        ans = ask("  motor to delete", None)
        if not ans or ans.lower() in ("skip", "s", "q", "quit", "none"):
            return None
        if re.fullmatch(r"#\d{1,2}", ans):
            idx = int(ans[1:]) - 1
            if 0 <= idx < len(entries):
                return entries[idx][0]
            print("  bad menu #")
            continue
        if re.fullmatch(r"\d{1,2}", ans):
            n = int(ans)
            if n in named_ids:
                return n
            if 1 <= n <= len(entries):
                return entries[n - 1][0]
            print("  not a named ID or menu #")
            continue
        target = parse_target(ans, allow_taken=True, reg=reg)
        if target is not None and target["id"] in named_ids:
            return target["id"]
        print("  couldn't parse — try 'L2 knee', a bus ID, or #1")


def delete_motor(bus: FeetechBus, reg: dict) -> bool:
    """Remove a named motor from the registry so a replacement can take its slot.

    Optionally resets a still-live servo's bus ID back to factory ID 1
    (unplug any other ID-1 first).  Dead/offline motors are registry-only.
    """
    print()
    print("=" * 60)
    print("  DELETE / REPLACE MOTOR")
    print("=" * 60)
    print("  Frees the joint slot in the registry.  Then use n to name")
    print("  a new factory (ID 1) servo into that slot.")
    print()

    sid = ask_named_servo(bus, reg)
    if sid is None:
        return False

    entry = reg.get("servos", {}).get(str(sid))
    if not entry:
        print(f"  ID {sid} is not in the registry.")
        return False

    name = entry.get("name", f"ID{sid}")
    joint = entry.get("joint", "?")
    live = set(bus.scan())
    on_bus = sid in live
    status = "connected" if on_bus else "offline (not answering)"

    print()
    print(f"  Selected: {name}  ID {sid}  joint {joint}  [{status}]")
    if not ask_yn(f"Delete '{name}' (ID {sid}) from the registry?", "n"):
        print("  cancelled.")
        return False

    del reg["servos"][str(sid)]
    # Also drop any stale duplicate keyed elsewhere for the same joint.
    for key in list(reg.get("servos", {}).keys()):
        other = reg["servos"][key]
        if other.get("joint") == entry.get("joint") and int(key) != sid:
            print(f"  also removing stale registry key ID {key} "
                  f"({other.get('name')})")
            del reg["servos"][key]
    save_registry(reg)
    print(f"  Removed '{name}' from {REGISTRY_PATH.name}.")
    print(f"  Slot free — {len(remaining_targets(reg))} open joint(s).")

    if on_bus:
        print()
        print(f"  ID {sid} is still on the bus (old hardware).")
        print("  For a clean replace: unplug it, OR reset it to factory")
        print(f"  ID {FACTORY_SERVO_ID} so it becomes an unnamed newcomer.")
        if ask_yn(f"Reset this servo ID {sid} → {FACTORY_SERVO_ID}?", "n"):
            live = set(bus.scan())
            if FACTORY_SERVO_ID in live:
                print(f"  ID {FACTORY_SERVO_ID} is already on the bus — "
                      "unplug that servo first, then retry delete/reset.")
            else:
                try:
                    limp_now(bus, sid)
                    print(f"  re-ID {sid} → {FACTORY_SERVO_ID} ...")
                    bus.set_id(sid, FACTORY_SERVO_ID)
                    time.sleep(0.15)
                    if bus.ping(FACTORY_SERVO_ID):
                        print(f"  OK — now factory ID {FACTORY_SERVO_ID}. "
                              "Unplug it if you're installing a different motor.")
                    else:
                        print("  re-ID may have failed — power-cycle and scan.")
                except Exception as exc:
                    print(f"  re-ID error: {exc}")
                finally:
                    limp_ids(bus, bus.scan())
    else:
        print("  Old motor was offline — unplug/replace the hardware, then")
        print("  plug a factory ID-1 servo and run action n.")

    print()
    print(f"  Next: action n → name the replacement as '{name}'.")
    return True


def print_actions_help() -> None:
    print("  Actions:")
    print("    n  name     — identify + name a NEW unnamed motor")
    print("    t  test     — rotate-test a motor already on the bus")
    print("    r  rename   — reassign name/slot for a live motor")
    print("    x  delete   — remove a named motor (free slot to replace)")
    print("    d  debug    — fault decode for all connected motors")
    print("    i  info     — query fw/version + voltage limits (SKU hint)")
    print("    z  zero     — redefine CURRENT pose as 0° (calibrate, no move)")
    print("    g  go-zero  — slowly DRIVE all joints to 0°")
    print("    l  limp     — torque OFF on all motors (free to hand-pose)")
    print("    f  demo     — demos (wave / rise / rise_turn / rise_show)")
    print("    p  probe    — retries + baud sweep for one OFFLINE named motor")
    print("    s  scan     — rescan the bus and refresh the connected list")
    print("    ?  help     — show this list")
    print("    q  quit")


def run_freeform(bus: FeetechBus, reg: dict, *, once: bool = False,
                 baud: int = BAUD_DEFAULT) -> None:
    """Main loop: name / test / rename / delete / debug / probe / scan."""
    print()
    print("Motor setup  (any order)")
    print("------------------------")
    print_actions_help()
    print()

    while True:
        print_status(reg)
        print_connected(bus, reg)
        live = set(bus.scan())
        named_live = live & {int(k) for k in reg.get("servos", {})}
        default = "n"
        if not remaining_targets(reg) and named_live:
            default = "t"
        elif named_live and FACTORY_SERVO_ID not in live:
            default = "n"

        ans = ask("  action [n/t/r/x/d/i/z/g/l/f/p/s/?/q]", default).lower()
        if ans in ("?", "help", "h", "actions"):
            print_actions_help()
            continue
        if ans in ("q", "quit", "exit", "done"):
            return
        if ans in ("s", "scan", "rescan", "ls", "list"):
            print("  Rescanning ...")
            live = set(bus.scan())
            limp_ids(bus, sorted(live))
            print_connected(bus, reg, live=live)
            continue
        if ans in ("l", "limp", "relax", "free"):
            ids = sorted(bus.scan())
            limp_ids(bus, ids)
            print(f"  Torque OFF on {len(ids)} motor(s) — limp / free to pose.")
            continue
        if ans in ("i", "info", "identity", "sku", "versions"):
            def _info_label(sid: int) -> str:
                entry = reg.get("servos", {}).get(str(sid))
                if entry:
                    return entry["name"]
                if sid == FACTORY_SERVO_ID:
                    return "factory"
                return "unnamed"
            print_identity_report(bus, sorted(live), label_fn=_info_label)
            continue
        if ans in ("z", "zero", "zeroing", "calibrate", "straight"):
            zero_straight_out(bus, reg)
            continue
        if ans in ("g", "go", "go-zero", "gozero", "home", "rest"):
            from inplace_demos import go_to_zero_pose
            if ask_yn("  Clear path — slowly drive all joints to 0°?", "y"):
                with keystroke_abort_watch() as abort_check:
                    go_to_zero_pose(bus, abort_check=abort_check)
            continue
        if ans in ("f", "fun", "demo", "demos", "show"):
            from inplace_demos import run_demo_menu
            run_demo_menu(bus)
            continue
        if ans in ("n", "name", "new", ""):
            if not remaining_targets(reg):
                print("  All 18 slots filled — use delete (x) to free a slot,")
                print("  or rename (r) to reassign.")
                if once:
                    return
                continue
            identify_and_name(bus, reg)
        elif ans in ("t", "test"):
            if not live:
                print("  No motors on the bus.")
            else:
                test_existing(bus, reg)
        elif ans in ("r", "rename", "reassign"):
            if not live:
                print("  No motors on the bus.")
            else:
                rename_existing(bus, reg)
        elif ans in ("x", "delete", "del", "rm", "remove", "forget",
                     "unassign", "replace"):
            delete_motor(bus, reg)
        elif ans in ("d", "debug", "diag", "faults", "led"):
            debug_bus(bus, reg, baud=baud)
        elif ans in ("p", "probe"):
            probe_offline(bus, reg, baud=baud)
        else:
            print("  unknown action — type ? for the list")
            continue

        if once:
            return
        print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--leg", type=int, default=None, choices=range(6),
                    metavar="N",
                    help="guided per-leg mode starting at this leg "
                         "(default is free-form any-order naming)")
    ap.add_argument("--once", action="store_true",
                    help="name a single servo then exit (free-form mode)")
    ap.add_argument("--status", action="store_true",
                    help="print registry + connected list (with alarms)")
    ap.add_argument("--debug", action="store_true",
                    help="dump status/fault decode for every servo on the bus")
    ap.add_argument("--reset", action="store_true")
    args = ap.parse_args(argv)

    if args.reset and REGISTRY_PATH.exists():
        if ask_yn(f"Delete {REGISTRY_PATH.name} and start over?", "n"):
            REGISTRY_PATH.unlink()
            print("  registry cleared.")

    reg = load_registry()
    if reg.get("servos") and reg.get("scheme") != "ids_2_to_19":
        print("Registry looks like the old ID 1..18 scheme.  "
              "Recommended: --reset and re-name under IDs 2..19.")
        if not args.status and ask_yn("Clear it now?", "y"):
            REGISTRY_PATH.unlink(missing_ok=True)
            reg = load_registry()
        elif args.status:
            print("NOTE: registry looks like the old ID 1..18 scheme; "
                  "run with --reset before naming under IDs 2..19.\n")

    print("STS3215 motor setup")
    print("-------------------")
    print(f"  Robot IDs: 2..19  (joint + {SERVO_ID_OFFSET})")
    print(f"  Factory ID {FACTORY_SERVO_ID} is NEVER assigned")
    print("  Bus: MCU UART bridge (Uno Q D0/D1→URT) or USB URT-2")
    print("  12 V on the URT screw terminals  (required)")
    if not args.status and not args.debug:
        print("  Mode: " + ("guided per-leg" if args.leg is not None
                            else "free-form (any order)"))
    print()
    print_status(reg)

    port = args.port or default_port()
    if port is None:
        raise SystemExit(
            "No bus found.  On Uno Q: flash feetech_bridge + wire URT "
            "UART→D0/D1.  Or plug a USB URT-2.")
    # Prefer shared open_feetech_bus (MCU bridge on board, USB elsewhere).
    bus = None
    for parent in (
        Path(__file__).resolve().parent.parent / "linux_control",
        Path.home() / "hexapod_sts" / "linux_control",
        Path(__file__).resolve().parent,
    ):
        if (parent / "mcu_feetech_bus.py").is_file():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            break
    try:
        from mcu_feetech_bus import open_feetech_bus  # type: ignore
        print(f"\nOpening bus ({port}) ...")
        bus, port = open_feetech_bus(port, baud=args.baud)
    except ImportError:
        if port in ("mcu", "MCU", "bridge", "ttyHS1"):
            raise SystemExit(
                "MCU bridge requested but mcu_feetech_bus.py not found "
                "(deploy linux_control to the board).")
        print(f"\nOpening {port} @ {args.baud} baud ...")
        bus = FeetechBus(port, args.baud)

    try:
        present = print_connected(bus, reg)
        limp_ids(bus, sorted(present))

        if args.debug:
            debug_bus(bus, reg, baud=args.baud)
            print("\nExiting (--debug is a one-shot pass; re-run or use the "
                  "free-form menu without --debug for n/t/r/d/p/s).")
            return

        if args.status:
            return

        if args.leg is not None:
            # Legacy guided flow.
            start = args.leg
            for leg in range(start, 6):
                finished = run_leg(bus, leg, reg)
                if not finished:
                    break
                if leg < 5:
                    print()
                    if not ask_yn(f"Leg {leg} done.  Continue to leg "
                                  f"{leg + 1}?", "y"):
                        print(f"  Resume later with:  "
                              f"python urt2_motor_setup.py --leg {leg + 1}")
                        break
            print()
            print("Session done.")
            print_status(reg)
            print_connected(bus, reg)
        else:
            run_freeform(bus, reg, once=args.once, baud=args.baud)
            print()
            print("Session done.")
            print_status(reg)
            print_connected(bus, reg)

        if len(reg.get("servos", {})) >= 18:
            print("All 18 servos named (IDs 2..19).  Verify with a bus "
                  "scan -- ID 1 should be absent.")
    finally:
        try:
            limp_ids(bus, bus.scan())
        except Exception:
            pass
        bus.close()


if __name__ == "__main__":
    main()
