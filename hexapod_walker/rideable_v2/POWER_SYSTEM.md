# `rideable_v2` — Power System (12S / "48 V" bus)

> The brief's electrical architecture — **battery → contactor/pre-charge
> → six fused leg branches → 18 AK80-64s** — adopted as-is, with the
> voltages and numbers pinned down. One correction of terminology:
> because the AK80-64 tops out at **12S**, the "48 V system" is
> concretely a **12S Li-ion pack: 50.4 V full / 44.4 V nominal / ~36 V
> empty**. A 13S "48 V" e-bike pack (54.6 V full) would exceed the
> actuator rating — do not substitute one.

---

## 1. Bus architecture

```
   [ 12S Li-ion pack, ~40 Ah / ~1.8 kWh ]
        │  50.4 V full / 44.4 V nom / ~36 V empty
        ├─ MAIN FUSE (Class-T ~100 A, at the pack +)
        ├─ pre-charge (resistor + relay) ─ MAIN CONTACTOR  ◄── opened by the e-stop
        │
        ├── branch fuse 30 A ──► LEG 1: yaw + hip + knee AK80-64   (3 × CAN)
        ├── branch fuse 30 A ──► LEG 2:  "            "
        ├── branch fuse 30 A ──► LEG 3:  "            "
        ├── branch fuse 30 A ──► LEG 4:  "            "
        ├── branch fuse 30 A ──► LEG 5:  "            "
        ├── branch fuse 30 A ──► LEG 6:  "            "
        │
        └── 48→24 V DC-DC (≥350 W) ──► logic + compute + contactor coil
                                     └► 12 × parking-lock solenoids (default OFF = pins engaged)
```

* **One voltage for all 18 actuators** — no per-joint rails (v1 needed a
  72/48 V split because it mixed actuator families; v2's single SKU
  deletes that converter).
* **Six fused leg branches** (the brief's idea, kept): a wiring fault or
  a failed motor takes out *one leg*, not the bus, and the machine can
  freeze, park, and drop its locks. 30 A per branch covers the
  3 × 7 A rated draw with headroom and rides through a single 19 A peak
  transient; it will *not* ride three simultaneous peaks — the
  controller staggers per-leg peak demands anyway for gait reasons.
* **24 V rail sized for all 12 lock solenoids held retracted at once**:
  12 × ~12 W (PWM-economized hold) ≈ 145 W + logic/compute → **≥350 W**
  with ample headroom. An undersized logic-class converter browning out
  would drop every pin mid-stride — v1's lesson, kept.

---

## 2. Current / power budget

Standing costs ~nothing: the **parking pins** hold, the motors idle,
and (unlike held brake coils) a locked park draws no solenoid power
at all
([`DRIVETRAIN.md` §6](DRIVETRAIN.md#6-load-holding-joint-side-parking-pin-locks-v1s-rule-better-hardware)).

| State | What's happening | Bus power | Bus current (@44.4 V) |
|---|---|---:|---:|
| **Parked (pins in)** | pins hold, motors + solenoids off | ~40–80 W | ~1–2 A |
| **Walking, cruise 0.3 m/s** | tripod, 12 solenoids held | **~0.9 kW** | ~20 A |
| **Walking, peak** | worst stride phase / small slope | **~1.8 kW** | ~40 A |
| **Stand-up transient** | lifting chassis + rider, several joints near peak, 1–2 s | ~3–4 kW | **70–90 A** |
| **Fault bound** | multiple AK80-64s at 19 A peak | — | size hardware ≥ ~100 A |

Design figures: wiring/contactor/BMS sized to the **70–90 A stand-up
transient**, main fuse **~100 A** (above transient, below main-lead
ampacity), and the controller sequences stand-up in two leg groups so
the un-sequenced worst case stays a fault bound, not an operating point.

---

## 3. Pack sizing

| Parameter | Value | Basis |
|---|---:|---|
| Chemistry / config | Li-ion, **12S** (~8–10P 21700) | AK80-64 max = 12S |
| Voltage | ~36 empty / **44.4 nom** / 50.4 full | 12 × (3.0/3.7/4.2) V |
| Capacity / energy | **~40 Ah / ~1.8 kWh** | below |
| Continuous discharge | ≥ **100 A** BMS + cells | transient + fault bound |
| Mass | **~10 kg** | mass budget row |

```
    cruise:   1.8 kWh / 0.9 kW ≈ 2.0 h theoretical
    derated:  ~80% DoD, transients, cool-downs → plan 45–75 min real walking
```

Standing between moves is genuinely free (pins in, solenoids off), so
endurance is set by walking time, not clock time.

---

## 4. Fusing, contactor & wiring

Golden rule (unchanged from v1 / the prototype): **the fuse protects the
wire, not the load.**

| Item | Spec | Why |
|---|---|---|
| Main fuse | Class-T **~100 A** at the pack + | High interrupt rating for lithium; above the 90 A transient. |
| Main lead | 6 AWG silicone | ~100 A path, short runs. |
| Main contactor | ≥ 120 A DC + **pre-charge resistor/relay** | Soft-charge 18 actuators' bus caps; e-stop opens it. |
| Branch fuses | **6 × 30 A** (blade/MIDI on a fused bus bar) | One per leg; 3 × 7 A rated per branch. |
| Branch wiring | 12 AWG per leg, XT30 → each actuator | AK80-64 uses XT30 power connectors. |
| DC-DC 48→24 V | **≥ 350 W** | 12 held lock solenoids (~145 W) + logic + contactor. |
| Bus bars | +/− distribution blocks | Don't neck 100 A through one terminal. |

CAN: the 18 actuators sit on 2–3 CAN buses (6–9 nodes each) with
termination at the ends; keep CAN dressed away from the belt runs and
power leads. (If the AKH70-48 hip swap ever happens, its dual-CAN
daisy-chain halves this harness — noted in
[`DRIVETRAIN.md` §7](DRIVETRAIN.md#7-the-akh70-48-alternative).)

---

## 5. The fail-safe lock power interlock

Identical philosophy to v1 (it is the safety heart of the machine):

* Lock solenoids on the 24 V rail, **default OFF** → springs drive the
  pins in.
* The **hardware e-stop drops the main contactor AND the solenoid
  rail** — removing power *engages* the locks. E-stop = power off =
  legs pinned with the rider supported (each joint settles ≤15° to the
  next lock hole). It must remove power, not command zero torque.
* Per-solenoid fail-safe drivers (default-open MOSFET with PWM
  economizer) so the controller can pin planted legs individually.
* Watchdog / brown-out on the controller de-energises the solenoid rail.
* **Controlled stop sequencing:** freeze gait → plant a valid tripod →
  align joints to lock holes → de-energise. Only an already-detected
  emergency skips straight to power-off, accepting the ≤15° settle.
* **Recommended addition:** normally-closed phase-short relays across
  each actuator's motor leads — de-energised, they short the phases and
  the 64:1 gearing turns that into strong speed-dependent damping, so a
  power-loss settle onto the pins is a slump, not a drop. Cheap
  insurance; validate on the single-joint rig first.

---

## 6. Hazards & commissioning

* 12S (≤50.4 V) sits under the 60 V LV line — a lesser shock hazard
  than v1's 84 V, but the **arc/fire energy of a 1.8 kWh pack is
  undiminished**: fuse at the pack, pre-charge, treat as live until
  metered dead, balance-charge in a fire-safe area.
* Commission on a current-limited bench supply, one actuator at a time;
  confirm every lock **holds at zero motor current** and **engages on
  power-off** before any weight goes on the legs
  ([`BOM.md` §L](BOM.md#l-bench-test-order-de-risk-before-buying-18)).
* Verify the 24 V rail under all-12-solenoids load *and* motor
  transients before first stand-up (voltage sag here = pins dropping
  mid-stride).

---

## 7. Assumptions & open questions

1. **0.9 kW cruise / 1.8 kW peak** are scaled from v1's estimates by
   mass and speed ratio (242/460 kg, 0.3/0.4 m/s); measure on the
   ballast machine and re-size the pack if reality is hungrier.
2. **Solenoid hold power ~12 W each** assumes a PWM economizer on each
   driver (full pull ~25 W is brief); without economizers, 12 × 25 W
   still fits the 350 W rail but with thin headroom — build the
   economizers.
3. **Branch fuse 30 A** assumes staggered peaks; if the gait
   controller ever commands 3 × 19 A on one leg, the branch opens —
   that is the intended protection, not a nuisance trip, but verify
   gait current profiles in ballast testing.
4. **CAN topology** (2–3 buses × 6–9 nodes at 1 Mbps classic CAN) needs
   a latency check at the 100–200 Hz control rate before committing the
   harness.
