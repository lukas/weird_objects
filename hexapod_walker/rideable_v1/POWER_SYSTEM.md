# `rideable_v1` — Power System (72 V bus)

> Sizing and architecture for the electrical power system: a **72 V (20S)
> Li-ion** main pack feeding the twelve RMD-X15 hip/knee actuators
> directly, a **72→48 V** rail for the six RMD-X8 yaw actuators, and a
> **72→24 V** rail for logic, the contactor, and the **fail-safe
> brake-release coils**. All figures are consistent with
> [`README.md`](README.md), [`DRIVETRAIN.md`](DRIVETRAIN.md), and
> [`BOM.md`](BOM.md).
>
> **This is a docs/design artifact.** Measure everything on a bench (and
> commission per [`../prototype_v1/POWER_SYSTEM.md`](../prototype_v1/POWER_SYSTEM.md)-style
> discipline) before it goes near a rider.

---

## 1. Bus architecture

```
   [ 72 V (20S) Li-ion pack, ~3 kWh ]
        │  84 V full / 72 V nom / 60 V empty
        ├─ main fuse (Class-T / MIDI ~80 A) ─ pre-charge ─ MAIN CONTACTOR
        │
        ├──────────────► 72 V bus ──► 12 × RMD-X15-450  (hip-pitch + knee)   ◄─ the big loads
        │
        ├── 72→48 V DC-DC (~300 W) ──► 6 × RMD-X8-120   (hip-yaw)            ◄─ small load
        │
        └── 72→24 V DC-DC ──► logic + compute + contactor coil
                            └► 12 × brake-release coils (fail-safe, defaults OFF)
```

* **72 V main pack** runs the twelve RMD-X15-450s (the hip-pitch + knee
  joints). These are the power-hungry joints; putting them straight on the
  pack avoids a big lossy converter.
* **48 V rail** for the six RMD-X8-120 yaw actuators, from a small
  72→48 V DC-DC. Yaw only fights friction (~50–80 N·m), so this rail is
  light.
* **24 V rail** for logic, compute, the contactor coil, and — critically —
  the **brake-release coils**. Because the brakes are power-off-engaged,
  this rail is what *releases* them; losing it re-engages every brake.

---

## 2. Current / power budget

72 V nominal. Standing costs ~nothing because the **brakes** hold the
load, not the motors ([`DRIVETRAIN.md` §4](DRIVETRAIN.md#4-the-knee-brake-load-holding-architecture)).

| Operating state | What's happening | Bus power | Bus current (@72 V) |
|---|---|---:|---:|
| **Standing (braked)** | brakes hold; motors idle | ~50–100 W (logic/compute only) | ~1 A |
| **Walking, cruise** | tripod gait, 0.4 m/s, dynamic torque well under continuous | **~1.5 kW** | ~21 A |
| **Walking, peak** | worst stride phase / small incline | **~2.5 kW** | **~35 A** |
| **Stand-up transient** | lifting the chassis + rider from a low pose, several joints near peak briefly | ~4–5 kW (~1–2 s) | ~55–70 A |
| **Absolute fault bound** | multiple X15s at peak current simultaneously | — | size hardware ≥ ~80 A, fuse to trip on *sustained* overload |

### Design figures

* **Continuous design load: ~1.5–2.0 kW (~21–28 A).**
* **Peak walking: ~2.5 kW (~35 A).**
* **Size wiring/contactor/BMS to the ~55–70 A stand-up transient**, fuse
  the main lead at **~80 A** (above transient, below wire ampacity), and
  sequence stand-up in the controller so not all legs push at once.

---

## 3. Pack sizing

| Parameter | Value | Basis |
|---|---:|---|
| Chemistry / config | Li-ion, **20S** (~9–10P) | 72 V nominal |
| Voltage | 60 V empty / **72 V nom** / 84 V full | 20 × (3.0 / 3.6 / 4.2) V |
| Energy | **~3 kWh** | target 30–60 min duty |
| Capacity | **~42 Ah** | 3 kWh ÷ 72 V |
| Continuous discharge | ≥ **80 A** (BMS + cells) | covers peak + transient |
| Mass | **~15–30 kg** (use ~20 kg) | see [mass budget](README.md#6-mass-budget-280-kg-dry) |

### Run time

```
    cruise:  3 kWh / 1.5 kW  ≈  2.0 h   (theoretical, continuous walking)
    derated: usable ~80% DoD, higher-power maneuvers, standing gaps
             →  plan for  ~30–60 min of real walking per charge
```

Standing between moves is nearly free (brakes hold at zero current), so
real endurance depends almost entirely on *how much you actually walk*.

---

## 4. Fusing, contactor & wiring

Golden rule (same as the prototype's power spec): **the fuse protects the
wire, not the load** — fuse rating ≤ wire ampacity, ~125% of continuous.

| Item | Spec | Why |
|---|---|---|
| **Main fuse** | Class-T (preferred) or MIDI, **~80 A**, at the pack + | High interrupt rating for a lithium pack; sized above the stand-up transient, below the main-lead ampacity. |
| **Main lead** | 8 AWG silicone (or larger) | ~80 A path from pack to contactor/bus. |
| **Main contactor** | ≥ 100 A DC, with **pre-charge resistor + relay** | Soft-charges the actuator bus caps, then closes; opened by the e-stop. |
| **Branch wiring** | 12 AWG per actuator pair | Each RMD-X15 branch. |
| **Bus bar** | +/− distribution, don't neck all current through one terminal | Same lesson as the prototype's burned PCA trace. |
| **DC-DC 72→48 V** | ≥ ~300 W | six yaw actuators. |
| **DC-DC 72→24 V** | ~150 W | logic + contactor + brake coils. |

---

## 5. The fail-safe brake power interlock

This is the safety heart of the electrical system and ties directly to
[`DRIVETRAIN.md` §4.4](DRIVETRAIN.md#44-fail-safe-behaviour).

* The **brake-release coils run off the 24 V rail** and **default OFF**.
  With no 24 V, or no controller enable, every brake is **spring-engaged**.
* The **hardware e-stop drops the main contactor AND the brake-release
  rail** — removing power *engages* the brakes. E-stop = power off =
  legs locked with the rider supported. It must **remove power**, not
  merely command zero motor torque.
* Each brake coil has its own **fail-safe driver** (relay/MOSFET that
  defaults open) so the controller can release brakes per-joint during a
  walk and re-engage them the instant a leg is planted.
* **Watchdog / brown-out** on the controller de-energises the brake rail →
  brakes engage. A hung controller cannot leave the machine free.

> **Sequencing caveat:** for a *controlled* stop, the supervisor must
> **freeze the gait, plant a valid tripod, then de-energise** — otherwise
> engaging brakes mid-swing drops the airborne corner. Only an
> already-detected emergency skips straight to power-off. See
> [`README.md` §9](README.md#9-safety--reality-check).

---

## 6. Hazards & commissioning

* **72 V DC is a shock and arc hazard.** Under charge the bus reaches
  **84 V**; under fault it can dump high current. Treat the bus as live
  until metered dead, fuse + contactor it, and add a bus-bleed indicator.
* **Pre-charge before closing the contactor** so the actuator bus
  capacitors don't draw a damaging inrush / weld the contacts.
* **Commission on a current-limited bench supply first**, one actuator at
  a time, watching amps and temperatures — never straight to the full
  pack. Confirm each brake **holds at zero motor current** and **engages
  on power-off** before any weight goes on the legs.
* **Charge the 20S pack on a proper balance charger** in a fire-safe area;
  lithium at this energy (~3 kWh) is a serious fire load.

---

## 7. Assumptions & open questions

1. **~1.5 kW cruise / ~2.5 kW peak** are estimates for 0.4 m/s on level,
   prepared ground; slopes and rougher terrain raise both.
2. **Stand-up transient (~55–70 A)** assumes an un-sequenced lift; a
   two-group stand-up roughly halves the peak. Size hardware to the
   un-sequenced peak regardless.
3. **~3 kWh / ~20 kg pack** balances endurance against mass; a bigger pack
   buys run time at the cost of the [mass budget](README.md#6-mass-budget-280-kg-dry).
4. **Brake coil voltage (24 V vs 48 V)** must match the brakes ordered in
   [`BOM.md`](BOM.md); the interlock logic is identical either way.
5. **CAN bus** carries actuator comms; keep it electrically isolated from
   the 72 V power path.
