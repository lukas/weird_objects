# Hexapod prototype_v1 — POWER-SYSTEM SPEC SHEET (corrected)

**Status: post-failure redesign of the servo power path.** This sheet
replaces the original "2 × 5 A BEC" scheme that melted wires and the
"20 A / 300 W adjustable buck at ~10 V" that overvolted the servos and
failed short. It is an **orderable** spec: concrete 2026 parts, real
ratings, rough prices, and links. It is a **docs/research artifact
only** — nothing here touches the running robot, the boards, or any
remote host. Measure everything on the bench before it goes near a servo.

Robot under spec: **18 × DS3225** (25 kg·cm, ~6 V hobby servos, 6 legs ×
3 joints), driven by **two PCA9685** boards (`0x40` / `0x41`, ~9 servos
each). Power: **3S LiPo (11.1 V nom, 12.6 V full, XT60)** → manual
anti-spark switch → step-down → PCA9685 V+. PWM control via the Arduino
(UNO Q); web control app with an ARM/DISARM gate.

---

## 0. What failed and why (read this first)

| # | Symptom | Root cause | Fix in this sheet |
|---|---------|-----------|-------------------|
| 1 | **V+ wires MELTED** | Original 5 A BECs fed ~9 servos each. 9 × ~2.5 A stall ≈ **22 A** demanded through a 5 A device and thin wire. The wire, not the fuse (there was none), was the weakest link — so the wire melted. | Size the supply to **peak**, not average (§1, §3); silicone **12 AWG** main + rail feeds sized to their fuse (§5); **fuse every rail** so the fuse opens before copper melts (§4). |
| 2 | **Buck FAILED SHORT (now passes input through)** | A generic "20 A / 300 W" adjustable buck (XINGYHENG 6–40 V→1.2–36 V) is really **~15 A continuous**; run near peak with poor airflow it cooked, and a shorted high-side MOSFET now feeds ~12 V straight through regardless of the pot. | Use a **fixed/lockable 6 V regulator built for digital servos** (§3), sized so continuous draw is < 50 % of rating, with **cooling** (§7). Retire the generic pot-adjust buck for the servo rail. |
| 3 | **Overvoltage — run at ~9.8–10 V** | DS3225 operating range is **4.8–6.8 V**. ~10 V forces extra current and heat, cooks the motor/driver, and is well outside spec. Some servos are likely damaged. | **Fixed 6.0 V** rail (§2). **Measure output before connecting any servo.** Replace suspect servos (§10). |
| 4 | **SMOKE when standing up** | Standing from a folded pose loads **all 6 legs (12 hip+knee servos) simultaneously** near stall — a coordinated ~20–35 A surge (§1). At 10 V into an undersized 5 A BEC and a thin V+ wire, that surge is where the heat/melt/smoke happened. | Budget for the **simultaneity peak** (§1); real headroom + cooling (§3, §7); **hardware cutoff** (§8) and **overcurrent auto-disarm** (§9) so a runaway surge is interrupted, not endured. |
| 5 | **Burned PCA9685 V+ path (9.8 V in, ~0.5 V at header)** | All servo current for a board was pushed through **one screw terminal and the PCB copper** to 9 headers. That copper can't carry ~22 A; it burned open (big volt-drop = burned trace). | **Do not run all rail current through one PCA screw terminal.** Distribute V+ from a **bus / fused distro** and inject at multiple points; keep **≤ ~6 loaded servos / ~15 A per board** (§6). |

**Bottom line:** the servos are 6 V parts, they were fed ~10 V through
5 A regulators and un-fused thin wire, and the standing-up move asks for
tens of amps all at once. Everything below sizes the path to that peak,
holds 6.0 V, fuses to the wire, and adds a real hardware e-stop.

---

## 1. Current budget — 18 × DS3225 at 6.0 V

DS3225 datasheet (Annimos/Banggood, 2026) electrical spec:

| Voltage | Idle current | Stall current | Stall torque |
|--------:|-------------:|--------------:|-------------:|
| 5.0 V | 4 mA | **2.1 A** | 24.5 kg·cm |
| 6.8 V | 5 mA | **2.9 A** | 28 kg·cm |

Interpolated **at 6.0 V, plan ~2.5 A stall per servo** (community
measurements land 2.1–2.9 A depending on batch; 2.5 A is the defensible
planning number).

| Operating state | What's happening | Per-servo | **Total (18)** |
|-----------------|------------------|----------:|---------------:|
| **Quiescent / limp (DISARMED)** | PWM off, servos limp; only board logic | ~0 A torque | **~0.1–0.3 A** |
| **Idle hold, foot in air** | Armed, holding centre, unloaded | ~50–150 mA | **~1–3 A** |
| **Standing hold (static)** | 6 legs bearing body weight, no motion | ~0.3–0.8 A on ~12 loaded joints | **~4–8 A** |
| **Walking average (tripod gait)** | 3 legs support + 3 swing, cyclic | mixed | **~8–14 A** |
| **STAND-UP transient (the smoke)** | All 6 legs' hip+knee push at once, near stall, ~1 s | ~1.5–2.5 A × ~12 | **~20–35 A** |
| **Absolute worst case** | All 18 stall simultaneously (fault / jam) | 2.5 A × 18 | **~45 A** |

### Defensible design figures

- **Continuous design load: ~15–20 A.** (Walking + margin.)
- **Peak design load: 40 A** (round the 20–35 A stand-up surge up to the
  ~45 A all-stall bound; size hardware to survive this, fuse to trip on
  a *sustained* version of it).

### Why standing up caused the smoke (simultaneity)

In steady walking, only ~3 legs are loaded at a time and joints share
the work, so average current is modest. **Standing up from a folded
pose is the one move where every leg loads at once**: all 6 hips and 6
knees drive the body upward against full weight, each near stall, in the
same ~1 second. That coordinated ~20–35 A surge (worse at 10 V) hit a
5 A BEC and a thin, un-fused V+ wire — so the energy dumped into copper
as heat instead of tripping a fuse. **The fix is to (a) size for this
peak, (b) fuse below the wire's melt point, and (c) sequence the
stand-up in firmware so legs rise in two groups rather than all together
(halves the peak) — but the hardware must still tolerate the full peak.**

---

## 2. Servo rail voltage — FIXED 6.0 V

- **Target: 6.0 V regulated, measured, locked.** DS3225 is a **4.8–6.8 V**
  servo. 6.0 V is the sweet spot: full-ish torque, comfortably inside
  spec, and standard for hobby BECs.
- **Why ~9.8 V was destroying servos:** 10 V is ~1.5× over the 6.8 V
  ceiling. The motor/H-bridge sees more voltage → more current → more
  heat per the same stall; insulation, brushes, and the driver run far
  outside their design point and cook. Any DS3225 that saw 10 V for more
  than brief moments should be treated as suspect.
- **7.4 V (2S "HV")?** Only for **HV-rated** servos. The plain DS3225 is
  **not** HV-rated (there is a separate `DS3225PRO` 8.4 V variant — you
  do **not** have that). **Default 6.0 V. Do not go to 7.4 V** on these.
- **MEASURE OUTPUT BEFORE CONNECTING SERVOS.** Set the regulator, put a
  meter on its output, confirm **6.0 V ± 0.1 V under a light load**, and
  only then wire it to a PCA V+ terminal. This one step would have
  prevented the 10 V event. If a supply is adjustable, **set-and-lock**
  (Castle Link lock, or a dab of paint/threadlock on the pot) so it
  can't drift.

---

## 3. Step-down / supply — recommendation

Two architectures were on the table. Both are viable; the recommendation
below is driven by the failure history (cheap adjustable buck died
short at overvoltage).

### Option A — Multiple high-current 6 V BEC/UBECs, split across rails ✅ RECOMMENDED

Split the 18 servos across **2–3 regulators**, each purpose-built for
high-torque digital servos, each set to **6.0 V**. Redundant (one dying
doesn't drop the whole robot), each individually within spec, and none
is a no-name pot-adjust module.

**Recommended part:**
[**Castle Creations CC BEC PRO** (P/N 010-0004-01)](https://www.castlecreations.com/en/bec-voltage-regulator/cc-bec-pro-010-0004-01)
— 20 A peak, **~15 A continuous** at low input voltage, output
**adjustable 4.9–12.5 V and lockable via Castle Link**, 2–12S input. ~$55 ea.

- **Buy 3** → set all three to 6.0 V → each feeds ~6 servos on its own
  fused rail (3 × 15 A continuous = 45 A of continuous capacity, well
  above the 15–20 A design load and covering the ~40 A peak with margin).
- Minimum viable: **2 ×** CC BEC Pro (30 A continuous / 40 A peak) if you
  also sequence the stand-up in firmware to halve the surge.

**Budget alternative:** [Hobbywing UBEC 10A HV (3S–14S)](https://www.hobbywingdirect.com/collections/ubec)
(~$39) — use one per ~3–4 servos (5–6 units). Cheaper per unit but more
wiring and more devices to fail; the switchable output includes 6 V.

### Option B — One big 6 V DC-DC regulator (20–40 A)

A single high-current 6 V rail is simpler to wire but is a single point
of failure and needs serious cooling.

- **Do NOT** reuse a generic "20 A / 300 W, 6–40 V→1.2–36 V" adjustable
  buck (the XINGYHENG-class module). Multiple 2026 vendors of that exact
  module state **20 A max, 15 A continuous recommended, fan required** —
  i.e. it is *already* undersized for a 40 A peak, and its pot-adjust
  high-side is exactly what failed short at ~10 V.
- If you want one big rail, buy a **genuinely rated** 6 V supply sized so
  the ~40 A peak is < 50 % of its rating: e.g. an enclosed/industrial
  **6 V, 50 A** regulated PSU with its own fan, or a purpose-built RC
  regulator rated ≥ 40 A continuous at 6 V. Treat any hobby module's
  sticker rating as ~2× optimistic and derate hard.

### Recommendation

**Option A with 2–3 × Castle CC BEC Pro locked at 6.0 V.** Rationale:
purpose-built for exactly this (multiple high-torque digital servos),
**fixed & lockable** voltage (kills the overvoltage failure mode),
redundant, and sized so the servo rail never runs near a module's limit.
This is the "what buck should I get" answer: **get Castle CC BEC Pros,
not another adjustable 300 W buck.**

> Ratings assume cooling and airflow. Every module above is a *thermal*
> device: a "20 A" sticker means 20 A **with a heatsink and moving air**.
> Cheap modules are routinely over-rated ~2×. Size well above expected
> peak and give it airflow (§7).

---

## 4. Fusing — the fuse protects the WIRE, not the load

**Golden rule:** a fuse must open **before the wire it sits on melts**,
so the fuse rating must be **≤ the wire's ampacity** and sized ~125 % of
the rail's continuous current. If a fuse is bigger than its wire can
carry, the wire becomes the fuse (that's the melt you already saw).

### Main-lead fuse (at the battery)

- **30–40 A bolt-down MIDI/ANL fuse within ~15 cm of the LiPo +
  terminal**, on the 12 AWG main lead. The 12 V input side of the
  regulators carries roughly `6 V × 40 A / (12 V × ~0.9) ≈ 22 A` at peak,
  so a **30 A MIDI** (or 40 A) protects the main lead and the whole
  downstream system.
- Holder: [Recoil / Ampper Mini-ANL (MIDI) inline holder](https://www.amazon.com/s?k=MIDI+ANL+inline+fuse+holder+30A)
  or an [ANL fuse block](https://www.amazon.com/s?k=ANL+fuse+holder+block).
- **AIC note:** a 3S 2200 mAh LiPo can dump very high fault current. A
  quality MIDI/ANL is adequate at this pack size, but understand the
  concept — for large lithium banks you'd step up to **Class-T** (20 kA
  interrupt). Not required here; a good MIDI/ANL is fine for a 2.2 Ah 3S.

### Per-rail fuses (one per servo rail)

- Put a **blade (ATC/ATO) fuse on every rail**, sized to that rail's
  wire and current:

  | Servos on rail | Worst-case stall | Rail wire | **Rail fuse (ATC)** |
  |---------------:|-----------------:|-----------|--------------------:|
  | 3 | ~7.5 A | 16 AWG (~10–13 A) | **10 A** |
  | 6 | ~15 A | 14 AWG (~15–20 A) | **15 A** |

- Use a **fused distribution block** so each rail is individually fused
  from a common 6 V bus: e.g. [Blue Sea Systems ST-Blade fuse block
  (100 A block / 30 A per circuit)](https://www.amazon.com/s?k=Blue+Sea+ST+blade+fuse+block)
  (~$25). This is the clean way to fuse 3–6 rails and gives you blown-fuse
  LEDs.
- **Coordination:** rail fuse ≤ rail-wire ampacity **and** < main fuse,
  so a rail fault trips *that* rail's blade fuse, not the 30 A main.

**"What fuse should I get" answer:** **30 A MIDI/ANL** on the main lead
at the battery, plus **10–15 A ATC blade** fuses per rail in a Blue Sea
ST-Blade fused distribution block (blade value chosen to match each
rail's wire gauge above).

---

## 5. Wire gauge — silicone, sized to fuse/current

Use **fine-strand silicone** wire everywhere on the power path (flexes,
high strand count, high temp rating). Approximate ampacity (chassis /
bundled, derate in still air):

| Run | Current | **Gauge** | Notes |
|-----|--------:|----------:|-------|
| LiPo main lead (battery → main fuse → regulators) | ~15–22 A | **12 AWG** | ~30 A ampacity; matches XT60 pigtails. |
| Regulator output → 6 V bus | up to ~20 A/reg | **12–14 AWG** | Size to each regulator's continuous rating. |
| 6 V bus → per-rail feed (fused) | ≤ 15 A | **14 AWG** | Fused at 15 A ATC. |
| 6 V bus → small rail (3 servos) | ≤ 10 A | **16 AWG** | Fused at 10 A ATC. |
| Servo pigtails (stock DS3225) | ~2.5 A ea | (as supplied) | Individual servo, fine. |

**Why the melt happened (gauge view):** ~22 A was pushed through wire
rated for a fraction of that (and through a PCA screw terminal / PCB
trace). Copper heats as I²R; past its ampacity the insulation melts.
Fixing this is two parts: **(a) fat enough wire for the current, and
(b) a fuse below the wire's melt point** so a fault opens the circuit
first. Both, not one.

---

## 6. Connectors / distribution — don't neck everything through one terminal

- **Distribute 6 V from a bus, not from one PCA screw terminal.** The
  burned PCA9685 (9.8 V in, 0.5 V at the header) is the classic "all
  current through one terminal + PCB trace" failure. The PCA9685 V+ copper
  is not rated for ~22 A.
- **Recommended distribution:**
  - A small **positive + negative bus bar** (e.g. [12 V 100 A dual-stud
    bus bar / junction block](https://www.amazon.com/s?k=12v+100A+bus+bar+M6+stud))
    or the Blue Sea fused block from §4 as the 6 V distribution point.
  - **Inject V+ into BOTH PCA9685 boards** with adequate gauge (14 AWG),
    each board on **its own fused rail** — and keep **≤ ~6 loaded servos
    / ~15 A per board** so the board's V+ trace is never the bottleneck.
  - If you keep 9 servos on a board, **do not** feed all 9 through the
    single V+ screw terminal at high load — use a **servo power
    distribution board** (breaks V+/GND out to each channel from a bus,
    bypassing the PCA's thin trace), or split each board's servos onto
    two injection points. The PCA9685 then carries **signal only** for
    the heavily-loaded channels.
  - Keep **all grounds common**: LiPo −, every regulator −, both PCA V+
    GND, both PCA logic GND, Arduino GND, bus-bar −. Verify with a
    continuity beep (existing WIRING.md §1 rule).
- Connectors: **XT60** for the pack and main lead (12 AWG silicone
  pigtails); ring lugs on bus-bar / ANL studs; keep servo 3-pin headers
  for signal.

---

## 7. Cooling for the regulator(s)

- Every regulator's rating assumes a heatsink **and moving air**. On a
  closed hexapod body, still air = derating.
- **Castle CC BEC Pro:** mount to metal / airflow; it self-limits with
  input voltage (15 A @ 16 V in). Keep them spaced, not stacked, in the
  `bec_cradle` area with a path for air.
- **If you use a big DC-DC:** bolt it to an aluminium plate/heatsink and
  add a **small 5 V/12 V fan** blowing across the heatsink for any load
  above ~10 A. The generic 300 W buck's own docs say "fan required above
  ~10 A" — take that literally.
- Add a cheap **thermal check** during bring-up: after a 30–60 s stance
  hold, nothing on the power path should be too hot to hold a finger on.

---

## 8. Hardware power cutoff — a REAL e-stop (defaults OFF)

**The current software "disarm" only stops PWM.** In `web_drive.py` the
DISARM / E-STOP / Relax buttons all send firmware `X`, which cuts PWM so
the servos go **limp** — but the **servo V+ rail stays powered**. A
wiring fault, a shorted servo, or a stuck driver is *not* addressed by a
PWM stop. A real e-stop must **remove power** from the servo rail.

**Add a high-current cutoff on the servo V+ path that DEFAULTS OFF and is
enabled only when armed:**

- **Option 1 — automotive relay (simplest, recommended for v1):** a
  **30/40 A SPST automotive relay** (Bosch/TE/Song-Chuan style) on the
  **12 V input to the regulators** (lower current side, ~15–22 A → one
  relay handles it). Drive the coil from an **Arduino GPIO through a
  relay-driver module** (transistor + flyback diode; a
  [logic-level relay module](https://www.amazon.com/s?k=arduino+relay+module+30A+automotive)
  is fine). GPIO **LOW / floating = coil off = rail dead**; the pin must
  be actively driven HIGH to arm. ~$8.
- **Option 2 — high-side power MOSFET / e-switch (solid-state):** a
  **high-side load switch** on the 6 V rail — a P-channel MOSFET (or a
  purpose-built RC electronic power switch) with a logic input, sized ≥ 2×
  the peak (≥ 80 A device / RDS(on) < 5 mΩ, on a heatsink) so it barely
  warms at 40 A. Silent, fast, no contacts to wear. Needs a proper
  high-side gate drive (P-FET or gate-driver IC), so it's more work than
  the relay; prefer a ready-made module.
- **Keep the manual anti-spark switch** as the physical, always-available
  e-stop (the human hand-on-switch). The relay/MOSFET is the *automatic*
  cutoff the firmware can open.

**Arming logic:** the cutoff should be **normally-open / off at power-on
and reset**. Only when the operator ARMs (and firmware is healthy) does
the GPIO drive it closed. Any DISARM / E-STOP / watchdog timeout / brown-
out → GPIO releases → **rail power removed**, not just PWM stopped. This
makes the existing ARM/DISARM gate control *power*, not just signal.

---

## 9. Current sensing + firmware auto-cutoff (ties into ARM/DISARM)

Give the firmware eyes on the amps so it can **auto-DISARM and open the
§8 cutoff on overcurrent** before wires heat.

**Sensor options:**

| Sensor | Type | Where | Notes |
|--------|------|-------|-------|
| [**INA226**](https://www.ti.com/product/INA226) I²C | High-side shunt + bus-V, digital | On the **6 V rail** (or per-rail) | 0.1 % accurate, reports A/V/W over I²C; needs a shunt sized to ~40 A (use an external e.g. 2 mΩ shunt, not the tiny 0.1 Ω module shunt). **Address clash:** INA226 defaults to **0x40 — same as PCA9685.** Jumper A0/A1 to **0x44/0x45** (or put it on the Pi's I²C bus, not the Arduino/PCA bus). |
| [**ACS758**](https://www.allegromicro.com/en/products/sense/current-sensor-ics/fifty-to-two-hundred-amp-integrated-conductor-sensor-ics/acs758) hall | Analog, galvanically isolated | On the **12 V main** or **6 V rail** | ACS758-050B (40 mV/A) or 100B; primary conductor is isolated 100 µΩ, handles the full peak, reads on one Arduino ADC pin via an RC filter. Best for the fat main lead. ACS770 is the newer equivalent. |

**Recommendation:** **ACS758-050B on the main lead** for a rugged,
isolated whole-robot amp reading the Arduino can sample directly; add an
**INA226 (jumpered off 0x40)** later if you want per-rail digital detail.

**Wiring notes:**

- **INA226:** VCC→5 V (or 3.3 V), GND→common, SDA/SCL→I²C bus (Pi bus, or
  Arduino bus **with the address jumpered off 0x40**), shunt across
  VIN+/VIN− in-line on the rail, VBUS→VIN−. Optional ALERT pin → an
  Arduino interrupt for a hardware overcurrent latch.
- **ACS758:** primary current through its two big terminals (in-line on
  the lead), VCC→5 V, GND→common, VIOUT→Arduino analog pin through a
  ~1 kΩ + 0.1 µF RC filter. Zero-current output ≈ VCC/2; amps =
  (Vout − VCC/2) / sensitivity.

**Where it hooks into firmware (spec only — do NOT implement here):**

- In `firmware/prototype_servo_bridge/` (and `prototype_walk/`), the
  ARM/DISARM state gates PWM today (`ARM` command arms; `X` disarms/limp).
- Add a periodic current read in the main loop. If measured current
  **exceeds a threshold** (e.g. > 25 A for > ~200 ms, or > 40 A
  instantaneously), the firmware should: **(1) drive the §8 cutoff GPIO
  to OFF (remove rail power), and (2) enter the DISARM state** (same path
  `X` uses to stop PWM) and report the fault over serial to `web_drive.py`.
- This makes ARM/DISARM control **both** PWM **and** the hardware cutoff,
  and adds an automatic overcurrent trip on top of the manual e-stop.
- Keep the trip in **hardware-latched** form where possible (INA226 ALERT
  or a comparator on the ACS758 output driving the cutoff directly), so a
  hung firmware loop still trips.

---

## 10. Bring-up / commissioning checklist

Do this on a **current-limited bench supply first** (WIRING.md §0), never
straight to the LiPo.

1. **Set 6.0 V no-load.** Dial each regulator to **6.0 V ± 0.1 V** with a
   meter on the *output*, no servos attached. Lock the setting.
2. **Short-check every board.** With power OFF, meter **V+ ↔ GND on each
   PCA9685** — expect open / high resistance, **not** a short. A short
   here (or a servo that saw 10 V) means damaged hardware; fix before
   powering.
3. **Fuse everything.** Main 30 A MIDI/ANL at the battery; each rail's
   ATC blade in the fused distro. Confirm each fuse value ≤ its wire.
4. **Wire the hardware cutoff (§8) defaulting OFF.** Confirm the rail is
   dead until the Arduino GPIO actively arms it, and that DISARM/E-STOP
   opens it.
5. **Bring load up one leg at a time, on a clamp meter.** Power one leg,
   command a stance, watch amps on the clamp meter and temperatures.
   Expect a few amps per leg; nothing should approach a fuse's rating in
   normal motion. Repeat leg by leg.
6. **Sequence the stand-up.** Verify (or add) a two-group stand-up so all
   6 legs don't push at once — this halves the §1 peak. Hardware must
   still tolerate the full peak.
7. **Replace damaged servos.** Any DS3225 that saw ~10 V, buzzes, runs
   hot, or draws abnormally high idle current → replace (you keep spares).
8. **Thermal soak.** Hold a full stance 30–60 s; nothing on the power
   path should be too hot to touch. Then move to the LiPo (WIRING.md §5).

---

## Orderable summary (the short answer)

| Role | Buy | Qty | ~$ each |
|------|-----|----:|--------:|
| **6 V regulator (the "buck")** | **Castle Creations CC BEC PRO 010-0004-01**, set & locked to **6.0 V** | **2–3** | $55 |
| **Main fuse** | **30 A MIDI/ANL** + inline bolt-down holder, at the battery | 1 | $10 |
| **Per-rail fuses** | **Blue Sea ST-Blade fused distribution block** + **10–15 A ATC** blades | 1 + blades | $25 |
| **Distribution** | 6 V + / − **bus bar** (100 A dual-stud) | 1 | $12 |
| **Main / rail wire** | **12 AWG** silicone (main), **14/16 AWG** silicone (rails) | as needed | — |
| **Hardware e-stop** | **30/40 A automotive relay + Arduino relay-driver module** on the 12 V feed (defaults OFF) | 1 | $8 |
| **Current sensor** | **ACS758-050B** (main lead) [+ INA226 jumpered off 0x40 for per-rail] | 1 (+1) | $12 |

Keep the **manual anti-spark switch** as the physical e-stop. Skip the
generic 300 W adjustable buck for the servo rail entirely.
