# `rideable_v2` — Actuators, Belt Stages & the Joint Load Path

> How 18 identical CubeMars AK80-64s drive the 18 joints through
> per-joint wide synchronous-belt stages, why the knee actuator lives at
> the **hip end of the femur**, why every joint gets its **own structural
> bearing**, and how the machine holds a rider at **zero motor current**
> — v1's fail-safe-brake *rule* kept, but implemented as **joint-side
> parking-pin locks** after the parts-spec pass showed COTS friction
> brakes at this torque are 4–8 kg each (§6). All numbers trace to
> [`README.md` §2](README.md#2-design-basis-decided) /
> [`design_spec.yaml`](design_spec.yaml) and are pinned by `make check`;
> orderable SKUs live in [`PARTS.md`](PARTS.md).

---

## 1. One actuator, three ratios

The design uses a **single actuator SKU** — the CubeMars AK80-64
(live-verified Aug 2026: 120 N·m peak / 48 N·m rated, 64:1 internal
planetary, 48 rpm rated / 75 rpm no-load @ 48 V, 19 A peak / 7 A rated,
850 g, 6–12S, $889.90) — and tunes each joint with the **belt-stage
ratio** instead of with actuator selection:

| Joint | Belt ratio | Teeth | Why this ratio |
|---|---:|---|---|
| Hip-pitch | **4:1** | 25T → 100T | Fights gravity through the largest moment arm — buys torque at the cost of speed. (25T, not 24T: stock Gates 8M sprockets are 22/25/26/28…T.) |
| Knee | **3:1** | 28T → 84T | Holding-dominated with a near-vertical tibia; needs less than the hip. |
| Hip-yaw | **2:1** | 36T → 72T | Doesn't fight gravity; preserves speed for stride generation. |

One SKU means one spare pool, one CAN driver stack, one thermal model —
and because ratio lives in the pulleys, **re-gearing a joint after
ballast testing is a ~$150 pulley swap, not a new actuator**. (v1
could not do this: its chain sprocket sets were the joint bearing
housing too.)

### Effective joint capability (belt η = 0.87, the brief's 85–90%)

| Joint | Peak τ | Continuous τ | Rated speed | No-load speed |
|---|---:|---:|---:|---:|
| Hip-pitch (4:1) | **418 N·m** | 167 N·m | 72 °/s | 112 °/s |
| Knee (3:1) | **313 N·m** | 125 N·m | 96 °/s | 150 °/s |
| Hip-yaw (2:1) | **209 N·m** | 84 N·m | 144 °/s | 225 °/s |

---

## 2. Layout: all three actuators at/near the body

```
        chassis rail (880 mm-wide body)
            │
   [yaw AK80-64] ──2:1 belt──► YAW SHAFT (tapered pair) ─ coxa (120 mm)
            │                                                        │
   [hip AK80-64] ──4:1 belt──► HIP SHAFT ◄─[hip parking pin]─────────┤
            (mounted on the coxa,   (paired tapered rollers,         │
             axis parallel to hip)   100T pulley bolted to femur;    │
                                     pin ring in the pulley web)     │
                                                                     │
   [knee AK80-64] ── mounted ~140 mm from the hip, on the femur ─────┤
            │        (opposite face from the hip pulley)             │
            └────── 3:1 belt runs ~212 mm down the femur ──────► KNEE SHAFT ◄─[knee parking pin]
                     (the belt IS the femur's tendon)           (84T pulley bolted
                                                                 to tibia hub)
                                                                     │
                                                            tibia (450 mm) → foot
```

**Why the knee motor moves inboard** (the brief's key layout idea, kept):
an actuator + small pulley + mount is ~1.4 kg. Hanging it at the knee,
350 mm from the hip axis, costs ~0.17 kg·m² of swing inertia per leg —
roughly **20–25% of the total leg swing inertia**. Moving it to the hip
end of the femur and running the 3:1 belt down the femur removes that
inertia, shortens the motor power/CAN cabling to almost nothing (no
service loop across two joints), and packs all six knee actuators where
the chassis can shield them.

Costs of the remote knee, stated honestly: the femur must carry belt
tension (~2.9 kN peak) as a compression/bending load between its two
pulley axes, the belt needs a tensioner + guard along the femur, and
knee joint stiffness now includes belt stretch. All three are handled in
[`STRUCTURE.md`](STRUCTURE.md) §3 and the tensioner spec in
[`BOM.md`](BOM.md) §B.

---

## 3. The belt stages

**Family: Gates Poly Chain GT Carbon 8MGT, 36 mm wide** (carbon tensile
cords — the high-tension, low-speed case is exactly what this belt line
is for; Gates markets it explicitly as a roller-chain replacement).
8 mm pitch keeps small pulleys in stock sizes, so all three single-stage
ratios are buildable — v1's 6:1 was not (it forced a Ø750 mm driven
pulley) and fell back to chain; at v2's ratios and half of v1's loads,
the belt architecture works. Full drive specs with belt part numbers and
center distances are in [`PARTS.md` §2](PARTS.md#2-belt-drives-18-stages).

| Joint | Teeth | Driven pulley PD | Belt (stock length) | Peak belt tension | Center distance |
|---|---|---:|---|---:|---:|
| Hip-pitch | 25T → 100T | Ø254.6 mm | 8MGT-960-36 | **3.28 kN** (at the 418 N·m motor-limited peak) | ~208 mm |
| Knee | 28T → 84T | Ø213.9 mm | 8MGT-896-36 | 2.93 kN | ~212 mm (down the femur) |
| Hip-yaw | 36T → 72T | Ø183.3 mm | 8MGT-720-36 | 2.28 kN | ~136 mm |

* **Tension check (catalog-anchored, no longer an assumption):** working
  tension = joint torque ÷ driven-pulley pitch radius. The worst case
  (hip, 3.28 kN) is transient and motor-limited; the *continuous* hip
  case is 167 N·m → 1.31 kN. The Gates Poly Chain GT Carbon design
  manual's own worked example runs a **12 mm** 8MGT at ~2.6 kN effective
  tension as a normal rated working point, and its Table 11 width
  constants scale the **36 mm** belt to ~3.0× that (**~7.7 kN class**) —
  so the hip peak carries **>2× margin**, which `make check` enforces.
  Have Gates application engineering sign off the final drive at order
  time ([`PARTS.md` §2](PARTS.md#2-belt-drives-18-stages)); fallbacks
  remain 5:1 at the hip or v1's duplex #40 chain.
* **Compared with v1's chain**: quieter, no lubrication, lighter, and
  the big aluminum driven pulley doubles as the joint's structural disc
  *and* the parking-lock ring (§6).
* **The driven pulley is bolted to the moving link's hub** (femur at the
  hip, tibia at the knee, coxa at the yaw), coaxial with the joint
  shaft. The actuator and its small pulley sit on the parent link.
* **Pretension is a structural load:** installed static tension per
  Gates' Formula 14 lands around 0.6× effective tension per span, so the
  joint shafts and femur see a **~5 kN shaft-pull bound** at the hip
  (pretension + working). The bearings (§4) and the femur section
  ([`STRUCTURE.md` §2](STRUCTURE.md#2-femur-350-mm-the-bending-critical-link))
  are sized for it.
* **The same pull acts on the DRIVER sprocket** — ~2.9 kN continuous at
  the hip, which would run the AK80-64's own output bearing (2.0 kN
  dynamic) at ~150% of rating and kill it. Every driver sprocket hub
  therefore rides an **outboard pilot bearing** (6905-2RS class, in the
  mount plate) that splits the belt pull with the motor bearing —
  [`PARTS.md` §2](PARTS.md#2-belt-drives-18-stages).

### Shock path

Ground impact arrives as foot force → tibia → joint shaft → **structural
bearings** (not the gearbox) → the belt sees it only as torque, and the
belt's compliance + the pulley's flex isolate the AK80-64's 64:1
planetary from impulsive loads. This is the brief's "shock loads are
less brutally transmitted into the gearbox" argument, and it is real —
but note the AK80-64 is *not* a QDD: 64:1 with 4.7 N·m backdrive torque
is effectively non-backdrivable, so the belt and the urethane foot pad
are the *only* compliance in the leg. Do not delete either.

---

## 4. The structural joint bearing (the load-path decision)

The brief's rule, adopted verbatim: **on a person-carrying robot, the
motor output bearing must not be the leg's structural bearing.**

```
    load path:   foot ─ tibia ─ JOINT SHAFT ─ paired structural bearings ─ link ─ chassis
    torque path: AK80-64 ─ small pulley ─ belt ─ driven pulley (bolted to link hub)
```

Per joint: a 25–30 mm steel shaft on **paired tapered-roller bearings**
(30205 at hip/knee, 32006 pair at yaw — SKUs in
[`PARTS.md` §3](PARTS.md#3-structural-joint-bearings-18-sets)), with the
driven pulley bolted to the moving link's hub so the belt only ever
carries torque. The pairs react both the foot loads and the **~5 kN
belt shaft-pull bound** (pretension + working tension). The AK80-64's
own output bearing (2.0 kN dynamic / 2.5 kN static basic rating) never
sees the foot load, the rider, or a misstep impact — it just turns a
25–36T pulley, and even that pulley's belt pull is shared with the
outboard pilot bearing (§3). 18 assemblies, ~1.0 kg and ~$140 each
([`BOM.md`](BOM.md) §C).

---

## 5. Torque & speed margins (against the design basis)

| Joint | Design load | Peak available | **Peak margin** | Continuous | Parked static |
|---|---:|---:|---:|---:|---:|
| Hip-pitch | **0.36 kN·m** (worst-in-stride) | 418 N·m | **~1.16×** | 167 N·m | ~237 N·m |
| Knee | 0.20 kN·m | 313 N·m | **~1.57×** | 125 N·m | ~28 N·m |
| Hip-yaw | 0.08 kN·m | 209 N·m | **~2.6×** | 84 N·m | ~0 |

* **The hip margin is thin (1.16×) and is the design's pinch point.**
  Mitigations, in order: (a) mass discipline — every 10 kg dry costs
  ~15 N·m ([`README.md` §6](README.md#6-mass-budget-165-kg-dry));
  (b) the controller keeps stride excursion ≤ ±0.15 m and the stance
  tucked at 0.30 m; (c) the **5:1 pulley swap** (522 N·m peak, 1.47×,
  −20% speed); (d) the **AKH70-48 hip swap** (§7).
* **Speed:** at 0.3 m/s tripod, stance legs sweep ~31 °/s *loaded*
  (well under every rated speed) and swing legs need ~94 °/s
  *unloaded* — inside the hip's 112 °/s no-load but without much
  margin. 0.4 m/s is a stretch goal that likely requires the knee to
  contribute more of the swing; treat it as a post-ballast experiment.
* **Thermal:** knee and yaw walking RMS are well under continuous, but
  the hip is not: in a **tripod** gait each stance hip averages ~247 N·m
  over the sweep at 50% duty → **RMS ~174 N·m ≈ 104% of the 167 N·m
  continuous rating**. Tripod is therefore a *maneuvering* gait
  (minutes, not hours); **sustained cruise is the ripple/wave gait** the
  design basis already names — 5 feet down cuts per-leg load to
  0.47 kN → hip RMS ~135 N·m (~81%), thermally sustainable. **Standing
  on motors is worst of all** — the parked hip (237 N·m) exceeds
  continuous outright. Hence §6, and `make check` pins both gait
  numbers.

---

## 6. Load holding: joint-side parking-pin locks (v1's rule, better hardware)

A rideable spends most of its life standing, and the parked hip load
alone would cook the motors:

```
    parked hip static  ≈ 0.79 kN × 0.30 m ≈ 237 N·m   >   167 N·m continuous
```

So v2 keeps v1's rule — **motors move the legs, something mechanical
holds the legs; the machine never stands on motor current** — but the
parts-spec pass killed the friction-brake implementation:

> **Reality check (Aug 2026):** the fast-shaft brake originally budgeted
> (~110 N·m holding, 0.65 kg, $250) **does not exist**. Real
> spring-applied brakes in that class — INTORQ BFK458-16/-18, Mayr
> ROBA-stop-M — are **4–8 kg and $300–600 each**. Twelve of them add
> ~50 kg, which alone eats the entire hip torque margin
> ([`README.md` §6](README.md#6-mass-budget-165-kg-dry)). See
> [`PARTS.md` §4](PARTS.md#4-parking-pin-locks-12) for the surveyed SKUs.

The replacement is simpler, lighter, and structurally better — a
**spring-applied, solenoid-retracted parking pin at each hip and knee**
(12 total, all mandatory; yaw doesn't fight gravity and gets none):

* **What:** a Ø12 hardened dowel pin in a double-shear clevis on the
  *parent* link, spring-driven into a **hardened hole ring machined into
  the driven pulley's web** at r = 90 mm (not 100: the knee's 84T tooth
  root sits at r≈102 and the ring bushings need rim wall under it).
  Power-off = pin extended = joint locked. A 24 V tubular solenoid
  (~25 W pull, ~12 W PWM-economized hold) retracts it for walking.
  ~0.35 kg and ~$80 per joint.
* **It holds at the JOINT, not the fast shaft** — pin shear at 90 mm
  radius gives a rated hold of 540 N·m (1.5× hip design) with ~10×
  actual shear margin. **The belt is no longer in the holding load
  path**: v1's chain-snap single-point-of-failure is designed out, and
  belt inspection drops from safety-critical to ordinary maintenance.
* **Engagement:** the controller parks by planting a tripod, aligning
  each joint to the nearest lock hole (holes every 15° of travel; it has
  encoders and motors — trivial), then dropping the pins and cutting
  motor current. Lock slop in the aligned park is the pin/hole clearance,
  ~±0.5° at the joint.
* **Power loss mid-motion:** the pin rides the pulley web and drops into
  the next hole within **≤15° of joint travel** — the leg settles
  slightly and locks. (Optionally, normally-closed phase-short relays
  give speed-dependent dynamic braking through the 64:1 reduction during
  that settle — see [`POWER_SYSTEM.md` §5](POWER_SYSTEM.md#5-the-fail-safe-lock-power-interlock).)
* **Fail-safe behaviour and the e-stop/rail interlock**: e-stop, blown
  fuse, dead battery, or watchdog timeout all drop the solenoid rail →
  all 12 pins extend → legs lock with the rider supported. Controlled
  stops sequence *freeze → plant tripod → align → lock*.

---

## 7. The AKH70-48 alternative

The brief asked for a live check on whether a newer integrated actuator
obsoletes the external reduction. Answer: **CubeMars AKH70-48 V1.0**
(shipped 03/2026): 222 N·m peak / 74 N·m rated, 48:1, 28 rpm rated /
35 no-load, 1.396 kg, dual 21-bit encoders (output-side!), dual-CAN
daisy-chain, **$698.90** — cheaper than the AK80-64 with ~2× the torque.

| Option | Hip peak | Hip cont | Hip speed (rated) | Δ$ (×18) | Δ mass (×18) |
|---|---:|---:|---:|---:|---:|
| AK80-64 @ 4:1 (baseline) | 418 N·m | 167 N·m | 72 °/s | — | — |
| AK80-64 @ 5:1 (pulley swap) | 522 | 209 | 58 | ~+$0 | ~+1 kg |
| AKH70-48 @ 3:1 (hip fallback) | 579 | 193 | 56 | −$3,438 | +9.8 kg |

It does **not** make the belt unnecessary — direct-driving the hip from
a 222 N·m unit would put the design load right at the actuator's peak
with no margin, no structural bearing decoupling, and no shock
isolation; every argument in §3–§4 still applies. But as the *motor
behind the belt* it is genuinely competitive, and its dual-CAN
daisy-chain would simplify an 18-node harness. Decision: **baseline
stays AK80-64** (single SKU, better speed, field-proven line); buy the
bench-rig actuator first ([`BOM.md` §L](BOM.md#l-bench-test-order-de-risk-before-buying-18))
and revisit at the full order if the hip margin looks short in testing.

---

## 8. Per-joint summary

| Joint (×6) | Actuator | Ratio (int × belt) | Peak τ | Cont τ | Rated speed | Design load | Parking lock |
|---|---|---|---:|---:|---:|---:|---|
| Hip-yaw | AK80-64 | 64:1 × 2:1 (36T→72T) | 209 N·m | 84 N·m | 144 °/s | 80 N·m | none |
| Hip-pitch | AK80-64 | 64:1 × 4:1 (25T→100T) | 418 N·m | 167 N·m | 72 °/s | 360 N·m | **joint-side pin, 540 N·m rated** |
| Knee | AK80-64 | 64:1 × 3:1 (28T→84T) | 313 N·m | 125 N·m | 96 °/s | 200 N·m | **joint-side pin, 540 N·m rated** |

Per leg: 3 × AK80-64 (all mounted at/near the body), 3 belt stages,
3 structural joint-bearing assemblies, 2 parking locks. Six legs → **18
actuators, 18 belt stages, 18 bearing assemblies, 12 locks.**

---

## 9. Assumptions & open questions

1. **Belt allowable (~7.7 kN class for 8MGT×36)** is catalog-anchored
   (Gates manual worked example × Table 11 width scaling), not a formal
   drive rating — have Gates application engineering sign off the three
   drives at order time. Fallbacks: 5:1 hip ratio or duplex #40 chain.
2. **η = 0.87** for every belt stage; a lossier stage shrinks all
   torque figures proportionally. Measure on the bench rig.
3. **Knee-belt routing** along the femur needs a real tensioner design
   (fixed-center + eccentric back-side idler) and a guard; belt stretch
   adds series compliance at the knee that the controller will feel.
4. **Lock-hole alignment**: the park sequence must align joints to the
   15° hole ring before dropping pins; a power-loss engage accepts up to
   15° of settle. If ballast testing shows 15° settle is too violent,
   double the hole count on the next pulley revision (pure machining).
5. **AK80-64 store price** may or may not include the driver-board
   option — confirm at order time (the spec assumes integrated driver,
   which the AK series normally has).
6. **Travel stops**: every joint gets mechanical hard stops sized for a
   full-torque runaway ([`STRUCTURE.md`](STRUCTURE.md) §5) — the
   software limits in `design_spec.yaml` are not a substitute.
7. **Single-shear joint shafts**: the hip/knee clevis plate sits on one
   side only (the belt wrap and the lock own the pulley face), so the
   Ø25 joint shaft root sees ~0.55 kN·m of cantilever bending at the
   hip's 5 kN pull bound. First-pass FEA (`tools/fea_joint_shaft.py`,
   belt + foot stacked) lands the working stress at ~220 MPa → SF ~3.0
   static on 4140 HT — the load enters over the hub clamp band, so the
   rigid-hub hand bound (SF ~1.8) was conservative. The moment also
   dictates the 30205 pair's mounting: **back-to-back** (O-arrangement)
   for effective spread; face-to-face would overload the rows
   ([`PARTS.md` §3](PARTS.md#3-structural-joint-bearings-18-sets)).
   Escalation stays Ø30 / 30206 in the same envelope
   ([`STRUCTURE.md` §6](STRUCTURE.md#6-assumptions--open-questions)).
