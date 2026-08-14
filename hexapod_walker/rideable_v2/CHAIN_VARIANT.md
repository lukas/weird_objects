# `rideable_v2` — Chain-Drive Variant (Carvera-manufacturable)

> Alternative drivetrain to the Poly Chain belt baseline in
> [`DRIVETRAIN.md`](DRIVETRAIN.md): **ANSI #40-2 duplex roller chain**
> with flat-plate sprockets sized so **every custom part is cuttable on
> a Makera Carvera** (360 × 240 × 140 mm envelope, 200 W spindle).
> Chain was v1's architecture and is already v2's documented fallback
> (`DRIVETRAIN.md` §9.1); this doc promotes it to a fully-specified
> variant. Numbers are pinned by `make check` alongside the belt
> baseline. The belt design remains the baseline; build whichever the
> shop situation favors.

**Why this exists:** the belt baseline's driven pulleys are its only
parts that need an outside machine shop — the hip 100T pulley
(Ø254.6 mm PD) exceeds a Carvera's 240 mm Y axis, and Gates does not
publish the Poly Chain GT groove profile, so self-cutting belt pulleys
means reverse-engineering a proprietary tooth on a machine at its
rigidity limit. ANSI roller-chain sprockets invert all of that: the
tooth form is a **published standard** (ASME B29.1), the sprockets are
**flat 2D through-cut plates**, and chain is famously tolerant of
imperfect sprockets (bicycle chainrings are hobby-machined aluminum
plates at a comparable tension class).

---

## 1. Drive selection

**ANSI #40-2 duplex** everywhere (12.7 mm pitch, 27.8 kN ultimate as a
duplex — the same chain v1 ran and v2's BOM names as the belt
fallback). One chain SKU, one master-link spare pool, one sprocket
plate thickness. Duplex is required at every joint: the yaw's 3.1 kN
peak already exceeds a single #40's slow-drive working allowable
(~2.3 kN at UTS/6).

Chain speed is trivially low (~0.13 m/s at rated motor speed), so this
is a slow-speed drive by every chain-rating standard: the limit is the
UTS/6 working-load rule, not fatigue or lubrication speed limits.

## 2. Geometry (all driven sprockets fit the Carvera envelope)

| Joint | Teeth | Ratio | Driver PD | Driven PD | Driven OD | Chain (even links) | C (center dist.) |
|---|---|---:|---:|---:|---:|---:|---:|
| Hip-pitch | **13T → 52T** | 4:1 | 53.1 mm | 210.3 mm | **217.6 mm** | 68 | ~211 mm |
| Knee | **14T → 42T** | 3:1 | 57.1 mm | 169.9 mm | 177.1 mm | 62 | ~208 mm |
| Hip-yaw | **18T → 36T** | 2:1 | 73.1 mm | 145.7 mm | 152.8 mm | 50 | ~141 mm |

* PD = p / sin(π/N), OD = p(0.6 + cot(π/N)), p = 12.7 mm — all three
  driven ODs clear the Carvera's 240 mm Y axis (hip by 22 mm).
* Ratios are exactly the baseline's, so every torque/speed/IK number in
  [`README.md` §2](README.md#2-design-basis-decided) carries over.
* Even link counts — no offset (cranked) links, which halve chain
  fatigue ratings.
* Center distances stay near the belt layout (hip ~211 vs 208, knee
  ~208 vs 212, yaw ~141 vs 136), so the motor mounts barely move; the
  knee chain runs down the femur exactly like the belt did, guard and
  all. Slack is taken by a spring idler (§6), not by sliding the motor.

## 3. Tension & margins

Chain efficiency is **0.95** (well-lubed roller chain, conservative)
vs the belt's assumed 0.87 — so joint torque *rises* for free:

| Joint | Peak τ (η=0.95) | Cont τ | Peak chain tension | vs UTS/6 = 4.63 kN | vs UTS (27.8 kN) |
|---|---:|---:|---:|---:|---:|
| Hip-pitch | **456 N·m** | 182 N·m | **4.34 kN** | 94% | SF 6.4 |
| Knee | 342 N·m | 137 N·m | 4.03 kN | 87% | SF 6.9 |
| Hip-yaw | 228 N·m | 91 N·m | 3.13 kN | 68% | SF 8.9 |

* Tension = joint torque ÷ driven pitch radius. The hip peak sits at
  **94% of the slow-drive working allowable** — legal but thin; it is a
  motor-limited transient (the continuous hip case is 1.7 kN, SF 16).
  If ballast testing dislikes it, #50-2 drops into +15% envelope.
* **Hip peak margin improves: 456/360 = 1.27×** (belt: 1.16×) — the
  efficiency gain relieves the design's pinch point.
* **Thermal improves:** hip continuous rating rises to 182 N·m, so the
  tripod-walk RMS of ~174 N·m is now *inside* continuous (~96% — still
  treat tripod as a maneuvering gait; ripple/wave cruise drops to ~74%).
* **No pretension.** A chain's slack side carries ~zero tension, so the
  joint-shaft pull is the tight side only: ~4.3 kN transient / ~1.7 kN
  continuous, *below* the belt's standing ~5 kN pretension+working
  bound the joints were FEA'd against (`tools/fea_joint_shaft.py`,
  `tools/fea_leg_nodes.py`) — every existing structural margin is
  conservative for this variant. The driver-side pilot bearing
  (6905-2RS, `PARTS.md` §2) stays: 4.3 kN transient still exceeds the
  AK80-64's 2.0 kN output bearing.

## 4. What gets worse (stated honestly)

1. **Lubrication + noise.** Chain needs periodic oiling (slow speed →
   brush-on every ride-month is fine) and is louder than Poly Chain.
2. **Backlash.** Slack side + tooth clearance ≈ ±0.3–0.7° at the joint
   even with the idler; the belt was near-zero. The controller feels it
   at torque reversals. The parking pins (±0.5° hole clearance) are
   unaffected.
3. **Shock isolation.** The belt's compliance protected the 64:1
   planetary from ground impulses; chain is stiff. Mitigate with the
   cush-drive ring (§6) and keep the urethane foot pad mandatory.
4. **Mass ≈ wash, slightly heavier.** #40-2 runs ~1.26 kg/m (~17 kg of
   chain fleet-wide vs ~6 kg of belt), partly offset by smaller/lighter
   driven sprockets and much lighter drivers. Budget-neutral within the
   ±5 kg honesty band — weigh leg #1 against the budget as always.
5. **Chain snap is a real failure mode again** — but unlike v1, the
   parking pin holds at the *joint*, so the chain is **not** in the
   safety-critical holding path. A snapped chain = a limp joint, not a
   dropped rider. The interlock argument survives unchanged.

## 5. The driven sprocket stack (the Carvera part)

Each hip/knee driven sprocket is a bolted stack, all plates cut flat:

```
   [tooth plate 52T, 6.35 mm 7075-T6]      ← ANSI B29.1 profile, through-cut
   [spacer ring, 8.0 mm 7075]              ← sets 14.38 mm duplex row spacing
   [tooth plate 52T, 6.35 mm 7075-T6]      ← doweled to plate 1 (4 × Ø6×20
                                              dowels, teeth clocked identical)
   [standoff ring, 5 mm 7075]              ← axial clearance: duplex side
                                              plates protrude ~2.3 mm past the
                                              tooth plate and the disc rim sits
                                              radially inside the chain envelope
   [lock/hub disc, Ø210 × 12 mm 7075]      ← hip/knee only: parking-pin hole
                                              ring at r = 90 mm (6 mm rim
                                              ligament) + hub bolt circle
```

The whole stack registers on a **Ø62 g6 boss** machined on the hub face
(chain runout is then set by the boss, not by M8 bolt clearance) and is
clamped by 6 × M8×50 class 12.9 into the tapped hub disc. Stack-up,
ligaments, pin travel, and chain-plane clearances are pinned numerically
by `tools/chain_assembly_check.py` (run by `make check`).

* 6.35 mm (1/4") plate is under the #40 max tooth width (~7.1 mm) —
  slightly narrow teeth are fine; the standard clearance rule applies.
* **The geometry files exist**: `tools/sprocket_profile.py` implements the
  full ANSI B29.1 tooth form (seating/working/topping curves) with
  numeric self-checks (roller seating, entry-articulation clearance, gap
  mouth vs roller). `tools/export_leg_test_prints.py` emits one-leg STLs
  for test printing (`test_prints_leg/`); `tools/export_leg_cnc.py` emits
  STEP solids + flat-plate DXFs and machining notes for an outside CNC
  order or the Carvera itself (`cnc_leg/`).
* The **lock disc replaces the belt pulley's web**: same Ø12.2 hardened
  ring bushings at r = 90 mm, same pin, solenoid, and 540 N·m rated hold
  as [`PARTS.md` §4](PARTS.md#4-parking-pin-locks-12). The disc overlaps
  the chain *radius* at every joint, but sits **axially inboard** behind
  the 5 mm standoff ring (2.7 mm running clearance to the duplex side
  plates) — the pin travels parallel to the joint axis into the disc,
  clear of the chain plane.
* **The lock guide is a SINGLE-CHEEK block, not a straddling clevis.**
  A two-cheek clevis cannot be installed: the sprocket stack + chain
  occupy the far side of the disc, and the Ø210 rim runs 15 mm past the
  pin ring (3D overlap check, 2026-08-14). The block sits on the hub
  side of the disc (~1 mm face gap), guides the Ø12 pin in a 36 mm bore,
  and the pin engages the bushing **10 mm blind** — total travel 11 mm
  inside the 12 mm solenoid stroke; pin bending at rated hold ~212 MPa
  on a hardened dowel (limit ~800). It must mount in the azimuth sector
  the femur never sweeps (~225° is free; mount opposite the femur's
  mid-range direction) — verified collision-free across the full hip
  range in 3D.
* **Material honesty:** 7075-T6 tooth plates are the bicycle-chainring
  precedent (aluminum rings live for years at ~1 kN sprint tensions;
  our *continuous* is 1.7 kN across 2 rows = 0.9 kN/row). Treat them as
  wear parts: inspect at every ballast milestone, and if hooking
  appears, re-cut in 1045 steel (slow on the Carvera but a one-time
  job) or waterjet them from a drawing you already have.

## 6. Cush drive + tensioner

* **Cush-drive ring: NOT viable as sketched — bolt the stack rigid.**
  Running the numbers killed it: 456 N·m peak through 6 Ø16 rubber
  bushings at r = 60 is ~6.6 MPa rubber shear vs ~1 MPa allowable —
  the bushings would tear in the first hard stop. Real motorcycle cush
  hubs use large rubber blocks with ~10× the elastomer area, which
  doesn't fit this stack. The Ø16 holes stay in the plates (they cost
  nothing and keep the option open for a properly-sized elastomer
  insert later), but the baseline is **rigid M8 clamping**; ground-shock
  isolation falls to the urethane foot pad (mandatory) and the chain's
  own slack-side compliance.
* **Tensioner:** one spring-loaded idler per stage on the slack side
  (stock #40 duplex 15T idler sprocket on a pivot arm), same eccentric
  mount points as the belt tensioner. Full-length chain guard on the
  femur run, as with the belt.

## 7. What to buy vs cut

| Part | Source | Notes |
|---|---|---|
| #40-2 chain, ~5.5 m + master links | Motion / Tsubaki / McMaster | Riveted, one spare loop per length |
| Driver sprockets 13T/14T/18T #40-2, steel | Motion / Martin / McMaster | Cheap catalog parts (~$20–40) — buy hardened, don't cut; bore + keyway on the Carvera |
| Idler sprockets 15T #40-2 + pivot hardware | Same | 18× |
| 7075-T6 plate: 6.35 mm + 8 mm + 12 mm | McMaster / Midwest | Tooth plates, spacers, lock discs |
| Driven tooth plates 52T/42T/36T ×2 per joint | **cut on Carvera** | ASME B29.1 profile from any sprocket generator |
| Spacer + standoff rings, lock/hub discs | **cut on Carvera** | Lock ring bushings pressed as in `PARTS.md` §4 |

Everything else in the machine (hubs, clevises, mounts, shafts-from-
stock, locks) was already Carvera-territory. **The outside machine shop
drops out of the build entirely** (the welded chassis remains a
fabrication job). Rough cost: chain stages land ~$130–170/joint vs the
belt stages' $250 budget — call it **$1.5–2k saved** plus the pulley
quote-cycle risk.

## 7b. Fastener schedule + assembly order (one joint)

Every fastened interface, sized (the docs previously said only "bolted"):

| Interface | Fastener | Qty/joint | Notes |
|---|---|---:|---|
| Sprocket stack → hub disc | M8×50 class 12.9 + hard washer | 6 | Tapped M8×14 in the hub disc; 25 N·m + threadlocker; stack registers on the Ø62 g6 hub boss |
| Tooth-plate clocking | Ø6×20 dowel ISO 2338 m6 | 4 | Through plates + spacer only |
| Lock-ring bushings | Ø12.2 ID hardened bushings, press | 24 | Ø18.00 −0.02/−0.04 bores, hip/knee discs only |
| Motor → mount plate | M5×16 12.9 (VERIFY vs AK80-64 STEP) | 8 | 9 N·m; pattern is still a placeholder |
| Mount plate → frame | M8×20 8.8 | 4 | Slotted holes not needed — idler takes slack |
| Lock block → lock tab | M8×25 8.8 | 4 | Tab is part of the coxa/femur weldment — 6 kN pin shear reaction |
| Solenoid → lock block | M4×10 | 2 | Plus reed-switch M3 pair |
| Femur hub plug → box beam | M8×55 8.8 through-bolt + nyloc | 4 | Cross-bolted through beam walls + retaining compound |
| Tibia hub → Ø50 tube | M8×65 through-bolt + nyloc | 2 | Cross-bolted, + retaining compound |
| Coxa clevis wing → crossbar | M10×30 10.9 | 4 | The FEA'd joint reaction path |
| Tensioner idler + pivot | M10 shoulder bolts + nylocs | 2 | Idler on 2× 6900-2RS if plain-bore |
| Chain guard | M5×10 + standoffs | ~4 | Full femur run |
| Chain closure | #40-2 master link | 1 | Riveted preferred at the hip |

Assembly order per joint (supersedes the belt-era "belt before lock"
sequence — a chain with a master link removes that constraint):

1. Press bushings into the lock disc; press bearings into the clevis
   (back-to-back); bolt sprocket stack to the hub boss (dowels first).
2. Fit hub + joint shaft into the clevis, preload, torque.
3. Bolt the lock block to its tab (azimuth opposite femur mid-range);
   verify pin spring-engage / solenoid-retract into the disc ring.
4. Mount the motor + pilot bearing; wrap the chain, close the master
   link, set the spring idler. The chain goes on LAST — no axial-slip
   constraint like the endless belt had.

## 8. Open questions / test gates

1. **Cut one 52T plate first** and run it against a stock steel 13T
   with a loaded chain before cutting seventeen more — tooth profile
   verification is one afternoon and de-risks the whole variant.
2. **7075 tooth wear rate** under 0.9 kN/row continuous: inspect at
   ballast milestones; steel re-cut is the escalation.
3. **Hip peak at 94% of UTS/6:** confirm on the bench rig at 456 N·m
   against a stop; escalation is #50-2 (+15% envelope, still fits).
4. **Chordal ripple** at the 13T hip driver is 2.9% — invisible at
   these joint speeds, but verify no lock-alignment hunting at park.
5. ~~Cush-drive keep/delete~~ **Decided: deleted** — 6.6 MPa rubber
   shear at peak vs ~1 MPa allowable (§6); stack bolts rigid.
6. The bench-rig order (`BOM.md` §L) becomes fully catalog + own-cut:
   1 × AK80-64 + stock 13T + one own-cut 52T stack + #40-2 loop +
   bearing set + pin lock ≈ **$1,150**, no machine shop in the loop.
