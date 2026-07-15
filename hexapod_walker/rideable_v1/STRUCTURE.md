# `rideable_v1` — Leg & Chassis Structure

> The legs are **welded 4130 chromoly steel space-frames** — triangulated
> truss structures, not solid beams and not carbon-fibre rod. This doc
> covers the load cases, why steel, the space-frame stress logic, and how
> to handle the stress concentrations that actually fail welded steel
> structures (joints, bolt holes, foot pads, welds). Loads trace back to
> the [design basis in `README.md` §2](README.md#2-design-basis-decided).

---

## 1. Load cases

Everything is sized to the **design foot load of 2.25 kN** (the 1.5 kN
nominal per-leg tripod load × 1.5 safety factor).

| Case | Foot load | Where it bites | Governing failure mode |
|---|---:|---|---|
| **Static stand** | 1.5 kN nominal / 2.25 kN design | vertical into the foot, held by the brakes | bending of leg members, weld yield |
| **Walk stride** | ~1.5 kN cyclic | swings direction each stride | **fatigue** at welds / notches |
| **Landing / step-down shock** | up to ~2.25 kN (~1.5 g on a planted foot) | impulsive, vertical + fore-aft | bending peak, weld crack initiation |
| **Foot side-load / stumble** | ~0.5 kN lateral | torsion + out-of-plane bending on the leg | buckling of a slender diagonal (checked, non-issue) |

**Headline bending load:** taking the design foot load through the
longest moment arm in the leg (foot to the knee/hip node, ~0.7 m):

```
    M_bend  ≈  2.25 kN × 0.70 m  ≈  1.6 kN·m
```

So each leg must carry a **~1.6 kN·m bending moment** at design load. How
that is reacted is the whole reason for the space-frame ([§3](#3-why-a-space-frame-the-stress-logic)).

---

## 2. Why 4130 steel, not carbon-fibre rod

Off-the-shelf carbon-fibre rod/tube is tempting (light, stiff) but wrong
for this job:

| | 4130 chromoly space-frame (chosen) | Stock CF rod/tube (rejected) |
|---|---|---|
| Concentrated joint loads | tough, ductile, yields before fracture | **brittle, notch-sensitive** — cracks at clamps/bolts |
| Impact / landing shock | absorbs, dents, warns | shatters or delaminates with little warning |
| Bolt holes & clamps | drill/ream/weld freely | drilling severs fibres → huge strength loss |
| Repair after a fall | weld/patch in a shop | scrap the part |
| Fatigue with a defined weld detail | well-characterised S-N data | matrix micro-cracking, hard to inspect |
| Cost / fabricability | cheap, TIG-weldable, forgiving | fittings + adhesives, precise, unforgiving |

A rideable machine that carries a human over a **fatigue-driven, impact-prone**
duty cycle wants a **ductile, weldable, damage-tolerant** material. 4130
chromoly is the classic choice (roll cages, aircraft fuselage trusses,
motorsport) precisely because it fails gracefully and takes concentrated
loads at welded/bolted nodes.

> **v2 weight-saving path:** a **CF-tube-with-bonded-aluminium-fittings**
> hybrid (tube in pure axial tension/compression, metal fittings taking
> the concentrated joint loads) is a legitimate future iteration to shave
> leg mass. It is deliberately **out of scope for v1** — the concentrated
> loads go into ductile steel first, and CF comes later once the geometry
> and loads are proven.

---

## 3. Why a space-frame — the stress logic

A single tube trying to carry 1.6 kN·m in **pure bending** would be badly
overstressed. Take a representative 38 mm OD × 2.4 mm wall 4130 tube:

```
    I  = (π/64)(D⁴ − d⁴) = (π/64)(38⁴ − 33.2⁴)  ≈  4.27×10⁴ mm⁴
    Z  = I / c = 4.27×10⁴ / 19  ≈  2,250 mm³
    σ  = M / Z = 1.6×10⁶ N·mm / 2,250 mm³  ≈  710 MPa   ✗  (> ~435 MPa yield)
```

That single member yields. A **space-frame** instead resolves the bending
moment into a **couple of axial forces** in parallel chords separated by a
frame depth `h`. With `h ≈ 150 mm`:

```
    F_chord  =  M / h  =  1.6×10⁶ N·mm / 150 mm  ≈  10.7 kN   (tension in one chord,
                                                               compression in the other)
    A_tube   =  (π/4)(38² − 33.2²)  ≈  268 mm²
    σ_axial  =  F_chord / A  =  10,700 / 268  ≈  40 MPa        ✓  (vs 435 MPa yield → SF ~11)
```

Turning bending into axial load in triangulated members drops the working
stress from ~710 MPa (fail) to ~40 MPa (huge margin). That is why the legs
are trusses, and it also keeps the **weld stress ranges low enough for
effectively infinite fatigue life** ([§5](#5-fatigue-is-the-design-driver)).

**Diagonals carry the shear.** The triangulating diagonals turn the
transverse (shear) load into axial force too; keep every panel triangulated
so no member is asked to bend.

---

## 4. Buckling is a non-issue

The compression chord (10.7 kN) is short. Euler critical load for a
pinned 38×2.4 tube, `L ≈ 0.4 m`:

```
    P_cr = π²EI / (KL)²
         = π² × 200,000 MPa × 4.27×10⁴ mm⁴ / (1.0 × 400 mm)²
         ≈  527,000 N  =  527 kN
    buckling SF = 527 kN / 10.7 kN  ≈  49×
```

Even the slender diagonals sit at double-digit buckling safety factors.
**Buckling does not govern** anywhere in the leg — bending-into-axial and
fatigue do. (Keep an eye on any single long, lightly-triangulated diagonal
during detailing, but nothing in the baseline layout is close.)

---

## 5. Fatigue is the design driver

The legs see a **fully-reversing-ish cyclic load every stride**. At a
~1.5 s stride and, say, ~100 h of ride life that is on the order of
**~2×10⁵ strides**, with multiple load reversals per stride — well into
the high-cycle regime where **fatigue, not static yield, sets the
allowable stress.**

* **Base 4130** has a high endurance limit (~half its UTS), but a **welded
  joint** knocks that down dramatically — a fillet weld toe is a stress
  raiser and behaves like a low fatigue-class detail (endurance strength
  order ~50–90 MPa stress *range* for typical welded steel details).
* The space-frame keeps **chord stress ~40 MPa** at design load (and the
  everyday cyclic range, at nominal rather than design load, ~27 MPa), so
  the stress *range* at the welds stays under the weld detail's
  endurance limit → **effectively infinite life** at the nodes.
* **Normalise / stress-relieve** the finished weldments to remove residual
  stress and refine the heat-affected zone.

> **Design rule:** keep the cyclic stress *range* at every weld toe below
> the endurance limit of its weld detail class (target < ~50 MPa range at
> the primary nodes). The truss geometry already does this; a real build
> must confirm it with FEA + a weld-detail fatigue check, because the
> welds — not the tubes — are where a steel leg cracks.

---

## 6. Stress-concentration handling (where it actually breaks)

Welded steel doesn't fail in the middle of a clean tube; it fails at
**geometric and metallurgical discontinuities**. Handle each:

### 6.1 Truss nodes (joints)

* **Never land a tube end onto the *side wall* of another tube** as the
  sole load path. Use **cut-and-fishmouthed tube-to-tube joints** with
  full-penetration or well-formed fillet welds, or gusset the node with
  a **laser-cut 4130 node plate** that spreads the load into all
  converging members.
* **Route the load through the node, not around it** — align member
  centrelines to intersect at a point so the joint carries axial load,
  not a prying moment.
* Grind weld toes smooth (or TIG-dress) at the highest-stress nodes to
  raise the local fatigue class.

### 6.2 Bolt holes (motor mounts, brake mounts, foot, chassis)

* A drilled hole is a ~3× local stress concentrator. Put holes **only in
  local doubler/boss plates**, not through a primary chord wall in a
  high-stress zone.
* **Ream, don't punch**, and **deburr both sides** — a torn/sharp hole
  edge is a crack starter.
* Use **generous edge distance** (≥ 2× hole diameter to any edge) and let
  the bolted joint clamp a machined boss, so the tube wall isn't the
  bearing surface.
* Grade-10.9 fasteners in shear/bearing at the motor and brake mounts
  (see [`BOM.md`](BOM.md)); the brake-mount interface in particular reacts
  the full ~160 N·m fast-shaft holding torque into the leg frame and must
  be a machined boss, not a tapped tube wall.

### 6.3 Foot pads

* The foot takes the impulsive **landing shock** straight up the tibia.
  Mount the urethane pad + steel disc to the tibia through a **machined
  foot boss** welded into a triangulated corner of the tibia truss, so the
  impact spreads into three members, not one tube end.
* Keep the foot's steel disc tapped (bolts thread into steel), and put a
  **fillet radius** everywhere the boss meets the tube — no sharp
  re-entrant corners at the most-shocked joint on the machine.

### 6.4 Welds

* **Full continuous fillets** at load-path joints; no stitch/skip welds on
  primary members (skip welds leave built-in crack starters at every
  restart).
* **Normalise the weldment** after welding to relieve residual stress.
* **Dye-penetrant or MPI inspect** every primary weld before first ride,
  and re-inspect after any fall (mirrors the maintenance rule in the
  parent [`../ASSEMBLY.md`](../ASSEMBLY.md)).
* Avoid piling welds on top of each other at a node (multiple HAZ
  overlaps embrittle the steel); design the node so members meet with
  space to lay clean, separated beads.

---

## 7. Chassis

The chassis is a **welded aluminium or steel tube frame with a deck and
saddle** — a one-off, not a per-leg part.

* Same space-frame philosophy: triangulate so the deck/saddle loads and
  the six hip-yaw reaction torques feed into axial members.
* Carries the **six hip-yaw actuators** at the perimeter, the **battery
  and electronics** centrally/low for a low centre of mass, and the
  **saddle + footrests + handlebar** for the rider.
* Keep the rider's centre of mass **low and centred** so it stays well
  inside the tucked-stance support triangle at 0.4 m/s.
* Steel chassis: same 4130/mild-steel weld rules as the legs. Aluminium
  chassis (6061-T6): lighter but welding **halves the local strength in
  the HAZ** — size aluminium weld joints accordingly and prefer bolted
  machined nodes where practical.

---

## 8. Representative member schedule (starting point)

Not final — a starting geometry for FEA to refine.

| Member | Section | Role |
|---|---|---|
| Leg main chords | 38 mm OD × 2.4 mm wall 4130 | carry the bending-couple axial loads (~10.7 kN) |
| Leg diagonals | 25 mm OD × 1.6 mm wall 4130 | triangulate; carry shear as axial load |
| Node / boss plates | 3–5 mm 4130 sheet, laser-cut | spread joint loads, host bolt bosses |
| Foot boss | machined steel, welded into tibia corner | react landing shock |
| Brake / motor mounts | machined boss + 10.9 bolts | react holding torque into the frame |
| Pivot hubs & clevises | machined steel, welded to truss ends | host the joint pins + driven sprockets |

Budget **~10 kg per leg (~60 kg for all six)** including hubs, clevises,
and mount plates — the bare tube of a femur+tibia truss alone is ~7 kg
(≈ 3.3 m of 38×2.4 chord at 2.1 kg/m per leg plus webs), so an earlier
~3 kg/leg target was not achievable in steel at these lengths
([`README.md` mass budget](README.md#6-mass-budget-360-kg-dry)).

---

## 9. Assumptions & open questions

1. **Frame depth `h ≈ 150 mm`** sets the chord force (M/h). A shallower
   leg raises chord stress; keep the truss deep where the moment is
   largest.
2. **Bending moment arm ≈ 0.7 m** for the 1.6 kN·m figure bounds the
   tucked geometry (the solved pose gives 0.63 m hip-to-foot straight-line
   distance); final leg lengths (still open) move it.
3. **4130 normalised properties** (yield ~435 MPa, UTS ~670 MPa,
   E = 200 GPa) are nominal; use certified material data for the real
   build.
4. **Fatigue life** is asserted from keeping weld stress ranges below the
   detail endurance limit — this **must** be confirmed by FEA + a weld
   fatigue-class check, since welds (not tubes) govern.
5. **v1 is all-steel.** The CF-hybrid weight-saving path is v2 and is not
   analysed here.
