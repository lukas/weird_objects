# `rideable_v2` — Structure: Aluminum Legs & the Wide Chassis

> Why v2's legs can be **6061-T6 aluminum box sections** where v1 needed
> welded 4130 steel truss, the femur-as-belt-tendon load case, the
> chassis that buys stability with width instead of leg length, and the
> mechanical travel stops. Loads trace to
> [`README.md` §2](README.md#2-design-basis-decided): **design foot load
> 1.19 kN** (0.79 kN nominal × 1.5) — roughly *half* of v1's 2.25 kN,
> on legs roughly *half* as long. Bending moments scale with both, which
> is why the material class drops.

---

## 1. Load cases

| Case | Load | Where it goes |
|---|---|---|
| Planted leg, worst-in-stride | 1.19 kN vertical at the foot, 0.45 m horizontal arm to the hip | Femur bending (≤ ~535 N·m at the hip hub), tibia mostly axial |
| Tripod handover / bad park | ~half vehicle weight on one leg ≈ 1.19 kN | Same paths — the 1.5× SF *is* this case; don't double-count |
| Knee-belt tension | 2.93 kN peak working + pretension → ~5 kN shaft-pull bound along the femur | Femur axial compression + local pad bending at the tensioner |
| Misstep / 0.1 m drop onto one foot | impulsive, ~2–3× static | Urethane pad + belt compliance absorb; bearings and hard stops take the rest |
| Rider mount (one peg loaded) | ~0.8 kN at a deck edge | Chassis torsion |

---

## 2. Femur (350 mm, the bending-critical link)

**Section: 60 × 40 × 3 mm 6061-T6 box** (long axis in the load plane).

```
    Z ≈ 9,100 mm³   →   σ = 535 N·m / 9,100 mm³ ≈ 59 MPa
    6061-T6 yield ≈ 276 MPa   →   SF ≈ 4.7 on the DESIGN load
```

The generous static SF is deliberate: **6061 has no fatigue endurance
limit**, so the working stress is kept low (~40 MPa at nominal loads,
~10⁷-cycle territory) and the legs are inspection items — dye-check the
hub welds/bolted joints at every belt inspection. If fatigue findings
appear during ballast testing, the drop-in escalation is 4130 tube in
the same envelope (~+1.5 kg/leg), not a redesign.

The knee belt runs along this box: the ~5 kN shaft-pull bound
(2.93 kN peak working tension + installed pretension) appears as
axial compression between the two pulley axes (σ ≈ 9 MPa on the
~550 mm² section — negligible; a 350 mm column at this section does not
buckle) plus local bending at the eccentric tensioner pad, which gets a
doubler plate. Belt guard bolts to the same face.

## 3. Tibia (450 mm, mostly axial)

**Section: Ø50 × 3 mm 6061-T6 round tube.** Near-vertical in stance
(the tucked pose puts the foot ~35 mm from under the knee), so it
carries ~1.19 kN axial with bending only from the worst-in-stride knee
arm (~0.185 m → ~220 N·m → σ ≈ 45 MPa, SF ≈ 6). Round tube because the
tibia sees load from all azimuths during yaw sweeps. The foot end
carries the urethane pad + load-cell boss ([`BOM.md`](BOM.md) §E).

## 4. Chassis (~880 mm wide × ~1.1 m long)

Welded 6061 tube ladder/perimeter frame + aluminum deck plate:

* The brief's trade — **wide body = stability without hip torque** — is
  realized here: the 440 mm yaw-ring radius contributes half of the
  860 mm foot stance radius, so the support polygon is bought with
  frame, not with moment arm. (Long sprawling legs would buy the same
  polygon at ~2× the hip torque.)
* Carries: 6 yaw actuator mounts at the perimeter, 6 hip-assembly
  clevises, battery + e-bay slung low and central, saddle post +
  footpegs + handlebar on the deck.
* Torsion case: one footpeg loaded during mount (~0.8 kN at the deck
  edge) — the perimeter frame gets one diagonal per bay.
* Target mass 20 kg incl. deck ([`README.md` §6](README.md#6-mass-budget-165-kg-dry)).

## 5. Mechanical travel stops (per the brief — kept, and sized)

Every joint gets **hard stops on the structural members** (not on the
belt side): urethane-faced steel tabs on the link hubs, engaging just
past the software limits in
[`design_spec.yaml`](design_spec.yaml) (`joint_limits_deg`). Sized for a
full-torque runaway into the stop — worst is the hip at 418 N·m, which
the tab reacts as ~3.4 kN at a 125 mm radius on the hub. A stop that
bends is acceptable once; a link that folds is not. Stops also define
the assembly/shipping pose so the machine cannot fold flat on a bench.

## 6. Assumptions & open questions

1. **Leg mass ~3.5 kg each** (box + tube + machined hubs + clevises) is
   a target with maybe ±0.5 kg honesty band; weigh the first leg
   against it — the mass budget's hip-margin note makes this
   load-bearing.
2. **Welded vs bolted hubs:** baseline is machined 6061 hub blocks
   bolted to the box/tube (no welds in the primary bending path,
   fatigue-friendlier, serviceable). Welding is allowed on the chassis,
   where stress is low.
3. **Hard-stop energy** for a full-speed (112 °/s) runaway at the hip is
   modest (~5 J at the link inertia) — the urethane facing handles it;
   verify once on the bench rig.
4. **First-pass FEA done** (gmsh + scikit-fem, `tools/fea_joint_shaft.py`
   and `tools/fea_leg_nodes.py`, renders in `full_robot_viz/fea_*.png`) on
   the three flagged nodes, using parametric solids of the draft geometry:
   femur hub ~89 MPa working → SF ~3.1 on 6061-T6 (0.97 mm hub deflection
   under belt + torque + foot); coxa clevis node ~85 MPa → SF ~3.3;
   joint shaft ~220 MPa → SF ~3.0 on 4140 HT. These retire the hand-calc
   worst cases, but re-run on the real machined geometry (fillets, bolt
   holes, keyways) before cutting metal — and nothing here covers fatigue.
5. **Single-shear joint shafts** (hip/knee): the clevis plate is on one
   side only — the pulley face must stay clear for the belt wrap and
   the parking pin — so the Ø25 shaft root carries ~0.55 kN·m of
   cantilever bending at the hip's 5 kN belt-pull bound. FEA
   (`tools/fea_joint_shaft.py`) puts the working stress at ~220 MPa with
   the foot load stacked → SF ~3.0 static on 4140 HT, better than the
   rigid-hub hand bound because the load actually enters over the hub
   clamp band. The same moment becomes a force couple across the 30205
   pair, which is why they mount **back-to-back** (see `PARTS.md` §3);
   the escalation remains a Ø30 shaft on 30206s in the same envelope.
