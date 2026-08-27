# cnc_chorn_overhead — CNC aluminum C-clamps, legs-over-head workspace

CONCEPT BUILD (not production).  User, Aug 2026: "ok can you make a
version of this with metal CNCed C clamps?  I'd also like to make them
a little bigger so this hexapod can put its legs over its head."
"This" = `concepts/rigid_hip` (single tower-seated bottom 6805, 6 mm
top plate with dust roofs, hip axis at world z 39.65, femur up-limit
−47.5° because the femur hits the plate).

**Result (measured by this build's sweep): femur up-limit −47.5° →
−107.5° (17.5° past vertical), first plate contact −115.0° at every
tested yaw (0, ±20, ±35), all rigid-hip checks still green.**

## What "C-clamp" means here (repo evidence)

`cad_step_test/build_chorn_step.py` + `docs/CHORN_VARIANT.md` define the
"C-horn variant": a **C-shaped bracket that clamps the servo output
disc AND the opposite idler disc at the hip/knee joints**, replacing
the printed clevis/yoke arms (the original experiment assumed a
*bought* aluminum C-horn plus printed adapters and spacers).  This
concept keeps that meaning but **CNCs a custom C-clamp** instead of
buying one: reach grows from the experiment's 29.5 mm (axis → web
inner face) to 42.0 mm, the loose spacers become integral pads, and
the whole plan profile is shaped by a measured plate-contact map so the
femur can swing up over the body.

## Parts (one design serves BOTH hip and knee — 12 clamps/robot)

| part | material | mass | replaces |
|---|---|---|---|
| `chorn_clamp_cnc` | CNC 6061-T6 | 37.7 g | printed yoke arms of `femur_link` (hip end) / `tibia_knee_yoke` (knee) |
| `femur_ovh_body` | PETG | 43.6 g | `femur_link` (the knee block half; the hip half IS the clamp now) |
| `tibia_ovh_socket` | PETG | 14.0 g | `tibia_knee_yoke` tube socket half |
| `knee_clamp_cap_ovh` | PETG | 25.1 g | `knee_clamp_cap` (tail re-shaped + new retention) |

Per-leg delta vs rigid_hip: **+23.9 g** (158.1 vs 134.2 g),
**+143.6 g/robot**.  452 g of aluminum total (12 clamps).
Robot height unchanged (the clamp is z-slimmer than the printed yoke:
52.5 vs 56.3 mm across the joint).  Torque note: the extra knee-side
clamp mass (37.7 g at FEMUR_LENGTH ≈ 80 mm) adds ~0.03 N·m static hip
moment (~1.5 % of an STS3215's stall) — negligible; overhead poses put
the leg CG above the hip, so gravity torque *reverses* there and never
exceeds normal stance loads.

## The clamp (what the sweep forced it to look like)

Two 3 mm blades with integral Φ19 ring pads land ON the nominal disc
faces (span 38.04), joined by a 3.8 mm web whose **outer face is
DATUM A** — it mates the frozen production femur wall plane
(x = 58.3 in link coords), so every downstream printed part keeps its
production interface.

* **Swan-neck plan profile.** A 1 mm raster of plate/hatch occupancy
  over the blade plan (both blade z-slabs, yaw {0, ±20, ±35}, pitch
  −95…−122.5) showed the plate rim sweeps a diagonal band through the
  old aft-bottom box — the very region the old blade→web load path
  used.  The band is notched out (`NOTCH_PTS`, straight walls, ≥0.9 mm
  margin to every mapped contact cell, all wall corners R2) and the
  load path drops into a **deep aft corridor** (notch floor −19.4 down
  to y −24, ~4.5 × 3 mm per blade at the pinch) that crosses the
  forbidden annulus at a steeper trailing angle and rises into the web
  root.  Down-swing (+25…+35) mapped clear to y −30.
* **Web top bevel** tracks the −115.7° iso-contact line — the web is
  the next wall after the blades, at −117.5°.
* **Integral pads** replace the chorn experiment's loose spacers; the
  disc recesses take the production Φ collar + 0.5 clearance.
* Same production disc-bolt pattern: 4× M3 on Ø14 PCD per disc face
  (8/clamp), threading into the metal discs' factory-tapped M3 holes.

## Web joint (clamp → printed part), per joint

* 2× **M3×10 SHCS** through the printed wall into **M3-6H tapped**
  holes in the web (upper row — reachable through the open servo well).
* 2× **M3×12 CSK** from the metal side (90° csk sunk in the web's inner
  face) into **T-slot nylocs** in the printed part (lower row — lives
  under the cradle floor where SHCS heads can't be reached).

## Knee cap retention (the wedge ate the old boss)

The overhead wedge chamfer removes the old inboard side-screw boss, so
the variant cap hangs a Φ4×6 heat-set insert boss under the tongue
line, bridged by a riser + a strap riding 0.3 over the servo top; an
**M3×25 SHCS up-screw** comes from the cradle underside into the
insert.  The cap's hip-side crown is shaved (roof + two corner wedges,
`CAP_SHAVE_*`) — that crown was the −112.5° contact before the shave.

## Workspace (measured by `sweep_femur_envelope`)

| | production | rigid_hip | cnc_chorn_overhead |
|---|---|---|---|
| femur up-limit (safe, 7.5° grid margin) | −80° | −47.5° | **−107.5°** |
| femur down-limit | +30° | +30° | +30° (unchanged) |
| tibia spin checked | −30…+20 baked | same | −50…+40 clear |

First contact by stage: blades vs plate rim −115° (yaw ±35),
knee-cap crown vs hatch −115° (yaw 0, ±20), web bevel −117.5°,
own coxa/cap/servo stack −130°.  The −110° safe limit is this reach's
physics ceiling (web bevel); we land 2.5° under it.

## Z-stack (unchanged from rigid_hip)

Deck/case top 10.25 → race 10.75…17.75 → hip axis **39.65** → plate
74.05…80.05 (hatch above).  Nothing in this concept moves the stack;
the growth is all in-plane (clamp reach 42.0/45.8 mm vs the
experiment's 29.5).

## CNC quoting notes (chorn_clamp_cnc.step)

* Material 6061-T6, qty 12 + spares, ~37.7 g finished, stock
  ≈ 65 × 45 × 60 mm.
* 3-axis: blades, pads, disc bores, notch pocket and plan profile all
  machine along the disc axis (±z, one flip); the 4 web holes drill/tap
  from the side (x) in one extra fixture.  Internal corners: notch
  walls R2 (Ø4 tool), plan fillets R3, web/blade gussets R2.5 —
  nothing smaller than R2 anywhere.
* Datums: **A** = web outer face (mates the printed wall — flatness /
  perpendicularity to the disc axis matter most), **B** = pad faces
  (disc span 38.04), **C** = Φ8 horn centre bore.
* Critical: pad span 38.04 is the NOMINAL disc-face span.  The PETG
  yoke ran `YOKE_SEAT_INTERF` squeeze per side; **metal must not
  preload the servo horn** — quote ±0.05 and expect a bench-tune shim
  pass (see needs-bench-tuning in `design_spec.yaml`).
* Tapped M3-6H ×2 per clamp; Φ3.4 + 90° CSK ×2 sunk in the web inner
  face.
* **No press fits in the metal** by design: pads land on the disc
  faces, collar recesses run 0.5 clearance.  Every bench-tuned PETG
  interference value stays in PETG parts and is flagged
  needs-bench-tuning in the spec rather than translated to aluminum.

## BOM delta vs rigid_hip (per leg)

* − prints: `femur_link`, `tibia_knee_yoke`, `knee_clamp_cap`
* + CNC: 2× `chorn_clamp_cnc`
* + prints: `femur_ovh_body`, `tibia_ovh_socket`, `knee_clamp_cap_ovh`
* + hardware: 4× M3×10 SHCS, 4× M3×12 CSK, 4× M3 nyloc, 1× M3×25 SHCS,
  1× Φ4×6 heat-set insert
* tibia CF tube: **16.6 mm shorter** than production (new socket mouth
  sits outboard so the tube tip lands 0.3 off the metal web)
* disc bolts: same 4× M3 pattern per face — verify grip length at
  assembly (blade+pad stack is 7.0/7.5 mm vs the yoke's arm stack)

## Assembly order (one joint)

1. Bolt the clamp to the printed part first (2× SHCS into the taps,
   2× CSK into the T-slot nylocs) — DATUM A face-to-face.
2. Offer the assembly to the joint: pads onto both disc faces.
3. 4+4 disc bolts into the factory-tapped discs.
4. Knee only: drop the cap in, M3×25 up-screw from the cradle
   underside into the insert.

## Pipeline

```sh
# full run: BREP export (uv 3.12 subprocess) + checks + sweep + scene
uv run python concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py
# fast re-check without re-export / without the sweep
uv run python concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py \
    --skip-brep --skip-sweep
```

Outputs: `step/*.step` (CNC quoting deliverable + manifest + zip
bundle), `step/stl/` tessellations, `stl/` baked scene set,
`scene.json` + `preview.png` (BuildViz), `sweep_report.json`.
Geometry truth lives ONLY in `build_cnc_chorn_step.py` (constants
imported from `make_cnc_chorn_variant.py`); `_probe_envelope.py` is the
mapping tool that shaped the plan profiles.
