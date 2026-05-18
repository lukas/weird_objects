# Xometry upload bundle — hexapod prototype

This directory is a self-contained order package for the **tabletop
hobby-servo hexapod prototype** (see `../PROTOTYPE.md` for the full
build guide). All 10 unique STL parts here are 3D-printable; ordered at
the quantities listed in `manifest.csv` they make exactly **one
walking robot** (52 parts total).

> Don't have a Xometry account? You can also upload these STLs to
> Shapeways, JLCPCB 3D-printing, PCBWay, Hubs, Sculpteo, Treatstock —
> the geometry is process-agnostic. The recommendations below assume
> Xometry MJF PA12, which is the cheapest-per-part for this kind of
> small mechanical hardware in 2026.

---

## What's in this directory

```
xometry_upload/
├── README.md                  -- this file
├── manifest.csv               -- qty, material, color, finish, notes per file
├── chassis_top.stl            -- qty 1  (small 140 mm hex deck for battery
│                                          + electronics + optional arm)
├── chassis_bottom.stl         -- qty 1  (full 200 mm structural plate with
│                                          per-leg bracket cutouts and bolts)
├── battery_holder.stl         -- qty 1
├── electronics_tray.stl       -- qty 1
├── coxa_bracket.stl           -- qty 6
├── coxa_link.stl              -- qty 6
├── femur_link.stl             -- qty 6
├── tibia_link.stl             -- qty 6
├── foot_pad.stl               -- qty 6   (TPU 95A — different process!)
└── servo_horn_adapter.stl     -- qty 18
```

Every STL is **watertight, manifold, and pre-oriented for FDM** with
flat faces on the build plate and any hollow servo pocket opening
toward +Z. For Multi-Jet Fusion (MJF) the orientation is irrelevant
(no supports needed) but the orientation also looks right in MeshLab /
Cursor's STL preview.

---

## Recommended order (Xometry MJF PA12)

1.  Go to <https://www.xometry.com/instant-quoting-engine/>.
2.  Drag-and-drop **all 9 STLs except `foot_pad.stl`** into the quoter.
    (The foot pad needs a flexible TPU process, so put it on a
    *separate* quote — see below.)
3.  For the 9 rigid parts, select:
    *   **Process:** 3D Printing → Multi-Jet Fusion (MJF)
    *   **Material:** PA12 Nylon
    *   **Finish:** As-printed (or "Black dye" if you want black parts —
        adds ~$0.50 / cm³)
    *   **Tolerance:** Standard (±0.3 mm)
4.  Set the per-file quantities from `manifest.csv` (scroll down on each
    part card and edit the qty box). Total: 46 rigid parts.
5.  Choose lead time — 7-day economy is fine; 3-day rush ~ 1.5 ×
    cheaper per cm³ but ships sooner.
6.  Submit. Typical shipping is 7 – 14 days for the economy tier.

**Foot pads** (separate quote):

1.  New quote at <https://www.xometry.com/instant-quoting-engine/>.
2.  Drag in `foot_pad.stl`.
3.  Process: **3D Printing → FDM TPU 95A** (or "TPU 70A" if you want
    extra-grippy soles — Xometry's softer durometer flavour, but it
    sometimes has minimum-order constraints).
4.  Quantity: **6**.
5.  Submit.

> **Why split the order?** MJF and FDM-TPU are different machines with
> different setup fees. Putting them on one quote sometimes works but
> Xometry occasionally bumps the lead time of the entire order to the
> slower of the two. Two separate quotes ship independently.

---

## Approximate cost (mid-2026 USD)

| Bucket | Volume | MJF unit cost | Bundle cost |
|---|---|---|---|
| 1 × chassis_top (68 cm³) | 68 cm³ | $20 / part | $20 |
| 1 × chassis_bottom (100 cm³) | 100 cm³ | $26 / part | $26 |
| 1 × battery_holder | 14 cm³ | $9 | $9 |
| 1 × electronics_tray | 21 cm³ | $11 | $11 |
| 6 × coxa_bracket (20 cm³) | 123 cm³ | $11 | $66 |
| 6 × coxa_link (28 cm³) | 171 cm³ | $14 | $84 |
| 6 × femur_link (41 cm³) | 243 cm³ | $19 | $114 |
| 6 × tibia_link (18 cm³) | 105 cm³ | $11 | $66 |
| 18 × servo_horn_adapter (0.5 cm³) | 9 cm³ | $4 (min part) | $72 |
| **MJF subtotal** | **959 cm³** | — | **~ $490** |
| 6 × foot_pad (TPU 95A, 6 cm³) | 36 cm³ | $8 | $48 |
| **TPU subtotal** | | | **~ $48** |
| Standard shipping (US, 7-day) | | | $20 – $40 |
| **Total** | **~ 995 cm³** | | **~ $560 – $580** |

Compare with: **self-printing on a $200 Bambu A1 / Ender 3 = ~ $20 in
filament, 28 hours of printer time.** If you have access to a printer
this is the dominant cheaper option by a factor of ~ 30. Xometry pays
for itself only if (a) you don't have a printer, (b) you want better
dimensional tolerance and surface finish than FDM gives you, or (c)
you want a single white-glove order to a customer with no in-house
fabrication.

---

## Alternative materials you can choose on the Xometry page

| Process | Material | Why pick it | Cost vs MJF |
|---|---|---|---|
| **MJF** | PA12 nylon | Default. Strong, isotropic, no supports. | 1.0 × |
| **MJF** | PA12 GB (glass-bead-filled) | Higher stiffness for the load-bearing tibia and femur links if you load the prototype with > 2 kg. | 1.3 × |
| **SLS** | PA12 nylon | Functionally identical to MJF, slightly grainier surface. | 1.05 × |
| **FDM** | PLA / PETG / ABS | Cheapest commercial option, but layer-line surface and visible support marks. | 0.4 × |
| **SLA** | Tough resin | Great for the `servo_horn_adapter` only — high resolution on small features. The big parts will warp if printed in resin. | 0.8 × for small parts |
| **MJF** | TPU 90A | Alternative for the foot pad if FDM TPU isn't available. Softer than 95A, more grip, slightly lower abrasion life. | 1.2 × the FDM TPU price |

---

## If you want to modify a part before ordering

The STLs in this directory are *generated* — they're rebuilt from the
parametric source every time you run:

```bash
./run.sh hexapod_walker/prototype/prepare_xometry_upload.py
```

Edit the geometry constants at the top of `hexapod_prototype.py`
(servo body dimensions, link lengths, chassis size, etc.), re-run the
generator, then re-upload. The reorientation rules in
`prepare_xometry_upload.py` will pick up your changes automatically.

---

## Verification before you click "Place order"

The script prints, for each STL, its envelope and per-part volume.
Sanity-check those against the `manifest.csv` values; both should
match. If a part suddenly grows by 10 ×, you probably introduced a
boolean-difference bug in `hexapod_prototype.py`.

For the conservative builder: open `chassis_top.stl`,
`chassis_bottom.stl`, `coxa_bracket.stl`, `coxa_link.stl`,
`femur_link.stl`, and `tibia_link.stl` in your favourite STL viewer
(MeshLab, PrusaSlicer,
Cursor's built-in STL viewer) and confirm:

*   Every hollow servo pocket opens toward **+Z** (the top).
*   No part has a dimension greater than **220 mm** in any axis (so
    each part fits a hobbyist 220 × 220 mm printer if you decide to
    self-print).
*   The bolt-hole pattern on each link's hub face is a **20.8 mm PCD,
    4 × Φ 3.2 mm** (M3 clearance), with the 4 holes at
    **0 / 90 / 180 / 270°** so they line up with the X-shaped plastic
    servo horn's arms — match this against the `servo_horn_adapter.stl`
    to confirm interface compatibility.
