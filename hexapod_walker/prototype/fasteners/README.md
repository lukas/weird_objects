# fasteners/ -- explicit fastener geometry

This directory holds the fastener CAD models that the build inspector
and the verifier consume.  Every fastener instance enumerated by
`fastener_registry.build_all_fastener_instances()` resolves through a
McMaster-Carr part number to one cache STL in this directory.

## How `make regen-fasteners` picks a source

For each unique McMaster part number, `make_fastener_meshes.py` looks
in priority order:

1. `fasteners/<part_number>.step` -- a real McMaster STEP download
   (highest fidelity).
2. `fasteners/<part_number>.stl`  -- a user-supplied STL.
3. `fasteners/_parametric.build_for_spec("<spec>")` -- the parametric
   fallback (visualization only, but dimensionally faithful at the
   head-d / cap-h / nut-AF level).

Whichever source wins gets exported to
`fasteners/<part_number>.cache.stl`.  At runtime the inspector +
verifier load ONLY the cache STL, so swapping a real STEP into place
later just requires re-running `make regen-fasteners`.

## Currently bundled cache STLs

| Part number | Spec | Used for |
|-------------|------|----------|
| 91290A115   | M3 x 14 SHCS, black-oxide steel | cradle servo-mount bolts |
| 91290A113   | M3 x 8  SHCS, black-oxide steel | link-to-X-horn bolts |
| 91290A123   | M3 x 30 SHCS, black-oxide steel (treated as M3 x 32 in the BOM -- swap to 91290A126 if you want exactly 35 mm) | coxa-bracket-to-chassis bolts |
| 92010A130   | M3 x 16 pan-head Phillips, A2 stainless | foot hinge pins |
| 91290A104   | M2.5 x 8 SHCS, black-oxide steel | servo spline center screws |
| 90576A102   | M3 nylon-insert lock nut (DIN 985), A2 stainless | every captive nut (cradle, chassis, foot hinge) |

## Swap in a real McMaster STEP

McMaster's product pages are rendered client-side and require a
logged-in session for the CAD download endpoint -- a plain
`WebFetch` of `https://www.mcmaster.com/<PART>/` only returns the
top-level catalog chrome, not the CAD link.  The reliable path is
the manual download:

> 1. In a normal browser, visit `https://www.mcmaster.com/<PART_NUMBER>/`,
>    e.g. <https://www.mcmaster.com/91290A115/>.
> 2. Click **Product Detail**, scroll to the **Drawings and 3D Models**
>    panel.
> 3. Click **3-D STEP**.  (If you want STL instead, pick **3-D STL**.)
> 4. Save the file as `fasteners/91290A115.step` (or `.stl`).
> 5. Run `make -C hexapod_walker/prototype regen-fasteners` to
>    refresh the cache.
> 6. Run `make -C hexapod_walker/prototype check-cad-fast` to
>    revalidate.

The same five-step recipe applies to every part number in the table
above.  You can drop in real STEPs incrementally; the parametric
fallback covers any missing ones until you do.

## The parametric fallback

`_parametric.py` builds each spec from `trimesh.creation` primitives:

* `make_m3_shcs(length)` -- M3 cap head (Phi 5.5, h 3.0) with a 2.5 mm
  hex socket carved into the top, plus a Phi 3 shank of the requested
  length.
* `make_m25_shcs(length)` -- M2.5 cap head (Phi 4.5, h 2.5) with a 2 mm
  hex socket, plus a Phi 2.5 shank.
* `make_m3_pan_head(length)` -- M3 pan head (Phi 6.0, h 2.4) with a
  Phillips cross slot, plus a Phi 3 shank.
* `make_m3_nyloc_nut()` -- DIN 985 hex body (AF 5.5, h 2.4) + nylon
  insert ring (Phi 4.7, h 1.6) + Phi 3 through bore.

The parametric meshes are visualization-only -- they are NOT load-
bearing or thread-accurate.  The verifier probes them as bounding
cylinders / cones, so visual fidelity at inspector zoom is all that
matters for now.

## Why `91290A123` for the M3 x 32 chassis bolts

McMaster sells M3 SHCS in 1 mm length increments.  The exact stock
that bridges the 16 mm flange + 4 mm chassis plate + 4 mm nyloc nut
budget (= 24 mm engaged + 8 mm head exposure = ~32 mm bolt) is
`91290A124` (M3 x 35 mm; closer to the actual budget).  We use
`91290A123` (M3 x 30 mm) above because the fastener_registry treats
this entry as "the next-stocked length" and it is what is currently
installed in the build photos.  When the real bolt length is
finalised, edit `PN_M3X32_SHCS` and `length_mm` in `fastener_registry.py`
and re-run `make regen-fasteners`.
