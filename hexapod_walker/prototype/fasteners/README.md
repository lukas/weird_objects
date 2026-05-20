# fasteners/ -- explicit fastener geometry

This directory holds the fastener CAD models that the build inspector
and the verifier consume.  Every fastener instance enumerated by
`fastener_registry.build_all_fastener_instances()` resolves through a
McMaster-Carr part number to one cache STL in this directory.

## Default: NopSCADlib via OpenSCAD

`make regen-fasteners` calls the OpenSCAD CLI on each
`fasteners/scad/<spec_id>.scad` file (one per registry spec) and
exports an ISO-accurate STL.  NopSCADlib does the modelling --
ISO 4762 / DIN 912 socket-head caps, ISO 7045 / DIN 7985 pan heads,
DIN 985 nylon-insert lock nuts -- so the cached mesh has the
correct head diameter, cap height, hex socket, and nylon collar
without us having to maintain handwritten primitives.

Install the toolchain once per machine:

```bash
# macOS (Apple Silicon needs the cask; the formula's Intel-only):
brew install --cask openscad
xattr -r -d com.apple.quarantine /Applications/OpenSCAD-2021.01.app
which openscad           # /opt/homebrew/bin/openscad
openscad --version       # OpenSCAD version 2021.01

# Linux / others -- the formula or distro package both ship the CLI.

# NopSCADlib goes into the OpenSCAD user-library directory so
# include <NopSCADlib/...> resolves without -I flags.
OSCAD_LIB="$HOME/Documents/OpenSCAD/libraries"
mkdir -p "$OSCAD_LIB"
git clone --depth 1 https://github.com/nophead/NopSCADlib.git \
    "$OSCAD_LIB/NopSCADlib"
git -C "$OSCAD_LIB/NopSCADlib" rev-parse --short HEAD  # pin in CI
```

The on-disk layout `make regen-fasteners` produces:

```
fasteners/
  scad/
    m2x8_shcs.scad        # one .scad per registry SPEC_*
    m3x8_shcs.scad
    m3x32_shcs.scad
    m3x16_pan.scad
    m2p5x8_shcs.scad
    m3_nyloc_nut.scad
  91290A005.cache.stl     # one per registry part number
  91290A005.cache.source.txt  # breadcrumb: "openscad NopSCADlib c9baa0e"
  ...
```

The `.cache.source.txt` breadcrumb records which stage of the
priority chain (below) produced the cached STL.  Useful when
debugging "did I actually get a NopSCADlib mesh or did it silently
fall back to the parametric placeholder?".

## Source-priority chain

For each unique McMaster part number, `make_fastener_meshes.py`
looks in priority order:

1. `fasteners/<part_number>.step` -- a real McMaster STEP download
   (highest fidelity; requires `python-occ` so `trimesh.load()` can
   read STEP).
2. `fasteners/<part_number>.stl`  -- a user-supplied STL override.
3. `fasteners/scad/<spec_id>.scad` -- NopSCADlib geometry rendered
   via the OpenSCAD CLI.  **This is the new default in 2026 onward.**
4. `fasteners/_parametric.build_for_spec("<spec>")` -- the
   handwritten parametric fallback.  Used only if (a) OpenSCAD
   isn't installed, OR (b) the NopSCADlib render failed.
   Visualization only.

Whichever source wins gets exported to
`fasteners/<part_number>.cache.stl`.  At runtime the inspector +
verifier load ONLY the cache STL, so swapping a real STEP into place
later just requires re-running `make regen-fasteners`.

## Currently bundled cache STLs

| Part number | Spec | Used for |
|-------------|------|----------|
| 91290A005   | M2 x 8  SHCS, black-oxide steel | **link-to-X-horn bolts (72)** -- used as self-tappers into the plastic 4-arm X-horn's existing Phi ~ 2.0 mm M2-sized arm holes.  May 2026 fastener-spec fix: an earlier draft listed these as M3 x 8 SHCS but a real DS3225 / MG996R / DS3218 X-horn won't accept an M3 shank -- the arm holes are Phi ~ 2.0 mm untapped through holes intended for M2 self-tap.  See the "M2 thread-former upgrade" note below for the optional McMaster 99461A340 swap. |
| 91290A113   | M3 x 8  SHCS, black-oxide steel | **cradle servo self-tap mounts (72)** -- driven vertically through each servo ear into a Phi 2.5 mm printed self-tap pilot in the cradle shelf below.  Same SKU works for both single-leg sub-assemblies and full-build kits. |
| 91290A123   | M3 x 30 SHCS, black-oxide steel (treated as M3 x 32 in the BOM -- swap to 91290A126 if you want exactly 35 mm) | coxa-bracket-to-chassis bolts |
| 92010A130   | M3 x 16 pan-head Phillips, A2 stainless | foot hinge pins |
| 91290A104   | M2.5 x 8 SHCS, black-oxide steel | servo spline center screws |
| 90576A102   | M3 nylon-insert lock nut (DIN 985), A2 stainless | coxa-bracket-to-chassis through-bolts (24) + foot hinge pins (6).  No longer used in the cradle (Design C revert: cradle bolts self-tap into the printed shelf). |

## Cradle bolts are self-tappers, not separate part numbers

The 72 vertical SHCS that thread DOWN through each servo ear into the
Phi 2.5 mm pilot hole in the cradle shelf below are M3 x 8 stock
(`91290A113`); we do NOT list a separate "self-tap" SKU.  This is
deliberate:

* M3 SHCS shanks are nominally Phi 3.0 mm; the printed Phi 2.5 mm
  pilot is undersized by ~0.5 mm so the SHCS's first revolution
  cuts (technically forms) a clean thread in PLA / PA12.  This is
  the same trick that DS3225 / MG996R-class servos already ship
  with (the manufacturer's M3 mounting hardware is a plain SHCS,
  too -- they assume the user is mounting into a printed or
  injection-moulded plastic part with a slightly undersized hole).
* Visual rendering already matches: at inspector zoom an SHCS and
  a thread-forming screw look identical from the head end.

If you DO want to swap in real thread-forming screws (e.g. McMaster
97525A540 for M3 x 12 self-tap), the registry change is local:
edit `_emit_cradle_fasteners` in `fastener_registry.py` to point at
a new `PN_M3_SELFTAP` + `SPEC_M3_SELFTAP` pair and add a
`fasteners/scad/m3_selftap.scad` (pan-head NopSCADlib placeholder is
fine -- see `_orient_for_registry` in `make_fastener_meshes.py`).

## X-horn bolts are also self-tappers (May 2026 M3 -> M2 fastener-spec fix)

The 72 link-to-X-horn bolts use the same self-tap-into-existing-pilot
trick as the cradle bolts, except the pilot is **already drilled by
the X-horn's injection moulder**: every plastic 4-arm X-horn that
ships with a DS3225 / MG996R / DS3218-class hobby servo has 4 x
Phi ~ 2.0 mm UNTAPPED through-holes in its arms (intended for M2
self-tap, per the manufacturer's mounting style).  An M3 SHCS does
NOT fit through those holes -- this fix swaps the link-to-X-horn
bolt from M3 SHCS (`91290A113`) to M2 SHCS (`91290A005`) and shrinks
the link pad's clearance bore from Phi 3.2 mm to
`XHORN_BOLT_M2_SELFTAP_HOLE_OD = 2.2 mm`.  Self-tap behaviour comes
from the X-horn's pre-drilled hole; the plain M2 SHCS forms a clean
thread in the plastic arm on first install
(`XHORN_BOLT_THREAD_ENGAGEMENT_MM = 3 mm` of engagement, matching
the X-horn arm's ~ 3 mm thickness).  Bolt length is M2 x 8: pad
(3-4 mm) + arm thread (3 mm) + head clearance (~ 1 mm) = 7-8 mm.

The cradle bolts (M3 x 8, `91290A113`) are mechanically symmetric
but ORTHOGONAL to this fix -- they live in printed shelf material,
not in the plastic X-horn.  Both rows stay in the BOM.

### Optional M2 thread-forming upgrade

If you'd rather not rely on the X-horn's Phi ~ 2.0 mm hole producing
a clean thread on first install (e.g. you're building 6+ legs and
worry about stripping out a stock M2 SHCS thread on the second or
third dis-assembly), McMaster sells dedicated thread-forming
screws for plastic:

* `99461A340` -- **M2 x 8 thread-forming for plastic**, recommended
  pick.  Hex socket, similar head geometry to the plain M2 SHCS so
  the inspector's HEX_KEY (8 mm x 30 mm) driver envelope is
  unchanged.
* `99461A330` -- M2 x 6 thread-forming, drop in if you prefer to
  bottom out the bolt in the X-horn arm without poking through.

To swap, edit `PN_M2X8_SHCS` in `fastener_registry.py` to
`"99461A340"` and re-run `make regen-fasteners`.  No SCAD edit
needed: NopSCADlib's `M2_cap_screw` head model is dimensionally
identical to a thread-former at inspector zoom.

## Axis convention

`fastener_registry.FastenerInstance` reports each fastener with:

* `head_world_xyz` -- the centre of the head's outboard face (the
  underside of an SHCS cap that bears on the printed part, or the
  outboard face of a nyloc nut where one is used).
* `axis_world` -- a unit vector pointing FROM the head INTO the
  material (the direction the bolt is driven in; for a nut, INTO
  the wall that holds the nut pocket).

`inspect_build.py`'s `_axis_to_transform()` then builds a 4 x 4 that
maps the **mesh's local +Z axis** onto `axis_world` and puts the
mesh origin at `head_world_xyz`.  Every cached STL in this directory
must therefore obey:

* Origin = head mating face / nut outboard face.
* +Z = body axis pointing INTO the material.

`fasteners/_parametric.py` already builds its meshes in that frame
(see its top-of-file docstring).  NopSCADlib does NOT: its
`screw(type, length)` module puts the head at +Z and the shank at
-Z, and `nut(type, nyloc=true)` puts the steel body at z = [0, t]
with the nyloc collar above it.  `make_fastener_meshes.py` therefore
reflects the OpenSCAD-rendered mesh across the XY plane (negate Z,
invert face winding) before writing the cache, so the cached STL
matches the parametric convention exactly.  After this fix:

| Spec | Cached STL bounds (Z) |
|------|----------------------|
| M2 x 8  SHCS    | z in [-2.0,  +8.0] mm  (cap at z<0, shank at z>0) |
| M3 x 8  SHCS    | z in [-3.0,  +8.0] mm |
| M3 x 30 SHCS    | z in [-3.0, +30.0] mm |
| M3 x 16 pan     | z in [-2.0, +16.0] mm |
| M2.5 x 8 SHCS   | z in [-2.5,  +8.0] mm |
| M3 nyloc nut    | z in [-4.0,   0.0] mm  (body at z<0, free side at z=0) |

If you author a new `.scad` recipe under `scad/`, follow the
NopSCADlib convention there (it's the natural one for OpenSCAD);
`_orient_for_registry()` in `make_fastener_meshes.py` is the single
point of conversion.

## Override with a real STEP

McMaster's product pages are rendered client-side and require a
logged-in session for the CAD download endpoint -- a plain
`WebFetch` of `https://www.mcmaster.com/<PART>/` only returns the
top-level catalog chrome, not the CAD link.  The reliable path is
the manual download:

> 1. In a normal browser, visit `https://www.mcmaster.com/<PART_NUMBER>/`,
>    e.g. <https://www.mcmaster.com/91290A113/>.
> 2. Click **Product Detail**, scroll to the **Drawings and 3D Models**
>    panel.
> 3. Click **3-D STEP**.  (If you want STL instead, pick **3-D STL**.)
> 4. Save the file as `fasteners/91290A113.step` (or `.stl`).
> 5. Run `make -C hexapod_walker/prototype regen-fasteners` to
>    refresh the cache.  `make_fastener_meshes.py` will pick up the
>    STEP at priority 1 and the breadcrumb will read `step:91290A113.step`.
> 6. Run `make -C hexapod_walker/prototype check-cad-fast` to
>    revalidate.

The same recipe applies to every part number in the table above.
You can drop in real STEPs incrementally; the NopSCADlib renders
cover any missing ones until you do, and the parametric primitives
are the final safety net.

## The parametric fallback

`_parametric.py` builds each spec from `trimesh.creation` primitives:

* `make_m2_shcs(length)` -- M2 cap head (Phi 3.8, h 2.0) with a 1.5 mm
  hex socket carved into the top, plus a Phi 2 shank of the requested
  length.
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
matters.  We keep `_parametric.py` around exclusively as the
last-resort fallback for users who don't want to install OpenSCAD.

## Why `91290A123` for the M3 x 32 chassis bolts

McMaster sells M3 SHCS in 1 mm length increments.  The exact stock
that bridges the 16 mm flange + 4 mm chassis plate + 4 mm nyloc nut
budget (= 24 mm engaged + 8 mm head exposure = ~32 mm bolt) is
`91290A124` (M3 x 35 mm; closer to the actual budget).  We use
`91290A123` (M3 x 30 mm) above because the fastener_registry treats
this entry as "the next-stocked length" and it is what is currently
installed in the build photos.  When the real bolt length is
finalised, edit `PN_M3X32_SHCS` and `length_mm` in `fastener_registry.py`
**and** the length in `fasteners/scad/m3x32_shcs.scad`, then re-run
`make regen-fasteners`.

## Pinned NopSCADlib commit

The OpenSCAD renders that ship in this repo were generated against:

    NopSCADlib @ c9baa0ed0faa23e849141c3d8c6728545d6af910

Re-pin this SHA whenever you upgrade NopSCADlib; if the upstream
changes a screw/nut macro signature, the OpenSCAD render will
either error out at regen time (the priority chain then falls back
to the parametric stub and prints a loud `WARN`), or silently
change the geometry -- a quick `view-build` diff against the
previous screenshot will catch the latter.
