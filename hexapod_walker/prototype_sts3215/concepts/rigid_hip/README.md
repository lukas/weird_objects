# Rigid-hip variant — third 6805 above each hip, full-size top chassis

Concept (user idea, Aug 2026): the production yaw joint rides a 6805-2RS
pair in chassis_bottom's tower, but everything above it — coxa link, hip
servo — is a cantilever.  This variant closes the structure from the top:

* **`hip_clamp_cap_rigid`** (print 6): the stock hip clamp cap
  (`make_servo_clamp_cap`) grown UP along the yaw axis — a Φ29 pedestal
  (inner-race seat, same role as the yaw hub's flange) and a Φ25.15
  press boss that a **third 6805-2RS** presses onto.  The boss lands on
  solid flange-bar material, >9 mm clear of the cap's M3 counterbores.
  Knee caps stay stock.
* **`chassis_top_rigid`** (print 1): a second 200 mm flat-to-flat, 4 mm
  hex FRAME (same footprint/thickness as chassis_bottom's sheet).  Six
  Φ44 full-height bosses pocket the bearings' outer races from below at
  Φ37.15, each race retained by a complete 360° Φ34 shoulder.  A large
  hex chunk of the middle (128 mm across flats) is cut out for the
  service hatch; the 140 mm `chassis_top` is not used in this variant.
* **`top_hatch_rigid`** (print 1): a removable 4 mm hex lid over the
  frame opening — take it off and the whole interior (electronics,
  wiring, yaw-cap bolts, standoffs) is open.  Carries the standoff
  pattern (±31.1, ±31.1), the electronics-deck pattern (±24.75,
  ±24.75) and the Φ40 centre hole, all of which fell inside the
  opening.  The electronics deck mounts on the hatch, so the lid lifts
  out WITH the electronics (tethered by wiring) for bench work.
* **`corner_pillar`** (print 6): hollow columns at the six corner
  azimuths tying the frame to chassis_bottom at the RIM — the actual
  top↔bottom structural connection (see below).  Each stands in a
  vacated corner Wago tray and doubles as the boss its hatch perimeter
  screw threads into.

Load path: hip moment → cap boss → top bearing → top plate →
**six rim pillars** → chassis_bottom + the five other legs.  Each yaw
axis becomes simply-supported instead of cantilevered, holding the hip
servo rigidly.

## The bottom pair loses a bearing — net bearing count unchanged

The production yaw joint stacked TWO 6805s only to form a 7 mm moment
couple.  With the top bearing providing a ~67 mm couple, the **lower
yaw bearing is omitted** — and that changes no production part: the
lower race was already the "floating" one (press-fit retention only),
while the upper bearing is the located one — outer race housed in the
bolt-on yaw cap's own Φ37.15 bore under its Φ34 lip, inner race seated
against the hub uflange by the horn clamp preload.  Its pocket in
chassis_bottom simply stays empty.  Robot total stays **12 bearings**,
same as production (2 per leg: one at the yaw cap, one at the top
plate).

## Stack (world Z, chassis_bottom sheet mid-plane = 0)

| z (mm) | plane |
|---|---|
| 75.55 | hip cap outer face (stock) |
| 81.05 | pedestal top = inner-race seat |
| 81.55 | plate ring bottom (0.5 clearance over the race seat) |
| 88.05 | race top = Φ34 shoulder = sheet bottom |
| 92.05 | deck face |

Yaw axes sit exactly on the hex edge midpoints (apothem 100), so each
Φ44 boss bulges half-outboard — same as the bottom towers.

## Removable service hatch

The full plate buried everything the production robot had in the open,
so the frame's middle opens up (the structure lives at the rim where
the bearing rings are):

* **Opening**: hex, 128 mm across flats, flats facing the rings — a
  14 mm web stays at every bearing ring, and the opening reaches
  furthest between the legs where the frame is widest.
* **Lid**: 4 mm hex (136 across flats) sitting ON the deck face with a
  1.5 mm registration lip dropping just inside the opening (0.3 mm/side
  clearance — straight drop-in verified at build time).
* **Retention**: 6× M3×16 button-heads that pass through the lid AND
  the frame into the corner pillars' top plugs (Φ2.5 self-tap pilots,
  insert-ready: drill Φ4 × 6), PLUS the 4 chassis standoff screws,
  which now pass through the hatch into the standoffs.  Chassis-hang
  loads run standoffs → hatch → deck face → frame in pure compression;
  the screws only see rebound.
* **Interior access** = remove 10 screws (6 perimeter + 4 standoff)
  and lift the lid — the frame, bearings and legs are untouched.  The
  yaw-cap join bolts (Φ23.5 around each yaw axis) are reachable
  through the opening with a long driver at ~14° tilt; the disc-horn
  clamp screws under the rings still need the full plate-off.

## Six rim pillars — how the top actually connects to the bottom

Each hip moment reacts as a force couple: push at the bottom tower,
pull at the top ring (or vice versa), i.e. **shear plus torsion
between the two plates at the rim**.  The four central M3 standoffs
(slender steel columns at rho 44) would carry that in bending and give
much of the rigidity right back; in this design they are demoted to
hatch/electronics anchors only.  The tie is six printed pillars at the
corner azimuths (rho 81.6, between adjacent legs — the only rim
territory outside every swing envelope):

* **Column**: hollow, elliptical section 20 mm radial × 14 mm
  tangential (walls 4 / 2.8), z 2 → 87.95, ~19 g each in PETG.  The
  tangential waist keeps a **measured 2.9 mm clearance** to the coxa
  tail sweep of both flanking legs across the whole yaw range —
  verified clear even for a full 360° hand-spin of a disassembled leg.
* **Top**: stops 0.1 mm short of the frame sheet (the six bearing
  RACES define the plate plane — the screws pull the sheet down onto
  the pillar; sand/shim a proud pillar, never let it rock).  Two Φ2.5
  pilots in a 10 mm solid plug: the hatch perimeter screw (one M3×16
  clamps lid → frame → pillar) and a dedicated frame screw at rho 87
  so the frame stays clamped with the lid off.
* **Foot = the corner Wago tray**: chassis_bottom already grows a
  three-walled tray for a 5-way Wago at every corner flat.  With the
  top frame on, those Wagos are buried under solid deck (no lever
  access), so they **relocate inboard under the service hatch**; the
  vacated bay becomes the pillar's socket — the foot fills it with
  0.3 mm clearance to the 2.4 mm production walls, which act as the
  shear/registration key.  chassis_bottom is NOT reprinted.
* **Foot bolts**: three M3 through-bolts with nyloc nuts on the belly
  (two inside the bay, one on an inboard tab that sits under the open
  hatch so its driver comes straight down).  The foot is its own
  drill jig: sit the pillar in the tray, drill Φ3.4 through the 8 mm
  sheet+floor, bolt.  A one-time bench drill mod, verified against
  open belly at all three spots.

Why this shape: light (~115 g total for all six), cheap (pure print,
~$3 of filament + 24 screws), strong (six large-section columns at
2× the standoff radius — slender-column estimates put plate shear ~3×
and torsion ~6× the four-standoff baseline, before counting the keyed
sockets), and easy (each pillar drops into a socket that already
exists on the production part and takes 4 screws).

## Fits — all bench-tuned production constants, nothing new

| interface | Φ (mm) | source constant |
|---|---|---|
| boss → inner race | 25.15 (+0.15 press) | `YAW_HUB_BOSS_OD` |
| pocket → outer race | 37.15 (+0.15 firm slip) | `YAW_TOWER_BORE_OD` |
| race shoulder | 34 | `YAW_TOWER_SHOULDER_OD` |
| ring wall | 44 = 37 + 2×3.5 | `YAW_TOWER_WALL` |

Boss tip has a Φ24×0.8 stepped lead-in; pocket mouths have a 0.8 mm
lead-in ring.

## Measured workspace trade-off (from the build-time sweep)

The full-size plate caps the femur's UP-swing (production workspace
envelope was femur −80°…+30°):

* femur vs plate: first contact at **−52.5°** (identical at yaw −35…+35)
* femur vs the boss/bearing stack: first contact at **−60°**
* ⇒ **safe femur up-limit −47.5°** (1 grid step + 2.5° margin), baked
  into the scene's hip joint limits.

Walking around `STANCE_FEMUR_DEG = −25` keeps ~22° of up-headroom.
**Deep tucks (stand-up belly curl, self-righting) exceed this** — check
those trajectories against −47.5° before putting this variant on the
robot.  Sweep used the stance-relative tibia angle (+75°); knee-range
variation only moves parts that stay outboard/below the plate.

## BOM delta vs production

* 6805-2RS count unchanged (12): the 6 lower yaw bearings move up to
  become the 6 top-plate bearings
* 4× M3 standoff stacks, ~90 mm (bottom sheet top z≈2 → hatch underside
  92.05; e.g. 50+40 F-F, or M3 threaded rod in printed sleeves) + M3
  screws down through the hatch into the stack tops — **non-structural**
  in this variant (hatch/electronics anchors only)
* 6× M3×16 button-heads for the hatch perimeter (through lid + frame
  into the pillar plugs)
* 6× M3×10 for the dedicated frame→pillar screws
* 18× M3×12 + 18× M3 nyloc nuts for the pillar feet (belly side)
* the 140 mm `chassis_top` deck + its 20 mm standoffs are not used
* the 6 corner 5-way Wagos relocate inboard under the hatch (longer
  leg-power pigtails at the corners; wiring change only)
* recommended: 12× M3 heat-set inserts (McMaster 94459A130) for the hip
  cap pilots, +12 for the pillar-top pilots — see "Disassembly & service"

## Assembly order

1. Build the robot as production but install only ONE yaw bearing per
   leg (the upper/located one; leave the lower pocket empty), and use
   `hip_clamp_cap_rigid` in place of the stock hip cap — same 2× M3
   into the same cradle pilots.
2. Move the 6 corner Wagos inboard: each one slides ~30 mm along its
   own corner azimuth to **rho ≈ 46, long side tangential, levers up**
   (floor verified free at rho 38–54 × ±20 mm on all six corners),
   velcro'd to the sheet — directly under the hatch opening, so levers
   stay reachable from above with the lid off.  Pigtails need
   ~30–35 mm extra.  Then sit each `corner_pillar` in its vacated
   tray, drill the three Φ3.4 foot holes through the sheet using the
   foot as the jig, and bolt down with M3×12 + belly nylocs.
3. Press a 6805 onto each cap boss until it seats on the Φ29 pedestal.
4. Lower `chassis_top_rigid` straight down onto all six races (pockets
   are lead-in chamfered; descent path verified clear at build time —
   the pillars stop 0.1 mm short so the races seat first), press until
   the shoulders touch the race tops, then drive the 6 frame→pillar
   screws.
5. Drop `top_hatch_rigid` into the frame opening (lip registers), drive
   the 6 perimeter M3×16s into the pillar plugs and the 4 standoff
   screws.

## Disassembly & service — the cap is a captive bearing carrier

The 6805 is pressed onto the cap boss **once and never removed**: the
serviceable interface is the cap-to-coxa BOLTED joint, not any bearing
fit.  Both cap bolts are reachable with the plate installed — the
outboard one sits beyond the hex edge under open sky, and the plate
has six **Φ7 driver pass-through holes** directly above the inboard
bolts (legs at yaw 0, ±2°; sight the screw head through the hole).
Verified at build time: 1.74 mm web to the bearing ring, clear
line-of-sight for a Φ6.5 driver shaft down to the cap counterbore.

* **Any service** (hip servo swap, leg work, plate off): set legs to
  yaw 0, remove the 6 inboard cap screws through the access holes, the
  6 outboard screws, the 6 frame→pillar screws and the 4 standoff
  screws — 22 screws, all from above/outside, **zero force** — then
  lift the plate: all six caps and bearings come with it as one rigid
  subassembly, leaving every hip servo sitting open in its cradle (the
  pillars stay bolted to the bottom).
* **Reassembly**: either lower the plate+caps unit back over all six
  servos, or bolt the caps onto the servos first and lower the bare
  plate onto the races (the original assembly order; descent verified
  clear both ways).
* **Repeated-service threads**: the cap bolts are now every-service
  fasteners, so per the repo's insert rule, drill the two cradle
  pilots Φ4.0 × 6 deep and fit M3 heat-set inserts (94459A130) — a
  bench mod on the existing coxa_link print, no reprint.
* **Last resort only**: the pocket fit is the bench-proven "firm
  finger-press" class (never separated in normal service), and the
  pedestal keeps two puller notches exposing the inner race underside
  in case a bearing ever truly needs to come off a boss.

### Why the cap attachment stays the production 2-bolt clamshell

The moment path does not run through the bolts: radial bearing loads
enter the cap and bear on the cradle through the tongue's snug ±x fit
in the cavity, the top lip over the cradle plate, the seat-drop ledge,
and the back/horn hooks — plastic-on-plastic interlocks on three
sides.  The two M3s only clamp the stack shut, which is exactly what
makes them safe to use as the quick-disconnect.  Alternatives
considered and rejected: a separate bolt-on bearing mast (puts a
bolted joint in series with the moment path, same two bolts anyway,
loses stiffness), a quick-release pin (adds play exactly where this
variant is buying rigidity), and through-bolts from below (needs
through-drilling the production cradle walls and inserts in the 4 mm
cap flange).  With inserts in the pilots and driver access through
the plate, the 2-bolt attachment is both the stiffest and the most
serviceable option.

## Print notes

* `chassis_top_rigid`: deck face down.  Pockets then print as upward
  blind bores (clean press walls); the Φ34→37.15 shoulder is a short
  internal bridge, same as the bottom tower prints.
* `top_hatch_rigid`: lid face down, lip up — flat print, no supports.
* `corner_pillar`: foot down, column up — no supports (the bore
  ceiling is a short internal bridge under the top plug).
* `hip_clamp_cap_rigid`: rest on the tongue face (outer face + boss
  up) so the press boss prints as a vertical cylinder; supports under
  the flange wings and hooks.  Printing in the stock flat orientation
  puts support scars on the press band — coupon-check the fit if you
  do that.
* Reused parts in `stl/` (coxa_link, femur_link, …) are for the viewer;
  print those from the main `stl_prototype/`.  Only the three parts
  above are new.  `*_DO_NOT_PRINT.stl` are COTS visuals.

## Build & view

```sh
# from the repo venv
python concepts/rigid_hip/make_rigid_hip_variant.py            # full checks
python concepts/rigid_hip/make_rigid_hip_variant.py --skip-sweep  # fast iter

npx buildviz register hexapod_walker/prototype_sts3215/concepts/rigid_hip \
    --build-id sts3215-rigid-hip
# http://127.0.0.1:5183/?build=sts3215-rigid-hip
```

Checks run at build time: watertightness, seated-stack placement, full
360° yaw sweep vs the plate, straight-down plate descent over all six
bearings, pillar clearances (seated robot, ±45° operating yaw with
margin, and an informational full hand-spin scan) and the femur
pitch×yaw contact sweep (fails the build if the safe limit ever eats
into walking headroom at −45°).
