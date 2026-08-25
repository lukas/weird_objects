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
* **`corner_pillar`** (print 6): plain solid elliptical columns at the
  six corner azimuths tying the frame to chassis_bottom at the RIM —
  the actual top↔bottom structural connection (see below).  Each
  stands where a corner Wago tray used to be and doubles as the boss
  its hatch perimeter screw threads into.
* **`coxa_link_rigid`** (print 6): the production coxa with four
  variant edits — servo-cradle corners rounded to the 38.2 mm yaw
  envelope so the plain columns clear the swinging leg by 5 mm at
  every angle, a Φ29 seat ring down to the relocated bottom bearing,
  a small Φ38 dust brim hovering 0.5 mm above the bearing top, and
  the vertical hub column **shortened 4 mm** (Φ52.4 skirt + platform
  disc deleted, cradle dropped) so the hip servo sits just above the
  bearing (see below).
* **`chassis_bottom_rigid`** (print 1): the production chassis with the
  six square tower platforms trimmed to the tower's own Φ44 cylinder —
  one matching curve from belly to bearing pocket — each tower rim
  raised 3 mm to the race-top plane so the bearing is fully housed
  (the Φ44 column ends exactly at the bearing top — see below), the
  dead cap-bolt ear lugs shaved off, the 18 pillar-foot bolt holes
  printed in, and the six corner Wago tray wall sets deleted (dead
  geometry in this variant — see below).  Every functional surface —
  bearing pocket, seat, well walls, strap slots — is production
  geometry.
* **`centre_wago_block`** (print 1): the corner Wago trays are gone
  (pillars stand there), so the power tree consolidates — 4× 5-port
  221-415 (two per net, jumpered) in one press-fit block at the
  chassis centre under the open hatch, replacing the 6 corner +
  2 trunk nuts (see below).

Load path: hip moment → cap boss → top bearing → top plate →
**six rim pillars** → chassis_bottom + the five other legs.  Each yaw
axis becomes simply-supported instead of cantilevered, holding the hip
servo rigidly.

## One tower-seated bottom bearing — the yaw bearing cap is deleted

The production yaw joint stacked TWO 6805s only to form a 7 mm moment
couple; with the top bearing providing a ~65 mm couple, one bottom
bearing suffices.  Earlier revisions of this variant kept that single
race where production located it — in the bolt-on `yaw_bearing_cap`'s
own bore, over an EMPTY chassis pocket.  That kept 6 printed caps and
18 M3 join screws in the build just to hold a race and provide
retention the top plate now provides anyway.  Simplified (user,
Aug 24):

* The single race drops into **chassis_bottom's own open-top Φ37.15
  pocket** — the production LOWER-race position, so the seat
  (coxa-local z 0.5) and the straight drop-in path are print-proven
  (the `chassis_bottom_rigid` reprint changes only cosmetic corners and
  adds foot holes; the pocket is untouched production geometry).
* The coxa hub grows a **Φ29 seat ring** (the production uflange OD,
  bearing only on the Φ25…29 inner-race land) from the uflange down to
  the relocated race top — free, since the coxa is already a variant
  print.
* The **`yaw_bearing_cap` is deleted**: −6 prints, −18 M3×8 screws,
  and one less part in the race-to-race tolerance stack.  Its two jobs
  moved: *radial housing* → the tower press band, *axial retention* →
  split by direction.  Hanging loads run hub → seat ring → inner race
  → outer race → tower seat; standing loads run up through the TOP
  bearing into the plate shoulder.  Each bearing takes one direction;
  no lip needed.
* **The tower wraps the full race** (user, Aug 24 rev 2: *"the bearing
  should be just above the horn and the motor should sit just above
  that — feel free to redesign"*): the rim is raised 3 mm from the
  production split plane (a 4 mm wrap that left the race's top 3 mm
  proud) to the **race-top plane** — the Φ37.15 pocket now houses the
  full 7 mm outer-race width (+75% press area; production's 4/3 split
  between tower and cap was a cap-housing artifact, not an optimum)
  and the Φ44 column ends exactly at the bearing top.  An earlier
  revision instead covered the proud band with a Φ44 skirt+curtain
  hanging from the coxa, flush with the tower — deleted: it made
  tower + race + skirt read as one continuous 16 mm chassis column.
  What remains coxa-side is a **Φ38 × 2 mm dust brim** hovering 0.5 mm
  above the rim/race plane and stopping 3 mm inside the tower Φ44, so
  it reads as coxa.  It **touches nothing** — a contact wiper would
  add friction, squeak PETG-on-PETG, and wear; 0.5 mm rides out print
  tolerance and bearing play.  The brim fully roofs the 2RS seal and
  outer-race band, and the seal itself remains the real dirt barrier
  (grit now has one turn under the brim instead of the old curtain's
  three — an accepted trade for the clean stack).
* Dropping the race 7 mm also stretches the bearing couple: mid-plane
  to mid-plane goes from ~58 mm (cap-held) to ~65 mm.

Robot total stays **12 bearings**, same as production (2 per leg: one
in the bottom tower, one at the top plate).

## Rounded tower platforms — chassis_bottom becomes a variant print

Each production yaw tower stands on a square outboard platform that
juts past the hex edge; its corners poke 8.4 mm diagonally past the
Φ44 tower.  Rounding them (user, Aug 24) makes `chassis_bottom_rigid`
(see `chassis_corner_round.png` for before/after).  A first revision
used a bolt-on R10 corner radius, which stacked three different arcs
at each corner (platform arc, the shallower chord the same cut left on
the narrower belly skirt, and the tower crown right above) — rejected:
*the curves must all match*.  Now:

* **Corners trimmed to the tower's own cylinder** (r 22.02 about the
  yaw axis, 0.02 proud of the Φ44 wall so the cut never grazes it).
  In production the tower already bulges through the platform band —
  its circle crosses the ±21.25 side faces at x 100±5.9 — so the trim
  just continues that same curve around the end and down the belly
  skirt.  Every height shows one curve, the tower's, and the side-face
  junction is the crease production already had.  The bearing pocket,
  seat, well walls and rim are untouched production geometry
  (asserted on every rebuild).
* **Dead cap-bolt ears shaved — all three**: with the
  `yaw_bearing_cap` deleted, the three M3 ear lugs per tower had no
  job.  The outboard one sat exactly on the trimmed corner and is
  shaved flush to the tower cylinder; the tangential one poked 6.8 mm
  past the platform silhouette and is shaved flush to the rim-wall
  face; the inboard (az 210) one — a free-standing column reaching
  r 28 from the yaw axis and z 19.75 — is cut flush to the
  **servo-mount deck top** (z 10.25).  Its below-deck root stays
  merged with the well-mouth collar (cutting there risks gouging the
  collar, and it invisibly stiffens the deck).  Above the deck,
  nothing pokes past the tower cylinder at any azimuth: the towers
  read as six clean bare columns.
* **18 pillar-foot holes printed in** (same constants as the pillar
  feet, aligned by construction) — no bench drilling on a fresh build.
* **Corner Wago trays deleted** (user, Aug 24: "they dont make any
  sense anymore in this version"): the splices live in
  `centre_wago_block` now, so the six 2.4 mm tray wall sets are dead
  geometry — everything above the sheet top is cut away (the 1 mm
  embed band inside the sheet stays as interior material).  The pillar
  feet register on their three M3 bolts instead of the old wall key.
* **Tower rims raised 3 mm** to the race-top plane (world 22.75) —
  the full-wrap housing described in the bottom-bearing section; the
  raise ring is unioned after every variant cut so the ear shaves
  never nick it.
* Net: −17 cm³ (318 vs 335).  **On an existing stock chassis print**
  most of this is a bench mod instead: saw/sand the corners back to
  the tower barrel and shave the ear lugs, drill the foot holes using
  the pillar feet as jigs, and glue a printed Φ44/Φ37.15 × 3 mm rim
  collar onto each tower top (the one edit that adds material).  The
  stock tray walls can stay — the feet were sized to fit them with
  0.3 mm clearance, so they just become a bonus shear key.

### What still stands at the corner flats (verified, Aug 24)

After the tray delete the corner flats were re-probed (both the mesh
and the BREP-derived STL): **nothing wago-era survives** above the
sheet top anywhere on the plate.  The walls still visible next to each
pillar foot are NOT tray remnants — they are, per feature:

* the two **yaw-servo cradle shells** flanking every corner flat (the
  2.4 mm printed walls with the notched inboard ends; each one has a
  yaw servo inside it and carries the servo-mount deck) — production
  geometry, fully load-bearing;
* the **yaw_servo_retainer** corner pads (a separate bolt-on part, not
  chassis) that keep the servo from dropping out of its well;
* the variant's own **pillar foot bar + inboard tab**.

They read like tray walls because they are the same wall thickness and
a similar height, but every one of them holds a servo.  Do not cut
them.  (`corner_flat_owners.png` — a z=8 section of the corner flat
with per-part colours — shows the ownership.)

### The joint column: horn → bearing → coxa (redesigned Aug 24)

The user read the old column as "rising higher than one bearing for
no reason": the Φ44 curtain hanging from the coxa was flush with the
Φ44 tower, so tower + proud race band + skirt looked like one 16 mm
chassis column (10.25 → 26.25).  Redesigned to the minimal legible
stack — the bearing sits just above the horn, the coxa (carrying the
hip motor) sits just above the bearing, and the Φ44 column now ends
exactly at the bearing top:

| world z (mm) | old | new |
|---|---|---|
| 10.25 | servo-mount deck top | unchanged |
| 15.25 | disc-horn top = coxa mount plane | unchanged |
| 15.75 | race seat (0.5 over the horn — the spinning inner ring must clear the static horn and screws) | unchanged |
| 15.75–22.75 | 6805 race (7 mm); tower wraps 4 mm, top 3 mm proud | 6805 race, **fully housed** |
| 19.75 | tower rim (Φ44) | *(rim raised — no edge here)* |
| 20.25–23.75 | coxa Φ44 curtain, flush with the tower | *(deleted)* |
| 22.75 | race top, hidden behind the curtain | **tower rim = race top; Φ44 column ends here** |
| 23.25–26.25 | Φ44 brim roof | — |
| 23.25–25.25 | — | **Φ38 dust brim** (0.5 above rim/race, 3 mm inset) |
| 22.75 up | Φ29 seat ring → coxa hub → hip servo | unchanged (ring lands on the inner-race top) |

The remaining heights are pinned by hardware: horn stack 15.25 +
0.5 mm horn clearance + 7 mm bearing = 22.75; the column cannot come
down further without a thinner bearing.  `joint_column_annotated.png`
is the annotated section of the NEW stack — every height band, what
it is, why it is there (regenerate with
`make_joint_column_figure.py`).

### The coxa hub column: shortened 4 mm (redesigned Aug 24 rev 3)

The user read the band ~25–40 mm above the deck as "largely
unnecessary" — and the audit agreed.  In the production coxa that
band held three things: the Φ29 seat-ring/boss spine (structural,
but taller than needed), the **Φ52.4 dust-lip skirt + 6 mm platform
disc** (world ~29–39.25 — a dust guard for the production
UPPER-bearing position, which this variant vacated when the race
moved down into the tower; the Φ38 brim at 23.25 already roofs the
relocated race, so the skirt+disc roofed air), and 2 mm of well-floor
lift.  What actually pins the coxa's height is the **M3×30 horn
screws living inside the hub**: their bench-pinned seat planes
(coxa-local 17.75 head / 16.75 centre) put the head tops at world
36.0, and the servo belly cannot sit below the heads.

| world z (mm) | old | new |
|---|---|---|
| 22.75–29.75 | Φ29 seat ring (to the uflange) | Φ29 seat ring (continues to 30.25) |
| 29.25 | *(mid-skirt)* | **cradle slab underside — the whole slab+cradle drops 4 mm as one rigid body** |
| 30.0 | *(mid-boss)* | hub boss truncated (keeps 0.75 into the slab) |
| ~29–39.25 | Φ52.4 dust-lip skirt + platform disc | **deleted** |
| 33.25 | hub boss top = cradle slab underside | — |
| 36.0 | M3×30 horn screw head tops | unchanged (seat planes bench-pinned) |
| 41.25 | servo well floor (5.25 over the heads) | **37.25** (1.25 over the heads) |
| 53.65 | hip axis | **49.65** |

The horn mount interface is untouched: same bolt pattern, same seat
depths, same screws — the head-access shafts and shank clearance
holes are re-cut through the dropped slab from the original seat
planes.  The remainder is the minimum for this screw stack: well
floor = head seat 17.75 + 3.0 head + 1.25 clearance; going lower
means shorter screws and re-tuning the bench-pinned seats.  Checked
on every rebuild (`check_coxa_column`): floor position, head
clearance, seat planes, shank passage, and the sub-slab silhouette
staying inside the seat ring + brim envelope.

Everything above rides down 4 mm with the cradle: hip axis
53.65 → 49.65, hip cap face 75.55 → 71.55, top-plate race seat
81.05 → 77.05, sheet 88.05..92.05 → **84.05..88.05**, pillars
85.95 → 81.95 mm long.  The robot's top deck drops 4 mm — a shorter,
stiffer sandwich (same members, less column length).

## Stack (world Z, chassis_bottom sheet mid-plane = 0)

| z (mm) | plane |
|---|---|
| 71.55 | hip cap outer face (stock) |
| 77.05 | pedestal top = inner-race seat |
| 77.55 | plate ring bottom (0.5 clearance over the race seat) |
| 84.05 | race top = Φ34 shoulder = sheet bottom |
| 88.05 | deck face |

(All five planes are 4 mm lower than the pre-shortening build —
they key off the hip axis, which dropped with the coxa cradle.)

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
  disc-horn clamp screws under the rings still need the full
  plate-off (there are no yaw-cap bolts anymore — the cap is deleted).

## Six rim pillars — how the top actually connects to the bottom

Each hip moment reacts as a force couple: push at the bottom tower,
pull at the top ring (or vice versa), i.e. **shear plus torsion
between the two plates at the rim**.  The four central M3 standoffs
(slender steel columns at rho 44) would carry that in bending and give
much of the rigidity right back; in this design they are demoted to
hatch/electronics anchors only.  The tie is six printed pillars at the
corner azimuths (rho 81.6, between adjacent legs — the only rim
territory outside every swing envelope):

* **Column**: plain solid ellipse, 20 mm radial × 14 mm tangential,
  z 2 → 83.95, ~25 g each in PETG.  Clearance to the swinging legs is
  **not** the column's problem: every part that rotates with a yaw
  joint is kept inside a **38.2 mm envelope** about its own axis (the
  coxa's cradle corners are rounded to that arc — see
  `coxa_link_rigid` below; the hip cap already fits at 37.0, the
  servo at 29.4).  The column surface sits 43.2 mm from each
  neighbouring axis, so **≥ 5 mm clearance holds at every yaw angle
  by construction** — measured 5.03 mm at build time, envelope and
  distance both asserted on every rebuild, verified clear for a full
  360° hand-spin of a disassembled leg.
* **`coxa_link_rigid`** (print 6): the production coxa with its two
  servo-cradle corner edges rounded to the 38.2 mm arc — at most
  2.16 mm comes off (they used to reach 40.36 mm) — plus the Φ29 hub
  seat ring and Φ38 dust brim for the tower-seated bearing (previous
  section).  Hub, horn drive, cradle pilots and cap seat are
  untouched — the price of a plain column instead of a scalloped one
  is a variant coxa print.
* **Top**: stops 0.1 mm short of the frame sheet (the six bearing
  RACES define the plate plane — the screws pull the sheet down onto
  the pillar; sand/shim a proud pillar, never let it rock).  Two Φ2.5
  pilots in a 10 mm solid plug: the hatch perimeter screw (one M3×16
  clamps lid → frame → pillar) and a dedicated frame screw at rho 87
  so the frame stays clamped with the lid off.
* **Foot = the old corner Wago tray bay**: production chassis_bottom
  grows a three-walled tray for a 5-way Wago at every corner flat.
  With the top frame on, those Wagos would be buried under solid deck
  anyway (no lever access), so the power splices **consolidate into
  the central block** (next section) and `chassis_bottom_rigid`
  **deletes the tray walls outright** (user, Aug 24).  The foot keeps
  the exact bay-sized footprint — every hole position unchanged — and
  registers on its three M3 bolts.  (On a stock chassis print the
  walls survive and the foot still drops in with 0.3 mm clearance;
  there they act as a bonus shear key.)
* **Foot bolts**: three M3 through-bolts with nyloc nuts on the belly
  (two inside the old bay footprint, one on an inboard tab that sits
  under the open hatch so its driver comes straight down).  The Φ3.4
  holes are **printed into `chassis_bottom_rigid`** at the same
  constants the foot uses, so they line up by construction; belly
  verified open at all three nut spots.  (Bench-modding a STOCK
  chassis print instead?  The foot is its own drill jig: sit the
  pillar in the tray, drill Φ3.4 through the 8 mm sheet+floor, bolt.)

Why this shape: light (~150 g total for all six), cheap (pure print,
~$3 of filament + 24 screws), strong (six large-section columns at
2× the standoff radius — slender-column estimates put plate shear ~3×
and torsion ~6× the four-standoff baseline), and easy (each pillar
sits flat on the sheet and takes 4 screws).

## Central splice block — the power tree consolidates 8 nuts → 4

With the corner trays gone (pillars stand there now), per-corner
splices no longer exist — `centre_wago_block` (print 1, ~11 g) replaces
the **6 corner Wagos AND the 2 trunk nuts** with 4× 5-port 221-415 in
one printed block at the chassis centre:

* **Layout**: two back-to-back press-fit rows sharing the middle wall
  (production tray constants: 0.15 mm wedge, 2.4 mm walls, walls stop
  ~1.9 mm below the nut top).  North pair = V+, south pair = GND; the
  two nuts of each net are jumpered, which leaves battery-in + six leg
  branches + one spare port per net.  Wire entries face outward, so
  the leg pigtails fan in radially without crossing the block.
* **Location**: centred on the origin, 66.9 × 44.1 mm footprint —
  probed and asserted against the real chassis_bottom solid: fully on
  solid sheet (VHB backing, no slot bridged), 2 mm shy of the battery
  strap slots (inner edge y = 24), 6+ mm from the standoff posts, and
  the battery-lead trunk pass (x 41…55) lands right at its east
  ports.  Corner radius 40.1 mm — the whole block sits under the
  64 mm hatch opening, **levers up, reachable from straight above with
  the lid off**; even a flipped-open lever (z ≈ 22) stays below the
  z 24 rotating band.
* **Mount**: VHB pad — the exact scheme the production corner trays
  used for months before merging into the chassis print, and here the
  wire pull is a gentle radial fan rather than one corner's inward
  yank.  No drilling; the chassis floor under it is untouched
  production sheet.
* **Wiring delta**: each leg's power pigtail runs corner → centre
  (~60–70 mm longer); battery leads come up through the existing trunk
  pass directly into the east ports.

## Fits — all bench-tuned production constants, nothing new

| interface | Φ (mm) | source constant |
|---|---|---|
| boss → inner race (top AND bottom) | 25.15 (+0.15 press) | `YAW_HUB_BOSS_OD` |
| pocket → outer race (top plate AND bottom tower) | 37.15 (+0.15 firm slip) | `YAW_TOWER_BORE_OD` |
| race shoulder | 34 | `YAW_TOWER_SHOULDER_OD` |
| ring wall | 44 = 37 + 2×3.5 | `YAW_TOWER_WALL` |
| hub seat ring → inner-race land | 29 (axial seat, not a fit) | `YAW_BEARING_INNER_OD` |
| dust brim (OD, over the housed race) | 38 = race OD + 2×0.5 — hovers 0.5 mm, never touches | `BRIM_*` in the generator |

Boss tip has a Φ24×0.8 stepped lead-in; the top plate's pocket mouths
have a 0.8 mm lead-in ring (the bottom tower keeps production's plain
open-top bore — drop-in, not blind-pressed).

## Measured workspace trade-off (from the build-time sweep)

The full-size plate caps the femur's UP-swing (production workspace
envelope was femur −80°…+30°):

* femur vs plate: first contact at **−52.5°** (identical at yaw −35…+35)
* femur vs the boss/bearing stack: first contact at **−60°**
* ⇒ **safe femur up-limit −47.5°** (1 grid step + 2.5° margin), baked
  into the scene's hip joint limits.
* The 4 mm coxa shortening did **not** move these numbers: plate, hip
  axis and femur pivot all dropped together, so the relative geometry
  (and the limit) is unchanged — re-verified by the sweep after the
  drop.

Walking around `STANCE_FEMUR_DEG = −25` keeps ~22° of up-headroom.
**Deep tucks (stand-up belly curl, self-righting) exceed this** — check
those trajectories against −47.5° before putting this variant on the
robot.  Sweep used the stance-relative tibia angle (+75°); knee-range
variation only moves parts that stay outboard/below the plate.

## BOM delta vs production

* 6805-2RS count unchanged (12): the 6 upper yaw bearings move up to
  become the 6 top-plate bearings; the 6 lower ones stay in their
  production tower pockets
* **−6 `yaw_bearing_cap` prints and −18 M3×8 cap join screws** — the
  cap is deleted (see "One tower-seated bottom bearing")
* 4× M3 standoff stacks, ~86 mm (bottom sheet top z≈2 → hatch underside
  88.05; e.g. 50+36 F-F, or M3 threaded rod in printed sleeves) + M3
  screws down through the hatch into the stack tops — **non-structural**
  in this variant (hatch/electronics anchors only)
* 6× M3×16 button-heads for the hatch perimeter (through lid + frame
  into the pillar plugs)
* 6× M3×10 for the dedicated frame→pillar screws
* 18× M3×12 + 18× M3 nyloc nuts for the pillar feet (belly side)
* the 140 mm `chassis_top` deck + its 20 mm standoffs are not used
* **−4 Wago 221-415**: the 6 corner + 2 trunk nuts become 4 in
  `centre_wago_block` (print 1, ~11 g, VHB pad; leg power pigtails
  ~60–70 mm longer — wiring change only)
* **6× `coxa_link_rigid` reprints** — the stock coxa neither fits the
  yaw envelope the plain columns rely on nor reaches the tower-seated
  race (same filament as 6 production coxas; the old prints become
  spares)
* **1× `chassis_bottom_rigid` reprint** (~313 cm³, same class as the
  production chassis print) — tower platforms trimmed to the tower
  cylinder, shaved ears, printed foot holes, corner Wago trays
  deleted.  A stock chassis print can be bench-modded instead
  (saw/sand corners + drill), skipping the reprint entirely
* recommended: 12× M3 heat-set inserts (McMaster 94459A130) for the hip
  cap pilots, +12 for the pillar-top pilots — see "Disassembly & service"

## Assembly order

1. Build the robot as production but with four swaps: build on
   `chassis_bottom_rigid` (or a bench-modded stock chassis); use
   `coxa_link_rigid` in place of the stock coxa; press ONE 6805 onto
   each coxa's hub boss from below until it seats against the Φ29
   seat ring, then drop the leg + bearing into the tower pocket (the
   production lower-race seat — no `yaw_bearing_cap`, no cap bolts;
   the raised rim swallows the full race, top flush) and couple the
   horn as production; and use `hip_clamp_cap_rigid` in place of the
   stock hip cap — same 2× M3 into the same cradle pilots.
2. Re-splice power at the centre: VHB `centre_wago_block` to the floor
   centred on the origin (footprint verified/asserted against the real
   chassis solid), seat 4× 221-415, jumper each pair, land the battery
   leads through the trunk pass into the east ports and each leg's
   (lengthened) pigtails into the fan-out ports.  The 6 corner and 2
   trunk nuts are retired.  Then sit each `corner_pillar` flat on the
   sheet at its corner flat and bolt down with M3×12 + belly nylocs
   through the printed foot holes (stock chassis: the tray walls are
   still there — drop the foot between them and drill first, using
   the foot as the jig).
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
* **Leg removal** (plate off): undo the horn screws and the whole leg
  lifts straight out of the tower pocket WITH its bottom bearing —
  verified drop-in/lift-out path at build time.  No cap to unbolt,
  no press fit to separate (the race stays pressed on the hub boss).
* **Reassembly**: either lower the plate+caps unit back over all six
  servos, or bolt the caps onto the servos first and lower the bare
  plate onto the races (the original assembly order; descent verified
  clear both ways).
* **Repeated-service threads**: the cap bolts are now every-service
  fasteners, so per the repo's insert rule, drill the two cradle
  pilots Φ4.0 × 6 deep and fit M3 heat-set inserts (94459A130) — a
  bench mod on the `coxa_link_rigid` print.
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
* `corner_pillar`: foot down, column up — plain solid column, no
  supports.
* `coxa_link_rigid`: print exactly like the production coxa (same
  orientation and supports).  The rounded corners change nothing; the
  Φ29 seat ring and Φ38 dust brim print as horizontal cylinder bands
  exactly like the uflange above them (the coxa prints on its side).
* `chassis_bottom_rigid`: print exactly like the production chassis
  (belly up, same supports).  The tower-cylinder trim, raised rims
  (a straight continuation of the Φ44/Φ37.15 barrel), shaved ears and
  foot holes change nothing about the print strategy.
* `centre_wago_block`: floor down — flat print, no supports; walls are
  plain vertical extrusions like the production trays.
* `hip_clamp_cap_rigid`: rest on the tongue face (outer face + boss
  up) so the press boss prints as a vertical cylinder; supports under
  the flange wings and hooks.  Printing in the stock flat orientation
  puts support scars on the press band — coupon-check the fit if you
  do that.
* Reused parts in `stl/` (femur_link, tibia_knee_yoke, …) are for the
  viewer; print those from the main `stl_prototype/`.  The variant
  prints are the parts above (including `coxa_link_rigid.stl` and
  `chassis_bottom_rigid.stl`, which REPLACE their stock parts).
  There is no `yaw_bearing_cap.stl` here — that production part is
  not used.  `*_DO_NOT_PRINT.stl` are COTS visuals.

## Build & view

```sh
# from the repo venv
python concepts/rigid_hip/make_rigid_hip_variant.py            # full checks
python concepts/rigid_hip/make_rigid_hip_variant.py --skip-sweep  # fast iter

npx buildviz register hexapod_walker/prototype_sts3215/concepts/rigid_hip \
    --build-id sts3215-rigid-hip
# http://127.0.0.1:5183/?build=sts3215-rigid-hip

# STEP/BREP exports (Onshape etc.) of all seven variant printables --
# see cad_step_test/README.md; verifies BREP vs these meshes at build time
uv run --no-project --python 3.12 --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_rigid_hip_step.py

# the STEP files themselves are also viewable in BuildViz (native STEP
# ingest; parts laid out side by side, print-checkable):
#   cd cad_step_test/out/step && npx buildviz push-step \
#     hip_clamp_cap_rigid.step chassis_top_rigid.step top_hatch_rigid.step \
#     corner_pillar.step centre_wago_block.step coxa_link_rigid.step \
#     chassis_bottom_rigid.step --build-id sts3215-rigid-hip-step
# http://127.0.0.1:5183/?build=sts3215-rigid-hip-step
```

Checks run at build time: watertightness, seated-stack placement, the
bottom joint (race on the tower seat, coxa/race contact = boss press
only, seat ring lands on the race top, rim proven AT the race-top
plane with air above it — nothing continues the Φ44 column — brim
present with its 0.5 running gap open, leg + bearing lift-out path),
the shortened coxa column (well floor at its computed plane, 1.25 mm
head clearance real, horn-screw seat planes and shank passages open
at the bench-pinned depths, sub-slab silhouette inside the seat-ring
+ brim envelope, slab clear of the tower rim),
the chassis variant (nothing outboard survives past the tower
cylinder, the trim never bit the tower wall, nothing past the tower
cylinder above the servo-mount deck at any azimuth up to the raised
rim with the az-210 root still solid below it, the full-wrap raise
ring complete on all six towers, foot holes open where the pillar
feet expect them, all six Wago tray wall sets gone with the sheet
still solid underneath), full 360° yaw sweep vs the
plate, straight-down plate descent over all six bearings, pillar
clearances (seated robot, ±45° operating yaw with margin, and an
informational full hand-spin scan) and the femur pitch×yaw contact
sweep (fails the build if the safe limit ever eats into walking
headroom at −45°).
