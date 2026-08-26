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
  stands where a corner Wago tray used to be and doubles as the screw
  boss for the hatch + frame screws — both stations carry **printed-in
  M3 heat-set insert bores** so frequent lid/plate removal never
  cycles a plastic thread (user, Aug 26).
* **`coxa_link_rigid`** (print 6): the production coxa with four
  variant edits — servo-cradle corners rounded to the 38.2 mm yaw
  envelope so the plain columns clear the swinging leg by 5 mm at
  every angle, a Φ29 seat ring down to the relocated bottom bearing
  (the bearing itself presses on the production Φ25.15 hub boss,
  now 5 mm lower on the same cylinder), a small Φ38 dust brim
  hovering 0.5 mm above the bearing top, and the vertical hub column
  **shortened 14 mm** (Φ52.4 skirt + platform disc deleted, **horn
  screws swapped M3×30 → M3×20** — see the hardware note below —
  cradle dropped) so the hip servo sits just above the bearing (see
  below).
* **`chassis_bottom_rigid`** (print 1): the production chassis with the
  six square tower platforms trimmed to the tower's own Φ44 cylinder —
  one matching curve from belly to bearing pocket — each tower's
  bearing pocket **lowered to the servo-mount deck** (Aug 25: the
  race seat ledge sits 0.5 mm over the deck / servo case top and the
  Φ44 column ends exactly at the new bearing top, world 17.75 — see
  below), the dead cap-bolt ear lugs shaved off, the 18 pillar-foot
  bolt holes printed in, and the six corner Wago tray wall sets
  deleted (dead geometry in this variant — see below).  Every
  remaining functional surface — press bore, well walls, strap
  slots — is production geometry.
* **`centre_wago_block`** (print 1): the corner Wago trays are gone
  (pillars stand there), so the power tree consolidates — 4× 5-port
  221-415 (two per net, jumpered) in one press-fit block at the
  chassis centre under the open hatch, replacing the 6 corner +
  2 trunk nuts (see below).

Load path: hip moment → cap boss → top bearing → top plate →
**six rim pillars** → chassis_bottom + the five other legs.  Each yaw
axis becomes simply-supported instead of cantilevered, holding the hip
servo rigidly.

> **⚠ PURCHASED-HARDWARE CHANGE (Aug 25, supersedes the Aug-24 M3×25
> step): the 30 yaw-horn screws are M3×20 SHCS on this variant, not
> the production M3×30s.**  Every seat plane sits exactly 10 mm
> deeper so the tips — and the thread engagement in the disc horn /
> output spline — are identical to the bench-tuned production stack.
> Do not install M3×30s or M3×25s in a `coxa_link_rigid` print:
> their tips bottom out on the servo case under the disc 10/5 mm
> early, jamming the horn off its seat and leaving the heads proud
> into the servo well.  See "The coxa hub column" below.

## One tower-seated bottom bearing — the yaw bearing cap is deleted

The production yaw joint stacked TWO 6805s only to form a 7 mm moment
couple; with the top bearing providing a ~56 mm couple, one bottom
bearing suffices.  Earlier revisions of this variant kept that single
race where production located it — first in the bolt-on
`yaw_bearing_cap`'s own bore, then (Aug 24) in the production
LOWER-race pocket.  The Aug 25 rev drops it a further 5 mm, to the
true physical floor:

* The single race sits in the tower's Φ37.15 press bore with its
  **bottom face 0.5 mm above the servo-mount deck top (world 10.75)**
  — which is simultaneously the yaw servo's case-top plane.  To go
  lower, the servo itself would have to sink into the chassis.
* **Why it CAN sit this low** (corrects the old *"the race cannot sit
  lower, it would rub the horn"* claim — that was architectural, not
  physical): the disc horn is not at the coxa mount plane at all.
  The real Φ20 tapped disc sits RECESSED in the servo's output face
  at world 4.25…6.25, UNDER the deck plate, and everything between
  the deck plane and the old race position is the coxa hub's own
  **Φ25.15 wide press boss** — which production already runs down to
  the deck plane, necking to the Φ20 drive nub that reaches the horn
  through the deck's Φ24 bore.  The horn and its screw heads (which
  seat high in the slab, never down here) are inside/below that
  boss.  So the inner race presses on the **same bench-tuned +0.15
  boss band as before, just 5 mm lower on the same cylinder** — no
  new collar, no new press geometry, no wall-thickness risk: the
  "collar" IS the production boss (solid to the Φ24 nub bore below,
  merged into the hub above; the rotating boss keeps 4.4 mm radial
  to the static ledge lip and 0.5 mm axial to the deck).
* The **outer-race seat ledge** is the production Φ34 shoulder
  relief, kept only as a 0.5 mm-proud step over the deck band and
  bored to Φ37.15 above: face r 17.0…18.575 — the same 1.5 mm race
  land as the production seat — solid at every azimuth and sitting
  directly on the 4 mm servo-mount deck.  It clears the rotating
  horn (r ≤ 10) and boss (r 12.575) by construction.  The tower
  hoop above it keeps the production Φ44/Φ37.15 section (3.425 mm
  wall); below, the ledge stands on the deck, not on a thin hoop —
  the servo well bore lives further down.
* The coxa hub grows a **Φ29 seat ring** (the production uflange OD,
  bearing only on the Φ25…29 inner-race land) down to the relocated
  race top (world 17.75) — free, since the coxa is already a variant
  print.
* The **`yaw_bearing_cap` is deleted**: −6 prints, −18 M3×8 screws,
  and one less part in the race-to-race tolerance stack.  Its two jobs
  moved: *radial housing* → the tower press band, *axial retention* →
  split by direction.  Hanging loads run hub → seat ring → inner race
  → outer race → deck-level tower ledge; standing loads run up through
  the TOP bearing into the plate shoulder.  Each bearing takes one
  direction; no lip needed.
* **The tower wraps the full race** (user, Aug 24 rev 2: *"the bearing
  should be just above the horn and the motor should sit just above
  that"*; Aug 25: *"why cant the bearing come down even more and be
  right on top of the servo?"*): the old tower band above the new
  seat plane is cut away and a fresh full-wrap Φ44/Φ37.15 ring runs
  from the deck band to the new race-top plane — the pocket houses
  the full 7 mm outer-race width and the Φ44 column ends exactly at
  the bearing top (world 17.75; the whole tower above the deck is
  now just 7.5 mm tall).  What remains coxa-side is a **Φ38 × 2 mm
  dust brim** hovering 0.5 mm above the rim/race plane and stopping
  3 mm inside the tower Φ44, so it reads as coxa.  It **touches
  nothing** — a contact wiper would add friction, squeak
  PETG-on-PETG, and wear; 0.5 mm rides out print tolerance and
  bearing play.  The brim fully roofs the 2RS seal and outer-race
  band, and the seal itself remains the real dirt barrier (grit has
  one turn under the brim — an accepted trade for the clean stack).
* The bearing couple is preserved: both races (bottom in the tower,
  top at the plate) dropped 5 mm together, so the ~56 mm mid-plane
  to mid-plane spacing is unchanged.

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
  **servo-mount deck top** (z 10.25).  Its below-deck root used to
  stay merged with the deck-skin roof; since rev 6 that roof (and the
  root with it) is gone — the ear centre sits at r 23.5 from the yaw
  axis, outside the tower keep cylinder.  Above the deck, nothing
  pokes past the tower cylinder at any azimuth: the towers read as
  six clean bare columns.
* **18 pillar-foot holes printed in** (same constants as the pillar
  feet, aligned by construction) — no bench drilling on a fresh build.
* **Corner Wago trays deleted** (user, Aug 24: "they dont make any
  sense anymore in this version"): the splices live in
  `centre_wago_block` now, so the six 2.4 mm tray wall sets are dead
  geometry — everything above the sheet top is cut away (the 1 mm
  embed band inside the sheet stays as interior material).  The pillar
  feet register on their three M3 bolts instead of the old wall key.
* **Bearing pockets lowered to the deck** (Aug 25) — the full-wrap
  housing described in the bottom-bearing section: the old band above
  the new seat plane is cut away (leaving the 0.5 mm-proud deck-level
  ledge) and a fresh Φ44/Φ37.15 ring is unioned to the new race top
  (world 17.75) after every variant cut, so the ear shaves never nick
  it.
* **Wire corridors + cradle shells flattened** (user, Aug 24 rev 5 +
  Aug 25 rev 6): the wago-era apparatus inboard of each seated yaw
  servo — cradle end wall, porch canopy, side-wall stubs — AND the
  cradle-shell run outboard of it (side walls + the deck-skin roof
  they carried, whose ends were the "two gray things" flanking each
  pillar) are cut back to the bare sheet, one box per leg (leg-frame
  x 50 → 100, |y| ≤ 20.5, sheet top → over the deck) minus the tower
  keep cylinder (r 21.95).  See "The flattened corner" below.
* **Tower flanks smoothed** (user, Aug 25 rev 7): the swing-relief
  protect-ring bulge on each tower's outboard flank (r up to 23.5,
  z 6.25–10.25) is shaved to the trim cylinder — see "The tower-flank
  bump" below.  The tower outer profile is now ONE vertical cylinder
  from the sheet top to the rim (asserted per rebuild, all six legs).
* Net: −67 cm³ (268 vs 335).  **On an existing stock chassis print**
  the cosmetic edits are still a bench mod (saw/sand the corners back
  to the tower barrel, shave the ear lugs and corridor walls, drill
  the foot holes using the pillar feet as jigs), but the Aug 25
  lowered pocket is NOT: it needs the Φ34 shoulder relief bored to
  Φ37.15 down to 0.5 mm over the deck and the tower cut 5 mm shorter
  — a boring-bar job on a printed part.  For this variant, reprint
  the chassis.  The stock tray walls can stay on a bench-modded
  print — the feet were sized to fit them with 0.3 mm clearance, so
  they just become a bonus shear key.

### The flattened corner (user decisions, Aug 24 rev 5 + Aug 25 rev 6)

The plate top inboard of each seated yaw servo used to carry the
wago-era **wire-corridor apparatus** — per leg (leg frame, yaw axis at
x 100, sheet top z 2, deck top z 10.25):

* the cradle's transverse **end wall** at x 61.5–64.5, full height
  z 2 → 10.25, standing 0.3 mm off the servo's inboard end face
  (x 64.8), with the harness notch at |y| ≤ 3.5;
* a **"porch" canopy skin** (z ≈ 8 → 10.25) roofing a floorless
  corridor down to x ≈ 50 (the harness aperture passes through the
  plate there — production geometry, still open);
* **side-wall stubs** at |y| ≈ 14–18.9.

Identification history, one line: the end wall was first probed and
kept as "the cradle's corner pier / servo end-stop / wire channel"
(Aug 24 evening) — the user overruled: *"take a step back that CRADLE
WALL isnt doing shit, just flatten it out and its fine"*, and
clarified the scope: *"this isnt wall flattening, its flatening random
bumps in the top of the chassis plate that serve no purpose"*.  With
the splices consolidated into `centre_wago_block`, the harness simply
exits the well into open air — none of that apparatus registered,
sealed or carried anything.  **Rev 5** cut that band back to the bare
sheet with one box per leg (x 50 → 64.65, |y| ≤ 20.5, z 2 → 12).

**Rev 6 (user, Aug 25): the cradle shell goes too.**  Rev 5 stopped at
the old wall-to-servo gap, which left the **cradle-shell run**
standing outboard of the cut face: the 5.6 mm side walls at |y|
13.2–18.9 (z 2 → 10.25) and the 2.2 mm deck-skin roof they carried
(z ≈ 8.05 → 10.25, spanning the full 37.6 mm between the wall outer
faces).  At the corners those shell ends read as *"two gray things
from the waygo sticking up on each side of the column"* (user) —
vertical cut faces flanking each pillar at ~10 mm, one from each
adjacent leg.  Measured before removing them: the roof hovers
**1.8 mm above the real servo case top** (the case front face is at
z 6.25; the "case top = deck top 10.25" plane is only the horn-boss
region inside the tower — the fat visual servo block hides this),
nothing bears on the roof (the bearing and the top plate ride the
towers and pillars), the walls sit 0.85 mm off the case flanks, and
their only job was holding that roof up.  The flatten box now runs to
the yaw axis (x 50 → 100) **minus the tower keep cylinder**
(r 21.95 about the axis), all six legs
(`pillar_stubs_annotated.png` — the user's camera angle, before/after;
`corner_flattened_annotated.png` — plan sections).  Chassis print
288 → 268 cm³.

**What actually registers each seated yaw servo** (all verified
against the cut mesh on every rebuild, `check_chassis_variant`):

* the **sheet-level well** (z 0 → 2): hugs the case at 0.75 mm/side
  for its full length — the production lateral register, below the
  cut plane and untouched;
* the **`yaw_servo_retainer`** below the sheet (z −40 → −6, bolts up
  from the belly into plate pilots at leg-frame x 71/87.5, |y| 21 —
  outside the cut in x, y and z): its grip jaws take the lateral
  loads, its bar takes the drop-out load;
* the **shell inside the tower keep cylinder** (well-mouth collar,
  pocket floor, seat ledge): registers the case at the output end —
  and it is structural: it carries the **inboard arc of the 6805
  seat** over the servo tunnel, which is why the keep cylinder is a
  hard boundary for the cut, not styling.

What you now see at a corner: bare sheet, the pillar, the Φ44 tower
boss, and the servo case itself standing 4.25 mm proud of the sheet —
the thing that was hiding behind the stubs.  (`corner_flat_owners.png`
— a z=8 section of the pre-flatten corner with per-part colours —
remains as the ownership reference for what the walls were.)  Since
rev 8 this is not just what you see but a RULE: above the bare sheet
top, the towers are the only material allowed at all (see "The
above-sheet WHITELIST" below).

### The tower-flank bump (user, Aug 25 rev 7)

After rev 6 the user's viewer close-up showed a stepped bulge still
riding each tower's outboard flank under the bearing: *"remove this
weird bump outside the bottom chassis in the part that hols the
bearing and make it vertically smooth"*.  Probing the shipped STL
(radius vs z vs azimuth about each yaw axis): from the sheet top the
flank is the clean trim cylinder (r 22.02) up to z 6.25, then two
arcs per leg step out to **exactly r 23.5** across z 6.25–10.25
(leg-frame az ~17–55° and ~300–314°, identical on all six legs), then
back to r 22.00 up to the rim.

Code origin: production's `_chassis_yaw_cradle_solid` clears the horn
with a +X swing relief, but protects a cylinder **1.5 mm fatter than
the tower** (`tower_protect`, r = 37/2 + 3.5 + 1.5 = 23.5), so the
rectangular cradle-shell box survives out to r 23.5 across the
mount-plate band (front face z 6.25 → plate top z 10.25).  In
production that margin ring carried the roots of the cap-bolt ear
bosses; in this variant the cap and its bolts are deleted, the ears
are shaved, and the ear/flatten cuts already removed most of the ring
— the two arcs were pure leftovers.  Verified before cutting: the
working wall (Φ44 hoop, r ≤ 22), the Φ34/Φ37.15 seat ledge (r ≤ 18.6)
and the deck all live inside the trim radius, and nothing mates with
the flank.

Rev 7 adds one ring cutter per leg (the corner-trim box footprint,
z 6.0–10.5, minus the same `CHB_TRIM_R` cylinder).  Result: the tower
outer profile reads r 22.00–22.02 at every z from the sheet top to
the rim — one vertical cylinder, nothing removed inside r 22, hoop
thickness around the race unchanged, rotating-part clearances only
grew.  `check_chassis_variant` now asserts per-leg cylindricity over
the whole sheet-top → rim band (the assert was verified to reject the
pre-shave STL).  Before/after from roughly the user's camera angle:
`tower_flank_smoothed.png` (its generator script needed the retired
trimesh builder's `bump_shave=False` knob and was deleted with the
STEP-first flip; the figure and script live in git history).

### The above-sheet WHITELIST (user, Aug 25 rev 8)

After all of the above, a top-down drawing of `chassis_bottom_rigid`
still showed thin diagonal slashes flanking the towers and L-shaped
brackets at the corner flats: *"Its incredible how many times ive
asked this but I'll try again, get rid of these L shaped bumps I
circled in red … all of them, not just the ones i circled"*.

The inventory (cross-section + vertex census, `probe_above_sheet.py`)
found the mesh pipeline **already clean** — above z 2.05 it carries
exactly six tower/deck columns and nothing else.  The offenders lived
only in the **STEP sidecar**: the base STEP chassis builder
(`cad_step_test/build_step_first_test.py::_chassis_wago_tray_solid`)
still models the RETIRED two-bay WAGO3 corner tray — 4.8 mm wider
(half-width 22.15 vs 17.325) than the production single-bay WAGO5
tray the variant's tray-delete cutter is sized for — so the old
tray's two side walls survived the delete at **all six corners**: 12
wall remnants, 2.4 mm thick, z 2 → 8.5, spanning corner-frame radial
76.75 → 100 at tangential |y| 19.75 → 22.15 (r 27.9 → 35.1 from the
nearest yaw axis).  Seen from above, one wall reads as a diagonal
slash near each tower and the wall + outer-wall stub corner reads as
an L at each flat; the "adjacent holes" in the drawing are the
printed pillar-foot bolt holes through the sheet.  They slipped every
earlier pass because each pass was a BLACKLIST — name a leftover, cut
it — and the STEP part's +1.5 % volume sat under the 2 % equivalence
gate.

Rev 8 flips the rule to a **whitelist**.  Above the bare sheet top
(z 2) the ONLY material allowed on `chassis_bottom_rigid` is:

1. **the six yaw towers** — everything within the trim cylinder
   (r ≤ 22.02 about each yaw axis): press bore, seat ledge, in-keep
   deck, rim;
2. **the servo deck plateaus** — measured from the servo well
   geometry this category is *degenerate*: the only deck a seated
   servo still needs (the 6805 seat's inboard arc over the servo
   tunnel + the well-mouth collar) lives entirely inside the tower
   keep cylinder, i.e. inside category 1 (rev 6 measured that
   nothing bears on or registers against the roof outside it);
3. **verifiable mates** — measured *empty*: the pillars bolt through
   holes in the SHEET and their feet stand ≥ 5.2 mm from any
   above-sheet material outside the towers, the retainer bolts from
   BELOW the sheet, the wago block is VHB-taped to the sheet at the
   centre.

Enforcement is structural, not per-feature: **both** pipelines now
apply one global cut — a box from the sheet top to above the rim
minus the six tower cylinders (`CHB_WL_*`), before the rim re-union —
so anything standing outside the whitelist is removed *whether or not
a pass ever named it*.  `chassis_whitelist_violations` (the vertex
census) is asserted in `check_chassis_variant` on every mesh build
AND against the STEP-derived STL in `build_rigid_hip_step.py`; run
against the pre-rev-8 STEP STL it reports 120 offending vertices
(worst r 35.13) and against the mesh STL 0 — on the mesh part the
cut removes exactly nothing (0.0000 mm³ of material in the cut
region, measured by boolean intersection; only the tessellation
shuffles through the extra boolean).  STEP-vs-mesh volume
delta: +1.51 % → −0.20 % (now the same tessellation slop as the other
parts).  Nothing below z 2 was touched; rotating-part clearances only
grew.  Proof drawing: `chassis_top_clean.png` (hub top view — hex
outline, holes, six towers, nothing else standing).

### The joint column: horn → bearing → coxa (Aug 24, dropped to the deck Aug 25)

The Aug 24 redesign made the stack legible (bearing over the horn,
coxa over the bearing, Φ44 column ending at the bearing top) but kept
the race at the production seat, 5.5 mm above the deck, on the belief
that the spinning inner ring had to clear a static horn below it.
**That was wrong** (user, Aug 25: *"why cant the bearing come down
even more and be right on top of the servo?"*): the horn ROTATES with
the coxa and the inner race — the static/rotating boundary is the
outer race + tower vs everything inside — and the real Φ20 disc is
recessed BELOW the deck anyway.  The bearing now sits as low as
physics allows, its floor set by the static deck / servo-case plane:

| world z (mm) | Aug 24 build | Aug 25 build (bearing on the deck) |
|---|---|---|
| 10.25 | servo-mount deck top = servo case top | unchanged |
| 10.75 | — | **race seat ledge** (0.5 over the deck/case — the physical floor) |
| 10.75–17.75 | *(hub boss band)* | **6805 race, fully housed** (presses on that same boss band) |
| 15.25 | coxa mount plane (real disc horn recessed at 4.25…6.25) | unchanged |
| 15.75 | race seat (believed pinned by horn clearance) | *(race moved down — no seat here)* |
| 15.75–22.75 | 6805 race, fully housed | *(now hub ring / brim territory)* |
| 22.75 → **17.75** | tower rim = race top; Φ44 column ends here | same role, **5 mm lower** |
| 23.25–25.25 → **18.25–20.25** | Φ38 dust brim (0.5 above rim/race, 3 mm inset) | same role, 5 mm lower |
| 24.25 → **19.25** | cradle slab underside (1.5 over the rim) | same margin, 5 mm lower |

The floor is now genuinely pinned by hardware: deck/servo-case top
10.25 + 0.5 mm static-vs-rotating clearance + 7 mm bearing = 17.75;
to go lower the servo itself would have to sink into the chassis.
`joint_column_annotated.png` is the annotated section of the NEW
stack — every height band, what it is, why it is there (regenerate
with `make_joint_column_figure.py`).

### The coxa hub column: shortened 14 mm (Aug 24 rev 3/4 + Aug 25 M3×20)

The user read the band ~25–40 mm above the deck as "largely
unnecessary" — and the audit agreed.  In the production coxa that
band held three things: the Φ29 seat-ring/boss spine (structural,
but taller than needed), the **Φ52.4 dust-lip skirt + 6 mm platform
disc** (world ~29–39.25 — a dust guard for the production
UPPER-bearing position, which this variant vacated when the race
moved down into the tower; the variant's own Φ38 brim already roofs
the relocated race, so the skirt+disc roofed air), and 2 mm of
well-floor lift.  **Rev 3** deleted the skirt+disc and dropped the
cradle 4 mm, down to the M3×30 horn screw heads.  What then still
pinned the height was **screw length alone**: the hub band between
the bearing brim and the servo well floor existed almost entirely to
house the M3×30s.  **Rev 4** swapped to M3×25 — the most the
then-current tower rim (race top at 22.75) allowed.  **Aug 25**
(user: *"worry about the screws last — I can buy any type of screw
and shorter ones would be better"*): with the bearing itself dropped
onto the deck, the rim moved down 5 mm and the slab can follow — all
five horn screws per hub (4 drive + 1 centre) become **M3×20**, and
every seat plane sinks by exactly the 10 mm length delta vs
production — tip = seat − length, so the tip planes and thread
engagement never move off the bench-tuned production stack: the
corner tips still break the Φ20 tapped disc's far face by 0.25 mm
(full 2 mm engagement), the centre screw still reaches the same
depth in the output spline's tapped bore.  `check_coxa_column`
asserts seat drop == length delta on every rebuild.  **M3×16 does
not fit**: another −4 would put the rotating cradle slab 2.5 mm
*below* the static tower rim — collision — so 20 is the shortest
standard length for this joint; it lands the slab 1.5 mm over the
rim (≥ 0.5 rotating-vs-static required), the same margin the M3×25
build had.

| world z (mm) | M3×30 build (rev 3) | M3×25 build (rev 4) | **M3×20 build (Aug 25)** |
|---|---|---|---|
| race band | 15.75–22.75 | 15.75–22.75 | **10.75–17.75 (on the deck)** |
| tower rim = bearing top | 22.75 | 22.75 | **17.75** |
| Φ38 dust brim | 23.25–25.25 | 23.25–25.25 | **18.25–20.25** |
| seat ring top | 30.25 | 25.25 | **20.25** (still 1 mm inside the slab) |
| cradle slab underside | 29.25 | 24.25 | **19.25** (1.5 over the rim) |
| hub boss truncation | 30.0 | 25.0 | **20.0** (keeps 0.75 into the slab) |
| horn screw head seats | 33.0 head / 32.0 centre | 28.0 / 27.0 | **23.0 / 22.0** (10 mm deeper, tips unchanged) |
| horn screw head tops | 36.0 | 31.0 | **26.0** |
| servo well floor | 37.25 | 32.25 | **27.25** (still 1.25 over the heads) |
| hip axis | 49.65 | 44.65 | **39.65** |
| hip cap face | 71.55 | 66.55 | **61.55** |
| top-plate race seat | 77.05 | 72.05 | **67.05** |
| top sheet | 84.05..88.05 | 79.05..83.05 | **74.05..78.05** |
| corner pillar length | 81.95 | 76.95 | **71.95** |

The horn mount interface AT the horn is untouched: same bolt
pattern, same tip depths, same engagement — the head-access shafts
and shank clearance holes are re-cut through the dropped slab from
the deeper seat planes.  The remainder is the minimum for the M3×20
stack: well floor = head seat 7.75 + 3.0 head + 1.25 clearance
(coxa-local).  Checked on every rebuild (`check_coxa_column`): floor
position, head clearance, seat planes, the seat-drop == length-delta
engagement guard, shank passage, the sub-slab silhouette staying
inside the seat ring + brim envelope, and the slab-to-rim clearance.

Everything above rides down with the cradle (14 mm total vs the
production-coxa stack, 5 mm vs the M3×25 build): hip axis 53.65 →
49.65 → 44.65 → **39.65**, hip cap face → **61.55**, top-plate race
seat → **67.05**, sheet → **74.05..78.05**, pillars → **71.95 mm**
long.  The robot's top deck drops 14 mm vs the pre-shortening build
— a shorter, stiffer sandwich (same members, less column length).

## Stack (world Z, chassis_bottom sheet mid-plane = 0)

| z (mm) | plane |
|---|---|
| 61.55 | hip cap outer face (stock) |
| 67.05 | pedestal top = inner-race seat |
| 67.55 | plate ring bottom (0.5 clearance over the race seat) |
| 74.05 | race top = Φ34 shoulder = sheet bottom |
| 78.05 | deck face |

(All five planes are 14 mm lower than the pre-shortening build — 4 mm
from the rev-3 skirt/platform delete, 5 mm from the Aug-24 M3×25
step, and 5 mm from the Aug-25 bearing drop + M3×20 screws — they key
off the hip axis, which dropped with the coxa cradle.)

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
* **Retention**: 6× M3×14 button-heads that pass through the lid AND
  the frame into **brass heat-set inserts in the corner pillar tops**
  (user, Aug 26: the lid comes off "a lot", so no screw ever cycles a
  printed thread — steel-into-brass survives unlimited service at full
  clamp preload, which is what keeps the plate's stiffness; magnets /
  quarter-turn latches were rejected for exactly that reason), PLUS
  the 4 chassis standoff screws, which now pass through the hatch into
  the (metal-threaded) standoffs.  Chassis-hang loads run standoffs →
  hatch → deck face → frame in pure compression; the screws only see
  rebound.
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

* **Column**: plain solid ellipse, 22 mm radial × 14 mm tangential
  (grown 20 → 22 radially for the insert bores' walls; the TANGENTIAL
  width — the leg-clearance direction — is pinned at 14), z 2 → 73.95,
  ~24 g each in PETG.  Clearance to the swinging legs is
  **not** the column's problem: every part that rotates with a yaw
  joint is kept inside a **38.2 mm envelope** about its own axis (the
  coxa's cradle corners are rounded to that arc — see
  `coxa_link_rigid` below; the hip cap already fits at 37.0, the
  servo at 29.4).  **≥ 5 mm clearance holds at every yaw angle
  by construction** — measured 5.01 mm at build time, envelope and
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
  the pillar; sand/shim a proud pillar, never let it rock).  Two **M3
  heat-set insert stations** in the solid plug (Φ4.0 × 8 install
  bores, each under a shallow Φ5.5 × 0.4 melt-relief counterbore so
  insert install displacement stays BELOW the seating plane and can
  never prop the frame): the hatch perimeter screw (one M3×14 clamps
  lid → frame → pillar) and a dedicated frame screw at rho 87 (M3×10)
  so the frame stays clamped with the lid off.  Walls asserted every
  build: 3.6 mm radial edge, ≥ 3 mm tangential, 6.8 mm web between
  the bores.
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
  the lid off**; a flipped-open lever (z ≈ 22) now tops the rotating
  band floor (slab bottom, world 19.25) but clears every yaw joint's
  rotating envelope by 21.7 mm RADIALLY (envelopes reach inboard only
  to r 61.8 from centre) — asserted on every rebuild.
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
* The 14 mm coxa shortening (including the Aug-25 bearing drop) did
  **not** move these numbers: plate, hip axis and femur pivot all
  dropped together, so the relative geometry (and the limit) is
  unchanged — re-verified by the sweep after every drop.

Walking around `STANCE_FEMUR_DEG = −25` keeps ~22° of up-headroom.
**Deep tucks (stand-up belly curl, self-righting) exceed this** — check
those trajectories against −47.5° before putting this variant on the
robot.  Sweep used the stance-relative tibia angle (+75°); knee-range
variation only moves parts that stay outboard/below the plate.

## BOM delta vs production

* **30× M3×20 SHCS replace the 30 M3×30 yaw-horn screws** (5 per hub:
  4 drive + 1 centre) — **purchased-hardware change**, the only screw
  spec that differs from the production build.  The M3×20 seats are
  cut 10 mm deeper so tip depth / horn engagement is identical;
  M3×30s and M3×25s physically do not fit the Aug-25 coxa (tips
  bottom out on the servo case 10/5 mm early)
* 6805-2RS count unchanged (12): the 6 upper yaw bearings move up to
  become the 6 top-plate bearings; the 6 lower ones sit on the
  deck-level tower ledges
* **−6 `yaw_bearing_cap` prints and −18 M3×8 cap join screws** — the
  cap is deleted (see "One tower-seated bottom bearing")
* 4× M3 standoff stacks, ~76 mm (bottom sheet top z≈2 → hatch underside
  78.05; e.g. 40+36 F-F, or M3 threaded rod in printed sleeves) + M3
  screws down through the hatch into the stack tops — **non-structural**
  in this variant (hatch/electronics anchors only)
* **12× M3 brass heat-set inserts** (~5.7 mm long, e.g. Ruthex RX-M3
  or McMaster 94459A130) for the pillar tops — 2 per pillar, printed
  Φ4.0 × 8 bores with melt-relief counterbores; install with a
  soldering iron (~230 °C for PETG) before the pillars are bolted down
* 6× M3×14 button-heads for the hatch perimeter (through lid + frame
  into the pillar-top inserts; lid 4 + frame 4 + ~5.7 engagement — an
  M3×16 would bottom out in the 8 mm bore)
* 6× M3×10 for the dedicated frame→pillar screws (into the second
  insert of each pillar)
* 18× M3×12 + 18× M3 nyloc nuts for the pillar feet (belly side)
* the 140 mm `chassis_top` deck + its 20 mm standoffs are not used
* **−4 Wago 221-415**: the 6 corner + 2 trunk nuts become 4 in
  `centre_wago_block` (print 1, ~11 g, VHB pad; leg power pigtails
  ~60–70 mm longer — wiring change only)
* **6× `coxa_link_rigid` reprints** — the stock coxa neither fits the
  yaw envelope the plain columns rely on nor reaches the tower-seated
  race (same filament as 6 production coxas; the old prints become
  spares)
* **1× `chassis_bottom_rigid` reprint** (~288 cm³, same class as the
  production chassis print) — tower platforms trimmed to the tower
  cylinder, bearing pockets lowered to the deck, shaved ears, printed
  foot holes, corner Wago trays deleted.  (The Aug-25 lowered pocket
  makes the reprint effectively mandatory — see the bench-mod note
  above)
* recommended: 12× more M3 heat-set inserts for the hip cap pilots
  (drill-and-fit, see "Disassembly & service") — the pillar-top pair
  is already printed-in and counted above

## Assembly order

1. Build the robot as production but with four swaps: build on
   `chassis_bottom_rigid`; use `coxa_link_rigid` in place of the
   stock coxa; press ONE 6805 onto each coxa's hub boss from below
   until it seats against the Φ29 seat ring (the race rides the
   boss's lowest 7 mm, stopping 0.5 mm short of the boss end), then
   drop the leg + bearing into the tower pocket — the outer race
   lands on the deck-level ledge, no `yaw_bearing_cap`, no cap
   bolts; the tower swallows the full race, top flush — and couple
   the horn as production (the M3×20 drive screws go in through the
   coxa's head-access shafts from above, exactly as the M3×30s did);
   and use `hip_clamp_cap_rigid` in place of the stock hip cap —
   same 2× M3 into the same cradle pilots.
2. Re-splice power at the centre: VHB `centre_wago_block` to the floor
   centred on the origin (footprint verified/asserted against the real
   chassis solid), seat 4× 221-415, jumper each pair, land the battery
   leads through the trunk pass into the east ports and each leg's
   (lengthened) pigtails into the fan-out ports.  The 6 corner and 2
   trunk nuts are retired.  Then heat-set the two M3 inserts into each
   `corner_pillar` top (flush or a hair below — the relief counterbore
   catches the melt), sit each pillar flat on the sheet at its corner
   flat and bolt down with M3×12 + belly nylocs through the printed
   foot holes (stock chassis: the tray walls are still there — drop
   the foot between them and drill first, using the foot as the jig).
3. Press a 6805 onto each cap boss until it seats on the Φ29 pedestal.
4. Lower `chassis_top_rigid` straight down onto all six races (pockets
   are lead-in chamfered; descent path verified clear at build time —
   the pillars stop 0.1 mm short so the races seat first), press until
   the shoulders touch the race tops, then drive the 6 frame→pillar
   screws.
5. Drop `top_hatch_rigid` into the frame opening (lip registers), drive
   the 6 perimeter M3×14s into the pillar-top inserts and the 4
   standoff screws.

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
  6 outboard screws, the 6 frame→pillar screws (into pillar-top brass
  inserts — cycle them as often as you like) and the 4 standoff
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
  (belly up, same supports).  The tower-cylinder trim, the lowered
  pocket (a straight Φ44/Φ37.15 barrel, now shorter, with the ledge
  as a printed internal step), shaved ears and foot holes change
  nothing about the print strategy.
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

## Pipeline: STEP-first (the official way)

The seven variant printables are authored **once**, as build123d /
OpenCascade BREP solids in `build_rigid_hip_step.py` (this directory).
That is where geometry gets edited.  Everything else derives from it:

1. `build_rigid_hip_step.py` exports each part as `.step` (the editable
   CAD truth, for Onshape/Fusion/FreeCAD) plus a tessellated `.stl`
   into `step/stl/`, and gates each tessellation on healing into a
   closed volume (+ the above-sheet whitelist census for the chassis).
2. `make_rigid_hip_variant.py` — the **assembly/check driver** — runs
   that exporter, loads the tessellations, builds the production /
   visual meshes (femur, boots, retainer, servo bodies, COTS
   stand-ins) from `hexapod_prototype.py`, runs the full geometric
   check suite on the assembled robot, copies the printables into the
   print set (`stl/`), and writes `scene.json` + `preview.png`.

The trimesh twins of the printables were **retired** in the Aug 26
flip (user: "the official way is to make step files") — their
rationale comments live in git history before that commit.
Production parts the variant reuses stay mesh-sourced from
`hexapod_prototype.py`, unchanged.

## Directory layout: `stl/` is the print set, `step/` is the CAD truth

* `stl/` — **the print set**: slice these.  The seven variant
  printables are healed copies of the BREP tessellations; the rest are
  production/visual meshes.
* `step/` — the canonical geometry exports from
  `build_rigid_hip_step.py`: `*.step` CAD/BREP exchange files,
  `step/stl/` raw tessellations, plus `rigid_hip_manifest.json` and
  `rigid_hip_step_first_bundle.zip`.  Regenerated artifacts, not in
  git.

## Build & view

```sh
# THE build command (BREP export -> assembly checks -> print set -> scene):
uv run python concepts/rigid_hip/make_rigid_hip_variant.py
#   --skip-sweep  fast geometry iterations (placeholder femur limit)
#   --skip-brep   reuse the existing step/stl/ exports

# geometry-only re-export (the driver runs this for you):
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy --with manifold3d \
  python concepts/rigid_hip/build_rigid_hip_step.py

npx buildviz register hexapod_walker/prototype_sts3215/concepts/rigid_hip \
    --build-id sts3215-rigid-hip
# http://127.0.0.1:5183/?build=sts3215-rigid-hip
# cloud mirror: tools/push_cloud_buildviz.py --build-id sts3215-rigid-hip
```

(The separate `sts3215-rigid-hip-step` viewer build and its
`publish_step_scene.py` are retired: with the flip, the main
`sts3215-rigid-hip` build **is** the STEP-pipeline geometry.)

Checks run at build time: watertightness, seated-stack placement, the
bottom joint (race on the deck-level ledge with its 0.5 mm servo-case
clearance asserted, coxa/race contact = boss press only, seat ring
lands on the race top, ledge solid at every azimuth with the old
15.75 seat proven GONE, rim proven AT the race-top plane with air
above it — nothing continues the Φ44 column — brim present with its
0.5 running gap open, leg + bearing lift-out path), the shortened
coxa column (well floor at its computed plane, 1.25 mm head clearance
real, horn-screw seat planes and shank passages open at the
bench-pinned depths, the seat-drop == length-delta engagement guard,
sub-slab silhouette inside the seat-ring + brim envelope, slab clear
of the tower rim), the chassis variant (nothing outboard survives
past the tower cylinder, the trim never bit the tower wall, nothing
past the tower cylinder above the servo-mount deck at any azimuth up
to the rim with the az-210 ear + its root proven gone, the rebuilt
full-wrap ring complete on all six towers, foot holes open where the
pillar feet expect them, all six Wago tray wall sets gone with the
sheet still solid underneath, the whole rev 5+6 flatten band proven
air outside the tower keep with the sheet intact below it and the
in-keep shell + pocket floor still standing, the rev-7 flank
cylindricity: no vertex within 45 mm of any yaw axis pokes past the
trim cylinder anywhere between the sheet top and the rim, and the
rev-8 above-sheet whitelist: no vertex above z 2.05 anywhere on the
part outside the six tower cylinders — also asserted against the
STEP-derived STL at export time), full 360° yaw sweep vs
the plate,
straight-down plate descent over all six bearings, pillar clearances
(seated robot, ±45° operating yaw with margin, and an informational
full hand-spin scan) and the femur pitch×yaw contact sweep (fails the
build if the safe limit ever eats into walking headroom at −45°).
