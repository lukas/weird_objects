# Hexapod Walker — Tabletop Prototype Build Guide

> A scaled-down sibling of the [full-size walker](../ASSEMBLY.md) intended
> for proving out the geometry, kinematics, and gait controller before
> you commit to industrial servomotors. Same architecture: regular hex
> chassis, six identical 3-DOF legs, alternating-tripod gait — but
> everything is shrunk roughly 6× and every joint is driven by a
> generic 25 kg·cm hobby servo (DS3225 case/geometry) instead of a
> $5000 harmonic-drive servomotor.
>
> Total parts cost: **~$150 – $300** in 2026 USD. A motivated builder
> can have it walking on a tabletop in a weekend.

![Cycles render of the prototype hexapod](renders/prototype.png)

---

## 1. Why a separate prototype?

The full-size walker is a serious build: a 280 kg vehicle, $40 k+ BOM,
custom CNC and casting work. Even if your end goal is a rideable
walker, you almost certainly want to:

1. **Validate the kinematics and gait controller end-to-end** on cheap
   hardware before machining anything in aluminum.
2. **Iterate on the leg link ratios** without a 6-week lead time on
   castings — you can re-print every link in 12 hours.
3. **Train the gait scheduler in simulation, then prove it on real
   hardware** at 1/200 the mass and 1/250 the cost.
4. **Show the design works** to collaborators, funders, or your spouse
   before committing to the bigger build.

The prototype is **mechanically and kinematically identical** to the
full-size walker. The same six-legged tripod gait, the same per-joint
PID + feed-forward controller architecture, even the same coxa : femur
: tibia ~ 1 : 4 : 5 ratio. You can re-use the gait code unchanged
when you graduate to the big version — just re-tune the gains.

---

## 2. Specification at a glance

| Property | Value |
|---|---|
| Configuration | 6 legs × 3 DOF, alternating-tripod gait |
| Overall envelope (foot-to-foot, fully extended) | ~ 580 × 670 × 110 mm |
| Standing height (chassis underside) | ~ 70 mm |
| Stride length | ~ 120 mm |
| Cruise speed (tripod gait) | ~ 80 mm/s |
| Vehicle dry mass | ~ 1.3 kg |
| Per-leg static load (tripod stance) | ~ 4.3 N (~ 0.43 kg) |
| Peak knee torque | ~ 0.6 N·m (~ 6 kg·cm) |
| Battery | 1 × 3S 2200 mAh LiPo (11.1 V → 5 V BEC) |
| Continuous draw (cruise) | ~ 1.5 A @ 5 V |
| Run time, level ground | ~ 30 min |

**Knee torque margin:** the design target is a DS3225 25 kg·cm metal-gear
digital servo.  At 6.8 V it gives a ~ 4× safety factor over the
worst-case knee torque.  The printed wells, tab pilots and RL servo
torque limits are all tuned around this DS3225-class case.

> **Design B (May 2026):** the printed `servo_horn_adapter` disc has
> been retired.  Each link now bolts directly onto the plastic 4-arm
> X-horn that ships with the servo, via **4 x M2 x 8 SHCS** (McMaster
> `91290A005`) through Phi `XHORN_BOLT_OD = 2.2 mm` holes on a
> `XHORN_BOLT_PCD = 20.8 mm` PCD plus a 16 mm x 1.6 mm central hub
> recess cut into the link's pad face.  Drops the printed-leg-bolt-up
> Z stack by ``HORN_ADAPTER_T`` (4 mm) per joint.
>
> **Why M2, not M3 (May 2026 fastener-spec fix):** a previous iteration
> drew these as M3 SHCS through Phi 3.2 mm clearance holes, which is
> physically WRONG -- the plastic 4-arm X-horn that ships with
> DS3225 / MG996R / DS3218-class servos has Phi ~ 2.0 mm UNTAPPED
> through-holes in its arms (intended for M2 self-tap per the
> manufacturer's mounting style), NOT M3 clearance.  An M3 SHCS
> literally will not fit through the X-horn's plastic arm.  The link
> pad's clearance bore is now Phi 2.2 mm (M2 + 0.2 mm FDM tolerance);
> the bolt self-taps into the X-horn's Phi ~ 2.0 mm pilot for
> `XHORN_BOLT_THREAD_ENGAGEMENT_MM = 3 mm` of engagement against the
> plastic arm.  Mechanically symmetric to the cradle bolt fix in
> `b447f88` (and the May 2026 heat-set follow-up; see Design D
> below): the cradle bolts stay M3 x 8 SHCS, only the **X-horn
> bolts** split off into their own SKU here.  See
> `fasteners/README.md` ("X-horn bolts are also self-tappers") for
> the optional M2 thread-forming upgrade (`99461A340`).
>
> **Design C (May 2026, reverted):** the servo is bolted into its
> cradle via 4 x **M3 x 8 SHCS driven vertically downward** through
> each servo ear and self-tapped into a **Phi 2.5 mm pilot hole** in
> the cradle shelf below.  This matches the **standard hobby-servo
> mounting style** for DS3225 / MG996R / SG90-class servos (the same
> scheme the pre-2026 prototype already used).  Pilot depth into the
> shelf = `SHCS_THREAD_ENGAGEMENT_MM` (= 8 mm), which yields ~4
> cycles of M3 thread engagement in PA12 / PLA.  Counts: **72 x M3
> x 8 SHCS** (reusing the existing link-to-X-horn SHCS stock --
> McMaster 91290A113 -- so there is **no new fastener SKU**).
>
> The brief horizontal-bolt + captive-nyloc-nut iteration that
> initially shipped under the "Design C" banner was retired after an
> audit found (a) the mounting axis on a real DS3225 is vertical
> (the ears mount with ear bolts going through the tab _along the
> motor's height axis_), (b) the +X hex pocket geometry collided with
> the wire channel routed along the cradle's outer +X wall, and (c)
> the remaining outer-wall thickness at the hex pocket's flats fell
> below `MIN_PRINT_T` (3 mm) on FDM PA12 / PLA at the existing wall
> thickness.  See the May 2026 commit history for the full audit.
>
> The matching **`coxa_bracket` drop-in slot Z fix** landed in the
> same change: `slot_z_min` was raised from `-6.0` to
> `BRACKET_SLOT_Z_MIN_RIB_CLEAR = -3.0`, so the slot now just clears
> the cradle's internal structural rib instead of biting 6 mm deep
> into the well wall.  The yaw cradle's effective shelf top moved
> UP by ~3 mm (closer to the nominal `WELL_RIM_Z` shared with the
> hip and knee cradles).
>
> **Design D (May 2026, current):** the brief Design C self-tap
> pilots (Phi `INSERT_M3_PILOT_OD` = 2.5 mm pilot through the printed
> shelf) were audited and found to be **structurally inadequate**.
> The audit measured the plastic remaining between the pilot wall
> and the nearest air gap in the +/-Y direction at all 12 cradle
> sites (4 bolts x 3 cradles); 7 of 12 sites had between **0.00
> and 1.50 mm of plastic** on at least one side (femur and
> coxa-link knee/hip cradles' outer walls don't extend past the
> servo body footprint, and the +X column collides with the
> existing wire channel).  The original `check_cradle_pilot_holes`
> only verified that the pilot cylinder existed; it never probed
> radially for surrounding material.
>
> The fix is to switch from "M3 self-tap into Phi 2.5 mm plastic
> pilot" to "**M3 SHCS into a heat-set brass insert** in a Phi
> `INSERT_M3_PILOT_OD` = 4.0 mm pocket, surrounded by a Phi
> `CRADLE_BOSS_OD` = 8.0 mm printed boss" (parameters in
> `hexapod_prototype.py` -- see the `INSERT_M3_*` / `CRADLE_BOSS_*`
> constant block).  The heat-set part is **McMaster 94459A130** (M3
> knurled brass, Phi 4 mm pilot, Phi 5.7 mm knurled OD, 5 mm length,
> ~ $0.10 ea).  This (a) gives real metal threads instead of
> stripping into PLA / PA12 after one cycle, and (b) the boss
> enlargement automatically solves the radial-engagement bug --
> every pilot now has at least `CRADLE_BOSS_MIN_WALL_MM` = 1.5 mm
> of plastic in every direction.
>
> **Cradle servo mount install order:**
> 1. Print the cradle (`coxa_bracket`, `coxa_link`, or `femur_link`).
> 2. Heat a soldering iron to ~ 220 deg C.
> 3. Drop a 94459A130 insert (knurled end down) into one of the four
>    Phi 4 mm pockets in the cradle boss top.
> 4. Touch the iron tip to the insert's flat (top) face and apply
>    **light downward pressure** for ~10-15 s, until the insert's
>    top face sits ~0.5 mm below the printed boss top (so the
>    eventual bolt head clamps the servo ear onto the printed
>    plastic, not onto the brass).
> 5. Let the insert cool for ~30 s.  Repeat for the other 3 pockets
>    in this cradle.
> 6. Drop the DS3225 servo into the cradle.
> 7. Thread an M3 x 8 SHCS through each servo ear into the brass
>    insert below and torque to ~ 0.5 Nm.
>
> Counts (per robot): **72 x M3 x 8 SHCS** (same SKU
> `91290A113` as before; spec string now
> `M3x8 SHCS into heat-set insert`) and **72 x M3 heat-set inserts**
> (new SKU `94459A130`).  Net SKU count grows from 6 -> 7.
> Verifier additions: `check_cradle_insert_pockets` now probes the
> Phi 4 mm pocket, an 8-azimuth **radial-material ring** at
> r = `INSERT_M3_PILOT_OD/2 + CRADLE_BOSS_MIN_WALL_MM` = 3.5 mm (the
> check that would have caught the original bug), and an 8-azimuth
> **heat-set knurl displacement ring** at r = 2.42 mm to confirm
> the boss has enough plastic for the brass knurl to displace.  See
> `_verify_prototype.py`'s `check_cradle_insert_pockets` body for
> the probe coordinates.
>
> **General rule for future printed-feature threaded joints:** any
> printed feature that takes a threaded fastener for **repeated
> assembly** (i.e. anything the user expects to disassemble and
> reassemble) MUST use a heat-set insert, not a self-tap pilot,
> unless the joint is one-time-use (e.g. the foot pad, chassis
> hardware).  Documented in `CAD_AGENT_INSTRUCTIONS.md`.

---

## Visual inspection

Three viewers share the same `part_palette.PART_COLORS` table, so a
given part type (e.g. `coxa_link` = green, `coxa_bracket` = orange,
`servo_body` = dark gray) is rendered with the *same* color in each:

- `make view-static`  — MuJoCo viewer, prototype frozen in stance pose
  (now colored by part type, body labels on by default).
- `make view`         — MuJoCo viewer, full physics + tripod gait.
- `make view-build`   — PyVista build inspector, colored + labeled
  parts, exploded-view slider, per-part-type hide toggle.

Forward extra flags through `ARGS=`, e.g.
`make view-build ARGS="--explode 1.0"` opens the inspector half-way
exploded, or
`make view-build ARGS="--screenshot artifacts/views/build_inspect.png"`
renders a single headless PNG and exits.

### PyVista build inspector controls

The `view-build` window prints these to stdout on launch:

**Mouse**

- **Hover** a part → its `part_type Lx  role` is shown in the top-of-window banner. No clutter from other parts.
- **Left-click** a part → every other part dims to 15% opacity while the picked part stays in place at full opacity. The clicked part does not move, so you can see exactly how it sits in the assembly without its neighbours obscuring it.
- **Double-click** a printable part → enter *focus-on-sub-assembly* mode: hide everything except the picked part, the servo that sits in its cradle (if any), the X-horn it bolts onto (if any), and every fastener whose role passes through it. The camera auto-fits the union bounds of the surviving group so the sub-assembly is unobstructed. Double-click the same part (or any empty space) to clear focus.
- **Left-click in empty space** (or press `I` / `Esc`) → restores every part's opacity to its full value (and clears focus if it was set).
- Left-drag still rotates the camera. Right-drag pans. Scroll zooms.

**Keyboard**

| Key | Action |
|---|---|
| `L` | Toggle the persistent all-labels-at-once cloud (off by default — hover is usually less noisy) |
| `E` | Toggle exploded view between 0.0 and 1.5 |
| `F` | Focus the **hovered** part's sub-assembly (same effect as double-click). Press `F` again on the same part (or on empty space) to clear focus. |
| `I` / `Esc` | Clear focus first, then isolation |
| `R` | Reset the camera view |
| `S` | Save a screenshot to `artifacts/views/build_inspect.png` |
| `Q` | Quit |

The bottom slider (0.0 – 2.0) scales each part's distance from the
chassis centroid for finer exploded-view control than the `E` toggle,
and the row of color-coded checkboxes along the left edge hides or
shows all instances of a given part type so you can isolate, say,
just the coxa brackets or just the servo bodies. While focus mode
is active the per-part-type checkboxes are suspended (focus mode
wins); their last setting is restored when focus clears. The
"fasteners" master toggle while focused hides only the fastener
members of the current sub-assembly, leaving the focus state
otherwise unchanged. The MuJoCo viewers' body-name labels default
to ON (toggle with `B` once inside the viewer).

For headless captures of one sub-assembly, pass `--focus
PART_TYPE/L<n>` (or just `PART_TYPE` for chassis-level parts), e.g.

```bash
make view-build ARGS="--focus coxa_link/L0 --screenshot /tmp/focus.png"
make view-build ARGS="--focus femur_link/L3 --screenshot /tmp/femur.png"
make view-build ARGS="--focus chassis_top    --screenshot /tmp/chassis.png"
```

`--focus` composes with `--explode` so you can render the focused
sub-assembly half-exploded too.

If `view-build` errors out saying STLs are missing, run
`make build` first to regenerate `stl_prototype/`.

---

## How to validate / regenerate

The STLs under `stl_prototype/` are derived artifacts — the source of
truth is `hexapod_prototype.py` (geometry) plus `design_spec.yaml`
(human-readable contract). After any parametric edit, run the
validate → render → report pipeline from the repo root:

```bash
make -C hexapod_walker/prototype check-cad        # full validate + render + report
make -C hexapod_walker/prototype check-cad-fast   # inner-loop variant
```

Both targets rebuild `stl_prototype/`, run the validators, render
4-view PNGs per part under `artifacts/views/`, and write
`artifacts/cad_report.md`. See [`CAD_WORKFLOW.md`](CAD_WORKFLOW.md) for
the full pipeline and [`CAD_AGENT_INSTRUCTIONS.md`](CAD_AGENT_INSTRUCTIONS.md)
for the rules LLM coding agents should follow when editing CAD.

The fastener cache STLs under `fasteners/` are *separately*
regenerated by `make regen-fasteners`, which invokes the OpenSCAD
CLI against the NopSCADlib-backed `.scad` files in `fasteners/scad/`
to produce ISO-spec hex-socket / pan-head / nyloc-nut geometry.  See
[`fasteners/README.md`](fasteners/README.md) for the priority chain
(real McMaster STEP -> user STL -> OpenSCAD/NopSCADlib -> parametric).

The legacy verifier `_verify_prototype.py` is parallelised via a
process pool and exposes five extra CLI flags for faster inner-loop
iteration:

| Flag | What it does |
|---|---|
| `--serial` | Skip the process pool entirely; run every check in the main process in declaration order. Use this when a worker traceback is mangled through pickle and you need a clean stack. |
| `--workers N` | Override the default worker count (default `min(8, os.cpu_count())`). |
| `--profile PATH` | Dump a cProfile snapshot of the parent process to `PATH` when the run finishes (combine with `--serial` to profile the entire suite in one process). |
| `--only CHECK_NAME` | Run only the named check(s). Repeatable: `--only "Servo clearance" --only "Workspace self-collision"`. Names match the declaration list (see top of `_verify_prototype.py` `CHECKS`). |
| `--inside-mode {rays,contains,both}` | Select the implementation used by `points_inside()`. Default is `rays` — the historical 6-axis ray vote that is robust against the boolean-union false positives we saw on early prototype geometry. `contains` swaps in `trimesh.Trimesh.contains()` (much faster, valid only on watertight meshes). `both` runs BOTH implementations on every probe, records each disagreement, and exits with code `2` if any are found — use this whenever you change the geometry, the trimesh stack, or any of the post-transform mesh hygiene assumptions to re-validate that the two implementations still agree. |

All five flags compose with the existing `--with-arm` flag. Parallel
output is printed in declaration order so a `diff` against the serial
baseline is byte-for-byte clean for every `[PASS]/[FAIL]` line.

### Why `--inside-mode rays` stays the default

A one-time `--inside-mode both` sweep across the full check suite
(May 2026) probed 1,819,776 points and found that the 6-axis ray vote
and `trimesh.Trimesh.contains()` DISAGREE on roughly 1.8 % of them
(32,685 mismatching points) even though every cached mesh passes
`check_watertight`. The mismatches concentrate in the standing-pose
self-collision and the workspace sweep where leg parts are probed
AFTER ``apply_transform``: at fine voxel pitches a non-trivial number
of probe points sit on or just inside surfaces, and the two
implementations classify those borderline points differently. The
ray vote is the more conservative answer for clearance checks (it
labels more points as INSIDE → flags more potential interference),
which is what we want for a print-once / order-once verifier, so we
leave it as the default. Re-run `--inside-mode both` whenever the
geometry pipeline changes and treat any rise in the mismatch count
as a signal that the two implementations have drifted.

### `check_screwdriver_access` envelopes

The screwdriver-access check probes a per-fastener cylindrical
clearance cone above each head along the driver-approach axis and
fails if any printed part intrudes by more than 30 mm³. Three
envelopes are dispatched off `FastenerInstance.spec`:

| Envelope | Dia × length | Used for |
|---|---|---|
| `HEX_KEY`  | 8 mm × 30 mm  | SHCS (M2x8 / M3x8 / M3x32) + the M2.5 spline center screw — anything driven with an L-shaped hex key short arm |
| `PHILLIPS` | 12 mm × 80 mm | `pan-head` / `Phillips` / `slotted` — currently just the M3x16 foot hinge bolt |
| `SOCKET`   | 12 mm × 50 mm | `nyloc nut` / generic `nut` — M3 nyloc driven with a 5.5 mm nut socket |

Spline center screws are technically a small Phillips on hobby
servos, but every such fastener is also explicitly SKIPped (captive
under the X-horn after assembly), so they're mapped to `HEX_KEY`
purely so the per-spec envelope table reads consistently. See the
top of `check_screwdriver_access` in `_verify_prototype.py` for the
exact dispatch logic.

---

## 3. STL files in `stl_prototype/`

Run the one-command prototype builder to generate every print/export bundle:

```bash
./run.sh hexapod_walker/prototype/build_all.py
```

For only the individual slicer-ready part STLs, run
`./run.sh hexapod_walker/prototype/hexapod_prototype.py`. All dimensions are
in millimetres. All individual STLs are sized to fit a 220 × 220 mm
3D-printer bed (Ender 3 / Bambu A1 mini class).

### 3.1 Body parts (one of each)

| File | Function | Suggested print settings |
|---|---|---|
| `chassis_top.stl` | Top hex deck (4 mm PLA, 200 mm flat-to-flat) | 0.2 mm layer, 25% gyroid infill, 4 walls |
| `chassis_bottom.stl` | Identical bottom plate | same as top |
| `battery_holder.stl` | Open-top tray for one 3S 2200 mAh LiPo | 0.2 mm layer, 20% infill |
| `electronics_tray.stl` | Mount plate for Arduino + PCA9685 | 0.2 mm, 20% infill, 2 walls |

### 3.2 Per-leg parts (print 6 sets)

| File | Function | Print orientation |
|---|---|---|
| `coxa_bracket.stl` | Horizontal flange + servo well (yaw motor hangs below). 4 vertical M3 bolts clamp the flange between the two chassis plates. | Flange on bed, well opening up |
| `coxa_link.stl` | U-arm driven by the yaw servo's horn; carries the hip-pitch servo in a side-loaded well. | Hub face down, well opening up |
| `femur_link.stl` | I-beam thigh with a slot through the spar so the knee servo body can slide past it during assembly. Top + bottom flange bridges connect the spar to the well. | Spar's broad face flat on bed; knee cradle sticks up with its opening pointing DOWN (closed cradle floor at the top of the print). |
| `tibia_link.stl` | Shin link with knee pad and foot socket at the far end. | Flat on bed |
| `foot_pad.stl` | Compliant foot — print in TPU for grip | Hub up, no supports |

### 3.3 Visualization (do not print)

| File | Function |
|---|---|
| `assembly_preview.stl` | All parts placed in standing pose. Open in MeshLab or Cursor's STL viewer to sanity-check before printing. |

### 3.4 Don't have a 3D printer? Order from a print service

Run `./run.sh hexapod_walker/prototype/prepare_xometry_upload.py` to build a
self-contained order package in `xometry_upload/`. The script
re-orients each part for printing (hollow servo pockets opening
toward +Z, broadest flat face on the build plate), consolidates the
two identical chassis plates into a single file with `qty=2`, and
emits a `manifest.csv` listing the recommended material, color, and
finish for every part. See `xometry_upload/README.md` for the full
upload-and-checkout flow on Xometry, Shapeways, JLCPCB, etc.

A complete bundle runs **~ $580 in MJF PA12** (Xometry, mid-2026)
versus **~ $20 in PLA filament** if you have access to an FDM
printer.

---

## 4. Bill of materials

### 4.1 Actuators (the long pole)

| Item | Spec | Qty | Approx. cost |
|---|---|---|---|
| Hobby servo | **DS3225 25 kg·cm metal-gear digital servo**, standard 40 × 20 × 38 mm case, 54 mm tab span, ~49.5 mm tab-hole spacing, output offset ~10 mm from case centre. Buy all 20 from the same listing/batch. | 18 + 2 spare | ~$13 each on AliExpress, ~$18 each on Amazon ($260 – $360 total) |

### 4.2 Power

| Item | Spec | Qty | Cost |
|---|---|---|---|
| LiPo battery | 3S 2200 mAh 25C, XT60 connector | 1 | $20 |
| BEC (5–6 V regulator) | 5 V 5 A switching, 3S input | 1 | $8 |
| LiPo charger | iSDT D2, B6AC, or any decent 3S balance charger | 1 | $30 |
| LiPo bag | Fire-safe charging | 1 | $10 |

### 4.3 Control electronics

| Item | Spec | Qty | Cost |
|---|---|---|---|
| Arduino Mega 2560 (clone) | ATmega 2560, 5 V | 1 | $15 |
| PCA9685 16-channel PWM driver | I²C, 12-bit | 2 | $4 each |
| MPU-6050 IMU | 6-DOF gyro + accel, I²C (optional but useful) | 1 | $4 |
| Jumper wires | F-F, 20 cm × 50 + servo extensions × 18 | — | $15 |
| Logic-level wiring + heat-shrink | — | — | $5 |

### 4.4 Fasteners

| Item | Qty | Notes |
|---|---|---|
| **M2 × 8 mm socket-head cap screws** (McMaster `91290A005`) | 72 + spares | **Link → X-horn bolts.** 4 per joint x 18 joints = 72.  Used as **self-tappers** into the plastic 4-arm X-horn's existing Phi ~ 2.0 mm M2-sized untapped arm holes -- the link's pad clearance bore is Phi 2.2 mm.  An earlier draft listed these as M3 x 8 but the X-horn's arms physically won't take an M3 shank -- see the Design B blurb in §2 for the May 2026 user-caught fastener-spec fix.  Optional thread-forming upgrade: `99461A340` (M2 x 8 thread-form for plastic). |
| M3 × 8 mm socket-head cap screws (McMaster `91290A113`) | 72 + spares | **Cradle servo-mount bolts into heat-set inserts** (Design D, May 2026).  4 per servo x 18 servos = 72, driven vertically through each servo ear into an M3 brass heat-set insert (McMaster `94459A130`) installed in a Phi 4 mm pocket in the cradle boss below.  Same physical SKU as the link-to-X-horn bolts pre-Design B; the spec string in the registry is `M3x8 SHCS into heat-set insert` so the verifier and BOM can tell cradle bolts apart from any future M3 x 8 use. |
| M3 brass heat-set inserts (McMaster `94459A130`) | 72 + spares | **Cradle servo-mount thread carriers** (Design D, May 2026).  M3 knurled brass insert, Phi 4 mm pilot, Phi 5.7 mm OD, 5 mm length.  Installed BEFORE the servo with a soldering iron at ~220 deg C, light downward pressure, ~10-15 s per insert.  See PROTOTYPE.md Design D blurb for the audit table that motivated the switch from self-tap pilots. |
| M3 × 12 mm | 24 | Standoffs between top + bottom chassis plates |
| M3 × 16 mm | 24 | Coxa bracket → chassis (4 × 6 = 24) |
| M3 nyloc nuts | 30 + spares | 24 on the coxa-bracket-to-chassis through-bolts plus 6 on the foot-pad hinge pins. The cradle servo mounts thread into brass heat-set inserts (Design D, May 2026) and do **not** use a nut. |
| M3 × 25 mm round standoffs (M-F) | 8 | Top-to-bottom chassis spacers |
| M2.5 self-tappers (horn → spline) | 18 | Ships with the servos.  Holds the plastic 4-arm X-horn onto the servo output spline. |

### 4.5 3D-printed material

| Item | Spec | Qty | Cost |
|---|---|---|---|
| PLA filament | 1.75 mm, any colour | ~ 250 g | $5 |
| TPU filament | 1.75 mm, 95A | ~ 50 g (foot pads only) | $5 |

### 4.6 Total

| Bucket | DS3225 build |
|---|---:|
| Actuators | ~$300 |
| Power | ~$70 |
| Electronics | ~$50 |
| Fasteners + filament | ~$20 |
| **Total** | **~ $440** |

---

## 5. Print plan

A single Ender 3-class printer runs the whole BOM in roughly **21 hours**:

| Pass | Parts | Bed | Time |
|---|---|---|---|
| 1 | 6 × coxa_bracket | 6 brackets pack onto a 220 mm bed | ~ 4 h |
| 2 | 6 × coxa_link | Same | ~ 4 h |
| 3 | 6 × femur_link | Same | ~ 5 h |
| 4 | 6 × tibia_link | Same | ~ 4 h |
| 5 | chassis_top + chassis_bottom + battery_holder + electronics_tray | ~ 4 h |
| 6 | 6 × foot_pad (TPU) | Single bed | ~ 1 h |

Tip: use 4 walls and 25 % gyroid infill for the bracket and link
parts — the servo cradles see the most load and the extra walls add
substantial stiffness for very little weight.

---

## 6. Assembly sequence

Allow ~ 4 hours for a first build, ~ 90 min for a second.

### 6.1 Per-leg sub-assembly (do all 6 in parallel)

> Both the **coxa bracket**, the **coxa link** (hip-pitch cradle), and
> the **femur link** (knee cradle) now use the same **Design D
> (May 2026)** servo-mount: 4 x M3 x 8 SHCS thread vertically DOWN
> through each servo ear into an M3 brass heat-set insert (McMaster
> `94459A130`) installed in a Phi 4 mm pocket in the cradle boss
> below.  The boss is Phi 8 mm in OD and 10 mm tall -- big enough
> that the audit's radial-material check (8 azimuths x 3.5 mm) is
> satisfied at every bolt site.  There are **no printed-in nut
> traps**, no horizontal +/-X clearance holes, and no captive nuts in
> the cradle.  Earlier May 2026 horizontal-nyloc (Design C, retired)
> and vertical self-tap (Design C revert, retired) iterations were
> both audited and replaced -- see the Design D blurb in §2 for the
> audit detail.
>
> **Press the heat-set inserts in BEFORE you drop the servo into the
> cradle**: heat a soldering iron to ~ 220 deg C, set a 94459A130
> insert (knurled end down) into one of the four Phi 4 mm pockets in
> the cradle boss top, and press the iron tip lightly DOWN onto the
> insert's flat face for ~ 10-15 s until the top face sits ~ 0.5 mm
> below the boss top.  Cool ~ 30 s.  Repeat for all 4 pockets.

1. **Yaw servo into coxa bracket:** press 4 x M3 heat-set inserts
   (`94459A130`) into the four Phi 4 mm pockets in the bracket's
   yaw-cradle boss top (see the heat-set install note above).  Drop
   the yaw servo straight DOWN through the body cutout in the
   bracket flange and into the well below; the servo's tabs land
   flush on the well rim, with the gear stack and output spline
   poking UP above the flange.  Drive 4 x M3 x 8 SHCS straight down
   through the ear's Phi 3.2 mm clearance hole into the brass
   insert in the boss below.  **Torque finger-tight + ~ 1/4 turn**:
   the brass thread is real metal but the cradle boss material that
   surrounds it is PLA / PA12 -- over-torque can still pull the
   insert out of the boss.
2. **Plastic horn on the yaw servo:** centre the servo, push a stock
   plastic 4-arm X-horn onto the spline at 0 deg, then secure it with
   the M3 horn-attach screw that ships in the servo bag.
3. **Coxa link:** drop the link's hub pad onto the X-horn -- the 16 mm
   recess on the underside of the pad seats the horn's central hub --
   and bolt the link to the horn with **4 x M2 x 8 SHCS** (McMaster
   `91290A005`) through the Phi 2.2 mm clearance holes on the
   20.8 mm PCD.  The bolts self-tap into the X-horn's existing
   Phi ~ 2.0 mm M2-sized arm holes; finger-tight + ~ 1/4 turn (the
   plastic arm is the structural thread -- over-torque strips it).
4. **Hip-pitch servo:** press 4 x M3 heat-set inserts (`94459A130`)
   into the coxa link's hip-cradle boss pockets, drop the hip servo
   into the cradle, and bolt it down with the same vertical M3 x 8
   into-insert pattern as the yaw servo (4 x M3 x 8 SHCS straight
   DOWN through each servo ear into the brass insert in the boss
   below).  Fit a plastic 4-arm X-horn perpendicular to the leg arm
   so the femur swings up and down.
5. **Femur:** seat the femur's hip-end pad on the hip horn (16 mm
   recess engaging the horn hub) and bolt to the horn with **4 x M2
   x 8 SHCS** (same `91290A005` stock as step 3; self-tap into the
   plastic horn arm).
6. **Knee servo:** press 4 x M3 heat-set inserts (`94459A130`) into
   the femur's knee-cradle boss pockets, drop the knee servo into
   the cradle, and bolt with 4 x M3 x 8 SHCS (vertical into the
   brass inserts).  Plastic X-horn perpendicular to the femur spar.
7. **Tibia:** seat the tibia's knee-end pad on the knee horn and
   bolt to the horn with **4 x M2 x 8 SHCS** (same `91290A005` stock).
8. **Foot pad:** push-fit into the tibia's foot socket, glue with CA
   if it's loose.

You now have a complete leg dangling from a coxa bracket. Repeat 6
times.

### 6.2 Final assembly

9. **Bottom chassis plate:** lay flat. Each coxa bracket's flange
   bolts to the chassis edge with **4 × M3 × 16 mm** caps — drive
   them straight down through the flange and through the matching
   bolt pattern in the chassis plate, capture with a nyloc on the
   underside. The two outboard bolts sit just inside the chassis
   perimeter; the two inboard bolts sit 16 mm further in. The flange
   sandwiches between the bottom plate and the standoff ring later
   in step 13.
10. **Stand-off posts:** screw 4 × M3 × 25 mm M-F standoffs into the
    inner bolt pattern.
11. **Battery holder + electronics tray:** bolt to the bottom plate
    using the same inner bolt pattern (the battery holder feet share
    the standoff bolt pattern, the tray sits adjacent).
12. **Wire it up:** see §7.
13. **Top chassis plate:** screw down onto the M3 standoff tops.

---

## 7. Wiring

### 7.1 One-servo bench test first

Before assembling the robot, test **one DS3225 + one PCA9685 + Arduino**
on the bench. This proves the exact servo listing you bought fits the
electrical and software assumptions before you print/bolt all 18 joints.

Bench wiring:

```text
Raspberry Pi / laptop  --USB serial-->  Arduino Mega
Arduino Mega SDA/SCL  ------------->  PCA9685 SDA/SCL
Arduino Mega 5V/GND   ------------->  PCA9685 VCC/GND  (logic only)
5-6 V BEC + / -       ------------->  PCA9685 V+ / GND (servo power)
DS3225 signal/+/-     ------------->  PCA9685 channel 0
```

Important: **all grounds must be common**: Pi USB ground, Arduino ground,
PCA9685 ground, and BEC/servo ground. Do **not** power the DS3225 from
the Arduino 5 V pin.

Upload the Arduino bridge sketch:

```bash
hexapod_walker/prototype/firmware/prototype_servo_bridge/prototype_servo_bridge.ino
```

Then from the Raspberry Pi or laptop:

```bash
python -m pip install pyserial
python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 centre
python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 0
python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 joint 0 20 --sweep
```

On macOS the port will look like `/dev/cu.usbmodem...`; on Raspberry Pi /
Linux it is usually `/dev/ttyACM0` or `/dev/ttyUSB0`.

Once one servo works, plug the same servo into channels 1 and 2 and run:

```bash
python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 1
python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 2
```

That validates the yaw / hip / knee channel order for one leg before
you connect all 18 servos.

### 7.2 Full robot wiring

```
                    +--------------+
                    |  Arduino     |
                    |  Mega 2560   |
                    +-+-+----+--+--+
                      | |    |  |
                  SDA/SCL   D2-D5
                      | |    |  |
            +---------+ |    |  +-- IMU MPU-6050 (I2C, addr 0x68)
            |           |    |
       +----+----+ +----+----+
       | PCA9685 | | PCA9685 |   I2C addr 0x40 + 0x41 (jumper)
       |  #1     | |  #2     |
       | ch 0-15 | | ch 0-1  |
       +-+-+-+...+ +-+-+-+...+
         | | |       | | |
        18 PWM lines to the servos:
            #1 ch  0-2  -> leg 0 (yaw, hip, knee)
            #1 ch  3-5  -> leg 1
            #1 ch  6-8  -> leg 2
            #1 ch  9-11 -> leg 3
            #1 ch 12-14 -> leg 4
            #1 ch 15    -> leg 5 yaw
            #2 ch  0-1  -> leg 5 hip + knee

  Power side (separate from logic):
            +-----------+
            | 3S LiPo   |  (XT60 -> on/off switch -> XT60 splitter)
            +-----+-----+
                  |
        +---------+---------+
        |                   |
     5V 5A BEC          5V 5A BEC
   (servo rail #1)    (servo rail #2)
        |                   |
        +-> PCA9685 #1 V+   +-> PCA9685 #2 V+
            (powers 9 servos)   (powers 9 servos)
```

Two BECs / two PCA9685s let you split the 18-servo current draw
across two regulators. A single 5 A BEC will brown out under
synchronous-walk current spikes (peak ~ 6 – 8 A across all 18 servos).

The Arduino gets its 5 V from one BEC's auxiliary output (or USB
during development) — DO NOT power the Arduino off the LiPo's raw
11.1 V via the barrel jack, the on-board regulator can't handle the
combined logic + sensor load.

---

## 8. Software

A starter Arduino sketch (~ 200 lines) is enough to walk the
prototype. You will need:

```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1(0x40);
Adafruit_PWMServoDriver pwm2(0x41);

// Pulse-width mapping (DS3225: 500 us = -90 deg, 2500 us = +90 deg)
const float PWM_MIN_US = 500.0;
const float PWM_MAX_US = 2500.0;

// Per-joint trim (calibrate after first power-up)
float trim_deg[18] = { /* ... */ };

void writeJoint(int joint_idx, float angle_deg) {
    Adafruit_PWMServoDriver& drv = (joint_idx < 16) ? pwm1 : pwm2;
    int chan = joint_idx % 16;
    float us = PWM_MIN_US
             + (angle_deg + 90 + trim_deg[joint_idx]) / 180.0
             * (PWM_MAX_US - PWM_MIN_US);
    drv.writeMicroseconds(chan, (int)us);
}
```

Then add a ~ 50 Hz loop that runs the inverse kinematics for each leg
(closed-form 3R IK — coxa yaw, hip pitch, knee pitch — there's a
classical solution; see e.g. the "Phantom X / Lynxmotion AX hexapod"
references) and a tripod-gait scheduler.

If you want to use ROS instead of bare-metal Arduino, swap the Arduino
for a Raspberry Pi 4 running `ros2_control` + the same PCA9685
driver, and the same gait code (Python or C++) you intend to use on
the full-size walker.

---

## 9. Tuning notes

* **Per-joint trim:** the first thing you do after assembly is set
  every joint to its "neutral" angle (femur and tibia horizontal,
  pointing radially out) and adjust the `trim_deg` array until each
  link physically lines up. Mechanical horn-spline mounting introduces
  ~ 14 ° of quantization, so you'll see ~ 7 ° trim on each joint.
* **Pulse-width range:** check the actual mechanical end-stops by
  sweeping each joint slowly from 500 µs to 2500 µs. Cheap servos
  often only achieve ~ 160 ° of travel, not the advertised 180 °, so
  clamp `angle_deg` to ±80 ° in software.
* **Gait period:** start at 2 s per cycle (very slow, easy to debug),
  speed up to 1 s once the IK and trim are dialled.
* **Power supply sag:** if the robot collapses momentarily during
  swing-to-stance transitions, the BECs are sagging. Switch to a
  beefier 5 A switching BEC (D-Link D24V50F5 or similar) or split
  the 18 servos across 3 BECs instead of 2.

---

## 10. Migration to the full-size walker

The prototype's value is that **everything you build above the joint
level transfers unchanged** to the full-size walker:

* The IK math is identical (same coxa / femur / tibia ratio).
* The gait scheduler is identical.
* The per-leg state machine (stance / swing / lift / re-plant) is
  identical.
* The body kinematics + IMU fusion are identical.

What changes when you scale up:

* The actuators talk EtherCAT or CAN instead of PWM, so the joint
  driver layer changes.
* Joint torque limits are 100× higher, so the gait scheduler needs to
  add a torque-aware planner (don't command a stride that requires
  more than 70 % of peak knee torque).
* Safety: a 280 kg vehicle needs an E-stop, brake-on-fault behaviour,
  and a "freewheel only when stationary" supervisor — none of which
  matter on a 1.3 kg tabletop unit.

Plan to spend ~ 3 months walking on the prototype, ironing the gait
and IK out, before you cut metal for the full-size build.

---

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot "twitches" but won't move | One BEC is browning out | Add a second BEC, verify 5 V steady on a scope under load |
| One leg drags | Per-joint trim is off, or that servo's output dead-band is unusually wide | Re-trim, or swap that servo (cheap servos have ± 10 ° unit-to-unit variation) |
| Knee servos overheat | Gait is over-loading (too high a chassis, too long a stride) | Lower chassis by re-targeting `STANCE_FEMUR_DEG = -15`, shorter stride |
| Robot drifts sideways | Tripod legs not lifting in sync | Increase swing duration; verify `trim_deg` for hip-pitch is consistent across all 6 legs |
| Foot slips on floor | TPU pad too smooth | Cut a small section of bicycle inner tube, glue inside the pad |

---

## 12. License & Disclaimer

Same as the parent project — personal exploration, no license declared
yet. The prototype is mechanically benign (small servos, small
batteries, no rider) but **3S LiPos can fight back if you puncture
them** — charge in a fire-safe bag and don't leave them charging
unattended.
