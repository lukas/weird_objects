# Hexapod Walker — Tabletop Prototype Build Guide

> A scaled-down sibling of the [full-size walker](../ASSEMBLY.md) intended
> for proving out the geometry, kinematics, and gait controller before
> you commit to industrial servomotors. Same architecture: regular hex
> chassis, six identical 3-DOF legs, alternating-tripod gait — but
> everything is shrunk roughly 6× and every joint is driven by a
> **FEETECH STS3215 (ST-3215-C018, 12 V / 30 kg·cm) serial-bus smart
> servo** instead of a $5000 harmonic-drive servomotor.
>
> Total parts cost: **~$615 – $1000** in 2026 USD (see
> [`PROTOTYPE_BOM.md`](PROTOTYPE_BOM.md)). A motivated builder can have
> it walking on a tabletop in a weekend.

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
| Battery | 1 × 3S 2200 mAh LiPo (11.1 V nom / 12.6 V full) |
| Servo rail | Raw 3S (12 V) via distributed fused power bus; see [`firmware/WIRING.md`](firmware/WIRING.md) §6 |
| Logic rail | 5 V from a XINGYHENG 12→5 V buck (Uno Q only) |
| Continuous draw (cruise) | ~ 9 A @ 12 V (all 18 servos walking), ~ 0.3 A @ 5 V (logic) |
| Run time, level ground | ~ 30 min |

**Knee torque margin:** the actuator is a FEETECH STS3215
(ST-3215-C018, 12 V / 30 kg·cm) serial-bus smart servo.  At 12 V it
gives a comfortable safety factor over the worst-case knee torque, and
its built-in 12-bit encoder closes the position loop and reports
load/voltage/current/temperature back over the bus.  The printed
cradles, disc-horn sandwich joints and RL servo torque limits are all
tuned around this STS3215-class case.

> **Design B (May 2026):** the printed `servo_horn_adapter` disc has
> been retired.  Each link now bolts directly onto a purchased servo
> horn.  Drops the printed-leg-bolt-up Z stack by ``HORN_ADAPTER_T``
> (4 mm) per joint.
>
> **June 2026 disc-horn switch:** the servo horn is now a **20 mm
> aluminum 25T disc** (Amazon B07D56FVK5, MG995 / MG996R type), NOT
> the plastic 4-arm X-horn.  Each link bolts onto the disc's flat top
> face via 4 x Phi 3.4 mm M3 clearance holes on a **14 mm bolt circle**
> (``XHORN_BOLT_PCD`` = 14.0, cross pattern at 0/90/180/270 deg) plus a
> Phi 9 x 2 mm central spline-collar clearance bore.  The 4 x M3 x 6
> SHCS thread DIRECTLY into the disc's M3 tapped holes -- the aluminium
> is the thread-engagement medium.  (The 14 mm value is interpreted as
> a bolt-circle DIAMETER; if calipers say it is a 14 mm square instead,
> it is a one-line change to ``XHORN_BOLT_PCD`` + the bolt angles.)
>
> **Design E (May 2026) mixed-mode cradle bolts:** each cradle has
> 4 vertical M3 x 8 SHCS driven DOWN through the servo's mounting
> tabs.  Sites split by X sign:
>   * The 2 -X bolts thread into M3 brass heat-set inserts (McMaster
>     `94459A130`) pressed into Phi 4 mm pockets inside Phi 8 mm
>     printed bosses -- real metal threads, full pull-out durability.
>   * The 2 +X bolts self-tap into bare Phi 2.5 mm pilots in the
>     existing well-wall material (no boss enlargement, no insert).
>     The Phi 8 mm heat-set boss footprint physically cannot coexist
>     with the +X wire-exit channel that the servo's molded wire
>     boot must pass through during insertion; see
>     `INSERT_M3_SELFTAP_*` in `hexapod_prototype.py` for the
>     design-decision rationale and `check_servo_insertion_path`
>     in `_verify_prototype.py` for the regression probe.
> Counts per robot: **72 x M3 x 8 SHCS** (36 -X into-insert + 36 +X
> self-tap, same physical stock for both engagement modes) plus
> **36 x M3 brass heat-set inserts** (-X column only).  Replaces
> the brief Design C captive-nyloc and Design D all-heat-set
> iterations.
>
> **Design F (May 2026) chassis_bottom-integrated yaw cradle:**
> the standalone `coxa_bracket` (orange flange + dangling servo
> well) has been folded into the `chassis_bottom` plate as a
> per-leg printed-in cradle that grows up 11 mm from the plate's
> top face.  The yaw servo drops INTO chassis_bottom rather than
> hanging from a screwed-on flange; the servo's output shaft pokes
> up between the two chassis plates via the Phi 12 mm pass-through
> cutouts in `chassis_top` (currently a no-op because the cradle
> sites sit just outside chassis_top's 70 mm apothem, but the
> cutout intent is documented in `make_chassis_top()`).  The wire-
> exit corridor and zip-tie post mirror to the cradle's -X
> (radially-INBOARD) face, putting each leg's harness mouth right
> next to the existing leg drop slot in chassis_bottom and
> shortening each yaw cable run by ~ 62 mm of Manhattan distance.
> Fastener counts per cradle are unchanged from Design E (4 x
> M3 x 8 SHCS, 2 of which thread into M3 heat-set inserts); the
> previous 24 x M3 x 14 coxa-bracket-to-chassis bolts (4 per leg)
> and the matching 4-hole-per-leg pattern in `chassis_top` /
> `chassis_bottom` are dropped.  The legacy `coxa_bracket` STL is
> retired in the final cleanup commit of this redesign.

---

## Visual inspection

Three viewers share the same `part_palette.PART_COLORS` table, so a
given part type (e.g. `coxa_link` = green, `foot_pad` = brown,
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
- **Left-click in empty space** (or press `I` / `Esc`) → restores every part's opacity to its full value.
- Left-drag still rotates the camera. Right-drag pans. Scroll zooms.

**Keyboard**

| Key | Action |
|---|---|
| `L` | Toggle the persistent all-labels-at-once cloud (off by default — hover is usually less noisy) |
| `E` | Toggle exploded view between 0.0 and 1.5 |
| `I` / `Esc` | Clear isolation |
| `R` | Reset the camera view |
| `S` | Save a screenshot to `artifacts/views/build_inspect.png` |
| `Q` | Quit |

The bottom slider (0.0 – 2.0) scales each part's distance from the
chassis centroid for finer exploded-view control than the `E` toggle,
and the row of color-coded checkboxes along the left edge hides or
shows all instances of a given part type so you can isolate, say,
just the coxa links or just the servo bodies. The MuJoCo
viewers' body-name labels default to ON (toggle with `B` once inside
the viewer).

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

The legacy verifier `_verify_prototype.py` is parallelised via a
process pool and exposes several CLI flags for faster inner-loop
iteration:

| Flag | What it does |
|---|---|
| `--fast` | Run only the 5 ESSENTIAL checks (watertightness, self-collision standing, fastener engagement, mating-face contact, cable clearance). ~5-15 s vs ~95-270 s for the full suite. Use between every edit. |
| `--all` | Explicit full-suite mode (same as no mode flag). |
| `--changed` | SMART SELECT: only run checks whose static input deps intersect what `git diff origin/main` changed. Rebuilds STLs in-memory to detect which printed parts actually changed bytes; consults the `CHECK_INPUTS` map. Compose with `--fast` to intersect with the essential set. Pass `--base-ref REF` to diff against a different ref. |
| `--no-cache` | Bypass the persistent per-check cache (`hexapod_walker/prototype/.verify_cache.sqlite`). Lookups are skipped; writes still happen. Use when you've edited a verifier helper that the cache key intentionally doesn't track. |
| `--serial` | Skip the process pool entirely; run every check in the main process in declaration order. Use this when a worker traceback is mangled through pickle and you need a clean stack. |
| `--workers N` | Override the default worker count (default `min(8, os.cpu_count())`). |
| `--profile PATH` | Dump a cProfile snapshot of the parent process to `PATH` when the run finishes (combine with `--serial` to profile the entire suite in one process). |
| `--only CHECK_NAME` | Run only the named check(s). Repeatable: `--only "Servo clearance" --only "Workspace self-collision"`. Names match the declaration list (see top of `_verify_prototype.py` `CHECKS`). |

Make targets wrap the common flag combinations:

| Target | Underlying command | Typical use |
|---|---|---|
| `make verify-prototype` | `_verify_prototype.py` (full suite, cache-aware) | Pre-commit / pre-print gate. |
| `make verify-fast` | `_verify_prototype.py --fast` | Inner-loop between every edit. |
| `make verify-changed` | `_verify_prototype.py --changed` | After a small CAD edit; selects only checks affected by the diff. |

All flags compose with the existing `--with-arm` flag. Parallel output
is printed in declaration order so a `diff` against the serial baseline
is byte-for-byte clean for every `[PASS]/[FAIL]` line.

The persistent per-check cache stores `(check_name, input_hash) ->
(result, runtime_s, message, timestamp)` rows in
`.verify_cache.sqlite` (gitignored). The hash mixes the check
function's own source, every STL file in `stl_prototype/`, and the
`hexapod_prototype` + `fastener_registry` module sources. A second
run with no inputs changed hits the cache for every check and lands
in ~1-2 s end-to-end. Rows older than 30 days are pruned on every
invocation. Bust with `--no-cache` or by deleting the sqlite file.

`--changed` walks `git diff origin/main` and rebuilds every STL
in-memory to detect which printed parts ACTUALLY changed bytes (so a
cosmetic comment-only edit to `hexapod_prototype.py` selects zero
checks). It consults two maps near the top of `_verify_prototype.py`:

* `CHECK_INPUTS` -- per-check set of printed-part STL names the check
  consults. Future checks MUST add an entry; missing entries default
  to `ALL_PRINTED_PARTS` (safe but defeats selection).
* `CHECK_SOURCE_DEPS` -- per-check set of registry-source tags
  (`fastener_registry`, `cable_keepouts`, ...) that should also
  trigger the check.

If `_verify_prototype.py` itself is in the diff, `--changed` falls
back to "select all"; the cache then dedups unchanged check sources
at exec time.

### Opt-in strength / failure-point check

A SEPARATE pipeline lives under `hexapod_walker/prototype/strength/`
that runs a closed-form Euler-Bernoulli beam-bending sanity check on
the slender links plus -- when the toolchain is available -- a
CalculiX linear-static FEA pass on every load-bearing printed part.
Output is `artifacts/strength_report.md` (with per-part stress-field
PNGs under `artifacts/strength/`).

This pipeline is **deliberately not** wired into `make check-cad`.
It's observation-only and you opt in explicitly:

```bash
make -C hexapod_walker/prototype check-strength            # PETG (default)
make -C hexapod_walker/prototype check-strength MATERIAL=pla
make -C hexapod_walker/prototype check-strength PARTS=tibia_link,femur_link SOLVER=beam
make -C hexapod_walker/prototype check-strength SOLVER=beam   # closed-form only
```

**Installing the toolchain (macOS):**

```bash
# Tetrahedral mesher (works on Apple Silicon).
brew install gmsh

# CalculiX solver (Intel macOS / Linux):
brew install costerwi/homebrew-calculix/calculix-ccx

# Python deps land in the repo's shared venv via run.sh; if you
# bypass run.sh, install manually:
uv pip install pygmsh meshio
```

As of May 2026 the `calculix-ccx` Homebrew formula does **not** build
on Apple Silicon (the bundled ARPACK fails to link `_dseupd_`);
`gmsh` works fine.  If `ccx` is missing the pipeline falls back to
the beam-bending check alone and notes the missing solver in the
report.  On Linux (Ubuntu) `apt install calculix-ccx gmsh` covers
both.

**What the load cases look like** (derived from the design constants
in `hexapod_prototype.py` and an inferred 2.15 kg assembled mass):

| Part | Load case | Source |
|---|---|---|
| `tibia_link`    | foot tip -Z impact = `robot_weight / 3 * 2g` | `FEMUR_LENGTH`, `TIBIA_LENGTH`, `XHORN_BOLT_PCD` |
| `femur_link`    | knee-end -Z impact = same `2g` foot load           | same + `FEMUR_LENGTH` |
| `coxa_link`     | DS3225 stall = 2.5 N-m about the yaw axis          | `COXA_LENGTH` + `DS3225_STALL_TORQUE` |
| `chassis_bottom`| dead load (battery + tray + 6 cradle/leg masses)   | `BATTERY_HOLDER_CENTRE_X`, yaw cradle XY |
| `chassis_top`   | dead load minus battery                            | same supports |
| `foot_pad`      | foot-strike pressure on the disk floor             | `FOOT_PAD_OD`, `FOOT_HINGE_PIN_HOLE_D` |

> **Legacy symbol names.** The opt-in `strength/` module has **not**
> been renamed to the STS3215 sandwich-joint part set: it still keys its
> load cases on the old single-spar names `tibia_link` / `femur_link` /
> `coxa_link` and on the constant `DS3225_STALL_TORQUE` (~2.5 N·m yaw
> stall, comparable to the STS3215's ~30 kg·cm ≈ 2.9 N·m). These are
> internal strength-tooling symbols only — the actual robot is driven by
> 18× FEETECH STS3215 serial-bus servos (see §2/§4). The CLI examples
> above use those legacy `PARTS=` names because that is what the module
> currently expects.

The strength module imports from `hexapod_prototype` but the reverse
is forbidden -- a missing FEA toolchain cannot break the geometry
build.

### `check_screwdriver_access` envelopes

The screwdriver-access check probes a per-fastener cylindrical
clearance cone above each head along the driver-approach axis and
fails if any printed part intrudes by more than 30 mm³. Three
envelopes are dispatched off `FastenerInstance.spec`:

| Envelope | Dia × length | Used for |
|---|---|---|
| `HEX_KEY`  | 8 mm × 30 mm  | SHCS (M3x8 / M3x14 / M3x32) + the M2.5 spline center screw — anything driven with an L-shaped hex key short arm |
| `HEX_KEY` (counter-bored) | 5.7 mm × 15 mm | `M3x6 SHCS` disc-horn bolts — recessed in a Phi 5.7 mm counter-bore on the tight 14 mm bolt circle, so the driver turns inside the bore (narrower envelope, mirrors the old M2 link-bolt special case) |
| `PHILLIPS` | 12 mm × 80 mm | `pan-head` / `Phillips` / `slotted` — currently just the M3x16 foot hinge bolt |
| `SOCKET`   | 12 mm × 50 mm | `nyloc nut` / generic `nut` — M3 nyloc driven with a 5.5 mm nut socket |

Spline center screws are technically a small Phillips on hobby
servos, but every such fastener is also explicitly SKIPped (captive
under the disc horn after assembly), so they're mapped to `HEX_KEY`
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
| `chassis_top.stl` | Top hex deck (4 mm PLA, 200 mm flat-to-flat).  Carries 2 x Phi 8 mm printed bosses near the +X edge for the `switch_holster` heat-set inserts, plus the 4 `DECK_COLUMN_XY` standoff sites that carry the stacked electronics deck. | 0.2 mm layer, 25% gyroid infill, 4 walls |
| `chassis_bottom.stl` | Single merged bottom plate (Jun 2026): a flat 200 mm hex with a ~8 mm floor that carries the 6 yaw-servo cradles, the split yaw-bearing tower bases, and the velcro-strap slots that retain the LiPo on the plate top face.  Prints face-DOWN, no supports. | same as top |
| `uno_q_tray.stl` | Lower electronics deck — carries the **Arduino Uno Q** (the brain + direct STS3215 TTL-bus driver).  Stacks on 4 M3 standoff columns above `chassis_top` at `DECK_LEVEL_1_STANDOFF_H` (16 mm). | 0.2 mm, 20% gyroid, 3 walls |
| `buck_tray.stl` | Upper electronics deck — carries the **XINGYHENG 12→5 V buck converter** that powers the Uno Q's logic.  Stacks on the `uno_q_tray` columns at `DECK_LEVEL_2_STANDOFF_H` (22 mm). | 0.2 mm, 20% gyroid, 2 walls |
| `spider_carapace.stl` | Domed cephalothorax shell (~141 x 124 x 34 mm) with the classic 8-eye spider face.  Bolts on as the 3rd deck level on 4 M3 standoffs above `buck_tray`; the open skirt + rear window keep every wire exit and the buck vented. | 0.2 mm, 10–15% gyroid, 3 walls, print rim-down |
| `switch_holster.stl` | Snap-in holster for one ~ 32 x 17 x 17 mm anti-spark on/off switch (the e-stop).  Bolted to chassis_top's +X edge via 2 x M3 x 10 SHCS that thread DOWN into M3 brass heat-set inserts captive in chassis_top's 2 printed bosses.  Toggle protrudes +X past the chassis edge for user access; XT60 pigtails exit the +/- Y end faces. | 0.2 mm, 25% infill, 3 walls |
| `imu_pad.stl` | 25 x 20 x 2 mm pad with 4 x Phi 8 mm bosses (3 mm tall) carrying M3 brass heat-set inserts on the GY-521 15 x 11 mm pattern (4 x Phi 3.0 mm clearance holes).  Bolts the MPU-6050 IMU breakout via 4 x M3 x 8 SHCS.  **No fasteners between the pad and chassis_top** -- the pad's flat smooth underside is bonded to chassis_top's centre with 3 mm double-sided foam tape, which serves AS the mount AND the vibration damper.  Sits at chassis (0, 0, chassis_top_top_z + 3 mm) -- the chassis centre of gravity, so gyro rates are not contaminated by linear-acceleration cross-coupling from body swing. | 0.2 mm, 25% infill, 3 walls |

### 3.2 Per-leg parts (print 6 sets)

The leg is a **disc-horn sandwich joint** design (Jun 2026): each hip
and knee joint drives a 20 mm aluminium 25T disc horn on the servo's
output spline and reuses a SECOND disc horn on the servo's rear idler
boss for passive support, with the yoke bolting identically to both.
The femur and tibia segments are Ø8 mm carbon-fibre tubes epoxy-bonded
into printed end-fittings, NOT single printed spars.

| File | Function | Print orientation |
|---|---|---|
| `coxa_link.stl` | Yaw pad + arm + hip fixed side (symmetric servo cradle; the passive disc horn rides the servo's own rear idler boss, so there is no separate bearing housing). | Yaw-pad face down, bracket opening up |
| `yaw_bearing_cap.stl` | TOP half of the SPLIT yaw-bearing tower; bolts onto each `chassis_bottom` tower with 3 x M3 x 8 SHCS to capture the spaced 6706 bearing pair.  Print flat, ring face down (no supports). | Ring face down |
| `femur_hip_yoke.stl` | Hip moving yoke (symmetric clevis — both arms bolt to a disc horn, driven on the front, passive on the rear boss) + Ø8 CF-tube socket. | Spine down |
| `femur_knee_bracket.stl` | Knee fixed side (symmetric servo cradle) + Ø8 CF-tube socket. | CF socket up |
| `tibia_knee_yoke.stl` | Knee moving yoke (symmetric clevis — driven + passive disc horns) + Ø8 CF-tube socket. | Spine down |
| `tibia_foot_fitting.stl` | Ø8 CF-tube socket + single foot-hinge TANG (M3 pin). | CF socket up |
| `foot_pad.stl` | Compliant foot with a 2-cheek FORK at +Z (May 2026 inversion: foot now carries the fork, tibia carries the tang). M3 x 16 pan-head pin + M3 nylock captures the joint. Print in TPU 95A for grip and ankle compliance. | Disk on bed, fork up. Fork cheeks are 3.5 mm thick TPU walls and the 6.4 mm slot between them prints in air -- no supports needed (vertical features only). |

> **Femur** = `femur_hip_yoke` + Ø8 CF tube + `femur_knee_bracket`.
> **Tibia** = `tibia_knee_yoke` + Ø8 CF tube + `tibia_foot_fitting` +
> `foot_pad`.  Cut each tube to length, epoxy into the printed sockets,
> and drive a transverse Ø2.5 mm roll pin through the cross-hole.

### 3.2a Per-joint parts (print 12 of each — 1 per hip + knee joint)

| File | Function | Print orientation |
|---|---|---|
| `servo_clamp_cap.stl` | Sandwich-joint clamp cap that traps the hip/knee servo body; 2 x M3 x 8 SHCS self-tap into the cradle ±X wall-end pilots. | Flat |
| `passive_horn_adapter.stl` | Press-fits the servo's rear idler boss and centres the reused 25T disc horn on the passive side; a central M2.5 screw retains it. | Collar down |

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
| Serial-bus servo | **FEETECH STS3215 (ST-3215-C018), 12 V / 30 kg·cm metal-gear smart serial-bus servo**, standard ~40 × 20 × 40 mm case with a 25T output spline carrying the flush 20 mm disc horn; a real END-face 10×10 mm M2.5 hole square for body retention.  Built-in 12-bit encoder; daisy-chained on a 1 Mbps half-duplex TTL bus with closed-loop position/load/voltage/current/temp feedback.  Buy all 20 from the same batch. | 18 + 2 spare | ~$18 – $25 each ($360 – $500 total) |

### 4.2 Power

> Power architecture (3S → distributed fused bus → per-leg branches +
> a separate 5 V logic buck) is documented in full in
> [`firmware/WIRING.md`](firmware/WIRING.md) §6.  There is **no servo
> BEC** — the STS3215 run directly off the raw 3S (12 V) rail.

| Item | Spec | Qty | Cost |
|---|---|---|---|
| LiPo battery | 3S 2200 mAh 25C+, XT60 connector.  Velcro-strapped to the top of `chassis_bottom` (no clip-in holder). | 1 | $20 |
| Velcro cinch straps | Hook-and-loop straps (~15–20 mm) through the chassis_bottom strap slots; retain the LiPo on the plate. | 1 set | $5 |
| XINGYHENG 12→5 V buck | 12 V→5 V step-down (3 A+); drops the 3S rail to 5 V for the Uno Q logic only (servos run on raw 12 V).  Mounts on `buck_tray`. | 1 | $8 |
| Power distribution bus bar | 12 V DC bus bar / fused distribution block (V+ + GND); the 6 per-leg power branches + the buck tap off it. | 1 | $10 |
| Main fuse + holder | 15–20 A blade/ANL fuse + inline holder between the anti-spark switch and the bus bar. | 1 | $8 |
| Per-branch fuse + holder (optional) | 5–7 A mini-blade fuse + holder, one per leg branch. | 6 | $1.50 each |
| 16–18 AWG silicone wire | Red + black, ~5 m each; the six per-leg V+/GND branches from the bus bar to each leg's 5264 injection pigtail. | 1 | $10 |
| XT60 pigtails | Male/female, 12–14 AWG silicone (battery cable + bus-bar feed). | 2 | $4 |
| Molex 5264 connector + crimp kit | 3-pin 2.5 mm kit (~20 sets) for the per-leg injection pigtails and the leg-to-leg signal+GND-only data jumpers. | 1 | $10 |
| Anti-spark on/off switch | XT60 in/out pigtails (the e-stop).  Snaps into `switch_holster.stl` on chassis_top's +X edge; toggle protrudes outside the chassis for user access. | 1 | $10 |
| LiPo charger | iSDT D2, B6AC, or any decent 3S balance charger | 1 | $30 |
| LiPo bag | Fire-safe charging | 1 | $10 |

### 4.3 Control electronics

> The whole control stack is a single **Arduino Uno Q** (on-board Linux
> SoC + MCU) that runs the Python gait/RL/teleop AND drives the
> half-duplex STS3215 TTL bus directly at 1 Mbps.  It replaces the old
> Arduino Mega 2560, the Raspberry Pi, the 2×PCA9685 PWM boards, AND any
> separate USB-to-TTL bus adapter.

| Item | Spec | Qty | Cost |
|---|---|---|---|
| Arduino Uno Q | On-board Linux SoC + MCU; runs Python gait/RL/teleop and drives the STS3215 bus directly.  Mounts on `uno_q_tray` via 4 x M3 brass heat-set inserts (McMaster 94459A130) + 4 x M3x8 SHCS. | 1 | $50 |
| microSD card | 32–64 GB, A1/A2 rated (OS/storage if not using on-board eMMC). | 1 | $10 |
| USB-C cable | Flashing / powering / console access to the Uno Q. | 1 | $8 |
| MPU-6050 IMU (GY-521) | 6-DOF gyro + accel, I²C, powered from 3V3.  Bolts to `imu_pad.stl` via 4 x M3x8 SHCS into M3 heat-set inserts. | 1 | $4 |
| FEETECH 3-pin serial-bus cables | DATA daisy-chain (servos ship with one each; buy a spare pack for the chassis-to-first-servo runs). | 1 pack | $8 |
| Dupont jumper wires + heat-shrink | I²C / logic wiring cleanup | — | $10 |

### 4.4 Fasteners

The authoritative, auto-generated fastener counts live in the
**Fasteners** table of [`PROTOTYPE_BOM.md`](PROTOTYPE_BOM.md) (300
total fasteners; edit `fastener_registry.py`, not the table).  The
key items:

| Item | Qty | Notes |
|---|---|---|
| M3 × 10 SHCS disc-horn bolts (`91290A114`) | 48 | DRIVEN hip + knee front-horn bolts on a 14 mm bolt circle into the disc's M3 tapped holes (2 mm longer because the flush output seats the driven disc 2 mm lower). |
| M3 × 8 SHCS disc-horn bolts (`91290A113`) | 72 | Yaw front horn + the two passive rear-boss horns, same 14 mm bolt circle into the disc's M3 tapped holes. |
| M2.5 × 8 SHCS, servo body retention (`91290A104`) | 48 | HIP + KNEE cradles only — 4 per cradle into the servo's real END-face 10×10 mm M2.5 hole square; the YAW cradle takes none (held by the `yaw_servo_retainer` strap instead). |
| M2.5 × 8 spline / passive-horn screws (`91290A104`) | 30 | Passive-horn retention + servo spline center screws (ship with the servos). |
| M3 × 8 SHCS (`91290A113`) | 54 | 8 deck board mounts (Uno Q + buck) + 4 imu_pad into heat-set inserts; 24 `servo_clamp_cap` + 18 `yaw_bearing_cap` self-tap. |
| M3 × 10 SHCS (`91290A114`) | 14 | chassis_top → brass-standoff bolts + deck standoff-column bolts + 2 switch_holster mount bolts. |
| M3 × 16 mm pan-head (`92010A130`) | 6 | Foot hinge pins (one per leg). |
| M3 nyloc nuts (`90576A102`) | 14 | 6 foot hinge pins + 4 under chassis_bottom (chassis standoffs) + 4 under chassis_top (deck columns). |
| M3 brass heat-set inserts (McMaster `94459A130`) | 18 | 8 deck trays + 4 imu_pad + 2 chassis_top (switch_holster) + 4 `spider_carapace` feet.  Soldering-iron installed at ~220 °C. |
| M3 × 32 mm M-F brass standoffs | 4 | chassis_top ↔ chassis_bottom across `CHASSIS_GAP` = 32 mm on the rotated-45° 35-mm-radius pattern. |
| M3 deck standoff columns, M-F brass | 12 | Stacked deck: 4 × 16 mm (chassis_top → uno_q_tray) + 4 × 22 mm (uno_q_tray → buck_tray) + 4 × ~24 mm (buck_tray → spider_carapace). |
| 6706-2RS ball bearings | 12 | Yaw joint spaced pair (2 per leg), Ø30 × Ø37 × 4 mm; captured by the split tower + `yaw_bearing_cap`. |
| Ø8 mm carbon-fibre tube | ~1 m | Femur + tibia leg segments (epoxy-bonded into the printed sockets). |
| Ø2.5 mm roll pins | 12 | Transverse CF-tube retention (2 per leg). |
| Two-part epoxy | 1 | Bonds the CF tubes into the printed sockets. |
| 3M VHB / 3 mm foam tape | ~25 × 20 mm | Bonds `imu_pad.stl` to chassis_top centre; doubles as the IMU vibration damper.  See §B.6 of `SHOPPING_LIST.md`. |

### 4.5 3D-printed material

| Item | Spec | Qty | Cost |
|---|---|---|---|
| PLA / PETG filament | 1.75 mm, any colour | ~ 1 kg | $20 |
| TPU filament | 1.75 mm, 95A | ~ 250 g (foot pads only) | $5 |
| MJF PA12 (optional) | 12 × `servo_clamp_cap` (PLA also OK) | — | print-service |

### 4.6 Total

Cost buckets mirror the **Rough Cost** table in
[`PROTOTYPE_BOM.md`](PROTOTYPE_BOM.md):

| Bucket | STS3215 build |
|---|---:|
| STS3215 serial-bus servos (20) | ~$360 – $500 |
| Arduino Uno Q + microSD + USB-C | ~$60 – $100 |
| XINGYHENG buck converter | ~$8 – $15 |
| Battery + charger + bag + switch + velcro | ~$70 – $120 |
| Power distribution (bus bar + fuses + silicone wire + 5264 kit) | ~$25 – $45 |
| Disc horns (30) + CF tube + roll pins + epoxy | ~$30 – $55 |
| Fasteners / standoffs / deck columns / wiring | ~$30 – $55 |
| Filament + MJF clamp caps | ~$30 – $55 |
| **Total** | **~ $615 – $1000** |

---

## 5. Print plan

A single Ender 3-class printer runs the whole BOM in roughly **22 hours**
(see the print queue in [`SHOPPING_LIST.md`](SHOPPING_LIST.md) §D):

| Pass | Parts | Time |
|---|---|---|
| 1 | 6 × `coxa_link` + 6 × `yaw_bearing_cap` | ~ 4 h |
| 2 | 6 × `femur_hip_yoke` + 6 × `femur_knee_bracket` | ~ 8 h |
| 3 | 6 × `tibia_knee_yoke` + 6 × `tibia_foot_fitting` | ~ 6 h |
| 4 | 12 × `passive_horn_adapter` + 12 × `servo_clamp_cap` (MJF PA12 or PLA) | ~ 5 h |
| 5 | `chassis_top` + `chassis_bottom` + `uno_q_tray` + `buck_tray` + `switch_holster` + `imu_pad` + `spider_carapace` | ~ 12 h |
| 6 | 6 × `foot_pad` (TPU) | ~ 1 h |

Tip: use 4 walls and 30–40 % gyroid infill for the yokes, brackets and
the split yaw-bearing tower cap — the servo cradles and bearing tower
see the most load and the extra walls add substantial stiffness for
very little weight.

---

## 6. Assembly sequence

Allow ~ 4 hours for a first build, ~ 90 min for a second.

> **Chassis_bottom is ONE printed part (Jun 2026 merge).** It is a
> 200 mm flat hex plate with a ~8 mm-thick flat floor that carries the 6
> yaw-servo cradles, the base half of each split yaw-bearing tower, and
> the velcro-strap slots that retain the LiPo on the plate's top face.
> It prints face-DOWN flat with no supports.  **Yaw servo retention:**
> each yaw servo seats into its open-bottom cradle and is captured by the
> bolt-on `yaw_servo_retainer` strap + anchor bolts + the output-face
> seat on the mount plate (the yaw cradle takes **no** M2.5 end-face
> bolts — the flush-horn refit dropped the yaw output ~5.5 mm so the
> servo hangs below the chassis floor; see `PROTOTYPE_BOM.md`).

> **The joint is a disc-horn sandwich (Jun 2026).** Each hip and knee
> joint drives a 20 mm aluminium 25T disc horn on the servo's output
> spline and reuses a SECOND disc horn on the servo's rear idler boss
> for passive support, so the moving yoke bolts identically to a disc on
> BOTH faces — no external ball bearing on the hip/knee. The femur and
> tibia are Ø8 mm carbon-fibre tubes epoxy-bonded into the printed
> end-fittings (with a transverse Ø2.5 mm roll pin), NOT single printed
> spars. Full joint detail is in the **Bench Test Order** of
> [`PROTOTYPE_BOM.md`](PROTOTYPE_BOM.md).

### 6.1 Per-leg sub-assembly (do all 6 in parallel)

> **Servo body retention (hip + knee cradles).** Each hip and knee
> cradle bolts the servo's real **END-face 10 × 10 mm M2.5 hole square**
> with 4 × M2.5 × 8 SHCS driven through the cradle's −X wall (head
> recessed in a wall counterbore, threading into the servo's own metal
> case), plus the printed `servo_clamp_cap` seated flush against the
> body's +Y face and retained by 2 × M3 × 8 SHCS self-tapping into the
> cradle's ±X wall-end pilots.  The YAW servo uses no cradle bolts (see
> the chassis_bottom note above).  There are **no heat-set inserts and
> no vertical tab bolts** anywhere on the leg — the servo threads bolt
> straight into the servo's own steel case.

1. **Bond the CF leg segments first.** Cut the Ø8 mm carbon-fibre tube
   to the femur and tibia lengths, epoxy each into its printed socket
   (`femur_hip_yoke` ↔ `femur_knee_bracket` for the femur,
   `tibia_knee_yoke` ↔ `tibia_foot_fitting` for the tibia), and drive a
   transverse Ø2.5 mm roll pin through each socket cross-hole.  Let the
   slow-cure epoxy fully set before loading the joints.
2. **Hip-pitch servo into the coxa link's hip cradle:** seat the servo
   into the `coxa_link` hip cradle and drive 4 × M2.5 × 8 SHCS through
   the cradle's −X wall into the servo's END-face hole square, then
   snap on the `servo_clamp_cap` and secure it with 2 × M3 × 8 SHCS
   self-tapping into the cradle wall-end pilots.
3. **Disc horns on the hip servo:** push a 20 mm aluminium 25T disc
   horn onto the output spline at 0° and retain it with the servo's
   M2.5 spline screw; centre the SECOND (passive) disc horn on the
   servo's rear idler boss with a `passive_horn_adapter` and retain it
   with one M2.5 screw.
4. **Femur onto the hip joint:** seat the `femur_hip_yoke`'s two clevis
   arms onto the driven (front) and passive (rear) disc horns and bolt
   each arm to its disc on the Ø14 cross — the **driven** front horn
   takes 4 × M3 × 10 SHCS (the flush output seats it 2 mm lower); the
   **passive** rear horn takes 4 × M3 × 8 SHCS — into the discs' M3
   tapped holes.  Use a 2.5 mm hex key.
5. **Knee servo into the femur knee bracket:** repeat step 2 on the
   `femur_knee_bracket`'s knee cradle (4 × M2.5 × 8 into the END-face
   square + the `servo_clamp_cap`), then fit the driven + passive disc
   horns as in step 3.
6. **Tibia onto the knee joint:** seat the `tibia_knee_yoke`'s clevis
   arms onto the driven + passive knee disc horns and bolt each arm to
   its disc (driven = 4 × M3 × 10, passive = 4 × M3 × 8) as in step 4.
7. **Foot pad:** slide the `foot_pad`'s FORK over the
   `tibia_foot_fitting`'s tang from below (the foot carries the fork,
   the tibia carries the single tang), align the M3 holes, and drive
   one M3 × 16 pan-head bolt (`92010A130`) THROUGH the fork + tang into
   an M3 nylock nut on the far cheek.  Tighten just until the joint is
   snug but still rotates freely; the hinge axis is parallel to the
   knee pitch axis so the foot pitches passively as the leg walks.

You now have a complete leg (yaw pad → hip → femur → knee → tibia →
foot) ready to drop onto its `chassis_bottom` yaw cradle. Repeat 6
times.

### 6.2 Final assembly

1. **Yaw servos + split bearing towers into chassis_bottom:** lay
   chassis_bottom flat.  Seat each yaw servo into its open-bottom
   cradle and secure it with the `yaw_servo_retainer` strap + anchor
   bolts (no M2.5 end-face bolts on the yaw cradle).  Fit the yaw
   driven disc horn on the output spline.  Assemble the **spaced
   6706 bearing pair**: drop the LOWER 6706 race into chassis_bottom's
   open-top Ø37 pocket, set the `coxa_yaw_hub` boss, drop the UPPER
   6706 into the `yaw_bearing_cap`, then pull the cap down with **3 ×
   M3 × 8 SHCS** self-tapping into the tower pilots to capture both
   races at the correct spacing.
2. **Coxa links onto the yaw joints:** bolt each leg's `coxa_link` yaw
   pad to its yaw driven disc horn with 4 × M3 × 8 SHCS on the Ø14
   cross into the disc's M3 tapped holes.  Each completed leg now hangs
   from its yaw axis.
3. **Battery:** velcro-strap the 3S LiPo to the top face of
   chassis_bottom, looping the cinch straps through the chassis_bottom
   strap slots (there is no clip-in `battery_holder` any more).
4. **Chassis standoffs + top plate:** screw 4 × M3 × 32 mm M-F brass
   standoffs into the rotated-45° 35-mm-radius pattern
   (`CHASSIS_STANDOFF_HOLES_XY` = (±35, 0) and (0, ±35)).  Each
   standoff's MALE thread drops DOWN through chassis_bottom's clearance
   hole; capture it from below with 4 × M3 nyloc nuts.  Drop
   `chassis_top.stl` onto the standoff tops and drive **4 × M3 × 10 mm
   SHCS** DOWN from above into the standoffs' female top threads.
   Install this **before** the switch_holster — the +X standoff sits
   under the holster's footprint.
5. **Switch holster + IMU on chassis_top:** soldering-iron-install the
   2 chassis_top boss inserts and the 4 imu_pad inserts (McMaster
   `94459A130`, ~220 °C).  Bolt the `switch_holster.stl` to chassis_top's
   +X edge with **2 × M3 × 10 mm SHCS**.  Bolt the MPU-6050 (GY-521) to
   `imu_pad.stl` with **4 × M3 × 8 mm SHCS**, peel a ~25 × 20 mm square
   of 3 mm foam tape onto the pad's flat underside, and press the pad
   down onto chassis_top centred at (0, 0) — placing the IMU at the
   chassis centre of gravity and using the foam as both adhesive AND
   vibration damper.  Route the 4 IMU wires (VCC→3V3, GND, SDA, SCL) to
   the Uno Q's I²C pins (see `firmware/WIRING.md` Stage F).
6. **Electronics deck (Arduino Uno Q + buck):** the stack rises on M3
   M-F brass standoff columns at the `DECK_COLUMN_XY` (±41, ±33)
   pattern above chassis_top.  Soldering-iron-install the 8 deck-tray
   board-mount inserts (4 `uno_q_tray` + 4 `buck_tray`).  Thread the
   level-1 columns (16 mm) through chassis_top — retained by M3 nyloc
   nuts underneath — bolt the **Arduino Uno Q** onto `uno_q_tray` with
   4 × M3 × 8 SHCS, and drop the populated tray onto the columns with
   4 × M3 × 10 SHCS.  Repeat for the level-2 columns (22 mm) →
   `buck_tray` carrying the **XINGYHENG 12→5 V buck converter**.
7. **Wire it up:** see §7 and `firmware/WIRING.md`.  Bench-test all 18
   joints on a current-limited supply BEFORE the LiPo (WIRING.md
   Stages A–F).
8. **Spider carapace:** as the 3rd deck level, run 4 more standoff
   columns (~24 mm) up from `buck_tray` to the `spider_carapace` feet
   (which carry M3 heat-set inserts opening down) and drive an M3 screw
   UP from each standoff into the dome foot so the shell lifts off for
   service.  The open skirt + rear window keep every wire exit and the
   buck vented.

---

## 7. Wiring

> **Wiring of record: [`firmware/WIRING.md`](firmware/WIRING.md).**
> That doc is the buildable, step-by-step plan (two power domains, the
> half-duplex serial bus, the distributed fused power harness, and the
> bench bring-up checklist). This section is only a summary; when the
> two disagree, `firmware/WIRING.md` wins. See also `PROTOTYPE_BOM.md`
> and `SHOPPING_LIST.md` for the parts.

The robot is **18× FEETECH STS3215** (ST-3215-C018, 12 V / 30 kg·cm)
**serial-bus** smart servos driven **directly by an Arduino Uno Q**
(on-board Linux SoC + MCU). There is **no Raspberry Pi, no PCA9685 PWM
boards, no DS3225 PWM servos, and no FE-URT-1 / Waveshare bus adapter**
— the Uno Q's UART drives the half-duplex TTL bus itself at 1 Mbps.
Each servo has a built-in 12-bit encoder and reports
position/load/voltage/current/temperature back over the bus, so there
are no external encoders.

### 7.1 One-servo bench test first

Before assembling the robot, bring up **one STS3215 on a
current-limited bench supply** (not the LiPo) so a wiring mistake trips
the supply instead of cooking hardware. This proves the exact servo you
bought talks on the bus before you bolt all 18 joints. Full procedure:
`firmware/WIRING.md` §0–§4 (Stages A–C).

In brief, with the bus signal wired to the Uno Q UART (per the FEETECH
half-duplex convention) and 12 V on the bus V+ rail, common ground:

```bash
python -m pip install feetech-servo-sdk pyserial
python pi_control/feetech_bus.py --port /dev/ttyACM0 scan          # expect [1] (factory ID)
python pi_control/feetech_bus.py --port /dev/ttyACM0 setid --from 1 --to 1
python pi_control/feetech_bus.py --port /dev/ttyACM0 joint 0 15 --sweep
python pi_control/feetech_bus.py --port /dev/ttyACM0 feedback      # reads back angle, volts, temp
```

On the Uno Q's Linux side the port is usually `/dev/ttyACM0` or
`/dev/ttyUSB0` (on a laptop it may be `/dev/cu.usbmodem…`). The
yaw / hip / knee joint order for one leg is validated by re-ID'ing the
three leg-0 servos to IDs 1/2/3 and chaining them (`WIRING.md` Stage D)
— there are no PCA channels to plug into, just unique bus IDs.

### 7.2 Full robot wiring

There are exactly **two domains** that share only ground (full detail
+ ASCII diagrams in `firmware/WIRING.md` §1, §2 and §6):

```
  DATA (one half-duplex TTL bus, daisy-chained, low current):
  Uno Q UART ─sig+GND─► ID1─►ID2─►ID3 ─┊─► ID4─►ID5─►ID6 ─┊─► … ─► ID18
                        └── leg 0 ──┘       └── leg 1 ──┘
                        leg-to-leg jumper = SIGNAL + GND only (no V+)
  Uno Q I²C (separate bus) ─► MPU-6050 IMU

  POWER (12 V, DISTRIBUTED — never daisy-chained leg-to-leg):
  3S LiPo ─XT60─► anti-spark switch ─► MAIN FUSE 15–20 A ─► BUS BAR
       BUS BAR ├─► leg 0 branch (16–18 AWG, opt 5–7 A fuse) ─► leg 0 V+/GND
               ├─► … one branch per leg …                  ─► leg 5 V+/GND
               └─► XINGYHENG 12→5 V buck ─► Uno Q 5 V logic
```

Why the power bus is **distributed** and not chained: the FEETECH
3-pin bus connector (Molex 5264) is rated ~3 A/pin, but all-18 walking
current is ~9 A and would melt the first upstream connector. So **DATA
stays chained, POWER is injected per-leg from the bus bar** — no single
5264 pin ever carries more than one leg (~1.5 A walking). The
leg-to-leg data jumper carries **signal + GND only (V+ omitted)**. The
Uno Q's 5 V comes from a 12→5 V buck off the bus bar — **never** from
the raw 12 V servo rail. The anti-spark switch is the e-stop. See
`firmware/WIRING.md` §6 for the per-branch current budget, wire gauges
and fuse sizing.

### 7.3 Per-leg harness recipe

With the serial bus, each leg is just **three STS3215s daisy-chained
with stock 3-pin bus cables**; the leg's bundle drops through that
leg's `chassis_bottom` drop slot into the inter-plate volume, where it
meets its **power branch from the bus bar** (V+/GND) and the **data
jumper** to the next leg. There are **no per-servo runs back to a
driver board** — the old PWM extension-cable star is retired. The
printed strain-relief features (zip-tie posts at each cradle wire-exit
+ the per-leg drop slot) still apply: anchor each servo's cable at its
cradle post and the full 3-cable bundle at the drop slot so a leg snag
mid-walk can't yank a connector.

The cable lengths are modelled by `pi_control/wire_harness_plan.py`
(per-joint serial-bus reach budget: cradle wire-exit → leg drop slot →
electronics-tray bus landing, plus a +30 mm slack loop per joint axis
the harness crosses). Print the BOM with:

```bash
python -m hexapod_walker.prototype.pi_control.wire_harness_plan
```

It emits one row per servo (joint, leg + axis, servo ID, cradle exit,
drop slot, bus landing, minimum path length, and how many 30 cm
extensions that joint needs on top of the stock STS3215 bus lead).
Every joint currently fits inside the stock bus lead. Re-run after any
electronics-tray or chassis geometry edit; the verifier check **[18]
Harness reach** (`_verify_prototype.check_harness_reach`) asserts the
printed reach budget still covers the geometry.

**Slack-loop sizing** (aim for at least this much "extra" bundle at
each joint axis the harness crosses; err larger — extra slack is
harmless, a deficit fights the kinematics):

* Yaw axis:  ~20 mm slack loop (the bundle twists ~90° across the yaw
  axis at full sweep).
* Hip axis:  ~30 mm slack loop (the hip-pitch range plus the
  `coxa_link`'s WELL_Z_DROP_EXTRA = 4 mm vertical hop).
* Knee axis: ~20 mm slack loop (largest single-axis sweep, but the
  bundle only crosses it on the knee cable, anchored close to the
  joint by the `femur_knee_bracket` knee-cradle zip-tie post).

---

## 8. Software

There is no PWM pulse-width layer: the **Arduino Uno Q** runs Python on
its Linux side and drives the half-duplex STS3215 TTL bus directly at
1 Mbps via the host-side driver `pi_control/feetech_bus.py`. Each servo
takes an absolute **position command** (12-bit, count 2048 = 0°) and
reports position/load/voltage/current/temperature back over the same
bus, so the joint loop is closed inside each servo — you command angles,
not microseconds.

The driver maps the 18 logical joints to servo IDs 1..18
(`ID = leg*3 + axis + 1`), enforces the safe per-axis angle limits
(yaw ±35°, hip −80…+30°, knee −20…+80°), applies per-joint trims from
`pi_control/feetech_trims.json`, and sync-writes goal positions while
reading back live feedback:

```python
from pi_control.feetech_bus import FeetechBus

bus = FeetechBus(port="/dev/ttyACM0")   # Uno Q UART → TTL bus
bus.centre()                            # all joints → 0°

# command one joint (degrees; clamped to the safe per-axis limit):
bus.write_joint(joint=2, deg=60)        # leg 0 knee toward stance

# sync-write all 18 goal positions per gait tick:
bus.write_all(goal_degrees)             # list of 18 angles

# read live feedback (angle / load / volts / temp / current):
print(bus.read_feedback(joint=2))
```

Wrap that in a ~ 50 Hz loop that runs the closed-form 3R inverse
kinematics for each leg (coxa yaw, hip pitch, knee pitch — see e.g. the
"Phantom X / Lynxmotion AX hexapod" references) and a tripod-gait
scheduler, sync-writing all 18 goal positions per tick. The same
Python gait/IK code runs on your laptop (bench, via a USB→TTL lead) or
on the Uno Q on-robot — only `--port` changes. See `firmware/WIRING.md`
§2b for how the Uno Q sits in the loop and `feetech_bus.py` for the
full command set (`scan`, `centre`, `joint`, `stance`, `relax`,
`hold`, `feedback`, `trim`).

---

## 9. Tuning notes

* **Per-joint trim:** the first thing you do after assembly is drive
  every joint to 0° (`feetech_bus.py … centre`) and null out any
  mounting offset with `feetech_bus.py … trim <joint> <deg>` (clamped
  ±30°, saved to `feetech_trims.json` and re-applied on every command).
  Mechanical disc-horn/spline mounting lands within ~ one spline tooth,
  so expect a few degrees of trim per joint. Unlike the old PWM servos
  the STS3215 is **absolute** — it knows its angle the instant it
  powers on (12-bit encoder), so you can `relax` it, pose by hand, and
  `feedback` reads exactly where it is (no blind centre-on-boot slam).
* **Travel range:** the safe per-axis limits (yaw ±35°, hip −80…+30°,
  knee −20…+80°) are enforced in software by `feetech_bus.py`. Sweep
  each joint slowly with `joint <j> <deg> --sweep` to confirm no
  mechanical binding before the software limit; fix offsets with
  `trim`, not by re-bolting the horn.
* **Gait period:** start at 2 s per cycle (very slow, easy to debug),
  speed up to 1 s once the IK and trim are dialled.
* **Power supply sag:** if the robot collapses momentarily during
  swing-to-stance transitions, a per-leg power branch or the bus-bar
  feed is undersized. Verify the distributed harness against
  `firmware/WIRING.md` §6 — heavier 16–18 AWG branches, solid bus-bar
  crimps, and the 15–20 A main fuse — and watch each servo's reported
  voltage with `feetech_bus.py … feedback --watch` under load (the
  STS3215 run on the raw 3S rail, so a sag shows up directly in the
  per-servo voltage). The Uno Q's logic is on its own 12→5 V buck and
  is unaffected by servo-rail sag.

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

* The actuators talk EtherCAT or CAN instead of the prototype's
  half-duplex serial bus, so the joint driver layer changes.
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
| Robot "twitches" but won't move | Servo-rail sag, a loose bus-bar/branch crimp, or a blown leg fuse | Check the distributed power harness (`firmware/WIRING.md` §6); watch per-servo voltage with `feetech_bus.py … feedback --watch` under load |
| A servo is silent on the bus / `scan` misses an ID | Duplicate or unset servo ID, or a broken DATA jumper to that leg | Re-ID that servo alone (`setid`), and confirm the leg-to-leg signal+GND jumper is intact |
| One leg drags | Per-joint trim is off | Re-trim with `feetech_bus.py … trim <joint> <deg>` (the encoder is absolute, so this is a software offset, not a re-bolt) |
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
