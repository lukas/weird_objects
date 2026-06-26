# Robot Cat — Bill of Materials (rev 10: cute BABY KITTEN, manufacturable split shell)

A buildable, **cute** cat-like quadruped modelled on the proven open-source robot
cat **Petoi Nybble**: **2 actuated DOF per leg in the sagittal plane** (no hip
abduction), a **passive rigid spine**, and a **passive tail** — **9 servos** total.

**Rev-10 makes it a baby kitten.** It pushes the proportions to peak neoteny —
bigger domed head (head ≈ ½ body), bigger/rounder low-set eyes, a shorter round
**pot-belly** body on shorter stubbier legs with **oversized paws**, softer round
ears, and a shorter **softly-curled** tail — and fixes the old "detached tail"
bug (the tail is now one continuous Catmull-Rom-smoothed loft rooted in the rump,
tail↔body gap **0.04 mm**). All of rev 9's manufacturability is preserved.

**Rev-9 makes it manufacturable.** Rev 8 nailed the cute look but was a single
solid shell. Rev 9 splits the body on the sagittal plane into screw-together
`shell_left` + `shell_right` halves with a removable **belly hatch**, a separate
printed **head**, **M2 self-tap screw bosses** + an alignment lip at the seam, and
integral **servo / board / battery mounts** — without changing the silhouette.
(Rev 8 had chased **neoteny** on a Nybble-class micro platform: ~10 g micro servos
+ a compact ESP32 board, a big round head, chubby body, stubby legs with hidden
actuators, collar + bell.)

Coordinate frame: +X forward (nose), +Y left, +Z up, millimetres.

## Overall dimensions / mass

| Property | Value |
|---|---|
| Overall bounding box (L×W×H) | **227 × 141 × 179 mm** (nose-to-tail-curl × track × ear-top) |
| Body length (chest→tail base) | ~126 mm |
| Head diameter | ~92 mm (head ≈ ½ body — big domed kitten head) |
| Shoulder / hip pivot height | ~64 / ~68 mm |
| Foot support polygon | 92 mm (fore/aft) × 80 mm (track) |
| **Powered DOF** | **9** (8 leg + 1 neck) |
| **Micro-servo mass** | **9 × 11 g = 99 g** |
| Disc horns | 9 × ~1.5 g ≈ 14 g |
| ESP32 controller board | ~16 g |
| 2S LiPo battery (~450 mAh) | ~32 g |
| Printed PLA (split shell halves + hatch, head, stubby legs, knee covers, tail, mounts) | ~430 g |
| **Estimated total mass** | **~591 g** |
| Printed parts (distinct bodies) | **~35** |
| M2 self-tap screws (shell seam + hatch) | **11** (7 seam + 4 hatch) |

> Class: a compact **baby kitten** (~227 mm, a few hundred grams), even shorter +
> rounder than the rev-8/9 cat — *small + plump is part of cute*. The body is
> modelled as a solid
> silhouette but costed/massed as a **hollow ~2 mm-wall print** (surface-area ×
> wall × PLA density).

## Servo choice — a micro servo, for cuteness

The full-size **STS3215** (45 × 34 mm, 60 g) forced revs 6–7's long spindly legs
and clunky external knee boxes. Cute needs small, so rev 8 drops to a **micro
metal-gear serial servo** (Feetech P1S / SCS-class, ~MG90 footprint) — the
actuator class the cute Nybble actually uses. At the light ~591 g kitten it still
clears the torque target with room (rev 10's shorter legs shorten the moment arm):

| Servo | Stall | Standing margin | Trot margin | Verdict @591 g |
|---|---|---|---|---|
| **Micro metal-gear** | **0.29 N·m** | **5.4×** | **2.7×** | **Chosen** — small *and* strong enough |
| FEETECH STS3215 (revs 6–7) | 2.94 N·m | huge | huge | too big → un-cute |

Small enough that the elbow/knee actuator **hides inside a chubby leg** and the
hip/shoulder actuator **hides inside the body** — no external hardware shows.

### Real micro-servo dimensions used

| Spec | Value |
|---|---|
| Case | ~**23 × 12 × 24 mm** |
| Mass | ~**11 g** |
| Stall torque | ~**0.29 N·m @ 8.4 V** (metal-gear, HV) |
| Output | small disc horn; driven bone seats on the horn top (~15 mm above the case) |
| Bus | serial / PWM micro servos on one bus from the board |

## DOF map (9 servos)

| Location | Joints (axis) | Servos |
|---|---|---|
| Each leg ×4 | shoulder/hip **pitch** (Y) + elbow/knee **pitch** (Y) | 2 × 4 = **8** |
| Neck | head **pan** (Z) | **1** |
| Spine | — passive, rigid fused trunk | 0 |
| Tail | — passive decorative curl | 0 |
| **Total** | | **9** |

## Hidden actuators

- **Shoulder/hip** servos mount **inboard** on a printed `servo_mount` bracket; the
  chubby shell completely covers them (they live in the belly/haunch cavity). From
  outside you see only the body and the leg emerging from under it.
- **Elbow/knee** servos sit at the joint, wrapped in a smooth **FUR `knee` bulge**
  (a filleted rounded box sized to the case + ~2.8 mm) — a chubby joint, not a box.
  The knee cover IS the servo housing (opens to seat the servo).
- **Neck** servo seats in the neck on a `servo_mount`, hidden by the **collar**.

All leg joints hinge about the lateral (Y) axis (no hip abduction). The
**elbow/knee servo rides its upper segment** (it moves with the limb).

## Manufacturing — split, screw-together body (rev 9, carried into rev 10)

The body is split on the **sagittal (Y=0)** plane so the seam hides on the
spine/belly midline and each half prints dome-up on its flat face.

| Feature | What |
|---|---|
| `shell_left` + `shell_right` | the two body halves; mate at a **0.2 mm** print-fit seam |
| `belly_hatch` | removable underside panel (x ∈ [−32, 28] mm) over the electronics bay |
| `seam_lip` | inner tongue along the dorsal seam → halves register (anti-shear) |
| `seam_bosses` | 7 M2 self-tap bosses straddling Y=0 → pull the halves together |
| `hatch_frame` | recessed rim + 4 corner bosses the belly hatch screws into |
| `head_mount` | socket gripping the neck column / neck-servo horn (head bolts on) |
| `servo_mount` ×5 | bracket (rib + 2 bosses) per shoulder/hip/neck servo |
| `controller_tray` + `board_standoffs` | ESP32 board on 4 M2 posts |
| `battery_cradle` + `battery_strap` | LiPo held in the belly cradle |

## Printed parts (PLA) — ~35 distinct bodies

| Part | Qty | Notes |
|---|---|---|
| Body shell halves (`shell_left`, `shell_right`) | 2 | split sagittally; bosses/lip/servo pockets print integral |
| Belly hatch | 1 | removable electronics-bay panel |
| Neck | 1 | short + thick; neck-pan servo seats in it |
| Head (skull + molded face + ears) | 1 | big, high, rounded; mounts on the neck via `head_mount` |
| Tail (one passive taper) | 1 | short soft curl over the back (51-ring continuous loft) |
| Front-leg segments (upper arm, forearm, paw) | 3 × 2 = 6 | short, stubby, mirrored L/R |
| Hind-leg segments (femur, shank, paw) | 3 × 2 = 6 | short, stubby, mirrored L/R |
| **FUR knee covers** (`knee`) | **4** | smooth bulges housing each elbow/knee servo |
| Collar + bell | 2 | coral collar + gold bell; hides the neck joint + wiring |
| Servo-mount brackets | 5 | shoulder ×2, hip ×2, neck |
| Head mount | 1 | head→neck socket |
| Controller tray + battery cradle + strap | 3 | electronics mounts (board standoffs print on the tray) |

Per-joint assembly: *fixed segment / mount → micro servo → disc horn → moving
segment seated on the horn*. Bone segments are trimmed from each joint so they
**butt the servo body / horn** — no gaps; rev 10 trims the lower bone ~13 mm
below the elbow/knee so its root tucks under the knee cover (not the bare bone
into the belly), and the legs sit further outboard so they clear the round belly.

## Fasteners

| Fastener | Size | Qty | Use |
|---|---|---|---|
| **Shell seam screws** | **M2 self-tap × ~8 mm** | **7** | join `shell_left`+`shell_right` into `seam_bosses` |
| **Belly-hatch screws** | **M2 self-tap × ~6 mm** | **4** | `belly_hatch` → `hatch_frame` corner bosses |
| Servo case-mount screws | M2 self-tap | ~2 per servo (≈18) | servo → `servo_mount` / knee cover |
| Disc-horn / link screws | M2 | ~1–2 per joint × 9 | retain each driven link on the horn |
| Board standoffs | M2 × ~5 mm | 4 | ESP32 board → tray |

> **Insert choice:** M2 **self-tapping screws into printed bosses** (boss OD 5.2 mm,
> ~2 mm pilot) are the standard at this scale. For parts opened repeatedly (the
> belly hatch), **M2 brass heat-set inserts** are an optional, more durable upgrade.

## Electronics / power (modelled, inside the belly cavity)

| Item | Real spec | Qty | Where it mounts |
|---|---|---|---|
| **ESP32 quadruped board** (NyBoard/BiBoard-class) | ~**46 × 30 × 9 mm**, ~16 g, on-board servo driver | 1 | low in the chest cavity at ≈ (24, 0, 62) mm, PCB flat, fully enclosed |
| **2S LiPo battery** (7.4 V, ~450 mAh) | ~**46 × 28 × 12 mm**, ~32 g | 1 | **low & ~centred in the belly** at ≈ (−8, 0, 37) mm, on the cradle floor (the floor auto-bridges down to the deeper rev-10 belly wall) |
| Controller tray (printed) | small plate | 1 | carries the board, tied to the cradle riser |
| Battery cradle + riser (printed) | floor + 2 rails + riser | 1 | holds the LiPo low; touches the inner belly wall |

**Why not the Arduino Uno Q?** Its 68.58 × 53.34 mm board is too big for a cute
compact kitten body without bloating it, so rev 8 switches to a Nybble-class
ESP32 board that fits the belly with room to spare and hides completely (the
rev-10 deeper pot-belly gives even more vertical clearance).

**Power/wiring:** the 2S pack feeds the servo bus + the board; the 9 micro servos
run from the board's servo bus (a drop to each shoulder/hip servo, a jumper to that
leg's knee servo; the neck servo taps the front, hidden by the collar).

## Rough cost / feasibility

| Item | Est. cost |
|---|---|
| 9 × micro metal-gear servos @ ~$4 | ~$36 |
| ESP32 quadruped board | ~$25 |
| 2S LiPo (~450 mAh) + lead | ~$8 |
| Wiring + M2 screws/bosses + standoffs | ~$10 |
| PLA + horns | ~$10 |
| **Total** | **~$89** |

> The cute micro platform is **much cheaper** (~$89) than the STS3215 build
> (~$250) — smaller servos, a small board, and far less plastic. The split
> shell + M2 screws add only ~$2.

## Feasibility findings (numbers, this build)

- **Static stability — verified.** Estimated total mass **~591 g**, CoM at
  **(x = 23, y = 0, z = 72.8) mm** projects to the ground **27 mm inside** the
  4-paw support polygon `[(-42,±40), (50,±36)]` → a stable standing stance even
  with the big kitten head forward (battery low + aft balances it).
- **Torque — adequate margin.** Knee moment arm **37 mm** (shorter stubby legs);
  micro-servo stall 0.29 N·m. **Standing knee torque ≈ 0.054 N·m → 5.4× margin**;
  trot ≈ 0.108 N·m → **2.7×**. Clears the ≥3× standing / ≥1.5× trot target with room.
- **Connectivity — verified buildable.** `check_assembly.py`: **one connected
  component** (all 75 parts touch within 2 mm), **0 hovering parts**, all 4 paws
  on z = 0. `check_overlaps.py`: **0 unexpected interpenetration** (split shell
  halves + hatch **mate** at the 0.2 mm seam, not interpenetrate). The rebuilt
  tail is one continuous mesh rooted **0.04 mm** from the rump (no detached tip).
- **Gait — kinematically feasible.** 92 × 80 mm support polygon with a 27 mm
  margin permits a statically-stable crawl; short stubby legs trade stride length
  for adorableness.
- **Limitations (honest).** Geared position micro servos + a passive spine + short
  legs → good for standing, posing, and a slow static walk; no trot/gallop/bound
  /jump at speed. Cuteness was prioritized over stride/agility on purpose.
