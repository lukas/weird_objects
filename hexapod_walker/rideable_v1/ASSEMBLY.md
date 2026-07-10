# `rideable_v1` — Assembly Directions

> **Status: design-draft assembly outline.** `rideable_v1` is a
> mechanical-design draft (no released parametric CAD or fabrication
> drawings yet — see [`README.md`](README.md) §1). This document captures the
> intended **build and assembly sequence** so the order of operations, the
> load-bearing interfaces, and the safety-critical steps are recorded now and
> not re-derived later. Dimensions and part numbers live in
> [`design_spec.yaml`](design_spec.yaml) and [`BOM.md`](BOM.md); the reasoning
> behind each subsystem lives in [`STRUCTURE.md`](STRUCTURE.md),
> [`DRIVETRAIN.md`](DRIVETRAIN.md), and [`POWER_SYSTEM.md`](POWER_SYSTEM.md).

The BuildViz scene under `full_robot_viz/` is a primitive stand-in
visualization of the assembled machine; open it in the central hub
(`http://127.0.0.1:5183/?build=rideable_v1`) to see the part placement this
sequence produces.

## 0. Before you build — de-risk (BOM §L)

Do **not** buy 18 actuators up front. Follow the bench-test order in
[`BOM.md`](BOM.md) §L: prove one knee joint (actuator + 6:1 secondary +
fail-safe brake) holds the standing load case ([`DRIVETRAIN.md`](DRIVETRAIN.md)
§4.3/§5) on a static rig before committing to the full lineup.

## 1. Fabricate the leg space-frames (×6)

Each leg is three welded 4130 chromoly space-frame segments — **coxa**
(hip-yaw → hip-pitch, ~150 mm), **femur** (hip-pitch → knee, ~600 mm), and
**tibia** (knee → foot, ~800 mm). Build them as triangulated 3-chord trusses
per [`STRUCTURE.md`](STRUCTURE.md) §3 and the member schedule in §8:

1. Jig and tack the main chords (38 mm OD × 2.4 mm wall) at the ~150 mm frame
   depth that sets the bending-couple chord force.
2. Add the diagonals (25 mm OD × 1.6 mm wall) to triangulate each bay.
3. Weld out, following the node/weld handling in §6.1 and §6.4 (no
   stress-raiser terminations at the truss nodes).
4. Weld in the motor-mount, brake-mount, and foot bolt bosses (§6.2) and the
   urethane-pad **foot** disc (§6.3, §E).

## 2. Build the chassis

Weld the two-level hex tube ring with verticals and radial spokes
([`STRUCTURE.md`](STRUCTURE.md) §7), then bolt/weld the aluminium **deck
plate** on top. The chassis carries the six hip-yaw actuators at the
perimeter and provides the low, central mounting volume for the battery and
e-bay.

## 3. Populate the joints with actuators + drivetrain

Per leg, working outboard from the body ([`DRIVETRAIN.md`](DRIVETRAIN.md) §2):

1. **Hip-yaw:** mount an RMD-X8-120 vertical-axis actuator to the chassis
   perimeter; bolt the coxa to its output (direct drive, no secondary).
2. **Hip-pitch:** mount an RMD-X15-450 and its 6:1 HTD-14M secondary
   (driven pulley on the joint axis) between coxa and femur.
3. **Knee:** mount an RMD-X15-450 and its 6:1 secondary between femur and
   tibia, then fit the **fail-safe spring-applied brake** on the actuator's
   **fast (input) shaft** (§4.1/§4.2). The knee brake is safety-critical —
   verify holding torque (~160 N·m at the fast shaft) before loading.

## 4. Wire the 72 V power system

Follow [`POWER_SYSTEM.md`](POWER_SYSTEM.md): mount the 72 V (20S) Li-ion pack
low and central, the e-bay (controllers + 72→48 V DC-DC for the hip-yaw rail)
adjacent, and install fusing, the main contactor, and the **fail-safe brake
power interlock** (§5) so that any power loss engages every knee brake.

## 5. Rider interface

Mount the suspension **saddle post** + **saddle**, the **controls**
(handlebar + grip + e-stop), and the two welded **footpegs** to the deck
([`README.md`](README.md) §4, [`BOM.md`](BOM.md) §F).

## 6. Commissioning (do not skip)

Before any rider load, run the commissioning + hazard checks in
[`POWER_SYSTEM.md`](POWER_SYSTEM.md) §6 and the safety/reality checks in
[`README.md`](README.md) §9: confirm each fail-safe brake engages on power
loss, verify the tucked-stance standing load case on all six legs, and only
then attempt a gaited walk at the ~0.4 m/s cruise target.
