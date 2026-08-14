# `rideable_v2` — Assembly Order (draft)

> Build order for the stand-in design; the BuildViz scene
> (`full_robot_viz/scene.json`, regenerate with `make viz`) shows every
> part named below in its home (parked) pose. Belt runs are not modelled
> in the scene — each stage's driver, driven pulley, and center distance
> are in [`PARTS.md` §2](PARTS.md#2-belt-drives-18-stages).

## 1. Single-joint bench rig first

Per [`BOM.md` §L](BOM.md#l-bench-test-order-de-risk-before-buying-18):
one hip joint (actuator → 8MGT-960-36 belt → 30205 shaft → lever arm),
its parking-pin lock, and a load to 0.36 kN·m — before ordering the
fleet.

## 2. Leg assembly (×6, identical)

1. **Coxa**: machine the yaw hub + box link + hip clevis as one weldment
   (`coxa_frame`). Press the yaw 32006 bearing pair; bolt the 72T yaw
   pulley (`yaw_pulley`) to the hub top.
2. **Hip joint**: press 30205 pairs into the coxa clevis **back-to-back**
   (O-arrangement — the single-shear moment needs the wide effective
   spread, [`PARTS.md` §3](PARTS.md#3-structural-joint-bearings-18-sets));
   fit the Ø25
   joint shaft (`hip_pivot_pin`) through the femur hub; bolt the 100T
   pulley (`hip_pulley`) — with its parking-pin hole ring — to the femur
   hub face. Mount the hip AK80-64 (`hip_actuator`) on the coxa face
   plate at C = 208 mm with the 25T sprocket hub through its **6905-2RS
   pilot bearing** ([`PARTS.md` §2](PARTS.md#2-belt-drives-18-stages)).
3. **Hip lock, then belt** (order matters): bench-verify the pin-lock
   clevis (`hip_lock`) spring-engage / solenoid-retract, then set it
   aside — the endless belt slides over the pulley **axially**, and a
   mounted lock body blocks that pass. Slip belt 8MGT-960-36 over the
   pulleys, THEN bolt the lock clevis to the coxa so the pin lines up
   with the web holes at r = 90 mm, fit the eccentric idler, and
   tension per [`PARTS.md` §2](PARTS.md#2-belt-drives-18-stages).
4. **Femur + knee drive**: the knee AK80-64 (`knee_actuator`) mounts
   ~140 mm from the hip axis on the femur's opposite face (with its own
   pilot bearing); its 3:1 belt (8MGT-896-36, C = 212 mm) runs down the
   femur to the 84T knee pulley (`knee_pulley`) bolted to the tibia
   hub. Knee joint shaft + 30205 pair as at the hip; knee lock
   (`knee_lock`) on the femur — same belt-before-lock order.
5. **Tibia + foot**: Ø50 tube (`tibia_frame`) to the ankle boss; foot
   pad + load cell (`foot`).
6. Fit belt guards and the mechanical travel stops
   ([`STRUCTURE.md` §5](STRUCTURE.md#5-mechanical-travel-stops)).

## 3. Chassis & body

1. Weld the hex frame (`chassis_frame`), fit the six vertex plates.
2. Hang the six yaw actuators (`yaw_actuator`) under the plates
   (C = 136 mm inboard of each yaw axis); fit yaw belts 8MGT-720-36.
3. Bolt each leg's yaw shaft (`yaw_pivot_pin`) up through the vertex
   plate.
4. Deck (`deck_plate`), battery (`battery`, low + central), e-bay
   (`e_bay`), saddle post + saddle, handlebar with e-stop (`controls`),
   footpegs.

## 4. Electrical & commissioning

Wire per [`POWER_SYSTEM.md`](POWER_SYSTEM.md) §1/§4 (pack → Class-T →
pre-charge + contactor → six 30 A branches; 24 V rail → 12 lock
solenoids through default-open drivers). Commission per
[`POWER_SYSTEM.md` §6](POWER_SYSTEM.md#6-hazards--commissioning) and the
ballast stages in [`README.md` §9](README.md#9-safety--ballast-protocol)
— **no rider until ballast testing passes.**
