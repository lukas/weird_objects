"""Build the full assembled PROTOTYPE hexapod walker (hobby-servo
edition) and export it as four category STLs the Blender render script
treats as separate materials.

Imports the part builders from `hexapod_prototype.py`, then places:
    - 1 chassis (top + bottom plate)
    - 1 battery holder + a 3S LiPo pack
    - 1 electronics tray + Arduino Nano + PCA9685 PWM driver
    - 6 leg sub-assemblies, each with:
        coxa bracket, coxa link, femur, tibia, foot
        + 3 hobby servo bodies (yaw, hip-pitch, knee-pitch)
        + 3 aluminum 25T disc servo horns (Amazon B07D56FVK5 -- the
          links now bolt DIRECTLY onto these with 4 x M3 x 6 SHCS; the
          printed servo_horn_adapter disc and the stock plastic 4-arm
          X-horn are both retired, see hexapod_prototype.py)

Outputs (in ./prototype_assembly/):
    frame.stl       printed PLA / PETG structural parts
    motors.stl      18 black plastic hobby servo bodies
    battery.stl     LiPo pack + Arduino + PCA9685
    soft.stl        rubber foot pads + cable jackets
    full.stl        single-mesh dump for non-Blender viewers

All output is rotated to Y-up so it matches the convention used by
build_full_assembly.py (and Blender's STL importer with up=Y).
"""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

import hexapod_prototype as HP


OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "prototype_assembly")
os.makedirs(OUT_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# Local helpers (re-using HP where possible)
# ---------------------------------------------------------------------------

def _cyl(radius: float, height: float, sections: int = HP.CYL_SECTIONS):
    return trimesh.creation.cylinder(radius=radius, height=height,
                                      sections=sections)


def _box(extents, center=(0.0, 0.0, 0.0)):
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(center)
    return m


# ---------------------------------------------------------------------------
# Visually rich hobby-servo body (used 18 x in the assembly)
# ---------------------------------------------------------------------------

def _hobby_servo_visual() -> trimesh.Trimesh:
    """A nicer-looking hobby servo than HP._servo_envelope.

    Local frame matches HP._servo_envelope:
        +Z = output shaft
        +X = body long axis (output offset toward +X)
        +Y = body short axis (depth)
        Origin = centre of body bottom face

    Adds details:
        - Chamfered body (subtle) with a top-cap groove
        - Output gear stack
        - Mounting tabs with a couple of tab-screws modelled
        - 3-wire cable bundle exiting on the -X side
    """
    parts = []

    # Body
    body = _box((HP.SERVO_BODY_W, HP.SERVO_BODY_D, HP.SERVO_BODY_H),
                center=(0, 0, HP.SERVO_BODY_H / 2.0))
    parts.append(body)

    # Top cap (slightly wider, models the gear case)
    cap = _box((HP.SERVO_BODY_W + 1.0, HP.SERVO_BODY_D + 1.0, 4.0),
               center=(0, 0, HP.SERVO_BODY_H + 2.0))
    parts.append(cap)

    # Mounting tabs
    tab_extra = (HP.SERVO_TAB_W - HP.SERVO_BODY_W) / 2.0
    for sx in (-1, 1):
        tab = _box((tab_extra, HP.SERVO_BODY_D, HP.SERVO_TAB_T),
                   center=(sx * (HP.SERVO_BODY_W / 2.0 + tab_extra / 2.0),
                            0, HP.SERVO_TAB_Z))
        parts.append(tab)

    # Output gear stack
    gear_lo = _cyl(HP.SERVO_OUTPUT_OD / 2.0, HP.SERVO_OUTPUT_H)
    gear_lo.apply_translation([HP.SERVO_OUTPUT_X, 0,
                                HP.SERVO_BODY_H + 4.0
                                    + HP.SERVO_OUTPUT_H / 2.0])
    parts.append(gear_lo)

    spline = _cyl(HP.SERVO_SPLINE_OD / 2.0, 3.0)
    spline.apply_translation([HP.SERVO_OUTPUT_X, 0,
                                HP.SERVO_BODY_H + 4.0
                                    + HP.SERVO_OUTPUT_H + 1.5])
    parts.append(spline)

    # Top-side decorative groove (simulated with a thin band recess)
    # -- skip for simplicity / triangle-count reasons.

    # Cable bundle on +X face -- the real DS3225's molded wire-exit boot
    # protrudes from the +X SHORT face (same side as the output gear); the
    # cradle's parametric ``_wire_exit_slot`` + ``WIRE_BOOT_*`` geometry
    # all puts the boot on the +X face, so the visual mesh must match.
    # Earlier versions drew this bundle on the -X face which made the
    # inspector renders disagree with the real harness routing.
    bundle = _cyl(2.2, 18.0)
    bundle.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    bundle.apply_translation([+HP.SERVO_BODY_W / 2.0 + 9.0,
                               0, HP.SERVO_BODY_H * 0.55])
    parts.append(bundle)

    return trimesh.util.concatenate(parts)


def _horn_visual() -> trimesh.Trimesh:
    """The 20 mm aluminum 25T DISC servo horn (Amazon B07D56FVK5) that
    every servo joint now drives.  Used 18 x to dress the output of
    every joint; the driven link bolts onto the disc's flat top face
    with 4 x M3 SHCS into the disc's tapped holes.

    Delegates to ``HP.make_disc_horn`` so the assembly, the static
    ``servo_horn.stl`` and the verifier all share ONE disc geometry.

    Local frame: bottom face (spline side) at z=0, top mating face at
    z=HP.HORN_STACK_H = HP.DISC_HORN_H = 5 mm, so callers that
    translate by ``horn_base_z`` get a disc whose mating face lands at
    ``horn_base_z + HP.HORN_STACK_H`` -- the same reference the plastic
    X-horn used, so no placement math changes.
    """
    return HP.make_disc_horn()


# ---------------------------------------------------------------------------
# Per-leg category meshes
# ---------------------------------------------------------------------------

def _build_leg(leg_index: int):
    """Return (frame_parts, motor_parts, soft_parts) for one leg in
    chassis frame.  Uses the same kinematic chain as
    HP._leg_in_body_frame but splits output by category and inserts a
    hobby-servo body + horn at every joint.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a),
                         apothem * np.sin(a),
                         0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    frame_parts = []
    motor_parts = []
    soft_parts  = []

    # Standing-pose pitch angles
    p  = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)

    R_a = rotation_matrix(a, [0, 0, 1])
    R_a_3 = R_a[:3, :3]

    # Yaw output reference height (above the chassis edge plane).
    # Must match HP._leg_in_body_frame's vertical stack.  May 2026:
    # the yaw servo migrated from the standalone ``coxa_bracket`` to
    # the integrated cradle inside ``chassis_bottom``.  Commit 4 of
    # the transition (this commit) flips the assembly's yaw-servo
    # placement so the tabs land on the chassis_bottom cradle's tab
    # shelf at chassis-z = CHASSIS_PLATE_T/2 + CRADLE_TAB_SHELF_Z =
    # +2 + +6 = +8 mm (= +8 mm UP from the bracket shelf which was
    # at chassis-z = 0).  The disc-horn / coxa_link mating plane
    # follows the servo: +8 mm UP from the legacy yaw_output_z =
    # +21.75 to the new +29.75.  See the constant block near the
    # CRADLE_BOSS_H_MM definition in hexapod_prototype.py for the
    # Path-A history that placed the cradle shelf at +6.
    #
    # Z-stack (chassis-z = 0 = chassis_bottom CENTRE; world-z post-
    # lift is chassis-z + chassis_lift):
    #   chassis_bottom top     z = +CHASSIS_PLATE_T/2 = +2
    #   cradle tab shelf       z = +CHASSIS_PLATE_T/2
    #                                + CRADLE_TAB_SHELF_Z = +8
    #   servo body bottom      z = shelf - WELL_RIM_Z = -19.25
    #   servo body top         z = body_bottom + SERVO_BODY_H = +18.75
    #   output-gear top        z = body_top + SERVO_OUTPUT_H = +24.75
    #   disc-horn top (yaw_output_z): z = gear_top + HORN_STACK_H = +29.75
    #   chassis_top bottom     z = +CHASSIS_GAP + CHASSIS_PLATE_T/2
    #                                - CHASSIS_PLATE_T/2 = +CHASSIS_GAP = +32
    #     (= 32 - 29.75 = 2.25 mm clearance from disc-horn top to top
    #      plate; only relevant at the yaw axis which sits outside
    #      chassis_top's 70-mm apothem hex anyway, so no XY overlap)
    #
    # The legacy bracket is still PLACED in the assembly during this
    # transition commit; the bracket's well well-z = -WELL_RIM_Z body
    # bottom would be at chassis-z = -27.25 (8 mm BELOW the new servo
    # position).  We place the servo at the NEW cradle position; the
    # bracket overlaps the cradle's outer shell in chassis-z [+2, +11]
    # and the servo body hovers above the bracket's well floor at
    # chassis-z = -19.25 (8 mm above the bracket's nominal well-
    # floor seating plane).  This visual overlap is intentional for
    # the transition; commit 8 retires the bracket entirely.
    _CRADLE_SHELF_CHASSIS_Z = (
        HP.CHASSIS_PLATE_T / 2.0 + HP.CRADLE_TAB_SHELF_Z
    )  # = +8 mm; chassis-z of the chassis_bottom cradle's tab shelf.

    yaw_output_z = HP.CHASSIS_YAW_OUTPUT_Z   # = +29.75 mm; disc-horn top
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    arm_t = 6.0       # MUST match make_coxa_link()'s arm_t
    hip_drop = HP.COXA_HIP_DROP
    hip_joint_local = np.array([HP.COXA_LENGTH, 0.0, hip_drop])
    Ry_p_3 = rotation_matrix(p, [0, 1, 0])[:3, :3]
    # Femur's knee-end joint axis is on the spar centreline at z=0
    # in femur local, so we don't add a 'drop' offset for the knee.
    knee_joint_local = hip_joint_local + Ry_p_3 @ np.array([HP.FEMUR_LENGTH,
                                                             0.0, 0.0])
    # NEW (May 2026 collinear-pad refactor): the femur / tibia NEW
    # local origins are their PAD MATING FACES, which sit HORN_STACK_H
    # above their respective joint axes along link +Y.  Femur +Y is
    # parallel to coxa +Y here (the hip-pitch axis), so the entire
    # leg shifts +HORN_STACK_H in coxa-Y as a rigid body relative to
    # the joint axes.  Pre-refactor the links' local origins were on
    # the joint axes themselves (no +Y offset).
    PAD_AXIS_OFFSET = np.array([0.0, HP.HORN_STACK_H, 0.0])

    def to_world(mesh):
        mesh.apply_transform(R_a)
        mesh.apply_translation(yaw_output_world)
        return mesh

    # ------------- Yaw servo (output up) ------------------------------
    # Servo body sits below the chassis with output gear poking up.
    # In bracket-local: body bottom at z = -SERVO_BODY_H, output at z=0.
    # The bracket itself is in leg-local coords with edge_mid at z=0.
    yaw_servo = _hobby_servo_visual()
    # The yaw servo body sits in the chassis_bottom-integrated yaw
    # cradle.  In the chassis frame (yaw axis at x=0, chassis_bottom
    # centre at z=0, chassis_bottom top at z = +CHASSIS_PLATE_T/2),
    # the body's bottom face lands at chassis-z =
    # _CRADLE_SHELF_CHASSIS_Z - WELL_RIM_Z (the cradle's tab shelf
    # rests under the servo's tab bottoms) and the body's long axis
    # is along +X with output offset toward +X.
    # ``_hobby_servo_visual`` has body bottom at z=0 and output at
    # (SERVO_OUTPUT_X, 0, ...), so we translate by
    # (-SERVO_OUTPUT_X, 0, _CRADLE_SHELF_CHASSIS_Z - WELL_RIM_Z) to
    # land the body bottom at the right chassis-z and the output
    # gear's centre on the yaw axis (chassis-x = 0).
    yaw_servo.apply_translation(
        [-HP.SERVO_OUTPUT_X, 0,
         _CRADLE_SHELF_CHASSIS_Z - HP.WELL_RIM_Z],
    )
    yaw_servo.apply_transform(R_a)
    yaw_servo.apply_translation(edge_mid)
    motor_parts.append(yaw_servo)

    # Yaw disc horn -- sits on top of the gear stack (above the body's
    # exposed top by SERVO_OUTPUT_H), drives the coxa link directly
    # via the 4 link-to-disc-horn M3 x 6 bolts (June 2026 disc-horn
    # switch; no more printed adapter disc).
    yaw_horn = _horn_visual()
    yaw_horn.apply_translation(
        [0, 0,
         _CRADLE_SHELF_CHASSIS_Z
         + (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
         + HP.SERVO_OUTPUT_H],
    )
    yaw_horn.apply_transform(R_a)
    yaw_horn.apply_translation(edge_mid)
    motor_parts.append(yaw_horn)

    # ------------- Coxa link (rotates with yaw, frame) -----------------
    cl = HP.make_coxa_link()
    cl.apply_transform(R_a)
    cl.apply_translation(yaw_output_world)
    frame_parts.append(cl)

    # ------------- Hip-pitch servo (output along +Y_local) -------------
    # The cradle in the coxa link expects the servo with:
    #   servo +Z (output) = leg +Y_local
    #   servo body extends in -Y_local from the output
    # Output point lives at (COXA_LENGTH, 0, hip_drop) in coxa-link
    # local.  In leg-local the servo's local frame is rotated by -90
    # about +X (servo +Z -> leg +Y, servo +Y -> leg -Z).
    hip_servo = _hobby_servo_visual()
    R_hip = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    hip_servo.apply_transform(R_hip)
    # After R_hip the servo's body bottom (z=0) maps to post-y=0
    # and the spline tip (z=SERVO_BODY_H+SERVO_OUTPUT_H) maps to
    # post-y=SERVO_BODY_H+SERVO_OUTPUT_H.  We want the spline tip to
    # land on the joint axis at (COXA_LENGTH, 0, hip_drop):
    delta = np.array([HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
                       -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
                       hip_drop])
    hip_servo.apply_translation(delta)
    to_world(hip_servo)
    motor_parts.append(hip_servo)

    # Hip horn -- bolted to the servo output, sticking out into +Y
    # from the cradle face.  The femur's hip-pad clamps onto this.
    hip_horn = _horn_visual()
    R_hip_horn = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    hip_horn.apply_transform(R_hip_horn)
    hip_horn.apply_translation([HP.COXA_LENGTH, 0.0, hip_drop])
    to_world(hip_horn)
    motor_parts.append(hip_horn)

    # ------------- Femur (rotates with hip-pitch) ----------------------
    fl = HP.make_femur_link()
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    to_world(fl)
    frame_parts.append(fl)

    # ------------- Knee-pitch servo (output along +Y_local) ------------
    # Lives at the femur's far end.  In NEW femur-local coords the
    # knee servo's spline tip is at (FEMUR_LENGTH, -HORN_STACK_H, 0)
    # (the joint axis is HORN_STACK_H below the link's NEW origin =
    # pad mating face); the body sits HORN_STACK_H deeper in -Y than
    # the pre-refactor placement so its world position is unchanged.
    knee_servo = _hobby_servo_visual()
    knee_servo.apply_transform(R_hip)
    delta = np.array([HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
                       -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H) - HP.HORN_STACK_H,
                       0])
    knee_servo.apply_translation(delta)
    knee_servo.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_servo.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    to_world(knee_servo)
    motor_parts.append(knee_servo)

    knee_horn = _horn_visual()
    knee_horn.apply_transform(R_hip_horn)
    knee_horn.apply_translation([HP.FEMUR_LENGTH, -HP.HORN_STACK_H, 0])
    knee_horn.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_horn.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    to_world(knee_horn)
    motor_parts.append(knee_horn)

    # ------------- Tibia (rotates with knee-pitch) ---------------------
    tl = HP.make_tibia_link()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local + PAD_AXIS_OFFSET)
    to_world(tl)
    frame_parts.append(tl)

    # ------------- Foot at tibia tip ----------------------------------
    # June 2026 crab-spike: the tibia curves to an integral spike, so there
    # is no separate hinged foot_pad to place.  The spike tip (part of the
    # tibia frame mesh) is the ground contact, and the chassis_lift solve
    # in main() lands the leg on that tip.
    return frame_parts, motor_parts, soft_parts


# ---------------------------------------------------------------------------
# Body parts builders (categorised)
# ---------------------------------------------------------------------------

def _body_frame_parts(chassis_lift):
    parts = []
    bot = HP.make_chassis_bottom()
    bot.apply_translation([0, 0, chassis_lift])
    parts.append(bot)

    top = HP.make_chassis_top()
    top.apply_translation([0, 0, chassis_lift + HP.CHASSIS_GAP
                                + HP.CHASSIS_PLATE_T])
    parts.append(top)

    # 4 stand-off posts between the plates on the rotated-45-deg 35-mm-
    # radius pattern (= HP.CHASSIS_STANDOFF_HOLES_XY = (+/-35, 0) and
    # (0, +/-35)).  May 2026 tray-mount fix: the standoffs MOVED off
    # HP.ELEC_CHASSIS_MOUNT_HOLES_XY (the +/-24.75 square pattern) so
    # the chassis_bottom plate could carry an M3 heat-set insert at
    # each of those 4 XY positions for the tray-mount bolts without
    # the brass standoff body conflicting with the insert pocket.
    # Each standoff is M3 M-F, length HP.CHASSIS_GAP = 32 mm.
    for (sx, sy) in HP.CHASSIS_STANDOFF_HOLES_XY:
        post = _cyl(2.5, HP.CHASSIS_GAP)
        post.apply_translation([sx, sy,
                                  chassis_lift + HP.CHASSIS_PLATE_T
                                      + HP.CHASSIS_GAP / 2.0])
        parts.append(post)

    return parts


def _body_battery_parts(chassis_lift):
    """Battery holder, LiPo pack, electronics tray + the three control
    boards mounted on it (Arduino Mega 2560, Raspberry Pi 4, PCA9685).

    ``bh_z0`` is the world z of the battery_holder's bottom face,
    which mates with the chassis_bottom mesh's TOP face.  The
    chassis_bottom mesh is centred on its own z = 0 (it's a hex
    extrusion of CHASSIS_PLATE_T = 4 mm so its top face is at
    chassis_lift + CHASSIS_PLATE_T / 2 = chassis_lift + 2 in world
    coordinates), so the holder's bottom face must also sit there.
    Pre-May-2026 the holder was translated by full CHASSIS_PLATE_T
    (= 4 mm) which left it floating 2 mm above the chassis_bottom
    mesh; the verifier's check_fastener_engagement caught this
    when battery_holder bolts started being enumerated.
    """
    parts = []
    bh_z0 = chassis_lift + HP.CHASSIS_PLATE_T / 2.0

    # Holder shell.  ``HP.BATTERY_HOLDER_CENTRE_X`` (= -25 mm) is the
    # single source of truth for the holder's chassis-frame X
    # position; the fastener_registry, _hex_plate, the verifier and
    # inspect_build all read the same constant so the chassis_bottom
    # bolt-pass holes line up with the holder's feet.
    bh = HP.make_battery_holder()
    bh.apply_translation([HP.BATTERY_HOLDER_CENTRE_X, 0, bh_z0])
    parts.append(bh)

    # The LiPo pack itself (visible inside the holder).  Standard 3S
    # 2200 mAh pack: ~ 105 x 35 x 25 mm with shrink-wrap label.
    lipo = _box((105.0, 35.0, 25.0),
                center=(HP.BATTERY_HOLDER_CENTRE_X, 0,
                         bh_z0 + HP.BATTERY_WALL + 25.0 / 2.0))
    parts.append(lipo)

    # Electronics tray + Mega 2560 + Pi 4 + PCA9685.  Tray base sits
    # 3 mm above the chassis_bottom top face (= bh_z0 + 3) so the
    # tray-local origin lands at chassis frame
    # (ELEC_TRAY_CENTRE_X, ELEC_TRAY_CENTRE_Y, bh_z0 + 3).  May 2026
    # tray-expansion pass moved the tray centre from the previous
    # (+35, 0) to (0, 0) so the tray's 4 chassis-mount holes
    # actually align with the chassis-side 35-mm-radius pattern at
    # chassis (+/-24.75, +/-24.75); the old offset put them at
    # (+10.25, +/-24.75) and (+59.75, +/-24.75) instead -- a
    # longstanding bug masked by the assembly preview being purely
    # visual.
    tray_top_z = bh_z0 + 3.0 + HP.ELEC_TRAY_T
    board_top_z = tray_top_z + HP.ELEC_STANDOFF_H
    et = HP.make_electronics_tray()
    et.apply_translation([HP.ELEC_TRAY_CENTRE_X, HP.ELEC_TRAY_CENTRE_Y,
                           bh_z0 + 3.0])
    parts.append(et)

    # Arduino Mega 2560 R3 (53.3 x 101.5 x 1.6 mm bare PCB; ~ 8 mm
    # overall including the ATmega2560 chip + headers + USB-B
    # connector underside).  Long axis along chassis +X (90 deg CW
    # from "as designed", which is how MEGA_HOLES is built).
    mega = _box((HP.MEGA_PCB_D, HP.MEGA_PCB_W, 8.0),
                center=(HP.ELEC_TRAY_CENTRE_X + HP.MEGA_CENTRE[0],
                         HP.ELEC_TRAY_CENTRE_Y + HP.MEGA_CENTRE[1],
                         board_top_z + 8.0 / 2.0))
    parts.append(mega)

    # Raspberry Pi 4 Model B / Pi 5 (85 x 56 x 1.5 mm bare PCB; ~ 18
    # mm overall with the Ethernet jack + dual USB-A 3.0 stack +
    # ATSoC + heat-sink).  Long axis along chassis +X (PI_HOLES is
    # the un-rotated board-centre offset set).
    pi = _box((HP.PI_PCB_W, HP.PI_PCB_D, 18.0),
              center=(HP.ELEC_TRAY_CENTRE_X + HP.PI_CENTRE[0],
                       HP.ELEC_TRAY_CENTRE_Y + HP.PI_CENTRE[1],
                       board_top_z + 18.0 / 2.0))
    parts.append(pi)

    # Adafruit PCA9685 PWM drivers (62 x 25 x 8 mm each).  Long axis
    # along chassis +Y (rotated 90 deg).  May 2026 "essentials"
    # pass: BOTH PCA9685s are now properly bolted to the tray (PCA1
    # at I2C 0x40, PCA2 at I2C 0x41).  Symmetric about the tray X
    # axis.
    for label, centre in (("PCA9685(0x40)", HP.PCA_CENTRE),
                          ("PCA9685(0x41)", HP.PCA2_CENTRE)):
        pca = _box((HP.PCA_PCB_D, HP.PCA_PCB_W, 8.0),
                   center=(HP.ELEC_TRAY_CENTRE_X + centre[0],
                            HP.ELEC_TRAY_CENTRE_Y + centre[1],
                            board_top_z + 8.0 / 2.0))
        parts.append(pca)

    # BEC cradle: snap-fit clip for 2 x switching BECs.  Sits on the
    # tray's TOP face in the corridor between Mega's +X edge and
    # PCA1's -X edge (BEC_CRADLE_CENTRE in tray-local coords).  May
    # 2026 "essentials" pass.
    bec = HP.make_bec_cradle()
    bec.apply_translation([HP.ELEC_TRAY_CENTRE_X + HP.BEC_CRADLE_CENTRE[0],
                            HP.ELEC_TRAY_CENTRE_Y + HP.BEC_CRADLE_CENTRE[1],
                            tray_top_z])
    parts.append(bec)

    # Switch holster: anti-spark on/off switch.  Sits on 2 printed
    # bosses on chassis_top's top face.  Bolted DOWN from above
    # with 2 x M3 x 10 SHCS into 2 x M3 heat-set inserts captive in
    # those chassis_top bosses.  Toggle protrudes past the chassis
    # +X flat for user access.  May 2026 "essentials" pass.
    chassis_top_top_z = (chassis_lift + HP.CHASSIS_PLATE_T
                          + HP.CHASSIS_GAP + HP.CHASSIS_PLATE_T / 2.0)
    holster_z = chassis_top_top_z + HP.SWITCH_HOLSTER_BOSS_H
    holster = HP.make_switch_holster()
    holster.apply_translation([HP.SWITCH_HOLSTER_CENTRE_X,
                                HP.SWITCH_HOLSTER_CENTRE_Y,
                                holster_z])
    parts.append(holster)

    # IMU pad: vibration-isolated mounting plate for the MPU-6050 /
    # GY-521 breakout (May 2026: IMU promoted from optional to
    # standard kit, now that the Pi 4 / Pi 5 expects orientation
    # feedback for closed-loop body-attitude control on uneven
    # terrain).  Sits at chassis (0, 0) -- the centre of mass of
    # the chassis -- on a 3 mm-thick strip of double-sided foam tape
    # that doubles as the mount AND the vibration damper.  No
    # fasteners between the pad and chassis_top; the foam tape is
    # the entire interface.  Cable corridor exits +X toward the I2C
    # cluster on the electronics_tray below (PCA9685s).
    imu_z = chassis_top_top_z + HP.IMU_PAD_TAPE_T
    imu_pad = HP.make_imu_pad()
    imu_pad.apply_translation([HP.IMU_PAD_CENTRE_X,
                                HP.IMU_PAD_CENTRE_Y,
                                imu_z])
    parts.append(imu_pad)

    # MPU-6050 / GY-521 breakout PCB visual mesh (simple slab; the
    # 8-pin header along the +X long edge is omitted for triangle-
    # count reasons).  Sits on TOP of the IMU pad's 4 bosses at z =
    # pad_bottom + IMU_PAD_T + IMU_PAD_BOSS_H.
    imu_top_z = imu_z + HP.IMU_PAD_T + HP.IMU_PAD_BOSS_H
    imu_pcb = _box((HP.IMU_PCB_W, HP.IMU_PCB_D, HP.IMU_PCB_T),
                    center=(HP.IMU_PAD_CENTRE_X,
                             HP.IMU_PAD_CENTRE_Y,
                             imu_top_z + HP.IMU_PCB_T / 2.0))
    parts.append(imu_pcb)

    return parts


def _body_soft_parts(chassis_lift):
    """No saddle/grips on the prototype.  The 6 foot pads come in via
    `_build_leg` so `soft.stl` is non-empty without us adding
    anything here."""
    return []


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _yup(mesh):
    mesh.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    return mesh


def _optional_arm_meshes(chassis_lift: float) -> list[trimesh.Trimesh]:
    """Build the optional arm in neutral pose, placed on top of the
    standing hexapod's chassis top plate.  Returns an empty list if
    the arm module can't be imported (e.g. ``prototype/arm/`` is
    missing).  No frame coordinate flip is applied here -- the caller
    must run them through the same ``_yup()`` rotation as the rest of
    the assembly meshes.
    """
    _arm_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "arm")
    if _arm_dir not in sys.path:
        sys.path.insert(0, _arm_dir)
    try:
        import arm as ARM_MOD  # noqa: WPS433
    except Exception as exc:
        print(f"  WARN: optional arm import failed ({exc!r}); skipping arm.")
        return []

    # Chassis-top plate centre in assembly frame:
    #   z_centre = chassis_lift + CHASSIS_GAP + CHASSIS_PLATE_T
    # so its TOP face (where the arm bracket lands) is half a plate
    # thickness above that:
    chassis_top_face_z = (chassis_lift
                          + HP.CHASSIS_GAP
                          + HP.CHASSIS_PLATE_T
                          + HP.CHASSIS_PLATE_T / 2.0)
    arm_parts = ARM_MOD._arm_in_chassis_frame(  # noqa: SLF001
        chassis_top_z=chassis_top_face_z,
    )
    return arm_parts


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-arm",
        action="store_true",
        default=os.environ.get("HEXAPOD_PROTOTYPE_WITH_ARM") == "1",
        help=(
            "Place the OPTIONAL 5-DOF arm on top of the chassis in the "
            "assembly preview (in addition to the standard 6-leg body). "
            "Off by default; respects HEXAPOD_PROTOTYPE_WITH_ARM=1 in "
            "the environment so build_all.py can propagate the flag."
        ),
    )
    # Tolerate being invoked without argv (e.g. via importlib.import_module
    # in build_all.py, which then calls .main()) by defaulting argv to []
    # so argparse doesn't read sys.argv twice.
    if argv is None:
        argv = []
    args, _ = parser.parse_known_args(argv)

    print("Building full hexapod walker PROTOTYPE assembly ...")
    if args.with_arm:
        print("  --with-arm: including optional arm in neutral pose.")

    probe_frame, probe_motor, probe_soft = _build_leg(0)
    probe_meshes = probe_frame + probe_motor + probe_soft
    z_min = min(float(m.bounds[0][2]) for m in probe_meshes)
    chassis_lift = -z_min
    print(f"  chassis_lift computed as {chassis_lift:.1f} mm "
          f"(crab-spike tip bottom at z = {z_min:.1f})")

    frame_meshes  = []
    motor_meshes  = []
    soft_meshes   = []
    battery_meshes = []

    frame_meshes.extend(_body_frame_parts(chassis_lift))
    battery_meshes.extend(_body_battery_parts(chassis_lift))
    soft_meshes.extend(_body_soft_parts(chassis_lift))

    for i in range(6):
        f, m, s = _build_leg(i)
        for part in f:
            part.apply_translation([0, 0, chassis_lift])
            frame_meshes.append(part)
        for part in m:
            part.apply_translation([0, 0, chassis_lift])
            motor_meshes.append(part)
        for part in s:
            part.apply_translation([0, 0, chassis_lift])
            soft_meshes.append(part)

    if args.with_arm:
        arm_meshes = _optional_arm_meshes(chassis_lift)
        if arm_meshes:
            frame_meshes.extend(arm_meshes)
            print(f"  arm: added {len(arm_meshes)} arm part(s) to the "
                  f"assembly preview.")

    def cat(name, meshes_list):
        # soft.stl is empty since the crab-spike retired the foot pads;
        # fall back to an empty mesh so the export + envelope print don't
        # choke on a None concatenate result.
        mesh = (trimesh.util.concatenate(meshes_list)
                if meshes_list else trimesh.Trimesh())
        _yup(mesh)
        path = os.path.join(OUT_DIR, name)
        mesh.export(path)
        ext = mesh.extents if mesh.faces.size else np.zeros(3)
        print(f"  wrote prototype_assembly/{name:18s}"
              f" {len(mesh.faces):>6d} faces"
              f"  envelope {ext[0]:5.1f} x {ext[1]:5.1f} x {ext[2]:5.1f} mm")
        return mesh

    frame   = cat("frame.stl",   frame_meshes)
    motors  = cat("motors.stl",  motor_meshes)
    battery = cat("battery.stl", battery_meshes)
    soft    = cat("soft.stl",    soft_meshes)

    full = trimesh.util.concatenate([frame, motors, battery, soft])
    full.export(os.path.join(OUT_DIR, "full.stl"))
    print(f"  wrote prototype_assembly/full.stl              "
          f"{len(full.faces):>6d} faces")

    print()
    print(f"OK -- wrote 5 STL files under prototype_assembly/")
    print(f"   Walker envelope (foot to foot):  {full.extents[0]/10:.1f} cm")
    print(f"   Walker standing height:          {full.extents[1]/10:.1f} cm")
    print(f"   Total triangles:                 {len(full.faces):,}")
    print()
    print("Next: run ./hexapod_walker/prototype/render_prototype.sh to produce a Cycles render.")


if __name__ == "__main__":
    main(sys.argv[1:])
