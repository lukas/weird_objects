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
        + 3 servo horn adapters

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

import os
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

    # Cable bundle on -X face
    bundle = _cyl(2.2, 18.0)
    bundle.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    bundle.apply_translation([-HP.SERVO_BODY_W / 2.0 - 9.0,
                               0, HP.SERVO_BODY_H * 0.55])
    parts.append(bundle)

    return trimesh.util.concatenate(parts)


def _horn_visual() -> trimesh.Trimesh:
    """A simple star-shaped servo horn (NOT the printed adapter; the
    plastic horn that ships with the servo).  Used 18 x to dress the
    output of every joint."""
    parts = []
    base = _cyl(8.0, 2.0)
    base.apply_translation([0, 0, 1.0])
    parts.append(base)

    # 4 arms
    for i in range(4):
        ang = i * np.pi / 2
        arm = _box((20.0, 4.0, 1.6))
        arm.apply_transform(rotation_matrix(ang, [0, 0, 1]))
        arm.apply_translation([0, 0, 0.8])
        parts.append(arm)

    return trimesh.util.concatenate(parts)


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

    # Yaw output reference height (above the chassis edge plane).  Same
    # constant used in HP._leg_in_body_frame.
    yaw_output_z = HP.SERVO_OUTPUT_H + HP.HORN_ADAPTER_T
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    arm_t = 4.0
    WALL = 2.5
    cradle_d = HP.SERVO_BODY_D + 2 * WALL
    hip_drop = -(cradle_d / 2.0 + arm_t / 2.0)
    hip_joint_local = np.array([HP.COXA_LENGTH, 0.0, hip_drop])
    Ry_p_3 = rotation_matrix(p, [0, 1, 0])[:3, :3]
    # Femur's knee-end joint axis is on the spar centreline at z=0
    # in femur local, so we don't add a 'drop' offset for the knee.
    knee_joint_local = hip_joint_local + Ry_p_3 @ np.array([HP.FEMUR_LENGTH,
                                                             0.0, 0.0])

    def to_world(mesh):
        mesh.apply_transform(R_a)
        mesh.apply_translation(yaw_output_world)
        return mesh

    # ------------- Coxa bracket (chassis-fixed, frame) -----------------
    cb = HP.make_coxa_bracket()
    cb.apply_transform(R_a)
    cb.apply_translation(edge_mid)
    frame_parts.append(cb)

    # ------------- Yaw servo (output up) ------------------------------
    # Servo body sits below the chassis with output gear poking up.
    # In bracket-local: body bottom at z = -SERVO_BODY_H, output at z=0.
    # The bracket itself is in leg-local coords with edge_mid at z=0.
    yaw_servo = _hobby_servo_visual()
    # Shift so its body bottom is at z = -SERVO_BODY_H in bracket coords
    # and its output offset (along bracket +X = leg +X) lines up with
    # x=0 (the yaw axis):
    # _hobby_servo_visual has body centre at (0, 0, BODY_H/2) and
    # output gear at (SERVO_OUTPUT_X, 0, ...).  We want output gear at
    # (0, 0, 0) -> translate by (-SERVO_OUTPUT_X, 0, -SERVO_BODY_H).
    yaw_servo.apply_translation([-HP.SERVO_OUTPUT_X, 0, -HP.SERVO_BODY_H])
    yaw_servo.apply_transform(R_a)
    yaw_servo.apply_translation(edge_mid)
    motor_parts.append(yaw_servo)

    # Yaw horn (sits on top of the output gear, drives the coxa link)
    yaw_horn = _horn_visual()
    yaw_horn.apply_translation([0, 0, HP.SERVO_OUTPUT_H + 4.0])
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
    # After R_hip: servo (X, Y, Z) -> leg (X, Z, -Y).  Servo's output
    # point (SERVO_OUTPUT_X, 0, SERVO_BODY_H) -> leg-local
    # (SERVO_OUTPUT_X, SERVO_BODY_H, 0).
    # Translate so that point lands at (COXA_LENGTH, 0, hip_drop):
    delta = np.array([HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
                       -HP.SERVO_BODY_H, hip_drop])
    hip_servo.apply_translation(delta)
    to_world(hip_servo)
    motor_parts.append(hip_servo)

    # Hip horn -- bolted to the servo output, sticking out into +Y
    # from the cradle face.  The femur's hip-pad clamps onto this.
    hip_horn = _horn_visual()
    R_hip_horn = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    hip_horn.apply_transform(R_hip_horn)
    hip_horn.apply_translation([HP.COXA_LENGTH, 0.5, hip_drop])
    to_world(hip_horn)
    motor_parts.append(hip_horn)

    # ------------- Femur (rotates with hip-pitch) ----------------------
    fl = HP.make_femur_link()
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local)
    to_world(fl)
    frame_parts.append(fl)

    # ------------- Knee-pitch servo (output along +Y_local) ------------
    # Lives at the femur's far end.  In femur-local coords its output
    # is at (FEMUR_LENGTH, 0, 0) -- on the spar centreline -- with
    # output along +Y_local.  Pre-rotated by femur pitch.
    knee_servo = _hobby_servo_visual()
    knee_servo.apply_transform(R_hip)
    delta = np.array([HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
                       -HP.SERVO_BODY_H, 0])
    knee_servo.apply_translation(delta)
    knee_servo.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_servo.apply_translation(hip_joint_local)
    to_world(knee_servo)
    motor_parts.append(knee_servo)

    knee_horn = _horn_visual()
    knee_horn.apply_transform(R_hip_horn)
    knee_horn.apply_translation([HP.FEMUR_LENGTH, 0.5, 0])
    knee_horn.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_horn.apply_translation(hip_joint_local)
    to_world(knee_horn)
    motor_parts.append(knee_horn)

    # ------------- Tibia (rotates with knee-pitch) ---------------------
    tl = HP.make_tibia_link()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local)
    to_world(tl)
    frame_parts.append(tl)

    # ------------- Foot at tibia tip ----------------------------------
    Ry_pt_3 = rotation_matrix(pt, [0, 1, 0])[:3, :3]
    foot_local = knee_joint_local + Ry_pt_3 @ np.array([HP.TIBIA_LENGTH,
                                                         0.0, 0.0])
    foot_world = R_a_3 @ foot_local + yaw_output_world

    foot = HP.make_foot_pad()
    FOOT_TOP_Z = 4.0 + 6.0 + HP.FOOT_HUB_HEIGHT
    foot.apply_translation([foot_world[0], foot_world[1],
                             foot_world[2] - FOOT_TOP_Z])
    soft_parts.append(foot)

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

    # 4 stand-off posts between the plates, where the centre-hole
    # bolt pattern lives
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        post = _cyl(2.5, HP.CHASSIS_GAP)
        post.apply_translation([35.0 * np.cos(a), 35.0 * np.sin(a),
                                  chassis_lift + HP.CHASSIS_PLATE_T
                                      + HP.CHASSIS_GAP / 2.0])
        parts.append(post)

    return parts


def _body_battery_parts(chassis_lift):
    """Battery holder, LiPo pack, electronics tray, Arduino, PCA9685."""
    parts = []
    bh_z0 = chassis_lift + HP.CHASSIS_PLATE_T

    # Holder shell
    bh = HP.make_battery_holder()
    bh.apply_translation([-25.0, 0, bh_z0])
    parts.append(bh)

    # The LiPo pack itself (visible inside the holder).  Standard 3S
    # 2200 mAh pack: ~ 105 x 35 x 25 mm with shrink-wrap label.
    lipo = _box((105.0, 35.0, 25.0),
                center=(-25.0, 0,
                         bh_z0 + HP.BATTERY_WALL + 25.0 / 2.0))
    parts.append(lipo)

    # Electronics tray + Arduino Nano + PCA9685
    et = HP.make_electronics_tray()
    et.apply_translation([35.0, 0, bh_z0 + 1.0])
    parts.append(et)

    # Arduino Nano (43 x 18 x 4 mm)
    arduino = _box((43.0, 18.0, 6.0),
                   center=(35.0 - 30.0, 0,
                            bh_z0 + 1.0 + HP.ELEC_TRAY_T + 5.0))
    parts.append(arduino)

    # PCA9685 PWM driver (62 x 25 x 8 mm)
    pca = _box((62.0, 25.0, 8.0),
               center=(35.0 + 25.0, 0,
                        bh_z0 + 1.0 + HP.ELEC_TRAY_T + 6.0))
    parts.append(pca)

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


def main() -> None:
    print("Building full hexapod walker PROTOTYPE assembly ...")

    # ---- Compute chassis_lift from the actual leg kinematics ----
    probe_frame, probe_motor, probe_soft = _build_leg(0)
    probe_meshes = probe_frame + probe_motor + probe_soft
    z_min = min(float(m.bounds[0][2]) for m in probe_meshes)
    chassis_lift = -z_min
    print(f"  chassis_lift computed as {chassis_lift:.1f} mm "
          f"(foot pad bottom at z = {z_min:.1f})")

    frame_meshes  = []
    motor_meshes  = []
    soft_meshes   = []
    battery_meshes = []

    # Body
    frame_meshes.extend(_body_frame_parts(chassis_lift))
    battery_meshes.extend(_body_battery_parts(chassis_lift))
    soft_meshes.extend(_body_soft_parts(chassis_lift))

    # 6 legs
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

    def cat(name, meshes_list):
        mesh = trimesh.util.concatenate(meshes_list)
        _yup(mesh)
        path = os.path.join(OUT_DIR, name)
        mesh.export(path)
        print(f"  wrote prototype_assembly/{name:18s}"
              f" {len(mesh.faces):>6d} faces"
              f"  envelope {mesh.extents[0]:5.1f} x "
              f"{mesh.extents[1]:5.1f} x {mesh.extents[2]:5.1f} mm")
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
    print("Next: run ./render_prototype.sh to produce a Cycles render.")


if __name__ == "__main__":
    main()
