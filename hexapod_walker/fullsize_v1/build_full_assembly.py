"""Build the FULL assembled hexapod walker (everything, fabricated or not)
and export it as five category STLs that the Blender render script
treats as separate materials.

Imports the part builders from `hexapod_walker.py`, then places:
    - 1 chassis + top deck + saddle post
    - 1 battery box (with two cell stacks visible inside)
    - 1 electronics bay
    - 6 leg sub-assemblies, each with:
        coxa bracket, coxa link, femur, tibia, foot
        + 3 motor housings (yaw, hip-pitch, knee-pitch)
    - 1 stylized seated rider (capsule person, ~1.75 m tall) for scale
    - handlebars for the rider to grip

Outputs (in ./assembly/):
    frame.stl       brushed-aluminum CNC/cast structural parts
    motors.stl      18 dark-anodized harmonic-drive servo housings
    battery.stl     matte-black battery box + 2 visible LFP cell packs
    soft.stl        urethane foot pads + saddle padding + handlebar grips
    rider.stl       stylized seated rider (denim grey)
    full.stl        single-mesh dump of everything (for non-Blender viewers)

All output is rotated to Y-up (the convention used by Blender's STL
importer when "Forward = -Z, Up = Y", and the convention used by Cursor's
default STL viewer).
"""

from __future__ import annotations

import os
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

import hexapod_walker as HW   # imports part builders + constants


OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assembly")
os.makedirs(OUT_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# Local helpers (reusing HW where possible)
# ---------------------------------------------------------------------------

def _cyl(radius: float, height: float, sections: int = HW.CYL_SECTIONS):
    return trimesh.creation.cylinder(radius=radius, height=height,
                                      sections=sections)


def _capsule(radius: float, length: float, sections: int = 24):
    """Capsule along +Z, total length = `length`. Cylinder of length
    `length - 2*radius` capped by two hemispheres."""
    if length < 2 * radius:
        # degenerate -> just a sphere
        return trimesh.creation.icosphere(subdivisions=2, radius=radius)
    cyl_len = length - 2 * radius
    body = _cyl(radius, cyl_len, sections=sections)
    top = trimesh.creation.icosphere(subdivisions=2, radius=radius)
    top.apply_translation([0, 0, cyl_len / 2.0])
    bot = trimesh.creation.icosphere(subdivisions=2, radius=radius)
    bot.apply_translation([0, 0, -cyl_len / 2.0])
    return trimesh.util.concatenate([body, top, bot])


def _box(extents, center=(0.0, 0.0, 0.0)):
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(center)
    return m


# ---------------------------------------------------------------------------
# Motor housing - reusable
# ---------------------------------------------------------------------------

def _motor_with_finned_body() -> trimesh.Trimesh:
    """A more visually-rich motor model than the bare HW.make_motor_housing.

    Local frame:  motor body extends from x = -MOTOR_LENGTH*0.7 to x = 0
    with finned cylinder body + a cable boss.  Output flange spans
    x in [0, MOTOR_LENGTH*0.3] at smaller diameter; output rotates around X.
    """
    body_len  = HW.MOTOR_LENGTH * 0.7
    flange_len = HW.MOTOR_LENGTH * 0.3

    parts = []

    # Main body cylinder
    body = _cyl(HW.MOTOR_OD / 2.0, body_len)
    body.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    body.apply_translation([-body_len / 2.0, 0, 0])
    parts.append(body)

    # Cooling fins (12 axial fins, 8 mm tall, 4 mm thick)
    for i in range(12):
        a = i * 2 * np.pi / 12
        fin = _box((body_len * 0.95, 4.0, 16.0))
        fin.apply_translation([0, 0, HW.MOTOR_OD / 2.0 - 4.0])
        fin.apply_transform(rotation_matrix(a, [1, 0, 0]))
        fin.apply_translation([-body_len / 2.0, 0, 0])
        parts.append(fin)

    # Mounting flange ring (between body and output)
    ring = _cyl(HW.MOTOR_OD / 2.0 + 6.0, 12.0)
    ring.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    ring.apply_translation([-6.0, 0, 0])
    parts.append(ring)

    # Output flange (the rotating face)
    out_flange = _cyl(HW.MOTOR_OUTPUT_OD / 2.0, flange_len)
    out_flange.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    out_flange.apply_translation([flange_len / 2.0, 0, 0])
    parts.append(out_flange)

    # Output hub raised face
    hub = _cyl(HW.MOTOR_OUTPUT_OD / 2.0 - 18.0, flange_len + 4.0)
    hub.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    hub.apply_translation([flange_len / 2.0, 0, 0])
    parts.append(hub)

    # Cable boss on the side (-Z)
    boss = _cyl(14.0, 30.0)
    boss.apply_translation([-body_len * 0.3, 0, -HW.MOTOR_OD / 2.0 - 8.0])
    parts.append(boss)

    # Encoder cap on the rear
    cap = _cyl(HW.MOTOR_OD / 2.0 - 10.0, 20.0)
    cap.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    cap.apply_translation([-body_len - 10.0, 0, 0])
    parts.append(cap)

    return trimesh.util.concatenate(parts)


# ---------------------------------------------------------------------------
# Per-leg category meshes
# ---------------------------------------------------------------------------

def _build_leg(leg_index: int):
    """Return (frame_parts, motor_parts, soft_parts) for one leg in chassis frame.

    Uses the same kinematic chain as HW._leg_in_body_frame but splits the
    output by visual category and inserts motor housings at every joint.
    """
    apothem = HW.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a),
                         apothem * np.sin(a),
                         0.0])
    outboard = np.array([np.cos(a), np.sin(a), 0.0])
    z_hat    = np.array([0.0, 0.0, 1.0])

    frame_parts = []
    motor_parts = []
    soft_parts  = []

    # Standing-pose pitch angles
    p  = np.deg2rad(HW.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HW.STANCE_FEMUR_DEG + HW.STANCE_TIBIA_DEG)

    # ----- Geometry constants from the redesigned parts -----
    plate_y_coxa  = HW._plate_y_center(90.0)              # arm width
    plate_y_femur = HW._plate_y_center(HW.FEMUR_TUBE_W)
    plate_y_tibia = HW._plate_y_center(HW.TIBIA_TUBE_W)
    JT = HW.JOINT_PLATE_T

    # The leg's local origin is the coxa link's yaw-output point in
    # world coordinates.
    yaw_output_height = (HW.MOTOR_LENGTH + HW.LINK_END_CAP_T) / 2.0
    coxa_link_origin = edge_mid + (HW.CHASSIS_TUBE / 2.0
                                    + HW.MOTOR_OD / 2.0 + 30.0) * outboard \
        + yaw_output_height * z_hat

    # Common: a helper that takes a mesh in *leg-local* coords (where the
    # leg's local +X = outboard, +Y = tangential, +Z = vertical-up) and
    # transforms it into world coords for this leg.
    R_a = rotation_matrix(a, [0, 0, 1])

    def to_world(mesh):
        mesh.apply_transform(R_a)
        mesh.apply_translation(coxa_link_origin)
        return mesh

    # ------------- Coxa bracket (chassis-fixed, frame) ----------------
    cb = HW.make_coxa_bracket()
    cb.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cb.apply_translation(edge_mid + (HW.CHASSIS_TUBE / 2.0) * outboard)
    frame_parts.append(cb)

    # ------------- Yaw motor (axis +Z, output on top) -----------------
    # Native motor: axis +X, body on -X (length body_len), output on +X
    # (length flange_len).  Rotate so +X -> +Z (i.e. rotate by -pi/2 about Y):
    body_len   = HW.MOTOR_LENGTH * 0.7
    flange_len = HW.MOTOR_LENGTH * 0.3
    yaw_motor = _motor_with_finned_body()
    yaw_motor.apply_transform(rotation_matrix(-np.pi / 2.0, [0, 1, 0]))
    # After rotation: body face at z=-body_len, output face at z=+flange_len.
    # We want output face at z=yaw_output_height in chassis coords -- i.e.
    # in leg-local at z = yaw_output_height as well (since coxa_link_origin
    # already has yaw_output_height in z and we'll add coxa_link_origin
    # afterward; but we want the motor's output to align with the coxa
    # link's bottom hub). Translation so output face goes to z = 0 in
    # leg-local (because leg-local has its origin at the coxa link's yaw
    # output, which is exactly the motor's output face).
    yaw_motor.apply_translation([0, 0, -flange_len])
    yaw_motor.apply_transform(rotation_matrix(a, [0, 0, 1]))
    # Translate to the chassis position (NOT through coxa_link_origin --
    # the yaw motor body's centreline goes through edge_mid + outboard
    # offset, NOT shifted by yaw_output_height in z).
    yaw_motor.apply_translation(edge_mid + (HW.CHASSIS_TUBE / 2.0
                                              + HW.MOTOR_OD / 2.0 + 30.0)
                                  * outboard
                                  + yaw_output_height * z_hat)
    motor_parts.append(yaw_motor)

    # ------------- Coxa link ------------------------------------------
    cl = HW.make_coxa_link()
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(coxa_link_origin)
    frame_parts.append(cl)

    # ------------- Hip-pitch motor (axis along +Y_local) --------------
    # In leg-local: motor body face at (COXA_LENGTH, plate_y_coxa - JT/2, 0),
    # motor extending in -Y for MOTOR_LENGTH, output face at
    # (COXA_LENGTH, plate_y_coxa - JT/2 - MOTOR_LENGTH, 0).
    # Native motor: +X is output direction. Rotate so +X -> -Y_local
    # (rotation by -pi/2 about +Z).
    hip_motor = _motor_with_finned_body()
    hip_motor.apply_transform(rotation_matrix(-np.pi / 2.0, [0, 0, 1]))
    # After rotation: body face (was at -X = -body_len) -> at +Y = +body_len.
    # We want body face at y_local = plate_y_coxa - JT/2.
    # Translate y by  (plate_y_coxa - JT/2) - body_len.
    hip_motor.apply_translation([HW.COXA_LENGTH,
                                  (plate_y_coxa - JT / 2) - body_len,
                                  0])
    to_world(hip_motor)
    motor_parts.append(hip_motor)

    # ------------- Femur (rotates with hip-pitch) ---------------------
    # Hip joint plane (= motor output face) in leg-local:
    #   (COXA_LENGTH, plate_y_coxa - JT/2 - MOTOR_LENGTH, 0)
    hip_joint_origin_local = np.array([HW.COXA_LENGTH,
                                        plate_y_coxa - JT / 2
                                            - HW.MOTOR_LENGTH,
                                        0.0])
    fl = HW.make_femur_link()
    # Pitch about the femur's local +Y (the joint axis).  Y is invariant
    # under Y-axis rotation, so the femur plate Y position stays
    # plate_y_femur after rotation.
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    # Translate so the femur's hip plate centre lands at the joint origin.
    # Femur plate centre in femur local = (0, plate_y_femur, 0).
    fl.apply_translation(hip_joint_origin_local
                          - np.array([0.0, plate_y_femur, 0.0]))
    to_world(fl)
    frame_parts.append(fl)

    # ------------- Knee-pitch motor (axis along +Y_local) -------------
    # Femur knee plate centre in leg-local (after femur translate +
    # rotate about Y by femur_pitch):
    #   = R_y(p) @ (FEMUR_LENGTH, plate_y_femur, 0) + hip_joint_origin_local
    #     - (0, plate_y_femur, 0)
    Ry_p_3 = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_plate_centre_local = Ry_p_3 @ np.array([HW.FEMUR_LENGTH,
                                                   plate_y_femur,
                                                   0.0]) \
        + hip_joint_origin_local \
        - np.array([0.0, plate_y_femur, 0.0])
    # Knee motor body face is on the -Y face of the femur knee plate:
    #   = knee_plate_centre_local + (0, -JT/2, 0)
    knee_motor_body_face_local = knee_plate_centre_local \
        + np.array([0.0, -JT / 2.0, 0.0])
    knee_motor = _motor_with_finned_body()
    knee_motor.apply_transform(rotation_matrix(-np.pi / 2.0, [0, 0, 1]))
    # After rotation, body face is at y=+body_len. Translate so it lands
    # at knee_motor_body_face_local.
    knee_motor.apply_translation(knee_motor_body_face_local
                                  - np.array([0.0, body_len, 0.0]))
    to_world(knee_motor)
    motor_parts.append(knee_motor)

    # ------------- Tibia ----------------------------------------------
    # Knee joint plane (= knee motor output face) in leg-local:
    knee_joint_origin_local = knee_motor_body_face_local \
        + np.array([0.0, -HW.MOTOR_LENGTH, 0.0])
    tl = HW.make_tibia_link()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_origin_local
                          - np.array([0.0, plate_y_tibia, 0.0]))
    to_world(tl)
    frame_parts.append(tl)

    # ------------- Foot at tibia tip -----------------------------------
    Ry_pt_3 = rotation_matrix(pt, [0, 1, 0])[:3, :3]
    tibia_translation_local = knee_joint_origin_local \
        - np.array([0.0, plate_y_tibia, 0.0])
    foot_local = tibia_translation_local \
        + Ry_pt_3 @ np.array([HW.TIBIA_LENGTH, 0.0, 0.0])
    R_a_3 = R_a[:3, :3]
    foot_world = R_a_3 @ foot_local + coxa_link_origin

    foot = HW.make_foot_pad()
    FOOT_PAD_TOP_Z = 14.0 + 18.0 + HW.FOOT_HUB_HEIGHT
    foot.apply_translation([foot_world[0], foot_world[1],
                             foot_world[2] - FOOT_PAD_TOP_Z])
    soft_parts.append(foot)

    return frame_parts, motor_parts, soft_parts


# ---------------------------------------------------------------------------
# Body parts builders (categorised)
# ---------------------------------------------------------------------------

def _body_frame_parts(chassis_lift):
    parts = []

    # Chassis hex frame
    chassis = HW.make_chassis_hex()
    chassis.apply_translation([0, 0, chassis_lift])
    parts.append(chassis)

    # Top deck plate
    deck = HW.make_chassis_top_deck()
    deck.apply_translation([0, 0, chassis_lift + HW.CHASSIS_HEIGHT / 2.0
                              + 6.0])
    parts.append(deck)

    # Saddle post + plate (the metal structure of the saddle mount;
    # actual saddle padding is a separate "soft" part)
    saddle = HW.make_saddle_mount()
    saddle.apply_translation([0, 0, chassis_lift + HW.CHASSIS_HEIGHT / 2.0
                                 + 18.0])
    parts.append(saddle)

    # Handlebars: a 350 mm wide tube on a stub mast forward of the saddle
    bar_z = chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 18.0 \
        + HW.SADDLE_POST_LENGTH * 0.85
    mast = _cyl(20.0, 320.0)
    mast.apply_translation([200.0, 0, bar_z - 320.0 / 2.0 + 60.0])
    parts.append(mast)
    bar = _cyl(18.0, 700.0)
    # Bar lies along Y; cylinder primitive is along Z
    bar.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
    bar.apply_translation([200.0, 0, bar_z + 60.0])
    parts.append(bar)

    # Electronics bay (forward of centre on the deck)
    eb = HW.make_electronics_bay()
    eb.apply_translation([260.0, 0,
                          chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 18.0])
    parts.append(eb)

    return parts


def _body_battery_parts(chassis_lift):
    parts = []

    # Battery enclosure
    bb = HW.make_battery_box()
    bb.apply_translation([-260.0, 0,
                          chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 18.0])
    parts.append(bb)

    # Two visible LiFePO4 cell packs INSIDE the box
    # Each pack ~ 380 × 130 × 180 mm (enough for 16s 50Ah prismatic cells)
    for sx in (-1, 1):
        cell = _box(
            (380.0, 130.0, 180.0),
            center=(-260.0 + sx * 75.0,
                    0.0,
                    chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 18.0
                       + HW.BATTERY_BOX_WALL + 90.0),
        )
        parts.append(cell)

    return parts


def _body_soft_parts(chassis_lift):
    parts = []

    # Saddle pad (sits on top of the saddle plate)
    saddle_top_z = chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 18.0 \
        + HW.SADDLE_POST_LENGTH + HW.SADDLE_PLATE_T
    pad = _box((HW.SADDLE_PLATE_W * 0.95,
                HW.SADDLE_PLATE_D * 0.95,
                40.0),
               center=(0, 0, saddle_top_z + 20.0))
    # Round the front a bit — chamfered front by clipping with a thin box
    parts.append(pad)

    # Saddle backrest
    back = _box((30.0, HW.SADDLE_PLATE_D * 0.9, 280.0),
                center=(-HW.SADDLE_PLATE_W * 0.45,
                        0,
                        saddle_top_z + 20.0 + 140.0))
    parts.append(back)

    # Handlebar grips
    bar_z = chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 18.0 \
        + HW.SADDLE_POST_LENGTH * 0.85 + 60.0
    for sy in (-1, 1):
        grip = _cyl(28.0, 130.0)
        grip.apply_transform(rotation_matrix(np.pi / 2, [1, 0, 0]))
        grip.apply_translation([200.0, sy * 270.0, bar_z])
        parts.append(grip)

    return parts


# ---------------------------------------------------------------------------
# Stylized rider for scale
# ---------------------------------------------------------------------------

def _build_rider(chassis_lift):
    """A simple, blocky 1.75 m tall rider seated on the saddle, hands
    forward on the bars.  Pure visual scale — no clothing detail."""
    saddle_top_z = chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 18.0 \
        + HW.SADDLE_POST_LENGTH + HW.SADDLE_PLATE_T + 40.0  # on top of pad

    parts = []

    # Pelvis / hips (sits on the saddle)
    pelvis = _box((280.0, 200.0, 130.0),
                   center=(0, 0, saddle_top_z + 65.0))
    parts.append(pelvis)

    # Torso (leans slightly forward)
    torso = _capsule(140.0, 600.0)
    # Native +Z; tilt 15° forward (i.e. about Y axis so top goes +X)
    torso.apply_transform(rotation_matrix(np.deg2rad(15.0), [0, 1, 0]))
    # After rotation top goes ~  forward +X by sin(15)*300 = 78
    torso.apply_translation([0, 0, saddle_top_z + 130.0 + 300.0 - 30.0])
    parts.append(torso)

    # Head
    head_z = saddle_top_z + 130.0 + 600.0 + 20.0
    head_x = 0.0 + np.tan(np.deg2rad(15.0)) * 400.0  # neck tilt offset
    head = trimesh.creation.icosphere(subdivisions=3, radius=110.0)
    head.apply_translation([head_x, 0, head_z])
    parts.append(head)

    # Arms (reaching forward to the handlebars at x ≈ 200, y ≈ ±270, z ≈ bar_z)
    bar_x, bar_z_grip = 200.0, chassis_lift + HW.CHASSIS_HEIGHT / 2.0 \
        + 18.0 + HW.SADDLE_POST_LENGTH * 0.85 + 60.0
    shoulder_z = saddle_top_z + 130.0 + 540.0
    for sy in (-1, 1):
        shoulder = np.array([0.0, sy * 200.0, shoulder_z])
        wrist    = np.array([bar_x - 30.0, sy * 270.0, bar_z_grip])
        seg = wrist - shoulder
        L = np.linalg.norm(seg)
        # Capsule along +Z by default; align to seg direction
        arm = _capsule(55.0, L)
        # Rotate +Z to seg
        seg_unit = seg / L
        z_axis = np.array([0, 0, 1])
        rot_axis = np.cross(z_axis, seg_unit)
        rot_norm = np.linalg.norm(rot_axis)
        if rot_norm > 1e-9:
            ang = np.arccos(np.clip(np.dot(z_axis, seg_unit), -1, 1))
            arm.apply_transform(rotation_matrix(ang, rot_axis / rot_norm))
        arm.apply_translation((shoulder + wrist) / 2.0)
        parts.append(arm)

    # Upper legs (thighs) — knees forward, feet on chassis-edge footrests
    foot_rest_x = HW.CHASSIS_FLAT_TO_FLAT / 2.0 - 40.0
    foot_rest_z = chassis_lift + HW.CHASSIS_HEIGHT / 2.0 + 30.0
    # Bring feet a bit inboard so they aren't dangling outside the chassis
    foot_rest_x *= 0.65
    for sy in (-1, 1):
        hip = np.array([0.0, sy * 90.0, saddle_top_z + 65.0])
        knee = np.array([foot_rest_x * 0.6, sy * 130.0,
                         saddle_top_z - 60.0])
        ankle = np.array([foot_rest_x, sy * 180.0, foot_rest_z])

        for a_pt, b_pt, r in [(hip, knee, 70.0), (knee, ankle, 55.0)]:
            seg = b_pt - a_pt
            L = np.linalg.norm(seg)
            limb = _capsule(r, L)
            seg_unit = seg / L
            z_axis = np.array([0, 0, 1])
            rot_axis = np.cross(z_axis, seg_unit)
            rot_norm = np.linalg.norm(rot_axis)
            if rot_norm > 1e-9:
                ang = np.arccos(np.clip(np.dot(z_axis, seg_unit), -1, 1))
                limb.apply_transform(rotation_matrix(ang,
                                                      rot_axis / rot_norm))
            limb.apply_translation((a_pt + b_pt) / 2.0)
            parts.append(limb)

    return trimesh.util.concatenate(parts)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _yup(mesh):
    """Rotate Z-up → Y-up so Blender (with default STL forward=-Z up=Y)
    and Cursor's STL viewer see the walker standing upright."""
    mesh.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    return mesh


def main() -> None:
    print("Building full hexapod walker assembly ...")

    # ---- Compute chassis_lift from the actual leg kinematics ----
    # Build a probe leg, find its lowest Z (the foot pad's bottom face);
    # chassis must lift the same amount so that point is on z=0.
    probe_frame, probe_motor, probe_soft = _build_leg(0)
    probe_meshes = probe_frame + probe_motor + probe_soft
    z_min = min(float(m.bounds[0][2]) for m in probe_meshes)
    chassis_lift = -z_min
    print(f"  chassis_lift computed as {chassis_lift:.0f} mm "
          f"(foot pad bottom at z = {z_min:.0f})")

    frame_meshes  = []
    motor_meshes  = []
    soft_meshes   = []
    battery_meshes = []

    # Body frame
    body = _body_frame_parts(chassis_lift)
    frame_meshes.extend(body)

    # Battery (own category)
    battery_meshes.extend(_body_battery_parts(chassis_lift))

    # Soft body parts
    soft_meshes.extend(_body_soft_parts(chassis_lift))

    # Six legs.  Each leg's parts already have their world XYZ baked in
    # via _build_leg; we just lift everything by chassis_lift so the
    # foot pads land on z = 0.
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

    # Rider
    rider = _build_rider(chassis_lift)

    # ---- export each category ----
    def cat(name, meshes_list):
        mesh = trimesh.util.concatenate(meshes_list) if isinstance(
            meshes_list, list) else meshes_list
        _yup(mesh)
        path = os.path.join(OUT_DIR, name)
        mesh.export(path)
        print(f"  wrote assembly/{name:20s} {len(mesh.faces):>7d} faces"
              f"  envelope {mesh.extents[0]:6.0f} x "
              f"{mesh.extents[1]:6.0f} x {mesh.extents[2]:6.0f} mm")
        return mesh

    frame   = cat("frame.stl",   frame_meshes)
    motors  = cat("motors.stl",  motor_meshes)
    battery = cat("battery.stl", battery_meshes)
    soft    = cat("soft.stl",    soft_meshes)
    rider_m = cat("rider.stl",   rider)

    # Combined "everything" STL (handy for non-Blender viewers)
    full = trimesh.util.concatenate([frame, motors, battery, soft, rider_m])
    full.export(os.path.join(OUT_DIR, "full.stl"))
    print(f"  wrote assembly/full.stl              {len(full.faces):>7d} faces")

    # Summary
    print()
    print(f"OK -- wrote 6 STL files under assembly/")
    print(f"   Walker envelope (foot to foot):  {full.extents[0]/1000:.2f} m")
    print(f"   Walker standing height (Y-up):   {full.extents[1]/1000:.2f} m")
    print(f"   Total triangles:                 {len(full.faces):,}")
    print()
    print("Next: run ./render_blender.sh to produce a Cycles render.")


if __name__ == "__main__":
    main()
