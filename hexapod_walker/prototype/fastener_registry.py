"""Explicit, world-space registry of every fastener in the hexapod prototype.

This module is the SINGLE SOURCE OF TRUTH for fastener placement.  Both
the build inspector (``inspect_build.py``) and the verifier
(``_verify_prototype.check_screwdriver_access``) read the same
``build_all_fastener_instances()`` list so a fastener never silently
drifts between the two tools.

Every entry mirrors one bolt-hole / nut-pocket cut that
``hexapod_prototype.py`` makes into a printed part.  The transforms
follow the SAME chain that ``build_prototype_assembly._build_leg`` /
``inspect_build._build_assembly_instances`` use to place each part in
the chassis frame, so adding ``chassis_lift`` to every fastener gives
the world pose used by the build inspector.

Frame
-----

The returned positions and axes live in the **pre-chassis-lift chassis
frame** (z = 0 is the chassis_bottom plate top face).  Callers that
need the lifted / Y-up render frame should apply the same lift /
rotation they apply to every other part.

Convention
----------

* ``head_world_xyz``: the centre of the fastener's HEAD outboard face --
  for an M3 SHCS this is the under-side of the cap (the face that
  bears on the part); for an M3 nyloc nut this is the FACE THAT FACES
  OUTWARD from the part (i.e. the visible outer face of the captive
  nut sitting in its hex pocket).
* ``axis_world``: a unit 3-vector pointing FROM the head INTO the
  material -- the direction you'd push the screw to drive it in.
  For a nut sat outboard of a wall, ``axis_world`` points from the
  outer (visible) nut face INTO the wall.

Enumerated categories (Design B + Design C, May 2026 revert)
------------------------------------------------------------

1. ``72 x M3 x 8 SHCS`` -- cradle servo-mount bolts (Design C revert).
   4 per cradle x 3 cradles (yaw / hip-pitch / knee) per leg x 6 legs
   = 72.  Driven VERTICALLY from above through each servo ear into a
   Phi SHCS_PILOT_OD = 2.5 mm self-tap pilot in the printed shelf
   below.  Shares the SAME stock (PN_M3X8_SHCS / 91290A113) as the
   link-to-X-horn bolts below, so the prototype now uses a SINGLE
   M3 SHCS length (M3 x 8) for both cradle and horn-clamp bolts.
   The brief May 2026 horizontal-nyloc iteration was retired after
   the audit surfaced (a) a wire-channel collision in the +X wall
   and (b) a >MIN_PRINT_T outer-wall violation around the Phi 5.6 mm
   hex pocket; see ``PROTOTYPE.md`` (Design C section) for the audit
   table and the revert rationale.

2. ``72 x M3 x 8 SHCS`` -- link-to-X-horn bolts.  4 per joint
   (HORN_BOLT_PCD = 20.8 mm circle) x (yaw + hip + knee) = 3 joints
   per leg x 6 legs.  Threads downward from the printed link's pad
   face into the plastic 4-arm X-horn that ships with the servo
   (Design B retired the printed adapter disc; the link clamps the
   X-horn directly).  Combined with the cradle bolts above this gives
   144 x M3 x 8 SHCS total.

3. ``18 x M2.5 x 8 spline center screw`` -- ships with the servo; sits
   captive between the servo spline and the plastic horn.  18 servos
   x 1 screw each = 18.  Special-cased in ``check_screwdriver_access``
   with a SKIP because the screw is hidden under the X-horn during
   normal assembly -- install before fitting the horn.

4. ``24 x M3 x 32 SHCS`` -- coxa-bracket-to-chassis bolts.  4 chassis
   bolts per coxa_bracket x 6 brackets = 24.  Threads down through the
   chassis_bottom plate, the bracket flange, into an M3 nyloc nut
   captured below the chassis plate.

5. ``24 x M3 nyloc nuts`` -- one per chassis-bracket bolt (see 4.).

6. ``6 x M3 x 16 pan-head bolts`` -- foot hinge pins.  1 per leg.

7. ``6 x M3 nyloc nuts`` -- one per foot hinge bolt.

Categories NOT enumerated yet (acknowledged future work; see
PROTOTYPE_BOM.md "Fasteners" auto-derived section):

* Electronics-tray board-mount screws (4 Arduino-Nano + 4 PCA9685
  standoffs through the printed tray).  These are small commodity
  M2 / M3 nylon hardware that ships with the boards.
* Battery-holder foot bolts (4 M3 through the printed holder's feet
  into the chassis_bottom plate).
* Chassis-stack standoff bolts (4 M3 through the chassis_top + chassis_bottom
  plates + the brass standoff column).  Hidden inside the chassis
  bay and not user-serviceable.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import hexapod_prototype as HP  # noqa: E402


# ---------------------------------------------------------------------------
# McMaster-Carr part numbers (verify with fasteners/README.md)
# ---------------------------------------------------------------------------
# Centralised so the BOM, the inspector legend, and the fasteners/
# README stay in sync.  If McMaster renumbers a SKU, edit here only.

PN_M3X8_SHCS     = "91290A113"   # M3 x 8  socket-head cap screw, black-oxide steel
PN_M3X32_SHCS    = "91290A123"   # M3 x 30 socket-head cap screw (closest stock to 32 mm)
PN_M3_NYLOC      = "90576A102"   # M3 nylon-insert lock nut, A2 stainless
PN_M3X16_PAN     = "92010A130"   # M3 x 16 pan-head Phillips, A2 stainless (foot hinge)
PN_M25X8_SHCS    = "91290A104"   # M2.5 x 8 socket-head cap screw (servo spline)
# May 2026 revert: the brief Design C horizontal-nyloc cradle bolt
# (PN_M3X14_SHCS = 91290A115) was retired in favour of vertical M3 x 8
# self-tap SHCS into Phi 2.5 mm printed pilots, reusing the existing
# M3 x 8 stock used for the link-to-X-horn bolts.

# Human-readable spec labels (used by the inspector and the BOM script).
SPEC_M3X8_SHCS   = "M3x8 SHCS"
SPEC_M3X32_SHCS  = "M3x32 SHCS"
SPEC_M3_NYLOC    = "M3 nyloc nut"
SPEC_M3X16_PAN   = "M3x16 pan-head"
SPEC_M25X8_SHCS  = "M2.5x8 spline screw"


# ---------------------------------------------------------------------------
# Public dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FastenerInstance:
    """One physical fastener placed in the assembled robot."""
    part_number: str          # e.g. "91290A115"
    spec: str                 # e.g. "M3x14 SHCS"
    head_world_xyz: np.ndarray  # 3-vector, mm (pre-chassis-lift chassis frame)
    axis_world: np.ndarray      # unit vector, FROM head INTO material
    role: str                 # e.g. "coxa_link L0 hip cradle -X top SHCS"
    leg_index: int | None = None
    joint: str | None = None       # 'yaw' / 'hip' / 'knee' (None for chassis/foot)
    length_mm: float | None = None  # bolt length (omit for nuts)
    cache_stl: str = ""             # filename in fasteners/ (filled by builder)
    # Non-None means the screwdriver-access check should SKIP this
    # fastener and report the given reason instead of probing geometry.
    # Use sparingly -- only for fasteners that are physically
    # impossible to driver-access AFTER assembly but are known to be
    # installable BEFORE the obstructing part is fitted (e.g. the
    # servo's spline center screw, or a cradle's captive nyloc nut
    # that is hand-immobilised by the printed hex pocket and never
    # needs a wrench).
    skip_screwdriver_reason: str | None = None

    def __post_init__(self):
        # Normalise the axis to a unit vector so callers can rely on it.
        ax = np.asarray(self.axis_world, dtype=float)
        n = float(np.linalg.norm(ax))
        if n > 1e-9 and abs(n - 1.0) > 1e-6:
            object.__setattr__(self, "axis_world", ax / n)


# ---------------------------------------------------------------------------
# Transform helpers (4x4 matrices)
# ---------------------------------------------------------------------------


def _T(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    m = np.eye(4)
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def _Rx(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1],
    ])


def _Ry(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1],
    ])


def _Rz(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])


def _apply_point(T: np.ndarray, p_local) -> np.ndarray:
    p = np.asarray(p_local, dtype=float)
    return (T[:3, :3] @ p) + T[:3, 3]


def _apply_dir(T: np.ndarray, v_local) -> np.ndarray:
    v = np.asarray(v_local, dtype=float)
    out = T[:3, :3] @ v
    n = float(np.linalg.norm(out))
    return out / n if n > 0 else out


# ---------------------------------------------------------------------------
# Per-cradle fastener generators
# ---------------------------------------------------------------------------
#
# Each cradle uses a ``_servo_well_solid``-frame bolt pattern: 4 bolts at
# well-local
#     (sx * SERVO_MOUNT_HOLE_X_OFFSET,
#      sy * SERVO_MOUNT_HOLE_Y_OFFSET,
#      shelf_top_z + SERVO_TAB_T/2)         -- = ear top
# with ``sx, sy in {-1, +1}``.  The bolt head sits ON TOP of the servo
# ear; the bolt axis is straight DOWN (-Z in well-local), threading into
# a Phi SHCS_PILOT_OD = 2.5 mm vertical self-tap pilot in the printed
# shelf below the ear.
#
# ``shelf_top_z`` is normally ``WELL_RIM_Z`` (coxa_link / femur cradles
# whose rim is intact).  The coxa_bracket's drop-in slot eats wall
# material above ``bracket-z = BRACKET_SLOT_Z_MIN_RIB_CLEAR`` so the
# bracket's effective shelf top sits ``BRACKET_SHELF_DROP_MM`` (= 3 mm)
# BELOW WELL_RIM_Z; the yaw cradle case uses that lower value so the
# bolt heads sit on the actual cut wall rather than 3 mm of empty air.
#
# To enumerate world-frame positions we compose the cradle's well-to-
# world 4x4 transform once, then map the 4 well-local positions through
# it.


# Bracket shelf drop -- duplicates ``BRACKET_SHELF_DROP_MM`` defined
# locally inside ``hexapod_prototype.make_coxa_bracket``.  Both values
# come from the same root cause: the bracket's drop-in slot cuts wall
# material from bracket-z = -3 down to the flange top, eating 3 mm
# off the bolt-site rim.  Keep them in sync.
_BRACKET_SHELF_DROP_MM = 3.0


def _well_bolt_local_pos_axis(shelf_top_z: float):
    """Return ``(positions, axes, signs)`` for the 4 cradle bolts at
    the given shelf top z (in well-local coords).

    Bolt head sits on the servo ear's TOP face, which is at
    ``shelf_top_z + SERVO_TAB_T`` (the tab thickness is SERVO_TAB_T and
    the tab BOTTOM rests on the shelf top; ear top = shelf_top +
    tab_t).
    """
    ear_top_z = shelf_top_z + HP.SERVO_TAB_T
    positions = [
        np.array([
            sx * HP.SERVO_MOUNT_HOLE_X_OFFSET,
            sy * HP.SERVO_MOUNT_HOLE_Y_OFFSET,
            ear_top_z,
        ])
        for sx in (-1, +1)
        for sy in (-1, +1)
    ]
    # Axis points FROM head INTO the material (downward into the
    # printed shelf), so well-local -Z.
    axes = [
        np.array([0.0, 0.0, -1.0])
        for _ in positions
    ]
    signs = [
        (sx, sy)
        for sx in (-1, +1)
        for sy in (-1, +1)
    ]
    return positions, axes, signs


def _wall_corner_label(sx: int, sy: int) -> str:
    x_face = "+X" if sx > 0 else "-X"
    y_corner = "top" if sy > 0 else "bot"
    return f"{x_face} {y_corner}"


def _yaw_cradle_T(leg_index: int) -> np.ndarray:
    """World 4x4 transform that maps the yaw cradle's well-local frame
    into the chassis frame (pre-lift)."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    # In make_coxa_bracket: well_dz = -WELL_RIM_Z, and the well is
    # translated by (-SERVO_OUTPUT_X, 0, well_dz) in bracket-local
    # coords; the bracket itself sits with its origin at edge_mid
    # rotated by ``a`` about Z.
    T = _T(*edge_mid) @ _Rz(a) @ _T(-HP.SERVO_OUTPUT_X, 0.0, -HP.WELL_RIM_Z)
    return T


def _hip_cradle_T(leg_index: int) -> np.ndarray:
    """World 4x4 transform for the hip-pitch cradle (lives in the coxa_link)."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + HP.HORN_STACK_H
    )
    # The well's transformation INSIDE make_coxa_link:
    #   rotation: R = rotation_matrix(-pi/2, [1, 0, 0])  (well +Z -> link +Y)
    #   translation: delta = (COXA_LENGTH - SERVO_OUTPUT_X,
    #                         -(SERVO_BODY_H + SERVO_OUTPUT_H), 0)
    #   plus an extra -Z shift well_z_drop, then a +Z shift by COXA_LIFT.
    # The link itself is rotated by ``a`` about Z and translated to
    # ``edge_mid + (0, 0, yaw_output_z)`` in the chassis frame.
    arm_t = HP.COXA_ARM_T
    well_z_drop = -(HP.WELL_D / 2.0 + arm_t / 2.0 + HP.WELL_Z_DROP_EXTRA)
    delta = np.array([
        HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
        -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
        0.0,
    ])
    # link-local transform: T_link_local = T(0,0,COXA_LIFT) @ T(0,0,well_z_drop) @ T(delta) @ R
    T_link_local = (
        _T(0.0, 0.0, HP.COXA_LIFT)
        @ _T(0.0, 0.0, well_z_drop)
        @ _T(*delta)
        @ _Rx(-np.pi / 2.0)
    )
    T = _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a) @ T_link_local
    return T


def _knee_cradle_T(leg_index: int) -> np.ndarray:
    """World 4x4 transform for the knee cradle (lives in the femur_link)."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + HP.HORN_STACK_H
    )
    hip_drop = HP.COXA_HIP_DROP
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    # The knee well lives in the FEMUR's local frame, translated by
    # delta_knee = (FEMUR_LENGTH - SERVO_OUTPUT_X,
    #               -(SERVO_BODY_H + SERVO_OUTPUT_H), 0)
    # and rotated by R_hip = R_x(-pi/2).  The femur itself is rotated
    # by R_y(p_femur) about the hip joint and then placed in the
    # coxa-link/yaw-output frame.
    delta_knee = np.array([
        HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
        -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
        0.0,
    ])
    T_femur_local = _T(*delta_knee) @ _Rx(-np.pi / 2.0)
    T_femur_in_link = _T(HP.COXA_LENGTH, 0.0, hip_drop) @ _Ry(p)
    T = (
        _T(*edge_mid)
        @ _T(0.0, 0.0, yaw_output_z)
        @ _Rz(a)
        @ T_femur_in_link
        @ T_femur_local
    )
    return T


def _emit_cradle_fasteners(
    *,
    T_well_to_world: np.ndarray,
    leg_index: int,
    joint: str,
    location: str,
    shelf_top_z: float = None,
) -> list[FastenerInstance]:
    """Emit the 4 vertical M3 self-tap SHCS for one cradle.

    ``shelf_top_z`` is the well-local z of the shelf surface the servo
    ear rests on -- ``WELL_RIM_Z`` for the femur / coxa_link cradles
    and ``WELL_RIM_Z - BRACKET_SHELF_DROP_MM`` for the coxa_bracket
    yaw cradle whose rim was eaten by the drop-in slot.
    """
    if shelf_top_z is None:
        shelf_top_z = HP.WELL_RIM_Z

    positions, axes, signs = _well_bolt_local_pos_axis(shelf_top_z)

    out: list[FastenerInstance] = []
    for (sx, sy), p_local, ax_local in zip(signs, positions, axes):
        head = _apply_point(T_well_to_world, p_local)
        axis = _apply_dir(T_well_to_world, ax_local)
        role_suffix = _wall_corner_label(sx, sy)
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=f"{location} {role_suffix} self-tap SHCS",
            leg_index=leg_index,
            joint=joint,
            length_mm=8.0,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
            # The new vertical bolt head sits ON TOP of the servo ear
            # at well-local z = shelf_top + SERVO_TAB_T with the +Z
            # hemisphere open to clear air at the SERVO LEVEL.  But
            # the cradle bolts are still CAPTIVE SUB-ASSEMBLY
            # fasteners -- they are tightened BEFORE the joint above
            # the cradle is closed (the coxa_link onto the yaw servo
            # horn, the femur onto the hip servo horn, the tibia onto
            # the knee servo horn, and the chassis_top stack onto the
            # bracket's chassis_bottom plate).  Once that next-stage
            # link / plate is in place, the driver cone going +Z from
            # a yaw cradle bolt hits chassis_top / electronics_tray
            # above the bracket, and a hip / knee cradle bolt's cone
            # is masked by the femur / tibia pad that bolts onto the
            # servo horn directly above the ear.  The verifier's
            # screwdriver-access check probes the FULLY-ASSEMBLED
            # robot so we SKIP these cradle bolts with the standard
            # sub-assembly-order rationale.
            skip_screwdriver_reason=(
                "captive sub-assembly fastener: the vertical M3 self-"
                "tap is torqued from above BEFORE the next stage's "
                "link / plate is bolted onto the cradle's servo horn "
                "(yaw cradle -> coxa_link, hip cradle -> femur, knee "
                "cradle -> tibia; bracket yaw cradle also requires "
                "the chassis-top stack to be added LAST).  See "
                "PROTOTYPE.md for the explicit sub-assembly order"
            ),
        ))
    return out


# ---------------------------------------------------------------------------
# Per-joint horn-bolt fastener generators
# ---------------------------------------------------------------------------
#
# Each rotary joint (yaw, hip-pitch, knee-pitch) clamps the printed link's
# pad onto the plastic 4-arm X-horn via 4 x M3 SHCS on HORN_BOLT_PCD =
# 20.8 mm.  Design B (May 2026) retired the printed adapter disc, so the
# bolts thread DIRECTLY from the link's pad into the X-horn that ships
# with the servo.  Bolt length = pad thickness + ~3 mm into the horn arm.
# Pad thicknesses:
#   * coxa_link hub:    COXA_LIFT + hub_t = 36 + 8 mm of pad/pedestal
#   * femur hip pad:    LINK_THICKNESS = 6 mm
#   * tibia knee pad:   LINK_THICKNESS = 6 mm
# The coxa_link hub is much taller; spec'd M3 x ~20-25 on a SHOPPING_LIST
# update.  For the link pads (femur/tibia) M3 x 8 is the right length.
# The user spec'd "M3 x 8 SHCS" total for all link-to-horn bolts, which
# implies the coxa_link's pedestal pass-through bolt is a SEPARATE bolt
# from the spec's 72 count.  We honour the user's enumeration and
# treat all 72 link-to-horn bolts as M3 x 8 (acknowledging the
# coxa_link hub bolt would in reality be M3 x 20-ish; document this
# in fasteners/README.md).

_HORN_BOLT_PCD_HALF = HP.HORN_BOLT_PCD / 2.0


def _emit_horn_fasteners_yaw(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-X-horn bolts at the yaw joint (coxa_link hub)."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + HP.HORN_STACK_H
    )
    # Link's coordinate frame: origin at the pad's BOTTOM face, +Z up.
    # The pad's TOP face (where the SHCS head sits) is at link-local
    # z = COXA_LIFT + hub_t.  We just need the top face; the 4 bolts
    # are at (PCD/2 cos(ang), PCD/2 sin(ang), top_z) and the axis
    # points -Z (into the material, toward the X-horn below).
    arm_t = HP.COXA_ARM_T
    hub_t = arm_t + 2.0
    top_z = HP.COXA_LIFT + hub_t
    T_link_to_world = _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a)
    out: list[FastenerInstance] = []
    for ang in HP.HORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            _HORN_BOLT_PCD_HALF * np.sin(ang),
            top_z,
        ])
        head = _apply_point(T_link_to_world, p_local)
        axis = _apply_dir(T_link_to_world, np.array([0.0, 0.0, -1.0]))
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"coxa_link L{leg_index} hub-to-X-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="yaw",
            length_mm=8.0,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
        ))
    return out


def _emit_horn_fasteners_hip(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-X-horn bolts at the hip-pitch joint (femur hip pad)."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + HP.HORN_STACK_H
    )
    hip_drop = HP.COXA_HIP_DROP
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    # Femur-local: hip pad at y in [HORN_STACK_H, HORN_STACK_H +
    # LINK_THICKNESS] = [5, 11].  Bolt heads sit on the +Y face of the
    # pad (y = 11), axis = -Y (into the material).
    pad_top_y = HP.HORN_STACK_H + HP.LINK_THICKNESS
    T_femur_in_link = _T(HP.COXA_LENGTH, 0.0, hip_drop) @ _Ry(p)
    T_femur_to_world = (
        _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a) @ T_femur_in_link
    )
    out: list[FastenerInstance] = []
    for ang in HP.HORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            pad_top_y,
            _HORN_BOLT_PCD_HALF * np.sin(ang),
        ])
        head = _apply_point(T_femur_to_world, p_local)
        axis = _apply_dir(T_femur_to_world, np.array([0.0, -1.0, 0.0]))
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"femur_link L{leg_index} hip-pad-to-X-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="hip",
            length_mm=8.0,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
        ))
    return out


def _emit_horn_fasteners_knee(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-X-horn bolts at the knee joint (tibia knee pad)."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + HP.HORN_STACK_H
    )
    hip_drop = HP.COXA_HIP_DROP
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    Ry_p_3 = _Ry(p)[:3, :3]
    knee_joint_local = np.array([HP.COXA_LENGTH, 0.0, hip_drop]) + Ry_p_3 @ np.array(
        [HP.FEMUR_LENGTH, 0.0, 0.0]
    )
    pad_top_y = HP.HORN_STACK_H + HP.LINK_THICKNESS
    T_tibia_in_link = _T(*knee_joint_local) @ _Ry(pt)
    T_tibia_to_world = (
        _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a) @ T_tibia_in_link
    )
    out: list[FastenerInstance] = []
    for ang in HP.HORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            pad_top_y,
            _HORN_BOLT_PCD_HALF * np.sin(ang),
        ])
        head = _apply_point(T_tibia_to_world, p_local)
        axis = _apply_dir(T_tibia_to_world, np.array([0.0, -1.0, 0.0]))
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"tibia_link L{leg_index} knee-pad-to-X-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="knee",
            length_mm=8.0,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
        ))
    return out


# ---------------------------------------------------------------------------
# Spline center screws (M2.5 x 8, captive under the X-horn after assembly)
# ---------------------------------------------------------------------------


def _emit_spline_fastener(leg_index: int, joint: str) -> list[FastenerInstance]:
    """One M2.5 x 8 spline screw per servo (18 total).

    The screw threads into the servo's spline collar.  In the servo's
    local frame the screw HEAD sits on top of the plastic horn at
    (SERVO_OUTPUT_X, 0, SERVO_BODY_H + SERVO_OUTPUT_H + PLASTIC_HORN_H);
    the axis points down (-Z in servo-local) into the spline.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    # Servo-local head position (x = output offset, y = 0, z = top of
    # plastic horn).
    head_local = np.array([
        HP.SERVO_OUTPUT_X,
        0.0,
        HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H + HP.PLASTIC_HORN_H,
    ])
    axis_local = np.array([0.0, 0.0, -1.0])  # into the spline

    if joint == "yaw":
        # Servo-local frame: same as bracket-local except for the
        # body-position shift (-SERVO_OUTPUT_X, 0, -WELL_RIM_Z) and
        # the bracket's Z rotation.
        T = _T(*edge_mid) @ _Rz(a) @ _T(-HP.SERVO_OUTPUT_X, 0.0, -HP.WELL_RIM_Z)
        role = f"yaw servo spline screw L{leg_index}"
    elif joint == "hip":
        yaw_output_z = (
            (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
            + HP.SERVO_OUTPUT_H
            + HP.HORN_STACK_H
        )
        hip_drop = HP.COXA_HIP_DROP
        delta = np.array([
            HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
            hip_drop,
        ])
        # Servo's local +Z is the output direction; in the coxa link
        # it's rotated by R_x(-pi/2) so servo +Z -> link +Y.
        T = (
            _T(*edge_mid)
            @ _T(0.0, 0.0, yaw_output_z)
            @ _Rz(a)
            @ _T(*delta)
            @ _Rx(-np.pi / 2.0)
        )
        role = f"hip servo spline screw L{leg_index}"
    elif joint == "knee":
        yaw_output_z = (
            (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
            + HP.SERVO_OUTPUT_H
            + HP.HORN_STACK_H
        )
        hip_drop = HP.COXA_HIP_DROP
        p = np.deg2rad(HP.STANCE_FEMUR_DEG)
        delta_knee = np.array([
            HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
            0.0,
        ])
        T_femur_in_link = _T(HP.COXA_LENGTH, 0.0, hip_drop) @ _Ry(p)
        T = (
            _T(*edge_mid)
            @ _T(0.0, 0.0, yaw_output_z)
            @ _Rz(a)
            @ T_femur_in_link
            @ _T(*delta_knee)
            @ _Rx(-np.pi / 2.0)
        )
        role = f"knee servo spline screw L{leg_index}"
    else:
        raise ValueError(f"unknown joint: {joint!r}")

    head = _apply_point(T, head_local)
    axis = _apply_dir(T, axis_local)
    return [
        FastenerInstance(
            part_number=PN_M25X8_SHCS,
            spec=SPEC_M25X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=role,
            leg_index=leg_index,
            joint=joint,
            length_mm=8.0,
            cache_stl=f"{PN_M25X8_SHCS}.cache.stl",
            # The servo spline center screw ships with the servo and
            # sits captive UNDER the plastic 4-arm X-horn after the
            # link bolts onto the horn (Design B retired the printed
            # adapter; the link's pad now bolts directly to the horn,
            # so the spline screw head is buried beneath the link).
            # Install it BEFORE fitting the horn.
            skip_screwdriver_reason=(
                "captive under the X-horn after assembly; install "
                "the spline screw BEFORE fitting the plastic horn"
            ),
        )
    ]


# ---------------------------------------------------------------------------
# Coxa-bracket -> chassis bolts (M3 x 32 SHCS) and their nyloc nuts
# ---------------------------------------------------------------------------


def _emit_chassis_bolts(leg_index: int) -> list[FastenerInstance]:
    """The 4 M3 chassis-bracket bolts drilled through the bottom plate.

    Pattern matches ``make_coxa_bracket()``'s chassis_holes:
        outboard pair: bracket-x = -BRACKET_FLANGE_INSET
        inboard  pair: bracket-x = -BRACKET_FLANGE_INSET - BRACKET_BOLT_PCD_X
        y          = +/- BRACKET_BOLT_PCD_Y / 2
    Heads sit ABOVE the bracket flange (+Z), axis points -Z into the
    flange + chassis stack.  Nut is below the chassis_bottom plate.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    T = _T(*edge_mid) @ _Rz(a)

    bolt_x_outboard = -HP.BRACKET_FLANGE_INSET
    bolt_x_inboard  = -HP.BRACKET_FLANGE_INSET - HP.BRACKET_BOLT_PCD_X
    bolt_ys = (-HP.BRACKET_BOLT_PCD_Y / 2.0, +HP.BRACKET_BOLT_PCD_Y / 2.0)

    out: list[FastenerInstance] = []
    for bx, x_label in ((bolt_x_outboard, "outb"), (bolt_x_inboard, "inb")):
        for by in bolt_ys:
            y_label = "+Y" if by > 0 else "-Y"
            # Head face: just above the bracket flange top (z = BRACKET_FLANGE_T).
            head_local = np.array([bx, by, HP.BRACKET_FLANGE_T])
            head = _apply_point(T, head_local)
            axis = _apply_dir(T, np.array([0.0, 0.0, -1.0]))
            out.append(FastenerInstance(
                part_number=PN_M3X32_SHCS,
                spec=SPEC_M3X32_SHCS,
                head_world_xyz=head,
                axis_world=axis,
                role=f"coxa_bracket L{leg_index} chassis bolt {x_label} {y_label} SHCS",
                leg_index=leg_index,
                joint=None,
                length_mm=32.0,
                cache_stl=f"{PN_M3X32_SHCS}.cache.stl",
                # The chassis bolts are CAPTIVE SUB-ASSEMBLY fasteners:
                # the bracket is bolted DOWN onto the chassis_bottom
                # plate BEFORE the coxa_link is bolted onto the yaw
                # servo's horn (the coxa_link's arm sits in the
                # chassis-bolt driver cone above the bracket flange,
                # so once the link is on, you cannot reach the
                # chassis bolts).
                skip_screwdriver_reason=(
                    "captive sub-assembly fastener: torqued BEFORE "
                    "the coxa_link is bolted onto the yaw servo horn. "
                    "The coxa_link's arm sits in the driver cone "
                    "above the bracket flange post-assembly"
                ),
            ))
            # Nut hangs below the chassis_bottom plate.  Chassis_bottom
            # plate top sits at z = 0; bottom at z = -CHASSIS_PLATE_T;
            # the nyloc nut sits with its outboard (downward-facing)
            # face at z = -CHASSIS_PLATE_T (under the plate).
            nut_local = np.array([bx, by, -HP.CHASSIS_PLATE_T])
            nut_head = _apply_point(T, nut_local)
            nut_axis = _apply_dir(T, np.array([0.0, 0.0, +1.0]))  # nut faces -Z; INTO material is +Z
            out.append(FastenerInstance(
                part_number=PN_M3_NYLOC,
                spec=SPEC_M3_NYLOC,
                head_world_xyz=nut_head,
                axis_world=nut_axis,
                role=f"coxa_bracket L{leg_index} chassis bolt {x_label} {y_label} nyloc nut",
                leg_index=leg_index,
                joint=None,
                length_mm=None,
                cache_stl=f"{PN_M3_NYLOC}.cache.stl",
            ))
    return out


# ---------------------------------------------------------------------------
# Foot hinge bolts (M3 x 16 pan-head, one per leg)
# ---------------------------------------------------------------------------


def _emit_foot_hinge_fastener(leg_index: int) -> list[FastenerInstance]:
    """The single M3 x 16 pan-head hinge pin that captures the foot pad
    in the tibia's clevis.  Bolt axis is parallel to the knee Y axis.
    Head sits on one cheek, nut on the other.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + HP.HORN_STACK_H
    )
    hip_drop = HP.COXA_HIP_DROP
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    Ry_p_3 = _Ry(p)[:3, :3]
    knee_joint_local = np.array([HP.COXA_LENGTH, 0.0, hip_drop]) + Ry_p_3 @ np.array(
        [HP.FEMUR_LENGTH, 0.0, 0.0]
    )
    T_tibia_in_link = _T(*knee_joint_local) @ _Ry(pt)
    T_tibia_to_world = (
        _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a) @ T_tibia_in_link
    )
    # Hinge pin axis is tibia-local +Y at (TIBIA_LENGTH, 0,
    # FOOT_HINGE_TIBIA_Z).  Clevis fork outer Y face on the +Y cheek
    # sits at y = +FOOT_HINGE_GAP/2 + FOOT_HINGE_CHEEK_T; the pan-head
    # bolt enters from outside that cheek face, axis = -Y.
    cheek_outer_y = HP.FOOT_HINGE_GAP / 2.0 + HP.FOOT_HINGE_CHEEK_T
    head_local = np.array([HP.TIBIA_LENGTH, +cheek_outer_y, HP.FOOT_HINGE_TIBIA_Z])
    head = _apply_point(T_tibia_to_world, head_local)
    axis = _apply_dir(T_tibia_to_world, np.array([0.0, -1.0, 0.0]))
    nut_local = np.array([HP.TIBIA_LENGTH, -cheek_outer_y, HP.FOOT_HINGE_TIBIA_Z])
    nut_head = _apply_point(T_tibia_to_world, nut_local)
    nut_axis = _apply_dir(T_tibia_to_world, np.array([0.0, +1.0, 0.0]))
    return [
        FastenerInstance(
            part_number=PN_M3X16_PAN,
            spec=SPEC_M3X16_PAN,
            head_world_xyz=head,
            axis_world=axis,
            role=f"foot L{leg_index} hinge pin pan-head",
            leg_index=leg_index,
            joint=None,
            length_mm=16.0,
            cache_stl=f"{PN_M3X16_PAN}.cache.stl",
        ),
        FastenerInstance(
            part_number=PN_M3_NYLOC,
            spec=SPEC_M3_NYLOC,
            head_world_xyz=nut_head,
            axis_world=nut_axis,
            role=f"foot L{leg_index} hinge pin nyloc nut",
            leg_index=leg_index,
            joint=None,
            length_mm=None,
            cache_stl=f"{PN_M3_NYLOC}.cache.stl",
        ),
    ]


# ---------------------------------------------------------------------------
# Top-level builder
# ---------------------------------------------------------------------------


def build_all_fastener_instances() -> list[FastenerInstance]:
    """Return every FastenerInstance in the assembled robot.

    Pure data; no rendering.  Mirrors the leg-by-leg transforms in
    ``build_prototype_assembly._build_leg`` so the world coordinates
    match the build inspector exactly.
    """
    out: list[FastenerInstance] = []
    for leg_index in range(6):
        # Cradle bolts (Design C, May 2026 revert: vertical M3 self-tap
        # SHCS into Phi 2.5 mm printed pilots).  The coxa_bracket's
        # shelf top sits BRACKET_SHELF_DROP_MM = 3 mm below WELL_RIM_Z
        # because its drop-in slot eats the rim above bracket-z = -3;
        # the coxa_link and femur cradles keep WELL_RIM_Z intact.
        out.extend(_emit_cradle_fasteners(
            T_well_to_world=_yaw_cradle_T(leg_index),
            leg_index=leg_index,
            joint="yaw",
            location=f"coxa_bracket L{leg_index} yaw cradle",
            shelf_top_z=HP.WELL_RIM_Z - _BRACKET_SHELF_DROP_MM,
        ))
        out.extend(_emit_cradle_fasteners(
            T_well_to_world=_hip_cradle_T(leg_index),
            leg_index=leg_index,
            joint="hip",
            location=f"coxa_link L{leg_index} hip cradle",
        ))
        out.extend(_emit_cradle_fasteners(
            T_well_to_world=_knee_cradle_T(leg_index),
            leg_index=leg_index,
            joint="knee",
            location=f"femur_link L{leg_index} knee cradle",
        ))

        # Link-to-X-horn bolts (Design B: no more printed adapter disc).
        out.extend(_emit_horn_fasteners_yaw(leg_index))
        out.extend(_emit_horn_fasteners_hip(leg_index))
        out.extend(_emit_horn_fasteners_knee(leg_index))

        # Servo spline center screws (3 servos x 6 legs = 18 total).
        out.extend(_emit_spline_fastener(leg_index, "yaw"))
        out.extend(_emit_spline_fastener(leg_index, "hip"))
        out.extend(_emit_spline_fastener(leg_index, "knee"))

        # Coxa-bracket -> chassis bolts (M3 x 32 SHCS + nyloc nuts).
        out.extend(_emit_chassis_bolts(leg_index))

        # Foot hinge pin (M3 x 16 pan-head + nyloc nut).
        out.extend(_emit_foot_hinge_fastener(leg_index))

    return out


# ---------------------------------------------------------------------------
# BOM helpers
# ---------------------------------------------------------------------------


def fastener_bom_rows() -> list[tuple[str, str, int, str]]:
    """Return (spec, part_number, qty, used_in) rows for the BOM table,
    aggregated across ``build_all_fastener_instances()``."""
    counts: dict[tuple[str, str], int] = {}
    usage: dict[tuple[str, str], set[str]] = {}
    for fi in build_all_fastener_instances():
        key = (fi.spec, fi.part_number)
        counts[key] = counts.get(key, 0) + 1
        usage.setdefault(key, set()).add(_usage_bucket(fi))
    rows = []
    for (spec, pn), qty in sorted(counts.items()):
        used = ", ".join(sorted(usage[(spec, pn)]))
        rows.append((spec, pn, qty, used))
    # Stable, human-friendly order: SHCS by length, then nuts.
    spec_order = {
        SPEC_M3X8_SHCS:  0,
        SPEC_M3X32_SHCS: 1,
        SPEC_M3X16_PAN:  2,
        SPEC_M25X8_SHCS: 3,
        SPEC_M3_NYLOC:   4,
    }
    rows.sort(key=lambda r: (spec_order.get(r[0], 9), r[0]))
    return rows


def _usage_bucket(fi: FastenerInstance) -> str:
    role = fi.role
    if "cradle" in role:
        # The 72 vertical SHCS that thread DOWN through each servo ear
        # into a Phi 2.5 mm printed self-tap pilot in the cradle shelf
        # (Design C revert -- see hexapod_prototype.SHCS_PILOT_OD).\n
        return "cradle servo self-tap mounts"
    if "X-horn" in role:
        return "link-to-X-horn bolts"
    if "spline screw" in role:
        return "servo spline center screws"
    if "chassis bolt" in role:
        return "coxa-bracket-to-chassis bolts"
    if "hinge" in role:
        return "foot hinge pins"
    return role


# ---------------------------------------------------------------------------
# Self-test (run via ``python fastener_registry.py``)
# ---------------------------------------------------------------------------


def _self_test_summary() -> str:
    rows = fastener_bom_rows()
    out = []
    out.append("Fastener registry self-test:")
    total = 0
    for spec, pn, qty, used in rows:
        out.append(f"  {qty:4d} x {spec:24s} {pn:10s}  {used}")
        total += qty
    out.append(f"  ---")
    out.append(f"  {total:4d} fasteners total")
    return "\n".join(out)


if __name__ == "__main__":
    print(_self_test_summary())
