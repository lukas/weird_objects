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
   below.  PN_M3X8_SHCS / 91290A113.  The brief May 2026 horizontal-
   nyloc iteration was retired after the audit surfaced (a) a wire-
   channel collision in the +X wall and (b) a >MIN_PRINT_T outer-wall
   violation around the Phi 5.6 mm hex pocket; see ``PROTOTYPE.md``
   (Design C section) for the audit table and the revert rationale.

2. ``72 x M2 x 8 SHCS`` -- link-to-X-horn bolts.  4 per joint
   (XHORN_BOLT_PCD = 20.8 mm circle) x (yaw + hip + knee) = 3 joints
   per leg x 6 legs.  Threads downward from the printed link's pad
   face into the plastic 4-arm X-horn that ships with the servo
   (Design B retired the printed adapter disc; the link clamps the
   X-horn directly).  PN_M2X8_SHCS / 91290A005 -- a plain M2 SHCS
   used as a self-tapper into the X-horn's existing Phi ~ 2.0 mm
   M2-sized arm hole (the plastic arm provides the actual thread
   engagement).  May 2026 fastener-spec fix: the X-horn arm holes
   are Phi ~ 2.0 mm, NOT Phi 3.2 mm M3 clearance -- an M3 SHCS
   literally would not fit through the plastic horn's arm.  See
   ``hexapod_prototype.py`` XHORN_BOLT_* docstring + ``fasteners/
   README.md`` for the McMaster thread-former upgrade path.

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

8. ``4 x M3 x 10 SHCS`` -- battery_holder foot bolts.  Driven UP
   through chassis_bottom (one per foot, on the BATTERY_FOOT_DX /
   DY square pattern) into the brass heat-set insert that lives in
   each holder foot.  May 2026 fix: the holder used to be enumerated
   under "not yet enumerated" with 4 M3 clearance holes in the feet
   that never matched anything in chassis_bottom; the registry now
   places both the bolts and their captive heat-set inserts so the
   verifier's check_fastener_engagement probes them on every run.

9. ``4 x M3 heat-set inserts`` -- one per battery_holder foot (see 8.).
   Mirrors the f03d59b cradle insert pattern: Phi 4.0 mm pocket cut
   from the foot's BOTTOM face with the brass insert recessed 0.5 mm
   so the bolt head clamps the chassis_bottom plate against plastic,
   not brass.

10. ``8 x M3 x 8 SHCS`` -- electronics_tray board-mount bolts for the
    Arduino Mega 2560 (4) and the PCA9685 PWM driver (4).  Threads
    DOWN through each board's M3 mounting hole into an M3 brass
    heat-set insert (McMaster 94459A130) embedded in a printed
    Phi 4 mm boss on top of the tray.  May 2026 single-tray
    Mega + Pi + PCA9685 layout.

11. ``8 x M3 heat-set inserts`` -- 4 captive in the Mega's bosses,
    4 in the PCA9685's bosses (see 10.).  Same McMaster
    ``94459A130`` part as the cradle + battery_holder inserts.

12. ``4 x M2.5 x 8 SHCS`` -- electronics_tray board-mount bolts for
    the Raspberry Pi 4 / Pi 5.  Threads DOWN through each Pi
    mounting hole into an M2.5 brass heat-set insert (McMaster
    94459A106).  PN_M25X8_BOARD_SHCS (same physical stock as the
    M2.5 spline screw, distinct role label).

13. ``4 x M2.5 heat-set inserts`` -- captive in the Pi's bosses (see
    12.).  McMaster ``94459A106``.

Categories NOT enumerated yet (acknowledged future work; see
PROTOTYPE_BOM.md "Fasteners" auto-derived section):

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
PN_M25X8_BOARD_SHCS = "91290A102"  # M2.5 x 8 SHCS used as a board-mount bolt
                                   # threaded into a Phi 3.0 mm M2.5 brass
                                   # heat-set insert in the electronics_tray.
                                   # Same stock as PN_M25X8_SHCS (the servo
                                   # spline screw); the distinct PN here
                                   # surfaces the role in the BOM so the
                                   # user buys both sets together.
PN_M25_HEATSET_INSERT = "94459A106"  # M2.5 brass heat-set insert, knurled
                                      # (McMaster).  Pilot Phi 3.0 mm,
                                      # length 4.0 mm, recommended pilot
                                      # depth 4.5 mm.  4 of these mount the
                                      # Raspberry Pi 4 / Pi 5 onto the
                                      # electronics_tray (May 2026).
PN_M3X10_SHCS    = "91290A114"   # M3 x 10 socket-head cap screw, black-oxide steel
                                  # (battery_holder foot bolts: head bears on
                                  # the UNDER face of chassis_bottom, threads
                                  # UP through the plate into a heat-set
                                  # insert in the holder foot above)
PN_M3_HEATSET_INSERT = "94459A130"   # M3 brass heat-set insert, knurled (McMaster)
PN_M3X32_SHCS    = "91290A123"   # M3 x 30 socket-head cap screw (closest stock to 32 mm)
PN_M3_NYLOC      = "90576A102"   # M3 nylon-insert lock nut, A2 stainless
PN_M3X16_PAN     = "92010A130"   # M3 x 16 pan-head Phillips, A2 stainless (foot hinge)
PN_M25X8_SHCS    = "91290A104"   # M2.5 x 8 socket-head cap screw (servo spline)
# Link-to-X-horn bolts (May 2026 M3 -> M2 fastener-spec fix; see the
# XHORN_BOLT_* docstring in hexapod_prototype.py).  Plain M2 SHCS used
# as a self-tapper into the X-horn's existing Phi ~ 2.0 mm arm hole;
# the X-horn plastic provides the actual thread engagement.  An
# optional thread-forming upgrade exists for in-plastic use --
# McMaster ``99461A340`` (M2 x 8 thread-forming) -- documented in
# fasteners/README.md.  We keep the plain SHCS as the default because
# the McMaster stock is reliable, the visual mesh is identical at
# inspector zoom, and self-tap behaviour is supplied by the X-horn's
# pre-drilled hole.
PN_M2X8_SHCS     = "91290A005"   # M2 x 8 socket-head cap screw, black-oxide steel
# May 2026 revert: the brief Design C horizontal-nyloc cradle bolt
# (PN_M3X14_SHCS = 91290A115) was retired in favour of vertical M3 x 8
# self-tap SHCS into Phi 2.5 mm printed pilots, reusing the existing
# M3 x 8 stock used as servo cradle mount bolts.

# Human-readable spec labels (used by the inspector and the BOM script).
SPEC_M3X8_SHCS   = "M3x8 SHCS"
SPEC_M3X10_SHCS  = "M3x10 SHCS"   # battery_holder foot bolts (4); threads
                                   # into M3 brass heat-set insert in the
                                   # holder foot above chassis_bottom.
# Cradle bolt spec post-heat-set switch (May 2026): same M3 x 8
# SHCS stock as ``SPEC_M3X8_SHCS`` (same P/N -- they are the same
# fastener), but with a distinct spec string so the verifier's
# ``check_screwdriver_access`` and BOM reports can identify cradle
# bolts that thread into a heat-set insert instead of a plastic
# pilot without needing to read the role / location strings.  The
# "SHCS" substring is preserved so the screwdriver-envelope
# dispatcher (which dispatches HEX_KEY off the "SHCS" substring)
# still picks the right driver envelope.
SPEC_M3X8_SHCS_INTO_INSERT = "M3x8 SHCS into heat-set insert"
# Heat-set insert spec: McMaster 94459A130 M3 brass knurled insert,
# Phi 4.0 mm pilot, Phi 5.7 mm OD, 5.0 mm length.  Installed with a
# soldering iron at ~220 deg C; the bolt threads into it from above.
SPEC_M3_HEATSET_INSERT = "M3 heat-set insert"
SPEC_M3X32_SHCS  = "M3x32 SHCS"
SPEC_M3_NYLOC    = "M3 nyloc nut"
SPEC_M3X16_PAN   = "M3x16 pan-head"
SPEC_M25X8_SHCS  = "M2.5x8 spline screw"
# Pi-mount bolt + insert specs (May 2026, electronics-tray expansion).
# Plain M2.5 x 8 SHCS used as a board-mount bolt; threads into an
# M2.5 brass heat-set insert (McMaster 94459A106) embedded in a
# Phi 3.0 mm pocket in the electronics_tray boss.  The "SHCS"
# substring is preserved so the screwdriver-envelope dispatcher
# (HEX_KEY) picks the right driver envelope; "M2.5" disambiguates
# the spline-screw entries above.
SPEC_M25X8_SHCS_INTO_INSERT = "M2.5x8 SHCS into heat-set insert"
SPEC_M25_HEATSET_INSERT     = "M2.5 heat-set insert"
SPEC_M2X8_SHCS   = "M2x8 SHCS"   # link-to-X-horn self-tap bolts


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
    """Emit the cradle servo-mount fasteners for one cradle (heat-set
    switch, May 2026).

    Each (sx, sy) site emits TWO ``FastenerInstance`` entries:

      * an ``M3 x 8 SHCS into heat-set insert`` bolt (P/N
        ``PN_M3X8_SHCS`` = 91290A113, spec
        ``SPEC_M3X8_SHCS_INTO_INSERT``) with its head sitting on the
        servo ear's top face; and
      * an ``M3 heat-set insert`` (P/N ``PN_M3_HEATSET_INSERT`` =
        94459A130, spec ``SPEC_M3_HEATSET_INSERT``) pressed into the
        printed Phi 4 mm pocket below the ear.  The insert's TOP
        face sits 0.5 mm below the shelf top so the bolt head clamps
        the servo ear down onto the printed boss top instead of the
        brass insert face (standard heat-set practice).

    ``shelf_top_z`` is the well-local z of the shelf surface the servo
    ear rests on -- ``WELL_RIM_Z`` for the femur / coxa_link cradles
    and ``WELL_RIM_Z - BRACKET_SHELF_DROP_MM`` for the coxa_bracket
    yaw cradle whose rim was eaten by the drop-in slot.
    """
    if shelf_top_z is None:
        shelf_top_z = HP.WELL_RIM_Z

    positions, axes, signs = _well_bolt_local_pos_axis(shelf_top_z)

    # Insert head sits 0.5 mm below the shelf top so the bolt head
    # clamps the servo ear down onto the printed boss top (standard
    # heat-set practice; the bolt should not seat directly on the
    # brass insert face).
    insert_head_local_z = shelf_top_z - 0.5

    out: list[FastenerInstance] = []
    for (sx, sy), p_local, ax_local in zip(signs, positions, axes):
        head = _apply_point(T_well_to_world, p_local)
        axis = _apply_dir(T_well_to_world, ax_local)
        role_suffix = _wall_corner_label(sx, sy)
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS_INTO_INSERT,
            head_world_xyz=head,
            axis_world=axis,
            role=f"{location} {role_suffix} M3 x 8 SHCS into insert",
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
                "captive sub-assembly fastener: the M3 x 8 SHCS "
                "threads into a heat-set insert and is torqued from "
                "above BEFORE the next stage's link / plate is "
                "bolted onto the cradle's servo horn (yaw cradle "
                "-> coxa_link, hip cradle -> femur, knee cradle "
                "-> tibia; bracket yaw cradle also requires the "
                "chassis-top stack to be added LAST).  See "
                "PROTOTYPE.md for the explicit sub-assembly order"
            ),
        ))

        # Heat-set insert entry coaxial with the bolt.  Insert head
        # at well-local z = shelf_top_z - 0.5 mm so the bolt head
        # clamps the ear onto the printed boss top, not the brass.
        insert_head_local = np.array([
            sx * HP.SERVO_MOUNT_HOLE_X_OFFSET,
            sy * HP.SERVO_MOUNT_HOLE_Y_OFFSET,
            insert_head_local_z,
        ])
        insert_head_world = _apply_point(T_well_to_world, insert_head_local)
        out.append(FastenerInstance(
            part_number=PN_M3_HEATSET_INSERT,
            spec=SPEC_M3_HEATSET_INSERT,
            head_world_xyz=insert_head_world,
            axis_world=axis,
            role=f"{location} {role_suffix} M3 heat-set insert",
            leg_index=leg_index,
            joint=joint,
            length_mm=HP.INSERT_M3_INSERT_LENGTH,
            cache_stl=f"{PN_M3_HEATSET_INSERT}.cache.stl",
            # The insert is PRESSED IN with a soldering iron BEFORE
            # any link / plate is bolted to the cradle's servo horn,
            # so it lives behind every screwdriver cone the assembler
            # would ever swing.  No driver clearance is required for
            # the insert itself (it has no head to drive), just for
            # the soldering-iron tip that installs it -- the same
            # axial clearance the M3 x 8 SHCS bolt needs, which is
            # captive for the same reason.
            skip_screwdriver_reason=(
                "heat-set insert installed with a soldering iron "
                "BEFORE the next stage's link / plate is added (see "
                "PROTOTYPE.md for the assembly order); no driver "
                "cone applies to the insert itself"
            ),
        ))
    return out


# ---------------------------------------------------------------------------
# Per-joint horn-bolt fastener generators
# ---------------------------------------------------------------------------
#
# Each rotary joint (yaw, hip-pitch, knee-pitch) clamps the printed link's
# pad onto the plastic 4-arm X-horn via 4 x M2 SHCS on XHORN_BOLT_PCD =
# 20.8 mm.  Design B (May 2026) retired the printed adapter disc, so the
# bolts thread DIRECTLY from the link's pad into the X-horn that ships
# with the servo.  The X-horn arm has a Phi ~ 2.0 mm untapped through
# hole (M2 self-tap-sized); the plain M2 SHCS self-taps a clean thread
# into the plastic on first install.  Bolt length budget:
#   * coxa_link hub:    COXA_LIFT + hub_t = 36 + 8 mm of pad/pedestal
#   * femur hip pad:    LINK_THICKNESS = 6 mm
#   * tibia knee pad:   LINK_THICKNESS = 6 mm
# All three pad thicknesses share a single M2 x 8 SHCS stock (the
# coxa_link's pedestal-bolt run is taller, but the bolt only needs to
# engage the X-horn's Phi 2.0 mm self-tap thread for
# XHORN_BOLT_THREAD_ENGAGEMENT_MM = 3 mm; the rest of the pedestal is
# a clearance pass-through).  M2 x 8 is the safe round-up for
# pad (3-4 mm) + X-horn thread (3 mm) + bolt-head clearance (1 mm)
# = 7-8 mm.  See the user's catch in the May 2026 audit and
# ``fasteners/README.md`` (PN 91290A005 entry).

_HORN_BOLT_PCD_HALF = HP.XHORN_BOLT_PCD / 2.0


def _emit_horn_fasteners_yaw(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-X-horn bolts at the yaw joint (coxa_link hub).

    Each M2 SHCS sits in a counter-bore cut into the TOP of the
    pedestal's bottom cap and clamps the cap DOWN onto the plastic
    X-horn that lives at link-local z in [-PLASTIC_HORN_H, 0].
    Geometry in link-local z (cap spans z in [0, PEDESTAL_CAP_T] =
    [0, 4] mm)::

        head bearing face  : z = PEDESTAL_CAP_T - COUNTERBORE_DEPTH
                             = 4 - 2.5 = 1.5  (= counter-bore floor)
        shaft clearance run: z in [0, 1.5]  (1.5 mm of cap below head)
        X-horn engagement  : z in [-PLASTIC_HORN_H, 0]  (the bolt
                             threads downward into the 5 mm-thick
                             plastic horn; the last 3 mm is the
                             design-required XHORN_BOLT_THREAD_
                             ENGAGEMENT_MM = 3 mm).
        bolt tip overshoot : z in [-PLASTIC_HORN_H - 1.5,
                                   -PLASTIC_HORN_H]  (the stock M2 x
                             8 SHCS is 1.5 mm longer than the
                             cap + horn stack so the tip pokes
                             through the horn into free air below).

    Before this fix (commit b5f7095): the head was placed at the
    hub's TOP face (link-local z = COXA_LIFT + hub_t = 44 mm); the
    bolt floated 36 mm above the X-horn entirely inside the printed
    hub.  check_fastener_engagement caught this with a
    "joins only 1 part [coxa_link]" failure on every yaw bolt.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = (
        (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
        + HP.SERVO_OUTPUT_H
        + HP.HORN_STACK_H
    )
    # Link-local z = 0 IS the cap's bottom mating face with the
    # X-horn; the link's transform places that face at world z =
    # yaw_output_z, which is exactly the X-horn's top face per
    # check_mating_face_contact's "coxa_link bottom <-> yaw X-horn
    # top" probe (gap = +0.00 mm).
    head_local_z = HP.PEDESTAL_CAP_T - HP.COUNTERBORE_DEPTH
    T_link_to_world = _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a)
    out: list[FastenerInstance] = []
    for ang in HP.XHORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            _HORN_BOLT_PCD_HALF * np.sin(ang),
            head_local_z,
        ])
        head = _apply_point(T_link_to_world, p_local)
        axis = _apply_dir(T_link_to_world, np.array([0.0, 0.0, -1.0]))
        out.append(FastenerInstance(
            part_number=PN_M2X8_SHCS,
            spec=SPEC_M2X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"coxa_link L{leg_index} hub-to-X-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="yaw",
            length_mm=8.0,
            cache_stl=f"{PN_M2X8_SHCS}.cache.stl",
            # Captive sub-assembly fastener.  Per PROTOTYPE.md
            # section 6.1 the 4 yaw M2 x 8 SHCS are driven in step 3
            # (coxa_link dropped onto yaw X-horn, bolts torqued from
            # above through the pedestal-cap counter-bore) BEFORE the
            # femur is mounted on the hip X-horn in step 5.  At
            # standing pose the +X yaw bolt (ang = 0 deg) is then
            # blocked from above by the femur's hip pad which tilts
            # up and back over the coxa_link hub when p_femur =
            # -25 deg; the other 3 yaw bolts happen to retain driver
            # access in the assembled state but are all driven at
            # the same assembly step before the femur is in place,
            # so the explicit allow-list applies uniformly.
            skip_screwdriver_reason=(
                "captive sub-assembly fastener: the M2 x 8 SHCS is "
                "torqued through the pedestal-cap counter-bore "
                "BEFORE the femur is mounted on the hip X-horn "
                "(PROTOTYPE.md section 6.1 step 3 vs step 5).  At "
                "standing pose (p_femur = -25 deg) the +X yaw bolt "
                "is blocked from above by the femur hip pad."
            ),
        ))
    return out


def _emit_horn_fasteners_hip(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-X-horn bolts at the hip-pitch joint (femur hip pad).

    The femur's hip pad is LINK_THICKNESS = 6 mm thick in +Y; the
    pad's -Y mating face touches the X-horn arm.  Counter-bores
    landed in 8745b05 cut Phi M2_HEAD_OD_CLEARANCE = 4 mm pockets
    COUNTERBORE_DEPTH = 2.5 mm deep into the pad's +Y outer face.
    Femur-local y of each face::

        +Y outer face        : y = HORN_STACK_H + LINK_THICKNESS = 11
        counter-bore floor   : y = 11 - 2.5 = 8.5  (head bearing)
        -Y mating face       : y = HORN_STACK_H = 5  (X-horn top)
        3 mm thread depth    : y in [2, 5]
        bolt tip overshoot   : y in [1.5, 2] (M2 x 8 SHCS overhangs
                               the 3 mm engagement target by ~ 1.5
                               mm; the bolt is sized for the
                               taller coxa_link cap + horn stack).

    Pre-fix the head was placed at the +Y OUTER face (y = 11); the
    head-bearing ring probes then landed inside the Phi 4 mm
    counter-bore void at y in [8.5, 11] and check_fastener_engagement
    reported "head bearing in air".
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
    # Counter-bore FLOOR (head bearing face) sits COUNTERBORE_DEPTH
    # below the pad's outer +Y face.
    head_local_y = (HP.HORN_STACK_H + HP.LINK_THICKNESS
                    - HP.COUNTERBORE_DEPTH)
    T_femur_in_link = _T(HP.COXA_LENGTH, 0.0, hip_drop) @ _Ry(p)
    T_femur_to_world = (
        _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a) @ T_femur_in_link
    )
    out: list[FastenerInstance] = []
    for ang in HP.XHORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            head_local_y,
            _HORN_BOLT_PCD_HALF * np.sin(ang),
        ])
        head = _apply_point(T_femur_to_world, p_local)
        axis = _apply_dir(T_femur_to_world, np.array([0.0, -1.0, 0.0]))
        out.append(FastenerInstance(
            part_number=PN_M2X8_SHCS,
            spec=SPEC_M2X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"femur_link L{leg_index} hip-pad-to-X-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="hip",
            length_mm=8.0,
            cache_stl=f"{PN_M2X8_SHCS}.cache.stl",
        ))
    return out


def _emit_horn_fasteners_knee(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-X-horn bolts at the knee joint (tibia knee pad).

    Mirrors ``_emit_horn_fasteners_hip``: head bearing face sits on
    the COUNTERBORE_DEPTH-deep pocket floor on the pad's outer +Y
    face, bolt axis points -Y INTO the pad and through the -Y mating
    face into the plastic X-horn below.  See that function's
    docstring for the y-coordinate breakdown.
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
    head_local_y = (HP.HORN_STACK_H + HP.LINK_THICKNESS
                    - HP.COUNTERBORE_DEPTH)
    T_tibia_in_link = _T(*knee_joint_local) @ _Ry(pt)
    T_tibia_to_world = (
        _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a) @ T_tibia_in_link
    )
    out: list[FastenerInstance] = []
    for ang in HP.XHORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            head_local_y,
            _HORN_BOLT_PCD_HALF * np.sin(ang),
        ])
        head = _apply_point(T_tibia_to_world, p_local)
        axis = _apply_dir(T_tibia_to_world, np.array([0.0, -1.0, 0.0]))
        out.append(FastenerInstance(
            part_number=PN_M2X8_SHCS,
            spec=SPEC_M2X8_SHCS,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"tibia_link L{leg_index} knee-pad-to-X-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="knee",
            length_mm=8.0,
            cache_stl=f"{PN_M2X8_SHCS}.cache.stl",
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
# Battery-holder foot bolts (M3 x 10 SHCS + M3 heat-set insert pair)
# ---------------------------------------------------------------------------


def _emit_battery_holder_fasteners() -> list[FastenerInstance]:
    """The 4 battery-holder foot bolts + their captive heat-set inserts.

    Mirrors the f03d59b cradle insert pattern: each foot has a
    Phi INSERT_M3_PILOT_OD = 4.0 mm x INSERT_M3_PILOT_DEPTH = 6.0 mm
    pocket cut from its BOTTOM face that holds an M3 brass heat-set
    insert (McMaster 94459A130).  The 4 M3 x 10 SHCS enter from
    BELOW: head face bears on the UNDER side of chassis_bottom
    (z = -CHASSIS_PLATE_T in the design frame), axis = +Z (UP), and
    threads engage the brass insert in the foot above.

    Length budget (design frame, z = 0 = chassis_bottom top face =
    battery_holder bottom face):

        head face         : z = -CHASSIS_PLATE_T = -4 mm
        chassis-plate run : z in [-4, 0]                (4 mm)
        recess gap        : z in [0, BATTERY_FOOT_INSERT_RECESS]
                            = [0, 0.5] mm  (plastic between bolt
                            head ring and insert face -- standard
                            heat-set practice so the head clamps the
                            chassis plate against plastic, not brass)
        insert engagement : z in [0.5, 5.5]              (5 mm)
        tip overshoot     : z in [5.5, 6.0]              (0.5 mm
                            free space inside the pocket's overdrill
                            before the closed top at z = 6).

    Total bolt run: 4 + 0.5 + 5 + 0.5 = 10 mm.  M3 x 10 SHCS
    (PN 91290A114) is the smallest stock length that meets the
    5 mm-insert-engagement target with the 4 mm chassis_bottom plate
    in the stack.

    The fastener is also a CAPTIVE SUB-ASSEMBLY: PROTOTYPE.md
    section 6.1 step 11 installs the battery_holder onto chassis_
    bottom AFTER the 4 heat-set inserts have been pressed into the
    feet but BEFORE the chassis_top + standoff stack is added.  The
    driver cone for the SHCS therefore swings clear in -Z (under
    the assembled robot) at the time of installation, not blocked
    by chassis_top from above.
    """
    out: list[FastenerInstance] = []
    # chassis_bottom is generated centred on z = 0 (its centre plane
    # at z = 0; bounds z in [-CHASSIS_PLATE_T/2, +CHASSIS_PLATE_T/2] =
    # [-2, +2]) and the verifier / build_prototype_assembly place it
    # at world (0, 0, chassis_lift) without flipping that convention.
    # The bolt head bearing face therefore sits at world z =
    # -CHASSIS_PLATE_T / 2 = -2 mm (chassis_bottom UNDER face) in the
    # pre-chassis-lift world frame the registry returns.
    chassis_bottom_face_z = -HP.CHASSIS_PLATE_T / 2.0
    # The battery_holder mesh is generated with its bottom face at
    # local z = 0 and is placed at world z = +CHASSIS_PLATE_T / 2 = +2
    # so its bottom face sits flush on chassis_bottom top face.  The
    # heat-set insert top face is BATTERY_FOOT_INSERT_RECESS = 0.5 mm
    # ABOVE the foot bottom (recessed INTO the foot from below so the
    # bolt head clamps chassis plastic, not brass).
    insert_top_z = HP.CHASSIS_PLATE_T / 2.0 + HP.BATTERY_FOOT_INSERT_RECESS
    for sx in (-1, 1):
        for sy in (-1, 1):
            # The battery_holder is OFFSET in X relative to the
            # chassis centre (HP.BATTERY_HOLDER_CENTRE_X = -25 mm);
            # the foot world X position is therefore
            # ``BATTERY_HOLDER_CENTRE_X + sx * BATTERY_FOOT_DX``.
            # build_prototype_assembly and the verifier's
            # _build_world_leg0_printed_parts apply the same offset
            # to the holder mesh, and _hex_plate's
            # ``with_battery_holder_holes`` pattern offsets the
            # chassis_bottom holes by the same amount.
            fx = HP.BATTERY_HOLDER_CENTRE_X + sx * HP.BATTERY_FOOT_DX
            fy = sy * HP.BATTERY_FOOT_DY
            x_label = "+X" if sx > 0 else "-X"
            y_label = "+Y" if sy > 0 else "-Y"

            head_world = np.array([fx, fy, chassis_bottom_face_z])
            axis_world = np.array([0.0, 0.0, 1.0])  # +Z (UP into material)
            out.append(FastenerInstance(
                part_number=PN_M3X10_SHCS,
                spec=SPEC_M3X10_SHCS,
                head_world_xyz=head_world,
                axis_world=axis_world,
                role=(
                    f"battery_holder foot bolt {x_label}{y_label} "
                    f"M3 x 10 SHCS into heat-set insert"
                ),
                leg_index=None,
                joint=None,
                length_mm=10.0,
                cache_stl=f"{PN_M3X10_SHCS}.cache.stl",
            ))

            insert_head_world = np.array([fx, fy, insert_top_z])
            out.append(FastenerInstance(
                part_number=PN_M3_HEATSET_INSERT,
                spec=SPEC_M3_HEATSET_INSERT,
                head_world_xyz=insert_head_world,
                axis_world=axis_world,
                role=(
                    f"battery_holder foot {x_label}{y_label} "
                    f"M3 heat-set insert"
                ),
                leg_index=None,
                joint=None,
                length_mm=HP.INSERT_M3_INSERT_LENGTH,
                cache_stl=f"{PN_M3_HEATSET_INSERT}.cache.stl",
                # The insert is pressed in with a soldering iron from
                # BELOW (foot inverted on the bench) BEFORE the
                # battery_holder is bolted onto chassis_bottom; no
                # driver cone applies to the brass insert itself.
                skip_screwdriver_reason=(
                    "heat-set insert installed with a soldering iron "
                    "BEFORE the battery_holder is bolted to "
                    "chassis_bottom (PROTOTYPE.md section 6.1 step "
                    "11); no driver cone applies to the brass insert"
                ),
            ))
    return out


# ---------------------------------------------------------------------------
# Electronics-tray board-mount fasteners (M3/M2.5 SHCS + heat-set inserts)
# ---------------------------------------------------------------------------


def _emit_electronics_tray_fasteners() -> list[FastenerInstance]:
    """The 12 electronics-tray board-mount fasteners + their captive
    heat-set inserts (May 2026, single-tray Mega + Pi + PCA9685
    layout).

    The tray's chassis-frame placement is
    ``(HP.ELEC_TRAY_CENTRE_X, HP.ELEC_TRAY_CENTRE_Y,
       HP.CHASSIS_PLATE_T / 2 + 3)``; the tray's TOP face sits at
    ``z = HP.CHASSIS_PLATE_T / 2 + 3 + HP.ELEC_TRAY_T`` and the
    standoff bosses extend ``HP.ELEC_STANDOFF_H`` mm above that.
    Each insert top face is recessed ``HP.INSERT_M3_PILOT_DEPTH``
    (or ``HP.INSERT_M25_PILOT_DEPTH``) below the boss top so the
    bolt head clamps the board onto the boss top, not directly
    onto the brass insert.

    All 12 board-mount sites are CAPTIVE SUB-ASSEMBLY fasteners:
    they are torqued during the electronics install BEFORE
    chassis_top is bolted onto the standoffs from above.  Once the
    top plate + (optional) arm bracket close over the tray a hex
    key cannot reach the heads, so ``skip_screwdriver_reason``
    marks them all SKIPped (same convention as the cradle bolts +
    the battery_holder foot bolts).
    """
    out: list[FastenerInstance] = []

    tray_top_z = HP.CHASSIS_PLATE_T / 2.0 + 3.0 + HP.ELEC_TRAY_T
    boss_top_z = tray_top_z + HP.ELEC_STANDOFF_H

    def _absolute_xy(centre, offsets):
        cx, cy = centre
        return [(cx + ox, cy + oy) for ox, oy in offsets]

    board_specs = (
        # (label, centre_xy, hole_offsets, pilot_depth, bolt_pn, bolt_spec,
        #  bolt_length, insert_pn, insert_spec, insert_length)
        (
            "Mega2560",
            HP.MEGA_CENTRE, HP.MEGA_HOLES,
            HP.INSERT_M3_PILOT_DEPTH,
            PN_M3X8_SHCS, SPEC_M3X8_SHCS_INTO_INSERT,
            8.0,
            PN_M3_HEATSET_INSERT, SPEC_M3_HEATSET_INSERT,
            HP.INSERT_M3_INSERT_LENGTH,
        ),
        (
            "Pi4",
            HP.PI_CENTRE, HP.PI_HOLES,
            HP.INSERT_M25_PILOT_DEPTH,
            PN_M25X8_BOARD_SHCS, SPEC_M25X8_SHCS_INTO_INSERT,
            8.0,
            PN_M25_HEATSET_INSERT, SPEC_M25_HEATSET_INSERT,
            HP.INSERT_M25_INSERT_LENGTH,
        ),
        (
            "PCA9685",
            HP.PCA_CENTRE, HP.PCA_HOLES,
            HP.INSERT_M3_PILOT_DEPTH,
            PN_M3X8_SHCS, SPEC_M3X8_SHCS_INTO_INSERT,
            8.0,
            PN_M3_HEATSET_INSERT, SPEC_M3_HEATSET_INSERT,
            HP.INSERT_M3_INSERT_LENGTH,
        ),
    )

    skip_reason = (
        "captive sub-assembly fastener: torqued during electronics "
        "install before chassis_top + (optional) arm baseplate "
        "close the chassis stack from above.  Once the top plate "
        "is on, a hex key cannot reach the board-mount heads"
    )

    for (label, centre, offsets, pilot_depth, bolt_pn, bolt_spec,
         bolt_len, insert_pn, insert_spec, insert_len) in board_specs:
        for (hx_tray, hy_tray) in _absolute_xy(centre, offsets):
            hx = HP.ELEC_TRAY_CENTRE_X + hx_tray
            hy = HP.ELEC_TRAY_CENTRE_Y + hy_tray

            # SHCS head bears on the BOARD's top face = boss top +
            # board thickness; conservative: place head face at the
            # boss top (matches inspect_build's bolt rendering and
            # avoids leaking a per-board PCB-thickness constant).
            # axis_world points DOWN into the insert.
            head_world = np.array([hx, hy, boss_top_z])
            axis_world = np.array([0.0, 0.0, -1.0])
            out.append(FastenerInstance(
                part_number=bolt_pn,
                spec=bolt_spec,
                head_world_xyz=head_world,
                axis_world=axis_world,
                role=(
                    f"electronics_tray {label} board-mount "
                    f"({bolt_spec})"
                ),
                leg_index=None,
                joint=None,
                length_mm=bolt_len,
                cache_stl=f"{bolt_pn}.cache.stl",
                skip_screwdriver_reason=skip_reason,
            ))

            # Insert top face sits ``pilot_depth - insert_len`` mm
            # below the boss top (debris-clearance overdrill).  Axis
            # = +Z (the bolt comes DOWN from above; the insert was
            # pressed in with a soldering iron BEFORE the bolt was
            # threaded in, so the insert is captive in the boss).
            insert_top_z = boss_top_z - (pilot_depth - insert_len)
            insert_head_world = np.array([hx, hy, insert_top_z])
            out.append(FastenerInstance(
                part_number=insert_pn,
                spec=insert_spec,
                head_world_xyz=insert_head_world,
                axis_world=np.array([0.0, 0.0, -1.0]),
                role=(
                    f"electronics_tray {label} board-mount "
                    f"{insert_spec}"
                ),
                leg_index=None,
                joint=None,
                length_mm=insert_len,
                cache_stl=f"{insert_pn}.cache.stl",
                skip_screwdriver_reason=(
                    "heat-set insert installed with a soldering "
                    "iron BEFORE the board is bolted to the "
                    "electronics_tray (PROTOTYPE.md section 6.1 "
                    "step 12); no driver cone applies to the "
                    "brass insert"
                ),
            ))

    return out


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

    # Chassis-level fasteners (no leg index).  Battery_holder feet
    # are bolted to chassis_bottom from BELOW via 4 x M3 x 10 SHCS
    # into M3 heat-set inserts (May 2026 fix; previously the holder
    # was un-enumerated and physically un-bolted).
    out.extend(_emit_battery_holder_fasteners())

    # Electronics-tray board-mount fasteners (May 2026, single-tray
    # Mega 2560 + Pi 4 + PCA9685 layout).  4 M3 + 4 M2.5 + 4 M3
    # SHCS into matching heat-set inserts.  All SKIPped from the
    # screwdriver-access probe because they're captive sub-assembly
    # fasteners (driven before chassis_top closes the stack).
    out.extend(_emit_electronics_tray_fasteners())

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
        SPEC_M2X8_SHCS:             0,
        SPEC_M25X8_SHCS:            1,
        SPEC_M25X8_SHCS_INTO_INSERT: 2,
        SPEC_M25_HEATSET_INSERT:    3,
        SPEC_M3X8_SHCS:             4,
        SPEC_M3X8_SHCS_INTO_INSERT: 5,
        SPEC_M3X10_SHCS:            6,
        SPEC_M3_HEATSET_INSERT:     7,
        SPEC_M3X32_SHCS:            8,
        SPEC_M3X16_PAN:             9,
        SPEC_M3_NYLOC:             10,
    }
    rows.sort(key=lambda r: (spec_order.get(r[0], 99), r[0]))
    return rows


def _usage_bucket(fi: FastenerInstance) -> str:
    role = fi.role
    if "electronics_tray" in role:
        if "heat-set insert" in role:
            return "electronics_tray heat-set inserts (Mega + Pi + PCA9685)"
        return ("electronics_tray board-mount bolts "
                "(Mega M3 + Pi M2.5 + PCA9685 M3 SHCS into inserts)")
    if "battery_holder" in role:
        if "heat-set insert" in role:
            return "battery_holder heat-set inserts"
        return "battery_holder foot bolts (M3x10 SHCS into heat-set insert)"
    if "heat-set insert" in role:
        # The 72 brass heat-set inserts (McMaster 94459A130) pressed
        # into the cradle bosses BEFORE the cradle servo is mounted.
        return "cradle heat-set inserts"
    if "cradle" in role:
        # The 72 vertical M3 x 8 SHCS that thread DOWN through each
        # servo ear into the brass heat-set insert in the cradle
        # boss below (May 2026 heat-set switch -- see hexapod_
        # prototype.INSERT_M3_* / CRADLE_BOSS_*).
        return "cradle servo mounts (M3 SHCS into heat-set insert)"
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
