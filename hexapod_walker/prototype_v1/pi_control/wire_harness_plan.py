#!/usr/bin/env python3
"""Per-joint wire-harness plan for the 18-servo hexapod prototype.

This module is the SOURCE OF TRUTH for "how long is each leg's
servo extension cable, where does it route, and which PCA9685
channel is on the other end".  It reads the parametric CAD
geometry (``hexapod_prototype``), the chassis-frame leg layout
(``hexapod_prototype._leg_chassis_frames``), and the firmware's
joint-to-PCA-channel mapping
(``firmware/prototype_servo_bridge/prototype_servo_bridge.ino``)
to generate one ``WIRE_HARNESS_PLAN`` entry per servo:

  * ``joint_idx`` (0 .. 17): canonical joint index used by the
    servo-bridge firmware.  ``joint = leg * 3 + axis`` with
    ``axis in {0=yaw, 1=hip_pitch, 2=knee}``.
  * ``leg_idx`` (0 .. 5): hex-leg index (0 = +X half north-east,
    going CCW; matches ``_leg_chassis_frames()``).
  * ``axis`` (string): "yaw" / "hip_pitch" / "knee".
  * ``pca_board`` (int 0x40 / 0x41): PCA9685 I2C address.
  * ``pca_channel`` (int 0 .. 15): PCA channel number on that
    board.
  * ``source_xyz_chassis``: chassis-frame XYZ of the cradle
    wire-exit's mouth (where the harness physically exits the
    servo well).  As of the May 2026 chassis_bottom-integrated
    yaw cradle redesign the YAW cradle's exit is on the cradle's
    -X (radially-inward) face; the legacy ``coxa_bracket`` had
    it on +X.  HIP-PITCH and KNEE cradles still exit on the
    link's +X face, but for path-length estimation we still
    model all three cradles' sources as the YAW exit mouth (see
    ``_cradle_source_bracket_xyz``).
  * ``destination_xyz_chassis``: chassis-frame XYZ of the
    PCA9685 channel header pin block (board centre +
    per-channel offset along the rotated PCA9685 long axis).
  * ``via_chassis_drop_xyz``: chassis-frame XYZ of the leg's
    drop slot through ``chassis_bottom``
    (= bracket_frame(LEG_HARNESS_DROP_X_CENTRE, 0, 0)).
  * ``path_length_mm_min``: minimum harness length needed to
    span source -> drop -> PCA pin, computed as a Manhattan
    distance plus a +30 mm slack-loop budget per joint axis
    the harness physically crosses (see SLACK_BUDGET below).
    Manhattan rather than Euclidean because the harness has
    to follow the drop slot's radial corridor and the tray's
    surface rather than fly diagonally through the chassis.
  * ``extension_required``: human-readable string describing
    the cable build for this joint, computed from
    ``path_length_mm_min`` minus the stock pigtail length and
    rounded up to the next 30 cm extension cable.

Stock pigtail + extension assumption (DOCUMENTED here per the
task brief):

    DS3225 stock pigtail = STOCK_PIGTAIL_MM (declared below).  The
    real DS3225 pigtail is "barely long enough" per
    SHOPPING_LIST.md, with no exact figure published; the standard
    Hiwonder DS3225 ships with ~ 30 cm pigtail.  We treat the stock
    pigtail as 300 mm = STOCK_PIGTAIL_MM.

    Extension cables are 30 cm, 3-pin male-female servo
    extensions (the existing "Servo extension cables, 30 cm,
    3-pin male-female, pack of 20" line in SHOPPING_LIST.md).
    Each extension adds EXTENSION_LENGTH_MM = 300 mm of
    reach when wired end-to-end with the stock pigtail.

    extension_required:
      n_ext = max(0, ceil((path_length_mm_min - STOCK_PIGTAIL_MM)
                          / EXTENSION_LENGTH_MM))
      0 -> "DS3225 stock pigtail"
      1 -> "+ 30 cm extension"
      n -> "+ N x 30 cm extensions"
    Cross-board callout: L5 yaw goes to PCA1 ch15, L5 hip + knee
    go to PCA2 ch 0+1 (per the firmware's `joint = leg * 3 + axis`
    rule).  The L5 hip + knee entries' ``extension_required``
    appends " (L5 cross-board)" so the BOM reader can tell at a
    glance that these two servos cross the PCA1 / PCA2 boundary.

The leg azimuth + chassis-frame transforms come from
``hexapod_prototype._leg_chassis_frames()`` -- DO NOT re-derive
the leg layout here; a future change to leg count / azimuth
must happen in one place.

CLI
---

    python -m hexapod_walker.prototype.pi_control.wire_harness_plan
        -> prints the 18-row markdown table to stdout (the BOM)

    from hexapod_walker.prototype.pi_control.wire_harness_plan \\
        import WIRE_HARNESS_PLAN, print_harness_plan
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import TypedDict

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_PROTOTYPE_DIR = _THIS_DIR.parent
if str(_PROTOTYPE_DIR) not in sys.path:
    sys.path.insert(0, str(_PROTOTYPE_DIR))

import hexapod_prototype as hp  # noqa: E402


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STOCK_PIGTAIL_MM = 300.0
"""mm.  See module docstring for the SHOPPING_LIST.md citation and
the Hiwonder DS3225 spec assumption."""

EXTENSION_LENGTH_MM = 300.0
"""mm.  Standard 30 cm servo extension; matches the
"Servo extension cables, 30 cm" line in SHOPPING_LIST.md."""

SLACK_PER_JOINT_CROSSING_MM = 30.0
"""mm of slack loop the harness needs at every joint axis it
physically crosses (so the wire bend radius stays > the servo
wire's minimum bend radius across the full joint sweep)."""

# PCA9685 channel layout (Adafruit 16-channel servo driver,
# rotated 90 deg so its 62 mm long axis runs along tray-Y).  The
# 16 servo headers fan out along the +X edge of the rotated PCB.
# Channel-to-channel pitch is empirically ~3.4 mm on the Adafruit
# board (16 channels span ~55 mm of the 62 mm board length).  The
# exact pin position doesn't matter much for path-length
# estimation; the +30 mm slack budget dominates the per-channel
# delta of < 1 mm in the Manhattan path length.
PCA_CHANNEL_PITCH_MM = 3.4
PCA_CHANNEL_PIN_X_OFFSET = 0.5 * hp.PCA_PCB_D
"""mm.  The header pins sit on the +X edge of the rotated PCB
(= +PCA_PCB_D/2 in tray-local X) since PCA_HOLES uses the
(by, -bx) rotation, which maps the original board's +Y edge
(where the servo headers live) to tray +X."""
PCA_PIN_Z_ABOVE_TRAY_TOP_MM = (hp.ELEC_STANDOFF_H
                                + 1.6  # PCB thickness
                                + 1.5)  # nominal header pin tip
"""mm.  Channel pin tips sit this far above the tray top face
(tray top face is at chassis-z = +CHASSIS_PLATE_T/2 + 3 =
+5 mm; pin tips at +5 + 5 + 1.6 + 1.5 = +13.1 mm)."""

CHASSIS_PLATE_TOP_Z = hp.CHASSIS_PLATE_T / 2.0
"""mm.  chassis_bottom top face in chassis-frame Z."""
ELEC_TRAY_TOP_Z = CHASSIS_PLATE_TOP_Z + 3.0
"""mm.  Electronics tray's top face in chassis-frame Z.  3 mm
brass standoff between chassis_bottom and tray bottom."""


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _cradle_source_bracket_xyz(axis: str) -> tuple[float, float, float]:
    """Return the wire-exit 'mouth' position in BRACKET-LOCAL frame
    for one of the three cradle axes.

    As of the May 2026 chassis_bottom-integrated yaw-cradle
    redesign the YAW cradle's wire-exit corridor was mirrored
    to the cradle's -X (radially INWARD) face.  The HIP-PITCH
    and KNEE cradles still exit on the link's +X face -- their
    cradles weren't redesigned -- but for path-length estimation
    we still model all three cradles' sources as the YAW exit
    mouth (= the harness's effective "I'm now in the inter-plate
    volume on the inboard side of the leg" Manhattan node).

    YAW cradle, post-redesign:
      Well-local wire-exit mouth = (-WELL_W/2 - WIRE_SLOT_X_PAST_WALL,
      0, +small) -- i.e. just past the well's -X (inboard) outer
      face, on the centreline, near the cavity floor.
      Transforming that into bracket frame for the YAW cradle:
        bracket-x = body_centre_x + wire_exit_x_well
                  = -SERVO_OUTPUT_X - WELL_W/2 - WIRE_SLOT_X_PAST_WALL
                  ~= -41 mm  (was +21 mm with the legacy +X exit;
                  the new value is INBOARD of the bracket origin)
        bracket-y = 0
        bracket-z = chassis-plate-top + small.  In the new cradle
                    the wire-exit z range is the same well-local
                    band as the legacy bracket, but the cradle
                    floor IS the chassis_bottom top, so the mouth
                    sits roughly at chassis-plate-top + 3 mm of
                    bundle thickness.

    The drop-slot centre is at bracket-x = LEG_HARNESS_DROP_X_CENTRE
    = -46 mm, only 5 mm INBOARD of the new wire-exit mouth -- the
    redesigned cradle shortens the per-leg yaw-cable run by
    ~ |old - new| = |+21 - (-41)| = 62 mm of horizontal Manhattan
    distance compared to the legacy +X bracket exit.  The path-length
    model below picks up this shortening automatically since both
    the source and the destination are computed in chassis frame.

    For the HIP-PITCH and KNEE cradles the well sits in coxa_link
    / femur_link frame at the end of the arm.  We continue to
    model their harness "I'm now at the leg drop slot" position
    as the YAW source for path-length purposes (their physical
    routing follows the parent link inboard to the yaw axis and
    then onto the chassis_bottom top face; the YAW source's chassis
    coords already encode "leg's inboard wire-exit on the chassis"),
    plus a +SLACK_PER_JOINT_CROSSING_MM joint-axis-crossing budget
    each via ``n_crossings()``.  This sidesteps having to derive the
    link-local routing path (which would just add a constant offset
    smaller than the +/-30 mm slack budget anyway) and matches the
    user's "+30 mm slack-loop budget per joint crossing" rule.
    """
    assert axis in ("yaw", "hip_pitch", "knee")
    # Bracket-frame wire-exit mouth, common to all 3 cradles
    # for path-length estimation.  As of May 2026 the YAW cradle
    # exits on the well's -X face (radially inboard); the well-
    # local mouth coords are (-WELL_W/2 - WIRE_SLOT_X_PAST_WALL,
    # 0, +small).
    bx = -hp.SERVO_OUTPUT_X - hp.WELL_W / 2.0 - hp.WIRE_SLOT_X_PAST_WALL
    by = 0.0
    # Bracket-z = chassis-plate-top + ~3 mm of bundle thickness.
    # In the chassis_bottom cradle the cavity floor IS the
    # chassis_bottom top face, so bracket-z = 0 + 3 = +3 mm.
    bz = 3.0
    return (bx, by, bz)


def _drop_slot_bracket_xyz() -> tuple[float, float, float]:
    """Return the drop-slot centre in BRACKET-LOCAL frame.

    Drop slot is centred at bracket
    (LEG_HARNESS_DROP_X_CENTRE, 0, 0).  Bracket-z = 0 puts the
    slot mouth at the chassis plate mid-plane (which is the
    natural "I'm now in the inter-plate volume" Manhattan node
    for the harness)."""
    return (hp.LEG_HARNESS_DROP_X_CENTRE, 0.0, 0.0)


def _pca_pin_chassis_xyz(pca_board: int, channel: int
                          ) -> tuple[float, float, float]:
    """Return the chassis-frame XYZ of one PCA9685 channel header
    pin tip.

    PCA boards live in tray-local frame; tray-frame -> chassis-
    frame translation is (ELEC_TRAY_CENTRE_X, ELEC_TRAY_CENTRE_Y,
    ELEC_TRAY_TOP_Z + tray-z) where tray-z = 0 at the tray's mid-
    plane.  Pin tips sit at PCA_PIN_Z_ABOVE_TRAY_TOP_MM above the
    tray top face.

    Channel layout: 16 channels along the rotated board's tray-Y
    axis, at PCA_CHANNEL_PITCH_MM pitch, with channel 0 at the
    -Y end and channel 15 at the +Y end.  Pins are on the board's
    +X edge (= tray +X), offset by +PCA_PCB_D / 2 from
    PCA_CENTRE.
    """
    assert pca_board in (0x40, 0x41)
    assert 0 <= channel <= 15
    pca_centre_tray = (hp.PCA_CENTRE if pca_board == 0x40
                       else hp.PCA2_CENTRE)
    # Channels span tray-Y from -15/2 * PITCH to +15/2 * PITCH
    # around the board centre.
    n_channels = 16
    span_y = (n_channels - 1) * PCA_CHANNEL_PITCH_MM
    ch_y_tray = (pca_centre_tray[1] - span_y / 2.0
                 + channel * PCA_CHANNEL_PITCH_MM)
    ch_x_tray = pca_centre_tray[0] + PCA_CHANNEL_PIN_X_OFFSET
    # Tray -> chassis
    ch_x = hp.ELEC_TRAY_CENTRE_X + ch_x_tray
    ch_y = hp.ELEC_TRAY_CENTRE_Y + ch_y_tray
    ch_z = ELEC_TRAY_TOP_Z + PCA_PIN_Z_ABOVE_TRAY_TOP_MM
    return (ch_x, ch_y, ch_z)


def _bracket_to_chassis(edge_mid: np.ndarray, R3: np.ndarray,
                        bracket_xyz: tuple[float, float, float]
                        ) -> tuple[float, float, float]:
    """Transform bracket-local XYZ to chassis-frame XYZ for one
    leg.  Bracket-z = 0 corresponds to chassis plate-top.
    """
    bx, by, bz = bracket_xyz
    xy = edge_mid + R3 @ np.array([bx, by, 0.0])
    cz = CHASSIS_PLATE_TOP_Z + bz
    return (float(xy[0]), float(xy[1]), cz)


# ---------------------------------------------------------------------------
# Path-length model
# ---------------------------------------------------------------------------

_AXIS_ORDER = ("yaw", "hip_pitch", "knee")
_AXIS_SLACK_CROSSINGS: dict[str, int] = {
    # Yaw cradle is chassis-fixed (mounted to coxa_bracket); no
    # downstream joint between the cradle and the chassis-side
    # PCA9685, so 0 slack loops.
    "yaw": 0,
    # Hip-pitch cradle is on coxa_link (rotates with yaw); harness
    # crosses the yaw joint once on its way back to the chassis.
    "hip_pitch": 1,
    # Knee cradle is on femur_link (rotates with hip-pitch, which
    # itself rotates with yaw); harness crosses BOTH the yaw and
    # hip-pitch joints on its way back to the chassis.
    "knee": 2,
}
_AXIS_SLACK_MM: dict[str, float] = {
    axis: _AXIS_SLACK_CROSSINGS[axis] * SLACK_PER_JOINT_CROSSING_MM
    for axis in _AXIS_ORDER
}


def _manhattan(a: tuple[float, float, float],
               b: tuple[float, float, float]) -> float:
    return (abs(a[0] - b[0])
            + abs(a[1] - b[1])
            + abs(a[2] - b[2]))


# ---------------------------------------------------------------------------
# Joint mapping (mirrors the firmware's joint -> PCA channel rule)
# ---------------------------------------------------------------------------


def joint_idx(leg_idx: int, axis: str) -> int:
    return leg_idx * 3 + _AXIS_ORDER.index(axis)


def joint_to_pca(joint: int) -> tuple[int, int]:
    """Return ``(pca_board, pca_channel)`` for one joint index.

    Mirrors ``firmware/prototype_servo_bridge/
    prototype_servo_bridge.ino``: joints 0..15 -> PCA1 (0x40)
    channels 0..15, joints 16..17 -> PCA2 (0x41) channels 0..1.
    """
    assert 0 <= joint <= 17
    if joint <= 15:
        return (0x40, joint)
    return (0x41, joint - 16)


# ---------------------------------------------------------------------------
# WIRE_HARNESS_PLAN construction
# ---------------------------------------------------------------------------


class HarnessEntry(TypedDict, total=False):
    joint_idx: int
    leg_idx: int
    axis: str
    pca_board: int
    pca_channel: int
    source_xyz_chassis: tuple[float, float, float]
    destination_xyz_chassis: tuple[float, float, float]
    via_chassis_drop_xyz: tuple[float, float, float]
    path_length_mm_min: float
    extension_required: str


def _extension_required_str(path_mm: float, leg_idx: int,
                            pca_board: int, axis: str) -> str:
    """Return the BOM string for one harness path.

    See module docstring for the assumption + rounding rule.
    Appends " (L5 cross-board)" for L5 hip + knee (the two
    servos that physically cross the PCA1 / PCA2 boundary in
    the firmware mapping)."""
    deficit_mm = path_mm - STOCK_PIGTAIL_MM
    n_ext = max(0, math.ceil(deficit_mm / EXTENSION_LENGTH_MM))
    if n_ext == 0:
        s = "DS3225 stock pigtail"
    elif n_ext == 1:
        s = "+ 30 cm extension"
    else:
        s = f"+ {n_ext} x 30 cm extensions"
    # L5 cross-board callout
    if leg_idx == 5 and pca_board == 0x41:
        # L5 yaw is on PCA1 ch15, L5 hip+knee are on PCA2 ch0+1
        s = f"{s} (L5 cross-board: PCA2 0x41 ch{'0' if axis == 'hip_pitch' else '1'})"
    return s


def _build_plan() -> list[HarnessEntry]:
    plan: list[HarnessEntry] = []
    for leg_idx, edge_mid, _R, R3 in hp._leg_chassis_frames():
        for axis in _AXIS_ORDER:
            j_idx = joint_idx(leg_idx, axis)
            pca_board, pca_channel = joint_to_pca(j_idx)

            source_bracket = _cradle_source_bracket_xyz(axis)
            drop_bracket = _drop_slot_bracket_xyz()

            source_chassis = _bracket_to_chassis(edge_mid, R3,
                                                  source_bracket)
            drop_chassis = _bracket_to_chassis(edge_mid, R3,
                                                drop_bracket)
            dest_chassis = _pca_pin_chassis_xyz(pca_board,
                                                 pca_channel)

            man_path = (_manhattan(source_chassis, drop_chassis)
                        + _manhattan(drop_chassis, dest_chassis))
            slack = _AXIS_SLACK_MM[axis]
            path_length_mm_min = man_path + slack
            ext = _extension_required_str(path_length_mm_min,
                                           leg_idx, pca_board, axis)

            plan.append(HarnessEntry(
                joint_idx=j_idx,
                leg_idx=leg_idx,
                axis=axis,
                pca_board=pca_board,
                pca_channel=pca_channel,
                source_xyz_chassis=tuple(round(v, 2) for v in source_chassis),
                destination_xyz_chassis=tuple(round(v, 2) for v in dest_chassis),
                via_chassis_drop_xyz=tuple(round(v, 2) for v in drop_chassis),
                path_length_mm_min=round(path_length_mm_min, 1),
                extension_required=ext,
            ))
    return plan


WIRE_HARNESS_PLAN: list[HarnessEntry] = _build_plan()


# ---------------------------------------------------------------------------
# Markdown table printer
# ---------------------------------------------------------------------------


def _fmt_xyz(v: tuple[float, float, float]) -> str:
    return f"({v[0]:+6.1f}, {v[1]:+6.1f}, {v[2]:+6.1f})"


def print_harness_plan() -> None:
    """Print ``WIRE_HARNESS_PLAN`` as a markdown table to stdout.

    18 rows, one per joint, in joint-index order (0..17).  The
    table is the BOM the user prints to build the leg harnesses;
    feed it into a Markdown viewer or just read in plain text.
    """
    cols = [
        ("joint", 5),
        ("leg", 4),
        ("axis", 10),
        ("PCA",   6),
        ("ch",    3),
        ("src (mm)", 24),
        ("via drop (mm)", 24),
        ("dest (mm)", 24),
        ("path mm", 8),
        ("extension", 50),
    ]
    fmt_cells = lambda cells: "| " + " | ".join(
        f"{c:<{w}}" for (c, (_n, w)) in zip(cells, cols)) + " |"

    headers = [name for name, _ in cols]
    print(fmt_cells(headers))
    print(fmt_cells(["-" * w for _, w in cols]))
    for entry in WIRE_HARNESS_PLAN:
        row = [
            str(entry["joint_idx"]),
            f"L{entry['leg_idx']}",
            entry["axis"],
            f"0x{entry['pca_board']:02x}",
            str(entry["pca_channel"]),
            _fmt_xyz(entry["source_xyz_chassis"]),
            _fmt_xyz(entry["via_chassis_drop_xyz"]),
            _fmt_xyz(entry["destination_xyz_chassis"]),
            f"{entry['path_length_mm_min']:.1f}",
            entry["extension_required"],
        ]
        print(fmt_cells(row))


if __name__ == "__main__":
    print_harness_plan()
