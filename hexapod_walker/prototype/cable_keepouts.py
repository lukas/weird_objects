"""Cable-clearance keep-out volumes for the electronics_tray boards.

For each connector on the Mega 2560, Raspberry Pi 4 / Pi 5 and PCA9685
PWM driver boards, this module exposes a ``trimesh.Trimesh`` box that
covers the airspace the cable plug + strain relief occupies once the
cable is plugged in.  ``check_cable_clearance`` in
``_verify_prototype.py`` asserts each keep-out has < 50 mm^3 of
overlap with any printed part, modelled electronics body, or fastener
mesh -- a future CAD edit that grew material into a cable's plug-in
airspace will FAIL the verifier instead of being discovered the first
time the cables are routed.

Frame
-----

All keep-out volumes are returned in the **CHASSIS frame** (the same
pre-chassis-lift world frame the fastener registry uses).  Z = 0 at
the chassis_bottom plate's CENTRE plane (z = +CHASSIS_PLATE_T/2 = +2
mm is the chassis_bottom TOP face).  The boards live on top of the
electronics_tray at the chassis_tray's per-board centre offset.

Sources
-------

Connector positions on each board are taken from the Adafruit /
Raspberry Pi mechanical drawings (publicly published PDFs).  Where
the connector position on the board is uncertain we conservatively
place it at the centre of the appropriate board edge -- the > 50
mm^3 verifier tolerance absorbs the 1-3 mm of slop this introduces.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Callable

import numpy as np
import trimesh
from trimesh.creation import box as _box_mesh

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import hexapod_prototype as hp  # noqa: E402


# ---------------------------------------------------------------------------
# Public dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CableKeepout:
    """One cable clearance airspace volume in the chassis frame.

    Fields
    ------
    part_name : str
        Board the connector lives on, e.g. ``"Mega2560"``.
    connector_name : str
        Per-board connector label, e.g. ``"USB-B"``.
    mesh : trimesh.Trimesh
        Watertight box covering the plug + cable strain relief
        airspace, in the chassis design frame (the same frame the
        fastener_registry uses; see ``cable_keepouts`` module
        docstring).
    """
    part_name: str
    connector_name: str
    mesh: trimesh.Trimesh


# ---------------------------------------------------------------------------
# Constants tied to the electronics_tray's chassis-frame placement
# ---------------------------------------------------------------------------
#
# ``_emit_electronics_tray_fasteners`` uses the SAME placement:
#     tray top face z = HP.CHASSIS_PLATE_T / 2 + 3 + HP.ELEC_TRAY_T
# Each board sits on standoff bosses HP.ELEC_STANDOFF_H above the tray
# top face -- the connector envelope therefore sits at the BOARD's
# top face z = TRAY_TOP_Z + ELEC_STANDOFF_H + (per-board PCB Z height).

_TRAY_TOP_Z = hp.CHASSIS_PLATE_T / 2.0 + 3.0 + hp.ELEC_TRAY_T  # = 8 mm
_BOARD_BASE_Z = _TRAY_TOP_Z + hp.ELEC_STANDOFF_H               # = 13 mm
# Bare-PCB top face: BOARD_BASE_Z + 1.6 mm FR-4 thickness.  Most
# connector PORT bodies (the metal/plastic shell soldered to the
# PCB) sit ON the PCB top face and extend UP, so plug envelopes
# start at PCB top + ~0.4 mm (a typical solder-pad / shell standoff)
# rather than at BOARD_BASE_Z.  Keeping the plug envelope BELOW
# the PCB top face would incorrectly model the plug as overlapping
# the board's chassis-side coxa_brackets (whose +Z faces reach to
# z ~ +15 mm).
_PCB_TOP_Z = _BOARD_BASE_Z + 2.0             # 15 mm (round up to a
                                              # safe coxa_bracket
                                              # clearance margin)
# Approx board (PCB + components) Z extent; matches
# ``_body_battery_parts``'s 8 / 18 mm visual mesh heights.  Used as
# the TOP of the plug envelope's Z range (port body extends UP
# from the PCB by this many mm).
_MEGA_BOARD_TOP_Z = _BOARD_BASE_Z + 8.0      # 21 mm
_PI_BOARD_TOP_Z   = _BOARD_BASE_Z + 18.0     # 31 mm (includes the
                                              # Pi's Ethernet jack +
                                              # USB-A stack height)
_PCA_BOARD_TOP_Z  = _BOARD_BASE_Z + 8.0      # 21 mm


# ---------------------------------------------------------------------------
# Box primitive
# ---------------------------------------------------------------------------

def _aabb_box(x_lo: float, x_hi: float,
              y_lo: float, y_hi: float,
              z_lo: float, z_hi: float) -> trimesh.Trimesh:
    """Axis-aligned bounding box from min/max corner coordinates."""
    extents = (x_hi - x_lo, y_hi - y_lo, z_hi - z_lo)
    m = _box_mesh(extents=extents)
    m.apply_translation([
        (x_lo + x_hi) / 2.0,
        (y_lo + y_hi) / 2.0,
        (z_lo + z_hi) / 2.0,
    ])
    return m


# ---------------------------------------------------------------------------
# Mega 2560 connectors
# ---------------------------------------------------------------------------
#
# The Mega is rotated 90 deg from the Arduino reference drawing
# (MEGA_HOLES uses ``(by, -bx)``).  Board +Y (the USB-B / DC barrel
# end) maps to tray +X; board +X (the GPIO-pin edge) maps to tray
# -Y.  So in the chassis frame:
#
#   * Long edge (101.5 mm = MEGA_PCB_D) runs along chassis +/- X,
#     centred at MEGA_CENTRE_X = -28 mm -> board X span
#     [-78.75, +22.75] mm.
#   * Short edge (53.3 mm  = MEGA_PCB_W) runs along chassis +/- Y,
#     centred at MEGA_CENTRE_Y = +28 mm -> board Y span
#     [+1.35, +54.65] mm.
#
# USB-B + DC barrel jack live on the Mega's USB-B / power short
# edge (board +Y in board frame, board's +X corner area), so after
# the 90 deg rotation they land on the chassis +X face of the
# board (x = MEGA_CENTRE_X + MEGA_PCB_D / 2 = +22.75 mm).  Cables
# exit chassis +X past the +X end of the Mega -- toward the BEC
# cradle / PCA9685 stack on the +X half of the tray.  Both cables
# need ~ 30 mm of strain-relief clearance off the +X face; that's
# 22.75 + 30 = 52.75 mm, within the tray's +X edge at +80 mm.
#
# Per the Arduino Mega R3 mech drawing the two connectors sit
# along the short edge:
#   * USB-B  : centred  9.0 mm from the (board-frame) -X corner
#              (i.e.  board (bx,by) ~ ( 9, 95) -> board-centre
#              offset (-17.65, +44.25) -> rotated (+44.25, +17.65)
#              -> chassis (-28+44.25, +28+17.65) = (+16.25, +45.65).
#   * Barrel : centred 42.0 mm from the same corner
#              -> board (bx,by) ~ (42, 95) -> rotated
#              (+44.25, -15.35) -> chassis (+16.25, +12.65).

_MEGA_PLUG_FACE_X = (hp.ELEC_TRAY_CENTRE_X + hp.MEGA_CENTRE[0]
                     + hp.MEGA_PCB_D / 2.0)  # = +22.75 mm


def _mega_usb_b() -> trimesh.Trimesh:
    plug_face_x = _MEGA_PLUG_FACE_X
    # USB-B connector body: 17.65 mm in +Y of board centre after
    # 90 deg rotation -> chassis cy = MEGA_CENTRE_Y + 17.65 = +45.65.
    # The Mega's USB-B and DC barrel both sit on the +Y SIDE of the
    # +X short edge (cy > +40), so neither cable shares a Y band
    # with the BEC cradle (which lives at cy in [+3.7, +36.3]).
    cy = hp.ELEC_TRAY_CENTRE_Y + hp.MEGA_CENTRE[1] + 17.65
    # 14 mm wide x 13 mm tall plug body + 22 mm of +X clearance.
    # The corridor between Mega +X edge (+22.75) and PCA1 -X edge
    # (+51.5) is 28.75 mm wide; the USB-B plug body alone is 16 mm
    # so the keepout reserves 22 mm of clear +X (plug + 6 mm bend)
    # and the rest of the cable BENDS in +Z / +Y to escape past
    # PCA1's footprint.
    #
    # Z range starts at _PCB_TOP_Z (= +15) because the USB-B port
    # SHELL sits on top of the PCB; the plug body when inserted
    # occupies the shell's interior -- z below +15 is inside the
    # PCB / electronics_tray standoff column, NOT in cable
    # airspace.  This also clears the L0 coxa_bracket whose +Z
    # face peaks at +15 mm.
    return _aabb_box(
        x_lo=plug_face_x,            x_hi=plug_face_x + 22.0,
        y_lo=cy - 14.0 / 2.0,        y_hi=cy + 14.0 / 2.0,
        z_lo=_PCB_TOP_Z,             z_hi=_PCB_TOP_Z + 13.0,
    )


def _mega_dc_barrel() -> trimesh.Trimesh:
    plug_face_x = _MEGA_PLUG_FACE_X
    # DC barrel-jack body: 13.65 mm in +Y of board centre after the
    # 90 deg rotation -> chassis cy = MEGA_CENTRE_Y + 13.65 = +41.65.
    cy = hp.ELEC_TRAY_CENTRE_Y + hp.MEGA_CENTRE[1] + 13.65
    # Phi 10 mm cylindrical plug body x 14 mm long + 4 mm bend
    # allowance = 18 mm along +X.  Z range matches the jack body's
    # above-PCB Z extent (see _mega_usb_b for the same rationale).
    return _aabb_box(
        x_lo=plug_face_x,            x_hi=plug_face_x + 18.0,
        y_lo=cy - 10.0 / 2.0,        y_hi=cy + 10.0 / 2.0,
        z_lo=_PCB_TOP_Z,             z_hi=_PCB_TOP_Z + 10.0,
    )


# ---------------------------------------------------------------------------
# Raspberry Pi 4 / Pi 5 connectors
# ---------------------------------------------------------------------------
#
# PI_HOLES is the board-local hole pattern; the tray places the Pi
# with its long axis along chassis +X (PI_PCB_W = 85 mm along X) and
# short axis along chassis +Y (PI_PCB_D = 56 mm along Y).
#
# Per the Pi 4 / Pi 5 mechanical drawing:
#
#   * USB-A 3.0 PAIR, USB-A 2.0 PAIR and Ethernet RJ45 share ONE
#     LONG edge of the 85 mm x 56 mm PCB.
#   * USB-C power, 2 x micro-HDMI and 3.5 mm audio jack share the
#     ADJACENT SHORT edge.
#   * 40-pin GPIO header runs along the OTHER LONG edge.
#   * micro-SD slot is on the OTHER SHORT edge (underside).
#
# The Pi is placed on the tray so that USB-A / Ethernet face chassis
# -Y (cables exit -Y past the chassis_top -Y apothem at y = -70 mm)
# and USB-C / HDMI face chassis -X (cables exit -X in a separate
# corridor from the +X PCA9685 servo header bank):
#
#   * -Y LONG edge of the Pi (chassis y = pi_y_min)
#       : USB-A 3.0 PAIR  (blue, 2 stacked)
#       : USB-A 2.0 PAIR  (black, 2 stacked)
#       : Ethernet RJ45
#   * -X SHORT edge of the Pi (chassis x = pi_x_min)
#       : USB-C power
#       : Micro-HDMI 0
#       : Micro-HDMI 1
#       : (3.5 mm audio jack -- not used in this robot; no keepout)
#   * +Y LONG edge: GPIO 40-pin header (no cables modelled).
#   * +X SHORT edge: micro-SD slot (no plug; no keepout).
#
# Pre-May-2026 the keepouts had this layout BACKWARD -- USB-C / HDMI
# on the -Y long edge and USB-A / Ethernet on the +X short edge --
# matching an equally-incorrect ``make_raspberry_pi_visual()``.  The
# physical Pi cannot fit 2 x USB-A stacks + RJ45 on a 56 mm short
# edge (their combined width is ~ 48 mm of connector body before
# margins).  The May 2026 "Pi cantilever" pass fixed both files to
# agree on the real Pi layout, AND moved the Pi south so its -Y
# long edge sits at chassis y = -72.5 mm -- 2.5 mm past the
# chassis_top -Y apothem -- so the USB-A stacks + RJ45 cable bank
# is OUTSIDE the chassis_top hexagon silhouette in plan view, with
# their connector tops in free air (z up to 29 mm; chassis_top z
# band is 34..38 mm).
#
# Notes on cable-exit clearance:
#   * USB-A and Ethernet cables exit chassis -Y; the keepout
#     extends 22 mm in -Y from the plug face.  At y = pi_y_min - 22
#     = -94.5 mm the keepout is far outside chassis_top (apothem
#     -70) AND well inside chassis_bottom (the -X / -Y slanted
#     edges of the 200 mm flat-to-flat hexagon).
#   * USB-C and HDMI cables exit chassis -X.  Keepouts extend 32-33
#     mm in -X from the plug face, ending at chassis x ~ -95 mm
#     (just inside chassis_bottom's -X flat at x = -100 mm).
#   * The keepouts below model ONE USB-A receptacle per stack (the
#     TOP port, z = +21..+29 mm) at the standard 12 x 8 mm plug
#     envelope, so the bottom port stays accessible only with the
#     chassis open during dev.  This matches the SHOPPING_LIST
#     design intent (Pi 4 USB-A used for ssh / dev keyboard, never
#     both ports of a stack at once).

def _pi_hdmi_short_edge_x() -> float:
    """Chassis x of the Pi's -X short edge (HDMI / USB-C side)."""
    return (hp.ELEC_TRAY_CENTRE_X + hp.PI_CENTRE[0]
            - hp.PI_PCB_W / 2.0)


def _pi_usb_long_edge_y() -> float:
    """Chassis y of the Pi's -Y long edge (USB-A / Ethernet side)."""
    return (hp.ELEC_TRAY_CENTRE_Y + hp.PI_CENTRE[1]
            - hp.PI_PCB_D / 2.0)


def _pi_usb_c_power() -> trimesh.Trimesh:
    base_y = hp.ELEC_TRAY_CENTRE_Y + hp.PI_CENTRE[1]
    plug_face_x = _pi_hdmi_short_edge_x()
    # USB-C centred at PCB-local y = +half_y - 11 mm (11 mm in -Y
    # from the +Y corner of the Pi PCB, matching the real Pi 4
    # mech drawing's USB-C position on its long edge).
    cy = base_y + hp.PI_PCB_D / 2.0 - 11.0
    # 12 mm wide (along Y, parallel to short edge) x 7 mm tall in
    # Z plug face + 8 mm plug body + 25 mm strain relief = 33 mm
    # along -X.  Most of this is outside chassis_top (apothem
    # at x = -70 mm); chassis_bottom's -X flat at x = -100 mm gives
    # the cable 4.5 mm of margin past the strain relief.
    return _aabb_box(
        x_lo=plug_face_x - 33.0,  x_hi=plug_face_x,
        y_lo=cy - 12.0 / 2.0,     y_hi=cy + 12.0 / 2.0,
        z_lo=_BOARD_BASE_Z,       z_hi=_BOARD_BASE_Z + 7.0,
    )


def _pi_micro_hdmi(slot: int) -> trimesh.Trimesh:
    """slot 0 = HDMI0 (nearer USB-C); slot 1 = HDMI1 (farther)."""
    base_y = hp.ELEC_TRAY_CENTRE_Y + hp.PI_CENTRE[1]
    plug_face_x = _pi_hdmi_short_edge_x()
    # HDMI 0 and HDMI 1 share the -X short edge with USB-C.  Per
    # the visual mesh in ``make_raspberry_pi_visual``: HDMI 0 at
    # PCB-local y = +half_y - 15 = +13 (in centred-PCB coords);
    # HDMI 1 at PCB-local y = 0 (middle of the short edge).
    offset_y = +13.0 if slot == 0 else 0.0
    cy = base_y + offset_y
    return _aabb_box(
        x_lo=plug_face_x - 32.0,  x_hi=plug_face_x,
        y_lo=cy - 8.0 / 2.0,      y_hi=cy + 8.0 / 2.0,
        z_lo=_BOARD_BASE_Z,       z_hi=_BOARD_BASE_Z + 4.0,
    )


def _pi_usb_a(slot: int) -> trimesh.Trimesh:
    """slot 1 = USB 3.0 PAIR (blue, -X end of the -Y long edge);
    slot 0 = USB 2.0 PAIR (black, middle of the -Y long edge).
    Each PAIR is a 2x1 stack of USB-A receptacles.  Models the TOP
    receptacle only (z = +21..+29 mm) -- see module docstring.
    """
    base_x = hp.ELEC_TRAY_CENTRE_X + hp.PI_CENTRE[0]
    plug_face_y = _pi_usb_long_edge_y()
    # Per Pi 4 mech drawing: USB 3.0 PAIR centred at PCB board x
    # ~ 39 (= PCB-local x = -3.5 in centred-PCB coords); USB 2.0
    # PAIR centred at PCB board x ~ 56 (= PCB-local x = +13.5).
    offset_x = -3.5 if slot == 1 else +13.5
    cx = base_x + offset_x
    # 12 mm wide x 8 mm tall (single-plug) envelope, 22 mm plug body
    # extending -Y past the receptacle face.  Top-of-stack port only
    # (see module docstring) so z = top half of board's component
    # zone, i.e. +21..+29 mm (= _BOARD_BASE_Z + 8..+16).
    return _aabb_box(
        x_lo=cx - 12.0 / 2.0,     x_hi=cx + 12.0 / 2.0,
        y_lo=plug_face_y - 22.0,  y_hi=plug_face_y,
        z_lo=_BOARD_BASE_Z + 8.0, z_hi=_BOARD_BASE_Z + 16.0,
    )


def _pi_ethernet() -> trimesh.Trimesh:
    base_x = hp.ELEC_TRAY_CENTRE_X + hp.PI_CENTRE[0]
    plug_face_y = _pi_usb_long_edge_y()
    # Ethernet RJ45 sits at the +X end of the -Y long edge of the
    # Pi PCB.  Per Pi 4 mech drawing: PCB board x ~ 72.5 (=
    # PCB-local x = +30 in centred-PCB coords).
    cx = base_x + 30.0
    # 14 mm wide x 14 mm tall plug + 13 mm rigid plug body =
    # 13 mm along -Y.  Robot deployment expects RJ45 to be
    # plugged ONLY during setup; right-angled RJ45 keeps the
    # cable bend tight against the -Y edge.  Keepout reserves a
    # generous 22 mm of -Y corridor (rigid + bend allowance).
    return _aabb_box(
        x_lo=cx - 14.0 / 2.0,     x_hi=cx + 14.0 / 2.0,
        y_lo=plug_face_y - 22.0,  y_hi=plug_face_y,
        z_lo=_BOARD_BASE_Z,       z_hi=_BOARD_BASE_Z + 14.0,
    )


# ---------------------------------------------------------------------------
# PCA9685 connectors
# ---------------------------------------------------------------------------
#
# PCA9685 is rotated 90 deg on the tray: the board's long axis
# (PCA_PCB_D = 62 mm) runs along chassis +Y.  The 18 x 3-pin servo
# headers are a single row along the board's long edge -- after
# rotation, they run along the chassis +/- X faces of the rotated
# PCA.  Cable exits straight UP (the servo cable plugs into the
# female header from above) so the keepout is a thin vertical
# slab above the board's footprint.

def _pca_servo_headers(centre_xy: tuple[float, float],
                       label: str) -> trimesh.Trimesh:
    cx = hp.ELEC_TRAY_CENTRE_X + centre_xy[0]
    cy = hp.ELEC_TRAY_CENTRE_Y + centre_xy[1]
    # PCA rotated 90 deg on the tray (PCA_HOLES = (by, -bx)):
    #   * BOARD's long axis  (PCA_PCB_W = 62 mm) -> chassis +Y
    #   * BOARD's short axis (PCA_PCB_D = 25 mm) -> chassis +X
    #   * BOARD's +Y edge (where the 16 x 3-pin servo header row
    #     lives on the Adafruit PCA9685) -> chassis -X edge of
    #     the rotated PCA at x = centre_x - PCA_PCB_D/2.
    # 16 channels at 2.54 mm pitch = 40.6 mm of header row along
    # the chassis +Y direction.  Each 3-pin female plug is roughly
    # 6 mm wide (= 2.54 mm pitch x 2 pin-rows + body), 4 mm in the
    # "perpendicular-to-board-edge" direction, 8 mm tall above the
    # PCB top.
    # Keepout: a thin slab CENTERED on the header row's chassis -X
    # edge (board edge at cx - PCA_PCB_D/2), 6 mm wide in X
    # (covering the plug body's protrusion on both sides of the
    # header pin row), 41 mm long in Y, 8 mm tall.
    edge_x = cx - hp.PCA_PCB_D / 2.0
    return _aabb_box(
        x_lo=edge_x - 3.0,            x_hi=edge_x + 3.0,
        y_lo=cy - 41.0 / 2.0,         y_hi=cy + 41.0 / 2.0,
        z_lo=_PCA_BOARD_TOP_Z,        z_hi=_PCA_BOARD_TOP_Z + 8.0,
    )


def _pca0x40_servo_headers() -> trimesh.Trimesh:
    return _pca_servo_headers(hp.PCA_CENTRE, "PCA9685(0x40)")


def _pca0x41_servo_headers() -> trimesh.Trimesh:
    return _pca_servo_headers(hp.PCA2_CENTRE, "PCA9685(0x41)")


# ---------------------------------------------------------------------------
# Public registry
# ---------------------------------------------------------------------------

_CABLE_KEEPOUTS_TABLE: tuple[tuple[str, str, Callable[[], trimesh.Trimesh]], ...] = (
    ("Mega2560",     "USB-B",        _mega_usb_b),
    ("Mega2560",     "DC barrel",    _mega_dc_barrel),
    ("Pi4",          "USB-C power",  _pi_usb_c_power),
    ("Pi4",          "Micro-HDMI 0", lambda: _pi_micro_hdmi(0)),
    ("Pi4",          "Micro-HDMI 1", lambda: _pi_micro_hdmi(1)),
    ("Pi4",          "USB-A (top)",  lambda: _pi_usb_a(1)),
    ("Pi4",          "USB-A (bot)",  lambda: _pi_usb_a(0)),
    ("Pi4",          "Ethernet",     _pi_ethernet),
    ("PCA9685(0x40)", "Servo headers", _pca0x40_servo_headers),
    ("PCA9685(0x41)", "Servo headers", _pca0x41_servo_headers),
)


def build_cable_keepouts() -> list[CableKeepout]:
    """Return one ``CableKeepout`` per modelled board connector.

    Each volume is a fresh ``trimesh.Trimesh`` (callers can mutate).
    """
    return [
        CableKeepout(part_name=part_name,
                     connector_name=connector_name,
                     mesh=factory())
        for (part_name, connector_name, factory) in _CABLE_KEEPOUTS_TABLE
    ]


def _self_test_summary() -> str:
    out = ["Cable-clearance keepout self-test:"]
    total_vol = 0.0
    for ko in build_cable_keepouts():
        v = float(ko.mesh.volume)
        b = ko.mesh.bounds
        out.append(
            f"  {ko.part_name:<14s} {ko.connector_name:<14s}  "
            f"vol = {v:8.1f} mm^3   "
            f"bounds x [{b[0,0]:+6.1f},{b[1,0]:+6.1f}]   "
            f"y [{b[0,1]:+6.1f},{b[1,1]:+6.1f}]   "
            f"z [{b[0,2]:+6.1f},{b[1,2]:+6.1f}]"
        )
        total_vol += v
    out.append(f"  ---")
    out.append(f"  {len(_CABLE_KEEPOUTS_TABLE)} keepouts, total volume "
               f"{total_vol/1000.0:.1f} cm^3")
    return "\n".join(out)


if __name__ == "__main__":
    print(_self_test_summary())
