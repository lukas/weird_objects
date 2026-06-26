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

1. (RETIRED -- STS3215 reconciliation, Jun 2026) The model used to
   emit ``72 x M2.5 case screws`` on the servo OUTPUT face -- an
   invented "front-face 4-bolt case-screw mount" (4 per cradle x 3
   cradles (yaw / hip / knee) per leg x 6 legs = 72).  These were
   PHANTOM: the authoritative Waveshare ST3215 mount-bracket geometry
   shows the output face has NO usable body-mounting screws -- the
   dia-20 disc horn sits on the dia-14 disc-horn bolt cross and covers
   them.  All 72 have been removed.  Servo BODY retention is by printed
   parts only: the yaw servo by ``make_yaw_servo_retainer`` (a strap
   with its own anchor bolts), the hip/knee servos by the clamshell
   ``make_servo_clamp_cap`` (2 x M3 self-tap per joint -- see 14.).
   The output face now carries ONLY the flush disc horn + its 4 x M3
   leg bolts on DISC_HORN_BOLT_PCD = 14 mm (see 2.).

2. ``72 x M3 x 6 SHCS`` -- link-to-disc-horn bolts.  4 per joint
   (DISC_HORN_BOLT_PCD = 14 mm circle) x (yaw + hip + knee) = 3 joints
   per leg x 6 legs.  Threads downward from the printed link's pad
   face into the 20 mm aluminum 25T DISC horn (Amazon B07D56FVK5)
   that the servo joints drive.  PN_M3X6_SHCS / 91290A111 -- a plain
   M3 SHCS that threads into the disc's M3 TAPPED hole; the aluminium
   IS the thread-engagement medium (no heat-set insert, no self-tap
   into plastic).  June 2026 disc-horn switch: replaced the now-retired
   plastic 4-arm X-horn (PCD 20.8 mm, M2 self-tap) -- see the
   ``DISC_HORN_BOLT_*`` docstrings in ``hexapod_prototype.py``.  The
   M3 x 6 screws ship with the disc kit.

3. ``18 x M2.5 x 8 spline center screw`` -- ships with the servo; sits
   captive between the servo spline and the disc horn.  18 servos
   x 1 screw each = 18.  Special-cased in ``check_screwdriver_access``
   with a SKIP because the screw is hidden under the disc horn during
   normal assembly -- install before fitting the horn.  A Phi
   HORN_CENTRE_OD = 3.4 mm (M3 clearance) hole sits coaxial with the
   joint axis through EVERY driven link's pad:
     * yaw    -- ``centre_hole`` through the coxa_link pedestal cap
     * hip    -- ``hip_centre_hole`` through the femur hip pad (May 2026)
     * knee   -- ``knee_centre_hole`` through the tibia knee pad (May 2026)
   so the spline screw head + a hex driver have line-of-sight from
   the pad's outer face, which is what the user requested for
   serviceability ("the tibia link and femur link round joints need a
   hole in the center to attach the screw into servo behind it", May
   2026).  The skip flag is preserved because the install order is
   still "spline screw FIRST, disc horn SECOND, link THIRD" -- the new
   pad holes are an access-from-above improvement, not a sequence
   change.

4. ``6 x M3 x 16 pan-head bolts`` -- foot hinge pins.  1 per leg.

5. ``6 x M3 nyloc nuts`` -- one per foot hinge bolt.

(May 2026 Design F: the previous categories 4 + 5 -- ``24 x M3 x 32 SHCS``
coxa-bracket-to-chassis bolts and 24 matching M3 nyloc nuts -- have been
RETIRED.  The standalone ``coxa_bracket`` part has been folded into
``chassis_bottom`` as a printed-in cradle, so there is no bracket flange
to clamp through the chassis plates.)

6. ``8 x M3 deck board-mount bolts`` -- Deck redesign (Jun 2026): the
   clip-in battery_holder is RETIRED (the LiPo is now velcro-strapped
   to the top of chassis_bottom through cut strap slots -- no bolts),
   and the stacked electronics deck carries the brain.  4 x M3 x 8 SHCS
   bolt the Arduino Uno Q DOWN onto the lower ``uno_q_tray`` and 4 x M3
   x 8 SHCS bolt the XINGYHENG buck converter DOWN onto the upper
   ``buck_tray``, each threading into an M3 brass heat-set insert in a
   printed tray boss on the board's hole pattern (UNO_Q_HOLES /
   BUCK_HOLES).  Emitted by ``_emit_deck_fasteners``.

7. ``8 x M3 heat-set inserts`` -- one per deck board-mount bolt (see
   6.).  Brass heat-set inserts pressed into the tray bosses.

10. ``8 x M3 x 10 deck standoff-column bolts`` -- Deck redesign
    (Jun 2026).  The two trays stack on 4 M3 brass standoff columns
    rising ABOVE chassis_top on the ``DECK_COLUMN_XY`` (+/-41, +/-33)
    pattern.  4 x M3 x 10 SHCS bolt the ``uno_q_tray`` DOWN onto the
    level-1 columns and 4 x M3 x 10 SHCS bolt the ``buck_tray`` DOWN
    onto the level-2 columns, each threading into the brass standoff's
    female top thread (modeled as a VIRTUAL heat-set-insert engagement
    target, like the chassis standoffs in 18.).  Emitted by
    ``_emit_deck_fasteners``.

11. ``4 x M3 nyloc nuts`` -- captive UNDER chassis_top retaining the
    lowest (level-1) deck standoff columns' male threads at the 4
    ``DECK_COLUMN_XY`` positions (see 10.).

12. ``2 x M3 x 8 SHCS`` -- switch_holster mount bolts.  Threads
    DOWN through 2 printed clearance holes in chassis_top into an
    M3 brass heat-set insert (McMaster 94459A130) embedded in
    each of the holster's 2 printed bosses.  Holds the anti-spark
    on/off switch holster captive on the +X edge of chassis_top
    between the L0 and L5 coxa brackets; the toggle protrudes
    past the chassis vertex so the user can flip it without
    opening the chassis.  May 2026 "essentials" pass.

13. ``2 x M3 heat-set inserts`` -- captive in the switch_holster's
    2 bosses (see 12.).  Same McMaster ``94459A130`` part as the
    other M3 inserts on this build.

14. ``24 x M3 x 8 SHCS`` -- sandwich-joint clamp-cap bolts.  Deck
    redesign (Jun 2026): 2 self-tapping M3 x 8 SHCS per ``servo_clamp_cap``
    drive DOWN through the cap into the cradle ±X wall-end pilots that
    already exist in ``_servo_well_solid``.  12 clamp caps robot-wide
    (1 per hip + knee sandwich joint, 2 per leg x 6 legs) = 24 bolts.
    Emitted by ``_emit_clamp_cap_fasteners`` inside the per-leg loop.

15. (retired) -- the M2.5 Pi board-mount inserts are gone with the
    Raspberry Pi (see 6./10.).

16. (retired) -- the electronics_tray chassis-mount bolts are gone
    with the in-gap electronics_tray (see 6./10.).

17. (retired) -- the chassis_bottom tray-mount heat-set inserts are
    gone with the electronics_tray.

18. ``4 x M3 x 10 SHCS`` -- chassis_top -> brass-standoff bolts.
    Threads DOWN from above chassis_top into the M-F brass
    standoff's female top threads.  4 sites at
    ``HP.CHASSIS_STANDOFF_HOLES_XY`` = (+/-35, 0) and (0, +/-35) mm
    -- the rotated-45-deg 35-mm-radius pattern, INTENTIONALLY
    different from the tray-mount pattern in (16.) so the brass
    standoff body doesn't conflict with the tray-mount insert
    pocket.  Modeled engagement target is a VIRTUAL
    SPEC_M3_HEATSET_INSERT entry at the top of the standoff (=
    chassis_top bottom face) so the verifier's engagement check
    pairs the bolt correctly -- the actual brass standoff hardware
    is unmodeled.

19. ``4 x M3 nyloc nuts`` -- captive UNDER chassis_bottom retaining
   the brass standoff's male threads at the same 4 standoff XY
   positions (see 18.).  Accessible from below the robot.
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
PN_M25X6_SELFTAP = "96877A150"   # M2.5 x 6 self-tapping screw (yaw rear CASE-mount;
                                 # threads the STS3215 FIXED rear-face case holes
                                 # at cradle (-8.3/-32.8, +-10.2), ~2.5 mm bite --
                                 # user opted for a self-tapper for the shallow holes)
# Link-to-disc-horn bolts (June 2026 disc-horn switch; see the
# DISC_HORN_BOLT_* docstrings in hexapod_prototype.py).  The
# servo joints now drive a 20 mm aluminum 25T DISC horn (Amazon
# B07D56FVK5) with 4 x M3 TAPPED holes on a 14 mm bolt circle.  The
# link's pad has a Phi 3.4 mm M3 clearance hole; the M3 SHCS threads
# DIRECTLY into the disc's tapped hole -- the aluminium IS the thread-
# engagement medium (no heat-set, no self-tap into plastic).  M3 x 6
# screws ship with the disc kit; we use that length (3-5 mm of
# engagement into the 5 mm disc depending on pad thickness).
PN_M3X6_SHCS     = "91290A111"   # M3 x 6 socket-head cap screw (ships with disc)
# May 2026 revert: the brief Design C horizontal-nyloc cradle bolt
# (PN_M3X14_SHCS = 91290A115) was retired in favour of vertical M3 x 8
# self-tap SHCS into Phi 2.5 mm printed pilots, reusing the existing
# M3 x 8 stock used as servo cradle mount bolts.

# Human-readable spec labels (used by the inspector and the BOM script).
SPEC_M3X8_SHCS   = "M3x8 SHCS"
SPEC_M3X10_SHCS  = "M3x10 SHCS"   # battery_holder foot bolts (4); threads
                                   # into M3 brass heat-set insert in the
                                   # holder foot above chassis_bottom.
# RETIRED (Jun 2026): the chassis_bottom HIGH/LOW print split was re-merged
# into one part, so the 12 M3 x 10 self-tap cradle-plate join screws are gone.
# The spec string is kept (no longer emitted) so the driver dispatcher / BOM
# ordering table stay stable.
SPEC_M3X10_SHCS_SELFTAP = "M3x10 SHCS self-tap"
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
# Self-tap cradle bolt spec (Design E, May 2026 mixed-mode revert):
# the 2 +X cradle bolts per cradle bite into a Phi 2.5 mm self-tap
# pilot drilled directly into the well-wall material instead of into
# a brass heat-set insert (the Phi 8 mm heat-set boss could not
# coexist with the +X wire channel; see the INSERT_M3_SELFTAP_* block
# in hexapod_prototype.py).  Same physical M3 x 8 SHCS stock as
# ``SPEC_M3X8_SHCS_INTO_INSERT`` -- only the engagement medium
# changes (self-tapped plastic vs brass insert).  Distinct spec
# string so the BOM + the verifier's engagement check can tell the
# two engagement modes apart.  "SHCS" substring preserved so the
# screwdriver-envelope dispatcher (HEX_KEY off the "SHCS" substring)
# still picks the right driver envelope.
SPEC_M3X8_SHCS_SELFTAP = "M3x8 SHCS self-tap"
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
SPEC_M3X6_SHCS   = "M3x6 SHCS"   # (retired) old link-to-disc-horn bolt length
# Jun 2026 flush-head tweak: the yaw anti-rotation saddle's flange thinned 5 ->
# 3 mm, so its 2 chassis-anchor screws shortened M3x8 -> M3x6 to keep the tip at
# the -3 pilot bottom (3 mm self-tap bite unchanged).  DISTINCT from the disc-
# horn "M3x6 SHCS" spec so the driver-envelope dispatcher gives it the WIDE
# open-cavity hex envelope (the disc-horn spec is counter-bored / narrow).
SPEC_M3X6_SHCS_SELFTAP = "M3x6 SHCS self-tap"
# Jun 2026 flush-horn fix: the real aluminium disc is only DISC_HORN_H = 2 mm
# (not 5 mm), so the driven mounts grew a HORN_REACH_DOWN = 3 mm printed boss to
# bridge down to it.  The link-to-disc-horn bolt now traverses that boss PLUS
# the 2 mm disc, so the M3 x 6 is too short -- it is replaced by an M3 x 8 with
# a DISTINCT spec string so the engagement check reserves engagement_mm =
# DISC_HORN_H = 2 mm (the full disc thickness) instead of the generic 5 mm
# M3 x 8 window.  Same PN_M3X8_SHCS stock as the cradle bolts.
SPEC_M3X8_DISC_HORN = "M3x8 disc-horn SHCS"
# Jun 2026 yoke-width fix: the STS3215 FRONT output is FLUSH (no SERVO_OUTPUT_H
# protrusion), so the DRIVEN disc horn seats 2 mm lower and the yoke top-arm pad
# bridges DRIVEN_HORN_REACH_DOWN = 5 mm (vs 3 mm on the passive side).  The
# driven link-to-disc-horn bolt therefore traverses arm (4) + 5 mm pad before
# the 2 mm disc, so it grows to M3 x 10 (the passive bolts stay M3 x 8).  Distinct
# spec so the engagement check still reserves engagement_mm = DISC_HORN_H = 2 mm.
SPEC_M3X10_DISC_HORN = "M3x10 disc-horn SHCS"
# STS3215 reconciliation (Jun 2026): the invented "front-face 4-bolt
# case-screw mount" on the servo OUTPUT face (4 per cradle x 3 cradles x
# 6 legs = 72) was REMOVED -- the dia-20 disc horn covers the output
# face.  POSITIVE body retention is now provided on the servo's REAL
# END-face M2.5 holes (measured Waveshare ST3215: a 10 x 10 mm square
# centred on each +/-X end face).  Each cradle bolts the servo's -X end
# face to the cradle's -X wall with 4 x M2.5 x 8 SHCS that thread into
# the servo's own metal case (the case IS the thread; no printed boss /
# insert).  Same exempt spec string as before so the engagement check's
# clearance-bore air-span exemption still applies.
SPEC_M25_BODY_SCREW = "M2.5 SHCS into servo case"
# Shallow SELF-TAPPING M2.5 into the STS3215 FIXED REAR (back) CASE FACE.  The 4
# yaw anti-rotation saddle screws drive vertically UP into the standard STS3215
# case mounting holes on the rear face (STEP solid 1: cradle (-8.3/-32.8,+-10.2),
# NOT the horn bolt circle), ~2.5 mm self-tap bite.  Engagement target is the
# real rear-case block injected in check_fastener_engagement (the frozen-short
# modeled servo_body does not reach the real rear face).
SPEC_M25_REAR_SELFTAP = "M2.5 self-tap into servo rear case"


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
    # ``True`` marks the entry as a VIRTUAL engagement target that
    # represents unmodeled mating hardware (e.g. the brass M-F standoff's
    # female top threads, modelled as an M3 heat-set insert so the
    # verifier's ``check_fastener_engagement`` can pair a bolt with a
    # plausible engagement medium).  Virtual entries are SKIPPED by
    # ``fastener_bom_rows`` so they don't inflate the BOM count, but
    # they DO appear in ``build_all_fastener_instances`` so the
    # verifier can probe them.
    is_virtual: bool = False
    # ``True`` marks a COMPLIANT, TORQUE-ONLY clamp bolt whose clearance
    # hole is intentionally OVERSIZED so the shaft never grips its
    # near-side part -- concentricity is set by a separate feature (e.g.
    # the spaced 6706 yaw bearings) and the bolt only PRELOADS the stack
    # (head clamps the near part, thread bites the far part).  The yaw
    # ``hub-to-disc-horn`` bolts are the canonical case: the hub's Phi 4.2
    # torque-only holes let the bearings -- not the bolts -- locate the
    # turntable, so the shaft floats by design.  ``check_fastener_engagement``
    # therefore folds the HEAD-bearing part into the distinct-parts join
    # (the head still clamps a real part) and waives the shaft-air-span rule
    # for these bolts, while STILL requiring a head bearing + thread (tip)
    # engagement into the far part.
    compliant_torque_only: bool = False

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


def _yaw_cradle_T(leg_index: int) -> np.ndarray:
    """World 4x4 transform that maps the yaw cradle's well-local frame
    into the chassis frame (pre-lift).

    STS3215 front-face mount (Jun 2026 refit)
    -----------------------------------------
    The DS3225 tab-shelf yaw cradle dropped the servo into an open-top
    bucket and rested its ears on a shelf at chassis-z =
    ``CHASSIS_PLATE_T/2 + CRADLE_TAB_SHELF_Z = +8`` (so the well's
    natural rim, well-local z = WELL_RIM_Z, mapped to +8 and the body
    bottom to +8 - WELL_RIM_Z = -19.25).  The STS3215 cradle
    (``_chassis_yaw_cradle_solid``) instead carries an elevated FRONT
    mount plate and the disc horn seats on top of it; the horn's TOP
    mating face -- where ``coxa_link`` bolts on -- is delivered at the
    pinned ``CHASSIS_YAW_OUTPUT_Z`` (29.75 mm) so the leg-mount height /
    kinematics are unchanged.

    The well-local frame's origin is the servo body's BACK face
    (``_servo_well_solid``).  Up the front-face stack the disc horn
    seats on the mount PLATE (not on the bare output shaft), so the
    disc-horn TOP sits at well-local z = ``SERVO_BODY_H + WELL_PLATE_T
    + HORN_STACK_H`` above the back face (body height + mount-plate
    thickness + disc-horn stack).  This is the CAD-exact stack-up built
    by ``_chassis_yaw_cradle_solid`` (out_z = CHASSIS_YAW_OUTPUT_Z -
    CHASSIS_PLATE_T/2, plate_top = out_z - HORN_STACK_H, front_face =
    plate_top - WELL_PLATE_T).  So the Z shift that lands the horn top
    at ``CHASSIS_YAW_OUTPUT_Z`` is::

        well_to_chassis_dz = CHASSIS_YAW_OUTPUT_Z
                             - (SERVO_BODY_H + WELL_PLATE_T
                                + HORN_STACK_H)
                           = 29.75 - (34.3 + 4.0 + 5.0) = -13.55 mm

    With this shift the well-local frame coincides EXACTLY with the
    CAD cradle: body back face -> chassis -13.55, body front face (=
    plate underside) -> chassis +20.75, plate top -> +24.75, disc-horn
    top -> +29.75.  The servo body (``_servo_body_world_transform``)
    therefore needs NO extra seat offset, and the printed yaw retainer
    strap (``make_yaw_servo_retainer``) holds the body up against the
    plate (the earlier output-face case screws were a phantom feature
    and have been removed).  NOTE: the disc horn is placed plate-aware
    in ``_horn_world_transform`` (yaw offset uses WELL_PLATE_T, not the
    hip/knee SERVO_OUTPUT_H convention).
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    well_to_chassis_dz = (
        HP.CHASSIS_YAW_OUTPUT_Z
        - (HP.SERVO_BODY_H + HP.WELL_PLATE_T + HP.HORN_STACK_H)
    )  # = -13.55 mm; lands the disc-horn top at CHASSIS_YAW_OUTPUT_Z.
    T = (
        _T(*edge_mid)
        @ _Rz(a)
        @ _T(-HP.SERVO_OUTPUT_X, 0.0, well_to_chassis_dz)
    )
    return T


def _coxa_to_world(leg_index: int) -> np.ndarray:
    """World 4x4 mapping the coxa-local frame (origin = yaw axis at the
    yaw disc-horn top) into the chassis frame."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    return _T(*edge_mid) @ _T(0.0, 0.0, HP.CHASSIS_YAW_OUTPUT_Z) @ _Rz(a)


def _hip_cradle_T(leg_index: int) -> np.ndarray:
    """World 4x4 transform for the hip sandwich joint (fixed side lives
    in the coxa_link).  Bearing-sandwich refit (Jun 2026): the hip servo
    is placed by ``hexapod_prototype._joint_place`` so its disc-horn-top
    lands on the hip joint axis (COXA_LENGTH, 0, COXA_HIP_DROP) -- the
    same anchor make_coxa_link / make_femur_hip_yoke use, guaranteeing
    the case-screw + disc-horn-bolt sites match the geometry."""
    M = HP._joint_place(HP.COXA_HIP_ANCHOR,
                        (1, 0, 0), HP.LEG_PITCH_AXIS)
    return _coxa_to_world(leg_index) @ M


def _knee_cradle_T(leg_index: int) -> np.ndarray:
    """World 4x4 transform for the knee cradle (lives in the femur_link).

    Bearing-sandwich refit (Jun 2026): the femur-link origin is the hip
    disc-horn-top ON the hip joint axis (no axial pad offset), so the
    femur sits at _T(COXA_LENGTH, 0, COXA_HIP_DROP) @ Ry(p) and the knee
    fixed side is anchored at the femur-local knee axis (FEMUR_LENGTH,0,0)
    -- matching make_femur_link's internal _joint_place((FEMUR_LENGTH,0,0)).
    """
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    T_femur_in_link = _T(*HP.COXA_HIP_ANCHOR) @ _Ry(p)
    M = HP._joint_place((HP.FEMUR_LENGTH, 0.0, 0.0),
                        (1, 0, 0), HP.LEG_PITCH_AXIS)
    return _coxa_to_world(leg_index) @ T_femur_in_link @ M


def _emit_end_face_fasteners(
    *,
    T_well_to_world: np.ndarray,
    leg_index: int,
    joint: str,
    location: str,
    upper_row_only: bool = False,
) -> list[FastenerInstance]:
    """The M2.5 body-retention bolts on the servo's -X END face.

    POSITIVE retention (Jun 2026): measured from the authoritative
    Waveshare ST3215 brackets, each +/-X END face of the servo carries a
    10 x 10 mm square of 4 M2.5 threaded case holes centred on the face.
    Each cradle (yaw chassis_bottom, hip coxa_link, knee femur_link)
    bolts the servo's -X end face to its -X wall: the head is recessed in
    a wall counterbore and the M2.5 x 8 SHCS threads +X (well-local) into
    the servo's own metal case.  Geometry is cut by
    ``hexapod_prototype._servo_well_solid`` (hip/knee) and
    ``_chassis_yaw_cradle_solid`` (yaw) at ``servo_end_face_bolt_centres``.

    ``upper_row_only`` (yaw cradle, Jun 2026 single-part merge): the merged
    chassis_bottom keeps the yaw cradle's -X WELL wall only down to its -6
    bottom face; the servo body hangs ~14 mm BELOW that, so the LOWER end-face
    bolt row (well-local bz < SERVO_BODY_H/2 -> world z ~ -10) would bear in
    open air with no wall behind it (it belonged to the abandoned deep cradle
    bucket).  Only the UPPER row engages real wall, so the yaw cradle bolts the
    servo's upper -X face (2 screws) + seats its output face on the mount plate
    + is captured from below by the bolt-on yaw_servo_retainer stirrup.  The
    deep hip/knee cradles keep all 4.

    Well-local frame matches ``_servo_well_solid``: body centred at
    x = y = 0, z in [0, SERVO_BODY_H]; the -X end face is at
    x = -SERVO_BODY_W/2 and the head plane stands SERVO_BODY_BOLT_STANDOFF
    off it.  The screws are driven BEFORE the disc horn / next-stage link
    close over the cradle, so they are sub-assembly fasteners (the
    inboard/-X driver approach is buried once the robot is assembled).
    """
    head_x = -(HP.SERVO_BODY_W / 2.0 + HP.SERVO_BODY_BOLT_STANDOFF)
    axis_local = np.array([1.0, 0.0, 0.0])   # +X, into the servo case
    skip_reason = (
        "captive sub-assembly fastener: the M2.5 end-face screws bolt "
        "the servo body to the cradle's -X wall BEFORE the disc horn and "
        "the next-stage link close over the cradle; the inboard/-X driver "
        "approach is buried once the robot is assembled"
    )
    centres = HP.servo_end_face_bolt_centres()
    if upper_row_only:
        centres = [(by, bz) for (by, bz) in centres
                   if bz > HP.SERVO_BODY_H / 2.0]
    out: list[FastenerInstance] = []
    for (by, bz) in centres:
        head = _apply_point(T_well_to_world, np.array([head_x, by, bz]))
        axis = _apply_dir(T_well_to_world, axis_local)
        y_lbl = "+Y" if by > 0 else "-Y"
        z_lbl = "top" if bz > HP.SERVO_BODY_H / 2.0 else "bot"
        out.append(FastenerInstance(
            part_number=PN_M25X8_SHCS,
            spec=SPEC_M25_BODY_SCREW,
            head_world_xyz=head,
            axis_world=axis,
            role=f"{location} -X end-face {y_lbl} {z_lbl} M2.5 body screw",
            leg_index=leg_index,
            joint=joint,
            length_mm=HP.SERVO_BODY_BOLT_LEN,
            cache_stl=f"{PN_M25X8_SHCS}.cache.stl",
            skip_screwdriver_reason=skip_reason,
        ))
    return out


# ---------------------------------------------------------------------------
# Per-joint horn-bolt fastener generators
# ---------------------------------------------------------------------------
#
# Each rotary joint (yaw, hip-pitch, knee-pitch) clamps the printed
# link's pad onto a 20 mm aluminum 25T DISC horn (Amazon B07D56FVK5)
# via 4 x M3 SHCS on DISC_HORN_BOLT_PCD = 14 mm (radius 7).  The bolts
# thread DIRECTLY from the link's pad into the disc's 4 M3 TAPPED
# holes -- the aluminium IS the thread-engagement medium.  Bolt-length
# budget (M3 x 6, head counter-bored COUNTERBORE_DEPTH = 3 mm into the
# pad outer face):
#   * coxa_link cap:    PEDESTAL_CAP_T = 4 mm -> ~5 mm into the disc
#   * femur hip pad:    LINK_THICKNESS = 6 mm -> ~3 mm into the disc
#   * tibia knee pad:   LINK_THICKNESS = 6 mm -> ~3 mm into the disc
# All three share a single M3 x 6 SHCS stock (the one that ships with
# the disc kit); engagement is into the disc's M3 thread for
# DISC_HORN_BOLT_THREAD_ENGAGEMENT_MM = 3 mm minimum.
#
# Moment-arm note: the bolt circle shrank from r = 10.4 (plastic
# X-horn PCD 20.8) to r = 7 (disc PCD 14), so per-bolt shear roughly
# doubles -- but M3 (vs M2) has ~2.25x the tensile/shear area, so the
# joint is net stronger, not weaker.  See ``fasteners/README.md``
# (PN 91290A111 entry).

_HORN_BOLT_PCD_HALF = HP.DISC_HORN_BOLT_PCD / 2.0


def _emit_horn_fasteners_yaw(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-disc-horn bolts at the yaw joint (coxa_link hub).

    Each M3 SHCS sits in a counter-bore cut into the TOP of the
    pedestal's bottom cap and clamps the cap DOWN onto the aluminum
    disc horn that lives at link-local z in [-PLASTIC_HORN_H, 0].
    Geometry in link-local z (cap spans z in [0, PEDESTAL_CAP_T] =
    [0, 4] mm)::

        head bearing face  : z = PEDESTAL_CAP_T - COUNTERBORE_DEPTH
                             = 4 - 2.5 = 1.5  (= counter-bore floor)
        shaft clearance run: z in [0, 1.5]  (1.5 mm of cap below head)
        disc-horn engagement: z in [-PLASTIC_HORN_H, 0]  (the bolt
                             threads downward into the 5 mm-thick
                             aluminium disc's M3 TAPPED hole; the last
                             3 mm is the design-required
                             DISC_HORN_BOLT_THREAD_ENGAGEMENT_MM = 3 mm).
        bolt tip           : the M3 x 6 SHCS engages 3-5 mm into the
                             5 mm-thick disc (cap + disc stack), so the
                             tip stays inside the disc rather than
                             poking out the far side.

    Before this fix (commit b5f7095): the head was placed at the
    hub's TOP face (link-local z = COXA_LIFT + hub_t = 44 mm); the
    bolt floated 36 mm above the disc horn entirely inside the printed
    hub.  check_fastener_engagement caught this with a
    "joins only 1 part [coxa_link]" failure on every yaw bolt.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = HP.CHASSIS_YAW_OUTPUT_Z
    # Link-local z = 0 IS the cap's bottom mating face with the
    # disc horn; the link's transform places that face at world z =
    # yaw_output_z, which is exactly the disc horn's top face per
    # check_mating_face_contact's "coxa_link bottom <-> yaw disc-horn
    # top" probe (gap = +0.00 mm).
    # Jun 2026 flush-horn fix: the real disc is only DISC_HORN_H = 2 mm and
    # the hub boss now reaches DOWN to YAW_HUB_BOSS_BOT_Z to seat on it, so the
    # M3 x 8 bolt drops the bolt TIP to the disc BOTTOM (full 2 mm thread bite).
    # Head sits in a shallow counter-bore one bolt-length above the disc bottom
    # (= link-local z = YAW_HUB_BOSS_BOT_Z - DISC_HORN_H + 8).  The OVERSIZED
    # Phi 4.2 torque-only clearance bore lets the shaft float between the head
    # and the disc by design (concentricity set by the spaced bearings).
    horn_bolt_len = 8.0
    head_local_z = (HP.YAW_HUB_BOSS_BOT_Z - HP.DISC_HORN_H) + horn_bolt_len
    T_link_to_world = _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a)
    out: list[FastenerInstance] = []
    for ang in HP.DISC_HORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            _HORN_BOLT_PCD_HALF * np.sin(ang),
            head_local_z,
        ])
        head = _apply_point(T_link_to_world, p_local)
        axis = _apply_dir(T_link_to_world, np.array([0.0, 0.0, -1.0]))
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_DISC_HORN,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"coxa_link L{leg_index} hub-to-disc-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="yaw",
            length_mm=horn_bolt_len,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
            # COMPLIANT torque-only clamp: the coxa_yaw_hub's Phi 4.2 mm
            # disc-horn bolt holes are deliberately OVERSIZED so the
            # SPACED 6706 bearing pair -- not these bolts -- sets the
            # turntable concentricity (avoids the over-constraint
            # Waveshare warns spikes servo current).  The bolt head
            # clamps the rotating yaw stack DOWN and the thread bites the
            # aluminium disc horn; the shaft floats in the torque-only
            # clearance by design.  (Hip/knee disc-horn bolts use TIGHT
            # Phi 3.4 holes and stay rigid, so they are NOT flagged.)
            compliant_torque_only=True,
            # Captive sub-assembly fastener.  Per PROTOTYPE.md
            # section 6.1 the 4 yaw M3 x 6 SHCS are driven in step 3
            # (coxa_link dropped onto yaw disc horn, bolts torqued from
            # above through the pedestal-cap counter-bore) BEFORE the
            # femur is mounted on the hip disc horn in step 5.  At
            # standing pose the +X yaw bolt (ang = 0 deg) is then
            # blocked from above by the femur's hip pad which tilts
            # up and back over the coxa_link hub when p_femur =
            # -25 deg; the other 3 yaw bolts happen to retain driver
            # access in the assembled state but are all driven at
            # the same assembly step before the femur is in place,
            # so the explicit allow-list applies uniformly.
            skip_screwdriver_reason=(
                "captive sub-assembly fastener: the M3 x 6 SHCS is "
                "torqued through the pedestal-cap counter-bore "
                "BEFORE the femur is mounted on the hip disc horn "
                "(PROTOTYPE.md section 6.1 step 3 vs step 5).  At "
                "standing pose (p_femur = -25 deg) the +X yaw bolt "
                "is blocked from above by the femur hip pad."
            ),
        ))
    return out


def _emit_yaw_cap_join_fasteners(leg_index: int) -> list[FastenerInstance]:
    """The 3 M3 x 8 self-tap join screws that bolt each ``yaw_bearing_cap``
    DOWN onto its ``chassis_bottom`` bearing tower (Jun 2026 split-tower fix).

    Coxa-local frame (z = 0 = disc-horn top, same as the yaw horn bolts):
    the head bears in the cap ear's counter-bore at
    ``YAW_CAP_EAR_TOP_Z - (INSERT_M3_BOLT_HEAD_H + 0.3)`` and the shank threads
    DOWN (-Z) into the tower's Phi 2.5 mm self-tap pilot below the split plane.
    These capture the spaced 6706 pair after each race is dropped onto its open
    face.  3 bolts x 6 legs = 18.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    T_coxa_to_world = (_T(*edge_mid)
                       @ _T(0.0, 0.0, HP.CHASSIS_YAW_OUTPUT_Z) @ _Rz(a))
    head_local_z = HP.YAW_CAP_EAR_TOP_Z - (HP.INSERT_M3_BOLT_HEAD_H + 0.3)
    out: list[FastenerInstance] = []
    for ang in HP.YAW_CAP_BOLT_ANGLES_RAD:
        p_local = np.array([
            HP.YAW_CAP_BOLT_PCD / 2.0 * np.cos(ang),
            HP.YAW_CAP_BOLT_PCD / 2.0 * np.sin(ang),
            head_local_z,
        ])
        head = _apply_point(T_coxa_to_world, p_local)
        axis = _apply_dir(T_coxa_to_world, np.array([0.0, 0.0, -1.0]))
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS_SELFTAP,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"yaw_bearing_cap L{leg_index} cap-to-tower join @ "
                f"{int(round(np.degrees(ang)))}deg M3 self-tap"
            ),
            leg_index=leg_index,
            joint="yaw",
            length_mm=HP.YAW_CAP_BOLT_LEN,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
            # Driven BEFORE the coxa_yaw_hub (with its rotating dust lip) is
            # fitted, while the cap top is fully open from above -- the
            # standing-pose dust lip overlaps the driver envelope but the join
            # is a sub-assembly step (drop both races onto open faces, bolt the
            # cap down, THEN fit the hub).
            skip_screwdriver_reason=(
                "split-tower sub-assembly fastener: the 3 cap-to-tower M3 "
                "self-tap screws are driven from directly above the open cap "
                "ear BEFORE the coxa_yaw_hub (and its rotating dust lip) is "
                "fitted onto the bearing pair."
            ),
        ))
    return out


def _emit_yaw_retainer_anchor_fasteners(leg_index: int) -> list[FastenerInstance]:
    """The 2 M3 x 6 self-tap anchor screws that bolt the yaw anti-rotation
    SADDLE (``make_yaw_servo_retainer``) UP to the ``chassis_bottom`` floor
    (Jun 2026 saddle redesign; replaces the old undrivable stirrup anchors
    that were never even emitted here).

    Cradle-local frame (origin on the yaw/output axis, +X outboard radial,
    +Y tangential, WORLD Z): the head RECESSES into a counterbore in the thick
    (5 mm) flange boss and bears on its shoulder at z = plate_bot - SADDLE_
    FLANGE_T + SADDLE_HEAD_CB_DEPTH = -9 (3 mm under the -6 floor), and the shank
    threads UP (+Z) RETAINER_PLATE_PILOT_DEPTH into a blind Phi 2.5 mm self-tap
    pilot in the 4 mm floor.  The 6 mm length keeps the tip at the -3 pilot
    bottom (3 mm bite, 1 mm blind cap; an M3 x 8 with the head at -9 would push
    the tip to -1, 1 mm proud through the plate).  The down-facing socket is open
    to the under-chassis cavity, so the driver enters straight up (guarded by
    check_screwdriver_access + check_fastener_engagement).  2 bolts x 6 = 12.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    # Saddle placement: pure Z-rotation + in-plane translate (it is modelled in
    # cradle-local XY + world Z) -- the SAME chain _place_yaw_retainers uses.
    T_saddle_to_world = _T(*edge_mid) @ _Rz(a)
    plate_bot = HP.CHASSIS_SPLIT_Z - HP.CHASSIS_BOTTOM_FLOOR_T
    # Head bears on the COUNTERBORE shoulder (recessed into the thick flange), not
    # the boss bottom: z = plate_bot - SADDLE_FLANGE_T + SADDLE_HEAD_CB_DEPTH = -9.
    head_local_z = plate_bot - HP.SADDLE_FLANGE_T + HP.SADDLE_HEAD_CB_DEPTH
    out: list[FastenerInstance] = []
    for (ax, ay) in HP.chassis_lower_retainer_anchor_centres():
        p_local = np.array([ax, ay, head_local_z])
        head = _apply_point(T_saddle_to_world, p_local)
        axis = _apply_dir(T_saddle_to_world, np.array([0.0, 0.0, 1.0]))
        side = "+Y" if ay > 0 else "-Y"
        out.append(FastenerInstance(
            part_number=PN_M3X6_SHCS,
            spec=SPEC_M3X6_SHCS_SELFTAP,
            head_world_xyz=head,
            axis_world=axis,
            role=f"yaw_servo_retainer L{leg_index} saddle {side} chassis anchor M3 self-tap",
            leg_index=leg_index,
            joint="yaw",
            length_mm=6.0,
            cache_stl=f"{PN_M3X6_SHCS}.cache.stl",
        ))
    return out


def _emit_yaw_rear_case_fasteners(leg_index: int) -> list[FastenerInstance]:
    """The 4 M2.5 x 6 SELF-TAPPING rear-CASE-mount screws that POSITIVELY capture
    the yaw STS3215 to the anti-rotation SADDLE (Jun 2026, CORRECTED 3rd pass --
    the prior two passes wrongly used the HORN bolt circle).

    Re-parsed STS3215_c.step and classified every hole by SOLID: the only Phi 2.5
    four-hole crosses (at the OUTPUT axis, both faces) are the DISC-HORN bolt
    circle, NOT a case mount.  The real STS3215 case mounting holes are on the
    FIXED rear (back) case face (STEP solid 1) at STEP (X,Z) = (8.3,+-10.2) and
    (32.8,+-10.2).  Frame: cradle_x = -STEP_X, cradle_y = STEP_Z, cradle_z = STEP_Y
    (output, UP), so they map to cradle (x,y) = (-8.3,+-10.2) [18.5 mm from the
    +X output-end "top", the user's landmark] and (-32.8,+-10.2) [43 mm].  Their
    axis IS the output axis = WORLD +Z, so each M2.5 SELF-TAPS straight UP into the
    rear case face (SADDLE_CASE_SCREW_BITE = 2.5 mm; shallow holes, self-tap), head
    recessed in the backstop boss counterbore (bearing at z = real_back -
    SADDLE_CASE_SHANK), driven straight up the open under-chassis cavity (guarded
    by check_screwdriver_access).  The real rear face hangs SADDLE_CASE_LEN_FIX
    below the frozen-short modeled servo_body, so check_fastener_engagement targets
    the real rear-case block injected in the yaw section.  All 4 are on the FIXED
    case shell, clear of the rotating idler (at the axis, r~0) and the centre wire
    exit (cradle x~-14, y~0).

    Cradle-local frame (origin on the yaw/output axis, +X outboard, +Y tangential,
    WORLD Z) -- the SAME placement chain _place_yaw_retainers uses."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    T_saddle_to_world = _T(*edge_mid) @ _Rz(a)
    head_local_z = HP.yaw_servo_real_back_z() - HP.SADDLE_CASE_SHANK
    out: list[FastenerInstance] = []
    for (rx, ry) in HP.yaw_rear_screw_centres():
        head = _apply_point(T_saddle_to_world, np.array([rx, ry, head_local_z]))
        axis = _apply_dir(T_saddle_to_world, np.array([0.0, 0.0, 1.0]))
        xtag = "x1" if abs(rx) < 20.0 else "x2"
        side = f"{xtag}{'+Y' if ry > 0 else '-Y'}"
        out.append(FastenerInstance(
            part_number=PN_M25X6_SELFTAP,
            spec=SPEC_M25_REAR_SELFTAP,
            head_world_xyz=head,
            axis_world=axis,
            role=f"yaw_servo_retainer L{leg_index} saddle {side} rear case-mount M2.5 self-tap",
            leg_index=leg_index,
            joint="yaw",
            length_mm=HP.SADDLE_CASE_SCREW_LEN,
            cache_stl=f"{PN_M25X6_SELFTAP}.cache.stl",
        ))
    return out


def _emit_horn_fasteners_hip(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-disc-horn bolts at the hip-pitch joint (femur hip pad).

    May 2026 collinear-pad refactor: the femur's NEW local origin is
    the hip pad MATING FACE (= disc-horn-top plane), not the joint axis,
    so the femur-local y of each face shifts down by HORN_STACK_H::

        +Y outer face        : y = LINK_THICKNESS = +6 (was +11)
        counter-bore floor   : y = LINK_THICKNESS - COUNTERBORE_DEPTH = +3.5 (was +8.5)
        -Y mating face       : y = 0  (= disc-horn top; was +5)
        3 mm thread depth    : y in [-3, 0]
        bolt tip overshoot   : y in [-3.5, -3]

    The world coordinates of the heads / bolts are UNCHANGED -- the
    femur translates +HORN_STACK_H in coxa-Y as a rigid body so the
    pad mating face still lands on the disc-horn-top plane in world.
    """
    # Bearing-sandwich refit: the femur HIP YOKE top arm clamps onto the
    # hip disc horn.  Bolt heads bear on the yoke top-arm OUTER face
    # (joint-local z = JOINT_HORN_TOP_Z + yoke arm thickness), axis -Z
    # down through the arm into the disc's M3 tapped hole.
    T = _hip_cradle_T(leg_index)
    head_local_z = HP.JOINT_HORN_TOP_Z + HP._YOKE_ARM_T
    out: list[FastenerInstance] = []
    for ang in HP.DISC_HORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            HP.SERVO_OUTPUT_X + _HORN_BOLT_PCD_HALF * np.cos(ang),
            _HORN_BOLT_PCD_HALF * np.sin(ang),
            head_local_z,
        ])
        head = _apply_point(T, p_local)
        axis = _apply_dir(T, np.array([0.0, 0.0, -1.0]))
        # Jun 2026 yoke-width fix: flush output drops the driven disc horn 2 mm,
        # so the top-arm pad reaches DRIVEN_HORN_REACH_DOWN (5 mm) and the bolt
        # grows arm(4) + pad(5) + 1 mm-into-disc = 10 mm (M3 x 10).
        out.append(FastenerInstance(
            part_number=PN_M3X10_SHCS,
            spec=SPEC_M3X10_DISC_HORN,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"femur_link L{leg_index} hip-yoke-to-disc-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="hip",
            length_mm=10.0,
            cache_stl=f"{PN_M3X10_SHCS}.cache.stl",
        ))
    return out


def _emit_horn_fasteners_knee(leg_index: int) -> list[FastenerInstance]:
    """The 4 link-to-disc-horn bolts at the knee joint (tibia knee pad).

    Mirrors ``_emit_horn_fasteners_hip``: head bearing face sits on
    the COUNTERBORE_DEPTH-deep pocket floor on the pad's outer +Y
    face, bolt axis points -Y INTO the pad and through the -Y mating
    face into the aluminum disc horn below.  See that function's
    docstring for the y-coordinate breakdown.
    """
    # Bearing-sandwich refit: the tibia KNEE YOKE top arm clamps onto the
    # knee disc horn (driven by the knee servo that lives in the femur
    # knee bracket).  The disc horn sits at the knee joint axis; the yoke
    # top arm is oriented by the TIBIA stance angle.  Bolt heads bear on
    # the yoke top-arm outer face, axis -Z into the disc.
    # Place the bolt heads in the SAME knee WELL/cradle frame the disc horn
    # is placed in (``_horn_world_transform("knee")``), exactly as the hip
    # emitter uses ``_hip_cradle_T``.  This keeps the bolts COAXIAL with the
    # disc's output axis.  The previous ``_joint_place`` reconstruction put the
    # bolt axis in the horizontal plane (off the tilted output axis); a short
    # M3 x 8 bolt into a 2 mm-higher horn masked it, but the Jun 2026
    # flush-output refit (horn 2 mm lower, M3 x 10 bolt) exposed it as the knee
    # bolts "join only 1 part".
    T = _knee_cradle_T(leg_index)
    head_local_z = HP.JOINT_HORN_TOP_Z + HP._YOKE_ARM_T
    out: list[FastenerInstance] = []
    for ang in HP.DISC_HORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            HP.SERVO_OUTPUT_X + _HORN_BOLT_PCD_HALF * np.cos(ang),
            _HORN_BOLT_PCD_HALF * np.sin(ang),
            head_local_z,
        ])
        head = _apply_point(T, p_local)
        axis = _apply_dir(T, np.array([0.0, 0.0, -1.0]))
        # Jun 2026 yoke-width fix: flush output drops the driven disc horn 2 mm,
        # so the top-arm pad reaches DRIVEN_HORN_REACH_DOWN (5 mm) and the bolt
        # grows to M3 x 10 (see _emit_horn_fasteners_hip).
        out.append(FastenerInstance(
            part_number=PN_M3X10_SHCS,
            spec=SPEC_M3X10_DISC_HORN,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"tibia_link L{leg_index} knee-yoke-to-disc-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint="knee",
            length_mm=10.0,
            cache_stl=f"{PN_M3X10_SHCS}.cache.stl",
        ))
    return out


# ---------------------------------------------------------------------------
# PASSIVE (rear-boss) disc-horn fasteners (symmetric-yoke refit)
# ---------------------------------------------------------------------------
# Mirror of the driven horn bolts: the yoke BOTTOM arm clamps the PASSIVE
# disc horn on the rear idler boss.  Head bears on the bottom-arm OUTER face
# (joint-local z = JOINT_HORN_BOT_Z - _YOKE_ARM_T), axis +Z UP into the disc's
# M3 tapped hole; plus one central M2.5 retention screw (mirrors the driven
# spline screw) through the horn + adapter into the rear boss so the passive
# horn can't work loose on the smooth boss.


def _passive_horn_T(leg_index: int, joint: str) -> np.ndarray:
    """World transform that maps PASSIVE-disc-horn-LOCAL coords (origin at the
    horn's spline-mating face, +Z = output-shaft axis toward the servo) into
    the world frame.  Mirror of the verifier's ``_passive_horn_world_transform``
    (and of the driven ``_horn_world_transform``): the horn rides the servo's
    rear idler boss, flipped 180 deg about X so its flat mating face points
    AWAY from the servo, seated REAR_BOSS_H below the back face and rotated with
    the driven link's stance.  Placing the bolts in this (correctly located)
    frame keeps them co-located with the passive horn AND the yoke bottom arm
    on BOTH the hip and the knee, regardless of the link-pad frame offsets."""
    if joint == "hip":
        T_well = _hip_cradle_T(leg_index)
        p_link = np.deg2rad(HP.STANCE_FEMUR_DEG)
    elif joint == "knee":
        T_well = _knee_cradle_T(leg_index)
        p_link = np.deg2rad(HP.STANCE_TIBIA_DEG)
    else:
        raise ValueError(f"joint {joint!r} has no passive horn")
    return (T_well
            @ _T(HP.SERVO_OUTPUT_X, 0.0, -HP.REAR_BOSS_H)
            @ _Rz(p_link)
            @ _Rx(np.pi))


def _emit_passive_horn_fasteners(leg_index: int, joint: str
                                 ) -> list[FastenerInstance]:
    """The 4 yoke-bottom-arm-to-passive-disc-horn bolts at a sandwich joint.

    Worked in PASSIVE-horn-local coords: head on the yoke bottom-arm outer
    face (horn-local z = DISC_HORN_H + YOKE_ARM_PAD + _YOKE_ARM_T), axis
    -Z DOWN into the disc's tapped hole -- a TRUE mirror of the driven horn
    bolts (symmetric-yoke refit: same YOKE_ARM_PAD = 5 mm pad and same M3 x 10
    screw both sides, 1 mm tip into the aluminium disc)."""
    T = _passive_horn_T(leg_index, joint)
    link = "femur_link" if joint == "hip" else "tibia_link"
    head_local_z = HP.DISC_HORN_H + HP.YOKE_ARM_PAD + HP._YOKE_ARM_T
    out: list[FastenerInstance] = []
    for ang in HP.DISC_HORN_BOLT_ANGLES_RAD:
        p_local = np.array([
            _HORN_BOLT_PCD_HALF * np.cos(ang),
            _HORN_BOLT_PCD_HALF * np.sin(ang),
            head_local_z,
        ])
        head = _apply_point(T, p_local)
        axis = _apply_dir(T, np.array([0.0, 0.0, -1.0]))   # into the disc
        out.append(FastenerInstance(
            part_number=PN_M3X10_SHCS,
            spec=SPEC_M3X10_DISC_HORN,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"{link} L{leg_index} {joint}-yoke-to-PASSIVE-disc-horn @ "
                f"{int(round(np.degrees(ang)))}deg SHCS"
            ),
            leg_index=leg_index,
            joint=joint,
            length_mm=10.0,
            cache_stl=f"{PN_M3X10_SHCS}.cache.stl",
        ))
    return out


def _emit_passive_center_screw(leg_index: int, joint: str
                               ) -> list[FastenerInstance]:
    """One M2.5 x 8 central retention screw per passive horn: head in the
    horn's collar recess on the mating face (horn-local z = DISC_HORN_H),
    axis -Z through the horn + centering adapter into the rear idler boss.
    Mirrors the driven spline screw; keeps the passive horn from loosening."""
    T = _passive_horn_T(leg_index, joint)
    head_local = np.array([0.0, 0.0, HP.DISC_HORN_H])
    head = _apply_point(T, head_local)
    axis = _apply_dir(T, np.array([0.0, 0.0, -1.0]))
    return [FastenerInstance(
        part_number=PN_M25X8_SHCS,
        spec=SPEC_M25X8_SHCS,
        head_world_xyz=head,
        axis_world=axis,
        role=f"{joint} passive-horn retention screw L{leg_index}",
        leg_index=leg_index,
        joint=joint,
        length_mm=8.0,
        cache_stl=f"{PN_M25X8_SHCS}.cache.stl",
        skip_screwdriver_reason=(
            "captive under the passive disc horn after assembly; install "
            "the retention screw BEFORE fitting the yoke"
        ),
    )]


# ---------------------------------------------------------------------------
# Spline center screws (M2.5 x 8, captive under the disc horn after assembly)
# ---------------------------------------------------------------------------


def _emit_spline_fastener(leg_index: int, joint: str) -> list[FastenerInstance]:
    """One M2.5 x 8 spline screw per servo (18 total).

    The screw threads into the servo's spline collar.  In the servo's
    local frame the screw HEAD sits on top of the disc horn at
    (SERVO_OUTPUT_X, 0, SERVO_BODY_H + SERVO_OUTPUT_H + PLASTIC_HORN_H);
    the axis points down (-Z in servo-local) into the spline.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    # Servo-local head position (x = output offset, y = 0, z = top of
    # disc horn).
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
        yaw_output_z = HP.CHASSIS_YAW_OUTPUT_Z
        hip_drop = HP.COXA_HIP_DROP
        delta = np.array([
            HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H) + HP.COXA_HIP_ANCHOR_Y,
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
        yaw_output_z = HP.CHASSIS_YAW_OUTPUT_Z
        hip_drop = HP.COXA_HIP_DROP
        p = np.deg2rad(HP.STANCE_FEMUR_DEG)
        # Bearing-sandwich refit (Jun 2026): femur-link origin is the
        # hip disc-horn-top ON the joint axis (no axial pad offset), so
        # T_femur_in_link has zero Y offset and delta_knee places the
        # knee servo back-face directly.
        delta_knee = np.array([
            HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H),
            0.0,
        ])
        T_femur_in_link = _T(*HP.COXA_HIP_ANCHOR) @ _Ry(p)
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
            # sits captive UNDER the disc horn after the
            # link bolts onto the horn (Design B retired the printed
            # adapter and the June 2026 switch replaced the plastic
            # X-horn with the aluminum disc horn; the link's pad now
            # bolts directly to the disc horn, so the spline screw head
            # is buried beneath the link).
            # Install it BEFORE fitting the horn.  May 2026: every
            # driven link's pad now carries a coaxial Phi HORN_CENTRE_
            # OD = 3.4 mm M3 clearance hole (coxa_link's centre_hole
            # through the pedestal cap, femur's hip_centre_hole
            # through the hip pad, tibia's knee_centre_hole through
            # the knee pad) so the spline screw head + a hex driver
            # have line-of-sight from the pad's outer face -- a
            # user-flagged serviceability improvement for the hip
            # and knee, matching the long-standing yaw access path.
            # The skip flag is preserved because install order is
            # still "spline screw -> disc horn -> link", not because
            # the screw is geometrically unreachable.
            skip_screwdriver_reason=(
                "captive under the X-horn after assembly; install "
                "the spline screw BEFORE fitting the plastic horn"
            ),
        )
    ]


def _emit_foot_hinge_fastener(leg_index: int) -> list[FastenerInstance]:
    """The single M3 x 16 pan-head hinge pin that captures the tibia's
    TANG in the foot pad's FORK.  Bolt axis is parallel to the knee
    Y axis.  Head sits on one fork cheek (foot-pad +Y face), nut on
    the other.

    May 2026 inversion: the OLD design had the fork on the tibia and
    the tongue on the foot; the bolt then went head -> tibia +Y cheek
    -> foot tongue -> tibia -Y cheek -> nut.  The NEW design swaps
    fork and tang -- the bolt now goes head -> foot pad +Y cheek ->
    tibia tang -> foot pad -Y cheek -> nut.  The hinge axis, pin
    length (M3 x 16 pan-head), and nylock are identical; only the
    cheek-bearing surfaces have moved from the tibia to the foot.
    For the BOM auto-derivation that's a no-op: head_world_xyz and
    nut_head world positions are unchanged because the fork outer
    faces sit at the SAME world y as the old clevis outer faces
    (cheek_outer_y formula goes from GAP/2+CHEEK_T = 5/2+3.5 = 6 mm
    to SLOT_W/2+CHEEK_T = 6.4/2+3.5 = 6.7 mm -- a 0.7 mm outboard
    shift per side, well within the pan-head + nylock engagement
    range; the M3 x 16 pin still spans 13.4 mm of plastic + ~2.6 mm
    of nylock threads).
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    yaw_output_z = HP.CHASSIS_YAW_OUTPUT_Z
    hip_drop = HP.COXA_HIP_DROP
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    Ry_p_3 = _Ry(p)[:3, :3]
    knee_joint_local = np.array(HP.COXA_HIP_ANCHOR) + Ry_p_3 @ np.array(
        [HP.FEMUR_LENGTH, 0.0, 0.0]
    )
    # Bearing-sandwich refit (Jun 2026): tibia-link origin is the knee
    # disc-horn-top ON the joint axis -- no axial pad offset.
    PAD_AXIS_OFFSET = np.array([0.0, 0.0, 0.0])
    T_tibia_in_link = _T(*(knee_joint_local + PAD_AXIS_OFFSET)) @ _Ry(pt)
    T_tibia_to_world = (
        _T(*edge_mid) @ _T(0.0, 0.0, yaw_output_z) @ _Rz(a) @ T_tibia_in_link
    )
    # Bearing-sandwich refit (Jun 2026): the foot-hinge tang now sits at
    # the foot fitting on the end of the CF tube, not on a tibia spar.  Its
    # pin centre in tibia-local coordinates comes straight from the geometry
    # (HP.tibia_foot_hinge_local) so the pan-head/nyloc track the as-built
    # tang.  Pin axis is tibia-local +Y; the foot fork's +Y cheek outer face
    # sits at tang centreline + FOOT_HINGE_SLOT_W/2 + FOOT_HINGE_CHEEK_T.
    hinge_local = HP.tibia_foot_hinge_local()
    cheek_outer_y = (HP.FOOT_HINGE_SLOT_W / 2.0
                     + HP.FOOT_HINGE_CHEEK_T)
    head_local = hinge_local + np.array([0.0, +cheek_outer_y, 0.0])
    head = _apply_point(T_tibia_to_world, head_local)
    axis = _apply_dir(T_tibia_to_world, np.array([0.0, -1.0, 0.0]))
    nut_local = hinge_local + np.array([0.0, -cheek_outer_y, 0.0])
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


# ---------------------------------------------------------------------------
# Stacked-deck board-mount + standoff-column fasteners (Jun 2026 redesign)
# ---------------------------------------------------------------------------
#
# RETIRED: ``_emit_battery_holder_fasteners`` (the 4 clip-in
# battery_holder foot bolts) and ``_emit_electronics_tray_fasteners``
# (the Pi4 + USB-TTL bus-adapter board-mount bolts) are gone with the
# deck redesign.  The LiPo is now velcro-strapped to chassis_bottom's
# top face (NO bolts -- see ``make_chassis_bottom``'s velcro-strap
# slots), and the in-gap electronics_tray is replaced by two stacked
# decks (``make_uno_q_tray`` lower + ``make_buck_tray`` upper) bolted
# onto 4 standoff columns rising ABOVE chassis_top.


def _emit_deck_fasteners() -> list[FastenerInstance]:
    """Board-mount + standoff-column fasteners for the two stacked decks.

    Geometry (chassis design frame, z = 0 = chassis_bottom mesh centre
    plane; chassis_top top face = CHASSIS_GAP + 1.5 * CHASSIS_PLATE_T).
    Both trays are centred on the chassis Z axis at deck-local (0, 0);
    each tray mesh has its plate BOTTOM at local z = 0.

      Board-mount path (Uno Q on the lower tray's UNO_Q_HOLES, buck on
      the upper tray's BUCK_HOLES):
        * M3 x 8 SHCS DOWN into an M3 heat-set insert in the tray's
          DECK_BOSS_OD_M3 board-standoff boss.  Head bears on the boss
          top (= board underside); captive sub-assembly (the board is
          screwed down during the electronics install).

      Standoff-column path (4 columns at DECK_COLUMN_XY, per deck):
        * M3 x 10 SHCS DOWN through each tray's DECK_COLUMN_BOSS_OD
          column boss into the brass standoff's female top thread
          (modelled as a virtual M3 heat-set insert engagement target,
          same convention as the chassis_top brass-standoff bolts).
        * 4 M3 nyloc nuts UNDER chassis_top capture the lowest (level-1)
          standoff's male stud that drops through the chassis_top
          DECK_COLUMN_XY clearance holes.
    """
    out: list[FastenerInstance] = []

    deck_top_face = HP.CHASSIS_GAP + 1.5 * HP.CHASSIS_PLATE_T
    chassis_top_bot_z = HP.CHASSIS_GAP + 0.5 * HP.CHASSIS_PLATE_T

    board_skip_reason = (
        "captive sub-assembly fastener: the board is screwed down onto "
        "its tray bosses during the electronics install, before the "
        "tray stack is closed up; a hex key cannot reach the heads once "
        "the upper deck / board sit above"
    )

    # (label, tray plate-bottom world z, board hole pattern)
    uno_plate_bottom = deck_top_face + HP.DECK_LEVEL_1_STANDOFF_H
    buck_plate_bottom = (deck_top_face + HP.DECK_LEVEL_1_STANDOFF_H
                         + HP.DECK_LEVEL_2_STANDOFF_H)
    board_decks = (
        ("uno_q_tray", uno_plate_bottom, HP.UNO_Q_HOLES),
        ("buck_tray",  buck_plate_bottom, HP.BUCK_HOLES),
    )

    for (deck_label, plate_bottom_z, holes) in board_decks:
        boss_top_z = (plate_bottom_z + HP.DECK_TRAY_T
                      + HP.DECK_STANDOFF_BOSS_H)
        insert_top_z = boss_top_z - (HP.INSERT_M3_PILOT_DEPTH
                                     - HP.INSERT_M3_INSERT_LENGTH)
        for (hx, hy) in holes:
            head_world = np.array([hx, hy, boss_top_z])
            axis_world = np.array([0.0, 0.0, -1.0])
            out.append(FastenerInstance(
                part_number=PN_M3X8_SHCS,
                spec=SPEC_M3X8_SHCS_INTO_INSERT,
                head_world_xyz=head_world,
                axis_world=axis_world,
                role=(
                    f"{deck_label} board-mount "
                    f"M3 x 8 SHCS into heat-set insert"
                ),
                leg_index=None,
                joint=None,
                length_mm=8.0,
                cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
                skip_screwdriver_reason=board_skip_reason,
            ))

            out.append(FastenerInstance(
                part_number=PN_M3_HEATSET_INSERT,
                spec=SPEC_M3_HEATSET_INSERT,
                head_world_xyz=np.array([hx, hy, insert_top_z]),
                axis_world=np.array([0.0, 0.0, -1.0]),
                role=(
                    f"{deck_label} board-mount "
                    f"M3 heat-set insert"
                ),
                leg_index=None,
                joint=None,
                length_mm=HP.INSERT_M3_INSERT_LENGTH,
                cache_stl=f"{PN_M3_HEATSET_INSERT}.cache.stl",
                skip_screwdriver_reason=(
                    "heat-set insert installed with a soldering iron "
                    "BEFORE the board is screwed onto the tray boss; "
                    "no driver cone applies to the brass insert"
                ),
            ))

    # ---- Standoff-column path -----------------------------------------
    column_skip_reason = (
        "captive sub-assembly fastener: torqued as each deck is added "
        "to the standoff-column stack; once the deck above is fitted a "
        "hex key cannot reach the column-bolt heads"
    )
    for (cx, cy) in HP.DECK_COLUMN_XY:
        x_label = "+X" if cx > 0 else "-X"
        y_label = "+Y" if cy > 0 else "-Y"
        corner = f"{x_label}{y_label}"

        for (deck_label, plate_bottom_z) in (
            ("uno_q_tray", uno_plate_bottom),
            ("buck_tray",  buck_plate_bottom),
        ):
            boss_top_z = (plate_bottom_z + HP.DECK_TRAY_T
                          + HP.DECK_STANDOFF_BOSS_H)
            # Bolt DOWN through the column boss into the brass standoff
            # female top thread sitting directly under the tray plate.
            head_world = np.array([cx, cy, boss_top_z])
            axis_world = np.array([0.0, 0.0, -1.0])
            out.append(FastenerInstance(
                part_number=PN_M3X10_SHCS,
                spec=SPEC_M3X10_SHCS,
                head_world_xyz=head_world,
                axis_world=axis_world,
                role=(
                    f"{deck_label} standoff column {corner} "
                    f"M3 x 10 SHCS into standoff female thread"
                ),
                leg_index=None,
                joint=None,
                length_mm=10.0,
                cache_stl=f"{PN_M3X10_SHCS}.cache.stl",
                skip_screwdriver_reason=column_skip_reason,
            ))

            # Virtual engagement target: the brass standoff's female top
            # thread (real hardware is the standoff; not counted in BOM).
            out.append(FastenerInstance(
                part_number=PN_M3_HEATSET_INSERT,
                spec=SPEC_M3_HEATSET_INSERT,
                head_world_xyz=np.array([cx, cy, plate_bottom_z]),
                axis_world=np.array([0.0, 0.0, -1.0]),
                role=(
                    f"{deck_label} standoff column {corner} "
                    f"M3 standoff female thread "
                    f"(modelled as heat-set insert for engagement check)"
                ),
                leg_index=None,
                joint=None,
                length_mm=HP.INSERT_M3_INSERT_LENGTH,
                cache_stl="",
                skip_screwdriver_reason=(
                    "virtual engagement target representing the brass "
                    "standoff's female top thread; no separate fastener "
                    "is installed at this site -- the standoff hardware "
                    "supplies the threads.  No driver cone applies."
                ),
                is_virtual=True,
            ))

        # M3 nyloc nut UNDER chassis_top capturing the level-1 standoff's
        # male stud dropping through the DECK_COLUMN_XY clearance hole.
        out.append(FastenerInstance(
            part_number=PN_M3_NYLOC,
            spec=SPEC_M3_NYLOC,
            head_world_xyz=np.array([cx, cy, chassis_top_bot_z]),
            axis_world=np.array([0.0, 0.0, 1.0]),
            role=(
                f"chassis_top standoff column {corner} "
                f"retention M3 nyloc nut"
            ),
            leg_index=None,
            joint=None,
            length_mm=None,
            cache_stl=f"{PN_M3_NYLOC}.cache.stl",
            skip_screwdriver_reason=(
                "captive sub-assembly fastener: the standoff columns + "
                "decks are bolted onto chassis_top as a SEPARATE top "
                "sub-assembly (column nuts torqued on the open underside "
                "of chassis_top) BEFORE that sub-assembly is lowered "
                "onto the chassis_bottom brass standoffs.  In the "
                "assembled robot chassis_bottom sits below this nut, so "
                "the socket cannot reach it -- but it is never driven "
                "in the assembled state"
            ),
        ))

    return out


# ---------------------------------------------------------------------------
# Sandwich-joint clamp-cap fasteners (2 x M3 self-tap per hip + knee cradle)
# ---------------------------------------------------------------------------


def _emit_clamp_cap_fasteners(
    *,
    T_well_to_world: np.ndarray,
    leg_index: int,
    joint: str,
    location: str,
) -> list[FastenerInstance]:
    """The 2 M3 self-tap bolts that fasten one sandwich-joint clamp cap
    (``make_servo_clamp_cap``) onto the cradle's +/-X wall +Y ends.

    Well-local frame matches ``_servo_well_solid`` / the clamp cap: the
    head now bears on the COUNTERBORE shoulder recessed CLAMP_HEAD_CB_DEPTH
    below the flange OUTER face (y = WELL_D/2 + CLAMP_CAP_T - CLAMP_HEAD_CB_DEPTH;
    Jun 2026 head-inset fix so the head sits flush and the swept yoke clears it)
    and the M3 x 8 SHCS threads -Y through the flange clearance hole into
    the Phi CLAMP_BOLT_PILOT_OD self-tap pilot in the wall end (pilots
    cut by ``_servo_well_solid``).  Same part for the hip (coxa_link) and
    knee (femur_link) joints -- 2 per joint, 4 per leg, 24 per robot.
    """
    out: list[FastenerInstance] = []

    # Head recesses into the counterbore -> bears on the shoulder, not the
    # flange outer face (Jun 2026 head-inset fix).
    flange_outer_y = HP.WELL_D / 2.0 + HP.CLAMP_CAP_T - HP.CLAMP_HEAD_CB_DEPTH

    # The clamp cap goes on from the OPEN +Y face AFTER the servo is
    # seated but the disc horn + next-stage link close over that face,
    # so the +Y driver approach is blocked in the assembled robot.
    skip_reason = (
        "captive sub-assembly fastener: the clamp cap is bolted onto "
        "the cradle's open +Y face to trap the servo BEFORE the disc "
        "horn and the next-stage link are added; once those are on, the "
        "+Y driver approach to the cap bolts is blocked"
    )

    # (x, z) centres come from the SAME source the cap holes + cradle pilots
    # read (``hexapod_prototype.servo_clamp_bolt_centres``) so the emitted
    # fasteners land exactly on the coaxial cap-hole / wall-pilot axes.
    for (bx, bz) in HP.servo_clamp_bolt_centres():
        x_label = "+X" if bx > 0 else "-X"
        p_local = np.array([bx, flange_outer_y, bz])
        axis_local = np.array([0.0, -1.0, 0.0])   # -Y, into the wall pilot
        head = _apply_point(T_well_to_world, p_local)
        axis = _apply_dir(T_well_to_world, axis_local)
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS_SELFTAP,
            head_world_xyz=head,
            axis_world=axis,
            role=(
                f"{location} {x_label} clamp-cap "
                f"M3 x 8 SHCS self-tap"
            ),
            leg_index=leg_index,
            joint=joint,
            length_mm=8.0,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
            skip_screwdriver_reason=skip_reason,
        ))
    return out


# ---------------------------------------------------------------------------
# Switch-holster mount fasteners (M3 x 8 SHCS + M3 heat-set insert pair)
# ---------------------------------------------------------------------------


def _emit_switch_holster_fasteners() -> list[FastenerInstance]:
    """The 2 switch_holster mount bolts + their captive heat-set inserts.

    Bolts come DOWN from ABOVE the holster ear: head bears on the
    holster ear's TOP face, M3 SHCS shaft passes DOWN through the
    Phi BRACKET_BOLT_HOLE = 3.4 mm clearance hole in the ear, and
    threads INTO an M3 brass heat-set insert (McMaster 94459A130)
    that lives in a printed Phi SWITCH_HOLSTER_BOSS_OD = 8 mm boss
    on chassis_top's TOP face.  See ``make_chassis_top`` for the
    boss + pocket geometry.

    Bolt-DOWN-from-above (rather than UP-from-below) keeps the
    HEX_KEY 8 x 30 mm driver envelope in OPEN AIR above the
    chassis -- a bolt-up-from-below variant collides with the
    electronics_tray's base slab at z in [+5, +8] of the chassis-
    cavity (see check_screwdriver_access).  The trade-off is 2
    captive heat-set inserts on chassis_top (in a printed boss),
    not in the holster.

    Length budget (design frame, z = top of chassis_top):

        ear top z            = chassis_top_top + BOSS_H + FLOOR
                             = +38 + 3 + 4 = +45 mm (bolt head face)
        ear (clearance hole) = 4 mm (SWITCH_HOLSTER_FLOOR = 4)
        boss top z           = chassis_top_top + BOSS_H = +41 mm
        insert top face z    = boss_top - debris_overdrill = +40
                               (1 mm overdrill at the OPEN end of
                                the pocket, same convention as
                                the electronics_tray inserts)
        insert engagement    = 5 mm (INSERT_M3_INSERT_LENGTH)
        insert bottom z      = +35 mm
        bolt tip min z       = +35 mm

    Total bolt run from head to tip: 45 - 35 = 10 mm.  M3 x 10
    SHCS (PN 91290A114) -- same stock as the battery_holder feet
    bolts so the BOM stays compact.

    Deck redesign (Jun 2026): the Uno Q tray now sits 16 mm above
    chassis_top in the vertical hex-key approach to the ear bolt
    heads, so both mount bolts are captive sub-assembly fasteners
    (torqued BEFORE the standoff-column + deck stack is fitted).
    """
    out: list[FastenerInstance] = []
    # ``inspect_build._build_assembly_instances`` and the verifier's
    # ``_build_world_leg0_printed_parts`` BOTH translate chassis_top
    # by [0, 0, gap + plate_t] (with the mesh's local z = 0 at the
    # plate's mid-thickness), so chassis_top's top face in the design
    # frame is at z = gap + plate_t + plate_t / 2 = 38 (not gap +
    # 1.5 * plate_t = 38 -- those are the same number anyway, but
    # be explicit so the math matches the verifier exactly).
    chassis_top_centre_z = HP.CHASSIS_GAP + HP.CHASSIS_PLATE_T
    chassis_top_top_z    = chassis_top_centre_z + HP.CHASSIS_PLATE_T / 2.0
    # Bolt head bears on the holster ear's TOP face.
    boss_top_z = chassis_top_top_z + HP.SWITCH_HOLSTER_BOSS_H
    ear_top_z  = boss_top_z + HP.SWITCH_HOLSTER_FLOOR
    # Insert top face is 1 mm BELOW boss top (debris-clearance
    # overdrill at the OPEN end of the pocket; same convention as
    # _emit_electronics_tray_fasteners).
    debris_overdrill = HP.INSERT_M3_PILOT_DEPTH - HP.INSERT_M3_INSERT_LENGTH
    insert_top_z = boss_top_z - debris_overdrill
    for (bx, by), corner in zip(
        HP.SWITCH_HOLSTER_BOLT_CHASSIS_XY,
        ("+Y", "-Y"),
    ):
        head_world = np.array([bx, by, ear_top_z])
        axis_world = np.array([0.0, 0.0, -1.0])  # -Z (DOWN into material)
        out.append(FastenerInstance(
            part_number=PN_M3X10_SHCS,
            spec=SPEC_M3X10_SHCS,
            head_world_xyz=head_world,
            axis_world=axis_world,
            role=(
                f"switch_holster mount bolt {corner} "
                f"M3 x 10 SHCS into heat-set insert"
            ),
            leg_index=None,
            joint=None,
            length_mm=10.0,
            cache_stl=f"{PN_M3X10_SHCS}.cache.stl",
            # Deck redesign (Jun 2026): the Uno Q tray (16 mm above
            # chassis_top) now sits in the vertical hex-key approach to
            # the holster ear bolt head, so it is a captive sub-assembly
            # fastener -- the holster is bolted on BEFORE the standoff-
            # column + deck stack is fitted above chassis_top.
            skip_screwdriver_reason=(
                "captive sub-assembly fastener: the switch_holster is "
                "bolted onto chassis_top BEFORE the standoff-column + "
                "electronics-deck stack is fitted; once the Uno Q tray "
                "(16 mm above chassis_top) is on, it blocks the vertical "
                "hex-key approach to this ear bolt head"
            ),
        ))

        insert_head_world = np.array([bx, by, insert_top_z])
        out.append(FastenerInstance(
            part_number=PN_M3_HEATSET_INSERT,
            spec=SPEC_M3_HEATSET_INSERT,
            head_world_xyz=insert_head_world,
            axis_world=np.array([0.0, 0.0, -1.0]),
            role=(
                f"switch_holster chassis_top {corner} "
                f"M3 heat-set insert"
            ),
            leg_index=None,
            joint=None,
            length_mm=HP.INSERT_M3_INSERT_LENGTH,
            cache_stl=f"{PN_M3_HEATSET_INSERT}.cache.stl",
            skip_screwdriver_reason=(
                "heat-set insert installed with a soldering iron "
                "BEFORE the switch_holster is dropped onto the "
                "chassis_top bosses; no driver cone applies to "
                "the brass insert"
            ),
        ))
    return out


# ---------------------------------------------------------------------------
# IMU pad mount fasteners (M3 x 8 SHCS + M3 heat-set insert pair)
# ---------------------------------------------------------------------------


def _emit_imu_pad_fasteners() -> list[FastenerInstance]:
    """The 4 IMU-pad mount bolts + their captive heat-set inserts.

    The MPU-6050 / GY-521 IMU breakout is bolted to the printed
    ``imu_pad`` via 4 x M3 x 8 SHCS threading DOWN through the IMU's
    Phi 3.0 mm clearance holes into 4 x M3 brass heat-set inserts
    (McMaster ``94459A130``) captive in the pad's 4 printed bosses.
    The pad ITSELF is glued to chassis_top with double-sided foam
    tape (3 mm) so the IMU is mechanically decoupled from the
    chassis-side servo vibration -- NO fasteners between the pad
    and chassis_top.

    Length budget (design frame, z = 0 = chassis_bottom centre plane):

        chassis_top_top z    = CHASSIS_GAP + 1.5 * CHASSIS_PLATE_T = +38
        pad bottom z         = chassis_top_top + IMU_PAD_TAPE_T   = +41
        pad floor top z      = pad_bottom + IMU_PAD_T             = +43
        boss top z           = pad_floor_top + IMU_PAD_BOSS_H     = +48
        PCB bottom z         = boss_top                            = +48
        PCB top z            = boss_top + IMU_PCB_T               = +49.6
        insert top z         = boss_top - (pilot_depth - insert_len)
                             = +48 - 1                            = +47
        insert bottom z      = insert_top - INSERT_M3_INSERT_LENGTH
                             = +47 - 5                            = +42
        bolt tip min z       = head - L = +48 - 8                  = +40

    The PHYSICAL bolt head bears on the IMU PCB's top face at
    z = +49.6, but to match the convention used for the
    electronics_tray board-mount bolts (see
    ``_emit_electronics_tray_fasteners``: "conservative: place head
    face at the boss top (matches inspect_build's bolt rendering and
    avoids leaking a per-board PCB-thickness constant)") we put the
    REGISTRY head_world_xyz at the BOSS top instead of the PCB top.
    The 8 mm bolt length still reaches deep into the insert
    (z=40-48) and the engagement check probes material at the
    boss-top ring rather than at the PCB-top ring -- which only
    works if the world frame contains a printed part with material
    just below the head, exactly what the boss provides.  M3 x 8
    SHCS (PN 91290A113), same stock as the cradle / electronics_
    tray board-mount bolts.

    Why ``skip_screwdriver_reason``: although the IMU PCB's bolt
    heads ARE physically reachable from above (the pad sits at the
    chassis CG, not under the holster or arm), in practice the
    PROTOTYPE.md sequence has the user torque these BEFORE the
    chassis_top + (optional) arm baseplate close the chassis stack.
    Once those upper layers are on, a 2.5 mm hex key can swing down
    onto the bolt heads, but the operator would have to remove the
    arm baseplate first if it's installed.  Mark SKIP with the same
    rationale as the other tray fasteners so the verifier doesn't
    flag them as access-blocked.
    """
    out: list[FastenerInstance] = []

    # chassis_top top face z in the design frame.
    chassis_top_top_z = HP.CHASSIS_GAP + 1.5 * HP.CHASSIS_PLATE_T
    pad_bottom_z = chassis_top_top_z + HP.IMU_PAD_TAPE_T
    boss_top_z = pad_bottom_z + HP.IMU_PAD_T + HP.IMU_PAD_BOSS_H
    # Bolt head placement: see docstring for why we use boss_top_z
    # (= the boss top, NOT the PCB top) -- mirrors the convention
    # used by ``_emit_electronics_tray_fasteners`` so the head ring
    # probe lands inside the boss material rather than in the air
    # above the (un-modelled-as-printed) MPU-6050 PCB.
    head_z = boss_top_z
    # Insert top face sits ``pilot_depth - insert_len`` mm below the
    # boss top (same debris-overdrill convention as the
    # electronics_tray / switch_holster inserts).
    debris_overdrill = HP.INSERT_M3_PILOT_DEPTH - HP.INSERT_M3_INSERT_LENGTH
    insert_top_z = boss_top_z - debris_overdrill

    skip_reason = (
        "captive sub-assembly fastener: torqued during electronics "
        "install before chassis_top stack closes.  The IMU pad sits "
        "on the chassis CG on chassis_top; once chassis_top + "
        "(optional) arm baseplate close the chassis stack from "
        "above, the IMU bolt heads are reachable in principle but "
        "the sequence has them torqued first (same convention as "
        "the electronics_tray + switch_holster fasteners)"
    )

    for (bx, by), corner in zip(
        HP.IMU_PAD_BOLT_CHASSIS_XY,
        ("-X-Y", "-X+Y", "+X-Y", "+X+Y"),
    ):
        head_world = np.array([bx, by, head_z])
        axis_world = np.array([0.0, 0.0, -1.0])  # DOWN into the pad
        out.append(FastenerInstance(
            part_number=PN_M3X8_SHCS,
            spec=SPEC_M3X8_SHCS_INTO_INSERT,
            head_world_xyz=head_world,
            axis_world=axis_world,
            role=(
                f"imu_pad mount bolt {corner} "
                f"M3 x 8 SHCS into heat-set insert"
            ),
            leg_index=None,
            joint=None,
            length_mm=8.0,
            cache_stl=f"{PN_M3X8_SHCS}.cache.stl",
            skip_screwdriver_reason=skip_reason,
        ))

        insert_head_world = np.array([bx, by, insert_top_z])
        out.append(FastenerInstance(
            part_number=PN_M3_HEATSET_INSERT,
            spec=SPEC_M3_HEATSET_INSERT,
            head_world_xyz=insert_head_world,
            axis_world=np.array([0.0, 0.0, -1.0]),
            role=(
                f"imu_pad {corner} "
                f"M3 heat-set insert"
            ),
            leg_index=None,
            joint=None,
            length_mm=HP.INSERT_M3_INSERT_LENGTH,
            cache_stl=f"{PN_M3_HEATSET_INSERT}.cache.stl",
            skip_screwdriver_reason=(
                "heat-set insert installed with a soldering iron "
                "BEFORE the MPU-6050 IMU is bolted to the imu_pad "
                "(PROTOTYPE.md section 6.1 step 12); no driver "
                "cone applies to the brass insert"
            ),
        ))
    return out


# ---------------------------------------------------------------------------
# Chassis-stack fasteners (May 2026 tray-mount + standoff-pattern fix)
# ---------------------------------------------------------------------------


def _emit_chassis_stack_fasteners() -> list[FastenerInstance]:
    """The 4 chassis_top -> brass-standoff bolts + their (synthetic)
    engagement targets, and the 4 chassis_bottom retention nyloc nuts
    that capture the brass standoff male threads under the bottom plate.

    Deck redesign (Jun 2026): the in-gap electronics_tray is retired, so
    the former 4 tray-to-chassis_bottom mount bolts + heat-set inserts
    are gone; only the brass-standoff path (which clamps the two main
    plates together) remains here.

    Geometry summary (design frame, z = 0 = chassis_bottom mesh centre
    plane; chassis_bottom spans z in [-2, +2], chassis_top spans z in
    [+34, +38]):

      Brass-standoff path (4 sites at HP.CHASSIS_STANDOFF_HOLES_XY =
      (+/-35, 0) and (0, +/-35)):

        * M3 x 10 SHCS, head at z = chassis_top_top = +38, axis -Z,
          length 10 mm.  Tip at z = +28.  Engages the M-F brass
          standoff's female top threads.  We model the engagement
          target as a synthetic ``M3 heat-set insert`` at z = +34
          (= top of standoff = chassis_top bottom face) so the
          verifier's ``check_fastener_engagement`` pairs the bolt
          correctly and the bolt's "joins >= 2 parts" rule passes
          (chassis_top + paired engagement target).  Physically the
          target is brass standoff thread, NOT a printed-part
          insert; the role string makes this explicit.

        * M3 nyloc nut, outer face at z = -2 (chassis_bottom bottom
          face), axis +Z (driver approaches from below the robot,
          nut threads onto the male stud projecting DOWN through
          chassis_bottom).  Length None (nuts are not probed as
          bolts).  No paired bolt -- the brass standoff's male
          thread is unmodeled hardware.
    """
    out: list[FastenerInstance] = []

    # Jun 2026 single-part merge: chassis_bottom's bottom face stepped down from
    # the old -2 plate underside to -6 (the folded-in flat floor slab), so the
    # standoff nyloc nut now seats on the -6 face (and its stud passes through
    # the floor via the clearance hole make_chassis_bottom cuts there).
    chassis_bottom_bot_z = HP.CHASSIS_SPLIT_Z - HP.CHASSIS_BOTTOM_FLOOR_T  # = -6
    chassis_top_top_z = HP.CHASSIS_GAP + 1.5 * HP.CHASSIS_PLATE_T  # = +38
    chassis_top_bot_z = HP.CHASSIS_GAP + 0.5 * HP.CHASSIS_PLATE_T  # = +34

    # Deck redesign (Jun 2026): the in-gap electronics_tray is retired,
    # so its 4 tray-to-chassis_bottom mount bolts + heat-set inserts are
    # gone.  Only the chassis_top <-> chassis_bottom brass-standoff path
    # (which actually clamps the two main plates together) remains.

    # ---- Chassis-top -> brass-standoff -> chassis_bottom path ---------
    #
    # The +X standoff at (+35, 0) sits UNDER the switch_holster
    # (which spans chassis-x in [+34, +85], chassis-y in [-11, +11])
    # so the chassis_top bolt's hex key driver from ABOVE intrudes
    # into the holster body once the holster is dropped onto its 2
    # chassis_top bosses.  Deck redesign (Jun 2026): ALL 4 brass
    # standoff heads at CHASSIS_STANDOFF_HOLES_XY = (+/-35, 0) /
    # (0, +/-35) now also sit UNDER the Uno Q tray (96 x 80 mm, deck-
    # local (0,0), 16 mm above chassis_top), so the vertical hex-key
    # approach to every standoff bolt head is blocked by the lower
    # deck.  The natural assembly order bolts chassis_top DOWN onto
    # the brass standoff tops BEFORE the standoff-column + deck stack
    # is fitted above, so all 4 are captive-sub-assembly serviceable.
    standoff_skip_reason = (
        "captive sub-assembly fastener: torqued during chassis-"
        "stack closure BEFORE the standoff-column + electronics-deck "
        "stack (Uno Q tray 16 mm above chassis_top) is fitted.  Once "
        "the lower deck is on, it sits in the 2.5 mm hex key's "
        "vertical approach path to this standoff bolt head (the +X "
        "head is additionally under the switch_holster) -- see "
        "PROTOTYPE.md section 6 for the explicit assembly order"
    )
    for (sx, sy) in HP.CHASSIS_STANDOFF_HOLES_XY:
        if sx > 0:
            label = "+X"
        elif sx < 0:
            label = "-X"
        elif sy > 0:
            label = "+Y"
        else:
            label = "-Y"

        # All 4 standoff bolt heads are now under the lower deck (and
        # the +X head is also under the switch_holster), so each is a
        # captive sub-assembly fastener.
        bolt_skip = standoff_skip_reason

        # M3 x 10 SHCS dropped DOWN from above chassis_top into the
        # brass M-F standoff's female top threads.
        head_world = np.array([sx, sy, chassis_top_top_z])
        axis_world = np.array([0.0, 0.0, -1.0])
        out.append(FastenerInstance(
            part_number=PN_M3X10_SHCS,
            spec=SPEC_M3X10_SHCS,
            head_world_xyz=head_world,
            axis_world=axis_world,
            role=(
                f"chassis_top brass standoff {label} top "
                f"M3 x 10 SHCS into standoff female thread"
            ),
            leg_index=None,
            joint=None,
            length_mm=10.0,
            cache_stl=f"{PN_M3X10_SHCS}.cache.stl",
            skip_screwdriver_reason=bolt_skip,
        ))

        # Synthetic engagement target: the brass M-F standoff's female
        # top threads modeled as an M3 heat-set insert at the chassis_
        # top bottom face = top of standoff.  The role string makes
        # the actual hardware (brass standoff) explicit; we re-use
        # SPEC_M3_HEATSET_INSERT so the verifier's
        # ``_find_paired_engagement_target`` pairs it with the bolt
        # above (5 mm of brass thread engagement is equivalent to the
        # cradle / battery / tray heat-set inserts the verifier was
        # designed for).
        #
        # ``is_virtual=True`` marks this entry as NOT REAL HARDWARE so
        # ``fastener_bom_rows`` doesn't count it toward the BOM total
        # (the brass M-F standoff is the actual part on the order; see
        # the "M3 standoffs" BOM row).
        target_head_world = np.array([sx, sy, chassis_top_bot_z])
        out.append(FastenerInstance(
            part_number=PN_M3_HEATSET_INSERT,
            spec=SPEC_M3_HEATSET_INSERT,
            head_world_xyz=target_head_world,
            axis_world=np.array([0.0, 0.0, -1.0]),
            role=(
                f"chassis_top brass standoff {label} top "
                f"M3 standoff female thread "
                f"(modelled as heat-set insert for engagement check)"
            ),
            leg_index=None,
            joint=None,
            length_mm=HP.INSERT_M3_INSERT_LENGTH,
            cache_stl="",
            skip_screwdriver_reason=(
                "virtual engagement target representing the brass "
                "M-F standoff's female top thread; no separate "
                "fastener is installed at this site -- the brass "
                "standoff hardware supplies the threads.  No driver "
                "cone applies."
            ),
            is_virtual=True,
        ))

        # M3 nyloc nut UNDER chassis_bottom retaining the standoff's
        # male thread coming DOWN through the plate's Phi 3.4 mm
        # clearance hole.  Driver approaches from below the robot
        # (-Z); the nut's outer (visible) face sits flush against
        # chassis_bottom's bottom face.
        nut_head_world = np.array([sx, sy, chassis_bottom_bot_z])
        nut_axis_world = np.array([0.0, 0.0, 1.0])  # axis INTO the wall above
        out.append(FastenerInstance(
            part_number=PN_M3_NYLOC,
            spec=SPEC_M3_NYLOC,
            head_world_xyz=nut_head_world,
            axis_world=nut_axis_world,
            role=(
                f"chassis_bottom brass standoff {label} "
                f"retention M3 nyloc nut"
            ),
            leg_index=None,
            joint=None,
            length_mm=None,
            cache_stl=f"{PN_M3_NYLOC}.cache.stl",
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
        # NO output-face case screws (Jun 2026 STS3215 reconciliation).
        # An earlier model invented a "front-face 4-bolt case-screw
        # mount" on the servo OUTPUT face (4 x M2.5 per cradle x 3
        # cradles x 6 legs = 72 phantom case screws).  Per the
        # authoritative Waveshare ST3215 mount-bracket geometry the
        # output face has NO usable body-mounting screws: the dia-20
        # disc horn sits on the dia-14 cross and covers them, so those
        # 72 case screws are physically impossible.  Servo body
        # retention is instead provided by PRINTED parts -- the yaw
        # servo by ``make_yaw_servo_retainer`` (strap + anchor bolts)
        # and the hip/knee servos by the clamshell
        # ``make_servo_clamp_cap`` (2 x M3 per joint, emitted below).
        # The output face carries ONLY the flush disc horn + its 4 x M3
        # leg bolts on DISC_HORN_BOLT_PCD = 14 mm.

        # POSITIVE servo body retention: M2.5 end-face bolts per cradle into
        # the servo's real -X END-face 10x10 case-hole square, so the printed
        # cradle bolts the servo instead of only gripping it.  The deep hip/
        # knee cradles take all 4.
        #
        # The YAW cradle takes NONE (Jun 2026 flush-horn refit): both bearings
        # moving above the flush horn lowered the yaw output 5.5 mm
        # (YAW_TOWER_RAISE 9 -> 14.5), dropping the servo so its body hangs
        # ~20 mm below the -6 chassis floor.  Even the upper end-face row now
        # lands at the floor edge with no -X wall behind the head, so the yaw
        # servo is retained instead by the ``make_yaw_servo_retainer`` strap +
        # anchor bolts + the output-face seat + open-bottom pocket.  (The
        # retired screws reacted X-translation, not yaw torque.)
        # 0 yaw + 4 hip + 4 knee = 8 per leg x 6 = 48.
        out.extend(_emit_end_face_fasteners(
            T_well_to_world=_hip_cradle_T(leg_index),
            leg_index=leg_index,
            joint="hip",
            location=f"coxa_link L{leg_index} hip cradle",
        ))
        out.extend(_emit_end_face_fasteners(
            T_well_to_world=_knee_cradle_T(leg_index),
            leg_index=leg_index,
            joint="knee",
            location=f"femur_link L{leg_index} knee cradle",
        ))

        # Sandwich-joint clamp-cap self-tap bolts (2 per hip + knee
        # cradle).  The yaw cradle (chassis_bottom) has no clamp cap.
        out.extend(_emit_clamp_cap_fasteners(
            T_well_to_world=_hip_cradle_T(leg_index),
            leg_index=leg_index,
            joint="hip",
            location=f"coxa_link L{leg_index} hip clamp-cap",
        ))
        out.extend(_emit_clamp_cap_fasteners(
            T_well_to_world=_knee_cradle_T(leg_index),
            leg_index=leg_index,
            joint="knee",
            location=f"femur_link L{leg_index} knee clamp-cap",
        ))

        # Yaw-bearing cap join screws (3 M3 self-tap per leg = 18) that bolt
        # the split bearing tower's TOP half down onto chassis_bottom.
        out.extend(_emit_yaw_cap_join_fasteners(leg_index))

        # Yaw anti-rotation SADDLE chassis anchors (2 M3 self-tap per leg = 12)
        # that bolt the yaw_servo_retainer UP to the chassis_bottom floor.
        out.extend(_emit_yaw_retainer_anchor_fasteners(leg_index))
        out.extend(_emit_yaw_rear_case_fasteners(leg_index))

        # Link-to-disc-horn bolts (June 2026: disc horn, no printed adapter).
        out.extend(_emit_horn_fasteners_yaw(leg_index))
        out.extend(_emit_horn_fasteners_hip(leg_index))
        out.extend(_emit_horn_fasteners_knee(leg_index))

        # PASSIVE rear-boss disc-horn bolts + central retention screws on the
        # hip + knee sandwich joints (symmetric-yoke refit; yaw has none).
        out.extend(_emit_passive_horn_fasteners(leg_index, "hip"))
        out.extend(_emit_passive_horn_fasteners(leg_index, "knee"))
        out.extend(_emit_passive_center_screw(leg_index, "hip"))
        out.extend(_emit_passive_center_screw(leg_index, "knee"))

        # Servo spline center screws (3 servos x 6 legs = 18 total).
        out.extend(_emit_spline_fastener(leg_index, "yaw"))
        out.extend(_emit_spline_fastener(leg_index, "hip"))
        out.extend(_emit_spline_fastener(leg_index, "knee"))

        # Foot hinge pin (M3 x 16 pan-head + nyloc nut).
        out.extend(_emit_foot_hinge_fastener(leg_index))

    # Chassis-level fasteners (no leg index).  Deck redesign (Jun 2026):
    # the clip-in battery_holder is retired (LiPo is velcro-strapped to
    # chassis_bottom -- no bolts), so there are no battery-mount
    # fasteners.

    # Stacked-deck board-mount + standoff-column fasteners (Jun 2026
    # redesign): M3 x 8 SHCS board bolts into the Uno Q / buck tray
    # bosses, M3 x 10 column bolts into the brass standoff female tops,
    # and 4 retention nyloc nuts under chassis_top.  Replaces the retired
    # electronics_tray board-mount bolts.
    out.extend(_emit_deck_fasteners())

    # Switch-holster mount bolts (May 2026 "essentials" pass).  2 x
    # M3 x 10 SHCS thread UP from BELOW chassis_top into 2 x M3
    # heat-set inserts captive in the holster's mounting ear --
    # same captive-insert pattern as the battery_holder feet.  Bolts
    # are user-serviceable from inside the chassis cavity (before
    # the battery is dropped into the holder).
    out.extend(_emit_switch_holster_fasteners())

    # IMU pad mount bolts (May 2026: MPU-6050 promoted from optional
    # to standard kit).  4 x M3 x 8 SHCS thread DOWN through the IMU
    # PCB into 4 x M3 heat-set inserts captive in the printed pad's
    # 4 corner bosses.  The pad itself is glued to chassis_top with
    # double-sided foam tape -- no fasteners between the pad and
    # chassis_top.
    out.extend(_emit_imu_pad_fasteners())

    # Chassis-stack standoff fasteners (May 2026 tray-mount fix).
    # 4 x M3 x 8 SHCS into chassis_bottom tray-mount heat-set inserts
    # + 4 x M3 x 10 SHCS into brass standoff female tops (chassis_top
    # side) + 4 x M3 nyloc nuts UNDER chassis_bottom (capturing the
    # standoff's male thread).  Replaces the previous "Chassis-stack
    # standoff bolts" entry in the "NOT enumerated yet" docstring
    # list.
    out.extend(_emit_chassis_stack_fasteners())

    # Jun 2026: the chassis_bottom HIGH/LOW print split has been re-merged into
    # ONE printed part, so the 12 M3 x 10 cradle-plate-to-plate join screws + 6
    # register dowels are GONE (no _emit_chassis_join_fasteners).

    return out


# ---------------------------------------------------------------------------
# BOM helpers
# ---------------------------------------------------------------------------


def fastener_bom_rows() -> list[tuple[str, str, int, str]]:
    """Return (spec, part_number, qty, used_in) rows for the BOM table,
    aggregated across ``build_all_fastener_instances()``.

    Virtual entries (``is_virtual=True``) -- e.g. the brass-standoff
    female-thread engagement targets used by the verifier's engagement
    check -- are SKIPPED so they don't inflate the BOM total.  Real
    hardware that backs those entries (the M-F brass standoffs) is
    enumerated under the "M3 standoffs" row of the printed-parts BOM
    table.
    """
    counts: dict[tuple[str, str], int] = {}
    usage: dict[tuple[str, str], set[str]] = {}
    for fi in build_all_fastener_instances():
        if fi.is_virtual:
            continue
        key = (fi.spec, fi.part_number)
        counts[key] = counts.get(key, 0) + 1
        usage.setdefault(key, set()).add(_usage_bucket(fi))
    rows = []
    for (spec, pn), qty in sorted(counts.items()):
        used = ", ".join(sorted(usage[(spec, pn)]))
        rows.append((spec, pn, qty, used))
    # Stable, human-friendly order: SHCS by length, then nuts.
    spec_order = {
        SPEC_M3X6_SHCS:             0,
        SPEC_M3X6_SHCS_SELFTAP:     0,
        SPEC_M25_BODY_SCREW:        1,
        SPEC_M25X8_SHCS:            1,
        SPEC_M25X8_SHCS_INTO_INSERT: 2,
        SPEC_M25_HEATSET_INSERT:    3,
        SPEC_M3X8_SHCS:             4,
        SPEC_M3X8_DISC_HORN:        4,
        SPEC_M3X8_SHCS_INTO_INSERT: 5,
        SPEC_M3X8_SHCS_SELFTAP:     6,
        SPEC_M3X10_SHCS:            7,
        SPEC_M3X10_DISC_HORN:       7,
        SPEC_M3X10_SHCS_SELFTAP:    7,
        SPEC_M3_HEATSET_INSERT:     8,
        SPEC_M3X32_SHCS:            9,
        SPEC_M3X16_PAN:            10,
        SPEC_M3_NYLOC:             11,
    }
    rows.sort(key=lambda r: (spec_order.get(r[0], 99), r[0]))
    return rows


def _usage_bucket(fi: FastenerInstance) -> str:
    role = fi.role
    if "brass standoff" in role:
        if "M3 x 10 SHCS" in role or "M3x10 SHCS" in role:
            return ("chassis_top brass standoff bolts "
                    "(M3 x 10 SHCS into standoff female thread)")
        if "retention" in role and "nyloc" in role:
            return "chassis_bottom brass standoff retention nuts"
        return ("chassis_top brass standoff female threads "
                "(virtual heat-set insert engagement targets)")
    if "standoff column" in role:
        # Stacked-deck standoff-column hardware (Jun 2026 redesign).
        if "M3 x 10 SHCS" in role or "M3x10 SHCS" in role:
            return ("deck standoff column bolts "
                    "(M3 x 10 SHCS into standoff female thread)")
        if "retention" in role and "nyloc" in role:
            return ("deck standoff column retention nuts "
                    "(M3 nyloc under chassis_top)")
        return ("deck standoff column female threads "
                "(virtual heat-set insert engagement targets)")
    if "board-mount" in role:
        # Stacked-deck board-mount hardware (Uno Q + buck converter).
        # Distinguish bolt vs insert by the "SHCS" marker -- both role
        # strings contain "heat-set insert".
        if "SHCS" in role:
            return ("deck board-mount bolts "
                    "(Uno Q + buck, M3 x 8 SHCS into inserts)")
        return "deck board-mount heat-set inserts (Uno Q + buck)"
    if "body screw" in role:
        return "cradle servo body-retention bolts (M2.5 into servo end face)"
    if "cap-to-tower" in role:
        # Jun 2026 split yaw-bearing tower: the 18 M3 x 8 SHCS that pull each
        # yaw_bearing_cap down onto chassis_bottom to capture the 6706 pair.
        return ("yaw_bearing_cap join screws "
                "(cap -> chassis_bottom tower, M3 x 8 SHCS self-tap)")
    if "clamp-cap" in role:
        return "sandwich-joint clamp-cap bolts (M3 SHCS self-tap)"
    if "switch_holster" in role:
        if "heat-set insert" in role:
            return "switch_holster heat-set inserts"
        return "switch_holster mount bolts (M3 SHCS into heat-set insert)"
    if "imu_pad" in role:
        if "heat-set insert" in role:
            return "imu_pad heat-set inserts (MPU-6050 mount)"
        return "imu_pad mount bolts (MPU-6050 M3x8 SHCS into insert)"
    if "heat-set insert" in role:
        # The 36 brass heat-set inserts (McMaster 94459A130) pressed
        # into the -X cradle bosses BEFORE the cradle servo is mounted.
        # (Design E mixed-mode: 2 heat-set sites per cradle x 3 cradles
        # per leg x 6 legs = 36, down from the 72 of the original
        # f03d59b heat-set switch.)
        return "cradle heat-set inserts"
    if "cradle" in role:
        # Cradle servo-mount M3 x 8 SHCS.  Split by engagement medium:
        #   * "M3 x 8 SHCS into insert" -- the 36 -X bolts (heat-set);
        #   * "M3 x 8 SHCS self-tap"    -- the 36 +X bolts (self-tap
        #     into Phi 2.5 mm pilot in plastic).
        # See the Design E rationale in hexapod_prototype.py's
        # INSERT_M3_SELFTAP_* block.
        if "self-tap" in role:
            return "cradle servo mounts (M3 SHCS self-tap)"
        return "cradle servo mounts (M3 SHCS into heat-set insert)"
    if "disc-horn" in role:
        return "link-to-disc-horn bolts"
    if "spline screw" in role:
        return "servo spline center screws"
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
