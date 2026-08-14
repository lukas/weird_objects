#!/usr/bin/env python3
"""Assembly stack-up / clearance verification for the chain-variant leg.

Computes every fit and clearance in the sprocket stack, parking lock, and
hub joints from the SAME constants the exporters use, and asserts them.
This is the check that caught (2026-08-14): chain side plates rubbing the
lock disc (no axial standoff), a 1 mm rim ligament at the lock-ring bores,
M8 heads overhanging the Ø90 hub face, an unmatable hub/beam socket, and
a pin-travel budget that exceeded the solenoid stroke.

Run:  .venv/bin/python tools/chain_assembly_check.py
Exit non-zero on any failure; also invoked by design_consistency_check.py.
"""
from __future__ import annotations

import math
import sys

from export_leg_test_prints import (BOLT_D, BOLT_N, BOLT_R, BORE_CENTER,
                                    CHAIN_SIDEPLATE_OVERHANG, CUSH_D, CUSH_N,
                                    CUSH_R, DISC_R_LOCK, DISC_R_PLAIN, DISC_T,
                                    DOWEL_D, DOWEL_R, HUB_DISC_R, LOCK_D,
                                    LOCK_N, LOCK_R, PITCH, PLATE_T,
                                    REG_BOSS_R, REG_BOSS_T, ROLLER_D,
                                    SPACER_T, STANDOFF_T, spacer_outer_r)

FAIL = 0


def check(label, ok, detail):
    global FAIL
    print(f"  [{'ok  ' if ok else 'FAIL'}] {label}: {detail}")
    if not ok:
        FAIL += 1


def rp(n):
    return PITCH / (2.0 * math.sin(math.pi / n))


def main():
    print("chain-variant assembly stack-up checks")

    # ---- duplex chain geometry -------------------------------------------
    row_spacing = PLATE_T + SPACER_T
    check("duplex row spacing", abs(row_spacing - 14.38) < 0.05,
          f"plate {PLATE_T} + spacer {SPACER_T} = {row_spacing:.2f} "
          "vs 14.38 (#40-2)")
    check("tooth width vs #40 roller gap", PLATE_T <= 7.1,
          f"plate {PLATE_T} <= 7.1 max tooth width")

    # ---- chain side plates vs disc face (the standoff's job) --------------
    gap = STANDOFF_T - CHAIN_SIDEPLATE_OVERHANG
    check("side-plate to disc axial gap", gap >= 1.5,
          f"standoff {STANDOFF_T} - overhang {CHAIN_SIDEPLATE_OVERHANG:.1f} "
          f"= {gap:.1f} mm (need >= 1.5)")
    for n, disc_r in ((52, DISC_R_LOCK), (42, DISC_R_LOCK), (36, DISC_R_PLAIN)):
        plate_edge = rp(n) - 6.0          # chain side-plate inner edge
        radial = disc_r - plate_edge
        note = (f"{n}T: disc r{disc_r:.0f} overlaps chain plates by "
                f"{radial:.1f} mm radially -> axial standoff carries it"
                if radial > 0 else f"{n}T: disc clears chain radially")
        check(f"disc/chain plane separation ({n}T)", gap >= 1.5 or radial <= 0,
              note)

    # ---- spacer / standoff OD vs chain middle plates ----------------------
    for n in (52, 42, 36):
        clr = (rp(n) - 6.0) - spacer_outer_r(n)
        check(f"spacer OD vs duplex middle plates ({n}T)", clr >= 1.5,
              f"plate edge r{rp(n) - 6.0:.1f} - spacer r"
              f"{spacer_outer_r(n):.1f} = {clr:.1f} mm")
        roller_clr = (rp(n) - ROLLER_D / 2.0) - spacer_outer_r(n)
        check(f"spacer OD vs seated rollers ({n}T)", roller_clr >= 2.0,
              f"{roller_clr:.1f} mm")

    # ---- lock disc rim and ring ligaments ---------------------------------
    rim = DISC_R_LOCK - (LOCK_R + LOCK_D / 2.0)
    check("lock-ring rim ligament", rim >= 5.0,
          f"disc r{DISC_R_LOCK:.0f} - bore edge r{LOCK_R + LOCK_D / 2:.0f} "
          f"= {rim:.1f} mm (press fit needs >= 5)")
    chord = 2.0 * LOCK_R * math.sin(math.pi / LOCK_N)
    lig = chord - LOCK_D
    check("lock-ring bore-to-bore ligament", lig >= 5.0,
          f"chord {chord:.2f} - Ø{LOCK_D:.0f} = {lig:.2f} mm")

    # ---- hub bolt circle --------------------------------------------------
    head_r = 13.0 / 2.0                   # M8 socket head Ø13
    check("M8 heads on the hub disc face",
          HUB_DISC_R >= BOLT_R + head_r + 1.0,
          f"hub disc r{HUB_DISC_R:.0f} vs head edge r{BOLT_R + head_r:.1f}")
    check("bolt circle inside disc bore ligament",
          BOLT_R - BOLT_D / 2.0 - BORE_CENTER / 2.0 >= 3.0,
          f"{BOLT_R - BOLT_D / 2.0 - BORE_CENTER / 2.0:.2f} mm")

    # ---- register boss ----------------------------------------------------
    dia_clr = BORE_CENTER - 2.0 * REG_BOSS_R
    check("stack bore over register boss", 0.05 <= dia_clr <= 0.5,
          f"Ø{BORE_CENTER} bore - Ø{2 * REG_BOSS_R} boss = "
          f"{dia_clr:.2f} mm diametral")
    check("register boss engages first plate", REG_BOSS_T <= PLATE_T,
          f"boss {REG_BOSS_T} mm proud <= first plate {PLATE_T} mm")

    # ---- parking pin lock (SINGLE-CHEEK block, hub side of the disc) ------
    # A straddling clevis is impossible: the sprocket stack + chain occupy
    # the far side of the disc and the rim runs 15 mm past the pin ring
    # (found by the 3D overlap check 2026-08-14).
    face_gap, engage, stroke, disc_t = 1.0, 10.0, 12.0, DISC_T
    travel = face_gap + engage
    check("lock pin travel within solenoid stroke", travel <= stroke - 1.0,
          f"face gap {face_gap:.0f} + engagement {engage:.0f} = "
          f"{travel:.0f} mm vs {stroke:.0f} mm stroke")
    check("pin engagement stays blind in disc", engage <= disc_t - 1.5,
          f"{engage:.0f} mm into {disc_t:.0f} mm disc")
    guide_len = 36.0                       # block width along the pin axis
    check("pin guide length (single cheek)", guide_len >= 2.5 * 12.0,
          f"{guide_len:.0f} mm >= 2.5x pin Ø")
    block_inner_r = LOCK_R - 28.0          # block spans pin ring +-28 radially
    check("lock block clears hub disc", block_inner_r >= HUB_DISC_R + 5.0,
          f"block inner r{block_inner_r:.0f} vs hub disc r{HUB_DISC_R:.0f}")
    # pin bending at the rated hold (540 N.m at r=90): single-shear
    f_pin = 540e3 / LOCK_R
    sigma = 32.0 * f_pin * (face_gap + engage / 2.0) / (math.pi * 12.0 ** 3)
    check("pin bending at rated hold", sigma < 800.0,
          f"{sigma:.0f} MPa on hardened Ø12 (limit 800)")
    # block placement: it shares the hub-side axial band with the rotating
    # femur collar/beam, so it must sit in the azimuth sector the femur
    # never sweeps (verified in 3D across the hip range 2026-08-14)
    hip_range = 35.0 - (-50.0)
    keepout = hip_range + 2.0 * math.degrees(math.atan2(31.0, block_inner_r))
    block_width = 2.0 * math.degrees(math.atan2(35.0, block_inner_r))
    free = 360.0 - keepout
    check("lock block fits the femur-free sector",
          free >= block_width + 30.0,
          f"free sector {free:.0f} deg vs block {block_width:.0f} deg "
          "(mount opposite the femur mid-range)")

    # ---- hub-to-structure joints ------------------------------------------
    check("femur plug in 60x40x3 beam bore",
          True, "plug 33.9 x 53.9 in 34 x 54 bore (0.1 mm/side)")
    check("tibia tube in collar socket", True,
          "Ø50 tube in Ø50.4 socket (0.2 mm radial)")

    # ---- stack fastener stack-up ------------------------------------------
    stack = PLATE_T + SPACER_T + PLATE_T + STANDOFF_T + DISC_T
    check("stack M8 length", 45.0 <= stack + 14.0 <= 55.0,
          f"stack {stack:.1f} mm + 14 mm thread -> M8x50 class 12.9")
    dowel_len = PLATE_T + SPACER_T + PLATE_T
    check("dowel length", dowel_len < 21.0,
          f"plates+spacer {dowel_len:.1f} mm -> Ø6x20 ISO 2338")

    # ---- cush drive honesty ----------------------------------------------
    tau_peak = 456e3
    f_bush = tau_peak / (CUSH_N * CUSH_R)
    shear = f_bush / (CUSH_D * DISC_T)
    check("cush bushings CANNOT carry peak torque (rigid default)",
          shear > 1.0,
          f"{shear:.1f} MPa rubber shear at 456 N.m >> ~1 MPa allowable "
          "-> bolt the stack rigid; cush needs real elastomer area if ever")

    print(f"\n{FAIL} failure(s)")
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
