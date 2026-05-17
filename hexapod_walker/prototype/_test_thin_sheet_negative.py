"""Negative test: confirm that the thin-sheet check would fire on the
original (unreinforced) coxa_link arm slab.  We rebuild make_coxa_link
WITHOUT the arm_cap_pos / arm_cap_neg ribs and run the check; the
expected result is FAIL with a >= ~2000-voxel cluster.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import trimesh

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_prototype as hp
from _verify_prototype import (
    _thin_sheet_clusters_for_part,
    FLIMSY_VOXEL_PITCH,
    MIN_SHEET_T_LOWER,
    MIN_SHEET_T_UPPER,
    MIN_SHEET_CLUSTER_VOX,
    MAX_SHEET_BUDGET_VOX,
)


def make_unreinforced_coxa_link():
    """Inline near-copy of make_coxa_link() with the arm cap ribs
    removed.  Keep the rest of the geometry identical so the
    difference is purely the structural cap.
    """
    arm_w = 22.0
    arm_t = hp.COXA_ARM_T

    hub_t = arm_t + 2.0
    hub = trimesh.creation.box((34.0, 34.0, hub_t))
    hub.apply_translation((0, 0, hub_t / 2.0))

    arm = trimesh.creation.box((hp.COXA_LENGTH + 28.0, arm_w, arm_t))
    arm.apply_translation(((hp.COXA_LENGTH + 28.0) / 2.0 - 12.0, 0,
                            arm_t / 2.0))

    well = hp._servo_well_solid()
    wire_slot = hp._wire_exit_slot()
    R = trimesh.transformations.rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    well.apply_transform(R)
    wire_slot.apply_transform(R)
    delta = np.array([hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
                       -(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
                       0.0])
    well.apply_translation(delta)
    wire_slot.apply_translation(delta)
    well_z_drop = -(hp.WELL_D / 2.0 + arm_t / 2.0)
    well.apply_translation([0.0, 0.0, well_z_drop])
    wire_slot.apply_translation([0.0, 0.0, well_z_drop])

    arm_x_extent     = hp.COXA_LENGTH + 28.0
    arm_x_centre     = arm_x_extent / 2.0 - 12.0
    arm_minus_y_edge = -arm_w / 2.0
    well_near_y      = hp.WELL_RIM_Z + delta[1]
    well_top_z       = well_z_drop + hp.WELL_D / 2.0
    bridge_y_min = well_near_y - 0.5
    bridge_y_max = arm_minus_y_edge + 0.5
    bridge_y_extent = bridge_y_max - bridge_y_min
    bridge_y_centre = (bridge_y_min + bridge_y_max) / 2.0
    bridge_z_min = -hp.COXA_BRIDGE_GUSSET_H + 0.5
    bridge_z_max = arm_t
    bridge_z_extent = bridge_z_max - bridge_z_min
    bridge_z_centre = (bridge_z_min + bridge_z_max) / 2.0
    bridge = trimesh.creation.box(
        (arm_x_extent, bridge_y_extent, bridge_z_extent))
    bridge.apply_translation((arm_x_centre, bridge_y_centre,
                                bridge_z_centre))

    gusset_y_min     = bridge_y_min
    gusset_y_max     = +arm_w / 2.0
    gusset_y_extent  = gusset_y_max - gusset_y_min
    gusset_y_centre  = (gusset_y_min + gusset_y_max) / 2.0
    gusset = trimesh.creation.box((30.0, gusset_y_extent, arm_t))
    gusset.apply_translation((hp.COXA_LENGTH - 14.0, gusset_y_centre,
                                arm_t / 2.0))

    gusset_under_x0    = hp.HIP_PAD_R + 1.0
    gusset_under_y_min = bridge_y_min
    gusset_under_y_max = -arm_w / 2.0 + 2.0
    gusset_under_z_min = -hp.COXA_BRIDGE_GUSSET_H
    gusset_under_z_max = 0.5
    gusset_under = trimesh.creation.box(
        (hp.COXA_BRIDGE_GUSSET_L,
         gusset_under_y_max - gusset_under_y_min,
         gusset_under_z_max - gusset_under_z_min))
    gusset_under.apply_translation((
        gusset_under_x0 + hp.COXA_BRIDGE_GUSSET_L / 2.0,
        (gusset_under_y_min + gusset_under_y_max) / 2.0,
        (gusset_under_z_min + gusset_under_z_max) / 2.0,
    ))

    body_unlifted = trimesh.boolean.union(
        [hub, arm, well, gusset, bridge, gusset_under])
    body_unlifted = trimesh.boolean.difference(
        [body_unlifted, wire_slot])
    body_unlifted.apply_translation([0.0, 0.0, hp.COXA_LIFT])

    pedestal = trimesh.creation.box((34.0, 34.0, hp.COXA_LIFT))
    pedestal.apply_translation((0, 0, hp.COXA_LIFT / 2.0))

    body_x_min   = -hp.SERVO_BODY_W / 2.0 - 1.0
    body_x_max   =  hp.SERVO_BODY_W / 2.0 + (hp.COXA_LENGTH - hp.SERVO_OUTPUT_X) + 1.0
    trough_x_ext = body_x_max - body_x_min
    trough_x_cen = (body_x_min + body_x_max) / 2.0
    body_top_z   = hp.COXA_LIFT - hp.WELL_D / 2.0 + hp.SERVO_BODY_D / 2.0
    trough_z_max = body_top_z + 1.0
    trough_z_min = 0.0
    trough_z_ext = trough_z_max - trough_z_min
    trough_z_cen = (trough_z_min + trough_z_max) / 2.0
    trough = trimesh.creation.box((trough_x_ext, 34.0, trough_z_ext))
    trough.apply_translation((trough_x_cen, 0.0, trough_z_cen))

    bolt_total_h = hp.COXA_LIFT + hub_t
    hub_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = trimesh.creation.cylinder(hp.HORN_BOLT_OD / 2.0,
                                       bolt_total_h * 4)
        h.apply_translation([hp.HORN_BOLT_PCD / 2.0 * np.cos(a),
                              hp.HORN_BOLT_PCD / 2.0 * np.sin(a),
                              bolt_total_h / 2.0])
        hub_holes.append(h)
    centre_hole = trimesh.creation.cylinder(hp.HORN_CENTRE_OD / 2.0,
                                              bolt_total_h * 4)
    centre_hole.apply_translation([0, 0, bolt_total_h / 2.0])

    body = trimesh.boolean.union([pedestal, body_unlifted])
    return trimesh.boolean.difference(
        [body, trough, *hub_holes, centre_hole])


def main():
    print("Negative test: thin-sheet check on UNREINFORCED coxa_link")
    print("-" * 72)
    mesh = make_unreinforced_coxa_link()
    print(f"Mesh: watertight={mesh.is_watertight}, "
          f"vol={mesh.volume:.1f} mm^3, "
          f"bounds={mesh.bounds}")

    clusters, biggest = _thin_sheet_clusters_for_part(
        mesh, FLIMSY_VOXEL_PITCH,
        min_chord_mm_lower=MIN_SHEET_T_LOWER,
        min_chord_mm_upper=MIN_SHEET_T_UPPER,
        min_cluster_vox=MIN_SHEET_CLUSTER_VOX)

    expected_fail = biggest > MAX_SHEET_BUDGET_VOX
    print(f"\nLargest cluster: {biggest} voxels "
          f"(budget {MAX_SHEET_BUDGET_VOX})")
    for c in clusters[:3]:
        ext = c['bbox_max'] - c['bbox_min']
        cx, cy, cz = c['centroid']
        print(f"  {c['voxel_count']:5d} vox  "
              f"chord={c['min_chord_mm']:.2f}-{c['max_chord_mm']:.2f} mm  "
              f"axes={c['thin_axes']:3s}  "
              f"centroid=({cx:+.1f},{cy:+.1f},{cz:+.1f})  "
              f"bbox={ext[0]:.1f}x{ext[1]:.1f}x{ext[2]:.1f}")

    print()
    print("Test result:", "PASS (would-fail as expected)"
                            if expected_fail
                            else "FAIL (negative test did NOT detect "
                                  "the unreinforced thin sheet -- "
                                  "the check is too lax)")
    sys.exit(0 if expected_fail else 1)


if __name__ == "__main__":
    main()
