"""Run JUST the new bridge-joint sub-check from check_flimsy_joints, to
iterate quickly on geometry/thresholds without paying for the full
voxelisation sweep across all parts.
"""
from __future__ import annotations
import os
import sys
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import hexapod_prototype as hp
from _verify_prototype import _check_coxa_link_bridge_joint


if __name__ == "__main__":
    # Monkey-patch the check to ALWAYS dump per-slice output for easier
    # before/after inspection (default behaviour only dumps when FAIL).
    import _verify_prototype as _vp
    _orig = _vp._check_coxa_link_bridge_joint

    def _verbose(mesh):
        ok = _orig(mesh)
        if ok:
            # Re-dump per-slice rows so PASS runs also show numbers.
            import numpy as np
            import shapely.geometry as _sg
            well_z_drop = -(hp.WELL_D / 2.0
                            + hp.COXA_ARM_T / 2.0
                            + hp.WELL_Z_DROP_EXTRA)
            well_top_z  = hp.COXA_LIFT + well_z_drop + hp.WELL_D / 2.0
            yoke_bot_z  = hp.COXA_LIFT
            x_win = (+17.0, +50.0)
            y_win = (-25.0,  -5.0)
            window = _sg.box(x_win[0], y_win[0], x_win[1], y_win[1])
            z_lo = well_top_z + _vp.COXA_LINK_BRIDGE_SLICE_DZ
            z_hi = yoke_bot_z - _vp.COXA_LINK_BRIDGE_SLICE_DZ
            n = max(int(round((z_hi - z_lo) / _vp.COXA_LINK_BRIDGE_SLICE_DZ)) + 1, 2)
            z_values = np.linspace(z_lo, z_hi, n)
            for z in z_values:
                polys = _vp._slice_polygons(mesh, float(z))
                if polys.is_empty:
                    print(f"           [..]  z = {z:6.2f}  empty slice")
                    continue
                inter = polys.intersection(window)
                if inter.is_empty:
                    print(f"           [..]  z = {z:6.2f}  empty in window")
                    continue
                area, y_c, ixx, y_min, y_max = _vp._ixx_about_centroid_x(inter)
                c = max(y_max - y_c, y_c - y_min)
                sx = ixx / c if c > 1e-6 else 0.0
                print(f"           [OK]  z = {z:6.2f}  "
                      f"area = {area:6.1f} mm^2  Sx = {sx:7.1f} mm^3")
        return ok

    mesh = hp.make_coxa_link()
    ok = _verbose(mesh)
    print()
    print(f"bridge joint: {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)
