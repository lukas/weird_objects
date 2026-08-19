"""Write ``extra_stl/coxa_link_horn_drive.stl`` -- REPLACEMENT one-piece
horn-drive coxa for the OLDER hexapod build.

Aug 19 2026 (user: "I have older version I made of the hexapod where the
coxa yaw hub and coxa hip bracket are combined and the bracket broke -
could you make me a new bracket with the screw holes in the bottom but
add all the new features we put in brackets like the screw holes to
screw in the motors?").

Base geometry: the LAST production state of the one-piece horn-drive
coxa, git ``fa28041d`` (parent of f8fb6fba, the 2026-06-18 bearing-pair
split that retired it).  That part is the yaw disc-horn PAD ("the screw
holes in the bottom": 4x M3 on the Phi 14 disc-horn circle + centre
spline screw + collar recess) bridged to the hip sandwich cradle (servo
bracket + 688 passive-bearing housing) -- i.e. the coxa yaw hub and the
coxa hip bracket COMBINED, exactly the broken part.  The vintage module
is rebuilt from git history at runtime so the replacement matches the
old robot's dimensions (COXA_LENGTH 25, WELL_D 29, ...), NOT today's
drifted constants (COXA_LENGTH 12.5 etc.).

Grafted NEW bracket features (from today's ``hexapod_prototype``):

  * FRONT-CASE screw capture (Aug 2026): 4x Phi 2.7 self-tap bores
    through the cradle's front mount plate into the servo's molded
    front-shell pilots, heads sunk FLUSH in Phi 5.2 x 2.2 counterbores
    (M2.5 x 6 self-tap, PN 96877A150).
  * END-FACE body bolts: 4x M2.5 x 8 through the cradle's -X wall into
    the servo's 10 x 10 mm end-face threaded square, heads recessed in
    Phi 5.0 counterbores.

NOT grafted -- rear retention tab: in the vintage FRONT-MOUNT cradle the
servo slides in from the BACK and seats against the front plate, so a
tab lying under the back face would block insertion (the tab only works
with today's lateral-drop-in clamshell).  With 4x M2.5 machine screws
(vintage front plate) + 4 front-case self-tappers + 4 end-face bolts the
motor is screwed from three sides anyway.

EXTRA part: not in the production print set, the scene, or the BOM.

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_extra_coxa_link_horn_drive.py
"""
from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

import hexapod_prototype as hp_now  # noqa: E402  (today's feature constants)

VINTAGE_SHA = "fa28041d68795865c499b4cfba7af6fbcb2f6e89"
VINTAGE_PATH = "hexapod_walker/prototype_sts3215/hexapod_prototype.py"
OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_NAME = "coxa_link_horn_drive.stl"


def _load_vintage():
    """Import the pinned-vintage hexapod_prototype from git history."""
    repo_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    src = subprocess.check_output(
        ["git", "show", f"{VINTAGE_SHA}:{VINTAGE_PATH}"], cwd=repo_root)
    with tempfile.NamedTemporaryFile(
            "wb", suffix="_hexapod_prototype_vintage.py",
            delete=False) as f:
        f.write(src)
        path = f.name
    spec = importlib.util.spec_from_file_location(
        "hexapod_prototype_vintage", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _cradle_feature_cuts(v):
    """The grafted screw features as cutting solids in the vintage hip
    cradle's WELL-LOCAL frame (origin at the servo back-face centre,
    +Z = output) -- positions from TODAY's servo survey, plate/wall
    geometry from the VINTAGE constants."""
    cuts = []

    # Servo dims must agree closely or the hole survey doesn't transfer.
    assert abs(v.SERVO_OUTPUT_X - hp_now.SERVO_OUTPUT_X) < 0.01
    assert abs(v.SERVO_BODY_W - hp_now.SERVO_BODY_W) < 0.5
    assert abs(v.SERVO_BODY_H - hp_now.SERVO_BODY_H) < 0.5

    # 1) Front-case capture: bores through the plate + flush counterbores.
    for (fx, fy) in hp_now.servo_front_case_hole_centres():
        bore = v._cyl(hp_now.FRONT_CASE_SCREW_OD / 2.0, v.WELL_PLATE_T + 4.0)
        bore.apply_translation([fx, fy, v.WELL_RIM_Z + v.WELL_PLATE_T / 2.0])
        cuts.append(bore)
        cb = v._cyl(hp_now.FRONT_CASE_CBORE_OD / 2.0,
                    hp_now.FRONT_CASE_CBORE_DEPTH + 1.0)
        cb.apply_translation(
            [fx, fy, v.WELL_H - (hp_now.FRONT_CASE_CBORE_DEPTH - 1.0) / 2.0])
        cuts.append(cb)

    # 2) End-face body bolts through the -X wall (same construction as
    # today's _servo_well_solid end_face_bolts block, vintage wall dims).
    body_face_x = -v.SERVO_BODY_W / 2.0
    wall_outer_x = -v.WELL_W / 2.0
    head_plane_x = body_face_x - hp_now.SERVO_BODY_BOLT_STANDOFF
    assert head_plane_x - wall_outer_x >= 1.0, "wall too thin for M2.5x8"
    cl_outer = wall_outer_x - 1.0
    cl_inner = body_face_x + hp_now.SERVO_MOUNT_THREAD_DEPTH
    from trimesh.transformations import rotation_matrix
    for (by, bz) in hp_now.servo_end_face_bolt_centres():
        hole = v._cyl(hp_now.SERVO_BODY_BOLT_OD / 2.0, cl_inner - cl_outer)
        hole.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        hole.apply_translation([0.5 * (cl_outer + cl_inner), by, bz])
        cuts.append(hole)
        cbore = v._cyl(hp_now.SERVO_BODY_BOLT_HEAD_OD / 2.0,
                       head_plane_x - cl_outer)
        cbore.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        cbore.apply_translation([0.5 * (cl_outer + head_plane_x), by, bz])
        cuts.append(cbore)
    return cuts


def _probe(mesh, pts_link, want_solid, label):
    inside = mesh.contains(np.asarray(pts_link, dtype=float))
    ok = inside.all() if want_solid else (~inside).all()
    state = "solid" if want_solid else "void"
    assert ok, (f"{label}: expected {state} at {np.round(pts_link, 2).tolist()}"
                f", contains={inside.tolist()}")
    print(f"  OK  {label} ({len(pts_link)} probe(s) {state})")


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    v = _load_vintage()
    coxa = v.make_coxa_link()   # the sandwich override = production vintage

    # Same placement the vintage builder used for the hip fixed side.
    M = v._joint_place((v.COXA_LENGTH, 0.0, v.COXA_HIP_DROP),
                       x_dir=(1, 0, 0), z_dir=(0, 1, 0))
    cuts = []
    for c in _cradle_feature_cuts(v):
        c.apply_transform(M)
        cuts.append(c)
    mesh = v._diff(coxa, *cuts)
    assert mesh.is_watertight, "grafted coxa mesh is not watertight"

    def w2l(pts):  # well-local -> coxa-link frame
        pts = np.asarray(pts, dtype=float)
        return pts @ M[:3, :3].T + M[:3, 3]

    print("feature checks:")
    # Bottom yaw-pad pattern (the screw holes in the bottom).
    r = v.DISC_HORN_BOLT_PCD / 2.0
    _probe(mesh, [(r * np.cos(a), r * np.sin(a), 1.0)
                  for a in v.DISC_HORN_BOLT_ANGLES_RAD] + [(0, 0, 1.0)],
           False, "bottom disc-horn 4x M3 + centre screw bores")
    _probe(mesh, [(r * np.cos(a + np.pi / 4.0), r * np.sin(a + np.pi / 4.0),
                   v.PEDESTAL_CAP_T / 2.0 + 1.0)
                  for a in v.DISC_HORN_BOLT_ANGLES_RAD],
           True, "yaw pad material between the horn bolts")

    fc = list(hp_now.servo_front_case_hole_centres())
    _probe(mesh, w2l([(fx, fy, v.WELL_RIM_Z + v.WELL_PLATE_T / 2.0)
                      for fx, fy in fc]),
           False, "front-case screw bores (NEW)")
    _probe(mesh, w2l([(fx, fy, v.WELL_H - hp_now.FRONT_CASE_CBORE_DEPTH / 2.0)
                      for fx, fy in fc]),
           False, "front-case head counterbores (NEW)")
    # Plate must stay solid just outboard of each hole rim.
    _probe(mesh, w2l([(fx, np.sign(fy) * (abs(fy) + 3.5),
                       v.WELL_RIM_Z + v.WELL_PLATE_T / 2.0)
                      for fx, fy in fc]),
           True, "plate material around the front-case holes")

    ef = list(hp_now.servo_end_face_bolt_centres())
    mid_x = 0.5 * (-v.WELL_W / 2.0 - v.SERVO_BODY_W / 2.0)
    _probe(mesh, w2l([(mid_x, by, bz) for by, bz in ef]),
           False, "end-face M2.5 body-bolt bores (NEW)")
    # Driver access: free air outboard of the -X wall on each bolt axis.
    _probe(mesh, w2l([(-v.WELL_W / 2.0 - d, by, bz)
                      for by, bz in ef for d in (3.0, 12.0)]),
           False, "end-face screw driver access (free air)")
    # Access above the plate for the front-case screws.
    _probe(mesh, w2l([(fx, fy, v.WELL_H + d)
                      for fx, fy in fc for d in (3.0, 12.0)]),
           False, "front-case screw driver access (free air)")

    path = os.path.join(OUT_DIR, OUT_NAME)
    mesh.export(path)
    print(f"wrote {path}")
    print(f"  extents: {np.round(mesh.extents, 1)}  "
          f"volume: {mesh.volume / 1000.0:.1f} cm3  "
          f"(vintage git {VINTAGE_SHA[:8]})")
    print("Hardware: 4x M3 + 1 centre screw -> yaw disc horn (bottom); "
          "4x M2.5 countersunk -> servo front mount square (as before); "
          "NEW 4x M2.5 x 6 self-tap -> front-case pilots; "
          "NEW 4x M2.5 x 8 -> servo end face.")


if __name__ == "__main__":
    main()
