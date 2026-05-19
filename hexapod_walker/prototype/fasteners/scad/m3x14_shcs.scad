// fasteners/scad/m3x14_shcs.scad
//
// ISO 4762 / DIN 912 M3 socket-head cap screw, length 14 mm.
// McMaster-Carr 91290A115 (black-oxide steel).  Mirrors the
// SPEC_M3X14_SHCS entry in fastener_registry.py.
//
// NopSCADlib output convention (raw, before Python post-process):
//   * Head:  z in [0, +head_height]    (head_height ~= 3 mm)
//   * Shank: z in [-length, 0]
// make_fastener_meshes.py mirrors across the XY plane (negate Z) so
// the cached STL matches fasteners/_parametric.py's convention:
//   * Head:  z in [-head_height, 0]
//   * Shank: z in [0, +length]
// (= mesh +Z runs along the shank, INTO the material).
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/screws.scad>

$fn = 32;
screw(M3_cap_screw, 14);
