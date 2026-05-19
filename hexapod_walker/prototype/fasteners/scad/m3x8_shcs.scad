// fasteners/scad/m3x8_shcs.scad
//
// ISO 4762 / DIN 912 M3 socket-head cap screw, length 8 mm.
// McMaster-Carr 91290A113 (black-oxide steel).  Mirrors the
// SPEC_M3X8_SHCS entry in fastener_registry.py.  See m3x14_shcs.scad
// for the head/shank coordinate convention; identical here.
include <NopSCADlib/utils/core/core.scad>
include <NopSCADlib/vitamins/screws.scad>

$fn = 32;
screw(M3_cap_screw, 8);
