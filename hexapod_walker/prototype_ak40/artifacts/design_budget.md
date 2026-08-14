```
AK40 hexapod -- derived design budget
============================================================
Mass budget:
  actuators (18x AK40-10 V3.0)                    3.42 kg
  printed links + CF tibias + feet (6 legs)       0.72 kg
  chassis plates + standoffs + fasteners          0.55 kg
  battery (6S 5000 mAh LiPo)                      0.72 kg
  compute (Pi 5 + 3x USB-CAN + buck + IMU)        0.22 kg
  wiring + e-stop loopkey + fuse + misc           0.17 kg
  TOTAL                                           5.80 kg   (56.9 N)

Static joint torque vs stance (per-foot load = W/legs):
  stance      ride  reach   hip3  knee3   hip6  knee6   (N*m; 3=tripod, 6=all legs)
  tall         233     81   1.54   0.59   0.77   0.30
  nominal      227     96   1.82   0.74   0.91   0.37
  crouch       140    200   3.79   2.01   1.90   1.01

  AK40-10 rated 1.3 N*m / peak 4.1 N*m.
  Rules: PARK poses (tall/nominal, 6 legs) must sit under
  RATED -- both do (<=0.91).  Tripod walking may exceed
  rated transiently (~50% stance duty keeps the thermal
  average under rated: nominal hip 1.82 -> avg 0.91).
  'crouch' exceeds rated even on 6 legs (1.90) -- it is a
  TRANSITIONAL pose (sit-down/stand-up path only, seconds,
  46% of peak); never park or walk there.

Printed part mass (solid PETG @1.27 g/cm^3; real prints
with walls+infill land ~55-70% of solid):
  chassis_bottom                   343.3 g
  chassis_top                      271.6 g
  coxa_link                         70.9 g
  femur_link                        38.7 g
  tibia_yoke                        17.4 g
  foot_boot                         18.5 g
  -> per-leg link set solid 127 g; x6 at ~0.65 solid factor ~= 0.50 kg (budget line: 0.72 kg incl. tubes+feet)
```
