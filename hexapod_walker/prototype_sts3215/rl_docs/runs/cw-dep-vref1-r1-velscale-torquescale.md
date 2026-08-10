# cw-dep-vref1-r1-velscale-torquescale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T20:07:20+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-torquescale

**hypothesis**: Plain English: does the hardware candidate still walk cleanly if the servo's ceiling SPEED is uncertain AND the battery is sagging under load AT THE SAME TIME -- two actuator-side realities the real robot has together, both individually PASSED solo on this line (velscale via cmddrop-velscale, torquescale via torquescale/-deadband/-gyronoise) but never paired. Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + dr.vel_scale=0.6,2.2 + dr.torque_scale=0.5,1.05) det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- composes free like every other actuator-axis pairing tonight. If-false: reduced torque margin combined with speed-ceiling uncertainty compounds worse than either alone -- flag as a real pre-attempt-#2 actuator risk.

**gate**: own-cfg (DR0.35 + dr.vel_scale=0.6,2.2 + dr.torque_scale=0.5,1.05) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto) +-20%; known fixed-draw crater (det/4 or det/5) pre-allowed as baseline

**refused_reason**: hexapod-mjx-train-11 already runs cw-dep-vref1-r1-kvscale-groundtilt — GPU pods host exactly one run; pick a free GPU pod.

