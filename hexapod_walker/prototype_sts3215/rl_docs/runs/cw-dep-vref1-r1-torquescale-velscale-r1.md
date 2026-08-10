# cw-dep-vref1-r1-torquescale-velscale-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T20:07:21+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-torquescale

**hypothesis**: Plain English: does the hardware checkpoint still walk cleanly if the motors are BOTH weaker-than-expected (torque droop 0.5-1.05x, already PASSed alone) AND slower-than-expected (speed ceiling 0.6-2.2x, already PASSed alone) at the same time -- the realistic case for a draining battery, which saps torque and speed together rather than independently? (r1: first attempt hit a busy-pod REFUSED + orphaned git tag before training, no science; retrying under a fresh name. Isolated from velscale's existing latency pairing to test just the two actuator-power axes.) Per P0 rule 3, k_current=0 (inherited from torquescale). If-true: own-cfg det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- composes free like every other actuator-realism pairing tonight (gainvar+torquescale already PASSed). If-false: weak torque AND a low speed ceiling compound (the controller can neither push harder nor move faster to compensate) -- flag as a real pre-attempt-#2 battery-sag risk.

**gate**: own-cfg (DR0.35 + dr.torque_scale=0.5,1.05 + dr.vel_scale=0.6,2.2) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20% (progress_ratio spread expected on the vel_scale axis per velscale's own precedent, not a defect); known fixed-draw crater (det/4) pre-allowed as baseline

**refused_reason**: hexapod-mjx-train-11 already runs cw-dep-vref1-r1-kvscale-groundtilt — GPU pods host exactly one run; pick a free GPU pod.

