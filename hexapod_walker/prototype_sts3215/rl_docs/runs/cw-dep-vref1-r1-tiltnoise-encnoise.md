# cw-dep-vref1-r1-tiltnoise-encnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T19:04:52+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1-tiltnoise

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if two different sensors are both noisier than nominal at once -- the tilt/attitude reading (already PASSed alone, 3x the default noise floor) AND the joint encoder reads (already PASSed alone, 0.5deg)? Both individually benign but never combined; unlike axes tied to the same physical part (e.g. gyronoise+tiltnoise, both IMU), this pairs two INDEPENDENT sensor subsystems (IMU vs joint encoders) that are both always slightly noisy on real hardware simultaneously. Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- combined sensor noise composes free like every other pairing tonight. If-false: two independent noisy channels compound into a control problem neither alone caused -- flag as a real pre-attempt-#2 sensing risk.

**gate**: own-cfg (DR0.35 + dr.tilt_noise_deg=1.0 + dr.encoder_noise_deg=0.5) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv; video frames watched det+sto

**refused_reason**: hexapod-mjx-train-7 already runs cw-dep-vref1-r1-fric-s1 — GPU pods host exactly one run; pick a free GPU pod.

