# cw-dep-vref1-r1-gyronoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T07:35:48+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: y4w38zva

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's 25deg tilt-termination safety envelope depends on a clean tilt-rate reading, but it has never trained with realistic gyro noise (1.5deg/s, validated elsewhere) -- directly relevant to P0 ruling 3 (a future rate-based safety term must not mistake noise spikes for real falls). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; gyro noise composes free like it does elsewhere. If-false: noisy angular-rate sensing interacts badly with the wide tilt envelope (spurious near-threshold behavior) -- flag before hardware.

**gate**: Own-cfg (DR0.35+gyro_noise1.5deg/s) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-noise retention clean; frames watched det

**verdict**: PASS -- gyro rate-noise (1.5deg/s) composes free onto the vref1-r1 hardware-candidate checkpoint. Own-cfg (DR0.35+noise) det+sto gv 6/6, 0 term, det slip/m med 1.01 sto med 1.13 -- inside vref1-r1's own band (0.89-1.13/1.13-1.36). DR0 no-noise retention gv 6/6, 0 term, det slip/m med 0.95 sto med 0.95 (excl. the known lineage fixed-draw sto/4 crater, slip 4.23 vs parent's own 5.97 on the identical draw -- same trait, milder). Own-cfg also shows a det/5+sto/0+sto/1 crater cluster (slip 1.9-2.3) but the IDENTICAL episode indices/magnitudes reproduce across gyronoise/imumount/latency -- three unrelated axes -- so this reads as a DR0.35+seed0 lineage draw pattern, not a gyro-noise-specific regression. Frames (det_1, det_5, sto crater) show the same low-amplitude six-leg creep as the parent, body level, no flag leg/drag/fall. Training finished clean (reward quarters 583/686/671/654 >= parent's 582/657/641/625).

