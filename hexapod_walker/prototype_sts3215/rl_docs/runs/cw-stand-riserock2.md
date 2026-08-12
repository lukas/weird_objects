# cw-stand-riserock2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-12T00:38:11+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**hypothesis**: The learned rise fails 10/10 on hardware with one signature (tilt_roll trip at tick ~227 mid-curl, 10.1-10.6deg roll, currents flat) while the sim curl stays <2deg under both actuator fits: the hardware curl rocks laterally and the policy has never SEEN a rocked curl. If hard1 (the deployed rise+hold specialist) trains with the new dr.rise_rock_* one-side fold bias (prob 0.5, 6-15deg, the bench-measured band; probed to rock hard1 curls into the trip band 5/8 and closable by dumb P-feedback), it will learn to level the curl instead of tripping.

**gate**: Forced rise-rock injection (prob 1, 10deg): rise valid_plant >= 4/6 det with zero tilt_roll terminations (parent hard1 baseline trips >=5/8 under the same injection). Retention: unrocked rise/hold/lower panel matches hard1 within noise (valid_plant delta <= 1/6, zero new falls).

**refused_reason**: hexapod-mjx-train-2 already runs cw-stand-riserock2-r1 — GPU pods host exactly one run; pick a free GPU pod.

