# cw-walk-tiltnoise-r5-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T05:23:49+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-tiltnoise

**wandb_id**: xhxx30eg

**hardware_ready**: False

**hypothesis**: 5th retry of cw-walk-tiltnoise (base+r1-r4 ALL died 0-step to the fleet launch-collision storm, gotcha 13b -- never got a science result on this axis, unlike its siblings imupos15/gyrobias3 which both closed NO-EFFECT). Same isolated axis unchanged: OPERATOR WISHLIST 13c untested IMU tilt-reading NOISE (dr.tilt_noise_deg=1.5, 5x the full-DR default) off the plain champion longdist-r2, dr-scale 0.0 otherwise.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.tilt_noise_deg=1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: NO-EFFECT (5th retry finally trained after 4 collision deaths). own-cfg (dr.tilt_noise_deg=1.5, 5x default) det 4/6 healthy (med prog 0.95 slip 1.28 fwd 1.38m) but eps4/5 crater (prog 0.48/0.35 slip 3.19/4.40 fwd 0.65/0.59m, video: legs still all cycling, slow paddle-shuffle, no flag-leg/fall). NAMED BASELINE measured this triage: champion longdist_r2 under the IDENTICAL 1.5deg tilt-noise spread, same seed (logs/ckpt_eval/champion_tiltnoise15_base) reproduces the SAME eps4/5 crater almost exactly (prog 0.47/0.39 slip 3.33/4.13 fwd 0.65/0.62m) and the same med (prog 0.92 slip 1.23 fwd 1.47m) -- no improvement outside noise. DR0 nominal retention clean (det 6/6 gv, slip med 0.97<=1.24). Same conclusion as torquescale/gyrobias3/gyronoise15/encodernoise/imupos15: this is the lineage's fixed-draw crater, independent of the DR axis; 5x tilt-noise exposure bought nothing. Single-axis calibration-DR ladder now fully exhausted; stop feeding this class.

