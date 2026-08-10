# cw-dep-vref1-r1-imubias

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: EVALUATED

**created**: 2026-08-10T18:39:20+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1-imumount

**wandb_id**: ctvcavvt

**hardware_ready**: False

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if the tilt sensor has a residual calibration bias (reads a few degrees off level even after imu_calibrate), distinct from a mounting-rotation error already tested (imumount) or gyro-rate noise already tested (gyronoise)? This is the attitude-BIAS axis, validated free on the older walk champion (cw-walk-imubias3, NO-EFFECT/PASS) but never composed onto the contract-exact hardware candidate, and directly relevant to the 25deg tilt-termination safety threshold vref1-r1 depends on. Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- attitude bias composes free like it did on the older champion. If-false: the bias interacts with the wide 25deg envelope in a new way on THIS checkpoint (early/late trips, or an exploit) -- flag before hardware.

**gate**: own-cfg (DR0.35 + dr.imu_bias_deg=3.0) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

**verdict**: PASS -- residual IMU calibration bias (3deg, distinct from mount-rotation/gyro-noise already tested) composes free onto the contract-exact hardware candidate, same as it did on the older walk champion. Own-cfg (DR0.35+bias) det/sto gv 6/6 each, 0 term, prog med 1.04/0.91, slip med 1.10/1.14 (within vref1-r1's own 0.89-1.13/1.13-1.36 band). DR0 retention det/sto gv 6/6, 0 term, prog med 1.04/1.02, slip med 1.04/0.97 (in-band). Degraded episodes (det/5,sto/0,1 own-cfg; det/4 gate) match the pre-registered lineage fixed-seed fingerprint exactly -- video-checked (det_0/4 gate, sto_0 own-cfg): level body, all six legs cycling, no flag-leg/drag/fall. Does not interact badly with the 25deg tilt-termination safety threshold in any of the 24 episodes across both passes (0 unexpected trips). Not independently hardware-ready; clears attitude-bias as a safe axis for the hardware candidate.

