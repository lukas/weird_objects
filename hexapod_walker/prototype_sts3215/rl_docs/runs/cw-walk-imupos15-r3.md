# cw-walk-imupos15-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:00:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-imupos15-r2

**wandb_id**: mqgm0b95

**hardware_ready**: False

**hypothesis**: RETRY #4 of cw-walk-imupos15 (r1, r1's precursor, and r2's direct-launch all died to the same host-wide EOFError launch-collision storm, 0 steps each -- confirmed NOT specific to this spec, a concurrent unrelated run failed identically at the same moment). Same spec unchanged; queued to backlog so the self-repairing drain places it once the current contention clears rather than fighting it live.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: NO-EFFECT. Own-cfg letter-pass (IMU-mount offset xy=15mm/z=0-20mm spread, det med fwd 1.32m>=1.2 gate, 12/12 gait_valid, 0 term) but champion baseline measured under the IDENTICAL imu-position spread (logs/ckpt_eval/longdist_r2_imupos_base) matches the trained checkpoint draw-for-draw, including the same 2/6 severe det craters (ep4/5: prog 0.49/0.50, slip 3.1-3.4/4.0-4.3, fwd ~0.6-0.7m in both). DR0 nominal retention clean (slip med 0.98, fwd med 1.53m, 6/6 gv). Frames watched det: normal alternating six-leg gait, no dragging/through-floor. IMU-position exposure training changed nothing measurable -- joins the sensor/calibration NO-DR-exposure ladder (now 12-for-12), the offset is unobservable from proprioception at these magnitudes just like gyro-bias/encoder-noise/zero-bias before it. No requeue.

