# cw-walk-imupos15-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T23:00:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-imupos15-r2

**wandb_id**: mqgm0b95

**hypothesis**: RETRY #4 of cw-walk-imupos15 (r1, r1's precursor, and r2's direct-launch all died to the same host-wide EOFError launch-collision storm, 0 steps each -- confirmed NOT specific to this spec, a concurrent unrelated run failed identically at the same moment). Same spec unchanged; queued to backlog so the self-repairing drain places it once the current contention clears rather than fighting it live.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

