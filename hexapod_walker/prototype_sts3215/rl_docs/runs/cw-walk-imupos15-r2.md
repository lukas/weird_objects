# cw-walk-imupos15-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:49:04+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-imupos15

**hypothesis**: RETRY #3 of cw-walk-imupos15 (r1 and r2 both died to the documented launch-collision EOFError at env-reset, 0 steps, no science result -- worker pipe broke during vec-env choreography amid a concurrent drain storm). Same spec unchanged, launched directly to a specific free pod (not via drain) to avoid a fourth collision. Isolated dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02 axis off champion longdist-r2.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: INFRA (not a science result): EOFError at env-reset again, 0 steps -- 4th consecutive failure for this axis. This attempt was a single direct launch_run.py launch to a specific idle pod (not a drain race), yet still crashed identically at the same moment a totally different concurrent cycles launch (cw-walk-joyfric-s1) also failed the same way on the same node -- confirms this is a live host-wide launch-collision storm (many simultaneous JAX/Warp compiles + ~9 concurrent eval_checkpoint procs at 300-470% CPU each), not something specific to this pod or spec. Not fighting it further this cycle; requeued to backlog for the self-repairing drain once contention clears.

**failed_reason**: run never appeared as 'running' in W&B within 240s

