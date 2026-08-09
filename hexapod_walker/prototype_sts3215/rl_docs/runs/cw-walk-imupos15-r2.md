# cw-walk-imupos15-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T22:48:13+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-imupos15

**hypothesis**: RETRY #3 of cw-walk-imupos15 (r1 and r2 both died to the documented launch-collision EOFError at env-reset, 0 steps, no science result -- worker pipe broke during vec-env choreography amid a concurrent drain storm). Same spec unchanged, launched directly to a specific free pod (not via drain) to avoid a fourth collision. Isolated dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02 axis off champion longdist-r2.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**refused_reason**: hexapod-mjx-train-3 code marker 38a9ea67546d03423ac48149c0dee5576a7f0dac != local HEAD 3801dbaf117e611ee1969294c91b06cbd44be212. Sync first: snapshot.sh --sync hexapod-mjx-train-3 (and snapshot/commit before that if the tree is dirty).

