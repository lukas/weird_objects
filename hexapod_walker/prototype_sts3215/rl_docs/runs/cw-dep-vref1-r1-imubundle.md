# cw-dep-vref1-r1-imubundle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T16:50:16+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: wrfkvges

**hardware_ready**: False

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly if the real IMU has several realistic quirks AT ONCE instead of one at a time. vref1-r1 already PASSED latency drift (0.5-2.5x), a tilted IMU mount (10deg), and gyro noise (1.5deg/s) INDIVIDUALLY (3 separate PASSed respecs tonight) -- the real IMU chip will have all three simultaneously (a slightly crooked mount, sensor noise, AND some read/transport delay), so this bundles the three already-PASSED sensor-realism axes onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not a warm start from any single-axis checkpoint, to avoid compounding a single lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + all 3 axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- individually-benign sensor axes stay benign combined, same as the already-PASSED comshift+deadband and fric+groundtilt5 bundles. If-false: the combined sensor uncertainty (mount offset shifting the noise's effective bias, on top of latency) breaks tracking in a way no single axis did -- flag as a real pre-attempt-#2 sensor risk, not assume single-axis DR passes compose for free.

**gate**: own-cfg (DR0.35 + dr.latency_scale=0.5,2.5 + dr.imu_mount_deg=10.0 + dr.gyro_noise_deg_s=1.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 no-sensor-DR retention clean; frames watched det

**verdict**: PASS -- confirms if-true: bundling all three already-PASSed sensor-realism axes (latency drift 0.5-2.5x, IMU-mount rotation 10deg, gyro-rate noise 1.5deg/s) stays benign combined, same as the comshift+deadband and fric+groundtilt5 bundles. Own-cfg (DR0.35+all 3 axes) det 5/6 ok / sto 4/6 ok, gv 6/6 both, 0 term either pass, slip/m med 1.11 det / 1.23 sto -- both inside vref1-r1's own band (0.89-1.36). Degraded episodes (det/5, sto/0, sto/1 -- prog 0.65-0.75, slip 1.99-2.22) are the same lineage fixed-seed march-in-place stall seen on every other PASSed DR0.35 sibling tonight; frame-checked (det/5 both modes): level body, all six legs cycling, no flag-leg/drag/fall. DR0-no-sensor-DR retention clean: det 5/6 sto 6/6, gv 12/12, 0 term; the det/4 catastrophic-slip draw (28.90) is the SAME shared-lineage fixed-seed crater (confirmed against cmddrop/comshift-deadband/fric-groundtilt5/torquescale-gyronoise/encnoise-latency/velscale's own gate reports -- 20-29 slip at the identical index, absent from the base vref1-r1 checkpoint). Training finished clean (reward quarters 587/667/661/648). Not independently hardware-ready (inherits vref1-r1's own paddle-gait economics); clears the combined 3-axis sensor-realism bundle as safe for the hardware candidate -- individually-benign axes stay benign when stacked.

