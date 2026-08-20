# cw-dep-bcgait1-hard1-steer7-middose1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-20T01:42:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-dep-bcgait1-hard1-steer5-fastprof1

**wandb_id**: wadldlj2

**hardware_ready**: False

**hypothesis**: Operator order (chat 2026-08-20 ~01:2x UTC — profile-headroom fork option B funded in parallel with steer6-fasttrack1): the intermediate-dose arm — write_speed=750/acc=40 (vel ceiling mirrors the write profile: ~66 deg/s, roughly the geometric midpoint between the ~350 counts/s sys-ID fit and the full 1500), slew 3.0 deg/tick = 75 deg/s sized to keep the servo profile, not the safety clamp, binding (mirroring the full-dose relationship) — on the IDENTICAL command-tracking MDP as steer6 (mode-3 leg-odometry velocity obs, k_walk_overspeed=2.0/tol 0.10, k_walk_heading=2.0, 0.05-0.10 m/s band, same yawm1 warm start, same 20M budget, same seed 12), so the two funded arms differ ONLY in the three dose keys. The canary proved the full dose breaks command tracking for every checkpoint zero-shot; the question here is whether a moderate raise buys real speed headroom WITHOUT the tracking collapse, and whether the tracking MDP closes the remaining gap faster at moderate dose. Prediction-if-true: pinned-speed panel monotone with prog_ratio in band at every row and slip within the 1.8/2.0 bars — if steer6 also passes, the operator picks by speed ceiling; if only this arm passes, the intermediate dose becomes the deployable contract candidate. Prediction-if-false: tracking breaks even at half dose (indicts the MDP, not the profile) or the ceiling is too low to matter (achieved speed ceiling not meaningfully above the 400/20 baseline) — STOP, report to operator, no autonomous dose interpolation.

**gate**: Pre-registered; all eval under the run OWN 750/40 cfg. PASS requires ALL: (1) training health — finite losses, no sustained KL-rollback storm; (2) periodic eval + final video keep in-band tall height and six-leg cycling (no re-crouch, no leg-sacrifice, no jitter storm); (3) COMMAND TRACKING: eval_checkpoint --pinned-speed-panel 0.04 0.06 0.08 0.10 det+sto — speed_med strictly monotone in the pinned command, prog_ratio_med 0.85-1.15 det (0.8-1.2 sto) at EVERY pinned speed, direction/heading error <= 25 deg med on the 0.06 and 0.08 rows; (4) RETENTION on the hard1-style fixed-command panel: gait_valid >= 5/6 det+sto, ZERO falls, slip <= 1.8/m det / 2.0 sto, roll settled; (5) TANGLE NO-REGRESSION: 24-ep direction-switch panel (6 families x 2 seeds x det+sto, DR-0 + own-DR-0.35, forced goal.walk_cmd_stage=2.0) 0/24 over_current, zero park/freeze/sacrificed-leg, probe_dirswitch_tangle yaw_sat_frac rot60-ON <= 0.011. (6) MATCHED-PARENT CONTROL before crediting or blaming training on any tracking claim (physics-axis injection rule): eval the steer3-yawm1 checkpoint zero-shot under the IDENTICAL 750/40 cfg, same eval seed; quote both slices. CHARGE HEALTH: reward_walk_overspeed and reward_walk_heading trend toward zero by run end. INFORMATIONAL: side-by-side read vs steer6-fasttrack1 (same seed, same MDP, dose is the only difference). ANY miss = STOP + report; no autonomous dose interpolation beyond the two operator-funded arms.

**verdict**: Fails its own pre-registered gate at the funding question: pinned-speed panel (0.04/0.06/0.08/0.10 m/s, DR-0) is NOT monotone-in-band -- prog_ratio 1.65/1.25/1.02/0.85 det (needs 0.85-1.15 at EVERY speed), 1.25/1.00/0.85/0.75 sto (needs 0.8-1.2) -- one weakly-scaling cadence (~0.066->0.085 m/s achieved across a 0.04-0.10 commanded range), not real speed control. Stress-mix retention (stage0, DR0+DR0.35): zero falls, gait_valid 6/6, tall six-leg gait on video (no exploit), but slip 1.6-5.1/m (bar <=1.8/2.0), direction_err 24-67deg, roll mostly class=leaning (never fully settles, though tail only 0.7-5.2deg, no danger). Matched-parent control (steer3-yawm1 zero-shot under the identical 750/40 profile) is WORSE on every axis (slip 2.0-18.1/m, dir_err 40-85deg) -- so the 20M of middose training genuinely improved tracking/slip over the untrained-under-profile parent, same story as the fastprof1 2M canary, but even at the full 20M acquisition budget and half the full dose it still does not cross the pre-registered band. Per its own pre-registration: ANY miss = STOP + report, no autonomous dose interpolation.

