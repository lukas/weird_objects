# cw-dep-bcgait1-hard1-steer6-fasttrack1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-20T01:44:12+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-bcgait1-hard1-steer5-fastprof1

**wandb_id**: 35z4dw4n

**hypothesis**: Operator order (chat 2026-08-20 ~01:2x UTC — profile-headroom fork option A funded): full raised servo profile (write_speed=1500/acc=80, vel ceiling mirrors the write profile, 5 deg/tick slew — the sim twin of the 08-19 bench stream upgrade whose canary proved 2.5x zero-shot speed) trained for the first time on an MDP that can OBSERVE and PRICE command tracking: goal.walk_obs_body_vel=3 (board-safe leg-odometry velocity on DR-corrupted signals; the mode-2 contract was blind to command error by construction), reward.k_walk_overspeed=2.0/tol=0.10 (linear band-exceedance charge — the Gaussian kernel saturates 2sigma out and k_walk_prog PAID the canary its 2.5x overspeed), reward.k_walk_heading=2.0 (1-cos charge; the canary drifted 50-60 deg), and a widened 0.05-0.10 m/s command band so speed is trained as a controllable variable across the pinned-panel range. Warm from steer3-yawm1 (anti-jam adapted, retention-clean), exact yawm1 stress-mix recipe otherwise, 20M steps, seed 12 paired with the steer7-middose1 sibling which differs ONLY in the three dose keys. Prediction-if-true: the pinned-speed panel turns monotone (speed_med tracks the command 0.04-0.10, prog_ratio_med in band at every row) while fixed-command slip returns to the 1.8/2.0 per-m bars and the tangle cure holds — a faster AND steerable download candidate. Prediction-if-false: speed stays profile-driven (flat 0.13-0.16 m/s cadence regardless of command) or the charges sit as an unresolved standing tax — STOP, report to operator, no autonomous dose or charge-k sweep beyond the two funded arms.

**gate**: Pre-registered; all eval under the run OWN raised-profile cfg. PASS requires ALL: (1) training health — finite losses, no sustained KL-rollback storm; (2) periodic eval + final video keep in-band tall height and six-leg cycling (no re-crouch, no leg-sacrifice, no jitter storm at the raised slew); (3) COMMAND TRACKING (the funding question): eval_checkpoint --pinned-speed-panel 0.04 0.06 0.08 0.10 det+sto — speed_med strictly monotone in the pinned command, prog_ratio_med 0.85-1.15 det (0.8-1.2 sto) at EVERY pinned speed, and direction/heading error <= 25 deg med on the 0.06 and 0.08 rows (canary generation: 48-53 deg, one cadence regardless of command); (4) RETENTION on the hard1-style fixed-command panel: gait_valid >= 5/6 det+sto, ZERO falls, slip <= 1.8/m det / 2.0 sto, roll settled; (5) TANGLE NO-REGRESSION: 24-ep direction-switch panel (6 families x 2 seeds x det+sto, DR-0 + own-DR-0.35, forced goal.walk_cmd_stage=2.0) 0/24 over_current, zero park/freeze/sacrificed-leg, probe_dirswitch_tangle yaw_sat_frac rot60-ON <= 0.011. CHARGE HEALTH: reward_walk_overspeed and reward_walk_heading trend toward zero by run end (resolved, not a standing tax — dragstance1 pattern is a FAIL). PASS -> propose (not launch) deployment prep to the operator (mode-3 estimator wiring in the hardware runner is deploy-side work that does not exist yet) plus any hardening continuation. ANY miss = STOP + report; no autonomous write_speed/acc/slew or charge-k sweep beyond this run and steer7-middose1.

