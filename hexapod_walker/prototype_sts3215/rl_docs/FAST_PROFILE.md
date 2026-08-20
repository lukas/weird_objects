# FAST PROFILE — command-tracking MDP prep for the fast-gait fork

Status 2026-08-20: PREP COMPLETE (fb_20260820T000059 item 3, all
default-off, nothing trained). The fork itself stays `[operator]`
(STATUS.md WAITING-ON "profile-headroom fork"): fund a long adaptation
run at 1500/80, pick an intermediate dose, or park. Items 1 (trapezoid
restart-on-rewrite semantics — needs a bench step-response trace
first) and 2 (bench characterization of 1500/80 under load) are
bench-gated and parked until the robot is back. Item 5 (staged dose
vs long adaptation) is explicitly not-autonomous.

Context: the operator-ordered canary `cw-dep-bcgait1-hard1-steer5-
fastprof1` (verdict 08-19 ~22:5x, W&B `yvkjp5xj`) proved the servo
profile (write_speed=1500 / write_acc=80 / `bus.servo_vel_max_counts_s`)
was the binding constraint on raw speed — every checkpoint walks
~2.5x faster zero-shot — but command tracking breaks for all of them
(prog_ratio 1.27–1.76 vs band 0.91–1.07, heading error 50–60°,
slip 2.4–8.1/m). This file records the three pre-launch code pieces
and the observation audit that any funded fast-gait arm builds on.

## (a) Pinned-speed eval panels — built

`eval_checkpoint.py --pinned-speed-panel [speeds...]` (no values =
0.04/0.06/0.08/0.10 m/s). Each row pins the walk command to ONE speed
(pure forward, no resample, no stops, wz=0, curricula disabled —
`pinned_speed_cfg()` is the single truth, reusable by other
harnesses), reporting as `walk@<speed>/<det|sto>` in report.json,
the console table and the W&B summary (`eval/<dr>/walk@0.060_det/...`).
Read `speed_med` and `prog_ratio_med` per row: a policy that has
learned speed as a CONTROLLABLE variable shows monotone achieved
speed tracking the pinned command; the canary generation shows one
fast cadence regardless of command. Default absent = report unchanged.

## (b) Overspeed + heading-error pricing — built

`reward.k_walk_overspeed`/`walk_overspeed_tol` and
`reward.k_walk_heading`/`walk_heading_min_speed_m_s` — see REWARD.md
(income table) for semantics and operating points. Why the legacy
stack could not price the canary's failure: the velocity kernel is a
Gaussian (saturates ~2σ out, so at 0.14 m/s vs a 0.06 command the
gradient is ~0) and `k_walk_prog` caps at 1.25 — overspeed still PAYS
1.25x full progress income. Charges, not gates: never shrunk by the
gait/income gates, zero in-band/aligned by construction. Banks green:
FASTPROF section of `test_task_semantics.py` (obeying the command
out-earns the same scripted gait driven at the canary's 2.5x ratio or
55° skew, and the margin comes from the NEW keys, not the legacy
kernel), units in `test_walk_fastprof_mdp.py`.

## (c) Observation sufficiency AUDIT — verdict: mode 2 is BLIND

Question (operator note item 3): do the obs carry enough
body-velocity/command-error signal under the raised profile?

**No — not on the deployed contract.** The entire cw-dep lineage
(hard1, steer*, fastprof1) trains with `goal.walk_obs_body_vel=2`,
which feeds meas := ref: the two "measured velocity" obs dims are a
COPY OF THE COMMAND. Command error (v_meas − v_ref) is therefore
unobservable by construction — the policy can only infer its true
speed from proprioceptive history (hist16 qdot/gyro/prev-action).
Measured consequences on record (estimator.py header, 08-11): mode-2
policies crouch 50–77 mm and creep where privileged (mode-1) policies
stand tall and track; and the fastprof canary could not even observe
its own 2.5x overspeed — consistent with 2M of adaptation improving
every metric yet nobody reaching the band. Training "speed as a
controllable variable" with a speed the policy cannot sense is asking
for open-loop cadence tables, not control.

**Latent trap found + fixed during the audit:** `walk_obs_body_vel=3`
was referenced by probes as "the estimator mode" but was NEVER
implemented — any cfg value other than 0/2 silently fell into the
PRIVILEGED simulator-velocity branch. A run launched with 3 would
have trained on ground truth while everyone believed it deployable.
Mode 3 is now wired for real: `rl_move.estimator.LegOdometryVelocity`
(board-safe numpy leg odometry, probe-validated by
`probe_estimator.py`) runs on the DR-CORRUPTED observed state
(encoders + gyro + tilt — the robot's own view), per-episode instance
on `MJX_SNAPSHOT_EXTRA` (pool-restore safe), same obs width as every
other mode (warm-start compatible). Modes 0/1/2 bit-exact unchanged.

**Recommendation for the funded arm (operator's pick):** train under
the raised profile with `goal.walk_obs_body_vel=3` + the (b) charges,
gate on the (a) pinned-speed panel (monotone command-tracking) plus
the usual hard1-retention/tangle bars. Note mode 3 IS a deployment
contract change: the hardware runner must compute the same estimator
at 25 Hz (it is plain numpy on signals the board already produces —
built for exactly that — but the runner wiring is deploy-side work
that does not exist yet).
