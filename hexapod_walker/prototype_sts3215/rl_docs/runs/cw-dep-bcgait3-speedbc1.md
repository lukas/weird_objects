# cw-dep-bcgait3-speedbc1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-21T22:56:48+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-dep-bcgait2-fastbc1

**wandb_id**: 4yitv3cc

**hardware_ready**: False

**hypothesis**: Teach the fast walker to obey the joystick's commanded speed instead of blasting past it. Operator order 20260821T224150Z (fast-gait lever): fresh BC clone of the native-cadence tripod teacher under the FULL fast servo profile, speed-conditioned across a 0.06-0.10 m/s band (speed varied by stride geometry at fixed clock — the physically valid knob; period_scale refuted 08-20), with the policy able to SEE its real speed via the deployable leg-odometry velocity obs (goal.walk_obs_body_vel=3) next to the separate command refs — the failed fastbc1/track1 arms trained blind under vel:=ref and could not observe their own 2x overspeed. The tested band-exceedance + heading charges (k_walk_overspeed=2.0/tol=0.10, k_walk_heading=2.0) make in-band travel the best-paying behavior (fastprof semantics bank green: obey out-earns overspeed by >50 and the margin comes from the charge, not the legacy kernel). Teacher preflight THIS cycle (logs/probe_speedband/full_*.json, pods train-0..3): realized speed strictly monotone in command, slip/m 1.5-2.8, 147mm tall, zero falls, clean 6-leg at 0.06/0.08/0.10; the 0.04/0.05 bands are REFUTED at teacher level (slip/m 2.9-3.8 vs the <=2-3 budget) and excluded. Clone preflight (ckpt closed-loop, mode-3 obs): zero falls, 146-147mm, all six legs cycling, slip/m 1.8-2.8, realized speed monotone; holdout act err 0.0119. Prediction-if-true: pinned-speed panel goes in-band at every band. Prediction-if-false: overspeed persists despite observable speed + charge = the pricing/obs lever is insufficient, STOP and report; strongest alternative: charges sit as an unresolved standing tax and speed collapses instead.

**gate**: At 2M: eval_checkpoint --pinned-speed-panel 0.06 0.08 0.10, det+sto, DR-0: ZERO falls; gait_valid 6/6 (tall clean all-six-leg, height in-band, no crouch reversion); direction error <= 30 deg med at every band; slip/m around baseline budget (det <= 2.2, sto <= 3.0); prog_ratio med in 0.75-1.25 at EVERY band on BOTH passes — especially no 2x overspeed. Report roll_tail/drag visual-quality stats vs fastbc1 (roll_tail 1.1-2.3 deg). PASS -> pre-registered next rung (operator order): several FIXED HEADINGS at pinned speeds; only after that, irregular direction changes. FAIL modes: overspeed persists = obs+charge lever insufficient, STOP + operator; speed collapses below 0.75 band-wide = charges bought obedience by stalling, STOP; falls/collapse = full-profile fresh-clone premise refuted, STOP. NO DOWNLOAD_ANSWER change unless a hardened successor beats bcgait1_hard1 on the session gate.

**verdict**: FAIL on every gate axis at DR-0 (pinned-speed panel 0.06/0.08/0.10, det+sto): 34/48 episodes ended in tilt_pitch falls (gate: ZERO), direction error med 58-80 deg (gate <=30), slip/m det ~3.0-3.5 / sto 8-11 (gate 2.2/3.0), and raw speed 0.12-0.14 m/s COMMAND-INVARIANT across the band — overspeed persists despite the leg-odometry vel obs + overspeed/heading charges. Strictly worse than parent cw-dep-bcgait2-fastbc1 (zero falls, correct direction, clean overspeed): RL under the charges destabilized the clone instead of buying obedience. Pre-registered FAIL mode hit: obs+charge lever insufficient AND falls -> STOP, fork returns to operator. Eval on repo-nominal sysid plant at snapshot cfe8160a (08-21 calibration commits add bench tooling only; no calibrated plant values committed in repo).

