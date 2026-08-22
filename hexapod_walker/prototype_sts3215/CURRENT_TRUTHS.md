# CURRENT TRUTHS - accepted facts and rulings

Last compacted: 2026-08-20 UTC. This file contains accepted current facts,
not campaign narrative. If history, logs, or old status prose disagree with
this file, this file wins.

## Mission

The goal is reliable real-robot joystick control: stand up, sit down, turn,
and walk where pointed. Reliability and safety outrank speed. Slip is a
calibration signal, not automatic failure.

## Current Top Ruling

SIM SPRINT is binding while the robot is off the bench for repair. The fleet
focuses on a download-ready MuJoCo rise+walk answer. Hardware attempts and
bench-only items stay parked until the operator says the robot is back.

## Current Download Answer

The measured product baseline is the hierarchy in `rl_docs/DOWNLOAD_ANSWER.md`:

- stance: `ppo_goal_cw_stand_footlow2_hard1`
- walk: `ppo_goal_cw_dep_bcgait1_hard1`
- session controller: re-anchor per mode, entry slew, STOP->stance hold, rot60 wrapper

Held-out session gate: det 0.967, sto 0.853 at n=600. This remains the
answer until a replacement beats the relevant gate and promotion contract.

## Known Gaps

- Post-lower rise is the weak session boundary. `postlower4` looks better only under remaining-rise semantics; changing the contract and promoting it are operator decisions.
- Takeoff roll transient is a hardware/sim-boundary issue; entry slew is the best measured sim mitigation so far.
- Learned stand-up on hardware is unproven. Scripted stand/sit glides remain fallback hardware tools.
- Fast gait is not deployable yet, but a live path exists. The raised motor profile destabilizes WARM-STARTED/transplanted policies (all 4 A/B canaries failed identically); a faster TripodGait cadence is refuted at the teacher level (08-20 preflight grid: every rung strictly worse at every profile dose). A fresh BC-INIT clone of the native-cadence teacher DOES survive RL fine-tune under the full profile: canary `cw-dep-bcgait2-fastbc1` PASSED discovery (tall, clean 6-leg gait, zero falls, direction correct, ~2x deployed speed) but overspeeds the commanded band 2x (nothing priced the excess under vel:=ref). Hardening rung `cw-dep-bcgait2-fastbc1-track1` (added `reward.k_walk_cmd_track=1.0`) FAILED its pre-registered gate — the tracking price made the overspeed WORSE (det 1.88x->2.10x, sto 1.20x->1.76x; own-DR sto 1.12x->1.92x). That lever is refuted. Operator order 20260821T224150Z (08-21) closed q_20260820T2330Z with the speed-conditioned-BC lever: fresh clone + mode-3 leg-odometry velocity obs (the failed arms trained blind under vel:=ref) + tested overspeed/heading charges; teacher preflight bounds the low-slip speed envelope to 0.06-0.10 m/s under the full profile (0.04/0.05 refuted, slip/m 2.9-3.8). That lever is now ALSO REFUTED: canary `cw-dep-bcgait3-speedbc1` FAILED its pinned-speed gate on every axis (34/48 tilt_pitch falls vs parent's zero, dir err 58-80 deg, slip det 3.0-3.5 / sto 8-11, raw speed 0.12-0.14 m/s command-invariant) — being able to SEE the speed plus priced charges did not produce obedience; RL under the charges destabilized the clone. The operator ordered a +4M continuation (`cw-dep-bcgait3-speedbc1-cont1`, overriding the pre-registered STOP) to test whether more training escapes the charge basin; it FAILED WORSE — the rollout-reward "recovery" was purely episodes getting shorter (per-tick reward stayed net-negative), and the own pinned-speed panel on the final checkpoint showed 48/48 falls (worse than the parent's 34/48) plus a new sacrificed-leg gait pathology, with no gain in direction/speed obedience. The operator's 08-22 gait-phase/direction-first lever (fb_20260822T000627: +2 sin/cos phase obs at the teacher clock, fixed 0.08 speed, heading obedience gated instead of speed) was executed as `cw-dep-bcgait4-phasedir1` and ALSO FAILED as an RL lever — 2M PPO degraded every axis vs the matched un-RL'd clone control (dir_err med 35.6->67.3 deg, speed 0.068->0.139 overspeed attractor, slip/m 1.81->4.17) though the phase input kept ZERO falls (first fast-RL arm that did not collapse). CRUCIAL COUNTERPOINT: the phase-conditioned BC clone itself (`ppo_goal_cw_bcgait_init_fullprof_phase1`, committed; holdout act err 0.0040) passes the ENTIRE direction-first curriculum with zero RL at the measured plant — all fixed headings incl. rear AND irregular heading changes at 0.08 (SKILLS row) — so for the fast gait, imitation currently strictly beats imitation+RL. Fast-gait RL levers refuted: faster cadence, k_walk_cmd_track, speed-obs+charges, more training steps, and phase-obs+fixed-speed AS RUN — i.e. with random fixed headings over the full +/-180 deg range from the first rung (`goal.walk_heading_max_rad=3.1416`). SCOPE (operator-keyed fb_20260822T003132): the phase INPUT is not refuted (it kept zero falls), and the operator's intended STAGED curriculum — forward-only first, then a small heading set, then full fixed headings, then irregular changes, with stronger gait preservation — is UNTRIED as an RL lever. The fork is operator-gated (adopt the zero-RL clone / staged-curriculum phase RL per fb_20260822T003132 / new RL pricing / park). Metric fact: direction_err_mean_deg has a ~35 deg tick-level floor from stride sway — judge clone-vs-child deltas, not the raw value. No-BC scratch walking stays closed. No download change unless a successor beats `bcgait1_hard1`.

## Real Robot Facts

- Scripted tripod gait walks, crabs, and turns, with visible loaded-foot slip and roughly half commanded travel.
- Sim plant uses the operator's MEASURED tibia (150 mm, commit a4beb8af 08-21; was 128) in physics, gait IK, and geometry checks — walk stance now ~169 mm tall. Every pre-08-22 checkpoint (incl. the download hierarchy) trained on the old 128 mm plant: cross-plant comparisons need matched controls, and re-gating the download answer on the measured plant is an open operator decision. The scripted tripod teacher is verified clean at the new plant (08-22 grid: 0.06-0.10 m/s x 4 headings, zero falls, slip/m 1.4-2.9, full fast profile).
- Working gaits rock about 10-20 degrees; large roll/tilt events and falls matter more than scalar reward.
- Learned stand-up previously tripped tilt_roll reliably on hardware; do not treat sim success alone as bench promotion.
- Servo/control facts in force: 25 Hz loop, 1.5 deg/tick stateful slew, loaded servo settle about 250-325 ms, air settle about 9 ms.
- No physical motion without an explicit operator ask.
- Robot-control/web edits should use the fast dev loop in
  `linux_control/dev_loop.sh`: `make robot-check`, `make robot-unit-check`,
  `make robot-status`, and `make robot-deploy`. These helpers do not move
  the robot; `robot-deploy` only restarts the web service. Use
  `make robot-resolve` for temporary IP overrides when `hexapod.local` is
  flaky; never commit a fixed board IP.

## Policy And Eval Facts

- Policies output 18 raw joint targets through the SafetyLayer.
- `cw-dep-vref1-r1` established the vx/vy-measured-as-reference contract used by the hardware walking base.
- `eval_session` is the main gate for stance/walk session candidates.
- Video/physical behavior outranks reward alone. A reward-passing cheat is a metric bug, not a skill.
- Matched-parent controls are mandatory for injected physics/sensor axes.

## Research Rulings

- Operator-authenticated orders obey first and ask after, unless blocked by safety, typos, failing tests/preflight, or mechanical impossibility.
- One-variable-per-run is repealed. Coupled bundles are allowed when the operator orders them or the mechanism requires them; pre-registration and honest verdicts still bind.
- Agent-doable work drains before backoff. If code/triage/precondition work is available, execute it instead of rereading docs.
- No broad doc sweeps during execution cycles. Read the startup packet, inspect only files needed for the current decision, then launch, triage, code, or exit with the concrete gate.
- Peripheral launches to fill GPUs are violations. Idle pods are acceptable when the blocker is operator, hardware, or specification-gated.

## Track Facts

- `hw` is the mainline.
- `multitask` pause was lifted, but it remains secondary during SIM SPRINT unless directly serving download readiness or explicitly ordered.
- `nobc` gait-from-scratch stays closed. The operator's 08-21 anti-slip/no-speed-target reopening (order 20260821T133626Z) ran exactly one canary, `cw-nobc-slipwalk1-r1`, and it froze (0.001 m travel, 0.34 m scuffing, 4 legs unused). Its reward stack was preflight-proven correct (SLIPWALK bank: travel out-earns stall/park/skate by 300-2000), so from-scratch gait's blocker is EXPLORATION from a blank init, not reward specification. Sub-line stopped by operator instruction; reopening needs a start that already moves.
- `quad`, `turn`, `arch`, and `dynrep` are secondary during SIM SPRINT unless directly serving download readiness or explicitly ordered.
- Recover/tangle was reopened by operator order 08-20. The recover champion (`ppo_goal_cw_recover_predictive1b_pop3_s13` + frozen encoder `cw-dynrep-tf-state2-recovered1.pt`) is packaged and sim-gate-verified through the deployment runner (`rl_move/sim/recover_runner.py`; DR-0 21/23, own-DR 22/23 on the 23-rung ladder — equal to the training-path gate). Recovery is an ADDITIONAL operator-requested mode; it does not change the rise+walk download answer. Flip is out of envelope; hardware items are bench-parked (`rl_docs/RECOVER_DEPLOY.md`).
